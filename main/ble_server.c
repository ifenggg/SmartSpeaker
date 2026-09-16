/**
 * @file ble_server.c
 * @brief 【已停用 / 不在编译中】BLE GATT Server（从机）实现 —— 见 ble_server.h 顶部说明
 *
 * 本工程的调试链路是"音箱始终作 BLE 主机（客户端）"，对端（手表 / 手机从机）提供 GATT 服务；
 * 因此这里的"音箱作从机"实现已移出编译，保留备用。
 *
 * 原事件流程（供参考）：
 *   GATTS 注册(REG) → 创建服务(CREATE) → 启动服务
 *   → 添加通知特征 FFF1(ADD_CHAR) → 添加 FFF1 的 CCCD(ADD_CHAR_DESCR)
 *   → 添加写特征 FFF2(ADD_CHAR) → 属性表就绪
 *   → 配置原始广播数据 → 开始广播
 *   → 手机连接(CONNECT，停止广播) → 手机写 CCCD 订阅通知 → 双向收发
 *   → 手机断开(DISCONNECT，重新广播)
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "ble_gap.h"
#include "ble_server.h"

static const char *TAG = "ble_server";

#define GATTS_APP_ID            0       /* 本工程仅注册一个 GATTS 应用 */

/* 服务句柄预算：服务声明 + 特征1声明 + 特征1值 + 特征1 CCCD + 特征2声明 + 特征2值 */
#define GATTS_NUM_HANDLE        6

/* 属性表创建步骤（GATTS 回调是异步的，用步骤变量串起创建顺序） */
typedef enum {
    DB_STEP_IDLE = 0,
    DB_STEP_WAIT_FFF1,
    DB_STEP_WAIT_CCCD,
    DB_STEP_WAIT_FFF2,
    DB_STEP_READY,
} db_step_t;

/* ---------------------------------------------------------------------
 * 内部状态（GATTS 回调运行于 BTC 任务，ble_server_send 由调试任务调用）
 * --------------------------------------------------------------------- */
static SemaphoreHandle_t s_mtx = NULL;

static esp_gatt_if_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0;              /* 0 = 无连接 */
static uint8_t  s_notify_mode = 0;          /* FFF1 订阅状态：0=未订阅 1=Notify 2=Indicate */
static uint16_t s_mtu = 23;

static uint16_t s_service_handle = 0;
static uint16_t s_notify_char_handle = 0;   /* 0xFFF1：音箱 → 手机 */
static uint16_t s_notify_cccd_handle = 0;
static uint16_t s_write_char_handle = 0;    /* 0xFFF2：手机 → 音箱 */
static db_step_t s_db_step = DB_STEP_IDLE;

static int      s_adv_cfg_pending = 0;      /* 还差几个"原始广播数据设置完成"事件 */
static bool     s_adv_data_ready = false;
static bool     s_adv_running = false;

static void (*s_rx_cb)(uint8_t *data, uint16_t len) = NULL;

/* 特征值缓冲 + 默认说明串（手机读特征时能看到怎么用） */
static const char *s_notify_doc = "ESP32 speaker: subscribe NOTIFY on 0xFFF1 to receive frames";
static const char *s_write_doc  = "ESP32 speaker: WRITE frames to 0xFFF2";

static char s_notify_value[BLE_SERVER_ATTR_MAX_LEN];
static char s_write_value[BLE_SERVER_ATTR_MAX_LEN];

static esp_attr_value_t s_notify_attr = {
    .attr_max_len = sizeof(s_notify_value),
    .attr_len     = 0,
    .attr_value   = (uint8_t *)s_notify_value,
};

static esp_attr_value_t s_write_attr = {
    .attr_max_len = sizeof(s_write_value),
    .attr_len     = 0,
    .attr_value   = (uint8_t *)s_write_value,
};

/* FFF2 最近一次收到的内容（调试助手读该特征时返回） */
static uint8_t s_last_rx[BLE_SERVER_ATTR_MAX_LEN];
static uint16_t s_last_rx_len = 0;

static esp_bt_uuid_t s_notify_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = { .uuid16 = BLE_SERVER_NOTIFY_CHAR_UUID },
};

static esp_bt_uuid_t s_write_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = { .uuid16 = BLE_SERVER_WRITE_CHAR_UUID },
};

static esp_bt_uuid_t s_cccd_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
};

static esp_gatt_srvc_id_t s_srvc_id = {
    .is_primary = true,
    .id = {
        .inst_id = 0,
        .uuid = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = BLE_SERVER_SERVICE_UUID } },
    },
};

/* 广播：60ms 间隔、可连接、公共地址（不使用 RPA，避免地址类型问题） */
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min        = 0x60,
    .adv_int_max        = 0x60,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ---------------------------------------------------------------------
 * 小工具
 * --------------------------------------------------------------------- */
static inline void ble_server_lock(void)
{
    if (s_mtx) {
        xSemaphoreTake(s_mtx, portMAX_DELAY);
    }
}

static inline void ble_server_unlock(void)
{
    if (s_mtx) {
        xSemaphoreGive(s_mtx);
    }
}

/* 组装原始广播数据：Flags + 16位服务UUID + 完整设备名；扫描响应里再放一次设备名 */
static void ble_server_config_adv_data(void)
{
    const char *name = BLE_SERVER_ADV_NAME;
    size_t nlen = strlen(name);
    if (nlen > 26) {
        nlen = 26;      /* 保证广播数据总长不超过 31 字节 */
    }

    uint8_t adv[31];
    uint8_t rsp[31];
    size_t alen = 0;
    size_t rlen = 0;

    /* ① Flags：通用可发现（本机同时支持经典蓝牙，故不声明 BR/EDR Not Supported） */
    adv[alen++] = 2;
    adv[alen++] = ESP_BLE_AD_TYPE_FLAG;
    adv[alen++] = ESP_BLE_ADV_FLAG_GEN_DISC;

    /* ② 16 位完整服务 UUID 列表：方便调试助手在广播里直接看到服务 */
    adv[alen++] = 3;
    adv[alen++] = ESP_BLE_AD_TYPE_16SRV_CMPL;
    adv[alen++] = (uint8_t)(BLE_SERVER_SERVICE_UUID & 0xFF);
    adv[alen++] = (uint8_t)((BLE_SERVER_SERVICE_UUID >> 8) & 0xFF);

    /* ③ 完整设备名（"Ace 2V"，含空格，合法） */
    adv[alen++] = (uint8_t)(nlen + 1);
    adv[alen++] = ESP_BLE_AD_TYPE_NAME_CMPL;
    memcpy(&adv[alen], name, nlen);
    alen += nlen;

    /* ④ 扫描响应里同样放设备名，兼容只看扫描响应的调试助手 */
    rsp[rlen++] = (uint8_t)(nlen + 1);
    rsp[rlen++] = ESP_BLE_AD_TYPE_NAME_CMPL;
    memcpy(&rsp[rlen], name, nlen);
    rlen += nlen;

    ble_server_lock();
    s_adv_cfg_pending = 2;
    s_adv_data_ready = false;
    ble_server_unlock();

    esp_err_t err = esp_ble_gap_config_adv_data_raw(adv, (uint32_t)alen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置广播数据失败: 0x%x", err);
    }
    err = esp_ble_gap_config_scan_rsp_data_raw(rsp, (uint32_t)rlen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置扫描响应数据失败: 0x%x", err);
    }
}

static void ble_server_adv_start_now(void)
{
    esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动广播失败: 0x%x", err);
    }
}

void ble_server_start_adv(void)
{
#if BLE_SERVER_AUTO_ADV
    bool ready;
    bool running;

    ble_server_lock();
    ready = s_adv_data_ready;
    running = s_adv_running;
    ble_server_unlock();

    if (running || s_db_step != DB_STEP_READY) {
        return;                                     /* 已在广播 / 属性表还没建好 */
    }
    if (ready) {
        ble_server_adv_start_now();                 /* 广播数据已配置过，直接开播 */
    } else {
        ble_server_config_adv_data();               /* 配置完成后在 GAP 回调里开播 */
    }
#else
    ESP_LOGW(TAG, "BLE_SERVER_AUTO_ADV=0，本模块不广播（仅保留 GATT 服务）");
#endif
}

void ble_server_stop_adv(void)
{
#if BLE_SERVER_AUTO_ADV
    ble_server_lock();
    bool running = s_adv_running;
    ble_server_unlock();
    if (running) {
        esp_ble_gap_stop_advertising();
    }
#endif
}

/* ---------------------------------------------------------------------
 * GAP 回调（由 ble_gap 统一分发）
 * --------------------------------------------------------------------- */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT: {
        bool go = false;
        ble_server_lock();
        if (s_adv_cfg_pending > 0 && --s_adv_cfg_pending == 0) {
            s_adv_data_ready = true;
            go = true;
        }
        ble_server_unlock();
        if (go) {
            ble_server_adv_start_now();
        }
        break;
    }

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "广播启动失败: 0x%x", param->adv_start_cmpl.status);
            ble_server_lock();
            s_adv_running = false;
            ble_server_unlock();
        } else {
            ble_server_lock();
            s_adv_running = true;
            ble_server_unlock();
            ESP_LOGI(TAG, "已开始广播：设备名 \"%s\"（服务 0x%04X / 通知 0x%04X / 写 0x%04X），等待手机调试助手接入",
                     BLE_SERVER_ADV_NAME, BLE_SERVER_SERVICE_UUID,
                     BLE_SERVER_NOTIFY_CHAR_UUID, BLE_SERVER_WRITE_CHAR_UUID);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ble_server_lock();
        s_adv_running = false;
        ble_server_unlock();
        ESP_LOGI(TAG, "广播已停止");
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------
 * GATTS 回调
 * --------------------------------------------------------------------- */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTS 应用注册失败: 0x%x", param->reg.status);
            break;
        }
        s_gatts_if = gatts_if;
        ESP_LOGI(TAG, "GATTS 应用注册成功, gatts_if=%d", s_gatts_if);
        if (esp_ble_gatts_create_service(gatts_if, &s_srvc_id, GATTS_NUM_HANDLE) != ESP_OK) {
            ESP_LOGE(TAG, "创建服务失败");
        }
        break;

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "创建服务失败: 0x%x", param->create.status);
            break;
        }
        s_service_handle = param->create.service_handle;
        s_db_step = DB_STEP_WAIT_FFF1;
        ESP_LOGI(TAG, "服务 0x%04X 已创建, handle=%d，添加通知特征 0x%04X",
                 BLE_SERVER_SERVICE_UUID, s_service_handle, BLE_SERVER_NOTIFY_CHAR_UUID);
        esp_ble_gatts_start_service(s_service_handle);
        /* 特征 0xFFF1：音箱→手机，Read + Notify */
        if (esp_ble_gatts_add_char(s_service_handle, &s_notify_char_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   &s_notify_attr, NULL) != ESP_OK) {
            ESP_LOGE(TAG, "添加特征 0x%04X 失败", BLE_SERVER_NOTIFY_CHAR_UUID);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "添加特征失败: 0x%x (handle=%d)", param->add_char.status,
                     param->add_char.attr_handle);
            break;
        }
        if (s_db_step == DB_STEP_WAIT_FFF1) {
            s_notify_char_handle = param->add_char.attr_handle;
            s_db_step = DB_STEP_WAIT_CCCD;
            ESP_LOGI(TAG, "通知特征 0x%04X 已添加, handle=%d，添加 CCCD(0x2902)",
                     BLE_SERVER_NOTIFY_CHAR_UUID, s_notify_char_handle);
            if (esp_ble_gatts_add_char_descr(s_service_handle, &s_cccd_uuid,
                                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                             NULL, NULL) != ESP_OK) {
                ESP_LOGE(TAG, "添加 CCCD 失败");
            }
        } else if (s_db_step == DB_STEP_WAIT_FFF2) {
            s_write_char_handle = param->add_char.attr_handle;
            s_db_step = DB_STEP_READY;
            ESP_LOGI(TAG, "写特征 0x%04X 已添加, handle=%d，属性表就绪，开始广播",
                     BLE_SERVER_WRITE_CHAR_UUID, s_write_char_handle);
            ble_server_start_adv();
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "添加 CCCD 失败: 0x%x", param->add_char_descr.status);
            break;
        }
        s_notify_cccd_handle = param->add_char_descr.attr_handle;
        s_db_step = DB_STEP_WAIT_FFF2;
        ESP_LOGI(TAG, "CCCD handle=%d，添加写特征 0x%04X", s_notify_cccd_handle,
                 BLE_SERVER_WRITE_CHAR_UUID);
        /* 特征 0xFFF2：手机→音箱，Read + Write + WriteNR */
        if (esp_ble_gatts_add_char(s_service_handle, &s_write_char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE |
                                   ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                   &s_write_attr, NULL) != ESP_OK) {
            ESP_LOGE(TAG, "添加特征 0x%04X 失败", BLE_SERVER_WRITE_CHAR_UUID);
        }
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "服务已启动, handle=%d", param->start.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ble_server_lock();
        s_conn_id = param->connect.conn_id;
        s_notify_mode = 0;
        ble_server_unlock();
        ESP_LOGI(TAG, "【手机】连接成功！已接入: %02x:%02x:%02x:%02x:%02x:%02x, conn_id=%d"
                      "（订阅 0x%04X 的通知后即可收到调试帧）",
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
                 s_conn_id, BLE_SERVER_NOTIFY_CHAR_UUID);
        /* 只保留一路手机连接：接入后停止广播，断开后自动恢复 */
        ble_server_stop_adv();
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ble_server_lock();
        s_conn_id = 0;
        s_notify_mode = 0;
        ble_server_unlock();
        ESP_LOGW(TAG, "【手机】连接断开, conn_id=%d, reason=0x%02x → 重新开始广播",
                 param->disconnect.conn_id, param->disconnect.reason);
        ble_server_start_adv();
        break;

    case ESP_GATTS_MTU_EVT:
        ble_server_lock();
        s_mtu = param->mtu.mtu;
        ble_server_unlock();
        ESP_LOGI(TAG, "手机协商 MTU = %d（单帧最大 %d 字节）", param->mtu.mtu,
                 param->mtu.mtu > 3 ? param->mtu.mtu - 3 : 20);
        break;

    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;

        /* ① 0xFFF1 的 CCCD：手机订阅 / 取消订阅通知 */
        if (s_notify_cccd_handle != 0 && handle == s_notify_cccd_handle && param->write.len == 2) {
            uint16_t cfg = (uint16_t)(param->write.value[0] | (param->write.value[1] << 8));
            ble_server_lock();
            s_notify_mode = (cfg & 0x0001) ? 1 : ((cfg & 0x0002) ? 2 : 0);
            uint8_t mode = s_notify_mode;
            ble_server_unlock();
            if (mode == 0) {
                ESP_LOGW(TAG, "【手机】已取消订阅 0x%04X 的通知 → 暂停向手机推送调试帧",
                         BLE_SERVER_NOTIFY_CHAR_UUID);
            } else {
                ESP_LOGI(TAG, "【手机】已订阅 0x%04X 的通知%s → 手机链路可双向收发调试帧",
                         BLE_SERVER_NOTIFY_CHAR_UUID, (mode == 2) ? "(Indicate)" : "");
            }
        }
        /* ② 0xFFF2 写入：手机 → 音箱，转给上层回调打印 */
        else if (s_write_char_handle != 0 && handle == s_write_char_handle) {
            if (param->write.is_prep) {
                ESP_LOGW(TAG, "收到长写(Prepare Write)，本调试通道未做分包重组，本包已忽略");
            } else if (param->write.len > 0) {
                uint16_t len = param->write.len;
                if (len > sizeof(s_last_rx)) {
                    len = sizeof(s_last_rx);
                }
                memcpy(s_last_rx, param->write.value, len);
                s_last_rx_len = len;
                if (s_rx_cb) {
                    s_rx_cb(param->write.value, param->write.len);
                }
            }
        }

        /* 需要应答的写（Write Request）必须回响应，否则手机会超时 */
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                        ESP_GATT_OK, NULL);
        }
        break;
    }

    case ESP_GATTS_READ_EVT: {
        /* 读 0xFFF2：返回最近收到的一帧；读 0xFFF1：返回用途说明串。
         * 注意 offset：特征值长度超过 (MTU-1) 时手机会用 Read Blob 分段读，
         * 必须按 offset 返回对应片段，否则手机会读到重复内容。 */
        const uint8_t *src;
        uint16_t total;

        if (param->read.handle == s_write_char_handle) {
            if (s_last_rx_len > 0) {
                src = s_last_rx;
                total = s_last_rx_len;
            } else {
                src = (const uint8_t *)s_write_doc;
                total = (uint16_t)strlen(s_write_doc);
            }
        } else {
            src = (const uint8_t *)s_notify_doc;
            total = (uint16_t)strlen(s_notify_doc);
        }

        uint16_t offset = param->read.offset;
        if (offset > total) {
            offset = total;                 /* 越界：返回空片段 */
        }
        uint16_t n = (uint16_t)(total - offset);

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.offset = offset;
        rsp.attr_value.len = n;
        if (n > 0) {
            memcpy(rsp.attr_value.value, src + offset, n);
        }

        if (param->read.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
        }
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:
        esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id,
                                    ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_CONF_EVT:
        if (param->conf.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "Indicate/Notify 确认异常: 0x%x", param->conf.status);
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------
 * 对外接口
 * --------------------------------------------------------------------- */
void ble_server_init(void)
{
    ESP_LOGI(TAG, "初始化 BLE GATT Server（设备名 \"%s\"，0x%04X: 通知 0x%04X / 写 0x%04X）",
             BLE_SERVER_ADV_NAME, BLE_SERVER_SERVICE_UUID,
             BLE_SERVER_NOTIFY_CHAR_UUID, BLE_SERVER_WRITE_CHAR_UUID);

    if (s_mtx == NULL) {
        s_mtx = xSemaphoreCreateMutex();
    }

    /* 两个特征的默认可读内容（说明用途，手机读特征即可看到） */
    size_t n = strlen(s_notify_doc);
    if (n > sizeof(s_notify_value)) {
        n = sizeof(s_notify_value);
    }
    memcpy(s_notify_value, s_notify_doc, n);
    s_notify_attr.attr_len = (uint16_t)n;

    n = strlen(s_write_doc);
    if (n > sizeof(s_write_value)) {
        n = sizeof(s_write_value);
    }
    memcpy(s_write_value, s_write_doc, n);
    s_write_attr.attr_len = (uint16_t)n;

    /* GAP 事件统一由 ble_gap 分发（协议栈只允许一个 GAP 回调） */
    esp_err_t ret = ble_gap_add_handler(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP 处理函数注册失败: 0x%x", ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS 回调注册失败: 0x%x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS 应用注册失败: 0x%x", ret);
        return;
    }

    /* 让 GAP 层的设备名与广播名一致（注意：不要与经典蓝牙的设备名混用） */
    ret = esp_ble_gap_set_device_name(BLE_SERVER_ADV_NAME);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 GAP 设备名失败: 0x%x", ret);
    }

    ESP_LOGI(TAG, "初始化完成，等待属性表创建完成事件");
}

int ble_server_send(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }

    uint16_t cid;
    uint8_t mode;
    uint16_t mtu;
    uint16_t ch;
    ble_server_lock();
    cid = s_conn_id;
    mode = s_notify_mode;
    mtu = s_mtu;
    ch = s_notify_char_handle;
    ble_server_unlock();

    if (cid == 0 || s_gatts_if == ESP_GATT_IF_NONE) {
        return -2;                          /* 手机未接入 */
    }
    if (mode == 0 || ch == 0) {
        return -3;                          /* 手机未订阅 0xFFF1 通知 */
    }
    uint16_t max_len = (mtu > 3) ? (uint16_t)(mtu - 3) : 20;
    if (len > max_len) {
        ESP_LOGW(TAG, "待发送数据 %u 字节超过当前 MTU 上限(%u)，已丢弃", (unsigned)len, (unsigned)max_len);
        return -4;
    }

    /* mode==2 表示对端订阅的是 Indicate，需要确认 */
    esp_err_t err = esp_ble_gatts_send_indicate(s_gatts_if, cid, ch, len, data, mode == 2);
    return (err == ESP_OK) ? 0 : (int)err;
}

void ble_server_set_rx_cb(void (*cb)(uint8_t *data, uint16_t len))
{
    s_rx_cb = cb;
}

bool ble_server_is_connected(void)
{
    ble_server_lock();
    bool connected = (s_conn_id != 0);
    ble_server_unlock();
    return connected;
}

bool ble_server_is_notify_enabled(void)
{
    ble_server_lock();
    bool enabled = (s_conn_id != 0) && (s_notify_mode != 0);
    ble_server_unlock();
    return enabled;
}
