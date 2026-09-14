/**
 * @file ble_client.c
 * @brief 蓝牙 BLE GATT Client（主机）模块实现
 *
 * 流程（ESP-IDF 5.x，服务发现自动进行）：
 *   注册 GATTC → 设扫描参数 → 扫描 → 名称/UUID 匹配 → gattc_open
 *   → 自动服务发现完成(DIS_SRVC_CMPL) → search_service 匹配目标服务
 *   → 本地缓存查询写/通知特征 → 写 CCCD 使能 Notify → 进入 CONNECTED
 *
 * 稳定性设计：
 *   - 状态机 + 三个 FreeRTOS 软件定时器（扫描/连接/重连）覆盖所有异步超时；
 *   - 异常断开延时自动重连；手动断开暂停重连；
 *   - 找到目标设备立即停止扫描，降低对 A2DP 音频的射频干扰。
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "ble_client.h"

static const char *TAG = "ble_client";

/* GATT Client 应用 ID（本工程仅注册一个） */
#define BLE_CLIENT_APP_ID       0

/* ---------------------------------------------------------------------
 * 内部状态机
 * --------------------------------------------------------------------- */
typedef enum {
    BLE_C_STATE_IDLE = 0,       /* 未连接（空闲） */
    BLE_C_STATE_SCANNING,       /* 扫描中 */
    BLE_C_STATE_CONNECTING,     /* 连接中（含服务发现 / 使能通知） */
    BLE_C_STATE_CONNECTED,      /* 已连接，Notify 已使能 */
} ble_c_state_t;

/* ---------------------------------------------------------------------
 * 全局上下文（BLE 回调运行于 btc 任务，定时器回调运行于 Tmr Svc 任务）
 * --------------------------------------------------------------------- */
static SemaphoreHandle_t s_state_mtx = NULL;   /* 保护跨任务共享状态 */
static esp_gatt_if_t    s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t         s_conn_id = 0;
static esp_bd_addr_t    s_remote_bda = {0};
static ble_c_state_t    s_state = BLE_C_STATE_IDLE;
static bool             s_auto_reconnect = true;   /* 是否自动重连（手动断开时置 false） */

/* 服务 / 特征句柄与匹配标记 */
static bool     s_svc_found = false;
static uint16_t s_svc_start = 0;
static uint16_t s_svc_end = 0;
static uint16_t s_write_char_handle = 0;
static uint16_t s_notify_char_handle = 0;
static uint16_t s_cccd_handle = 0;

/* 由宏解析出的目标 UUID */
static esp_bt_uuid_t s_svc_uuid;
static esp_bt_uuid_t s_write_char_uuid;
static esp_bt_uuid_t s_notify_char_uuid;
static bool s_have_svc_uuid = false;
static bool s_have_write_uuid = false;
static bool s_have_notify_uuid = false;

/* 上层接收回调 */
static void (*s_rx_cb)(uint8_t *data, uint16_t len) = NULL;

/* 定时器（均一次性） */
static TimerHandle_t s_scan_timer = NULL;
static TimerHandle_t s_conn_timer = NULL;
static TimerHandle_t s_reconnect_timer = NULL;

/* 扫描参数 */
static esp_ble_scan_params_t s_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,   /* 若对端用随机地址，可改为 BLE_ADDR_TYPE_RANDOM */
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = BLE_CLIENT_SCAN_INTERVAL,
    .scan_window        = BLE_CLIENT_SCAN_WINDOW,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
};

/* ---------------------------------------------------------------------
 * 前向声明
 * --------------------------------------------------------------------- */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void ble_c_start_scan(void);
static void ble_c_start_reconnect(void);
static void ble_c_on_connected(void);
static void ble_c_on_connect_failed(void);
static void ble_c_on_disconnected(esp_gatt_conn_reason_t reason);
static void ble_c_query_chars(void);

/* ---------------------------------------------------------------------
 * 状态访问（互斥保护）
 * --------------------------------------------------------------------- */
static ble_c_state_t ble_c_state_get(void)
{
    ble_c_state_t st;
    if (s_state_mtx) {
        xSemaphoreTake(s_state_mtx, portMAX_DELAY);
    }
    st = s_state;
    if (s_state_mtx) {
        xSemaphoreGive(s_state_mtx);
    }
    return st;
}

static void ble_c_set_state(ble_c_state_t st)
{
    if (s_state_mtx) {
        xSemaphoreTake(s_state_mtx, portMAX_DELAY);
    }
    s_state = st;
    if (s_state_mtx) {
        xSemaphoreGive(s_state_mtx);
    }
}

/* ---------------------------------------------------------------------
 * 工具函数
 * --------------------------------------------------------------------- */

/* 构造 16 位 UUID */
static void ble_c_uuid16_set(esp_bt_uuid_t *out, uint16_t val)
{
    out->len = ESP_UUID_LEN_16;
    out->uuid.uuid16 = val;
}

/* 比较两个 UUID 是否相等（兼容 16 位与 128 位交叉比较） */
static bool uuid_equal(const esp_bt_uuid_t *a, const esp_bt_uuid_t *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }
    if (a->len == b->len) {
        if (a->len == ESP_UUID_LEN_16) {
            return a->uuid.uuid16 == b->uuid.uuid16;
        }
        if (a->len == ESP_UUID_LEN_32) {
            return a->uuid.uuid32 == b->uuid.uuid32;
        }
        if (a->len == ESP_UUID_LEN_128) {
            return memcmp(a->uuid.uuid128, b->uuid.uuid128, ESP_UUID_LEN_128) == 0;
        }
        return false;
    }
    /* 长度不同：16 位 vs 128 位，取 128 位 UUID 的低 16 位比较 */
    if (a->len == ESP_UUID_LEN_128 && b->len == ESP_UUID_LEN_16) {
        return (a->uuid.uuid128[12] | (a->uuid.uuid128[13] << 8)) == b->uuid.uuid16;
    }
    if (a->len == ESP_UUID_LEN_16 && b->len == ESP_UUID_LEN_128) {
        return a->uuid.uuid16 == (b->uuid.uuid128[12] | (b->uuid.uuid128[13] << 8));
    }
    return false;
}

/* 在广播数据 / 扫描响应中解析设备名（先 CMPL 后 SHORT） */
static uint8_t *ble_c_resolve_name(uint8_t *adv, uint16_t adv_len, uint16_t scan_rsp_len, uint8_t *out_len)
{
    uint8_t *p = NULL;
    p = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_NAME_CMPL, out_len);
    if (p != NULL) return p;
    if (scan_rsp_len) {
        p = esp_ble_resolve_adv_data_by_type(adv + adv_len, scan_rsp_len, ESP_BLE_AD_TYPE_NAME_CMPL, out_len);
        if (p != NULL) return p;
    }
    p = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_NAME_SHORT, out_len);
    if (p != NULL) return p;
    if (scan_rsp_len) {
        p = esp_ble_resolve_adv_data_by_type(adv + adv_len, scan_rsp_len, ESP_BLE_AD_TYPE_NAME_SHORT, out_len);
        if (p != NULL) return p;
    }
    return NULL;
}

/* 判断广播数据段内是否包含目标服务 UUID（兼容 16 位 / 128 位广播） */
static bool ble_c_adv_has_svc_uuid(uint8_t *buf, uint16_t len)
{
    if (!s_have_svc_uuid) {
        return false;
    }
    uint16_t target = s_svc_uuid.uuid.uuid16;
    uint8_t found_len = 0;

    /* 16 位服务 UUID 列表（AD 类型 0x02/0x03） */
    uint8_t *p = esp_ble_resolve_adv_data_by_type(buf, len, ESP_BLE_AD_TYPE_16SRV_CMPL, &found_len);
    if (p == NULL) {
        p = esp_ble_resolve_adv_data_by_type(buf, len, ESP_BLE_AD_TYPE_16SRV_PART, &found_len);
    }
    if (p != NULL) {
        for (int i = 0; i + 1 < found_len; i += 2) {
            if ((p[i] | (p[i + 1] << 8)) == target) {
                return true;
            }
        }
    }

    /* 128 位服务 UUID（AD 类型 0x06/0x07）：取低 16 位比较 */
    p = esp_ble_resolve_adv_data_by_type(buf, len, ESP_BLE_AD_TYPE_128SRV_CMPL, &found_len);
    if (p == NULL) {
        p = esp_ble_resolve_adv_data_by_type(buf, len, ESP_BLE_AD_TYPE_128SRV_PART, &found_len);
    }
    if (p != NULL && found_len >= ESP_UUID_LEN_128) {
        if ((p[12] | (p[13] << 8)) == target) {
            return true;
        }
    }
    return false;
}

/* 调试：打印每个扫描到的设备（名称 / 地址 / RSSI），便于核对目标设备标识 */
static void ble_c_dump_scan(esp_ble_gap_cb_param_t *scan)
{
#if BLE_CLIENT_SCAN_LOG_ALL
    uint8_t name_len = 0;
    uint8_t *name = ble_c_resolve_name(scan->scan_rst.ble_adv,
                                       scan->scan_rst.adv_data_len,
                                       scan->scan_rst.scan_rsp_len, &name_len);
    char buf[33];
    int n = (name != NULL && name_len < (int)sizeof(buf)) ? (int)name_len : 0;
    if (n > 0) {
        memcpy(buf, name, n);
    }
    buf[n] = '\0';
    ESP_LOGI(TAG, "[扫描] %02x:%02x:%02x:%02x:%02x:%02x RSSI=%-4d 名称='%s'",
             scan->scan_rst.bda[0], scan->scan_rst.bda[1], scan->scan_rst.bda[2],
             scan->scan_rst.bda[3], scan->scan_rst.bda[4], scan->scan_rst.bda[5],
             scan->scan_rst.rssi, (n > 0) ? buf : "(无)");
#endif
}

/* 扫描结果是否匹配目标设备（名称前缀优先，服务 UUID 兜底） */
static bool ble_c_scan_match(esp_ble_gap_cb_param_t *scan)
{
    uint8_t *adv = scan->scan_rst.ble_adv;
    uint16_t adv_len = scan->scan_rst.adv_data_len;
    uint16_t scan_rsp_len = scan->scan_rst.scan_rsp_len;

    /* 未配置任何过滤条件：连接第一个扫描到的设备（调试用） */
    if (TARGET_DEV_NAME_PREFIX[0] == '\0' && !s_have_svc_uuid) {
        ESP_LOGW(TAG, "未配置名称前缀/服务 UUID，连接第一个扫描到的设备");
        return true;
    }

    /* ① 名称前缀匹配 */
    if (TARGET_DEV_NAME_PREFIX[0] != '\0') {
        uint8_t name_len = 0;
        uint8_t *name = ble_c_resolve_name(adv, adv_len, scan_rsp_len, &name_len);
        if (name != NULL) {
            size_t plen = strlen(TARGET_DEV_NAME_PREFIX);
            if (name_len >= plen && memcmp(name, TARGET_DEV_NAME_PREFIX, plen) == 0) {
                return true;
            }
        }
    }

    /* ② 服务 UUID 匹配（名称不匹配或解析不到时兜底） */
    if (s_have_svc_uuid) {
        if (ble_c_adv_has_svc_uuid(adv, adv_len)) {
            return true;
        }
        if (scan_rsp_len && ble_c_adv_has_svc_uuid(adv + adv_len, scan_rsp_len)) {
            return true;
        }
    }

    return false;
}

/* ---------------------------------------------------------------------
 * 扫描 / 连接 / 重连控制
 * --------------------------------------------------------------------- */

/* 开始扫描（0 = 持续扫描，由 s_scan_timer 负责超时） */
static void ble_c_start_scan(void)
{
    if (ble_c_state_get() == BLE_C_STATE_SCANNING) {
        return;
    }
    ESP_LOGI(TAG, "开始扫描目标设备...");
    ble_c_set_state(BLE_C_STATE_SCANNING);
    if (esp_ble_gap_start_scanning(0) != ESP_OK) {
        ESP_LOGE(TAG, "启动扫描失败");
        ble_c_set_state(BLE_C_STATE_IDLE);
        ble_c_start_reconnect();
        return;
    }
    xTimerStart(s_scan_timer, 0);
}

/* 进入重连流程 */
static void ble_c_start_reconnect(void)
{
    ble_c_set_state(BLE_C_STATE_IDLE);
    if (!s_auto_reconnect) {
        ESP_LOGI(TAG, "自动重连已关闭（手动断开）");
        return;
    }
    ESP_LOGI(TAG, "%d ms 后自动重连", (int)BLE_CLIENT_RECONNECT_DELAY_MS);
    xTimerStart(s_reconnect_timer, 0);
}

/* 连接成功（Notify 已使能） */
static void ble_c_on_connected(void)
{
    xTimerStop(s_conn_timer, 0);
    ble_c_set_state(BLE_C_STATE_CONNECTED);
    ESP_LOGI(TAG, "目标设备已连接，Notify 已使能");
}

/* 连接 / 发现失败：清理并重连 */
static void ble_c_on_connect_failed(void)
{
    xTimerStop(s_conn_timer, 0);
    uint16_t cid = s_conn_id;
    s_conn_id = 0;
    if (cid != 0) {
        esp_ble_gattc_close(s_gattc_if, cid);
    }
    ble_c_start_reconnect();
}

/* 连接断开：清理并（按需）重连 */
static void ble_c_on_disconnected(esp_gatt_conn_reason_t reason)
{
    /* 防止 CLOSE_EVT 与 DISCONNECT_EVT 重复处理 */
    if (ble_c_state_get() == BLE_C_STATE_IDLE) {
        return;
    }
    xTimerStop(s_scan_timer, 0);
    xTimerStop(s_conn_timer, 0);
    ble_c_set_state(BLE_C_STATE_IDLE);
    s_conn_id = 0;
    s_svc_found = false;
    s_write_char_handle = 0;
    s_notify_char_handle = 0;
    s_cccd_handle = 0;
    ESP_LOGW(TAG, "连接断开, reason=0x%02x", reason);
    ble_c_start_reconnect();
}

/* 在本地 GATTC 缓存中查询目标特征并写 CCCD 使能 Notify */
static void ble_c_query_chars(void)
{
    /* 查可写特征 */
    if (s_have_write_uuid) {
        esp_gattc_char_elem_t ch;
        uint16_t cnt = 1;
        esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_conn_id,
                                                              s_svc_start, s_svc_end,
                                                              s_write_char_uuid, &ch, &cnt);
        if (st == ESP_GATT_OK && cnt > 0) {
            s_write_char_handle = ch.char_handle;
            ESP_LOGI(TAG, "找到可写特征: handle=0x%04x", s_write_char_handle);
        } else {
            ESP_LOGW(TAG, "未找到可写特征");
        }
    }

    /* 查通知特征 */
    if (s_have_notify_uuid) {
        esp_gattc_char_elem_t ch;
        uint16_t cnt = 1;
        esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_conn_id,
                                                              s_svc_start, s_svc_end,
                                                              s_notify_char_uuid, &ch, &cnt);
        if (st == ESP_GATT_OK && cnt > 0) {
            s_notify_char_handle = ch.char_handle;
            ESP_LOGI(TAG, "找到通知特征: handle=0x%04x", s_notify_char_handle);
        } else {
            ESP_LOGW(TAG, "未找到通知特征");
        }
    }

    /* 写 CCCD（0x2902）使能 Notify（0x0001） */
    if (s_notify_char_handle != 0) {
        esp_bt_uuid_t cccd = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
        };
        esp_gattc_descr_elem_t desc;
        uint16_t cnt = 1;
        esp_gatt_status_t st = esp_ble_gattc_get_descr_by_char_handle(s_gattc_if, s_conn_id,
                                                                      s_notify_char_handle,
                                                                      cccd, &desc, &cnt);
        if (st == ESP_GATT_OK && cnt > 0) {
            s_cccd_handle = desc.handle;
            uint16_t notify_en = 0x0001;    /* 0x0001=Notify, 0x0002=Indicate */
            esp_err_t err = esp_ble_gattc_write_char_descr(s_gattc_if, s_conn_id, s_cccd_handle,
                                                           sizeof(notify_en), (uint8_t *)&notify_en,
                                                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "写 CCCD 失败: 0x%x", err);
                ble_c_on_connect_failed();
            }
            /* 成功后在 ESP_GATTC_WRITE_DESCR_EVT 中置 CONNECTED */
            return;
        } else {
            ESP_LOGW(TAG, "未找到 CCCD 描述符，无通知能力");
        }
    }

    /* 未配置 / 未找到通知特征：仍视为连接成功（仅可写，无通知） */
    ble_c_on_connected();
}

/* ---------------------------------------------------------------------
 * GAP（扫描）回调
 * --------------------------------------------------------------------- */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ble_c_start_scan();
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "扫描启动失败: 0x%x", param->scan_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "扫描已启动，正在搜索目标设备...");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            break;
        }
        if (ble_c_state_get() != BLE_C_STATE_SCANNING) {
            break;
        }
        ble_c_dump_scan(param);          /* 调试：打印扫描到的设备 */
        if (!ble_c_scan_match(param)) {
            break;
        }

        /* 找到目标设备：立即停止扫描，降低对 A2DP 音频的射频干扰 */
        esp_ble_gap_stop_scanning();
        xTimerStop(s_scan_timer, 0);
        ble_c_set_state(BLE_C_STATE_CONNECTING);

        memcpy(s_remote_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
        ESP_LOGI(TAG, "发现目标设备，发起连接: %02x:%02x:%02x:%02x:%02x:%02x",
                 s_remote_bda[0], s_remote_bda[1], s_remote_bda[2],
                 s_remote_bda[3], s_remote_bda[4], s_remote_bda[5]);

        esp_err_t err = esp_ble_gattc_open(s_gattc_if, param->scan_rst.bda,
                                           param->scan_rst.ble_addr_type, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "发起连接失败: 0x%x", err);
            ble_c_start_reconnect();
            break;
        }
        /* 连接 + 服务发现 + 使能通知 的总超时 */
        xTimerStart(s_conn_timer, 0);
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "扫描已停止");
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------
 * GATT Client 回调
 * --------------------------------------------------------------------- */
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT: {
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTC 应用注册失败: 0x%x", param->reg.status);
            break;
        }
        s_gattc_if = gattc_if;
        ESP_LOGI(TAG, "GATTC 应用注册成功, gattc_if=%d", s_gattc_if);
        /* 设置扫描参数（完成事件里启动扫描） */
        esp_err_t err = esp_ble_gap_set_scan_params(&s_scan_params);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "设置扫描参数失败: 0x%x", err);
        }
        break;
    }

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "物理连接建立, conn_id=%d", param->connect.conn_id);
        break;

    case ESP_GATTC_OPEN_EVT: {
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATT 打开失败: 0x%x", param->open.status);
            ble_c_on_connect_failed();
            break;
        }
        s_conn_id = param->open.conn_id;
        ESP_LOGI(TAG, "GATT 连接已建立, conn_id=%d, mtu=%d", s_conn_id, param->open.mtu);
        /* 协商更大 MTU（可选）；服务发现由协议栈自动完成 */
        esp_ble_gattc_send_mtu_req(s_gattc_if, s_conn_id);
        break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG, "MTU 协商完成: %d", param->cfg_mtu.mtu);
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT: {
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "服务发现失败: 0x%x", param->dis_srvc_cmpl.status);
            ble_c_on_connect_failed();
            break;
        }
        ESP_LOGI(TAG, "服务发现完成，搜索目标服务...");
        s_svc_found = false;
        s_svc_start = 0;
        s_svc_end = 0;
        /* 在本地缓存中搜索所有服务（NULL = 全部），匹配目标服务 */
        esp_err_t err = esp_ble_gattc_search_service(s_gattc_if, s_conn_id, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "搜索服务失败: 0x%x", err);
            ble_c_on_connect_failed();
        }
        break;
    }

    case ESP_GATTC_SEARCH_RES_EVT:
        if (s_have_svc_uuid && !s_svc_found &&
            uuid_equal(&param->search_res.srvc_id.uuid, &s_svc_uuid)) {
            s_svc_start = param->search_res.start_handle;
            s_svc_end = param->search_res.end_handle;
            s_svc_found = true;
            ESP_LOGI(TAG, "匹配到目标服务: handle [0x%04x ~ 0x%04x]", s_svc_start, s_svc_end);
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "搜索服务状态错误: 0x%x", param->search_cmpl.status);
            ble_c_on_connect_failed();
            break;
        }
        if (!s_have_svc_uuid) {
            /* 未配置服务 UUID：退化到全范围查询特征 */
            s_svc_start = 0x0001;
            s_svc_end = 0xFFFF;
            ble_c_query_chars();
        } else if (s_svc_found) {
            ble_c_query_chars();
        } else {
            ESP_LOGE(TAG, "未找到目标服务");
            ble_c_on_connect_failed();
        }
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "CCCD 写入失败: 0x%x", param->write.status);
            ble_c_on_connect_failed();
        } else {
            ESP_LOGI(TAG, "Notify 已使能");
            ble_c_on_connected();
        }
        break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGD(TAG, "写特征结果: handle=0x%04x status=0x%x", param->write.handle, param->write.status);
        break;

    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGD(TAG, "读特征结果: handle=0x%04x status=0x%x", param->read.handle, param->read.status);
        break;

    case ESP_GATTC_NOTIFY_EVT:
        /* 收到对端通知 / 指示数据，转发给上层回调 */
        if (s_rx_cb) {
            s_rx_cb(param->notify.value, param->notify.value_len);
        }
        break;

    case ESP_GATTC_CLOSE_EVT:
        ESP_LOGI(TAG, "GATT 虚拟连接关闭, reason=0x%x", param->close.reason);
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ble_c_on_disconnected(param->disconnect.reason);
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------
 * 定时器回调
 * --------------------------------------------------------------------- */
static void scan_timer_cb(TimerHandle_t t)
{
    if (ble_c_state_get() == BLE_C_STATE_SCANNING) {
        ESP_LOGW(TAG, "扫描超时，未找到目标设备");
        esp_ble_gap_stop_scanning();
        ble_c_start_reconnect();
    }
}

static void conn_timer_cb(TimerHandle_t t)
{
    if (ble_c_state_get() == BLE_C_STATE_CONNECTING) {
        ESP_LOGW(TAG, "连接/服务发现超时，清理并重连");
        uint16_t cid = s_conn_id;
        s_conn_id = 0;
        if (cid != 0) {
            esp_ble_gattc_close(s_gattc_if, cid);
        }
        ble_c_start_reconnect();
    }
}

static void reconnect_timer_cb(TimerHandle_t t)
{
    if (ble_c_state_get() == BLE_C_STATE_IDLE && s_auto_reconnect) {
        ble_c_start_scan();
    }
}

/* ---------------------------------------------------------------------
 * 对外接口实现
 * --------------------------------------------------------------------- */
void ble_client_init(void)
{
    ESP_LOGI(TAG, "初始化 BLE GATT Client");

    s_state_mtx = xSemaphoreCreateMutex();

    /* 创建一次性定时器 */
    s_scan_timer = xTimerCreate("ble_scan", pdMS_TO_TICKS(BLE_CLIENT_SCAN_TIMEOUT_MS),
                                pdFALSE, NULL, scan_timer_cb);
    s_conn_timer = xTimerCreate("ble_conn", pdMS_TO_TICKS(BLE_CLIENT_CONNECT_TIMEOUT_MS),
                                pdFALSE, NULL, conn_timer_cb);
    s_reconnect_timer = xTimerCreate("ble_reconn", pdMS_TO_TICKS(BLE_CLIENT_RECONNECT_DELAY_MS),
                                     pdFALSE, NULL, reconnect_timer_cb);

    /* 构造 16 位目标 UUID（宏为 0 则跳过对应过滤 / 查询） */
    if (TARGET_SERVICE_UUID != 0) {
        ble_c_uuid16_set(&s_svc_uuid, TARGET_SERVICE_UUID);
        s_have_svc_uuid = true;
    }
    if (WRITE_CHAR_UUID != 0) {
        ble_c_uuid16_set(&s_write_char_uuid, WRITE_CHAR_UUID);
        s_have_write_uuid = true;
    }
    if (NOTIFY_CHAR_UUID != 0) {
        ble_c_uuid16_set(&s_notify_char_uuid, NOTIFY_CHAR_UUID);
        s_have_notify_uuid = true;
    }

    /* 注册 GAP 与 GATTC 回调 */
    esp_err_t ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP 回调注册失败: 0x%x", ret);
        return;
    }
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC 回调注册失败: 0x%x", ret);
        return;
    }
    /* 应用注册成功后（ESP_GATTC_REG_EVT）再启动扫描 */
    ret = esp_ble_gattc_app_register(BLE_CLIENT_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC 应用注册失败: 0x%x", ret);
        return;
    }

    s_auto_reconnect = true;
    ESP_LOGI(TAG, "初始化完成，等待应用注册事件");
}

int ble_client_send(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    if (ble_c_state_get() != BLE_C_STATE_CONNECTED) {
        return -2;
    }
    if (s_write_char_handle == 0) {
        return -3;
    }
    esp_err_t err = esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_write_char_handle,
                                             len, data,
                                             ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
    return (err == ESP_OK) ? 0 : (int)err;
}

void ble_client_set_rx_cb(void (*cb)(uint8_t *data, uint16_t len))
{
    s_rx_cb = cb;
}

uint8_t ble_client_get_state(void)
{
    ble_c_state_t st = ble_c_state_get();
    if (st == BLE_C_STATE_CONNECTED) {
        return BLE_CLIENT_STATE_CONNECTED;
    }
    if (st == BLE_C_STATE_IDLE) {
        return BLE_CLIENT_STATE_DISCONNECTED;
    }
    return BLE_CLIENT_STATE_CONNECTING;
}

void ble_client_connect(void)
{
    s_auto_reconnect = true;
    if (ble_c_state_get() == BLE_C_STATE_IDLE) {
        ble_c_start_scan();
    } else {
        ESP_LOGI(TAG, "已在连接流程中或已连接");
    }
}

void ble_client_disconnect(void)
{
    s_auto_reconnect = false;
    xTimerStop(s_scan_timer, 0);
    xTimerStop(s_conn_timer, 0);
    xTimerStop(s_reconnect_timer, 0);
    esp_ble_gap_stop_scanning();

    if (ble_c_state_get() != BLE_C_STATE_IDLE && s_conn_id != 0) {
        ESP_LOGI(TAG, "手动断开连接");
        esp_ble_gattc_close(s_gattc_if, s_conn_id);
        /* 后续 CLOSE / DISCONNECT 事件会清理状态；因 auto_reconnect=false 不会重连 */
    } else {
        ble_c_set_state(BLE_C_STATE_IDLE);
    }
}
