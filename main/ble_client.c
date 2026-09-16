/**
 * @file ble_client.c
 * @brief 蓝牙 BLE GATT Client（主机）模块实现
 *
 * 流程：
 *   注册 GATTC → 设扫描参数 → 扫描 → 名称前缀匹配（无名设备用服务 UUID 兜底）
 *   → 暂存地址/地址类型 → stop_scanning → **等 SCAN_STOP_COMPLETE**
 *   → esp_ble_gattc_enh_open()（显式 own_addr_type / remote_addr_type / 连接参数）
 *   → 服务发现完成(DIS_SRVC_CMPL) → search_service 匹配目标服务
 *   → 查询写/通知特征 → 写 CCCD 使能 Notify → 进入 CONNECTED
 *
 * 稳定性设计（针对 reason=0x3e“链路未建立”做过加固）：
 *   - 先停扫描、等 SCAN_STOP_COMPLETE 事件后再发起连接，避免控制器在扫描未释放时建链失败；
 *   - 连接参数（地址类型、本地地址类型、interval、timeout）全部显式指定，不依赖默认值；
 *   - GAP 事件由 ble_gap.c 统一分发（Bluedroid 只允许注册一个 GAP 回调）；
 *   - 异常断开/失败走同一套清理逻辑，重连采用指数退避，连续失败到上限后暂停（可手动重连）；
 *   - 不再打印每个扫描结果，只在命中目标时打印一行关键信息（名称/地址/类型/RSSI/长度）。
 */
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "ble_gap.h"
#include "ble_client.h"

static const char *TAG = "ble_client";

/* GATT Client 应用 ID（本工程仅注册一个 client） */
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

/* 名称不符设备的提示缓存（同一地址只提示一次，避免刷屏） */
#define BLE_C_STRANGER_CACHE    8

/* ---------------------------------------------------------------------
 * 全局上下文（BLE 回调运行于 btc 任务，定时器回调运行于 Tmr Svc 任务）
 * --------------------------------------------------------------------- */
static SemaphoreHandle_t s_state_mtx = NULL;   /* 保护跨任务共享状态 */
static esp_gatt_if_t    s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t         s_conn_id = 0;
static ble_c_state_t    s_state = BLE_C_STATE_IDLE;
static bool             s_auto_reconnect = true;   /* 是否自动重连（手动断开时置 false） */

/* 目标设备暂存（在扫描命中时填充，等扫描真正停止后再发起连接） */
static bool              s_pending_open = false;   /* 已命中目标，等待 SCAN_STOP_COMPLETE */
static esp_bd_addr_t     s_target_bda = {0};
static esp_ble_addr_type_t s_target_addr_type = BLE_ADDR_TYPE_PUBLIC;
static char              s_target_name[33] = {0};

/* 重连控制 */
static uint32_t s_fail_count = 0;               /* 连续失败次数（成功后清零） */
static bool     s_reconnect_scheduled = false;  /* 是否已排程一次重连（去重） */

/* 链路状态 */
static bool     s_congested = false;

/* 服务 / 特征句柄与匹配标记 */
static bool     s_svc_found = false;
static uint16_t s_svc_start = 0;
static uint16_t s_svc_end = 0;
static uint16_t s_write_char_handle = 0;
static uint16_t s_write_char_uuid_used = 0;   /* 实际命中的写特征 UUID（0xFFF2 或 0xFFF1） */
static uint16_t s_notify_char_handle = 0;
static uint16_t s_cccd_handle = 0;

/* 由宏解析出的目标 UUID（写特征有三个候选：FFF2 → FFF1 → FFF3） */
static esp_bt_uuid_t s_svc_uuid;
static esp_bt_uuid_t s_notify_char_uuid;
static esp_bt_uuid_t s_write_primary_uuid;
static esp_bt_uuid_t s_write_fallback_uuid;
static esp_bt_uuid_t s_write_fallback2_uuid;
static bool s_have_svc_uuid = false;
static bool s_have_notify_uuid = false;
static bool s_have_write_primary = false;
static bool s_have_write_fallback = false;
static bool s_have_write_fallback2 = false;
static bool s_notify_registered = false;   /* 是否已调用 esp_ble_gattc_register_for_notify */

/* ---------------------------------------------------------------------
 * 连接参数档位表
 *   第一档就是手表实测可用的参数（30~50ms）；失败时按表循环换档重试。
 * --------------------------------------------------------------------- */
typedef struct {
    const char *name;
    uint16_t scan_interval;
    uint16_t scan_window;
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t latency;
    uint16_t timeout;
} ble_c_conn_profile_t;

static const ble_c_conn_profile_t s_profiles[] = {
    /* 0：手表实测可用（连 OV_WATCH 一次成功、双向收发正常，见 README 第 6 节） */
    { "30~50ms/超时4s", BLE_CLIENT_INIT_SCAN_INTERVAL, BLE_CLIENT_INIT_SCAN_WINDOW,
      BLE_CLIENT_CONN_INTERVAL_MIN, BLE_CLIENT_CONN_INTERVAL_MAX,
      BLE_CLIENT_CONN_LATENCY, BLE_CLIENT_CONN_TIMEOUT },
    /* 1：备用档，间隔更宽松，给响应较慢的模组留余量 */
    { "50~80ms/超时6s", BLE_CLIENT_INIT_SCAN_INTERVAL, BLE_CLIENT_INIT_SCAN_WINDOW,
      0x28, 0x40, 0, 0x0258 },
};

/* ---------------------------------------------------------------------
 * 调试期用过、对 OV_WATCH **无效**的档位（保留备查；需要时按上面同样格式加回表里）：
 *
 *   { "协议栈默认(12.5~15ms)", ... 不指定 phy_mask，交给协议栈默认 ... }
 *     → 能建立物理连接，但约 6s 后 supervision timeout 断开
 *       （BT_HCI: hcif disc complete hdl 0x0, rsn 0x8；对端跟不上 12.5ms 的连接间隔）。
 *       说明：Bluedroid 默认 BTM_BLE_CONN_INT_MIN_DEF=10(12.5ms) / MAX_DEF=12(15ms)。
 *
 *   { "30~50ms+本机随机地址", ... own_addr_type=BLE_ADDR_TYPE_RANDOM + esp_ble_gap_set_rand_addr() ... }
 *     → 用于排查对端是否按"中心地址"做过滤/绑定；本次未用到（第一档即连上）。
 * --------------------------------------------------------------------- */
#define BLE_C_PROFILE_COUNT     (sizeof(s_profiles) / sizeof(s_profiles[0]))

static uint8_t s_profile_idx = 0;   /* 当前使用的档位；连上后保持不变，失败则前进 */

/* 上层接收回调 */
static void (*s_rx_cb)(uint8_t *data, uint16_t len) = NULL;

/* 定时器（均一次性） */
static TimerHandle_t s_scan_timer = NULL;
static TimerHandle_t s_conn_timer = NULL;
static TimerHandle_t s_reconnect_timer = NULL;

/* 名称提示缓存 */
static uint8_t s_stranger_bda[BLE_C_STRANGER_CACHE][ESP_BD_ADDR_LEN];
static uint8_t s_stranger_count = 0;

/* 扫描参数（仅用于"发现目标设备"阶段） */
static esp_ble_scan_params_t s_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,   /* 全工程不使用 RPA，避免地址类型不匹配 */
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
static void ble_c_reset_link(void);
static void ble_c_schedule_reconnect(void);
static void ble_c_fail_and_reconnect(const char *why);
static void ble_c_open_target(void);
static void ble_c_on_connected(void);
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

/* 该广播里是否带目标服务 UUID（广播 + 扫描响应） */
static bool ble_c_scan_has_svc_uuid(const esp_ble_gap_cb_param_t *scan)
{
    const uint8_t *adv = scan->scan_rst.ble_adv;
    uint16_t adv_len = scan->scan_rst.adv_data_len;
    uint16_t rsp_len = scan->scan_rst.scan_rsp_len;
    if (adv_len && ble_c_adv_has_svc_uuid((uint8_t *)adv, adv_len)) {
        return true;
    }
    if (rsp_len && ble_c_adv_has_svc_uuid((uint8_t *)(adv + adv_len), rsp_len)) {
        return true;
    }
    return false;
}

/* 同一地址只提示一次 */
static bool ble_c_stranger_first_seen(const uint8_t *bda)
{
    for (uint8_t i = 0; i < s_stranger_count; i++) {
        if (memcmp(s_stranger_bda[i], bda, ESP_BD_ADDR_LEN) == 0) {
            return false;
        }
    }
    if (s_stranger_count < BLE_C_STRANGER_CACHE) {
        memcpy(s_stranger_bda[s_stranger_count++], bda, ESP_BD_ADDR_LEN);
    }
    return true;
}

/**
 * 扫描结果是否命中目标设备。
 *   ① 名称前缀命中主目标（手表 OV_WATCH）或备用目标（手机从机 "Ace 2V"，若配置）→ 命中；
 *   ② 有名字但不匹配：忽略（避免误连到别的广播 FFF0 的设备），必要时提示一次；
 *   ③ 设备没有名字（解析不到）：退化为服务 UUID 匹配，并提示一次便于核对地址。
 */
static bool ble_c_scan_match(esp_ble_gap_cb_param_t *scan, char *name_out, size_t name_out_sz)
{
    uint8_t *adv = scan->scan_rst.ble_adv;
    uint16_t adv_len = scan->scan_rst.adv_data_len;
    uint16_t scan_rsp_len = scan->scan_rst.scan_rsp_len;

    name_out[0] = '\0';
    if (TARGET_DEV_NAME_PREFIX[0] == '\0' && TARGET_DEV_NAME_PREFIX_ALT[0] == '\0' && !s_have_svc_uuid) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            ESP_LOGW(TAG, "未配置名称前缀/服务 UUID，将连接第一个扫描到的设备（调试模式）");
        }
        return true;
    }

    /* 解析并净化设备名 */
    uint8_t name_len = 0;
    uint8_t *name = ble_c_resolve_name(adv, adv_len, scan_rsp_len, &name_len);
    size_t copy = 0;
    if (name != NULL) {
        copy = name_len;
        if (copy > name_out_sz - 1) {
            copy = name_out_sz - 1;
        }
        memcpy(name_out, name, copy);
    }
    name_out[copy] = '\0';
    for (size_t i = 0; i < copy; i++) {
        if ((uint8_t)name_out[i] < 0x20 || (uint8_t)name_out[i] > 0x7e) {
            name_out[i] = '?';
        }
    }

    /* ① 名称前缀匹配（主目标 / 备用目标） */
    if (TARGET_DEV_NAME_PREFIX[0] != '\0' || TARGET_DEV_NAME_PREFIX_ALT[0] != '\0') {
        const char *prefixes[2] = { TARGET_DEV_NAME_PREFIX, TARGET_DEV_NAME_PREFIX_ALT };
        for (int i = 0; i < 2; i++) {
            size_t plen = strlen(prefixes[i]);
            if (plen == 0) {
                continue;
            }
            if (copy >= plen && memcmp(name_out, prefixes[i], plen) == 0) {
                return true;
            }
        }
        if (copy > 0) {
            /* 有名字但不是目标：只有当它也广播 FFF0 时才提示一次（可能是误匹配来源） */
            if (ble_c_scan_has_svc_uuid(scan) && ble_c_stranger_first_seen(scan->scan_rst.bda)) {
                ESP_LOGW(TAG, "忽略名称不符设备: 名称='%s' 地址=%02x:%02x:%02x:%02x:%02x:%02x RSSI=%d（它也广播 0x%04X）",
                         name_out,
                         scan->scan_rst.bda[0], scan->scan_rst.bda[1], scan->scan_rst.bda[2],
                         scan->scan_rst.bda[3], scan->scan_rst.bda[4], scan->scan_rst.bda[5],
                         scan->scan_rst.rssi, TARGET_SERVICE_UUID);
            }
            return false;
        }
    }

    /* ② 无名设备：服务 UUID 兜底 */
    if (s_have_svc_uuid && ble_c_scan_has_svc_uuid(scan)) {
        if (ble_c_stranger_first_seen(scan->scan_rst.bda)) {
            ESP_LOGW(TAG, "无名称设备命中服务 0x%04X，按兜底策略连接，请确认地址是否为手表: "
                          "%02x:%02x:%02x:%02x:%02x:%02x RSSI=%d",
                     TARGET_SERVICE_UUID,
                     scan->scan_rst.bda[0], scan->scan_rst.bda[1], scan->scan_rst.bda[2],
                     scan->scan_rst.bda[3], scan->scan_rst.bda[4], scan->scan_rst.bda[5],
                     scan->scan_rst.rssi);
        }
        return true;
    }
    return false;
}

/* ---------------------------------------------------------------------
 * 扫描 / 连接 / 重连控制
 * --------------------------------------------------------------------- */

/* 开始扫描（0 = 持续扫描，由 s_scan_timer 负责超时） */
static void ble_c_start_scan(void)
{
    if (ble_c_state_get() != BLE_C_STATE_IDLE) {
        return;             /* 只在空闲状态启动扫描，避免与连接流程打架 */
    }
    if (TARGET_DEV_NAME_PREFIX_ALT[0] != '\0') {
        ESP_LOGI(TAG, "开始扫描目标设备（名称 \"%s\" 或 \"%s\" / 服务 0x%04X）...",
                 TARGET_DEV_NAME_PREFIX, TARGET_DEV_NAME_PREFIX_ALT, TARGET_SERVICE_UUID);
    } else {
        ESP_LOGI(TAG, "开始扫描目标设备（名称 \"%s\" / 服务 0x%04X）...",
                 TARGET_DEV_NAME_PREFIX, TARGET_SERVICE_UUID);
    }
    ble_c_set_state(BLE_C_STATE_SCANNING);
    s_pending_open = false;
    if (esp_ble_gap_start_scanning(0) != ESP_OK) {
        ESP_LOGE(TAG, "启动扫描失败");
        ble_c_set_state(BLE_C_STATE_IDLE);
        ble_c_schedule_reconnect();
        return;
    }
    xTimerStart(s_scan_timer, 0);
}

/* 统一的链路清理：停表 + 状态复位（不含重连动作） */
static void ble_c_reset_link(void)
{
    xTimerStop(s_scan_timer, 0);
    xTimerStop(s_conn_timer, 0);
    s_conn_id = 0;
    s_pending_open = false;
    s_svc_found = false;
    s_svc_start = 0;
    s_svc_end = 0;
    s_write_char_handle = 0;
    s_write_char_uuid_used = 0;
    s_notify_char_handle = 0;
    s_cccd_handle = 0;
    s_notify_registered = false;
    s_congested = false;
    ble_c_set_state(BLE_C_STATE_IDLE);
}

/* 排程一次重连（指数退避，去重） */
static void ble_c_schedule_reconnect(void)
{
    if (!s_auto_reconnect) {
        return;                     /* 手动断开：不复位重连 */
    }
    if (s_reconnect_scheduled) {
        return;                     /* 已有一次重连在排队，避免重复 */
    }
    if (BLE_CLIENT_RECONNECT_MAX_FAIL > 0 && s_fail_count >= BLE_CLIENT_RECONNECT_MAX_FAIL) {
        ESP_LOGE(TAG, "已连续失败 %u 次，暂停自动重连；请确认手表是否处于可连接状态/是否被手机占用，"
                      "排查后调用 ble_client_connect() 重新开始", (unsigned)s_fail_count);
        return;
    }

#if BLE_CLIENT_CONN_PROFILE_LADDER
    /* 每失败一次换下一档连接参数（循环），自动找到对端能接受的参数 */
    if (BLE_C_PROFILE_COUNT > 1) {
        uint8_t next = (uint8_t)((s_profile_idx + 1) % BLE_C_PROFILE_COUNT);
        if (next != s_profile_idx) {
            s_profile_idx = next;
            ESP_LOGW(TAG, "连接参数换档 → 档位 %u/%u：%s",
                     (unsigned)(s_profile_idx + 1), (unsigned)BLE_C_PROFILE_COUNT,
                     s_profiles[s_profile_idx].name);
        }
    }
#endif

    uint32_t shift = (s_fail_count < 5) ? s_fail_count : 5;
    uint32_t delay = BLE_CLIENT_RECONNECT_DELAY_MIN_MS << shift;
    if (delay > BLE_CLIENT_RECONNECT_DELAY_MAX_MS) {
        delay = BLE_CLIENT_RECONNECT_DELAY_MAX_MS;
    }
    s_fail_count++;
    s_reconnect_scheduled = true;
    ESP_LOGI(TAG, "%u ms 后自动重连（已连续失败 %u 次）", (unsigned)delay, (unsigned)s_fail_count);
    xTimerChangePeriod(s_reconnect_timer, pdMS_TO_TICKS(delay), 0);
    xTimerStart(s_reconnect_timer, 0);
}

/* 失败处理：清理链路 + 排程重连 */
static void ble_c_fail_and_reconnect(const char *why)
{
    if (why != NULL) {
        ESP_LOGW(TAG, "%s", why);
    }
    uint16_t cid = s_conn_id;
    ble_c_reset_link();
    if (cid != 0 && s_gattc_if != ESP_GATT_IF_NONE) {
        esp_ble_gattc_close(s_gattc_if, cid);
    }
    ble_c_schedule_reconnect();
}

/* 连接成功（Notify 已使能） */
static void ble_c_on_connected(void)
{
    xTimerStop(s_conn_timer, 0);
    s_reconnect_scheduled = false;
    s_fail_count = 0;
    ble_c_set_state(BLE_C_STATE_CONNECTED);
    ESP_LOGI(TAG, "【连接成功】对端='%s'，Notify %s，写特征=0x%04X，通知特征=0x%04X"
                  "（参数档位 %u/%u: %s；conn_id=%d 地址=%02x:%02x:%02x:%02x:%02x:%02x）",
             s_target_name,
             s_notify_registered ? "已使能" : "未使能（对端无通知特征，只能单向下发）",
             s_write_char_uuid_used, s_notify_char_handle,
             (unsigned)(s_profile_idx + 1), (unsigned)BLE_C_PROFILE_COUNT, s_profiles[s_profile_idx].name,
             s_conn_id,
             s_target_bda[0], s_target_bda[1], s_target_bda[2],
             s_target_bda[3], s_target_bda[4], s_target_bda[5]);
}

/* 扫描已停：正式发起连接（按当前档位指定连接参数与地址类型） */
static void ble_c_open_target(void)
{
    if (ble_c_state_get() != BLE_C_STATE_CONNECTING) {
        return;
    }

    const ble_c_conn_profile_t *pf = &s_profiles[s_profile_idx];

    /* 显式参数：同时写进"该设备的首选参数"和本次 enh_open 的参数，确保真正生效
     * （手表实测：30~50ms 可用；协议栈默认 12.5~15ms 会 supervision timeout） */
    esp_err_t perr = esp_ble_gap_set_prefer_conn_params(s_target_bda, pf->interval_min,
                                                        pf->interval_max, pf->latency, pf->timeout);
    if (perr != ESP_OK) {
        ESP_LOGW(TAG, "设置首选连接参数失败: 0x%x", perr);
    }

    esp_ble_conn_params_t cp = {
        .scan_interval       = pf->scan_interval,
        .scan_window         = pf->scan_window,
        .interval_min        = pf->interval_min,
        .interval_max        = pf->interval_max,
        .latency             = pf->latency,
        .supervision_timeout = pf->timeout,
        .min_ce_len          = 0,
        .max_ce_len          = 0,
    };

    esp_ble_gatt_creat_conn_params_t ccp = {0};
    memcpy(ccp.remote_bda, s_target_bda, ESP_BD_ADDR_LEN);
    ccp.remote_addr_type   = s_target_addr_type;
    ccp.is_direct          = true;
    ccp.is_aux             = false;
    ccp.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;        /* 使用公共地址，避免地址类型不匹配 */
    ccp.phy_mask           = ESP_BLE_PHY_1M_PREF_MASK;
    ccp.phy_1m_conn_params = &cp;

    ESP_LOGI(TAG, "发起连接: %02x:%02x:%02x:%02x:%02x:%02x "
                  "(档位 %u/%u: %s; addr_type=%d, own_addr_type=%d)",
             s_target_bda[0], s_target_bda[1], s_target_bda[2],
             s_target_bda[3], s_target_bda[4], s_target_bda[5],
             (unsigned)(s_profile_idx + 1), (unsigned)BLE_C_PROFILE_COUNT, pf->name,
             s_target_addr_type, BLE_ADDR_TYPE_PUBLIC);

    esp_err_t err = esp_ble_gattc_enh_open(s_gattc_if, &ccp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "发起连接调用失败: 0x%x", err);
        ble_c_fail_and_reconnect(NULL);
    }
}

/**
 * 在本地 GATTC 缓存中查找特征，并注册通知。
 *   · 通知特征：0xFFF1（必须带 Notify 属性）；
 *   · 写特征  ：先找 0xFFF2（对端作 GATT Server 时的写特征），找不到再回退 0xFFF1（手表）；
 *   · 使能通知：与官方 gattc 例程一致 —— 先 esp_ble_gattc_register_for_notify()，
 *               再在 ESP_GATTC_REG_FOR_NOTIFY_EVT 里写 CCCD。
 */
static void ble_c_query_chars(void)
{
    /* ① 通知特征（对端 → 音箱） */
    if (s_have_notify_uuid) {
        esp_gattc_char_elem_t ch;
        uint16_t cnt = 1;
        esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_conn_id,
                                                              s_svc_start, s_svc_end,
                                                              s_notify_char_uuid, &ch, &cnt);
        if (st == ESP_GATT_OK && cnt > 0 && (ch.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
            s_notify_char_handle = ch.char_handle;
            ESP_LOGI(TAG, "找到通知特征 0x%04X: handle=0x%04x (props=0x%02x)",
                     NOTIFY_CHAR_UUID, s_notify_char_handle, ch.properties);
        } else {
            ESP_LOGW(TAG, "未找到可通知特征 0x%04X", NOTIFY_CHAR_UUID);
        }
    }

    /* ② 写特征（音箱 → 对端）：依次尝试 0xFFF2（手机）→ 0xFFF1（手表）→ 0xFFF3（手表备用） */
    const uint16_t candidate_uuid[3] = { WRITE_CHAR_UUID_PRIMARY, WRITE_CHAR_UUID_FALLBACK,
                                         WRITE_CHAR_UUID_FALLBACK2 };
    const bool candidate_have[3] = { s_have_write_primary, s_have_write_fallback, s_have_write_fallback2 };
    const esp_bt_uuid_t *candidate[3] = { &s_write_primary_uuid, &s_write_fallback_uuid,
                                          &s_write_fallback2_uuid };

    for (int i = 0; i < 3 && s_write_char_handle == 0; i++) {
        if (!candidate_have[i]) {
            continue;
        }
        esp_gattc_char_elem_t ch;
        uint16_t cnt = 1;
        esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_conn_id,
                                                              s_svc_start, s_svc_end,
                                                              *(candidate[i]), &ch, &cnt);
        if (st == ESP_GATT_OK && cnt > 0 &&
            (ch.properties & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR))) {
            s_write_char_handle = ch.char_handle;
            s_write_char_uuid_used = candidate_uuid[i];
            ESP_LOGI(TAG, "找到写特征 0x%04X: handle=0x%04x (props=0x%02x)",
                     candidate_uuid[i], s_write_char_handle, ch.properties);
        }
    }
    if (s_write_char_handle == 0) {
        ESP_LOGW(TAG, "未找到可写特征（0x%04X / 0x%04X / 0x%04X），本链路只能接收数据",
                 WRITE_CHAR_UUID_PRIMARY, WRITE_CHAR_UUID_FALLBACK, WRITE_CHAR_UUID_FALLBACK2);
    }

    /* ③ 注册通知（真正的 CCCD 使能在 REG_FOR_NOTIFY_EVT 里做） */
    if (s_notify_char_handle != 0) {
        esp_err_t err = esp_ble_gattc_register_for_notify(s_gattc_if, s_target_bda, s_notify_char_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "注册通知失败: 0x%x", err);
            ble_c_fail_and_reconnect(NULL);
        } else {
            s_notify_registered = true;
        }
        return;
    }

    /* 未配置 / 未找到通知特征：仍视为连接成功（仅可写，无通知） */
    ble_c_on_connected();
}

/* ---------------------------------------------------------------------
 * GAP（扫描）回调（由 ble_gap 统一分发）
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
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            break;
        }
        if (ble_c_state_get() != BLE_C_STATE_SCANNING) {
            break;
        }

        char name[33];
        if (!ble_c_scan_match(param, name, sizeof(name))) {
            break;
        }

        /* 命中目标：先暂存信息，停止扫描，等 SCAN_STOP_COMPLETE 事件后再发起连接 */
        memcpy(s_target_bda, param->scan_rst.bda, ESP_BD_ADDR_LEN);
        s_target_addr_type = param->scan_rst.ble_addr_type;
        snprintf(s_target_name, sizeof(s_target_name), "%s", name);

        s_pending_open = true;
        ble_c_set_state(BLE_C_STATE_CONNECTING);
        xTimerStop(s_scan_timer, 0);
        xTimerStart(s_conn_timer, 0);       /* 覆盖"停扫描→连接→服务发现→使能通知"全过程 */

        ESP_LOGI(TAG, "命中目标设备: 名称='%s' 地址=%02x:%02x:%02x:%02x:%02x:%02x "
                      "地址类型=%d RSSI=%d adv=%u rsp=%u evt=%d → 已停止扫描，等待停扫完成后连接",
                 s_target_name,
                 s_target_bda[0], s_target_bda[1], s_target_bda[2],
                 s_target_bda[3], s_target_bda[4], s_target_bda[5],
                 s_target_addr_type, param->scan_rst.rssi,
                 param->scan_rst.adv_data_len, param->scan_rst.scan_rsp_len,
                 param->scan_rst.ble_evt_type);

        esp_ble_gap_stop_scanning();
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "扫描已停止");
        if (s_pending_open) {
            s_pending_open = false;
            ble_c_open_target();            /* 扫描真正停稳后再建链 */
        }
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

    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(TAG, "物理连接建立, conn_id=%d", param->connect.conn_id);
#if BLE_CLIENT_MTU_REQUEST_ENABLE
        /* 与官方 gattc 例程一致：物理连接建立后立即发起 MTU 协商 */
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        if (mtu_ret != ESP_OK) {
            ESP_LOGW(TAG, "发起 MTU 协商失败: 0x%x", mtu_ret);
        }
#endif
        break;
    }

    case ESP_GATTC_OPEN_EVT: {
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATT 打开失败: 0x%x（0x85=ESP_GATT_ERROR；链路未建立请看断开事件的 reason=0x3e）",
                     param->open.status);
            ble_c_fail_and_reconnect(NULL);
            break;
        }
        s_conn_id = param->open.conn_id;
        ESP_LOGI(TAG, "GATT 连接已建立, conn_id=%d, mtu=%d", s_conn_id, param->open.mtu);
        break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG, "MTU 协商完成: %d", param->cfg_mtu.mtu);
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        /* 与官方 gattc 例程一致：注册通知成功后，写该特征的 CCCD(0x2902) 使能通知 */
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "注册通知失败: 0x%x", param->reg_for_notify.status);
            ble_c_fail_and_reconnect(NULL);
            break;
        }
        ESP_LOGI(TAG, "通知注册成功（handle=0x%04x），写 CCCD 使能通知...", param->reg_for_notify.handle);

        esp_bt_uuid_t cccd = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
        };
        esp_gattc_descr_elem_t desc;
        uint16_t cnt = 1;
        esp_gatt_status_t st = esp_ble_gattc_get_descr_by_char_handle(s_gattc_if, s_conn_id,
                                                                      param->reg_for_notify.handle,
                                                                      cccd, &desc, &cnt);
        if (st != ESP_GATT_OK || cnt == 0) {
            ESP_LOGE(TAG, "未找到 CCCD(0x2902): 0x%x", st);
            ble_c_fail_and_reconnect(NULL);
            break;
        }

        s_cccd_handle = desc.handle;
        uint16_t notify_en = 0x0001;    /* 0x0001=Notify, 0x0002=Indicate */
        esp_err_t err = esp_ble_gattc_write_char_descr(s_gattc_if, s_conn_id, s_cccd_handle,
                                                       sizeof(notify_en), (uint8_t *)&notify_en,
                                                       ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "写 CCCD 失败: 0x%x", err);
            ble_c_fail_and_reconnect(NULL);
        }
        /* 成功后在 ESP_GATTC_WRITE_DESCR_EVT 中置 CONNECTED */
        break;
    }

    case ESP_GATTC_DIS_SRVC_CMPL_EVT: {
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "服务发现失败: 0x%x", param->dis_srvc_cmpl.status);
            ble_c_fail_and_reconnect(NULL);
            break;
        }
        ESP_LOGI(TAG, "服务发现完成，搜索目标服务...");
        s_svc_found = false;
        s_svc_start = 0;
        s_svc_end = 0;
        /* 搜索目标服务（与官方 gattc 例程一致：带 UUID 过滤） */
        esp_err_t err = esp_ble_gattc_search_service(s_gattc_if, s_conn_id,
                                                     s_have_svc_uuid ? &s_svc_uuid : NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "搜索服务失败: 0x%x", err);
            ble_c_fail_and_reconnect(NULL);
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
            ble_c_fail_and_reconnect(NULL);
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
            ESP_LOGE(TAG, "未找到目标服务 0x%04X", TARGET_SERVICE_UUID);
            ble_c_fail_and_reconnect(NULL);
        }
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "CCCD 写入失败: 0x%x", param->write.status);
            ble_c_fail_and_reconnect(NULL);
        } else {
            ble_c_on_connected();
        }
        break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "写特征失败: handle=0x%04x status=0x%x", param->write.handle, param->write.status);
        }
        break;

    case ESP_GATTC_READ_CHAR_EVT:
        if (param->read.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "读特征失败: handle=0x%04x status=0x%x", param->read.handle, param->read.status);
        }
        break;

    case ESP_GATTC_NOTIFY_EVT:
        /* 收到对端通知 / 指示数据，转发给上层回调 */
        if (s_rx_cb) {
            s_rx_cb(param->notify.value, param->notify.value_len);
        }
        break;

    case ESP_GATTC_CONGEST_EVT:
        s_congested = param->congest.congested;
        ESP_LOGI(TAG, "链路%s", s_congested ? "拥塞（暂停发送）" : "拥塞解除");
        break;

    case ESP_GATTC_CLOSE_EVT:
        ESP_LOGI(TAG, "GATT 虚拟连接关闭, reason=0x%x", param->close.reason);
        break;

    case ESP_GATTC_DISCONNECT_EVT: {
        if (ble_c_state_get() == BLE_C_STATE_IDLE && s_conn_id == 0) {
            break;              /* 已由 OPEN 失败路径清理过，避免重复上报 */
        }
        ESP_LOGW(TAG, "连接断开, reason=0x%02x（0x3e=链路未建立，0x08=超时，0x13=对端主动断开，0x16=本机断开）",
                 param->disconnect.reason);
        ble_c_reset_link();
        if (s_auto_reconnect) {
            ble_c_schedule_reconnect();
        } else {
            ESP_LOGI(TAG, "自动重连已关闭（手动断开）");
        }
        break;
    }

    default:
        break;
    }
}

/* ---------------------------------------------------------------------
 * 定时器回调
 * --------------------------------------------------------------------- */
static void scan_timer_cb(TimerHandle_t t)
{
    if (ble_c_state_get() != BLE_C_STATE_SCANNING) {
        return;
    }
    ESP_LOGW(TAG, "扫描超时(%d ms)，未找到目标设备", BLE_CLIENT_SCAN_TIMEOUT_MS);
    esp_ble_gap_stop_scanning();
    ble_c_reset_link();
    ble_c_schedule_reconnect();
}

static void conn_timer_cb(TimerHandle_t t)
{
    if (ble_c_state_get() != BLE_C_STATE_CONNECTING) {
        return;
    }
    ESP_LOGW(TAG, "连接/服务发现超时(%d ms)，清理并重连", BLE_CLIENT_CONNECT_TIMEOUT_MS);
    ble_c_fail_and_reconnect(NULL);
}

static void reconnect_timer_cb(TimerHandle_t t)
{
    s_reconnect_scheduled = false;
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

    /* GATT 公共配置：本机 MTU（客户端/服务端共用；不主动发起协商） */
    esp_err_t ret = esp_ble_gatt_set_local_mtu(BLE_CLIENT_LOCAL_MTU);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置本地 MTU 失败: 0x%x", ret);
    }

    /* 创建一次性定时器 */
    s_scan_timer = xTimerCreate("ble_scan", pdMS_TO_TICKS(BLE_CLIENT_SCAN_TIMEOUT_MS),
                                pdFALSE, NULL, scan_timer_cb);
    s_conn_timer = xTimerCreate("ble_conn", pdMS_TO_TICKS(BLE_CLIENT_CONNECT_TIMEOUT_MS),
                                pdFALSE, NULL, conn_timer_cb);
    s_reconnect_timer = xTimerCreate("ble_reconn", pdMS_TO_TICKS(BLE_CLIENT_RECONNECT_DELAY_MIN_MS),
                                     pdFALSE, NULL, reconnect_timer_cb);

    /* 构造 16 位目标 UUID（宏为 0 则跳过对应过滤 / 查询） */
    if (TARGET_SERVICE_UUID != 0) {
        ble_c_uuid16_set(&s_svc_uuid, TARGET_SERVICE_UUID);
        s_have_svc_uuid = true;
    }
    if (WRITE_CHAR_UUID_PRIMARY != 0) {
        ble_c_uuid16_set(&s_write_primary_uuid, WRITE_CHAR_UUID_PRIMARY);
        s_have_write_primary = true;
    }
    if (WRITE_CHAR_UUID_FALLBACK != 0) {
        ble_c_uuid16_set(&s_write_fallback_uuid, WRITE_CHAR_UUID_FALLBACK);
        s_have_write_fallback = true;
    }
    if (WRITE_CHAR_UUID_FALLBACK2 != 0) {
        ble_c_uuid16_set(&s_write_fallback2_uuid, WRITE_CHAR_UUID_FALLBACK2);
        s_have_write_fallback2 = true;
    }
    if (NOTIFY_CHAR_UUID != 0) {
        ble_c_uuid16_set(&s_notify_char_uuid, NOTIFY_CHAR_UUID);
        s_have_notify_uuid = true;
    }

    /* 本机只作主机、不广播，但把 BLE 名称设好，便于对端 App 里辨认 */
    ret = esp_ble_gap_set_device_name(BLE_CLIENT_LOCAL_NAME);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 BLE 本机名称失败: 0x%x", ret);
    }

    /* GAP 事件统一由 ble_gap 分发（协议栈只保存一个 GAP 回调，避免被别的模块顶掉） */
    ret = ble_gap_add_handler(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP 处理函数注册失败: 0x%x", ret);
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
    if (s_congested) {
        return -4;
    }
#if BLE_CLIENT_WRITE_WITH_RSP
    const esp_gatt_write_type_t wtype = ESP_GATT_WRITE_TYPE_RSP;
#else
    const esp_gatt_write_type_t wtype = ESP_GATT_WRITE_TYPE_NO_RSP;
#endif
    esp_err_t err = esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_write_char_handle,
                                             len, data, wtype, ESP_GATT_AUTH_REQ_NONE);
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

const char *ble_client_get_peer_name(void)
{
    return (s_target_name[0] != '\0') ? s_target_name : "-";
}

void ble_client_connect(void)
{
    s_auto_reconnect = true;
    s_fail_count = 0;
    s_reconnect_scheduled = false;
    if (ble_c_state_get() == BLE_C_STATE_IDLE) {
        ble_c_start_scan();
    } else {
        ESP_LOGI(TAG, "已在连接流程中或已连接");
    }
}

void ble_client_disconnect(void)
{
    s_auto_reconnect = false;
    s_reconnect_scheduled = false;
    xTimerStop(s_scan_timer, 0);
    xTimerStop(s_conn_timer, 0);
    xTimerStop(s_reconnect_timer, 0);
    esp_ble_gap_stop_scanning();

    uint16_t cid = s_conn_id;
    if (cid != 0 && s_gattc_if != ESP_GATT_IF_NONE) {
        ESP_LOGI(TAG, "手动断开连接");
        esp_ble_gattc_close(s_gattc_if, cid);   /* 后续 DISCONNECT_EVT 会做清理 */
    } else {
        ble_c_reset_link();
        ESP_LOGI(TAG, "手动断开完成（当前无连接）");
    }
}
