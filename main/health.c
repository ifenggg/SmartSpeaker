/**
 * @file health.c
 * @brief 手表健康数据：帧重组与解析、NVS 掉电保存、health 页刷新、睡眠标志联动
 *
 * 协议与行为约定见 health.h 顶部说明。这里记录几个实现要点：
 *   1. BLE 回调（蓝牙协议栈上下文）里**只做轻活**：按 \r\n 重组整帧、拆 key=value、
 *      更新 RAM、置"有新数据/待落盘/睡眠标志变化"标志；
 *      耗时的 NVS 落盘、串口推送、AVRCP/功放/灯带动作用 1 秒节拍任务完成。
 *   2. NVS 只保存上屏用的 4 个值（HR/SPO2/STEP/ST）+ 一个"曾经收到过帧"的标志；
 *      SLEEP 标志按约定不持久化，T/H/P 只留 RAM。写入节流 ≥60 秒（帧间隔 1 分钟）。
 *   3. 睡眠减弱用"独立系数"（audio_vol / led_strip 各一个千分比系数），
 *      不改用户设置的音量值与保存的 RGB —— 屏幕上的音量条/三色条仍是用户值。
 */
#include "health.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "uart.h"           /* uart_send_text */
#include "ui.h"             /* ui_get_page */
#include "ble_client.h"     /* ble_client_get_state / ble_client_send / ble_client_set_rx_cb */
#include "bt_app_core.h"    /* bt_sleep（AVRCP 暂停 + 关功放） */
#include "amp.h"            /* amp_set_power */
#include "led_strip.h"      /* leds_on / leds_off / leds_set_dim_permille */
#include "audio_vol.h"      /* audio_vol_set_dim_permille */

static const char *TAG = "HEALTH";

/* ============================ 数据帧常量 ============================ */
#define HEALTH_FRAME_HEAD   "$DATA,"        /* 传感器数据帧头 */
#define HEALTH_FRAME_REQ    "$GET,DATA\r\n" /* 请求一次传感器数据 */
#define HEALTH_FRAME_END    "$END\r\n"      /* 停止传感器上报 */

#define HEALTH_LINE_MAX     192             /* 单帧最大长度 */

/* ============================ 数值存储 ============================ */
/* 上屏值：0 表示无效（协议约定 0 为无效值） */
static struct {
    bool has_data;      /* 是否至少收到过一帧传感器数据（决定"什么都不发"还是发 "-"） */
    int  hr;            /* 心率 30~200 次/分 */
    int  spo2;          /* 血氧 1~100 % */
    int  step;          /* 步数 1~65535 步 */
    int  st_deci;       /* 睡眠时长，单位 0.1 小时（保留 1 位小数） */
} s_store;

/* 不上屏的字段：仅 RAM（协议要求"解析并保存但不需要写进 NVS"） */
static float s_temp;        /* 温度 ℃ */
static int   s_humi;        /* 湿度 %RH */
static float s_press;       /* 气压 hPa */

/* ============================ 内部状态 ============================ */
static char   s_line[HEALTH_LINE_MAX];      /* 行重组缓冲 */
static size_t s_line_len;

static bool   s_new_frame;          /* 有新帧待刷新屏幕 */
static bool   s_nvs_dirty;          /* 有变更待落盘 */
static uint32_t s_nvs_last_ms;      /* 上次落盘时刻（0=还没写过，允许立即写） */
static bool   s_stop_pending;       /* $END 没发出去（链路不在），连上后补发 */

/* 睡眠标志状态机 */
static uint8_t  s_sleep_flag = HEALTH_SLEEP_AWAKE;   /* 最近一次收到的标志 */
static bool     s_sleep_pending_valid = false;
static uint8_t  s_sleep_pending;
static bool     s_sleep_locked = false;             /* SLEEP=2 锁定中 */

/* 减弱系数斜坡 */
typedef enum {
    DIM_IDLE = 0,
    DIM_DOWN,       /* 向 10% 走（1 秒一个台阶，共 60 步） */
    DIM_UP,         /* 向 100% 走（1 秒一个台阶，共 3 步） */
} dim_mode_t;

static dim_mode_t s_dim_mode = DIM_IDLE;
static uint16_t   s_dim = 1000;         /* 当前系数（千分比） */
static uint16_t   s_dim_applied = 1000; /* 已经下发给 audio/led 的值 */

/* 链路状态轮询 */
static uint8_t s_ble_state_prev = 0xFF;

#define DIM_STEP_DOWN   ((1000 - HEALTH_DIM_FLOOR_PERMILLE) / (HEALTH_DIM_DOWN_MS / HEALTH_TICK_MS))
#define DIM_STEP_UP     ((1000 - HEALTH_DIM_FLOOR_PERMILLE) / (HEALTH_DIM_UP_MS  / HEALTH_TICK_MS))

/* ============================ 工具 ============================ */
static void health_task(void *arg);

/** 把一个浮点睡眠时长（小时）转成 0.1 小时整数，超范围视为无效 */
static int st_to_deci(float hours)
{
    if (hours <= 0.0f || hours > 24.0f) {
        return 0;
    }
    int deci = (int)(hours * 10.0f + 0.5f);
    return (deci > 0 && deci <= 240) ? deci : 0;
}

/** 数值范围判定（协议：0 一律视为无效） */
static int clamp_range(int v, int lo, int hi)
{
    if (v < lo || v > hi) {
        return 0;       /* 超范围按无效处理 */
    }
    return v;
}

/* ============================ NVS ============================ */
#define HEALTH_NVS_NS   "health"

static void health_nvs_load(void)
{
    nvs_handle_t h;

    if (nvs_open(HEALTH_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "NVS 中还没有健康数据（首次上电）");
        return;
    }

    uint8_t has = 0;
    int32_t v = 0;

    if (nvs_get_u8(h, "has", &has) == ESP_OK && has) {
        if (nvs_get_i32(h, "hr",   &v) == ESP_OK) s_store.hr = v;
        if (nvs_get_i32(h, "spo2", &v) == ESP_OK) s_store.spo2 = v;
        if (nvs_get_i32(h, "step", &v) == ESP_OK) s_store.step = v;
        if (nvs_get_i32(h, "st",   &v) == ESP_OK) s_store.st_deci = v;
        s_store.has_data = true;
        ESP_LOGI(TAG, "NVS 载入: 心率=%d 血氧=%d 步数=%d 睡眠=%.1f 小时",
                 s_store.hr, s_store.spo2, s_store.step, s_store.st_deci / 10.0f);
    } else {
        ESP_LOGI(TAG, "NVS 标记为空（尚未收到过数据）");
    }
    nvs_close(h);
}

static void health_nvs_flush_if_due(void)
{
    if (!s_nvs_dirty) {
        return;
    }

    uint32_t now = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    if (s_nvs_last_ms != 0 && (now - s_nvs_last_ms) < HEALTH_NVS_MIN_INTERVAL_MS) {
        return;     /* 节流：两次落盘间隔不小于 HEALTH_NVS_MIN_INTERVAL_MS */
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(HEALTH_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS 打开失败: %s", esp_err_to_name(err));
        return;
    }

    bool ok = true;
    ok = ok && (nvs_set_u8 (h, "has",  s_store.has_data ? 1 : 0) == ESP_OK);
    ok = ok && (nvs_set_i32(h, "hr",   s_store.hr)      == ESP_OK);
    ok = ok && (nvs_set_i32(h, "spo2", s_store.spo2)    == ESP_OK);
    ok = ok && (nvs_set_i32(h, "step", s_store.step)    == ESP_OK);
    ok = ok && (nvs_set_i32(h, "st",   s_store.st_deci) == ESP_OK);
    if (ok) {
        ok = (nvs_commit(h) == ESP_OK);
    }
    nvs_close(h);

    if (ok) {
        s_nvs_dirty = false;
        s_nvs_last_ms = now;
        ESP_LOGI(TAG, "NVS 已保存: 心率=%d 血氧=%d 步数=%d 睡眠=%.1f 小时",
                 s_store.hr, s_store.spo2, s_store.step, s_store.st_deci / 10.0f);
    } else {
        ESP_LOGW(TAG, "NVS 写入失败，下次节拍重试");
    }
}

/* ============================ 帧解析 ============================ */
/** 按 key=value 应用一个字段（大小写不敏感；未知 key 忽略） */
static void health_apply_kv(const char *key, const char *val, bool *sensor_seen)
{
    if (strcasecmp(key, "T") == 0) {
        float f = strtof(val, NULL);
        /* 协议：0 为无效值；有效范围 -10~50 ℃ */
        s_temp = (f != 0.0f && f >= -10.0f && f <= 50.0f) ? f : 0.0f;
        *sensor_seen = true;
    } else if (strcasecmp(key, "H") == 0) {
        s_humi = clamp_range(atoi(val), 1, 100);
        *sensor_seen = true;
    } else if (strcasecmp(key, "HR") == 0) {
        int v = clamp_range(atoi(val), 30, 200);
        if (v != s_store.hr) { s_store.hr = v; s_nvs_dirty = true; }
        *sensor_seen = true;
    } else if (strcasecmp(key, "SPO2") == 0) {
        int v = clamp_range(atoi(val), 1, 100);
        if (v != s_store.spo2) { s_store.spo2 = v; s_nvs_dirty = true; }
        *sensor_seen = true;
    } else if (strcasecmp(key, "STEP") == 0) {
        int v = clamp_range(atoi(val), 1, 65535);
        if (v != s_store.step) { s_store.step = v; s_nvs_dirty = true; }
        *sensor_seen = true;
    } else if (strcasecmp(key, "P") == 0) {
        float f = strtof(val, NULL);
        /* 协议：0 为无效值；有效范围 300~1100 hPa */
        s_press = (f != 0.0f && f >= 300.0f && f <= 1100.0f) ? f : 0.0f;
        *sensor_seen = true;
    } else if (strcasecmp(key, "SLEEP") == 0) {
        int v = atoi(val);
        if (v == HEALTH_SLEEP_AWAKE || v == HEALTH_SLEEP_DROWSY || v == HEALTH_SLEEP_ASLEEP) {
            s_sleep_pending = (uint8_t)v;
            s_sleep_pending_valid = true;
        }
    } else if (strcasecmp(key, "ST") == 0) {
        int deci = st_to_deci(strtof(val, NULL));
        if (deci != s_store.st_deci) { s_store.st_deci = deci; s_nvs_dirty = true; }
        *sensor_seen = true;
    }
}

/** 处理一整行（找到 $DATA, 或独立睡眠帧） */
static void health_handle_line(char *line)
{
    bool sensor_seen = false;
    const char *p;

    /* 允许带前导空白/噪声 */
    p = strchr(line, '$');
    if (p == NULL) {
        return;                     /* 不是本模块的帧，忽略 */
    }
    p++;

    if (strncmp(p, "DATA,", 5) == 0) {
        p += 5;                     /* 跳过帧头 */
    } else if (strncasecmp(p, "SLEEP", 5) == 0) {
        /* 独立睡眠帧：$SLEEP,1 / $SLEEP=1 */
        const char *v = p + 5;
        while (*v == ',' || *v == '=' || *v == ' ') v++;
        int val = atoi(v);
        if (val == HEALTH_SLEEP_AWAKE || val == HEALTH_SLEEP_DROWSY || val == HEALTH_SLEEP_ASLEEP) {
            s_sleep_pending = (uint8_t)val;
            s_sleep_pending_valid = true;
            ESP_LOGI(TAG, "收到独立睡眠帧: SLEEP=%d", val);
        } else {
            ESP_LOGW(TAG, "独立睡眠帧格式无法识别: \"%s\"", line);
        }
        return;
    } else {
        ESP_LOGW(TAG, "收到未识别的帧（原样打印，便于对齐协议）: \"%s\"", line);
        return;
    }

    /* 把内容按逗号切开，每段再按 '=' 切成 key/value */
    char *save1 = NULL;
    for (char *tok = strtok_r((char *)p, ",", &save1); tok != NULL;
         tok = strtok_r(NULL, ",", &save1)) {
        char *eq = strchr(tok, '=');
        if (eq == NULL) {
            continue;               /* 没有等号：跳过（容忍格式噪声） */
        }
        *eq = '\0';
        const char *key = tok;
        const char *val = eq + 1;
        while (*key == ' ') key++;
        while (*val == ' ') val++;
        health_apply_kv(key, val, &sensor_seen);
    }

    if (sensor_seen) {
        s_store.has_data = true;
        s_new_frame = true;
    }

    ESP_LOGI(TAG, "数据帧: T=%.1f℃ H=%d%% HR=%d SPO2=%d STEP=%d P=%.1fhPa SLEEP=%d ST=%.1fh",
             s_temp, s_humi, s_store.hr, s_store.spo2, s_store.step, s_press,
             s_sleep_pending_valid ? (int)s_sleep_pending : (int)s_sleep_flag,
             s_store.st_deci / 10.0f);
}

/** BLE 通知回调：按 \r\n 重组整帧 */
void health_on_ble_frame(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '\r') {
            continue;               /* 帧尾的 \r 丢弃，用 \n 收尾 */
        }
        if (c == '\n') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                health_handle_line(s_line);
                s_line_len = 0;
            }
            continue;
        }
        if (s_line_len < sizeof(s_line) - 1) {
            s_line[s_line_len++] = c;
        } else {
            s_line_len = 0;         /* 超长：丢弃重来，避免半帧误判 */
            ESP_LOGW(TAG, "帧过长，已丢弃");
        }
    }
}

/* ============================ 上屏 ============================ */
/** 无效值占位符 */
#define HEALTH_PLACEHOLDER  "-"

void health_ui_push(void)
{
    char v_st[16], v_spo2[16], v_hr[16], v_step[16];

    if (ui_get_page() != UI_PAGE_HEALTH) {
        return;                     /* 只在 health 页推送（控件是页面私有的） */
    }
    if (!s_store.has_data) {
        ESP_LOGD(TAG, "尚未收到过任何数据帧 → 四个控件都不发送");
        return;
    }

    s_new_frame = false;            /* 本次推送已覆盖最新数据 */

    /* t0 ← 睡眠时长（0.1 小时 → "7.5"） */
    if (s_store.st_deci > 0) {
        snprintf(v_st, sizeof(v_st), "%d.%d", s_store.st_deci / 10, s_store.st_deci % 10);
    } else {
        snprintf(v_st, sizeof(v_st), HEALTH_PLACEHOLDER);
    }
    uart_send_text("t0.txt", v_st);

    /* t1 ← 血氧 */
    if (s_store.spo2 > 0) {
        snprintf(v_spo2, sizeof(v_spo2), "%d", s_store.spo2);
    } else {
        snprintf(v_spo2, sizeof(v_spo2), HEALTH_PLACEHOLDER);
    }
    uart_send_text("t1.txt", v_spo2);

    /* t2 ← 心率 */
    if (s_store.hr > 0) {
        snprintf(v_hr, sizeof(v_hr), "%d", s_store.hr);
    } else {
        snprintf(v_hr, sizeof(v_hr), HEALTH_PLACEHOLDER);
    }
    uart_send_text("t2.txt", v_hr);

    /* t3 ← 步数 */
    if (s_store.step > 0) {
        snprintf(v_step, sizeof(v_step), "%d", s_store.step);
    } else {
        snprintf(v_step, sizeof(v_step), HEALTH_PLACEHOLDER);
    }
    uart_send_text("t3.txt", v_step);

    ESP_LOGI(TAG, "health 页已刷新: t0(睡眠)=%s t1(血氧)=%s t2(心率)=%s t3(步数)=%s",
             v_st, v_spo2, v_hr, v_step);
}

/* ============================ 请求 / 停止 ============================ */
void health_request_from_watch(void)
{
    if (ble_client_get_state() != BLE_CLIENT_STATE_CONNECTED) {
        ESP_LOGW(TAG, "手表未连接，跳过数据请求（进页只显示已存数据）");
        return;
    }

    int r = ble_client_send((uint8_t *)HEALTH_FRAME_REQ, (uint16_t)strlen(HEALTH_FRAME_REQ));
    ESP_LOGI(TAG, "向手表请求传感器数据: \"$GET,DATA\" → ret=%d", r);

    if (r == 0) {
        s_stop_pending = false;     /* 新一轮请求开始，之前欠的停止帧作废 */
    }
}

void health_stop_stream(void)
{
    if (ble_client_get_state() != BLE_CLIENT_STATE_CONNECTED) {
        s_stop_pending = true;      /* 链路不在：等连上补发 */
        ESP_LOGW(TAG, "手表未连接，$END 待链路恢复后补发");
        return;
    }

    int r = ble_client_send((uint8_t *)HEALTH_FRAME_END, (uint16_t)strlen(HEALTH_FRAME_END));
    ESP_LOGI(TAG, "通知手表停止传感器上报: \"$END\" → ret=%d", r);

    s_stop_pending = (r != 0);
}

/* ============================ 减弱系数斜坡 ============================ */
static void health_dim_apply(void)
{
    if (s_dim == s_dim_applied) {
        return;
    }
    s_dim_applied = s_dim;
    audio_vol_set_dim_permille(s_dim);      /* 音量减弱（不改变用户设定值） */
    leds_set_dim_permille(s_dim);           /* 灯带亮度减弱（不改变保存的 RGB） */
    ESP_LOGD(TAG, "减弱系数 → %u‰", (unsigned)s_dim);
}

static void health_ramp_tick(void)
{
    if (s_dim_mode == DIM_DOWN) {
        if (s_dim > HEALTH_DIM_FLOOR_PERMILLE) {
            s_dim = (s_dim >= HEALTH_DIM_FLOOR_PERMILLE + DIM_STEP_DOWN)
                    ? (uint16_t)(s_dim - DIM_STEP_DOWN) : HEALTH_DIM_FLOOR_PERMILLE;
        } else {
            s_dim_mode = DIM_IDLE;          /* 已到 10%：保持 */
        }
        health_dim_apply();
    } else if (s_dim_mode == DIM_UP) {
        if (s_dim < 1000) {
            s_dim = (uint16_t)((s_dim + DIM_STEP_UP > 1000) ? 1000 : (s_dim + DIM_STEP_UP));
        } else {
            s_dim_mode = DIM_IDLE;          /* 已回到 100% */
        }
        health_dim_apply();
    }
}

/* ============================ 睡眠标志 ============================ */
static void health_apply_sleep_pending(void)
{
    if (!s_sleep_pending_valid) {
        return;
    }
    s_sleep_pending_valid = false;
    s_sleep_flag = s_sleep_pending;

    /* SLEEP=2：暂停 + 关功放 + 关灯带 + 锁定（已锁定则保持，不重复动作） */
    if (s_sleep_flag == HEALTH_SLEEP_ASLEEP) {
        if (s_sleep_locked) {
            ESP_LOGI(TAG, "SLEEP=2（已处于锁定状态，保持）");
            return;
        }
        s_sleep_locked = true;
        s_dim_mode = DIM_IDLE;
        s_dim = 1000;                       /* 系数归位：解锁后即为用户设定 */
        health_dim_apply();
        bt_sleep();                         /* AVRCP 暂停 + 关功放 */
        leds_off();                         /* 关灯带 */
        ESP_LOGW(TAG, "手表报告睡眠(2) → 暂停音乐、关功放与灯带；"
                      "锁定至 A2DP 重连或用户按播放");
        return;
    }

    /* 锁定期间：0/1 不自动恢复（但用户手动操作始终有效） */
    if (s_sleep_locked) {
        ESP_LOGI(TAG, "SLEEP=%u 但当前处于睡眠锁定，忽略自动恢复", s_sleep_flag);
        return;
    }

    if (s_sleep_flag == HEALTH_SLEEP_DROWSY) {
        s_dim_mode = DIM_DOWN;              /* 收到 1 不重启斜坡：继续降 / 保持 */
        ESP_LOGI(TAG, "手表报告准备入睡(1) → 音量与灯带在 %d 秒内线性降到 %d‰",
                 HEALTH_DIM_DOWN_MS / 1000, HEALTH_DIM_FLOOR_PERMILLE);
    } else {
        if (s_dim < 1000) {
            s_dim_mode = DIM_UP;            /* 3 秒内线性升回 100% */
            ESP_LOGI(TAG, "手表报告清醒(0) → 音量与灯带在 %d 秒内线性回到 100%%",
                     HEALTH_DIM_UP_MS / 1000);
        } else {
            s_dim_mode = DIM_IDLE;
        }
    }
}

static void health_unlock(const char *why)
{
    if (!s_sleep_locked) {
        return;
    }
    s_sleep_locked = false;
    s_dim_mode = DIM_IDLE;
    s_dim = 1000;
    health_dim_apply();
    leds_on();                              /* 恢复灯带（保存的 RGB / 原灯效） */
    amp_set_power(true);                    /* 恢复功放（音乐由 btpla / A2DP 侧负责） */
    ESP_LOGW(TAG, "睡眠锁定已解除（%s）→ 音量与灯带恢复正常", why);
}

void health_on_a2dp_connected(void)
{
    health_unlock("A2DP 重连");
}

void health_on_user_play(void)
{
    health_unlock("用户按播放");
}

/* ============================ 链路轮询 ============================ */
static void health_link_tick(void)
{
    uint8_t st = ble_client_get_state();

    if (st == BLE_CLIENT_STATE_CONNECTED && s_ble_state_prev != BLE_CLIENT_STATE_CONNECTED) {
        ESP_LOGI(TAG, "手表链路已连接（当前页面: %s）", ui_page_name(ui_get_page()));
        if (ui_get_page() == UI_PAGE_HEALTH) {
            health_request_from_watch();    /* 停在 health 页：重新请求 */
        } else if (s_stop_pending) {
            health_stop_stream();           /* 补发欠下的停止帧 */
        }
    } else if (st != BLE_CLIENT_STATE_CONNECTED && s_ble_state_prev == BLE_CLIENT_STATE_CONNECTED) {
        ESP_LOGW(TAG, "手表链路已断开");
    }
    s_ble_state_prev = st;
}

/* ============================ 节拍任务 ============================ */
static void health_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HEALTH_TICK_MS));

        health_link_tick();             /* 链路状态（补发 $END / 重新请求） */
        health_apply_sleep_pending();   /* 睡眠标志动作（暂停/关断/斜坡） */
        health_ramp_tick();             /* 减弱系数步进 */
        health_nvs_flush_if_due();      /* NVS 落盘（节流） */

        if (s_new_frame && ui_get_page() == UI_PAGE_HEALTH) {
            s_new_frame = false;        /* 停在 health 页：收到新帧立即刷新 */
            health_ui_push();
        }
    }
}

/* ============================ 初始化 ============================ */
void health_init(void)
{
    health_nvs_load();

    /* 注册 BLE 接收回调（手表数据从这里进来） */
    ble_client_set_rx_cb(health_on_ble_frame);

    s_dim = 1000;
    s_dim_applied = 1000;
    s_ble_state_prev = ble_client_get_state();

    xTaskCreate(health_task, "health_task", 1024 * 4, NULL, 5, NULL);
    //          入口函数   函数名称       栈深      参数  优先级 句柄

    ESP_LOGI(TAG, "健康数据模块就绪（NVS %s，减弱系数 1000‰）",
             s_store.has_data ? "已载入" : "为空");
}

/* ============================ 查询接口 ============================ */
uint8_t health_get_sleep_flag(void)
{
    return s_sleep_flag;
}

uint16_t health_get_dim_permille(void)
{
    return s_dim;
}
