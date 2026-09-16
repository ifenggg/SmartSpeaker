/**
 * @file ble_debug.c
 * @brief 【调试阶段代码，已停用】BLE 双向通信调试模块
 *
 * ⚠ 本文件当前**不参与编译**（main/CMakeLists.txt 里已注释），源码保留备查。
 *
 * 停用原因：这是排查"手表连不上/收不到数据"期间临时加的调试脚手架 ——
 *   连上设备后每 5 秒主动发一帧 `$PING,<序号>`，并把收到的帧按对端名称打印到串口。
 *   问题解决、进入正常功能开发后不再需要，故停用（正式业务应直接使用
 *   main/ble_client.c 暴露的两个接口：`ble_client_send()` 与 `ble_client_set_rx_cb()`）。
 *
 * 需要重新启用时：
 *   1) 把 main/CMakeLists.txt 里 "ble_debug.c" 的注释去掉；
 *   2) main/main.c 里 `#include "ble_debug.h"` 与 `ble_debug_start();` 的注释去掉；
 *   3) 把下面的 `#if 0` 改成 `#if 1`。
 */
#if 0   /* ================== 调试阶段代码，已停用 ================== */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ble_client.h"
#include "ble_debug.h"

static const char *TAG = "ble_debug";

/* 调试帧发送周期 */
#define DEBUG_SEND_PERIOD_MS     5000   /* 每 5 秒一条 */
/* 链路刚建立后的首帧延迟：避免对端刚建连就被灌数据 */
#define DEBUG_FIRST_TX_DELAY_MS  300

/**
 * 把原始帧转成可见文本：\r\n → "\\r\\n"，不可打印字节 → "\\xNN"
 * （直接打印原始数据会把日志撑断行，也不便于一眼确认帧尾是否正确）
 */
static void frame_to_text(const uint8_t *data, uint16_t len, char *out, size_t out_sz)
{
    size_t n = 0;
    if (out_sz == 0) {
        return;
    }
    for (uint16_t i = 0; i < len && (n + 5) < out_sz; i++) {
        uint8_t c = data[i];
        if (c == '\r') {
            out[n++] = '\\';
            out[n++] = 'r';
        } else if (c == '\n') {
            out[n++] = '\\';
            out[n++] = 'n';
        } else if (c >= 0x20 && c <= 0x7e) {
            out[n++] = (char)c;
        } else {
            int w = snprintf(&out[n], out_sz - n, "\\x%02X", c);
            if (w <= 0) {
                break;
            }
            n += (size_t)w;
        }
    }
    out[n] = '\0';
}

/* 接收回调：对端（手表或手机）通过通知/指示发来的帧 */
static void debug_rx_cb(uint8_t *data, uint16_t len)
{
    char txt[96];
    frame_to_text(data, len, txt, sizeof(txt));
    ESP_LOGI(TAG, "RX[%s][%u]: %s", ble_client_get_peer_name(), (unsigned)len, txt);
}

/* 调试发送任务：连上设备后才周期发送调试帧 */
static void debug_send_task(void *arg)
{
    uint32_t seq = 0;
    char frame[32];
    char txt[48];
    bool ready_prev = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DEBUG_SEND_PERIOD_MS));

        bool ready = (ble_client_get_state() == BLE_CLIENT_STATE_CONNECTED);

        /* 链路状态变化：打印明确的就绪 / 断开日志 */
        if (ready != ready_prev) {
            if (ready) {
                ESP_LOGI(TAG, "链路就绪（对端='%s'）→ 开始每 %d ms 收发调试帧",
                         ble_client_get_peer_name(), (int)DEBUG_SEND_PERIOD_MS);
                vTaskDelay(pdMS_TO_TICKS(DEBUG_FIRST_TX_DELAY_MS));
            } else {
                ESP_LOGW(TAG, "链路断开 → 暂停收发调试帧");
            }
        }
        ready_prev = ready;

        /* 没连上就不发数据（接收本来就只可能发生在连上之后） */
        if (!ready) {
            continue;
        }

        int len = snprintf(frame, sizeof(frame), "$%s,%lu\r\n",
                           FRAME_CMD_PING, (unsigned long)(++seq));
        int ret = ble_client_send((uint8_t *)frame, (uint16_t)len);

        frame_to_text((const uint8_t *)frame, (uint16_t)len, txt, sizeof(txt));
        ESP_LOGI(TAG, "TX[%d] %s => %s:%d", len, txt, ble_client_get_peer_name(), ret);
    }
}

void ble_debug_start(void)
{
    ble_client_set_rx_cb(debug_rx_cb);

    xTaskCreate(debug_send_task, "ble_dbg_tx", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "调试流程已启动：连上手表或手机后，每 %d ms 发一帧 $PING,<序号>（收到帧按对端名称打印）",
             (int)DEBUG_SEND_PERIOD_MS);
}

#endif  /* ================== 调试阶段代码结束 ================== */
