/**
 * @file ble_debug.c
 * @brief BLE 双向通信调试模块实现
 *
 * 调试流程：
 *   - 接收：把 Notify 收到的字符串帧打印到终端；
 *   - 发送：每 5 秒主动发送一条调试帧 "$PING,<递增序号>\r\n"，
 *           手表端收到后应原样回传（$PING,<同序号>），以验证双向通路与丢包。
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ble_client.h"
#include "ble_debug.h"

static const char *TAG = "ble_debug";

/* 调试帧发送周期 */
#define DEBUG_SEND_PERIOD_MS   5000   /* 每 5 秒一条 */

/* 接收回调：把收到的帧打印到终端 */
static void debug_rx_cb(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return;
    }
    char buf[64];
    int n = (len < (int)sizeof(buf) - 1) ? (int)len : (int)sizeof(buf) - 1;
    memcpy(buf, data, n);
    buf[n] = '\0';

    /* 去掉帧尾 \r\n，便于终端打印 */
    if (n >= 2 && buf[n - 2] == '\r' && buf[n - 1] == '\n') {
        buf[n - 2] = '\0';
        n -= 2;
    } else if (n >= 1 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) {
        buf[n - 1] = '\0';
        n -= 1;
    }

    ESP_LOGI(TAG, "RX[%d]: %s", n, buf);
}

/* 调试发送任务：周期性发送调试帧 */
static void debug_send_task(void *arg)
{
    uint32_t seq = 0;
    char frame[32];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DEBUG_SEND_PERIOD_MS));

        int len = snprintf(frame, sizeof(frame), "$%s,%lu\r\n",
                           FRAME_CMD_PING, (unsigned long)(++seq));
        int ret = ble_client_send((uint8_t *)frame, (uint16_t)len);
        ESP_LOGI(TAG, "TX[%d]: %s => ret=%d", len, frame, ret);
    }
}

void ble_debug_start(void)
{
    ble_client_set_rx_cb(debug_rx_cb);

    xTaskCreate(debug_send_task, "ble_dbg_tx", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "调试流程已启动：接收帧打印到终端，每 %d ms 发送调试帧 $PING,<序号>",
             (int)DEBUG_SEND_PERIOD_MS);
}
