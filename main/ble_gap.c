/**
 * @file ble_gap.c
 * @brief BLE GAP 事件统一分发器实现（详见 ble_gap.h）
 */
#include "esp_log.h"
#include "ble_gap.h"

static const char *TAG = "ble_gap";

/* 最多支持的 GAP 事件处理者数量（当前只用了 1 个：ble_client） */
#define BLE_GAP_MAX_HANDLERS    4

static ble_gap_handler_t s_handlers[BLE_GAP_MAX_HANDLERS];
static size_t s_handler_count = 0;
static bool s_sys_cb_registered = false;

/* 协议栈唯一回调：把事件按注册顺序转发给所有处理者 */
static void ble_gap_sys_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    for (size_t i = 0; i < s_handler_count; i++) {
        if (s_handlers[i] != NULL) {
            s_handlers[i](event, param);
        }
    }
}

esp_err_t ble_gap_add_handler(ble_gap_handler_t handler)
{
    if (handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 第一次调用时注册系统回调（只能注册一次，之后由本模块转发） */
    if (!s_sys_cb_registered) {
        esp_err_t err = esp_ble_gap_register_callback(ble_gap_sys_cb);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "注册 GAP 回调失败: 0x%x", err);
            return err;
        }
        s_sys_cb_registered = true;
    }

    /* 幂等：同一个函数不重复登记 */
    for (size_t i = 0; i < s_handler_count; i++) {
        if (s_handlers[i] == handler) {
            return ESP_OK;
        }
    }
    if (s_handler_count >= BLE_GAP_MAX_HANDLERS) {
        ESP_LOGE(TAG, "GAP 处理函数已满(%d)", BLE_GAP_MAX_HANDLERS);
        return ESP_ERR_NO_MEM;
    }

    s_handlers[s_handler_count++] = handler;
    ESP_LOGI(TAG, "已登记 GAP 处理函数 #%u", (unsigned)s_handler_count);
    return ESP_OK;
}

esp_err_t ble_gap_reregister(void)
{
    /* esp_ble_gap_register_callback() 是**覆盖式**赋值，重复注册同一个分发函数是安全的；
     * 整机休眠会把 Bluedroid disable 再 enable，这里重新注册一次做保险
     * （处理者列表 s_handlers[] 在 RAM 里，不需要重建）。 */
    esp_err_t err = esp_ble_gap_register_callback(ble_gap_sys_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "重新注册 GAP 回调失败: 0x%x", err);
        return err;
    }
    s_sys_cb_registered = true;
    ESP_LOGI(TAG, "GAP 回调已重新注册（整机休眠唤醒后调用）");
    return ESP_OK;
}
