/**
 * @file ble_gap.h
 * @brief BLE GAP 事件统一分发器
 *
 * 背景（重要）：
 *   ESP-IDF 的 Bluedroid 只保存 **一个** BLE GAP 回调
 *   （esp_ble_gap_register_callback() → btc_profile_cb_set()，是覆盖式赋值），
 *   任何模块再次注册都会把前一个顶掉，造成"某模块收不到扫描/广播事件"的隐蔽故障。
 *
 * 用法：
 *   本工程中需要 GAP 事件的模块统一用 ble_gap_add_handler() 注册自己的处理函数
 *   （当前：ble_client = 主机扫描/连接），由本模块注册唯一的系统回调并转发。
 *   handler 会收到**所有** GAP 事件，各模块只处理自己关心的，其余忽略即可。
 */
#pragma once

#include "esp_err.h"
#include "esp_gap_ble_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GAP 事件处理函数原型（与 esp_gap_ble_cb_t 一致） */
typedef void (*ble_gap_handler_t)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/**
 * @brief 注册一个 GAP 事件处理函数（幂等：同一函数重复注册只保留一份）
 * @note  必须在蓝牙控制器 + Bluedroid 使能之后调用（即 bt_init() 之后）
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG 参数为空；ESP_ERR_NO_MEM 处理函数已满
 */
esp_err_t ble_gap_add_handler(ble_gap_handler_t handler);

#ifdef __cplusplus
}
#endif
