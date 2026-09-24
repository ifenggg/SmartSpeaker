#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_gap_ble_api.h"
#include "uart.h"

void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/* =====================================================================
 * 蓝牙开/关函数分工（串口屏"蓝牙总开关"必须按这个来调）
 * ---------------------------------------------------------------------
 *   bt_init()         初始化：上电调用一次（NVS + 控制器双模开启 + Bluedroid 使能）
 *   bt_a2dp_work()    开：调度级 —— 注册 A2DP/AVRCP、恢复任务、进入可发现（可反复调用）
 *   bt_app_shutdown() 关：调度级 —— 反初始化 A2DP/AVRCP、释放 I2S（可反复调用）
 *
 * 即：初始化已在上电执行，所以开/关不能调用 bt_init()，只能调用这一对调度函数。
 * ===================================================================== */
void bt_app_start(void);
void bt_app_shutdown(void);
void bt_a2dp_work(void);
void bt_init(void);

/**
 * @brief 蓝牙功能层是否已开启（串口屏蓝牙开关 n0 的状态回读用）
 * @return true=已开启 false=已关闭
 */
bool bt_a2dp_is_on(void);

/* =====================================================================
 * 整机休眠：双模蓝牙整体关/开
 * ---------------------------------------------------------------------
 * bt_stack_sleep() 把蓝牙关到"可以安全进 light sleep"的状态：
 *   A2DP/AVRCP 反初始化 + 释放 I2S + BLE 链路收尾 + esp_bluedroid_disable()
 *   + esp_bt_controller_disable()（用 disable，不用 deinit —— 见 bt_a2dp.c 说明）
 * bt_stack_wake() 反顺序恢复：控制器 enable → Bluedroid enable → GAP 回调重注册
 *   → bt_a2dp_work()（仅当休眠前总开关是开的）→ BLE 主机链路恢复扫描。
 * ===================================================================== */

/**
 * @brief 关闭双模蓝牙（整机休眠前调用；调用前应先 bt_sleep() 让对端暂停）
 * @return ESP_OK=已关闭（可进 light sleep）；其他=关闭失败，**不要**进 light sleep
 */
esp_err_t bt_stack_sleep(void);

/**
 * @brief 恢复双模蓝牙（整机唤醒、收到有效指令后调用）
 * @param a2dp_on true=恢复 A2DP/AVRCP 功能层（休眠前总开关是开的时传 true）
 * @return ESP_OK=成功；其他=失败（BLE/串口屏不受影响，可再触发一次）
 */
esp_err_t bt_stack_wake(bool a2dp_on);
