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
