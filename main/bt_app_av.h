#ifndef __BT_APP_AV_H__
#define __BT_APP_AV_H__

#include <stdint.h>
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "uart.h"

#define BT_AV_TAG       "BT_AV"
#define BT_RC_TG_TAG    "RC_TG"
#define BT_RC_CT_TAG    "RC_CT"

/**
 * @brief  A2DP接收器的回调函数
 *
 * @param [in] event  event id
 * @param [in] param  回调参数
 */
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/**
 * @brief  A2DP接收器音频数据流的回调函数
 *
 * @param [out] data 应用程序任务写入的数据流
 * @param [in]  len   数据流长度（以字节为单位）
 */
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);

/**
 * @brief  AVRCP控制器的回调函数
 *
 * @param [in] event  event id
 * @param [in] param  回调参数
 */
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

/**
 * @brief  AVRCP目标回调函数
 *
 * @param [in] event  event id
 * @param [in] param  回调参数
 */
void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

extern uint8_t write_data_sleep_flag;
void bt_i2s_driver_uninstall(void);
extern uint8_t bt_con_flag;

/**
 * @brief 当前是否正在播放（音频流处于 STARTED）
 */
bool bt_a2d_is_playing(void);

/**
 * @brief 当前播放进度百分比（0-100），供串口屏切页后补发进度条数值
 */
uint8_t bt_a2d_get_progress(void);

#endif /* __BT_APP_AV_H__*/
