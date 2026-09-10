#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"
#include <uart.h>

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz分辨率，1个时钟周期=0.1微秒 （LED灯带需要高分辨率）
#define RMT_LED_STRIP_GPIO_NUM      13

#define EXAMPLE_LED_NUMBERS         60
#define EXAMPLE_CHASE_SPEED_MS      100

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED灯带编码器配置类型
 */
typedef struct {
    uint32_t resolution; /*!< 编码器分辨率，以Hz为单位 */
} led_strip_encoder_config_t;

/**
 * @brief 创建用于将LED灯条像素编码为RMT符号的RMT编码器
 *
 * @param[in] config 编码器配置
 * @param[out] ret_encoder 编码器配置
 * @return
 *      -对于任何无效参数，返回ESP_ERR_INVALID_ARG
 *      -创建LED灯条编码器时内存不足，导致ESP_ERR_NO_MEM错误
 *      -如果创建编码器成功，则返回ESP_OK
 */
esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

void leds_deinit(void);
void leds_init(void);
void update_led(uint32_t red,uint32_t green,uint32_t blue);
void leds_mo1(void);
void leds_mo2(void);
void led_block(void);
void leds_reon(void);

#ifdef __cplusplus
}
#endif
