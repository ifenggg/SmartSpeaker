/**
 * @file amp.c
 * @brief PAM8403 D 类功放驱动实现
 *
 * 设计要点：
 *   - MUTE/SHDN 均为 GPIO 输出，逻辑严格对应：MUTE 低=静音、SHDN 高=开启；
 *   - 上电与重新上电均做「先静音 → SHDN 拉高 → 延时 → 恢复静音态」防爆音序列；
 *   - 低电量用强制关断标记（最高优先级），期间屏蔽其他逻辑的开机请求；
 *   - 延时关断用一次性 FreeRTOS 定时器，开机即取消待定的关断。
 */
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "amp.h"

static const char *TAG = "amp";

static bool s_muted = false;        /* 当前静音状态 */
static bool s_powered = false;      /* 当前电源状态 */
static bool s_low_batt_off = false; /* 低电量强制关断标记（最高优先级） */
static TimerHandle_t s_off_timer = NULL;  /* 延时关断定时器 */

/* 延时关断定时器回调 */
static void amp_off_timer_cb(TimerHandle_t t)
{
    amp_set_power(false);
}

void amp_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << AMP_MUTE_GPIO) | (1ULL << AMP_SHDN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    /* 上电防爆音：先静音上电，稳定后再取消静音 */
    gpio_set_level(AMP_SHDN_GPIO, 1);   /* SHDN 拉高：开启 */
    gpio_set_level(AMP_MUTE_GPIO, 0);   /* MUTE 拉低：先静音 */
    s_powered = true;
    s_muted = true;
    vTaskDelay(pdMS_TO_TICKS(AMP_POWER_ON_DELAY_MS));  /* 软启动延时 */
    gpio_set_level(AMP_MUTE_GPIO, 1);   /* MUTE 拉高：取消静音 */
    s_muted = false;

    s_off_timer = xTimerCreate("amp_off", pdMS_TO_TICKS(AMP_PAUSE_OFF_DELAY_MS),
                               pdFALSE, NULL, amp_off_timer_cb);

    ESP_LOGI(TAG, "PAM8403 初始化完成：SHDN=1(开启) MUTE=1(非静音)");
}

void amp_set_mute(bool mute)
{
    if (mute == s_muted) {
        return;
    }
    s_muted = mute;
    gpio_set_level(AMP_MUTE_GPIO, mute ? 0 : 1);   /* 拉低=静音，拉高=取消静音 */
    ESP_LOGI(TAG, "静音: %s", mute ? "开" : "关");
}

void amp_set_power(bool on)
{
    /* 低电量锁定期间禁止开启 */
    if (on && s_low_batt_off) {
        ESP_LOGW(TAG, "低电量锁定，忽略开机请求");
        return;
    }
    if (on == s_powered) {
        return;
    }
    s_powered = on;

    if (on) {
        /* 上电防爆音：临时静音 → SHDN 拉高 → 延时 → 恢复目标静音态 */
        gpio_set_level(AMP_MUTE_GPIO, 0);
        gpio_set_level(AMP_SHDN_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(AMP_POWER_ON_DELAY_MS));
        gpio_set_level(AMP_MUTE_GPIO, s_muted ? 0 : 1);
    } else {
        gpio_set_level(AMP_SHDN_GPIO, 0);   /* SHDN 拉低：关断 */
    }

    if (s_off_timer) {
        xTimerStop(s_off_timer, 0);   /* 取消待定的延时关断 */
    }
    ESP_LOGI(TAG, "功放电源: %s", on ? "开" : "关");
}

void amp_power_off_delayed(uint32_t delay_ms)
{
    if (delay_ms == 0) {
        amp_set_power(false);
        return;
    }
    if (s_off_timer) {
        xTimerChangePeriod(s_off_timer, pdMS_TO_TICKS(delay_ms), 0);
        xTimerStart(s_off_timer, 0);
    }
}

void amp_set_low_batt_off(bool off)
{
    s_low_batt_off = off;
    amp_set_power(!off);
}

bool amp_is_muted(void)
{
    return s_muted;
}

bool amp_is_powered(void)
{
    return s_powered;
}
