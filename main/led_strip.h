/**
 * @file led_strip.h
 * @brief WS2812 灯带驱动（RMT）+ 开关/灯效状态机
 *
 * 开/关函数分工（重要，串口屏"灯带开关"必须按这个来调）：
 *   leds_init()    初始化：上电调用一次，只创建 RMT 资源并清空灯带，**不点亮**
 *   leds_on()      开：调度级 —— 使能 RMT 通道 + 恢复上次颜色（可反复调用）
 *   leds_off()     关：调度级 —— 先熄灭再关闭 RMT 通道（可反复调用）
 *   leds_deinit()  反初始化：真正释放 RMT 通道/编码器（仅调试或整机休眠用，
 *                  **不要**把它挂到"关灯"命令上，否则再开灯只能重新初始化）
 *
 * 即：初始化已在上电执行，所以串口屏的开/关只能调用 leds_on()/leds_off()，
 *     绝不能调用 leds_init()（第二次创建同引脚 RMT 通道会失败并触发断言重启，
 *     这正是"关灯后再开关灯无响应"的根因之一）。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_encoder.h"
#include <uart.h>

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz分辨率，1个时钟周期=0.1微秒 （LED灯带需要高分辨率）
#define RMT_LED_STRIP_GPIO_NUM      33

#define EXAMPLE_LED_NUMBERS         30
#define EXAMPLE_CHASE_SPEED_MS      100

/* 灯效编号（0 = 静态颜色；与串口屏"灯效选择"按钮对应，命令串见 ui.c 命令表） */
#define LED_MODE_STATIC             0
#define LED_MODE_RAINBOW            1   /* 灯效1：流动彩虹（命令 led1），亮度 100% */
#define LED_MODE_BREATH             2   /* 灯效2：柔和呼吸灯（命令 led2），R→G→B 轮流，亮度刻意弱于彩虹 */

/* 呼吸灯参数（可调） */
#define LED_BREATH_MAX              102     /* 最大亮度（≈40%），刻意比彩虹的 100% 弱 */
#define LED_BREATH_PERIOD_MS        2000    /* 单色一次完整呼吸用时（渐亮+渐暗） */
#define LED_BREATH_STEP_MS          33      /* 刷新周期（约 30fps，保证柔和无跳变） */

/* 上电首次开灯的默认颜色（保存值：用户通过三色条改动后即被替换，直到下次上电） */
#define LED_DEFAULT_R               50
#define LED_DEFAULT_G               50
#define LED_DEFAULT_B               50

/* 亮度系数满量程（千分比）：1000 = 不减弱。睡眠模式（手表 SLEEP=1）用 */
#define LED_DIM_FULL_PERMILLE       1000

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

void leds_init(void);                                   /* 上电初始化（不点亮，默认颜色 50,50,50 已保存） */
void leds_deinit(void);                                 /* 反初始化（释放 RMT，仅调试/休眠用） */
void leds_on(void);                                     /* 开（调度级）：恢复保存的 RGB 并点亮 */
void leds_off(void);                                    /* 关（调度级）：只熄灭，保存的 RGB 保留 */
void leds_reon(void);                                   /* 兼容旧命令名：等价 leds_on() */
void update_led(uint32_t red, uint32_t green, uint32_t blue);   /* 直接刷新颜色（内部有开关/使能保护） */
void leds_mo1(void);                                    /* 灯效1：流动彩虹（仅设置灯效，不做开关判断） */
void leds_mo2(void);                                    /* 灯效2：柔和呼吸灯（仅设置灯效，不做开关判断） */
void led_block(void);                                   /* 三色滚动条：从 ui_res 解析 "R/G/B<0-255>" */

/* ---- 串口屏 h0/h1/h2 属性回传形式（"h0.val=50"）用的单通道设置 ---- */
/**
 * @brief 设置单条颜色通道并保存（未开灯时只保存不点亮）
 * @param idx 0=R 1=G 2=B
 * @param value 0-255
 */
void leds_set_color(uint8_t idx, uint32_t value);

/**
 * @brief 选择灯效（只改模式；关灯状态下由调用方决定是否忽略）
 */
void leds_set_mode(uint8_t mode);

/**
 * @brief 退出灯效并恢复保存的静态 RGB（灯带保持亮）
 */
void leds_set_static(void);

bool leds_is_on(void);                                  /* 灯带开关状态（供串口屏刷新开关控件 n0） */
uint8_t leds_get_mode(void);                            /* 当前灯效编号 */
void leds_get_rgb(uint32_t *r, uint32_t *g, uint32_t *b);   /* 当前保存的颜色（供串口屏刷新三色条 h0/h1/h2） */

/**
 * @brief 设置亮度系数（千分比，1000 = 不减弱）
 * @param permille 0~1000；睡眠联动用（SLEEP=1 时逐级降到 100 = 10%）
 * @note  系数在"发送灯珠数据前"乘到像素上，**不改变保存的 RGB**，
 *        所以串口屏三色条显示的仍是用户设定的颜色；灯效也一并变暗。
 */
void leds_set_dim_permille(uint16_t permille);

/**
 * @brief 读取当前亮度系数（千分比）
 */
uint16_t leds_get_dim_permille(void);

#ifdef __cplusplus
}
#endif
