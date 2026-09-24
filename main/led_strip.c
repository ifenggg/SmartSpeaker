/**
 * @file led_strip.c
 * @brief WS2812 灯带（RMT）驱动 + 开关/灯效状态机
 *
 * 【本次修复的 bug：串口屏"关灯"后再次开关灯无响应】
 *   1) 旧 ledon 直接调 leds_init()：RMT 通道已经存在，同引脚二次创建必然失败，
 *      ESP_ERROR_CHECK 触发断言 → 芯片复位重启（表现为"无响应"）；
 *   2) 旧 leds_reon() 先 update_led() 再 rmt_enable()：向**已关闭**的 RMT 通道
 *      发数据 → ESP_ERROR_CHECK 断言复位；
 *   3) 旧 led_mode 任务用嵌套 while(1) 死循环跑彩虹，关灯（rmt_disable）之后
 *      仍在每 50ms 调 update_led() → 复位。
 *   现在：开关状态由 s_leds_on / s_leds_ready 统一守卫，所有底层调用先判状态，
 *        且 ledon/ledoff 只走 leds_on()/leds_off()（调度级），不再碰初始化函数。
 */
#include "esp_check.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>       /* sinf：呼吸灯的正弦亮度包络 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"
#include <stdint.h>

static const char *LTAG = "led";

uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

/* ============================ 灯带状态 ============================ */
static bool     s_leds_ready = false;               /* RMT 资源已创建 */
static bool     s_leds_on    = false;               /* 逻辑开关：上电默认关，由串口屏开启 */
static uint8_t  s_led_mode   = LED_MODE_STATIC;     /* 当前灯效 */
/* 保存的颜色：上电默认 50,50,50（每次上电第一次开灯用这个）；
 * 用户通过三色条改动、或关灯状态下发来 RGB，都会更新并保存（仅断电丢失） */
static uint32_t s_rgb[3]     = {LED_DEFAULT_R, LED_DEFAULT_G, LED_DEFAULT_B};

/* 亮度系数（千分比）：睡眠联动用。只作用于"发出去的像素"，
 * led_strip_pixels 始终保存原始值，所以保存的 RGB / 屏幕三色条不受影响 */
static uint16_t s_dim_permille = LED_DIM_FULL_PERMILLE;
static uint8_t  led_strip_tx[EXAMPLE_LED_NUMBERS * 3];

rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static TaskHandle_t s_led_mode_task = NULL;

static void led_mode(void *arg);
static void led_strip_show(void);

/* =====================================================================
 * 三色滚动条：串口屏发来的 "R<0-255>" / "G<0-255>" / "B<0-255>"
 * ===================================================================== */
void led_block(void)
{
    if (!s_leds_ready || ui_res[0] == '\0') {
        return;
    }

    char comp = ui_res[0];
    int value = atoi(ui_res + 1);

    uint8_t idx;
    if (comp == 'R')      idx = 0;
    else if (comp == 'G') idx = 1;
    else if (comp == 'B') idx = 2;
    else                  return;

    leds_set_color(idx, (uint32_t)value);
    /* 注意：颜色调节可能被串口屏连续拖动触发，用 LOGD 避免默认日志级别下刷屏 */
    ESP_LOGD(LTAG, "颜色改为 R%lu G%lu B%lu", s_rgb[0], s_rgb[1], s_rgb[2]);
}

/* 设置单条通道（h0.val/h1.val/h2.val 与 R/G/B 前缀命令共用）：
 * 拖动颜色条即视为"用户要静态颜色"（退出灯效，否则会被灯效下一帧覆盖）；
 * 未开灯时只保存不点亮（保存值在下次开灯时使用）。 */
void leds_set_color(uint8_t idx, uint32_t value)
{
    if (!s_leds_ready || idx > 2) {
        return;
    }
    if (value > 255) {
        value = 255;
    }
    s_led_mode = LED_MODE_STATIC;
    s_rgb[idx] = value;

    if (s_leds_on) {
        update_led(s_rgb[0], s_rgb[1], s_rgb[2]);
    }
}

/*RMT远程收发控制器*/

//RMT 编码器
typedef struct {
    rmt_encoder_t base;     //接口
    rmt_encoder_t *bytes_encoder;   //颜色数据
    rmt_encoder_t *copy_encoder;    //处理重置代码，用于识别边界
    int state;      //状态机变量（发数据or发重置代码）
    rmt_symbol_word_t reset_code;   //重置代码（告知 LED 灯带 当前数据帧结束）
} rmt_led_strip_encoder_t;

//RMT 编码器编码函数（把LED颜色数据编码为RMT硬件可传输的符号序列）
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    switch (led_encoder->state) {
    case 0: // 发送RGB数据
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: //发送重置代码
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);//表示当前数据帧结束
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; //重置完成，返回
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; //若缓存满，跳转到out
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

//删除 LED 灯带编码器
static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

//重置 LED 灯带编码器
static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

//创建新的 LED 灯带编码器
esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, LTAG, "invalid argument");
    led_encoder = rmt_alloc_encoder_mem(sizeof(rmt_led_strip_encoder_t));
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, LTAG, "no mem for led strip encoder");
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    //配置led灯带（WS2812）通信时序
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,    //电平
            .duration0 = 0.3 * config->resolution / 1000000, // 电平时长T0H=0.3us
            .level1 = 0,
            .duration1 = 0.9 * config->resolution / 1000000, // 电平时长T0L=0.9us
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 0.9 * config->resolution / 1000000, // T1H=0.9us
            .level1 = 0,
            .duration1 = 0.3 * config->resolution / 1000000, // T1L=0.3us
        },
        .flags.msb_first = 1 // WS2812 传输位顺序: G7...G0R7...R0B7...B0
        //指定数据按 *最高位优先* 传输，符合 WS2812 的通信协议（颜色分量按 G→R→B 顺序，每个分量从最高位开始传
    };
    //创建字节编码器
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, LTAG, "create bytes encoder failed");
    //创建复制编码器
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, LTAG, "create copy encoder failed");

    uint32_t reset_ticks = config->resolution / 1000000 * 50 / 2; // 计算重置代码的时长，默认 50us
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
//若错误，清理资源
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}

//转换HSV颜色模型到RGB颜色模型
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    //RGB 分量的最大值，由亮度 v 决定（v × 2.55 是将 [0, 100] 映射到 [0, 255]）
    uint32_t rgb_max = v * 2.55f;   
    //RGB 分量的最小值，由饱和度 s 决定（饱和度越高，rgb_min 越接近 0，颜色越鲜艳）
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    //色调分区
    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    //RGB 分量的 “调整量”，用于在区间内平滑过渡颜色（根据偏移角度 diff 计算）
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    //红黄
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    //黄绿
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    //绿青
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    //青蓝
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    //蓝紫
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    //紫红
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

/* =====================================================================
 * 底层：真正往灯带发一帧（不做开关判定，调用者必须已经确认通道已使能）
 * ===================================================================== */
static void led_strip_show(void)
{
    if (!s_leds_ready || led_chan == NULL || led_encoder == NULL) {
        return;
    }

    /* 亮度系数：把"原始像素"缩放后发送（led_strip_pixels 保持不变，
     * 因此保存的 RGB 与屏幕三色条不受睡眠减弱影响） */
    if (s_dim_permille != LED_DIM_FULL_PERMILLE) {
        for (size_t i = 0; i < sizeof(led_strip_pixels); i++) {
            led_strip_tx[i] = (uint8_t)(((uint32_t)led_strip_pixels[i] * s_dim_permille)
                                        / LED_DIM_FULL_PERMILLE);
        }
    } else {
        memcpy(led_strip_tx, led_strip_pixels, sizeof(led_strip_tx));
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // 非循环传输
    };

    /* 这里刻意不用 ESP_ERROR_CHECK：灯带异常不该把整机拖去重启。
     * 注意：本函数会被彩虹灯效以 50ms 周期调用，禁止打印日志刷屏。 */
    esp_err_t err = rmt_transmit(led_chan, led_encoder, led_strip_tx,
                                 sizeof(led_strip_tx), &tx_config);
    if (err != ESP_OK) {
        return;
    }
    rmt_tx_wait_all_done(led_chan, portMAX_DELAY);  //等待传输完成
}

/* 亮度系数（千分比）：变化时立刻重发一帧，让"静态颜色"也能跟着变暗/变亮 */
void leds_set_dim_permille(uint16_t permille)
{
    if (permille > LED_DIM_FULL_PERMILLE) {
        permille = LED_DIM_FULL_PERMILLE;
    }
    if (permille == s_dim_permille) {
        return;
    }
    s_dim_permille = permille;

    if (s_leds_ready && s_leds_on) {
        led_strip_show();
    }
}

uint16_t leds_get_dim_permille(void)
{
    return s_dim_permille;
}

/* =====================================================================
 * 初始化 / 开 / 关 / 反初始化
 * ===================================================================== */
void leds_init(void)
{
    /* 幂等保护：初始化只允许执行一次。
     * 重复调用会二次创建同引脚 RMT 通道 → 失败 → 断言复位（旧 ledon 的 bug）。 */
    if (s_leds_ready) {
        ESP_LOGW(LTAG, "灯带已初始化，忽略重复的初始化调用");
        return;
    }

    //创建 RMT 发送通道
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    ESP_LOGI(LTAG, "已创建 RMT 发送通道");

    //安装 LED 灯带编码器
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_LOGI(LTAG, "已安装 LED 灯带编码器");

    //使能 RMT 发送通道
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    ESP_LOGI(LTAG, "已使能 RMT 发送通道");

    s_leds_ready = true;
    s_leds_on = false;              /* 上电默认不点亮：等串口屏"灯带开关" */
    s_led_mode = LED_MODE_STATIC;
    /* 保存的颜色保持 LED_DEFAULT_R/G/B（50,50,50），首次开灯直接用它 */

    /* 上电先发一帧全 0，清掉灯带上电随机状态，保证"上电是灭的" */
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    led_strip_show();

    if (s_led_mode_task == NULL) {
        xTaskCreate(led_mode, "led_mode", 1024 * 2, NULL, tskIDLE_PRIORITY + 1, &s_led_mode_task);
        //          入口函数     函数名称      栈深      参数          优先级             句柄
    }

    ESP_LOGI(LTAG, "灯带初始化完成（默认关闭，保存颜色 R%lu G%lu B%lu）",
             s_rgb[0], s_rgb[1], s_rgb[2]);
}

/* 开（调度级）：串口屏"灯带开关"打开走这里 */
void leds_on(void)
{
    if (!s_leds_ready) {
        ESP_LOGW(LTAG, "灯带尚未初始化，无法开启");
        return;
    }
    if (!s_leds_on) {
        /* 顺序必须是"先使能通道，再发数据"：旧 leds_reon() 顺序颠倒，向已关闭
         * 的通道发送触发断言复位，这就是"关灯后再开灯无响应"的直接原因。 */
        esp_err_t err = rmt_enable(led_chan);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(LTAG, "使能 RMT 通道失败: %s", esp_err_to_name(err));
            return;
        }
        s_leds_on = true;
    }

    if (s_led_mode == LED_MODE_STATIC) {
        update_led(s_rgb[0], s_rgb[1], s_rgb[2]);   /* 恢复上次颜色 */
    }
    ESP_LOGI(LTAG, "灯带已开启（灯效=%u, R%lu G%lu B%lu）",
             s_led_mode, s_rgb[0], s_rgb[1], s_rgb[2]);
}

/* 关（调度级）：串口屏"灯带开关"关闭走这里 */
void leds_off(void)
{
    if (!s_leds_ready) {
        return;
    }
    if (!s_leds_on) {
        ESP_LOGI(LTAG, "灯带本来就是关闭状态，忽略");
        return;
    }

    update_led(0, 0, 0);        /* 关闭通道前先熄灭（此时通道还是使能的） */
    s_leds_on = false;

    esp_err_t err = rmt_disable(led_chan);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(LTAG, "关闭 RMT 通道失败: %s", esp_err_to_name(err));
    }
    ESP_LOGI(LTAG, "灯带已关闭");
}

/* 兼容旧命令名（ledreon） */
void leds_reon(void)
{
    leds_on();
}

/* 反初始化：真正释放 RMT 资源。仅调试阶段或整机休眠使用，不挂"关灯"命令 */
void leds_deinit(void)
{
    if (!s_leds_ready) {
        return;
    }

    /* 【必须无条件 disable】RMT 通道在 leds_init() 里创建后就已经 enable 了。
     * 旧实现只在"当时是亮着"时才 rmt_disable()，而整机休眠时灯带通常是熄灯状态，
     * 于是 rmt_del_channel() 以 "channel not in init state" 失败——通道和 GPIO 预留
     * 都不会释放，下次 leds_init() 就报 "GPIO 33 is not usable, maybe conflict with
     * others"，并且每轮休眠泄漏一个 RMT 通道（几次之后 rmt_new_tx_channel 的
     * ESP_ERROR_CHECK 就会把整机复位）。 */
    if (s_leds_on) {
        update_led(0, 0, 0);
        s_leds_on = false;
    }
    if (led_chan) {
        esp_err_t err = rmt_disable(led_chan);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(LTAG, "关闭 RMT 通道失败: %s", esp_err_to_name(err));
        }
    }

    if (led_encoder) {
        rmt_del_encoder(led_encoder);
        led_encoder = NULL;
    }
    if (led_chan) {
        esp_err_t err = rmt_del_channel(led_chan);
        if (err != ESP_OK) {
            /* 极少数情况（通道状态没收干净）会失败：这里**不能**把 s_leds_ready 置假，
             * 否则下次 leds_init() 会在旧通道还在的情况下再建一个（泄漏 → 最终断言复位）。
             * 保留 ready 标记并保留通道句柄，下一轮休眠会再试一次释放。 */
            ESP_LOGE(LTAG, "释放 RMT 通道失败: %s（本轮保留资源，下轮再试）", esp_err_to_name(err));
            return;
        }
        led_chan = NULL;
    }
    s_leds_ready = false;
    ESP_LOGI(LTAG, "灯带 RMT 资源已释放（反初始化完成）");
}

/* 直接刷新颜色：内部带"已初始化 + 已开灯"双重守卫，绝不会操作已关闭的通道 */
void update_led(uint32_t red, uint32_t green, uint32_t blue)
{
    if (!s_leds_ready || !s_leds_on) {
        return;
    }
    if (red > 255)   red = 255;
    if (green > 255) green = 255;
    if (blue > 255)  blue = 255;

    for (int i = 0; i < EXAMPLE_LED_NUMBERS; i++)
    {
        //将 RGB 数据存入数组（顺序：绿→蓝→红，匹配灯带协议）
        led_strip_pixels[i * 3 + 0] = (uint8_t)green;
        led_strip_pixels[i * 3 + 1] = (uint8_t)blue;
        led_strip_pixels[i * 3 + 2] = (uint8_t)red;
    }
    led_strip_show();
}

/* =====================================================================
 * 灯效
 * ===================================================================== */
/* 灯效基准时刻：切换灯效时复位，让呼吸灯从"最暗"开始，而不是从任意相位切入 */
static volatile uint32_t s_effect_base_ms = 0;

void leds_set_mode(uint8_t mode)
{
    if (s_led_mode != mode) {
        s_effect_base_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    }
    s_led_mode = mode;
}

/* 退出灯效：恢复保存的静态颜色（灯带保持亮） */
void leds_set_static(void)
{
    leds_set_mode(LED_MODE_STATIC);
    if (s_leds_on) {
        update_led(s_rgb[0], s_rgb[1], s_rgb[2]);
    }
}

void leds_mo1(void)
{
    leds_set_mode(LED_MODE_RAINBOW);
    ESP_LOGI(LTAG, "灯效1：流动彩虹（亮度 100%）");
}

void leds_mo2(void)
{
    leds_set_mode(LED_MODE_BREATH);
    ESP_LOGI(LTAG, "灯效2：柔和呼吸灯（R→G→B 轮流，最大亮度 %d‰）", LED_BREATH_MAX * 100 / 255);
}

/**
 * 灯效任务：单层循环 + 状态判断。
 * 旧实现用嵌套 while(1) 跑彩虹且无法退出，关灯后仍向已关闭的 RMT 通道发数据，
 * 会断言复位（这也是"关灯后再操作无响应"的根因之一）。
 */
static void led_mode(void *arg)
{
    uint32_t red = 0, green = 0, blue = 0;
    uint16_t start_rgb = 0;
    const int FLOW_SPEED_MS = 50;   // 彩虹流动速度
    const int HUE_STEP = 5;         // 色调步长（数值越小过渡越平滑）

    while (1) {
        /* 关灯 / 未初始化：什么都不做（既不刷新也不报错） */
        if (!s_leds_ready || !s_leds_on) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (s_led_mode == LED_MODE_RAINBOW) {
            for (int j = 0; j < EXAMPLE_LED_NUMBERS; j++) {
                // 计算当前灯珠的色调（hue）；加入全局偏移量 start_rgb 实现流动效果
                uint16_t hue = (j * 360 / EXAMPLE_LED_NUMBERS + start_rgb) % 360;
                // HSV 转 RGB（饱和度、亮度均为100，确保颜色鲜艳）
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                // 将 RGB 数据存入数组（顺序：绿→蓝→红，匹配灯带协议）
                led_strip_pixels[j * 3 + 0] = (uint8_t)green;
                led_strip_pixels[j * 3 + 1] = (uint8_t)blue;
                led_strip_pixels[j * 3 + 2] = (uint8_t)red;
            }
            led_strip_show();

            start_rgb = (start_rgb + HUE_STEP) % 360;   // 偏移色调（让彩虹整体向前流动）
            vTaskDelay(pdMS_TO_TICKS(FLOW_SPEED_MS));
        }
        else if (s_led_mode == LED_MODE_BREATH) {
            /* 柔和呼吸灯：R→G→B 依次，每个颜色做一次"渐亮 + 渐暗"。
             * 亮度 = 正弦包络（0 → LED_BREATH_MAX → 0），峰值刻意低于彩虹的 100%，
             * 避免夜间刺眼；正弦包络比三角波更柔和、无折点。 */
            uint32_t now = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
            uint32_t el   = now - s_effect_base_ms;                 /* 灯效已运行毫秒数 */
            uint32_t step = el / LED_BREATH_PERIOD_MS;              /* 第几次呼吸 */
            uint32_t phase = el % LED_BREATH_PERIOD_MS;             /* 本次呼吸内相位 */

            /* sin(0)=0 → 最暗，sin(π/2)=1 → 最亮，sin(π)=0 → 最暗 */
            float env = sinf(3.14159265f * (float)phase / (float)LED_BREATH_PERIOD_MS);
            uint32_t level = (uint32_t)((float)LED_BREATH_MAX * env + 0.5f);

            red = green = blue = 0;
            switch (step % 3) {                                      /* 三色轮流 */
            case 0:  red = level;   break;
            case 1:  green = level; break;
            default: blue = level;  break;
            }

            for (int j = 0; j < EXAMPLE_LED_NUMBERS; j++) {
                led_strip_pixels[j * 3 + 0] = (uint8_t)green;
                led_strip_pixels[j * 3 + 1] = (uint8_t)blue;
                led_strip_pixels[j * 3 + 2] = (uint8_t)red;
            }
            led_strip_show();
            vTaskDelay(pdMS_TO_TICKS(LED_BREATH_STEP_MS));
        }
        else {
            /* 静态颜色：无需本任务刷新（改颜色时由 leds_set_color() 直接刷新） */
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* =====================================================================
 * 状态查询（供串口屏切页后刷新按钮/滚动条状态）
 * ===================================================================== */
bool leds_is_on(void)
{
    return s_leds_on;
}

uint8_t leds_get_mode(void)
{
    return s_led_mode;
}

void leds_get_rgb(uint32_t *r, uint32_t *g, uint32_t *b)
{
    if (r) *r = s_rgb[0];
    if (g) *g = s_rgb[1];
    if (b) *b = s_rgb[2];
}
