#include "esp_check.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"
#include <stdint.h>

extern uint8_t ble_rec_buf[20];     //ble服务端接收到的数据包（颜色控制）
QueueHandle_t led_queue = NULL;     //灯带队列句柄
uint8_t leds_flag;                  //灯带初始化标志位
extern char ui_res[10];

static uint8_t ledmode;
static void led_mode(void *arg);

//led任务函数
static void led_task(void* arg)
{
    uint32_t io_num;
    for (;;) {          //永久循环
        if (xQueueReceive(led_queue, &io_num, portMAX_DELAY)) {        //portMAX_DELAY：永久阻塞，直到队列中有数据（不会超时）
            //update_led(ble_rec_buf[0],ble_rec_buf[1],ble_rec_buf[2]);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void led_block(void)
{
    static uint32_t r=0,g=0,b=0;
    vTaskDelay(pdMS_TO_TICKS(50));
    if(leds_flag)
    {
        if(ui_res[0]=='R')
            r = atoi(ui_res+1);
        else if(ui_res[0]=='G')
            g = atoi(ui_res+1);
        else if(ui_res[0]=='B')
            b = atoi(ui_res+1);
        update_led(r,g,b);
        //注意：颜色调节可能被串口屏连续拖动触发，用 LOGD 避免默认日志级别下刷屏
        ESP_LOGD("leds","led颜色改为%lu,%lu,%lu",r,g,b);
    }
        
}

/*RMT远程收发控制器*/

const char *LTAG = "led";

uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

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

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
void leds_init(void)
{
    // //颜色分量
    // uint32_t red = 0;
    // uint32_t green = 0;
    // uint32_t blue = 0;
    // //色调
    // uint16_t hue = 0;
    // uint16_t start_rgb = 0;

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

    led_queue = xQueueCreate(10, sizeof(uint32_t));    //创建gpio队列
    //xTaskCreate(led_task, "led_task", 2048, NULL, 10, NULL);      //创建IO任务
    //              入口函数        函数名称（终端可看）   栈深  参数 优先级 句柄
    leds_flag = 1;

    xTaskCreate(led_mode, "led_mode", 1024 * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    //          入口函数     函数名称      栈深      参数          优先级             句柄

    update_led(50,50,50);
    uart_send("h0.val=50");
    uart_send("h1.val=50");
    uart_send("h2.val=50");

    //彩虹灯追逐效果配置
    // ESP_LOGI(LTAG, "Start LED rainbow chase");
    // rmt_transmit_config_t tx_config = {
    //     .loop_count = 0, // 非循环传输
    // };
    // while (1) {
    //     for (int i = 0; i < 3; i++) {
    //         for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3) {
    //             // 计算当前灯珠的色调（hue）
    //             hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
    //             //HSV 转 RGB（饱和度、亮度均为 100，确保颜色鲜艳）
    //             led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
    //             //将 RGB 数据存入数组（顺序：绿→蓝→红，匹配灯带协议）
    //             led_strip_pixels[j * 3 + 0] = green;
    //             led_strip_pixels[j * 3 + 1] = blue;
    //             led_strip_pixels[j * 3 + 2] = red;
    //         }
    //         //发送 RGB 数据到 LED 灯带
    //         ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    //         ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY)); //等待传输完成
    //         vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    //         //清空像素数组（熄灭 LED）
    //         memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    //         // 再次发送空数据，让 LED 熄灭
    //         ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    //         ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    //         vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    //     }
    //     // 偏移色调（让下一轮彩虹“移动”60°）
    //     start_rgb += 60;
    // }
}

uint32_t leds_last[3];
void update_led(uint32_t red,uint32_t green,uint32_t blue)
{
    for (int i = 0; i < EXAMPLE_LED_NUMBERS; i++) 
    {
        //记录当前的颜色值
        leds_last[0] = green;
        leds_last[1] = blue;
        leds_last[2] = red;
        //将 RGB 数据存入数组（顺序：绿→蓝→红，匹配灯带协议）
        led_strip_pixels[i * 3 + 0] = green;
        led_strip_pixels[i * 3 + 1] = blue;
        led_strip_pixels[i * 3 + 2] = red;
    }
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // 非循环传输
    };
    //发送 RGB 数据到 LED 灯带
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY)); //等待传输完成
    //注意：本函数被彩虹流动效果以 50ms 周期调用，禁止在此打印日志，否则会刷屏并拖慢音频任务
}

void leds_mo1(void)
{
    ESP_LOGI("leds","流动彩虹");
    ledmode = 1;
}

void leds_mo2(void)
{
    ESP_LOGI("leds","模式2");
    ledmode = 2;
}

static void led_mode(void *arg)
{
    //颜色分量
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    //色调
    uint16_t hue = 0;
    uint16_t start_rgb = 0;
    const int FLOW_SPEED_MS = 50;
    // 色调变化的步长（数值越小，过渡越平滑）
    const int HUE_STEP = 5;

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if(ledmode==1)
        {
            while (1) {
                // 为每个LED设置颜色，形成连续的彩虹渐变
                for (int j = 0; j < EXAMPLE_LED_NUMBERS; j++) {
                    // 计算当前灯珠的色调（hue）
                    // 加入全局偏移量start_rgb实现流动效果
                    // 每个LED之间有固定的色调差，形成连续彩虹
                    hue = (j * 360 / EXAMPLE_LED_NUMBERS + start_rgb) % 360;
                    
                    // HSV 转 RGB（饱和度、亮度均为100，确保颜色鲜艳）
                    led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                    
                    // 将 RGB 数据存入数组（顺序：绿→蓝→红，匹配灯带协议）
                    led_strip_pixels[j * 3 + 0] = green;
                    led_strip_pixels[j * 3 + 1] = blue;
                    led_strip_pixels[j * 3 + 2] = red;
                }
                
                // 发送 RGB 数据到 LED 灯带
                update_led(green,blue,red);
                
                // 等待一段时间，控制流动速度
                vTaskDelay(pdMS_TO_TICKS(FLOW_SPEED_MS));
                
                // 偏移色调（让彩虹整体向前流动）
                start_rgb = (start_rgb + HUE_STEP) % 360;
            }
        }
        if(ledmode==2)
        {

        }
    }

    
}


void leds_deinit(void)
{
    update_led(0,0,0);
    ESP_LOGI("leds","已关灯");
    leds_flag=0;
    ESP_ERROR_CHECK(rmt_disable(led_chan));
    ESP_LOGI(LTAG, "已关闭 RMT 发送通道");
}
void leds_reon(void)
{
    uart_send("h0.val=%lu",leds_last[0]);
    uart_send("h1.val=%lu",leds_last[1]);
    uart_send("h2.val=%lu",leds_last[2]);
    update_led(leds_last[2],leds_last[0],leds_last[1]);
    leds_flag=1;
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    ESP_LOGI("leds","已重启led配置");
}
