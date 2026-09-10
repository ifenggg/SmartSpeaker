#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <bt_a2dp.h>
#include <ble.h>
#include "ble_client.h"
#include "led_strip.h"
#include "uart.h"
#include "sd.h"
#include <adc.h>

#define GPIO_INPUT_PIN   (1ULL << GPIO_NUM_0)
#define LONG_PRESS_TIME 2000  // 长按阈值：1s

/* ***********
    Main变量声明
    ***********
*/
extern char ui_res[10];
extern uint8_t s_volume;    //音箱音量
extern uint8_t sendflag;

/* *************
    类状态机处理UI
    ************
*/
//定义命令处理函数的函数指针类型
typedef void (*CommandHandler)(void);
//定义命令与处理函数的映射关系
typedef struct {
    const char* command;       // 命令字符串
    CommandHandler handler;    // 对应的处理函数
    size_t cmd_len;            // 命令长度（提前计算，优化效率）
} CommandMap;
//初始化指令映射表
static const CommandMap commandMap[] = {
    {"bt",     bt_page,         strlen("bt")},
    {"tb",     bt_page,         strlen("tb")},
    {"bton",   bt_a2dp_work,    strlen("bton")},
    {"btoff",  bt_app_shutdown, strlen("btoff")},
    {"btpau",  bt_sleep,        strlen("btpau")},
    {"btpla",  bt_work,         strlen("btpla")},
    {"btne",   bt_ne,           strlen("btne")},
    {"btla",   bt_la,           strlen("btla")},
    {"btsil",  bt_sli,          strlen("btsil")},
    {"btnoi",  bt_noi,          strlen("btnoi")},
    {"sdon",   sd_init,         strlen("sdon")},
    {"sdoff",  sd_deinit,       strlen("sdoff")},
    {"sdpau",  sd_pau,          strlen("sdpau")},
    {"sdpla",  sd_pla,          strlen("sdpla")},
    {"sdne",   sd_ne,           strlen("sdne")},
    {"sdla",   sd_la,           strlen("sdla")},
    {"ledon",  leds_init,       strlen("ledon")},
    {"ledoff", leds_deinit,     strlen("ledoff")},
    {"ledreon",leds_reon,       strlen("ledreon")},
    {"led1",   leds_mo1,        strlen("led1")},
    {"led2",   leds_mo2 ,       strlen("led2")},
};
//命令处理的主函数
void process_command(const char* ui_res) {
    if (!ui_res) return;

    size_t res_len = strlen(ui_res);
    int map_size = sizeof(commandMap) / sizeof(commandMap[0]);

    if(ui_res[0]=='R'||ui_res[0]=='G'||ui_res[0]=='B')
    {
        led_block();
    }
    else if(ui_res[0]=='V')
    {
        bt_vo();
    }

    // 遍历命令映射表，查找匹配的命令
    for (int i = 0; i < map_size; i++) {
        // 先比较长度（快速排除不匹配项），再比较内容
        if (res_len == commandMap[i].cmd_len && 
            strcmp(ui_res, commandMap[i].command) == 0) {
            // 找到匹配项，执行对应处理函数
            commandMap[i].handler();
            return; // 处理完即退出
        }
    }

    // 处理未匹配的命令
    // ESP_LOGW("COMMAND", "未知命令: %s", ui_res);
}

/* *************
    按键初始化
    ************
*/
void key_init(void)
{
    gpio_config_t gpio_in = {};
    gpio_in.intr_type = GPIO_INTR_DISABLE;
    gpio_in.mode = GPIO_MODE_INPUT;
    gpio_in.pin_bit_mask = GPIO_INPUT_PIN; 
    gpio_in.pull_down_en = 0; 
    gpio_in.pull_up_en = 1;    
    gpio_config(&gpio_in);
}
//定义判断长按的变量
uint8_t key,key_last = 1;   //按键状态
bool is_long_press; //是否已经长按
static TickType_t press_start = 0;  // 按键按下的起始时间
TickType_t press_long;  //长按的时间
//按键任务
static void key_scan(void* arg)
{
    while(1)
    {
        key = gpio_get_level(GPIO_NUM_0);  //读取GPIO电平

        if (key == 0 && key_last == 1)  // 按下
        {
            press_start = xTaskGetTickCount();  // 记录按下开始时间
            is_long_press = false;  // 重置长按标记
        }
        // 按键长按（已按下且未触发过长按）
        else if (key == 0 && key_last == 0 && !is_long_press)
        {
            // 计算按下持续时间（转换为毫秒）
            TickType_t press_duration = pdTICKS_TO_MS(xTaskGetTickCount() - press_start);
            if (press_duration >= LONG_PRESS_TIME)
            {
                is_long_press = true;  // 标记为已长按
                uart_send("rst.val=1");
                vTaskDelay(pdMS_TO_TICKS(100)); 
                ESP_LOGI("esp","已重启");
                esp_restart();
            }
        }
        // 4. 检测按键松开（短按逻辑）
        else if (key == 1 && key_last == 0)
        {
            if (!is_long_press)
            {
                uart_send("page 1");
                // 短按
                ESP_LOGI("esp","已短按");
            }
            // 松开后重置计时
            press_start = 0;
        }

        // 5. 更新上一次按键状态
        key_last = key;
        vTaskDelay(pdMS_TO_TICKS(20));  
    }
}

void app_main(void)
{
    uart_2_init();  //串口屏通讯
    bt_init();      //蓝牙双模开启(总开关)
    bt_a2dp_work();     //蓝牙播放功能(经典蓝牙)
    ble_client_init();  //BLE GATT Client（主机模式，连接OV-Watch手表）—— 替代原 BLE Server 逻辑
    //ble_gatt_init();  //gatt协议(ble蓝牙) —— 原 BLE Server 初始化，已废弃（见 ble_client.c）
    // ble_gattc_init();    //客户端gatt
    // ble_gatts_init();    //服务端gatt
    //sd_init();
    //adc_init();
    //leds_init();
    key_init();
    xTaskCreate(key_scan, "key_scan", 2048, NULL, 10, NULL);      //创建key扫描任务
//              入口函数 终端显示函数名 栈深  参数 优先级 句柄
    while(1)
    {   
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // process_command(ui_res);    //启动命令分发
        // memset(ui_res, 0, sizeof(ui_res));
    }
}
