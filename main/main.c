#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include <bt_a2dp.h>
#include "ble_client.h"
// #include "ble_debug.h"   // 【调试阶段代码，已停用】调试帧收发模块（见 main/ble_debug.c 顶部说明）
#include "amp.h"
#include "audio_vol.h"
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
/* =====================================================================
 * 串口屏接口说明（本文件只负责"分发"，具体命令格式由串口屏工程决定）
 * ---------------------------------------------------------------------
 * 接收链路：uart.c 的 rx_task 把串口屏发来的可打印 ASCII 过滤后写入 ui_res[10]，
 *          由本文件的 process_command() 分发。
 *
 * 一、前缀命令（先判首字符，可带参数）
 *     'V' + 0..127        → 本机音量，交 bt_vo()           例："V100"
 *     'R'/'G'/'B' + 数值  → 氛围灯颜色，交 led_block()      例："R255"
 *
 * 二、映射表命令（整串精确匹配，见下表 commandMap）
 *     "bt" / "tb"          切换蓝牙页/其他页
 *     "bton" / "btoff"     蓝牙开 / 关
 *     "btpau" / "btpla"    蓝牙暂停 / 播放
 *     "btne" / "btla"      下一首 / 上一首
 *     "btsil" / "btnoi"    静音 / 开声（本机音量 0 / 恢复）
 *     "sdon"/"sdoff"/"sdpau"/"sdpla"/"sdne"/"sdla"   SD 卡播放控制
 *     "ledon"/"ledoff"/"ledreon"/"led1"/"led2"       氛围灯控制
 *
 * 三、扩展方式
 *     定长命令   → 在 commandMap 里增加一行 { "命令串", 处理函数, strlen("命令串") }
 *     变长命令   → 仿照上面 'V' 的写法，在 process_command() 里增加一个首字符分支
 * ===================================================================== */

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

#if 0   /* ===================== 【调试阶段代码，已停用】 =====================
 * HCI 取证模式：调试"手表不应答（reason=0x3e / 0x08）"时用过。
 * 需要时把下面的 #if 0 改成 #if CONFIG_BT_HCI_LOG_DEBUG_EN，
 * 并在 menuconfig → Component config → Bluetooth → [x] Enable Bluetooth HCI debug mode。
 * 打开后本任务每秒把 HCI 数据流打印到串口：
 *   1) 串口重定向：idf.py -p COM21 monitor | Tee-Object all_log.txt
 *   2) 转 btsnoop：python tools/bt/bt_hci_to_btsnoop.py -p all_log.txt -o watch --has-ts
 *   3) Wireshark 打开 parsed_log_watch.btsnoop.log，看 LE Create Connection 参数与是否收到
 *      LE Connection Complete（无回包 = 对端不应答）。
 * ==================================================================== */
extern void bt_hci_log_hci_data_show(void);
extern void bt_hci_log_hci_adv_show(void);

static void hci_log_dump_task(void *arg)
{
    while (1) {
        bt_hci_log_hci_data_show();
        bt_hci_log_hci_adv_show();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif  /* ==================== 调试阶段代码结束 ==================== */

void app_main(void)
{
    uart_2_init();  //串口屏通讯
    amp_init();     //PAM8403 功放初始化（静音/电源控制，防开机爆音）
    audio_vol_init();   //软件音量级配初始化（必须在蓝牙起流之前，用于消除大音量炸音）
    bt_init();      //蓝牙双模开启(总开关)
    bt_a2dp_work();     //蓝牙播放功能(经典蓝牙)

    ble_client_init();  //BLE GATT Client（主机）：扫描并连接手表 OV_WATCH（服务 0xFFF0 / 特征 0xFFF1）
    // ble_debug_start();  //【调试阶段代码，已停用】连上后每 5 秒发 $PING,<序号> 并打印收到的帧

    //leds_init();
    //leds_mo1();
    key_init();
    xTaskCreate(key_scan, "key_scan", 2048, NULL, 10, NULL);      //创建key扫描任务
//              入口函数 终端显示函数名 栈深  参数 优先级 句柄

#if 0   /* 【调试阶段代码，已停用】HCI 调试模式的任务启动 */
    xTaskCreate(hci_log_dump_task, "hci_dump", 3072, NULL, 3, NULL);
    ESP_LOGW("esp", "已开启 HCI 调试模式：串口会持续打印 HCI 数据（排查链路层问题用）");
#endif

    // 说明：音箱在本工程中**只作 BLE 主机**，连接手表；
    //       BLE 侧调试代码（main/ble_debug.c、HCI 取证）、手机从机反向方案
    //       （main/ble_server.c）与早期示例（main/ble.c）均已停用/移出编译，源码保留备查。

    while(1)
    {   
        process_command(ui_res);    //启动命令分发（串口屏音量/氛围灯命令由此生效）
        memset(ui_res, 0, sizeof(ui_res));
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
