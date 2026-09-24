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
#include "ui.h"          // 串口屏交互层（页面状态机 + 命令分发 + 切页状态补发）
#include "health.h"      // 手表健康数据（解析 + NVS 保存 + health 页刷新 + 睡眠联动）
#include "sd.h"
#include <adc.h>

#define GPIO_INPUT_PIN   (1ULL << GPIO_NUM_0)
#define LONG_PRESS_TIME 2000  // 长按阈值：2000ms

/* ***********
    Main变量声明
    ***********
*/

/* =====================================================================
 * 串口屏接口说明
 * ---------------------------------------------------------------------
 * 串口屏（陶晶驰 TJC3224T124）的所有交互已集中到 ui.c / ui.h：
 *   · 接收链路： uart.c 的 rx_task 按帧尾 0xFF 0xFF 0xFF（或空闲间隔）断帧
 *                → 命令队列 → ui.c 的 ui_task → ui_process_command()
 *   · 切页补发： 屏幕每次切页会发回页面名（f1/main/leds/set/bt/health/sd），
 *                ui.c 记录当前页面并调用 ui_refresh_page() 把该页需要的状态重发一次
 *   · 命令表：   见 ui.c 顶部的 commandMap 与 R/G/B、V 前缀命令
 * 本文件只负责：开机初始化各功能模块 + 按键扫描。
 * ===================================================================== */

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
                uart_send("page 1");    // 短按：回到串口屏第 1 页（f1 待机界面）
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
    /* ---------- 1. 串口屏链路 + 交互层（最先起，方便开机就把状态刷到屏幕） ---------- */
    uart_2_init();      //串口屏通讯（陶晶驰协议：发送自动补帧尾，接收断帧）
    ui_init();          //串口屏交互层：页面状态机 + 命令分发（必须在 uart_2_init 之后）

    /* ---------- 2. 音频通路（功放 + 软件音量级配） ---------- */
    amp_init();         //PAM8403 功放初始化（静音/电源控制，防开机爆音）
    audio_vol_init();   //软件音量级配初始化（必须在蓝牙起流之前，用于消除大音量炸音）

    /* ---------- 3. 蓝牙：上电即开（双模），同时可由串口屏开关 ----------
     * bt_init()        初始化：控制器(BTDM)+Bluedroid，上电只执行一次
     * bt_a2dp_work()   功能层"开"：注册 A2DP/AVRCP 并进入可发现
     * bt_app_shutdown()功能层"关"
     * 串口屏"蓝牙总开关"走的正是这一对函数（bton/btoff），不会再调用 bt_init()。 */
    bt_init();          //蓝牙双模开启(总开关：初始化)
    bt_a2dp_work();     //蓝牙功能开启(经典蓝牙 A2DP/AVRCP)

    /* ---------- 4. BLE 主机：连接手表（health 页数据来源） ---------- */
    ble_client_init();  //BLE GATT Client（主机）：扫描并连接手表 OV_WATCH（服务 0xFFF0 / 特征 0xFFF1）
    health_init();      //健康数据：NVS 载入上次数值 + 注册手表数据回调 + 睡眠标志联动

    /* ---------- 5. 灯带：只初始化不点亮 ----------
     * leds_init() 只创建 RMT 资源并把灯带清成"灭"；
     * 点亮/熄灭由串口屏"灯带开关" → leds_on() / leds_off() 控制（调度级，不是初始化）。 */
    leds_init();

    /* ---------- 6. 按键 ---------- */
    key_init();
    xTaskCreate(key_scan, "key_scan", 2048, NULL, 10, NULL);      //创建key扫描任务
//              入口函数 终端显示函数名 栈深  参数 优先级 句柄

    // 说明：音箱在本工程中**只作 BLE 主机**，连接手表；
    //       BLE 侧调试代码（main/ble_debug.c、HCI 取证）、手机从机反向方案
    //       （main/ble_server.c）与早期示例（main/ble.c）均已停用/移出编译，源码保留备查。

    while(1)
    {
        /* 串口屏命令已由 ui.c 的 ui_task 用队列分发（不再在主循环里 500ms 轮询：
         * 轮询既慢又会在 clear 之前重复处理同一条命令）。主循环只做低频看护。 */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* =====================================================================
 *                      【调试阶段代码区】
 * ---------------------------------------------------------------------
 * 说明：以下调用都不参与上电流程，需要调试/验证时才启用。
 *       启用方式：去掉对应的 #if 0 / 注释。
 * ===================================================================== */

#if 0   /* ---------- 调试项 1：ADC 电量检测（本阶段明确禁止启用） ----------
 * 当前阶段要求：**不能初始化 ADC，也不能调用 ADC 电量采集**。
 * 因此 adc_init() 不参与上电流程，全局 battery 也不会被更新；
 * 串口屏显示的电量由 main/ui.c 固定发送 60（s_battery 初值）。
 * 以后硬件确认（电池分压接到 GPIO14 / ADC2_CH6）后，再：
 *   1) 去掉下面 adc_init() 的注释；
 *   2) 恢复 main/adc.c 里被注释掉的 ui_set_battery(battery) 调用。
 */
    // adc_init();      //ADC 电量检测（每 3 秒首测，之后每分钟一次；低电量联动功放）
#endif  /* ---------------- 调试项 1 结束 ---------------- */

#if 0   /* ---------- 调试项 2：上电自动跑灯效（当前不需要：灯带上电必须是灭的） ----------
 * 灯带点亮现在完全由串口屏"灯带开关"控制，所以上电不再自动跑彩虹灯效。
 * 需要单独验证灯效时再打开（注意：灯带开关是关的，还要先 leds_on() 才会亮）。
 */
    // leds_mo1();      //上电即进入"流动彩虹"灯效
#endif  /* ---------------- 调试项 2 结束 ---------------- */

#if 0   /* ---------- 调试项 3：HCI 取证模式（排查"手表不应答 reason=0x3e/0x08"时用过） ----------
 * 需要时把下面的 #if 0 改成 #if CONFIG_BT_HCI_LOG_DEBUG_EN，
 * 并在 menuconfig → Component config → Bluetooth → [x] Enable Bluetooth HCI debug mode。
 * 打开后本任务每秒把 HCI 数据流打印到串口：
 *   1) 串口重定向：idf.py -p COM21 monitor | Tee-Object all_log.txt
 *   2) 转 btsnoop：python tools/bt/bt_hci_to_btsnoop.py -p all_log.txt -o watch --has-ts
 *   3) Wireshark 打开 parsed_log_watch.btsnoop.log，看 LE Create Connection 参数与是否收到
 *      LE Connection Complete（无回包 = 对端不应答）。
 */
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

static void hci_debug_start(void)
{
    xTaskCreate(hci_log_dump_task, "hci_dump", 3072, NULL, 3, NULL);
    ESP_LOGW("esp", "已开启 HCI 调试模式：串口会持续打印 HCI 数据（排查链路层问题用）");
    ble_debug_start();  //【调试阶段代码，已停用】连上后每 5 秒发 $PING,<序号> 并打印收到的帧
}
#endif  /* ---------------- 调试项 3 结束 ---------------- */
