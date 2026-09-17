/**
 * @file uart.h
 * @brief 串口屏（陶晶驰 TJC / USART HMI）UART 链路层
 *
 * 硬件：UART2  TX=GPIO17 → 屏幕 RX ；RX=GPIO16 ← 屏幕 TX ，115200 8N1
 *
 * =====================================================================
 * 一、发往串口屏的指令（uart_send / uart_send_text）
 * ---------------------------------------------------------------------
 *   · 帧结构：  <指令正文（可打印 ASCII）> + 0xFF 0xFF 0xFF
 *              （三个 0xFF 是陶晶驰协议规定的帧尾，由 uart_send 自动补，
 *                调用者只写正文，千万不要自己再拼帧尾）
 *   · 常用指令（本工程会用到）：
 *        page main            页面跳转（本工程用不到，界面切换由屏幕自己完成）
 *        obj.val=123          数值控件 / 滑块 / 进度条 / 数字控件赋值（不加引号）
 *        obj.txt="文本"       文本控件赋值（加双引号，正文里的 " 与 \ 需转义）
 *        obj.picc=7           图片 / 切图控件换图
 *        sendme               请求屏幕回报"当前页面 ID"（回复 0x66 <页ID> FF FF FF）
 *   · 参考：http://wiki.tjc1688.com/commands/index.html
 *
 * =====================================================================
 * 二、串口屏发来的数据（rx_task 断帧 → ui.c 的命令表分发）
 * ---------------------------------------------------------------------
 *   1) 带帧尾（推荐）：屏幕用 get 指令、勾选控件"发送键值"、或用 printh 自行补
 *      FF FF FF —— 收到帧尾即断帧，不会出现两条命令粘连。
 *   2) 不带帧尾：屏幕用 prints 原样发字符串（陶晶驰 prints 不带任何起止符），
 *      此时只能按"串口空闲间隔"断帧（见 UI_RX_IDLE_MS），因此**两次发送之间
 *      必须留出间隔**，否则会粘成一条。工程里推荐做法见 docs/串口屏对接遗漏清单.md。
 *
 *   二进制返回码（与文本命令自动区分，判定规则见 frame_is_binary()）：
 *        0x65 <页ID> <控件ID> <事件>   控件按下(0x01)/弹起(0x00)（需勾选"发送键值"）
 *        0x66 <页ID>                   当前页面 ID（sendme 的回复）
 *        0x67 <x> <y> <事件>           触摸坐标
 *        0x70 / 0x71 <数据…>           字符串 / 数值变量数据（get 指令的回复）
 *        0x88                          屏幕启动成功（屏幕上电/复位完成）
 *        0x00~0x24 等                  指令执行出错码（bkcmd != 0 时返回）
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一条"串口屏→音箱"命令的最大长度（含 '\0'）。
 * 10 字节的旧缓冲会把 "b0.val=100" 这类陶晶驰属性回传截断，故放大到 32。 */
#define UI_CMD_MAX          32

/* 无帧尾（prints 风格）时的断帧空闲阈值（ms）：
 * 连续 UI_RX_IDLE_MS 没有新字节，才把已累计的可打印内容当成一条完整命令。 */
#define UI_RX_IDLE_MS       30

/* 当前正在处理的那条命令（历史遗留：led_block()/bt_vo()/bt_page() 直接读它）。
 * 写入者是 ui.c 的命令分发任务，其它任务只读。 */
extern char ui_res[UI_CMD_MAX];

/* 串口屏当前页面 ID（由 0x66 回复更新；0xFF = 未知）。
 * 屏幕页面 ID 与页面名称的对照关系由上位机页面顺序决定，需在 ui.c 中配置。 */
extern volatile uint8_t ui_page_id;

/* 初始化 UART2（驱动程序 + 接收任务 + 命令队列） */
void uart_2_init(void);

/**
 * @brief 发送一条陶晶驰指令（自动补帧尾 FF FF FF，带互斥保护）
 * @param format printf 风格格式串，例：uart_send("obj.val=%u", v);
 * @note  多个任务会并发调用（蓝牙元数据 / 电量 / 灯带 / 页面刷新），
 *        本函数内部加互斥锁，保证"正文 + 帧尾"不会被别的指令插队切断。
 */
void uart_send(const char *format, ...);

/**
 * @brief 给文本控件赋值：obj.txt="<text>"（自动转义 " 与 \，剔除控制字符）
 * @param obj_txt_attr 属性名，例："t2.txt"
 * @param text         UTF-8 文本
 */
void uart_send_text(const char *obj_txt_attr, const char *text);

/**
 * @brief 取出一条串口屏命令（阻塞等待，由 ui.c 的分发任务调用）
 * @return true=取到，false=超时
 */
bool uart_recv_ui_cmd(char *out, uint32_t out_sz, uint32_t wait_ms);

/* TJC 二进制返回码回调（由 ui.c 注册，uart.c 只负责解析与转发） */
typedef void (*uart_tjc_key_cb_t)(uint8_t page_id, uint8_t comp_id, uint8_t event);
typedef void (*uart_tjc_page_cb_t)(uint8_t page_id);
void uart_set_tjc_binary_cbs(uart_tjc_key_cb_t key_cb, uart_tjc_page_cb_t page_cb);

/**
 * @brief 屏幕状态回调（0x86 自动休眠 / 0x87 自动唤醒 / 0x88 启动完成）
 * @note  0x86/0x87 由屏幕自动上报，是判定休眠/唤醒最可靠的依据；
 *        exit 超时判定（ui.c）作为兜底保留。
 */
typedef void (*uart_tjc_event_cb_t)(uint8_t code);
void uart_set_tjc_event_cb(uart_tjc_event_cb_t event_cb);

#ifdef __cplusplus
}
#endif
