/**
 * @file ble_debug.h
 * @brief BLE 双向通信调试模块（调试阶段使用，正式协议落地后可移除）
 *
 * 帧格式（字符串，KT6368A 透传，BLE 层无帧协议）：
 *     $<指令码>[,<参数1>[,<参数2>[,...]]]\r\n
 *   例：$PING,1\r\n    $BAT,86\r\n    $LED,255,0,128\r\n
 *   规则：
 *     - 帧头 '$'；指令码 2~4 位大写字母；参数逗号分隔（0~4 个，十进制数字或短文本）；
 *     - 帧尾 "\r\n"；单帧总长（含帧头帧尾）≤ 20 字节（KT6368A 默认 MTU23 载荷上限）。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* 指令码（调试阶段固定表） */
#define FRAME_CMD_PING   "PING"   /* 心跳/连通性 */
#define FRAME_CMD_BAT    "BAT"    /* 电量上报（手表→音箱，参数 0~100） */
#define FRAME_CMD_VOL    "VOL"    /* 音量同步（音箱→手表，参数 0~100） */
#define FRAME_CMD_LED    "LED"    /* 灯控（音箱→手表，参数 R,G,B） */
#define FRAME_CMD_TIME   "TIME"   /* 时间同步（手表→音箱，参数 时间戳） */

/**
 * @brief 启动 BLE 调试流程
 *  - 把收到的帧打印到终端；
 *  - 周期性发送调试帧：$PING,<递增序号>。
 */
void ble_debug_start(void);

#ifdef __cplusplus
}
#endif
