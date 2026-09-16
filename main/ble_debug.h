/**
 * @file ble_debug.h
 * @brief 【调试阶段代码，已停用】BLE 双向通信调试模块声明
 *
 * ⚠ 本文件当前**不参与编译**（main/CMakeLists.txt 已注释 ble_debug.c）。
 *   它是排查手表链路期间临时加上的调试脚手架（定期发 $PING + 打印收到的帧），
 *   问题已解决后停用；正式业务请直接使用 main/ble_client.h 的：
 *     · `ble_client_send()`        —— 向手表发数据
 *     · `ble_client_set_rx_cb()`   —— 注册接收回调
 *   重新启用步骤见 main/ble_debug.c 顶部说明。
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
#define FRAME_CMD_TIME   "TIME"   /* 时间同步（手表→手表，参数 时间戳） */

/**
 * @brief 启动 BLE 调试流程
 *  - 注册接收回调，收到的帧按对端名称打印到终端；
 *  - 连上手表或手机后，周期性发送 `$PING,<递增序号>`。
 *  @note 须在 ble_client_init() 之后调用。
 */
void ble_debug_start(void);

#ifdef __cplusplus
}
#endif
