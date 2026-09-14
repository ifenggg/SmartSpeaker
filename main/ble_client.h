/**
 * @file ble_client.h
 * @brief 蓝牙 BLE GATT Client（主机）模块 —— 智能音箱连接 OV-Watch 手表 / 手机自研 BLE 从机
 *
 * 设计目标：
 *   1. 独立封装 BLE 客户端逻辑，与 A2DP 经典蓝牙音频链路完全解耦，双模共存；
 *   2. 集中配置目标设备参数（名称 / UUID），预留空值待填充；
 *   3. 内置状态机 + 超时 + 自动重连，扫描结束后立即停止以降低对 A2DP 音频的干扰。
 *
 * 依赖：本模块须在蓝牙控制器与 Bluedroid 已使能（bt_init() 之后）再调用 ble_client_init()。
 */
#pragma once

#include <stdint.h>
#include "esp_gattc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 一、目标设备参数 —— 【集中配置区】
 * =====================================================================
 * 说明：
 *   1. 名称前缀与服务 UUID 至少填一项；名称前缀优先于服务 UUID。
 *   2. 手表为固定 16 位 UUID：主服务 FFF0，特征 FFF1(写+通知)/FFF2(读+通知)/FFF3(写)。
 *   3. 本模块当前仅用「写 + 通知」双向通道：特征1(FFF1) 同时用于 Write 与 Notify；
 *      特征2(FFF2, Read+Notify) 与特征3(FFF3, Write) 暂未使用（预留扩展 Read）。
 * ===================================================================== */
#define TARGET_DEV_NAME_PREFIX      "OV_WATCH"   /* 手表广播名称（KT6368A 透传模块，固定） */
#define TARGET_SERVICE_UUID         0xFFF0   /* 16 位主服务 UUID（固定） */
#define WRITE_CHAR_UUID             0xFFF1   /* 16 位写特征 UUID（特征1：Write + Notify，音箱→手表） */
#define NOTIFY_CHAR_UUID            0xFFF1   /* 16 位通知特征 UUID（特征1：Write + Notify，手表→音箱） */

/* =====================================================================
 * 二、扫描 / 连接 / 重连参数（可调）
 * ===================================================================== */
#define BLE_CLIENT_SCAN_TIMEOUT_MS      10000   /* 单轮扫描超时（毫秒） */
#define BLE_CLIENT_CONNECT_TIMEOUT_MS   10000   /* 连接 + 服务发现 + 使能通知 总超时（毫秒） */
#define BLE_CLIENT_RECONNECT_DELAY_MS   3000    /* 异常断开后重连延时（毫秒） */
#define BLE_CLIENT_SCAN_INTERVAL        0x50    /* 扫描间隔 = 0x50 * 0.625ms = 100ms（降低对 A2DP 的干扰） */
#define BLE_CLIENT_SCAN_WINDOW          0x30    /* 扫描窗口 = 0x30 * 0.625ms = 60ms */
#define BLE_CLIENT_MTU                  247     /* 请求的 MTU（Notify 大数据量时使用） */
#define BLE_CLIENT_SCAN_LOG_ALL         1       /* 调试：打印每个扫描到的设备（名称/地址/RSSI）；排查完可置 0 */

/* =====================================================================
 * 三、连接状态（ble_client_get_state 返回值）
 * ===================================================================== */
#define BLE_CLIENT_STATE_DISCONNECTED   0   /* 未连接 */
#define BLE_CLIENT_STATE_CONNECTING     1   /* 连接中（含扫描 / 连接 / 服务发现 / 使能通知） */
#define BLE_CLIENT_STATE_CONNECTED      2   /* 已连接（Notify 已使能，可收发） */

/* =====================================================================
 * 四、对外接口
 * ===================================================================== */

/**
 * @brief 初始化 BLE Client（系统启动时调用一次，须在 bt_init() 之后）
 */
void ble_client_init(void);

/**
 * @brief 向已连接的对端发送数据（Write Without Response）
 * @param data  数据指针
 * @param len   数据长度（字节）
 * @return 0 成功；非 0 失败（-1 参数非法 / -2 未连接 / -3 未配置可写特征 / 其他为底层错误码）
 */
int ble_client_send(uint8_t *data, uint16_t len);

/**
 * @brief 注册接收回调（Notify 数据到达时回调）
 * @param cb  void cb(uint8_t *data, uint16_t len)；data 仅在回调期间有效，请及时拷贝
 */
void ble_client_set_rx_cb(void (*cb)(uint8_t *data, uint16_t len));

/**
 * @brief 获取当前连接状态
 * @return 0 未连接 / 1 连接中 / 2 已连接
 */
uint8_t ble_client_get_state(void);

/**
 * @brief 手动连接（恢复自动重连并立即发起扫描连接）
 */
void ble_client_connect(void);

/**
 * @brief 手动断开（并暂停自动重连，直到再次调用 ble_client_connect）
 */
void ble_client_disconnect(void);

#ifdef __cplusplus
}
#endif
