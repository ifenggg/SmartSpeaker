/**
 * @file ble_server.h
 * @brief 【已停用 / 不在编译中】BLE GATT Server（从机）—— 手机调试助手接入音箱
 *
 * ⚠ 本模块目前**不参与编译**（见 main/CMakeLists.txt），文件保留备用。
 *
 * 停用原因（设计已明确）：
 *   本工程的调试链路是"**音箱始终作为 BLE 主机（客户端）**，对端（手表 / 手机）才是从机"。
 *   手机端由 BLE 调试助手以外设（从机）模式广播 "Ace 2V"，音箱作为主机去连它并订阅/写入
 *   对应特征（FFF1 通知 / FFF2 写），实现见 ble_client.c + ble_debug.c。
 *   本文件是早前"反向"方案（音箱作从机、手机作主机接入音箱）的实现，与上述设计相反，
 *   因此停用；若将来确实需要音箱同时充当从机，可重新加入编译并调用 ble_server_init()。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 一、对外配置
 * ===================================================================== */
#define BLE_SERVER_ADV_NAME         "Ace 2V"    /* 广播设备名（允许空格，最长 26 字节） */
#define BLE_SERVER_SERVICE_UUID     0xFFF0      /* 主服务 UUID（16 位） */
#define BLE_SERVER_NOTIFY_CHAR_UUID 0xFFF1      /* 音箱→手机：Read + Notify */
#define BLE_SERVER_WRITE_CHAR_UUID  0xFFF2      /* 手机→音箱：Read + Write + WriteNR */
#define BLE_SERVER_ATTR_MAX_LEN     512         /* 特征值最大长度（Notify 实际受 MTU 限制） */

/* 广播开关：置 0 则只保留 GATT 服务、不广播（排查"BLE 主机连手表"与广播共存干扰时可用） */
#define BLE_SERVER_AUTO_ADV         1

/* =====================================================================
 * 二、对外接口
 * ===================================================================== */

/**
 * @brief 初始化 BLE GATT Server（含服务注册与广播，系统启动时调用一次）
 * @note  须在 bt_init() 之后；与 ble_client_init() 无先后要求（GAP 事件由 ble_gap 统一分发）
 */
void ble_server_init(void);

/**
 * @brief 向已订阅的手机发送一帧数据（从 0xFFF1 以 Notify 下发）
 * @param data 数据指针（函数内部会拷贝，调用返回后即可释放）
 * @param len  数据长度
 * @return 0 成功；
 *         -1 参数非法；-2 手机未连接；-3 手机未订阅通知；-4 数据超过当前 MTU 上限；
 *         其他为底层错误码
 */
int ble_server_send(uint8_t *data, uint16_t len);

/**
 * @brief 注册接收回调（手机向 0xFFF2 写入时回调）
 * @param cb void cb(uint8_t *data, uint16_t len)；data 仅在回调期间有效，请及时拷贝
 */
void ble_server_set_rx_cb(void (*cb)(uint8_t *data, uint16_t len));

/** @brief 手机是否已接入（GATT 连接已建立） */
bool ble_server_is_connected(void);

/** @brief 手机是否已订阅 0xFFF1 的通知（可以给它发帧） */
bool ble_server_is_notify_enabled(void);

/**
 * @brief 广播开关（可在运行时控制；BLE_SERVER_AUTO_ADV=0 时本函数无效）
 */
void ble_server_start_adv(void);
void ble_server_stop_adv(void);

#ifdef __cplusplus
}
#endif
