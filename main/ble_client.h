/**
 * @file ble_client.h
 * @brief 蓝牙 BLE GATT Client（主机）模块 —— 智能音箱连接 OV-Watch 手表
 *
 * 设计目标：
 *   1. 独立封装 BLE 客户端逻辑，与 A2DP 经典蓝牙音频链路解耦、双模共存；
 *   2. 集中配置目标设备参数（名称 / UUID）；
 *   3. 内置状态机 + 指数退避重连 + 统一清理，扫描结束立即停止以降低对 A2DP 的干扰。
 *
 * 关键流程（针对 0x3e 连接失败做过加固）：
 *   扫描命中 → 暂存地址/地址类型 → stop_scanning → **等 SCAN_STOP_COMPLETE**
 *   → esp_ble_gattc_enh_open()（显式 own_addr_type / remote_addr_type / 连接参数）
 *   → 服务发现 → 查询特征 → 写 CCCD 使能 Notify → CONNECTED
 *
 * 依赖：本模块须在蓝牙控制器与 Bluedroid 已使能（bt_init() 之后）再调用 ble_client_init()。
 * 注意：GAP 事件由 ble_gap.c 统一分发（Bluedroid 只允许注册一个 GAP 回调）。
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
 * 音箱作为 BLE 主机（客户端），连接手表（KT6368A 透传模组，GATT Server）：
 *   主目标：手表（广播名前缀见 TARGET_DEV_NAME_PREFIX，当前 "LSWOV"）；
 *   兜底：手表的**名字只出现在扫描响应里**（广播只有 Flags + 服务 UUID，7 字节），
 *         一旦扫描响应没收到（信号弱时很常见），名字就解析不到 —— 此时按服务
 *         UUID 0xFFF0 兜底匹配（会打印一次告警便于核对地址）。
 *
 * 特征（手表固定布局，已实测）：
 *   主服务 0xFFF0；特征 0xFFF1（props=0x14：Write Without Response + Notify）
 *     · 通知（手表→音箱）：订阅 0xFFF1 的 CCCD
 *     · 写  （音箱→手表）：候选顺序 0xFFF2 → 0xFFF1 → 0xFFF3，按属性自动选择
 * ===================================================================== */
#define TARGET_DEV_NAME_PREFIX      "LSWOV"   /* 手表广播名前缀 */

/* 【调试用】对端作 BLE 从机时的第二个检索目标（手机 BLE 调试助手的外设模式）。
 * 调试期填 "Ace 2V" 可让音箱去连手机从机做对照测试；量产只连手表时保持 ""（不检索）。 */
#define TARGET_DEV_NAME_PREFIX_ALT  ""

#define TARGET_SERVICE_UUID         0xFFF0       /* 16 位主服务 UUID（固定） */
#define NOTIFY_CHAR_UUID            0xFFF1       /* 通知特征（对端→音箱） */
#define WRITE_CHAR_UUID_PRIMARY     0xFFF2       /* 写特征候选1：手机作从机时 */
#define WRITE_CHAR_UUID_FALLBACK    0xFFF1       /* 写特征候选2：手表 FFF1(写+通知) */
#define WRITE_CHAR_UUID_FALLBACK2   0xFFF3       /* 写特征候选3：手表 FFF3(写)，前两个都不可写时用 */

/* 本机 BLE 名称（只作主机，不广播；对端 App 里显示用） */
#define BLE_CLIENT_LOCAL_NAME       "ESP_speaker-BLE"

/* =====================================================================
 * 二、扫描 / 连接 / 重连参数（可调）
 * ===================================================================== */
#define BLE_CLIENT_SCAN_TIMEOUT_MS      10000   /* 单轮扫描超时（毫秒） */
#define BLE_CLIENT_CONNECT_TIMEOUT_MS   10000   /* 连接 + 服务发现 + 使能通知 总超时（毫秒） */

/* 扫描参数：间隔 100ms / 窗口 60ms（降低对 A2DP 的干扰） */
#define BLE_CLIENT_SCAN_INTERVAL        0x50
#define BLE_CLIENT_SCAN_WINDOW          0x30

/* 发起连接时控制器自己的扫描窗口（连接参数档位表里使用） */
#define BLE_CLIENT_INIT_SCAN_INTERVAL   0x60    /* 60ms  */
#define BLE_CLIENT_INIT_SCAN_WINDOW     0x30    /* 30ms  */

/* 连接参数（仅在档位表中使用）
 *   interval 单位 1.25ms：0x18=30ms，0x28=50ms，0x40=80ms；timeout 单位 10ms：0x0190=4000ms
 * 约束：timeout*10 >= (1+latency) * (interval_max*5/2)，否则 enh_open 会拒绝参数 */
#define BLE_CLIENT_CONN_INTERVAL_MIN    0x18
#define BLE_CLIENT_CONN_INTERVAL_MAX    0x28
#define BLE_CLIENT_CONN_LATENCY         0
#define BLE_CLIENT_CONN_TIMEOUT         0x0190

/* ============================ 连接参数档位表 ============================
 * 实测结论（关键）：
 *   手表 KT6368A 模组**只能在 30~50ms 这类较长间隔下稳定工作**：
 *     · 协议栈默认档（12.5~15ms，Bluedroid 的 BTM_BLE_CONN_INT_*_DEF）→ 能建链，
 *       但约 6s 后 supervision timeout 断开（reason=0x08，对端跟不上 12.5ms 间隔）；
 *     · 30~50ms/超时4s → 一次连接成功，服务发现/订阅/双向收发均正常。
 * 因此量产档位表把"30~50ms"放在第一档（见 ble_client.c 的 s_profiles[]）。
 * 每失败一次自动换下一档（循环），成功的那一档会被记住用于后续重连。
 * 置 0 则只使用第一档。 */
#define BLE_CLIENT_CONN_PROFILE_LADDER  1

/* 重连退避：3s → 6s → 12s → 24s → 48s → 60s(上限)；连续失败达到上限后暂停自动重连 */
#define BLE_CLIENT_RECONNECT_DELAY_MIN_MS   3000
#define BLE_CLIENT_RECONNECT_DELAY_MAX_MS   60000
#define BLE_CLIENT_RECONNECT_MAX_FAIL       5   /* 0 = 一直重连不放弃 */

/* 本地 MTU（GATT 公共配置；与已验证例程一致取 500） */
#define BLE_CLIENT_LOCAL_MTU            500

/* 连接建立后是否主动发起 MTU 协商（与已验证例程一致：开启） */
#define BLE_CLIENT_MTU_REQUEST_ENABLE   1

/* 写特征方式：0 = Write Without Response；1 = Write With Response（可确知对端是否收到） */
#define BLE_CLIENT_WRITE_WITH_RSP       0

/* 命中设备时若 RSSI 低于该值，打印"信号太弱、建链大概率失败"的告警（dBm）
 *
 * 实测结论（手表 LSWOV / 0f:02:b1:26:cd:1b）：
 *   RSSI -63 ~ -74 dBm → 30~50ms 参数可稳定建链并双向收发；
 *   RSSI -89 ~ -96 dBm → 建链必失败（reason=0x3e），且扫描响应(rsp)恒为 0
 *                        —— 说明对端已经听不到我们发的包（SCAN_REQ / CONNECT_IND）。
 *   RSSI 越差越要先怀疑距离/遮挡：手机能连只因为手机射频比 ESP32 模组好。
 *
 * 现场要求：音箱与手表尽量靠近（建议 1m 内、无遮挡）；-85 dBm 以下不要指望能连上。
 * 可选加固（默认未启用）：若希望"只在信号足够好时才建链"，可在 gap_event_handler 的
 *   SCAN_RESULT 分支里对 rssi < 该阈值的情况直接 break（继续扫描等待更强的广播）。 */
#define BLE_CLIENT_RSSI_WARN_DBM        (-85)

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
 * @brief 向已连接的手表发送数据
 * @param data  数据指针
 * @param len   数据长度（字节）
 * @return 0 成功；-1 参数非法 / -2 未连接 / -3 未配置可写特征 / -4 链路拥塞；其他为底层错误码
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
 * @brief 获取当前（或最近一次）连接的对端名称
 * @return 对端广播名；未连过任何设备时返回 "-"
 */
const char *ble_client_get_peer_name(void);

/**
 * @brief 手动连接（恢复自动重连、清零失败计数并立即发起扫描）
 */
void ble_client_connect(void);

/**
 * @brief 手动断开（并暂停自动重连，直到再次调用 ble_client_connect）
 */
void ble_client_disconnect(void);

#ifdef __cplusplus
}
#endif
