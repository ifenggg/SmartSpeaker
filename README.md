# Smart_Speaker（ESP32 双模蓝牙智能音箱）

ESP32（ESP-IDF v5.5.2）双模蓝牙音箱：经典蓝牙 A2DP 播放 + 串口屏 UI + PAM8403 功放 +
软件音量级配 + 氛围灯带 + SD 卡播放，并在此基础上实现 **BLE 主机链路：稳定连接手表 OV_WATCH 并双向通信**。

## 一、BLE 设计（音箱作主机 / 手表作从机）

| 文件 | 作用 | 状态 |
| --- | --- | --- |
| `main/ble_client.c` / `.h` | **BLE 主机（客户端）**：扫描 → 匹配手表 → 停扫描 → 建链 → 服务发现 → 订阅通知 → 收发 | **启用（核心）** |
| `main/ble_gap.c` / `.h` | GAP 事件统一分发（Bluedroid 只允许注册一个 GAP 回调，防止被后续模块顶掉） | 启用 |
| `main/ble_debug.c` / `.h` | 调试脚手架：连上后每 5s 发 `$PING,<序号>` + 打印收到的帧 | 已停用（源码保留） |
| `main/ble_server.c` / `.h` | 早期"音箱作 BLE 从机"的反向方案（已被最终设计取代） | 已停用（源码保留） |
| `main/ble.c` | 更早期 BLE 服务端+客户端示例实现 | 已废弃（`#if 0`） |

**对端（手表）特征约定（实测）**

| 项 | 值 |
| --- | --- |
| 广播名 | `OV_WATCH...`（**有时不广播名称**，只有扫描响应里带名 → 代码按服务 UUID 兜底匹配） |
| 地址 | `0f:02:b1:26:cd:1b`（公共地址，addr_type=0） |
| 主服务 | `0xFFF0`（handle 0x0001 ~ 0x0009） |
| 特征 `0xFFF1` | props = `0x14`（**Write Without Response + Notify**）→ 通知订阅与写数据都用它 |
| 特征候选 | 写特征按属性自动选择：`0xFFF2` → `0xFFF1` → `0xFFF3`（手表命中 `0xFFF1`） |
| MTU | 协商到 500 |

**连接流程（最终实现）**
`扫描(名称 OV_WATCH 优先，无名则按服务 FFF0 兜底) → 停扫描 → 等 SCAN_STOP_COMPLETE`
`→ set_prefer_conn_params + esp_ble_gattc_enh_open(显式 1M PHY 参数) → 物理连接建立`
`→ MTU 协商 → 服务发现(带 UUID 过滤) → 找 FFF1 通知特征 / 写特征 → register_for_notify`
`→ 写 CCCD 使能通知 → 【连接成功】`
异常断开后按指数退避（3/6/12/24/48s）自动重连，连续失败 5 次后暂停（可调 `ble_client_connect()` 重试）。

**连接参数档位表**（`main/ble_client.c` 的 `s_profiles[]`）

| 档位 | 参数 | 说明 |
| --- | --- | --- |
| 1 | **30~50 ms / latency 0 / 超时 4 s** | **手表实测一次连接成功、双向收发正常** → 首选 |
| 2 | 50~80 ms / latency 0 / 超时 6 s | 备用档（给响应更慢的模组留余量） |

每失败一次自动换下一档（循环），成功的那一档会被记住用于后续重连并打印日志。
（调试期用过的"协议栈默认 12.5~15 ms"与"本机随机地址"两档已注释保留在源码里备查。）

## 二、编译 / 烧录

```powershell
$env:IDF_PATH='D:\Q\ESP32\ESPIDF_5.5.2\v5.5.2\esp-idf'
$env:IDF_TOOLS_PATH='D:\Q\ESP32\tools_5.5.2'
$env:PATH='D:\Q\ESP32\tools_5.5.2\tools\idf-python\3.11.2;' + $env:PATH   # 让 export 找到 3.11 环境
. "$env:IDF_PATH\export.ps1"
idf.py build
idf.py -p COM21 flash monitor
```

> 若 `export.ps1` 报 "Python virtual environment ... not found"，是因为 PATH 里的 Python 3.12 被用来匹配
> `idf5.5_py3.12_env`；把 `tools\idf-python\3.11.2` 放到 PATH 最前面即可。

## 三、上电日志与验收

```
ble_client: 初始化 BLE GATT Client
ble_gap:    已登记 GAP 处理函数 #1
ble_client: GATTC 应用注册成功, gattc_if=4
ble_client: 开始扫描目标设备（名称 "OV_WATCH" / 服务 0xFFF0）...
ble_client: 命中目标设备: 名称='OV_WATCH' 地址=0f:02:b1:26:cd:1b 地址类型=0 RSSI=-63 adv=7 rsp=18 evt=0 → 已停止扫描，等待停扫完成后连接
ble_client: 扫描已停止
ble_client: 发起连接: 0f:02:b1:26:cd:1b (档位 1/2: 30~50ms/超时4s; addr_type=0, own_addr_type=0)
ble_client: 物理连接建立, conn_id=0
ble_client: GATT 连接已建立, conn_id=0, mtu=23
ble_client: 服务发现完成，搜索目标服务...
ble_client: 匹配到目标服务: handle [0x0001 ~ 0x0009]
ble_client: 找到通知特征 0xFFF1: handle=0x0003 (props=0x14)
ble_client: 找到写特征 0xFFF1: handle=0x0003 (props=0x14)
ble_client: 通知注册成功（handle=0x0003），写 CCCD 使能通知...
ble_client: MTU 协商完成: 500
ble_client: 【连接成功】对端='OV_WATCH'，Notify 已使能，写特征=0xFFF1，通知特征=0x0003（参数档位 1/2: 30~50ms/超时4s；conn_id=0 地址=0f:02:b1:26:cd:1b）
```

看到 `【连接成功】` 即链路可用，之后用 `ble_client_send()` / `ble_client_set_rx_cb()` 收发业务数据。

## 四、问题解决经过：`reason=0x3e` → 稳定连接（本次对话核心问题）

### 4.1 现象（最初）

ESP32 作主机连手表，每次都在建链阶段失败，`hcif disc complete: hdl 0x0, rsn 0x3e` 每秒一次、共 4 次，
随后 `连接断开, reason=0x3e`：
- `0x3e` = HCI **Connection Failed to be Established**（链路层未建立，即对端没有应答 `CONNECT_IND`）；
- 4 次 = 主机 1 次 + 自动重试 3 次（`CONFIG_BT_GATTC_CONNECT_RETRY_COUNT=3`）。

### 4.2 排查过程与关键证据

| 步骤 | 结论 |
| --- | --- |
| 通读日志 | 日志里 28 条 `$PING` 全是 `ret=-2` → **那条记录里从未连上过**，"收到第一条消息后断开"是另一个（更早的）故障现象，需分开看 |
| 检查工程 | 当时 `main.c` 有一行 `/ble_gatt_init();`（语法错误，工程根本编译不过）→ 修正；同时确认旧 BLE Server 代码（`ble.c`）已不在编译 |
| 排除"回调重复注册" | 日志里 `gattc_conn_cb: if=3 / if=4` 曾疑似重复注册；对照 Espressif 官方 issue（同一份日志格式，[esp-idf#12970](https://github.com/espressif/esp-idf/issues/12970)）确认：`if=3` 是 Bluedroid 内部 bta_dm 注册的 client，`if=4` 才是应用的 → **不是问题** |
| 对齐官方 gattc 例程 | 用已验证可用的官方例程逐项对齐：`enh_open` 只给地址类型、服务搜索带 UUID 过滤、`register_for_notify()` 后再写 CCCD、写特征按 `WRITE/WRITE_NR` 属性选择、`CONNECT_EVT` 里协商 MTU |
| **对照实验（决定性）** | 同一份固件连**手机从机（BLE 调试助手外设模式，广播名 "Ace 2V"，FFF1=通知 / FFF2=写）可稳定通信** → 客户端协议栈、回调流程、BTDM+A2DP 共存环境、扫描参数全部排除，**问题只可能在对端/连接参数** |
| 翻协议栈源码 | 找到根因线索：不指定连接参数时 Bluedroid 用**默认连接间隔 12.5~15 ms**（`btm_ble_api.h`：`BTM_BLE_CONN_INT_MIN_DEF=10`→12.5 ms、`MAX_DEF=12`→15 ms、超时 6 s），而手机作中心一般请求 30~50 ms |
| 实现"连接参数自动换档" | 失败自动换档并记录成功档位，用数据说话而不是猜 |

### 4.3 根因

**手表 KT6368A 透传模组无法在 12.5~15 ms 的连接间隔下工作**：
- 间隔太短时，对端要么来不及在锚点应答 → 主机报 `0x3e`（链路未建立）；
- 有时能建链，但很快 supervision timeout → 主机报 `0x08`（超时）。

这解释了"手机能连手表、ESP32 连不上"：手机（Android/iOS）作中心时请求的是 30~50 ms。
修复后的实测日志把这个因果链完整证明了 —— **先试默认档，`0x08` 超时；换到 30~50 ms，一次成功**：

```
I (2224)  ble_client: 发起连接: 0f:02:b1:26:cd:1b (档位 1/4: 协议栈默认(12.5~15ms); ...)
I (2774)  ble_client: 物理连接建立, conn_id=0
W (8804)  BT_HCI: hcif disc complete: hdl 0x0, rsn 0x8 dev_find 1        ← 6s 后监督超时（对端跟不上 12.5ms）
W (8814)  ble_client: 连接断开, reason=0x08
W (8824)  ble_client: 连接参数换档 → 档位 2/4：30~50ms/超时4s             ← 自动换档
I (14724) ble_client: 发起连接: 0f:02:b1:26:cd:1b (档位 2/4: 30~50ms/超时4s; ...)
I (16984) ble_client: 【连接成功】对端='OV_WATCH'，Notify 已使能，写特征=0xFFF1，通知特征=0x0003（参数档位 2/4: 30~50ms/超时4s；...）
I (21894) ble_debug:  TX[9] $PING,1\r\n => OV_WATCH:0                      ← 音箱→手表 OK
I (22994) ble_debug:  RX[OV_WATCH][9]: $PING,9\r\n                        ← 手表→音箱 OK
```

### 4.4 最终修复与保留的加固项

1. **连接参数首档改为 30~50 ms / 超时 4 s**（`s_profiles[0]`），并显式通过
   `esp_ble_gap_set_prefer_conn_params()` + `esp_ble_gattc_enh_open(phy_1m_conn_params)` 下发；
   备用档 50~80 ms；失败自动换档。
2. **先停扫描、等 `SCAN_STOP_COMPLETE` 再建链**（避免控制器在扫描未释放时发起连接）。
3. **名称匹配 + 服务 UUID 兜底**：手表有时只广播 7 字节、名字在扫描响应里甚至没有
   （实测第一次就是靠"无名设备命中 0xFFF0"兜底命中的），因此保留兜底并打印一次告警便于核对地址。
4. **GAP 事件单一回调分发**（`ble_gap.c`）：Bluedroid 只保存一个 GAP 回调，避免后续模块互相顶掉。
5. **通知流程与官方例程一致**：`register_for_notify()` → `REG_FOR_NOTIFY_EVT` 里写 CCCD(0x2902)。
6. **写特征按属性自动选择**：`0xFFF2` → `0xFFF1` → `0xFFF3`（手表命中 `0xFFF1`）。
7. 连接成功后清零失败计数、记住可用参数档位；断线指数退避重连；拥塞时发送返回 `-4` 不硬发。

### 4.5 换手表/换模组时的提醒

`30~50 ms` 是**实测这块 KT6368A 模组可用**的参数；若以后换模组/批次又出现连不上或连上即断，
先按第 4.3 节的思路确认对端能接受的连接间隔范围（必要时在 `s_profiles[]` 里增档），
再用第五节的 HCI 抓包确认链路层行为。

## 五、调试手段（均已停用，需要时按注释里的步骤启用）

| 手段 | 文件 | 启用方法 |
| --- | --- | --- |
| 调试帧收发（每 5s 发 `$PING` + 打印收到的帧） | `main/ble_debug.c/.h` | 去掉 `CMakeLists.txt` 里 `"ble_debug.c"` 的注释；`main.c` 里取消 `#include "ble_debug.h"` 与 `ble_debug_start();` 的注释；把文件顶部 `#if 0` 改成 `#if 1` |
| HCI 抓包（转 btsnoop 用 Wireshark 看链路层） | `main/main.c` 顶部 `#if 0` 块 | 把 `#if 0` 改成 `#if CONFIG_BT_HCI_LOG_DEBUG_EN`，并在 menuconfig → Component config → Bluetooth → `[x] Enable Bluetooth HCI debug mode`；串口重定向到文件后用 `tools/bt/bt_hci_to_btsnoop.py -p all_log.txt -o watch --has-ts` 转换 |
| 手机从机对照测试（把手机当"假手表"） | `main/ble_client.h` | 把 `TARGET_DEV_NAME_PREFIX_ALT` 填成手机从机广播名（如 `"Ace 2V"`）；手机需工作在**外设/从机模式**：服务 `0xFFF0`、`0xFFF1`=Notify、`0xFFF2`=Write |
| 双模共存对照（判断经典蓝牙/A2DP 是否干扰 BLE） | `main/main.c` / `main/bt_a2dp.c` | 注释掉 `bt_a2dp_work();`，并把 `bt_init()` 里 `ESP_BT_MODE_BTDM` 改成 `ESP_BT_MODE_BLE` |
| 手机作主机接入音箱（反向方案） | `main/ble_server.c/.h` | 已停用；需要时把 `ble_server.c` 加回 `CMakeLists.txt` 的 SRCS，并在 `main.c` 里调用 `ble_server_init()` |

## 六、对外接口（业务代码用这些）

```c
void    ble_client_init(void);                                   /* 启动即调用（bt_init() 之后） */
int     ble_client_send(uint8_t *data, uint16_t len);            /* 0=成功 -2=未连接 -3=无写特征 -4=拥塞 */
void    ble_client_set_rx_cb(void (*cb)(uint8_t *data, uint16_t len));  /* 手表发来的数据（回调内数据仅当时有效） */
uint8_t ble_client_get_state(void);                              /* 0=未连接 1=连接中 2=已连接 */
const char *ble_client_get_peer_name(void);                      /* 对端广播名（未连接返回 "-"） */
void    ble_client_connect(void);                                /* 手动重连（恢复自动重连并清零失败计数） */
void    ble_client_disconnect(void);                             /* 手动断开（暂停自动重连） */
```

## 七、排查开关（`main/ble_client.h`）

| 开关 | 默认 | 作用 |
| --- | --- | --- |
| `TARGET_DEV_NAME_PREFIX` | `"OV_WATCH"` | 手表广播名前缀 |
| `TARGET_DEV_NAME_PREFIX_ALT` | `""` | 【调试】第二个检索目标（手机从机），留空=不检索 |
| `BLE_CLIENT_CONN_PROFILE_LADDER` | `1` | 失败自动换连接参数档位；0 = 只用首档 |
| `WRITE_CHAR_UUID_PRIMARY/FALLBACK/FALLBACK2` | `FFF2/FFF1/FFF3` | 写特征候选顺序 |
| `BLE_CLIENT_MTU_REQUEST_ENABLE` | `1` | 连接后主动协商 MTU |
| `BLE_CLIENT_WRITE_WITH_RSP` | `0` | 1 = 用 Write With Response（可确知对端是否收到） |
| `BLE_CLIENT_RECONNECT_MAX_FAIL` | `5` | 连续失败多少次后暂停自动重连（0 = 一直重连） |
