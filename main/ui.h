/**
 * @file ui.h
 * @brief 串口屏（陶晶驰 TJC）交互层：页面状态机 + 命令分发 + 切页状态补发
 *
 * 设计要点（对应串口屏"不支持跨界面刷新"的限制）：
 *   1. 屏幕上每个控件默认是**私有**的：切页后离开时内存被释放，再回来会恢复成初始值。
 *      因此**每次切页都必须由音箱把该页的数据/开关/按钮状态补发一次**；
 *   2. 屏幕每次切页都会发回"页面名称"（用户在上位机里配置的），本模块据此
 *      记录当前页面并调用 ui_refresh_page() 补发该页状态；
 *   3. 屏幕在"页面离开事件"里会发 **exit**：
 *        · 正常切页 → exit 之后紧接着就收到新页面名，按切页处理；
 *        · 休眠超时退出页面 → exit 之后一直等不到新页面名，
 *          超过 UI_EXIT_GRACE_MS 即判定**屏幕已休眠**（UI_PAGE_SLEEP），
 *          此后不再向屏幕推送页面数据；屏幕被触摸唤醒后（重新收到任何命令）
 *          自动恢复，并补发电量 + 发送 sendme 查询当前页面。
 *   4. 所有"切页后要补发什么"的逻辑集中在 ui_refresh_page() 一个函数里。
 *
 * 命令分发链路：
 *   屏幕 → uart.c(rx_task 断帧) → 命令队列 → ui_task → ui_process_command()
 *        ├─ 页面名    → ui_enter_page() → ui_refresh_page() 补发状态
 *        ├─ exit      → 休眠判定（见上）
 *        ├─ n0/n2/h0/h1/h2.val → 按当前页面判定含义（见 ui.c 顶部）
 *        ├─ R/G/B…    → led_block()      三色滚动条（旧写法）
 *        ├─ V<0-127>  → bt_vo()          音量条（旧写法）
 *        └─ 命令表    → 蓝牙/灯带/SD 等处理函数
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 串口屏页面（与上位机 7 个界面一一对应） */
typedef enum {
    UI_PAGE_UNKNOWN = 0,    /* 未知（还没收到过页面名） */
    UI_PAGE_F1,             /* f1     待机界面（与音箱无关，不发送任何数据） */
    UI_PAGE_MAIN,           /* main   主界面：只交互电量 */
    UI_PAGE_LEDS,           /* leds   灯带控制：开关 n0 + 三色条 h0/h1/h2 + 灯效 led1/led2 */
    UI_PAGE_SET,            /* set    屏幕设置：只交互电量 */
    UI_PAGE_BT,             /* bt     蓝牙音频：开关 n0 + 音量条 h0 + 音量开关 n2 + 歌词区 */
    UI_PAGE_HEALTH,         /* health 健康数据：本阶段只交互电量 */
    UI_PAGE_SD,             /* sd     SD 卡播放：本阶段只交互电量 */
    UI_PAGE_SLEEP,          /* 屏幕已休眠（收到 exit 后一直等不到新页面名） */
} ui_page_t;

/* 收到 exit 后等待"新页面名"的宽限时间：超时仍没等到即判定屏幕休眠 */
#define UI_EXIT_GRACE_MS        1500

/* 启动交互层（必须在 uart_2_init() 之后调用） */
void ui_init(void);

/* 当前串口屏页面（供各功能模块判断"要不要往屏幕发状态"） */
ui_page_t ui_get_page(void);
const char *ui_page_name(ui_page_t page);

/* 屏幕是否已休眠（收到 exit 后 UI_EXIT_GRACE_MS 内没有新页面名） */
bool ui_is_screen_asleep(void);

/* 处理一条屏幕命令（内部使用；也可在调试代码里手动调用） */
void ui_process_command(const char *cmd);

/* 主动查询屏幕当前页面 ID：发 sendme，屏幕回复 0x66 <页ID> */
void ui_query_page(void);

/* ---------------- 电量（各页右上角 "ba.val"） ---------------- */
/**
 * @brief 上报电量百分比（由 adc.c 测量后调用；数值变化时自动推送到屏幕）
 */
void ui_set_battery(uint8_t percent);
/**
 * @brief 按当前缓存值强制补发一次电量（切页时调用）
 */
void ui_push_battery(void);

/* ---------------- 各页状态补发（切页后调用） ---------------- */
/**
 * @brief 补发指定页面的全部状态（电量 + 该页控件状态）
 * @note  串口屏跨页控件会复位，切页后必须调用本函数
 */
void ui_refresh_page(ui_page_t page);

/* 蓝牙页：连接状态（t1.txt） */
void ui_push_bt_conn_state(void);
/* 蓝牙页：播放/暂停图标（b1.picc）+ 播放进度（j0.val） */
void ui_push_bt_play_state(void);
/* 蓝牙页：曲目/歌词文本；传 NULL 表示"用上次缓存的内容重发"（切页时用这个） */
void ui_push_bt_track_text(const char *title, const char *artist);
/* 灯带页：三色滚动条当前值（h0/h1/h2 = R/G/B，0-255） */
void ui_push_led_state(void);
/* 灯带页：灯带开关控件状态（n0.val：1=开 2=关） */
void ui_push_led_switch(void);
/* 蓝牙页：蓝牙开关控件状态（n0.val：1=开 2=关） */
void ui_push_bt_switch(void);
/* 蓝牙页：音量滑动条（h0.val 0-127）+ 音量开关（n2.val：1=静音 2=有声） */
void ui_push_bt_volume(void);
/* 蓝牙页：只同步音量开关 n2.val（由静音状态推导） */
void ui_push_bt_mute_switch(void);
/**
 * @brief 静音状态变化时同步屏幕 n2 开关（音量 0 = 静音 = n2 显示"开"）
 * @note  音量来源有三处（串口屏音量条 / btsil 静音 / btnoi 开声），统一在这里收敛
 */
void ui_sync_mute_switch(void);

#ifdef __cplusplus
}
#endif
