/**
 * @file ui.c
 * @brief 串口屏（陶晶驰 TJC）交互层实现：页面状态机 + 命令分发 + 切页状态补发
 *
 * =====================================================================
 * 一、屏幕 → 音箱（rx_task 断帧后投队列，本文件分发）
 * ---------------------------------------------------------------------
 *   · 切页发回**页面名**：f1 / main / leds / set / bt / health / sd
 *        → ui_enter_page() → ui_refresh_page() 补发该页状态
 *   · 开关控件（两个页面同名 n0，靠当前页面区分；屏幕侧已二值化 1=开/0=关）：
 *        leds 页：n0.val=1 → 开灯带 ；n0.val=0 → 关灯带
 *        bt   页：n0.val=1 → 开蓝牙 ；n0.val=0 → 关蓝牙
 *        （同时兼容旧命令串 ledon/ledoff 与 bton/btoff；旧值 2 也容错认作"关"）
 *   · 滑动条（两个页面同名 h0，靠当前页面区分）：
 *        leds 页：h0/h1/h2.val=0-255 → R/G/B（同时兼容旧串 "R255"/"G128"/"B064"）
 *        bt   页：h0.val=0-127       → 本机音量
 *   · 音量开关：n2.val=1 → 静音 ；n2.val=0 → 开启声音（同时兼容 btsil/btnoi）
 *   · 播放/暂停：btpla / btpau（均通过 AVRCP 与手机同步）
 *   · 上下曲：btne / btla ；灯效：led1（流动彩虹）/ led2（柔和呼吸灯）
 *   · 陶晶驰原生返回码：0x65 控件事件、0x66 页面 ID（sendme 回复）
 *
 * =====================================================================
 * 二、音箱 → 屏幕（uart_send / uart_send_text，自动补帧尾 FF FF FF）
 * ---------------------------------------------------------------------
 *   · 所有页（f1 除外）：电量        ba.val=<0-100>
 *   · leds：开关 n0.val=1(开)/0(关) + 开关切图 b0.picc=14(开)/18(关) 、
 *           三色条 h0/h1/h2.val=0-255（每次开灯都补发一次）
 *   · bt  ：开关 n0.val=1(开)/0(关) + 开关切图 b0.picc=7(开)/6(关) 、
 *           音量条 h0.val=0-127 、音量开关 n2.val=1=静音/0=有声 、
 *           t1.txt 连接状态 、t2.txt 曲名 、t5.txt 歌手 、b1.picc 播放图标 、j0.val 进度
 *   · 控件名在各页是**页面私有**的（n0/b0 两页都叫这个名字），所以推送前都会校验当前页面，
 *     避免把 h0（leds 页=R，bt 页=音量）发到错误的页面上去。
 * =====================================================================
 */
#include "ui.h"
#include "uart.h"
#include "led_strip.h"
#include "bt_a2dp.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "audio_vol.h"
#include "health.h"     /* 健康数据：health 页刷新、$GET,DATA 请求 / $END 停止、睡眠锁定解锁 */
#include "sd.h"
#include "power.h"      /* 整机休眠：有效指令计活动 + 唤醒后等外设恢复完成 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "UI";

/* 一条"有效指令"开始执行的统一入口：
 *   ① 记一次活动（整机休眠倒计时的"串口屏有效指令"一路；**未识别的命令不算**）；
 *   ② 若刚从整机休眠中醒来（外设还在恢复），先等恢复完成再执行本条命令 ——
 *      否则 bton/btpla 会作用在还没使能的蓝牙协议栈上（bt_a2dp_work() 里的
 *      assert(esp_avrc_ct_init() == ESP_OK) 会直接把芯片复位）。
 * 只在"确认识别成功"的分支里调用，未知命令/粘连拆分失败/错误帧都不走这里。 */
#define UI_VALID_CMD()   do {                               \
        power_note_activity(POWER_SRC_SCREEN);              \
        power_wait_ready(POWER_WAIT_READY_MS);              \
    } while (0)

/* ============================ 模块状态 ============================ */
static ui_page_t s_page = UI_PAGE_UNKNOWN;      /* 屏幕当前页面；UI_PAGE_SLEEP = 已休眠 */
static ui_page_t s_page_awake = UI_PAGE_UNKNOWN;/* 休眠前所在的页面（唤醒时恢复，便于立刻正确解析命令） */

/* 电量：本阶段不初始化 ADC、也不做 ADC 采集，固定发送 60（见 ui_set_battery 注释） */
static uint8_t   s_battery = 60;
static bool      s_battery_valid = true;

/* 曲目/歌词文本缓存：屏幕切页后歌词控件会复位，需要用它重发一次 */
static char s_title[80] = "";
static char s_artist[80] = "";

/* 屏幕休眠判定：收到 exit 后等待新页面名，超时即判定休眠 */
static bool       s_exit_pending = false;
static TickType_t s_exit_tick    = 0;

/* 静音状态（由本机音量是否为 0 推导）：用于同步屏幕 n2 音量开关 */
static bool s_mute_state_known = false;
static bool s_mute_state = false;

/* 蓝牙开关防抖：1 秒内的重复 bton/btoff 直接忽略（按串口屏要求） */
#define BT_SWITCH_DEBOUNCE_MS   1000
static bool       s_bt_sw_first = true;
static TickType_t s_bt_sw_tick  = 0;

/* 陶晶驰开关控件取值：屏幕侧已统一为**二值化** —— 1=开，0=关。
 * 兼容：旧编码里"关"曾用 2，接收时把 2 也认作"关"（见 ui_sw_is_off()），
 *       避免屏幕工程里个别遗留的 2 让命令失效。 */
#define TJC_SW_ON           1
#define TJC_SW_OFF          0
#define TJC_SW_OFF_LEGACY   2

/* 页面名（屏幕切页时发回）→ 页面枚举 */
static const struct {
    const char *name;
    ui_page_t   page;
} s_page_names[] = {
    {"f1",     UI_PAGE_F1},
    {"main",   UI_PAGE_MAIN},
    {"leds",   UI_PAGE_LEDS},
    {"set",    UI_PAGE_SET},
    {"bt",     UI_PAGE_BT},
    {"health", UI_PAGE_HEALTH},
    {"sd",     UI_PAGE_SD},
};

/* ============================ 内部函数声明 ============================ */
typedef void (*CommandHandler)(void);

static void ui_task(void *arg);
static void ui_enter_page(ui_page_t page);
static void ui_on_page_exit(void);
static bool ui_is_exit_cmd(const char *cmd);
static void ui_poll_sleep(void);
static void ui_enter_sleep(void);
static void ui_wake_from_sleep(void);
static bool ui_page_accepts_battery(void);
static void ui_on_tjc_key(uint8_t page_id, uint8_t comp_id, uint8_t event);
static void ui_on_tjc_page_id(uint8_t page_id);
static void ui_on_tjc_event(uint8_t code);
static bool ui_handle_token(const char *tok);
static bool ui_resync(const char *s);
static bool ui_sw_is_off(int val);

/* 命令处理 */
static void cmd_led_on(void);
static void cmd_led_off(void);
static void cmd_led1(void);
static void cmd_led2(void);
static void cmd_led_effect(uint8_t mode);
static void cmd_bt_on(void);
static void cmd_bt_off(void);
static void cmd_bt_play(void);
static void cmd_bt_pause(void);
static void cmd_bt_mute(void);
static void cmd_bt_unmute(void);
static void cmd_bt_set_volume(int vol);
static bool ui_attr_int(const char *cmd, const char *attr, int *out);
static bool ui_bt_switch_allowed(void);

/* ============================ 命令表 ============================ */
//定义命令与处理函数的映射关系
typedef struct {
    const char* command;       // 命令字符串
    CommandHandler handler;    // 对应的处理函数
    size_t cmd_len;            // 命令长度（提前计算，优化效率）
} CommandMap;

//初始化指令映射表
static const CommandMap commandMap[] = {
    /* ---- 蓝牙（开/关是"调度级"函数，初始化 bt_init() 已在上电执行） ---- */
    {"bton",   cmd_bt_on,       strlen("bton")},    // 蓝牙开（1 秒防抖）
    {"btoff",  cmd_bt_off,      strlen("btoff")},   // 蓝牙关（1 秒防抖）
    {"btpau",  cmd_bt_pause,    strlen("btpau")},   // 暂停音乐：AVRCP 同步 + 关功放（不挂起任务）
    {"btpla",  cmd_bt_play,     strlen("btpla")},   // 播放音乐：AVRCP 同步 + 开功放
    {"btne",   bt_ne,           strlen("btne")},    // 下一首（AVRCP FORWARD）
    {"btla",   bt_la,           strlen("btla")},    // 上一首（AVRCP BACKWARD）
    {"btsil",  cmd_bt_mute,     strlen("btsil")},   // 静音（音量开关 n2 同步为"开"）
    {"btnoi",  cmd_bt_unmute,   strlen("btnoi")},   // 开启声音（n2 同步为"关"）

    /* ---- SD 卡（本阶段不开发，仅保留命令入口） ---- */
    {"sdon",   sd_init,         strlen("sdon")},
    {"sdoff",  sd_deinit,       strlen("sdoff")},
    {"sdpau",  sd_pau,          strlen("sdpau")},
    {"sdpla",  sd_pla,          strlen("sdpla")},
    {"sdne",   sd_ne,           strlen("sdne")},
    {"sdla",   sd_la,           strlen("sdla")},

    /* ---- 灯带（开/关只走 leds_on()/leds_off()，绝不调用 leds_init()） ---- */
    {"ledon",  cmd_led_on,      strlen("ledon")},
    {"ledoff", cmd_led_off,     strlen("ledoff")},
    {"ledreon",cmd_led_on,      strlen("ledreon")}, // 旧命令名兼容
    {"led1",   cmd_led1,        strlen("led1")},    // 灯效1：流动彩虹（再按一次退出灯效）
    {"led2",   cmd_led2,        strlen("led2")},    // 灯效2：柔和呼吸灯（再按一次退出灯效）
};

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

/* =====================================================================
 * 命令分发
 * ===================================================================== */
void ui_process_command(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') {
        return;
    }

    /* 兼容历史处理函数：led_block() / bt_vo() 直接读全局 ui_res */
    strncpy(ui_res, cmd, UI_CMD_MAX - 1);
    ui_res[UI_CMD_MAX - 1] = '\0';

    /* 统一的"解析后指令"日志：串口屏发来的每一条命令都在这里打印一行
     *（页面名 / exit / n0.val=1 / h0.val=50 / R255 / ledon … 全走这里）。 */
    ESP_LOGI(TAG, "解析指令: \"%s\"", ui_res);

    /* 0) 休眠状态：收到 exit 之外的任何命令都说明屏幕被触摸唤醒了 */
    if (s_page == UI_PAGE_SLEEP) {
        if (ui_is_exit_cmd(ui_res)) {
            return;                 /* 休眠中重复的 exit：保持休眠 */
        }
        ui_wake_from_sleep();
    }

    /* 1) 整条命令直接匹配 */
    if (ui_handle_token(ui_res)) {
        return;
    }

    /* 2) 整条不认识：屏幕错误帧密集时串口上没有 30ms 空闲，多条 prints 文本会粘成
     *    一条（例："ledsledon"）：按已知词表切开逐条执行，避免被"掩盖"后无响应。 */
    if (ui_resync(ui_res)) {
        return;
    }

    /* 3) 实在认不出来：打印出来便于核对屏幕工程配置 */
    ESP_LOGW(TAG, "未知命令: \"%s\"", ui_res);
}

/**
 * @brief 匹配并执行**一条**命令（进入前 ui_res 会被设置为该条）
 * @return true=已识别并执行；false=不认识
 */
static bool ui_handle_token(const char *tok)
{
    int val = 0;

    if (tok == NULL || tok[0] == '\0') {
        return false;
    }

    /* 保证依赖全局 ui_res 的历史处理函数（led_block/bt_vo）拿到的是本条命令 */
    if (tok != ui_res) {        /* 同一指针时无需拷贝，也避免 strncpy 自重叠 */
        strncpy(ui_res, tok, UI_CMD_MAX - 1);
        ui_res[UI_CMD_MAX - 1] = '\0';
    }

    /* a) 页面名：屏幕每次切页都会发回页面名 → 记录页面并补发该页状态 */
    for (size_t i = 0; i < ARRAY_SIZE(s_page_names); i++) {
        if (strcmp(ui_res, s_page_names[i].name) == 0) {
            UI_VALID_CMD();
            ui_enter_page(s_page_names[i].page);
            return true;
        }
    }

    /* b) exit：屏幕"页面离开事件"——正常切页时随后就到页面名，否则判定休眠 */
    if (ui_is_exit_cmd(ui_res)) {
        UI_VALID_CMD();
        ui_on_page_exit();
        return true;
    }

    /* c) 陶晶驰属性回传形式（"控件.属性=数值"），按当前页面判定含义 */
    if (ui_attr_int(ui_res, "n0.val", &val)) {
        UI_VALID_CMD();
        /* leds 页与 bt 页的开关控件同名 n0，只能用当前页面区分。
         * 屏幕侧开关已是二值：1=开，0=关。 */
        if (s_page == UI_PAGE_LEDS) {
            if (val == TJC_SW_ON)        cmd_led_on();
            else if (ui_sw_is_off(val))  cmd_led_off();
        } else if (s_page == UI_PAGE_BT) {
            if (val == TJC_SW_ON)        cmd_bt_on();
            else if (ui_sw_is_off(val))  cmd_bt_off();
        } else {
            ESP_LOGW(TAG, "收到 n0.val=%d，但当前页面未知（%s）："
                          "无法判断是灯带开关还是蓝牙开关，已忽略",
                     val, ui_page_name(s_page));
        }
        return true;
    }
    if (ui_attr_int(ui_res, "n2.val", &val)) {
        UI_VALID_CMD();
        /* 音量开关：1=开=静音，0=关=开启声音 */
        if (val == TJC_SW_ON)        cmd_bt_mute();
        else if (ui_sw_is_off(val))  cmd_bt_unmute();
        return true;
    }
    if (ui_attr_int(ui_res, "h0.val", &val)) {
        UI_VALID_CMD();
        if (s_page == UI_PAGE_BT) {
            cmd_bt_set_volume(val);                     /* bt  页 h0 = 音量条 0-127 */
        } else if (s_page == UI_PAGE_LEDS) {
            leds_set_color(0, (uint32_t)val);           /* leds 页 h0 = R 0-255 */
        } else {
            ESP_LOGW(TAG, "收到 h0.val=%d，但当前页面未知（%s）："
                          "无法判断是音量还是 R 分量，已忽略",
                     val, ui_page_name(s_page));
        }
        return true;
    }
    if (ui_attr_int(ui_res, "h1.val", &val)) {
        UI_VALID_CMD();
        if (s_page == UI_PAGE_LEDS) {
            leds_set_color(1, (uint32_t)val);           /* leds 页 h1 = G 0-255 */
        }
        return true;
    }
    if (ui_attr_int(ui_res, "h2.val", &val)) {
        UI_VALID_CMD();
        if (s_page == UI_PAGE_LEDS) {
            leds_set_color(2, (uint32_t)val);           /* leds 页 h2 = B 0-255 */
        }
        return true;
    }

    /* d) 旧命令串：带参数的前缀型命令 */
    if (ui_res[0] == 'R' || ui_res[0] == 'G' || ui_res[0] == 'B') {
        UI_VALID_CMD();
        led_block();        // 三色滚动条："R255" / "G128" / "B064"
        return true;
    }
    if (ui_res[0] == 'V') {
        UI_VALID_CMD();
        bt_vo();            // 音量条：旧串 "V0" ~ "V127"
        ui_sync_mute_switch();
        return true;
    }

    /* e) 定长命令表：先比长度（快速排除），再比内容 */
    size_t len = strlen(ui_res);
    for (size_t i = 0; i < ARRAY_SIZE(commandMap); i++) {
        if (len == commandMap[i].cmd_len && strcmp(ui_res, commandMap[i].command) == 0) {
            ESP_LOGD(TAG, "执行命令: %s", ui_res);
            UI_VALID_CMD();
            commandMap[i].handler();
            return true;
        }
    }

    return false;
}

/* 取"数字串"长度（属性值/前缀参数用） */
static size_t digit_run_len(const char *s)
{
    size_t n = 0;

    if (s[0] == '-') {
        n++;                    /* 允许负号（非法值由各处理函数限幅） */
    }
    while (s[n] >= '0' && s[n] <= '9') {
        n++;
    }
    return n;
}

/**
 * @brief 把"粘成一条"的字符串按已知命令词表切开逐条执行
 *
 * 背景：屏幕的错误帧（如 04 FF FF FF）密集时，串口上长时间没有 30ms 空闲，
 * 多条不带帧尾的 prints 文本会粘在一起（例："ledsledon"、"ledonbton"），
 * 整条匹配必然失败，表现就是"命令被掩盖、操作没响应"。
 *
 * 规则：从左到右最长匹配——页面名 / exit / 命令表 / 属性形式(n?.val=/h?.val=) /
 *       前缀形式(R|G|B|V + 数字)；认不出的字符跳过 1 字节继续找。
 * @return true=至少识别并执行了一条
 */
static bool ui_resync(const char *s)
{
    static const char *const attr_keys[] = {"n0.val=", "n2.val=", "h0.val=", "h1.val=", "h2.val="};
    size_t n = strlen(s);
    size_t i = 0;
    bool any = false;
    char tok[UI_CMD_MAX];

    while (i < n) {
        size_t best = 0;        /* 本位置匹配到的最长命令长度 */

        /* 1) 属性形式：属性名 + 数字 */
        for (size_t k = 0; k < ARRAY_SIZE(attr_keys); k++) {
            size_t kl = strlen(attr_keys[k]);
            if (strncmp(&s[i], attr_keys[k], kl) == 0) {
                size_t dl = digit_run_len(&s[i + kl]);
                if (dl > 0 && (kl + dl) > best) {
                    best = kl + dl;
                }
            }
        }

        /* 2) 词表命令（页面名 + exit + 命令表），取最长匹配 */
        for (size_t k = 0; k < ARRAY_SIZE(s_page_names); k++) {
            size_t kl = strlen(s_page_names[k].name);
            if (kl > best && strncmp(&s[i], s_page_names[k].name, kl) == 0) {
                best = kl;
            }
        }
        {
            const char *ex = "exit";
            size_t kl = 4;
            if (kl > best && strncasecmp(&s[i], ex, kl) == 0) {
                best = kl;
            }
        }
        for (size_t k = 0; k < ARRAY_SIZE(commandMap); k++) {
            size_t kl = commandMap[k].cmd_len;
            if (kl > best && strncmp(&s[i], commandMap[k].command, kl) == 0) {
                best = kl;
            }
        }

        /* 3) 前缀形式：R/G/B/V + 数字 */
        if ((s[i] == 'R' || s[i] == 'G' || s[i] == 'B' || s[i] == 'V')) {
            size_t dl = digit_run_len(&s[i + 1]);
            if (dl > 0 && (1 + dl) > best) {
                best = 1 + dl;
            }
        }

        if (best == 0 || best >= sizeof(tok)) {
            i++;                /* 认不出的字符：跳过继续找 */
            continue;
        }

        memcpy(tok, &s[i], best);
        tok[best] = '\0';
        ESP_LOGI(TAG, "解析指令(粘连拆分): \"%s\"", tok);
        ui_handle_token(tok);
        any = true;
        i += best;
    }

    return any;
}

/* 屏幕"离开页面 / 休眠退出页面"的提示串（大小写不敏感，容错用） */
static bool ui_is_exit_cmd(const char *cmd)
{
    return (strcasecmp(cmd, "exit") == 0);
}

/* 收到的开关值是否代表"关"：屏幕现为二值 0=关，同时兼容旧编码的 2 */
static bool ui_sw_is_off(int val)
{
    return (val == TJC_SW_OFF || val == TJC_SW_OFF_LEGACY);
}

/* 解析 "属性=数值"（例如 h0.val=50 → attr="h0.val"） */
static bool ui_attr_int(const char *cmd, const char *attr, int *out)
{
    size_t n = strlen(attr);

    if (strncmp(cmd, attr, n) != 0 || cmd[n] != '=') {
        return false;
    }
    if (cmd[n + 1] == '\0') {
        return false;
    }
    if (out) {
        *out = atoi(cmd + n + 1);
    }
    return true;
}

/* =====================================================================
 * 页面状态机
 * ===================================================================== */
static void ui_enter_page(ui_page_t page)
{
    if (s_exit_pending) {
        /* exit 之后紧跟着页面名 = 正常切页（不是休眠） */
        s_exit_pending = false;
        ESP_LOGD(TAG, "exit 后收到页面名 → 属于正常切页");
    }
    s_page = page;
    s_page_awake = page;
    ESP_LOGI(TAG, "屏幕切到页面 [%s] → 补发该页状态", ui_page_name(page));
    /* 同一页面重复收到页面名也要补发（屏幕重进该页时控件同样会复位） */
    ui_refresh_page(page);
}

/**
 * 屏幕发来 exit（"页面离开事件"：切页前 / 休眠超时退出页面时都会发）。
 * 此时还不知道是哪种情况：先挂起"待确认"，等新页面名或超时。
 */
static void ui_on_page_exit(void)
{
    /* 离开 health 页要通知手表停止传感器上报（exit 是切页与休眠退出都会发的），
     * 注意：此时 s_page 仍是"正在离开的那一页" */
    if (s_page == UI_PAGE_HEALTH) {
        health_stop_stream();
    }

    s_exit_pending = true;
    s_exit_tick = xTaskGetTickCount();
    ESP_LOGI(TAG, "屏幕发来 exit（离开当前页）→ 等待新页面名，%d ms 内没有则判定休眠",
             UI_EXIT_GRACE_MS);
}

/**
 * 休眠轮询：由 ui_task 在"队列空闲"时调用。
 * exit 之后超过 UI_EXIT_GRACE_MS 仍没有新页面名 → 判定屏幕已休眠。
 */
static void ui_poll_sleep(void)
{
    if (!s_exit_pending || s_page == UI_PAGE_SLEEP) {
        return;
    }
    if (pdTICKS_TO_MS(xTaskGetTickCount() - s_exit_tick) < UI_EXIT_GRACE_MS) {
        return;
    }
    ESP_LOGW(TAG, "exit 后 %d ms 未收到页面名 → 判定屏幕已休眠", UI_EXIT_GRACE_MS);
    ui_enter_sleep();
}

static void ui_enter_sleep(void)
{
    if (s_page == UI_PAGE_SLEEP) {
        return;                         /* 已在休眠态，避免重复打印 */
    }

    /* 屏幕自动休眠（0x86）可能没有 exit，这里兜底：仍在 health 页就先停掉手表上报 */
    if (s_page == UI_PAGE_HEALTH) {
        health_stop_stream();
    }

    s_exit_pending = false;
    s_page = UI_PAGE_SLEEP;             /* s_page_awake 保留：唤醒时恢复用 */
    ESP_LOGW(TAG, "判定串口屏已休眠（暂停向屏幕推送页面状态）");

    /* 若以后需要在屏幕休眠时做联动（例如关灯带 / 关功放省电），在这里加一处即可 */
}

/**
 * 屏幕被触摸唤醒：重新收到命令即视为唤醒。
 * 页面状态先恢复到休眠前那一页（这样紧接着的 h0.val 等命令能立刻正确解析），
 * 同时发 sendme 让屏幕回报真实页面；配有页面ID对照表时会自动补发该页状态。
 */
static void ui_wake_from_sleep(void)
{
    if (s_page != UI_PAGE_SLEEP) {
        return;
    }
    s_page = s_page_awake;
    ESP_LOGW(TAG, "屏幕已唤醒（恢复到页面 [%s]，并发送 sendme 查询真实页面）",
             ui_page_name(s_page));

    uart_send("sendme");
    /* 兜底：先把电量刷一次（若屏幕上报页面名/页面ID，还会再完整补发一次该页状态） */
    ui_push_battery();

    /* 唤醒后仍在 health 页：重新请求一次传感器数据（休眠时已发过 $END） */
    if (s_page == UI_PAGE_HEALTH) {
        health_request_from_watch();
    }
}

/**
 * 整机从 light sleep 醒来、外设恢复完成后调用（power.c）。
 *
 * 与"屏幕被触摸唤醒"（ui_wake_from_sleep）不同：这里是**主控**醒过一轮，
 * 期间屏幕可能自己休眠过/切过页/重启过，所以必须重新问一次页面并补发状态，
 * 否则屏幕上的开关/音量可能和音箱内部状态不一致。
 */
void ui_on_mcu_wake(void)
{
    if (s_page == UI_PAGE_SLEEP) {
        /* 屏幕自己也在休眠：此时不推送（睡了就不该打扰它），等它被触摸唤醒后
         * 走正常的 ui_wake_from_sleep() 补发。 */
        ESP_LOGI(TAG, "整机唤醒：屏幕仍处于休眠态 → 暂不推送，等屏幕被触摸唤醒");
        return;
    }

    ESP_LOGI(TAG, "整机唤醒：向屏幕重新同步（sendme + 电量 + 当前页 [%s] 状态）",
             ui_page_name(s_page));
    uart_send("sendme");
    ui_push_battery();
    ui_refresh_page(s_page);
}

/* 屏幕自动上报的状态（0x86 休眠 / 0x87 唤醒 / 0x88 启动完成） */
static void ui_on_tjc_event(uint8_t code)
{
    switch (code) {
    case 0x86:
        /* 屏幕自己报告进入睡眠：最可靠，直接采信（exit 超时判定只是兜底） */
        s_exit_pending = false;
        ui_enter_sleep();
        break;
    case 0x87:
        /* 屏幕自动唤醒 = 有人碰了屏幕 → 算用户活动（随后通常还会来页面名/控件命令）。
         * 本回调跑在 rx_task 里，这里只记活动，恢复动作交给 power 看护任务。 */
        power_note_activity(POWER_SRC_SCREEN);
        ui_wake_from_sleep();
        break;
    case 0x88:
        /* 屏幕刚上电/复位：它不会主动发页面名，靠 uart.c 已发出的 sendme 回报页面 ID */
        ESP_LOGI(TAG, "屏幕启动完成，等待页面 ID 回报（页面ID对照表见 ui_on_tjc_page_id）");
        break;
    default:
        break;
    }
}

ui_page_t ui_get_page(void)
{
    return s_page;
}

bool ui_is_screen_asleep(void)
{
    return (s_page == UI_PAGE_SLEEP);
}

const char *ui_page_name(ui_page_t page)
{
    switch (page) {
    case UI_PAGE_F1:     return "f1";
    case UI_PAGE_MAIN:   return "main";
    case UI_PAGE_LEDS:   return "leds";
    case UI_PAGE_SET:    return "set";
    case UI_PAGE_BT:     return "bt";
    case UI_PAGE_HEALTH: return "health";
    case UI_PAGE_SD:     return "sd";
    case UI_PAGE_SLEEP:  return "sleep";
    default:             return "unknown";
    }
}

void ui_query_page(void)
{
    ESP_LOGI(TAG, "向屏幕查询当前页面 ID（sendme）");
    uart_send("sendme");
}

/* =====================================================================
 * 切页状态补发（"哪些内容要补"全部集中在这里）
 * ===================================================================== */
void ui_refresh_page(ui_page_t page)
{
    switch (page) {
    case UI_PAGE_F1:
        /* f1 待机页与音箱无任何交互：不发任何数据（连电量也不发） */
        ESP_LOGD(TAG, "f1 页无交互，不发数据");
        break;

    case UI_PAGE_SLEEP:
        /* 屏幕已休眠：不推送任何数据，等唤醒后再补发 */
        ESP_LOGD(TAG, "屏幕休眠中，不推送数据");
        break;

    case UI_PAGE_MAIN:
    case UI_PAGE_SET:
    case UI_PAGE_SD:        /* sd 暂时只刷新电量，SD 功能待后续开发 */
        ui_push_battery();
        break;

    case UI_PAGE_HEALTH:
        /* health 页：电量 + 已存健康数据（t0~t3）；
         * 随后向手表请求一次最新传感器数据，回包到达后由 health 模块再刷一次 */
        ui_push_battery();
        health_ui_push();
        health_request_from_watch();
        break;

    case UI_PAGE_LEDS:
        ui_push_battery();
        ui_push_led_switch();   // n0.val：1=开 0=关（+ b0.picc=14/18）
        ui_push_led_state();    // h0/h1/h2.val：保存的 R/G/B
        break;

    case UI_PAGE_BT:
        ui_push_battery();
        ui_push_bt_switch();    // n0.val：1=开 0=关（+ b0.picc=7/6）
        ui_push_bt_volume();    // h0.val 音量条 + n2.val 音量开关
        ui_push_bt_conn_state();            // t1.txt
        ui_push_bt_track_text(NULL, NULL);  // t2.txt/t5.txt（用缓存重发）
        ui_push_bt_play_state();            // b1.picc + j0.val
        break;

    default:
        break;
    }
}

/* =====================================================================
 * 电量（f1 / 休眠 除外，其它页右上角控件 ba.val）
 * ===================================================================== */
/* 当前页面是否接收电量推送：f1 无交互、屏幕休眠时不推 */
static bool ui_page_accepts_battery(void)
{
    ui_page_t p = ui_get_page();

    return (p == UI_PAGE_UNKNOWN || p == UI_PAGE_MAIN || p == UI_PAGE_LEDS ||
            p == UI_PAGE_SET || p == UI_PAGE_BT ||
            p == UI_PAGE_HEALTH || p == UI_PAGE_SD);
}

void ui_push_battery(void)
{
    if (!ui_page_accepts_battery()) {
        return;
    }
    uart_send("ba.val=%u", (unsigned)s_battery);
}

/**
 * @brief 上报电量百分比
 *
 * 【本阶段约定】不能初始化 ADC、也不能调用 ADC 采样，所以电量固定为 60：
 *   · 开机默认值就是 60（s_battery 初值），上电后第一次切页即发 `ba.val=60`；
 *   · adc.c 里的电量转发已按你的要求注释停用，这里保留接口，
 *     等电池分压硬件（GPIO14 / ADC2_CH6）确认后再由 adc.c 调用。
 */
void ui_set_battery(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    if (s_battery_valid && percent == s_battery) {
        return;     /* 数值没变不重复发送，避免刷屏串口 */
    }
    s_battery = percent;
    s_battery_valid = true;
    ESP_LOGI(TAG, "电量更新为 %u%% → 推送到屏幕", (unsigned)s_battery);
    ui_push_battery();
}

/* =====================================================================
 * 灯带页状态推送
 * ===================================================================== */
void ui_push_led_switch(void)
{
    if (ui_get_page() != UI_PAGE_LEDS) {
        ESP_LOGD(TAG, "不在 leds 页，跳过 n0 灯带开关状态");
        return;
    }
    /* 开关控件 n0.val：1=开 0=关（屏幕侧已二值化）
     * 切图控件 b0.picc：14=开 18=关（同一页里用图片同步显示开关状态） */
    if (leds_is_on()) {
        uart_send("n0.val=%u", TJC_SW_ON);
        uart_send("b0.picc=14");
    } else {
        uart_send("n0.val=%u", TJC_SW_OFF);
        uart_send("b0.picc=18");
    }
}

void ui_push_led_state(void)
{
    uint32_t r = 0, g = 0, b = 0;

    if (ui_get_page() != UI_PAGE_LEDS) {
        ESP_LOGD(TAG, "不在 leds 页，跳过三色条状态");
        return;
    }

    leds_get_rgb(&r, &g, &b);
    /* 三色滚动条：h0/h1/h2 ← R/G/B（0-255） */
    uart_send("h0.val=%lu", r);
    uart_send("h1.val=%lu", g);
    uart_send("h2.val=%lu", b);
}

/* =====================================================================
 * 蓝牙页状态推送
 * ===================================================================== */
void ui_push_bt_switch(void)
{
    if (ui_get_page() != UI_PAGE_BT) {
        ESP_LOGD(TAG, "不在 bt 页，跳过 n0 蓝牙开关状态");
        return;
    }
    /* 开关控件 n0.val：1=开 0=关（屏幕侧已二值化）
     * 切图控件 b0.picc：7=开 6=关 */
    if (bt_a2dp_is_on()) {
        uart_send("n0.val=%u", TJC_SW_ON);
        uart_send("b0.picc=7");
    } else {
        uart_send("n0.val=%u", TJC_SW_OFF);
        uart_send("b0.picc=6");
    }
}

void ui_push_bt_mute_switch(void)
{
    if (ui_get_page() != UI_PAGE_BT) {
        ESP_LOGD(TAG, "不在 bt 页，跳过 n2 音量开关状态");
        return;
    }
    /* n2.val：1=开=静音，0=关=开启声音（屏幕侧开关已二值化） */
    uart_send("n2.val=%u", (audio_vol_get() == 0) ? TJC_SW_ON : TJC_SW_OFF);
}

void ui_push_bt_volume(void)
{
    if (ui_get_page() != UI_PAGE_BT) {
        ESP_LOGD(TAG, "不在 bt 页，跳过音量条状态");
        return;
    }
    /* 音量条 h0：0-127，与音箱内部本机音量一一对应（不做换算） */
    uart_send("h0.val=%u", (unsigned)audio_vol_get());
    ui_push_bt_mute_switch();
}

void ui_sync_mute_switch(void)
{
    bool now = (audio_vol_get() == 0);

    if (s_mute_state_known && now == s_mute_state) {
        return;     /* 静音状态没变，不重复发送 */
    }
    s_mute_state = now;
    s_mute_state_known = true;
    ui_push_bt_mute_switch();
}

void ui_push_bt_conn_state(void)
{
    /* t1.txt：蓝牙连接状态（切到蓝牙页时补发一次） */
    uart_send_text("t1.txt", bt_con_flag ? "已连接" : "未连接");
}

void ui_push_bt_play_state(void)
{
    /* b1.picc：播放/暂停图标（沿用旧代码切图号：7=播放中，6=暂停） */
    uart_send("b1.picc=%u", bt_a2d_is_playing() ? 7 : 6);
    /* j0.val：播放进度百分比 */
    uart_send("j0.val=%u", (unsigned)bt_a2d_get_progress());
}

void ui_push_bt_track_text(const char *title, const char *artist)
{
    if (title && title[0]) {
        strncpy(s_title, title, sizeof(s_title) - 1);
        s_title[sizeof(s_title) - 1] = '\0';
    }
    if (artist && artist[0]) {
        strncpy(s_artist, artist, sizeof(s_artist) - 1);
        s_artist[sizeof(s_artist) - 1] = '\0';
    }

    /* 文本先缓存；只有屏幕停在蓝牙页时才真正发送（歌词区是页面私有控件） */
    if (ui_get_page() != UI_PAGE_BT) {
        ESP_LOGD(TAG, "不在 bt 页，曲目/歌词已缓存，等切页后补发");
        return;
    }

    /* t2.txt = 曲名（AVRCP 属性 0x1 TITLE），t5.txt = 歌手（0x2 ARTIST） */
    uart_send_text("t2.txt", s_title);
    uart_send_text("t5.txt", s_artist);
}

/* =====================================================================
 * 命令处理：灯带
 * ===================================================================== */
static void cmd_led_on(void)
{
    leds_on();                  /* 点亮并恢复保存的 RGB（上电首次 = 50,50,50） */
    ui_push_led_switch();       /* n0.val=1 */
    ui_push_led_state();        /* 每次打开都要把 RGB 数值发给屏幕刷新三色条 */
    ESP_LOGI(TAG, "串口屏：灯带已开");
}

static void cmd_led_off(void)
{
    leds_off();                 /* 只熄灭，保存的颜色保留 */
    ui_push_led_switch();       /* n0.val=2 */
    ESP_LOGI(TAG, "串口屏：灯带已关");
}

static void cmd_led1(void)
{
    cmd_led_effect(LED_MODE_RAINBOW);
}

static void cmd_led2(void)
{
    cmd_led_effect(LED_MODE_BREATH);
}

/**
 * 灯效命令处理：
 *   · 灯带开关未打开 → 忽略（打印警告）
 *   · 收到与当前相同的灯效 → 退出灯效，恢复保存的静态 RGB（灯带保持亮）并把
 *     h0/h1/h2 发给屏幕，让三色条显示恢复后的颜色
 *   · 收到不同灯效 → 切换
 */
static void cmd_led_effect(uint8_t mode)
{
    const char *name = (mode == LED_MODE_BREATH) ? "led2 柔和呼吸灯" : "led1 流动彩虹";

    if (!leds_is_on()) {
        ESP_LOGW(TAG, "灯带开关未打开，忽略灯效命令（%s）", name);
        return;
    }

    if (leds_get_mode() == mode) {
        leds_set_static();
        ui_push_led_state();
        ESP_LOGI(TAG, "再次收到 %s → 退出灯效，恢复保存的颜色", name);
        return;
    }

    leds_set_mode(mode);
    ESP_LOGI(TAG, "灯效切换为 %s", name);
}

/* =====================================================================
 * 命令处理：蓝牙
 * ===================================================================== */
/* 蓝牙开关防抖：1 秒内的重复命令直接忽略（只对蓝牙开关生效） */
static bool ui_bt_switch_allowed(void)
{
    TickType_t now = xTaskGetTickCount();

    if (!s_bt_sw_first) {
        uint32_t dt = (uint32_t)pdTICKS_TO_MS(now - s_bt_sw_tick);
        if (dt < BT_SWITCH_DEBOUNCE_MS) {
            return false;
        }
    }
    s_bt_sw_first = false;
    s_bt_sw_tick = now;
    return true;
}

static void cmd_bt_on(void)
{
    if (!ui_bt_switch_allowed()) {
        ESP_LOGW(TAG, "蓝牙开关命令过于频繁（< %d ms），忽略本次开启", BT_SWITCH_DEBOUNCE_MS);
        return;
    }
    bt_a2dp_work();             /* 调度级"开"：注册 A2DP/AVRCP + 可发现 */
    ui_push_bt_switch();        /* n0.val=1 */
    ESP_LOGI(TAG, "串口屏：蓝牙已开");
}

static void cmd_bt_off(void)
{
    if (!ui_bt_switch_allowed()) {
        ESP_LOGW(TAG, "蓝牙开关命令过于频繁（< %d ms），忽略本次关闭", BT_SWITCH_DEBOUNCE_MS);
        return;
    }
    bt_app_shutdown();          /* 调度级"关"：反初始化 A2DP/AVRCP */
    ui_push_bt_switch();        /* n0.val=2 */
    ESP_LOGI(TAG, "串口屏：蓝牙已关");
}

static void cmd_bt_play(void)
{
    bt_work();                  /* AVRCP 播放 + 开功放（与手机同步） */
    health_on_user_play();      /* 用户主动播放 → 解除手表睡眠(SLEEP=2)锁定 */
}

static void cmd_bt_pause(void)
{
    bt_sleep();                 /* AVRCP 暂停 + 关功放（与手机同步） */
}

static void cmd_bt_mute(void)
{
    bt_sli();                   /* 记忆当前音量并置 0 */
    ui_push_bt_volume();        /* 音量条归 0 + n2 同步为"开(静音)" */
    ESP_LOGI(TAG, "串口屏：静音");
}

static void cmd_bt_unmute(void)
{
    bt_noi();                   /* 恢复到静音前的音量 */
    ui_push_bt_volume();
    ESP_LOGI(TAG, "串口屏：开启声音");
}

/* 蓝牙页音量条：0-127 直接对应本机音量 */
static void cmd_bt_set_volume(int vol)
{
    if (vol < 0) {
        vol = 0;
    }
    if (vol > AUDIO_VOL_MAX) {
        vol = AUDIO_VOL_MAX;
    }
    audio_vol_set((uint8_t)vol);
    /* 音量被拖到 0 时，屏幕上的 n2 音量开关要同步成"开(静音)"；
     * 不把 h0 原样回发，避免和用户正在拖动的滑动条互相抢状态 */
    ui_sync_mute_switch();
}

/* =====================================================================
 * 陶晶驰二进制返回码（uart.c 解析后回调到这里）
 * ===================================================================== */
static void ui_on_tjc_key(uint8_t page_id, uint8_t comp_id, uint8_t event)
{
    /* 屏幕勾选"发送键值"后，控件按下/弹起会返回 0x65 <页ID> <控件ID> <事件>。
     * 本工程当前用"文本命令/属性回传"实现按钮交互，这里先把事件打印出来，
     * 便于后续需要时改成"按页ID+控件ID"驱动（不受命令粘连影响）。
     * 注意：本回调跑在 uart.c 的最高优先级 rx_task 里，只记活动、绝不做重活。 */
    ESP_LOGI(TAG, "屏幕控件事件: 页ID=%u 控件ID=%u %s（尚未映射到功能）",
             page_id, comp_id, event == 0x01 ? "按下" : "弹起");
    power_note_activity(POWER_SRC_SCREEN);      /* 手指真的按了屏幕 → 算用户活动 */
}

static void ui_on_tjc_page_id(uint8_t page_id)
{
    /* sendme 的回复：页面 ID 由上位机的页面顺序决定。
     * 只有把下表的"页面 ID → 页面名"填好，才能在"屏幕先上电、音箱后上电（或音箱复位）"
     * 这种屏幕不会再主动发页面名的情况下，正确补发当前页状态。
     * 填写示例（按上位机左侧页面列表顺序，从 0 开始）：
     *     {"f1", "main", "leds", "set", "bt", "health", "sd"} */
    static const char *const s_page_id_names[] = {
        /* 0 */ NULL, /* 1 */ NULL, /* 2 */ NULL, /* 3 */ NULL,
        /* 4 */ NULL, /* 5 */ NULL, /* 6 */ NULL,
    };

    const char *name = (page_id < ARRAY_SIZE(s_page_id_names)) ? s_page_id_names[page_id] : NULL;
    if (name == NULL) {
        ESP_LOGW(TAG, "屏幕当前页面 ID=%u，但[页面ID到页面名]对照表尚未配置"
                      "（见 docs/串口屏对接遗漏清单.md）", page_id);
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(s_page_names); i++) {
        if (strcmp(name, s_page_names[i].name) == 0) {
            ESP_LOGI(TAG, "屏幕当前页面: %s（ID=%u）", name, page_id);
            ui_enter_page(s_page_names[i].page);    /* 内部会清掉 exit 待确认标记 */
            return;
        }
    }
    ESP_LOGW(TAG, "页面 ID=%u 对应名称 \"%s\" 不在已知页面名列表中", page_id, name);
}

/* =====================================================================
 * 启动
 * ===================================================================== */
static void ui_task(void *arg)
{
    char cmd[UI_CMD_MAX];

    while (1) {
        /* 100ms 超时：收到命令就分发；空闲时轮询"exit 后是否已进入休眠" */
        if (uart_recv_ui_cmd(cmd, sizeof(cmd), 100)) {
            ui_process_command(cmd);
        } else {
            ui_poll_sleep();
        }
    }
}

void ui_init(void)
{
    /* 接收陶晶驰原生二进制返回码（0x65 控件事件 / 0x66 页面ID / 0x86~0x88 屏幕状态） */
    uart_set_tjc_binary_cbs(ui_on_tjc_key, ui_on_tjc_page_id);
    uart_set_tjc_event_cb(ui_on_tjc_event);

    xTaskCreate(ui_task, "ui_task", 1024 * 3, NULL, 5, NULL);
    //          入口函数 函数名称    栈深      参数  优先级 句柄
    ESP_LOGI(TAG, "串口屏交互层已启动（等待屏幕发来页面名/控件命令）");
}
