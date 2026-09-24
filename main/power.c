/**
 * @file power.c
 * @brief 整机休眠实现（详见 power.h 顶部说明）
 *
 * 状态机（全部状态由本文件的看护任务驱动）：
 *
 *   AWAKE ──四类活动源静默 POWER_IDLE_TIMEOUT_MS──> GOING_SLEEP
 *     GOING_SLEEP：关功放/灯带/双模蓝牙 → 复查"关断期间是否又来了有效指令" →
 *                  配唤醒源 → 等串口空闲 → esp_light_sleep_start()
 *     ASLEEP    ：芯片在 light sleep 里（唯一唤醒源 = 串口屏 RX 低电平）
 *     WAKE_PENDING：已醒、外设仍关着；等一条有效指令（POWER_WAKE_GRACE_MS）
 *     RESUMING  ：按休眠前状态恢复灯带/功放/蓝牙 → AWAKE（下一轮倒计时）
 *
 * 关键取舍（都是实测/文档核对过的，不要随手改）：
 *   1. 唤醒源用 light-sleep 专用的 GPIO 电平唤醒，不用 ext1、不用 UART 唤醒 —— 原因见 power.h；
 *   2. 蓝牙用 esp_bluedroid_disable() + esp_bt_controller_disable()（**不是** deinit），
 *      因为 IDF 头文件写明控制器 init/deinit "should be called only once"，而 disable→enable
 *      是官方支持的反复循环；带着使能的蓝牙进 light sleep 会把 BT 时钟/状态搞坏；
 *   3. 进睡前必须确认串口线空闲（高电平 + 驱动缓冲为空），否则 light sleep 会因"唤醒触发
 *      已有效"立刻返回 ESP_ERR_SLEEP_REJECT，变成忙等；
 *   4. 所有关断/恢复动作都在本任务的上下文里做：ui.c 的 ui_enter_sleep()/屏幕事件回调
 *      可能跑在 uart.c 的最高优先级 rx_task 里，在那里做 RMT/蓝牙动作会丢串口字节。
 *
 * 联调步骤、可调参数、已知限制见 docs/整机休眠对接说明.md。
 */
#include "power.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "uart.h"           /* uart_recv / 丢弃残帧 / 串口屏 RX 引脚定义 */
#include "ui.h"             /* ui_on_mcu_wake()：唤醒后重新同步屏幕 */
#include "amp.h"            /* amp_set_power / amp_is_powered */
#include "led_strip.h"      /* leds_off / leds_deinit / leds_init / leds_on ... */
#include "bt_a2dp.h"        /* bt_a2dp_is_on / bt_stack_sleep / bt_stack_wake */
#include "bt_app_core.h"    /* bt_sleep（AVRCP 暂停 + 关功放） */
#include "ble_client.h"     /* bt_stack_sleep/wake 内部使用 */

static const char *TAG = "POWER";

/* ============================ 状态 ============================ */
typedef enum {
    POWER_ST_AWAKE = 0,     /* 外设全开，倒计时运行 */
    POWER_ST_GOING_SLEEP,   /* 正在关断（这段时间内新的有效指令会取消休眠） */
    POWER_ST_ASLEEP,        /* 已在 light sleep（卡在 esp_light_sleep_start 里） */
    POWER_ST_WAKE_PENDING,  /* 已醒、外设仍关，等有效指令 */
    POWER_ST_RESUMING,      /* 正在恢复外设 */
} power_state_t;

static volatile power_state_t s_state = POWER_ST_AWAKE;
static volatile bool     s_resume_req = false;    /* 有有效指令要求恢复外设 */
static volatile TickType_t s_last[POWER_SRC_MAX]; /* 各活动源最近一次活动时刻 */
static TickType_t        s_snapshot[POWER_SRC_MAX];/* 关断前的活动快照（复查用） */
static TickType_t        s_wake_tick = 0;         /* 从 light sleep 醒来的时刻 */
static TaskHandle_t      s_task = NULL;

/* 休眠前的外设状态（light sleep 保留 RAM，醒来按它恢复） */
static bool    s_was_leds_on  = false;
static uint8_t s_was_led_mode = 0;
static bool    s_was_amp_on   = false;
static bool    s_was_bt_on    = false;

static void power_task(void *arg);
static void power_shutdown_all(void);
static void power_restore_all(const char *why);
static bool power_enter_light_sleep(void);

/* ============================ 工具 ============================ */
const char *power_state_name(void)
{
    switch (s_state) {
    case POWER_ST_AWAKE:        return "AWAKE";
    case POWER_ST_GOING_SLEEP:  return "GOING_SLEEP";
    case POWER_ST_ASLEEP:       return "ASLEEP";
    case POWER_ST_WAKE_PENDING: return "WAKE_PENDING";
    case POWER_ST_RESUMING:     return "RESUMING";
    default:                    return "?";
    }
}

bool power_is_awake(void)
{
    return (s_state == POWER_ST_AWAKE);
}

static void power_reset_timestamps(void)
{
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < POWER_SRC_MAX; i++) {
        s_last[i] = now;
    }
}

static void power_snapshot_activity(void)
{
    for (int i = 0; i < POWER_SRC_MAX; i++) {
        s_snapshot[i] = s_last[i];
    }
}

/* 关断期间是否又来了有效活动（有效指令 / 手表数据 / 音频流 / AVRCP） */
static bool power_activity_changed(void)
{
    for (int i = 0; i < POWER_SRC_MAX; i++) {
        if (s_last[i] != s_snapshot[i]) {
            return true;
        }
    }
    return false;
}

/* 最久的那个"静默时长"是否已超过阈值 */
static bool power_idle_expired(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t limit = pdMS_TO_TICKS(POWER_IDLE_TIMEOUT_MS);

    for (int i = 0; i < POWER_SRC_MAX; i++) {
        if ((TickType_t)(now - s_last[i]) < limit) {
            return false;
        }
    }
    return true;
}

/* ============================ 对外接口 ============================ */
void power_note_activity(power_src_t src)
{
    if (src >= POWER_SRC_MAX) {
        return;
    }

    s_last[src] = xTaskGetTickCount();

    /* 只要不是"全外设可用"，这条有效活动就意味着要（或已经在）恢复外设。
     * 注意 AWAKE 之外的状态都要置位：命令可能在 light sleep 刚被唤醒、
     * 看护任务还没把状态切到 WAKE_PENDING 的那一瞬间就到达（否则会漏恢复）。 */
    if (s_state != POWER_ST_AWAKE) {
        s_resume_req = true;
        if (s_task) {
            xTaskNotifyGive(s_task);    /* 立即让看护任务处理，不等 1 秒心跳 */
        }
    }
}

void power_wait_ready(uint32_t timeout_ms)
{
    TickType_t start;

    if (s_state == POWER_ST_AWAKE) {
        return;                         /* 常规路径：零开销直接返回 */
    }

    start = xTaskGetTickCount();
    while (s_state != POWER_ST_AWAKE) {
        if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - start) >= timeout_ms) {
            ESP_LOGW(TAG, "等待外设恢复超时（%u ms，当前状态=%s），继续执行本条命令",
                     (unsigned)timeout_ms, power_state_name());
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================ 关断 ============================ */
/* 等串口线空闲：高电平（无起始位）+ 驱动接收缓冲为空。
 * 不满足就不能进 sleep —— 唤醒触发已经有效时 esp_light_sleep_start() 会立刻返回，
 * 白白关掉一遍外设。 */
static esp_err_t power_wait_uart_idle(void)
{
    for (int i = 0; i < 20; i++) {
        size_t pending = 0;

        if (uart_get_buffered_data_len(UART_NUM_2, &pending) != ESP_OK) {
            pending = 0;
        }
        if (pending == 0 && gpio_get_level(UART_SCREEN_RXD_GPIO) != 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "串口屏 RX（GPIO%d）仍有数据或一直为低：本次不进休眠",
             UART_SCREEN_RXD_GPIO);
    return ESP_ERR_TIMEOUT;
}

/* 配唤醒源并进睡；返回 false = 没能进睡（已经做过恢复） */
static bool power_enter_light_sleep(void)
{
    esp_err_t err;

    /* 唤醒源：串口屏 RX 低电平。
     * GPIO16 不是 RTC IO，所以这里走 light-sleep 专用的 GPIO 电平唤醒（任意 IO 可用）。 */
    err = gpio_wakeup_enable(UART_SCREEN_RXD_GPIO, GPIO_INTR_LOW_LEVEL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置 GPIO%d 唤醒失败: %s", UART_SCREEN_RXD_GPIO, esp_err_to_name(err));
        power_restore_all("配置唤醒源失败");
        return false;
    }
    err = esp_sleep_enable_gpio_wakeup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "使能 GPIO 唤醒源失败: %s", esp_err_to_name(err));
        power_restore_all("使能唤醒源失败");
        return false;
    }

    /* GPIO16/17 挂在 VDD_SDIO 电源域上：sleep 期间必须保持该域供电，否则引脚采不到电平 */
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);

    if (power_wait_uart_idle() != ESP_OK) {
        power_restore_all("串口仍在收数据");
        return false;
    }

    /* 丢掉进睡前残留（多为上一轮响应），并让 rx_task 把正在累积的半帧也丢掉 */
    uart_rx_discard_pending();

    s_state = POWER_ST_ASLEEP;
    ESP_LOGW(TAG, "→ 进入 light sleep（唯一唤醒源：串口屏 RX GPIO%d 低电平）",
             UART_SCREEN_RXD_GPIO);

    err = esp_light_sleep_start();

    /* ---------------- 从这里开始就是"醒来之后" ---------------- */
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    gpio_wakeup_disable(UART_SCREEN_RXD_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    s_wake_tick = xTaskGetTickCount();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "light sleep 未进入（%s，cause=%d）→ 直接恢复外设",
                 esp_err_to_name(err), (int)cause);
        power_restore_all("进睡被拒（唤醒触发已有效或参数非法）");
        return false;
    }

    ESP_LOGW(TAG, "← 已从 light sleep 唤醒（cause=%d，RX 电平=%d）",
             (int)cause, gpio_get_level(UART_SCREEN_RXD_GPIO));

    /* 唤醒瞬间到达的字节已经被 UART 丢掉（sleep 期间 UART 被 IDF 停掉），
     * 清掉驱动缓冲与半帧，避免把残片当命令解析 */
    uart_rx_discard_pending();

    s_state = POWER_ST_WAKE_PENDING;
    if (s_resume_req) {
        power_restore_all("唤醒瞬间就收到了有效指令");
        return true;
    }

    ESP_LOGW(TAG, "等待有效指令：%u ms 内没有就再次进睡（无效数据不计）",
             (unsigned)POWER_WAKE_GRACE_MS);
    return true;
}

static void power_shutdown_all(void)
{
    s_state = POWER_ST_GOING_SLEEP;

    ESP_LOGW(TAG, "串口屏/手表BLE/A2DP/AVRCP 连续 %u 秒无活动 → 开始整机休眠",
             (unsigned)(POWER_IDLE_TIMEOUT_MS / 1000));

    /* 记住休眠前的外设状态（唤醒后按它恢复） */
    s_was_leds_on  = leds_is_on();
    s_was_led_mode = leds_get_mode();
    s_was_amp_on   = amp_is_powered();
    s_was_bt_on    = bt_a2dp_is_on();
    power_snapshot_activity();

    /* ① 功放：SHDN 拉低 */
    amp_set_power(false);

    /* ② 灯带：先熄灭再把 RMT 资源释放掉（led_mode 任务保留，leds_init 可重复调用） */
    leds_off();
    leds_deinit();

    /* ③ 经典蓝牙：AVRCP 暂停（对端也停）→ 反初始化 A2DP/AVRCP + 释放 I2S + 关双模蓝牙控制器 */
    bt_sleep();
    vTaskDelay(pdMS_TO_TICKS(200));         /* 给对端一点时间处理 PAUSE */
    if (bt_stack_sleep() != ESP_OK) {
        ESP_LOGE(TAG, "双模蓝牙关闭失败：取消本次休眠（绝不带着使能的蓝牙进 light sleep）");
        power_restore_all("关蓝牙失败");
        return;
    }

    /* ④ 关断期间又来了有效指令 → 放弃休眠（无效数据不理会，所以这里只看有效活动） */
    if (power_activity_changed()) {
        power_restore_all("关断期间收到有效指令");
        return;
    }

    /* ⑤ 配唤醒源 → 等串口空闲 → 进 light sleep */
    (void)power_enter_light_sleep();
}

/* ============================ 恢复 ============================ */
static void power_restore_all(const char *why)
{
    s_state = POWER_ST_RESUMING;
    ESP_LOGW(TAG, "开始恢复外设：%s", why ? why : "(未说明)");

    /* ① 双模蓝牙：控制器 + 协议栈 + BLE 主机链路总要恢复；A2DP/AVRCP 功能层按休眠前总开关 */
    esp_err_t err = bt_stack_wake(s_was_bt_on);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙恢复失败: %s（串口屏/灯带/功放不受影响，可稍后再试）",
                 esp_err_to_name(err));
    }

    /* 错峰：BT modem 的 PHY 校准是第一个大电流尖峰，等它过去再点灯带 */
    vTaskDelay(pdMS_TO_TICKS(POWER_RESTORE_STEP_MS));

    /* ② 灯带：重新创建 RMT 资源，并按休眠前的开关/灯效恢复 */
    leds_init();
    if (s_was_leds_on) {
        leds_set_mode(s_was_led_mode);
        leds_on();
    }

    /* 错峰：灯带之后再开功放（喇叭是最大的一路负载） */
    vTaskDelay(pdMS_TO_TICKS(POWER_RESTORE_STEP_MS));

    /* ③ 功放：按休眠前状态恢复（内部带防爆音软启动；若休眠前是关的就保持关）
     * 若现场电源偏弱、唤醒时仍容易把屏幕拉重启，可把这里改成不主动开功放
     * （A2DP 真正起流时 bt_app_av.c 会自己 amp_set_power(true)，功能不受影响） */
    if (s_was_amp_on) {
        amp_set_power(true);
    }

    /* ④ 屏幕重新同步：休眠期间屏幕可能自己休眠过/切过页，发 sendme 并补发当前页状态 */
    ui_on_mcu_wake();

    /* ⑤ 下一轮倒计时从"现在"开始 */
    power_reset_timestamps();
    s_resume_req = false;
    s_state = POWER_ST_AWAKE;
    ESP_LOGW(TAG, "外设已恢复（灯带=%s 灯效=%u 功放=%s A2DP=%s）→ 进入下一轮 %u 秒倒计时",
             s_was_leds_on ? "开" : "关", (unsigned)s_was_led_mode,
             s_was_amp_on ? "开" : "关", s_was_bt_on ? "开" : "关",
             (unsigned)(POWER_IDLE_TIMEOUT_MS / 1000));
}

/* ============================ 看护任务 ============================ */
static void power_task(void *arg)
{
    while (1) {
        /* 1 秒心跳；收到任务通知（有活动/需要恢复）立即返回 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POWER_TICK_MS));

        switch (s_state) {
        case POWER_ST_AWAKE:
            if (power_idle_expired()) {
                power_shutdown_all();
            }
            break;

        case POWER_ST_WAKE_PENDING:
            if (s_resume_req) {
                power_restore_all("收到有效指令");
            } else if ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - s_wake_tick)
                       >= POWER_WAKE_GRACE_MS) {
                ESP_LOGW(TAG, "唤醒后 %u ms 内没有有效指令（无效数据不计）→ 重新进睡",
                         (unsigned)POWER_WAKE_GRACE_MS);
                s_resume_req = false;
                (void)power_enter_light_sleep();    /* 外设本来就关着，只需再配唤醒源+进睡 */
            }
            break;

        default:
            /* GOING_SLEEP / ASLEEP / RESUMING：由正在执行的流程负责，不在心跳里插手 */
            break;
        }
    }
}

void power_init(void)
{
    power_reset_timestamps();

    /* 固定到 core 0：与上电路径保持一致（app_main 被固定在 core 0，bt_init() 就是在那里
     * 做的控制器/协议栈初始化），避免"控制器 enable/disable 跨核调用"这种额外变量。 */
    xTaskCreatePinnedToCore(power_task, "power_task", 1024 * 6, NULL, 4, &s_task, 0);
    //                     入口函数      函数名称       栈深       参数 优先级 句柄  核心

    ESP_LOGI(TAG, "整机休眠看护已启动：静默 %u 秒后关功放/灯带/双模蓝牙并 light sleep；"
                  "唤醒源 = 串口屏 RX GPIO%d（低电平）",
             (unsigned)(POWER_IDLE_TIMEOUT_MS / 1000), UART_SCREEN_RXD_GPIO);
}
