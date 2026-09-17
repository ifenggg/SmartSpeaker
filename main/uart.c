/**
 * @file uart.c
 * @brief 串口屏（陶晶驰 TJC）UART 链路层：发送（自动补帧尾）+ 接收断帧
 *
 * 协议要点见 uart.h 顶部说明。本文件只做"链路层"：
 *   发送：uart_send()  →  正文 + 0xFF 0xFF 0xFF（互斥保护，防止多任务交叉切断帧尾）
 *   接收：rx_task()    →  按帧尾 FF FF FF 断帧；没有帧尾时按空闲间隔断帧
 *                     →  文本命令投入队列，二进制返回码交回调（ui.c）
 */
#include "uart.h"

#include <stdio.h>
#include <stdlib.h>

#define TAG              "UART_SCREEN"

#define SEND_BUF_SIZE    256    /* 格式化缓冲区大小 */
#define RX_BUF_SIZE      512    /* 单次读取缓冲区大小 */

/* 驱动接收环形缓冲区：屏幕在错误帧密集时可能连续猛发，留足余量避免溢出丢字节
 * （溢出丢字节正是"真实命令被掩盖"的另一种可能） */
#define RX_RINGBUF_SIZE  (RX_BUF_SIZE * 2)

#define TXD_PIN          (GPIO_NUM_17)   /* ESP32 TX → 屏幕 RX */
#define RXD_PIN          (GPIO_NUM_16)   /* ESP32 RX ← 屏幕 TX */

/* ---------------------------------------------------------------------
 * 调试日志开关
 *   UART_RX_RAW_DUMP：把屏幕发来的数据**原样打印**（十六进制 + 可见字符）。
 *                     平时关闭；只有核对"屏幕到底发了哪些字节"时才临时改 1。
 *   UART_TX_LOG     ：把音箱发给屏幕的每一条指令也打印出来
 *                     （核对"切页补发了什么"时打开）。
 * 解析后的指令统一由 ui.c 打印成 "解析指令: ..."，无需依赖上面两个开关。
 * ------------------------------------------------------------------- */
#define UART_RX_RAW_DUMP    0
#define UART_TX_LOG         0       /* 1=同时打印音箱发往屏幕的每条指令（核对切页补发/开关状态值时临时打开） */
#define UART_RX_DUMP_COLS   16      /* 每行打印的字节数 */
#define UART_DUMP_MAX       256     /* 单次最多打印的字节数（防止异常时刷屏） */

/* 无帧尾时的最大累积长度（超过则丢弃最旧内容，防止内存越界与命令拼接过长） */
#define RX_ACC_SIZE      128

/* ---------------- 全局状态 ---------------- */
char ui_res[UI_CMD_MAX];                /* 当前命令（供旧处理函数直接读取） */
volatile uint8_t ui_page_id = 0xFF;     /* 串口屏当前页面 ID，0xFF=未知 */

static QueueHandle_t s_ui_cmd_queue = NULL;     /* 串口屏命令队列（元素 = UI_CMD_MAX 的字符串） */
static SemaphoreHandle_t s_tx_mutex = NULL;     /* 发送互斥锁 */
static uart_tjc_key_cb_t  s_key_cb  = NULL;     /* 0x65 控件点击回调 */
static uart_tjc_page_cb_t s_page_cb = NULL;     /* 0x66 页面 ID 回调 */
static uart_tjc_event_cb_t s_event_cb = NULL;   /* 0x86/0x87/0x88 屏幕状态回调 */

static void rx_task(void *arg);
static void uart_dump_raw_rx(const uint8_t *data, size_t len);
static void handle_frame(const uint8_t *body, size_t len);
static void handle_binary_code(const uint8_t *body, size_t len);
static bool frame_is_binary(const uint8_t *body, size_t len);
static bool frame_is_known_code_layout(const uint8_t *body, size_t len);
static const char *tjc_code_desc(uint8_t code);
static size_t frame_trim(const uint8_t *body, size_t len);
static bool frame_try_attr_synth(const uint8_t *body, size_t len, char *out, size_t out_sz);
static void post_text_cmd(const uint8_t *body, size_t len);

/* =====================================================================
 * 初始化
 * ===================================================================== */
void uart_2_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 本工程不使用 UART 事件（rx_task 用 uart_read_bytes 主动读取），
     * 因此事件队列传 NULL，避免驱动往无人读取的队列里塞事件。 */
    uart_driver_install(UART_NUM_2, RX_RINGBUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    s_tx_mutex = xSemaphoreCreateMutex();
    s_ui_cmd_queue = xQueueCreate(8, UI_CMD_MAX);

    xTaskCreate(rx_task, "uart_rx_task", 1024 * 3, NULL, configMAX_PRIORITIES - 1, NULL);
    //          入口函数     函数名称      栈深        参数  优先级             句柄

    ESP_LOGI(TAG, "串口屏链路已就绪（UART2 115200 8N1, TX=%d, RX=%d）", TXD_PIN, RXD_PIN);
}

void uart_set_tjc_binary_cbs(uart_tjc_key_cb_t key_cb, uart_tjc_page_cb_t page_cb)
{
    s_key_cb = key_cb;
    s_page_cb = page_cb;
}

void uart_set_tjc_event_cb(uart_tjc_event_cb_t event_cb)
{
    s_event_cb = event_cb;
}

/* =====================================================================
 * 发送
 * ===================================================================== */
void uart_send(const char *format, ...)
{
    /* 陶晶驰帧尾：三条 0xFF（协议规定，不是字符串结束符） */
    static const uint8_t tail[3] = {0xFF, 0xFF, 0xFF};

    char buf[SEND_BUF_SIZE];

    va_list args;                       /* 可变参数列表 */
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {     /* 超长则截断，宁可少发也不要越界 */
        n = sizeof(buf) - 1;
        ESP_LOGW(TAG, "指令被截断（>=%d 字节）", SEND_BUF_SIZE - 1);
    }

    /* 互斥：保证"正文 + 帧尾"连续发出，不被打断成两条半指令 */
    if (s_tx_mutex) {
        xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    }
    uart_write_bytes(UART_NUM_2, buf, n);
    uart_write_bytes(UART_NUM_2, tail, sizeof(tail));
    if (s_tx_mutex) {
        xSemaphoreGive(s_tx_mutex);
    }

#if UART_TX_LOG
    /* 核对"切页后补发了什么"时打开（默认关，见文件顶部 UART_TX_LOG） */
    ESP_LOGI(TAG, "TX→屏幕: \"%s\"", buf);
#endif
}

void uart_send_text(const char *obj_txt_attr, const char *text)
{
    char esc[200];
    size_t n = 0;

    if (!obj_txt_attr || !text) {
        return;
    }

    /* 陶晶驰字符串赋值：双引号内需要转义 " 与 \ ；控制字符（含换行）会让指令错乱，直接丢弃 */
    for (const char *p = text; *p && n < sizeof(esc) - 2; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') {
            esc[n++] = '\\';
            esc[n++] = (char)c;
        } else if (c >= 0x20 && c != 0x7F) {
            esc[n++] = (char)c;
        }
    }
    esc[n] = '\0';

    uart_send("%s=\"%s\"", obj_txt_attr, esc);
}

bool uart_recv_ui_cmd(char *out, uint32_t out_sz, uint32_t wait_ms)
{
    if (!s_ui_cmd_queue || !out || out_sz == 0) {
        return false;
    }
    if (xQueueReceive(s_ui_cmd_queue, out, pdMS_TO_TICKS(wait_ms)) != pdTRUE) {
        return false;
    }
    out[out_sz - 1] = '\0';
    return true;
}

/* =====================================================================
 * 接收：断帧
 * ===================================================================== */
uint8_t rx[RX_BUF_SIZE + 1];
static uint8_t s_acc[RX_ACC_SIZE];      /* 累积缓冲区 */

/* =====================================================================
 * 原始数据打印（"把串口屏发来的信息原样打印"）
 * ---------------------------------------------------------------------
 * 打印的是 uart_read_bytes() 读到的**原始字节**，不做任何过滤：
 *   · 帧尾 FF FF FF 会照原样出现（便于确认屏幕有没有补帧尾）
 *   · 不可见/高位字节打印成十六进制（例如 prints 发出的小端二进制数值）
 *   · 右侧 |...| 列是可见字符（不可见字节显示为 .）
 * 例： h0.val=50（纯文本）      → 68 30 2E 76 61 6C 3D 35 30 FF FF FF |h0.val=50...|
 *      h0.val=50（文本+二进制） → 68 30 2E 76 61 6C 3D 32 00 00 00 FF FF FF |h0.val=2....|
 * ===================================================================== */
static void uart_dump_raw_rx(const uint8_t *data, size_t len)
{
#if UART_RX_RAW_DUMP
    char hex[UART_RX_DUMP_COLS * 3 + 1];
    char txt[UART_RX_DUMP_COLS + 1];

    if (len == 0) {
        return;     /* 空闲超时返回 0，不打印 */
    }
    if (len > UART_DUMP_MAX) {
        len = UART_DUMP_MAX;
    }

    for (size_t off = 0; off < len; off += UART_RX_DUMP_COLS) {
        size_t n = len - off;
        if (n > UART_RX_DUMP_COLS) {
            n = UART_RX_DUMP_COLS;
        }
        for (size_t i = 0; i < n; i++) {
            uint8_t c = data[off + i];
            snprintf(&hex[i * 3], 4, "%02X ", c);
            txt[i] = (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
        }
        hex[n * 3] = '\0';
        txt[n]    = '\0';
        ESP_LOGI(TAG, "RX原样[%3u] %-47s |%s|", (unsigned)off, hex, txt);
    }
#else
    (void)data;
    (void)len;
#endif
}

void rx_task(void *arg)
{
    size_t acc_len = 0;

    while (1) {
        /* 超时 = 断帧空闲阈值：读不到字节说明屏幕这一批数据发完了 */
        const int length = uart_read_bytes(UART_NUM_2, rx, RX_BUF_SIZE,
                                           pdMS_TO_TICKS(UI_RX_IDLE_MS));

        if (length <= 0) {
            /* 空闲：把累计内容当作一条"无帧尾"命令（prints 风格）处理 */
            if (acc_len > 0) {
                handle_frame(s_acc, acc_len);
                acc_len = 0;
            }
            continue;
        }

        /* 原样打印本次读到的所有字节（含帧尾、不可见字符） */
        uart_dump_raw_rx(rx, (size_t)length);

        for (int i = 0; i < length; i++) {
            /* 累积溢出保护：丢掉最旧一个字节，保证不越界、命令不无限增长 */
            if (acc_len >= sizeof(s_acc)) {
                memmove(s_acc, s_acc + 1, sizeof(s_acc) - 1);
                acc_len = sizeof(s_acc) - 1;
            }
            s_acc[acc_len++] = rx[i];

            /* 帧尾检测：0xFF 0xFF 0xFF → 断帧（不含帧尾本身） */
            if (acc_len >= 3 &&
                s_acc[acc_len - 3] == 0xFF &&
                s_acc[acc_len - 2] == 0xFF &&
                s_acc[acc_len - 1] == 0xFF) {
                handle_frame(s_acc, acc_len - 3);
                acc_len = 0;
            }
        }
    }
}

/**
 * 去掉尾部的 \r \n。
 * 陶晶驰里常用 `printh 0d 0a` 给 prints 的内容补一个换行，若不先剥掉，
 * 会被下面的 frame_is_binary() 误判成"二进制返回码"，命令就丢了。
 */
static size_t frame_trim(const uint8_t *body, size_t len)
{
    while (len > 0 && (body[len - 1] == '\r' || body[len - 1] == '\n')) {
        len--;
    }
    return len;
}

/**
 * 判定一帧是"陶晶驰二进制返回码"还是"用户配置的文本命令"。
 * 规则：只要含不可打印字节（<0x20 或 >0x7E）就是二进制帧；
 *       另外 0x24('$') 单字节是"串口缓冲区溢出"通知。
 * 这样既能解析 0x65/0x66/0x70/0x71 等返回码，又不会把
 * "page 1"（首字节 0x70='p'）这类文本命令误判成 0x70 返回码。
 */
static bool frame_is_binary(const uint8_t *body, size_t len)
{
    if (len == 0) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (body[i] < 0x20 || body[i] > 0x7E) {
            return true;
        }
    }
    if (len == 1 && body[0] == 0x24) {   /* 24 FF FF FF：串口缓冲区溢出 */
        return true;
    }
    return false;
}

/**
 * 把 "h0.val=" + <小端二进制数值> 还原成 ASCII 命令 "h0.val=50"。
 *
 * 背景：陶晶驰的 `prints h0.val,0` 发的是**4 字节小端二进制**，如果屏幕工程用
 *     prints "h0.val=",0     // ASCII 前缀
 *     prints h0.val,0        // 小端二进制数值
 * 拼出来，音箱收到的就是"可打印前缀 + 不可打印数值"，按普通文本解析会被丢掉。
 * 这里做一次还原，使两种写法（纯文本 / 文本+二进制）都能工作。
 */
static bool frame_try_attr_synth(const uint8_t *body, size_t len, char *out, size_t out_sz)
{
    size_t k = (size_t)-1;

    /* 只在前 16 字节内找 ".val="（控件名不会很长） */
    for (size_t i = 0; i + 5 <= len && i < 16; i++) {
        if (memcmp(&body[i], ".val=", 5) == 0) {
            k = i;
            break;
        }
    }
    if (k == (size_t)-1) {
        return false;
    }

    const size_t prefix_len = k + 5;            /* 含 ".val=" */
    for (size_t i = 0; i < prefix_len; i++) {
        if (body[i] < 0x20 || body[i] > 0x7E) {
            return false;                       /* 前缀本身不是 ASCII，不是这种写法 */
        }
    }

    const size_t tail = len - prefix_len;
    if (tail == 0 || tail > 4) {                /* 0=纯文本（走文本通道）；>4 不是数值 */
        return false;
    }

    uint32_t v = 0;
    for (size_t i = 0; i < tail; i++) {
        v |= (uint32_t)body[prefix_len + i] << (8 * i);     /* 小端 */
    }

    int n = snprintf(out, out_sz, "%.*s%lu", (int)prefix_len, (const char *)body, (unsigned long)v);
    return (n > 0 && (size_t)n < out_sz);
}

/**
 * 判定一帧是不是"标准陶晶驰二进制返回码"的固定结构。
 * 用途：区分"真正的返回码帧"和"未带帧尾的文本命令与返回码粘连成的混合帧"
 *       （例如 "leds" + 04 FF FF FF → 帧体 6C 65 64 73 04）。
 * 注意 0x65/0x66/0x67/0x68/0x70/0x71 同时也是可打印字符('e'/'f'/'g'/'h'/'p'/'q')，
 * 所以必须连长度一起判断，才能和 "exit"、"page 1" 这类文本命令区分开。
 */
static bool frame_is_known_code_layout(const uint8_t *body, size_t len)
{
    switch (body[0]) {
    case 0x65:  return (len == 4 && body[3] <= 0x01);   /* 控件按下/弹起：页ID+控件ID+事件 */
    case 0x66:  return (len == 2);                      /* 当前页面 ID */
    case 0x67:  return (len == 5);                      /* 触摸坐标 x(2)+y(2)+事件 */
    case 0x68:  return (len == 4);                      /* 睡眠模式触摸事件 */
    case 0x70:                                          /* 字符串/数值变量数据 */
    case 0x71:  return (len >= 2);
    case 0x86:                                          /* 自动进入睡眠 */
    case 0x87:                                          /* 自动唤醒 */
    case 0x88:                                          /* 启动成功 */
    case 0x89:                                          /* 开始 SD 卡升级 */
    case 0xFD:                                          /* 透传完成 */
    case 0xFE:  return (len == 1);                      /* 透传就绪 */
    default:    return (len == 1 && body[0] <= 0x24);   /* 00~24 指令执行错误码 */
    }
}

/* 陶晶驰返回码含义（对照官方《串口屏返回数据格式》），便于一眼定位屏幕工程的问题 */
static const char *tjc_code_desc(uint8_t code)
{
    switch (code) {
    case 0x00: return "无效指令";
    case 0x01: return "指令成功执行";
    case 0x02: return "控件ID无效（当前页面没有这个控件名）";
    case 0x03: return "页面ID无效";
    case 0x04: return "图片ID无效（picc/pic/pic1/pic2 指定的图片不存在，或该控件不是切图/图片控件）";
    case 0x05: return "字库ID无效";
    case 0x06: return "文件操作失败";
    case 0x09: return "CRC校验失败";
    case 0x11: return "波特率设置无效";
    case 0x12: return "曲线控件ID号或通道号无效";
    case 0x1A: return "变量名称无效";
    case 0x1B: return "变量运算无效";
    case 0x1C: return "赋值操作失败";
    case 0x1D: return "掉电存储空间操作失败";
    case 0x1E: return "参数数量无效";
    case 0x1F: return "IO操作失败";
    case 0x20: return "转义字符使用错误";
    case 0x23: return "变量名称太长";
    case 0x24: return "串口缓冲区溢出";
    default:   return "未知返回码";
    }
}

/* 二进制返回码分发 */
static void handle_binary_code(const uint8_t *body, size_t len)
{
    if (len == 0) {
        return;
    }

    /* ★ 静默忽略的返回码：0x04 = 图片ID无效。
     * 屏幕工程里图片号不对时会**持续**回这个码，按串口屏要求既不处理也不打印，
     * 直接从字节流里跳过、继续解析后面的真实命令。
     * 额外好处：不打印就不会占用串口输出时间，避免接收缓冲区被冲溢出、
     * 把真正的命令（ledon/ledoff/bton…）挤掉。
     * 需要排查时把日志级别调到 DEBUG 才能看到这一行。 */
    if (body[0] == 0x04) {
        ESP_LOGD(TAG, "忽略返回码 0x04（%s）", tjc_code_desc(body[0]));
        return;
    }

    switch (body[0]) {
    case 0x65:  /* 控件点击事件：0x65 <页ID> <控件ID> <事件> */
        if (len >= 4) {
            ESP_LOGI(TAG, "解析指令: 控件事件 页ID=%u 控件ID=%u %s",
                     body[1], body[2], body[3] == 0x01 ? "按下" : "弹起");
            if (s_key_cb) {
                s_key_cb(body[1], body[2], body[3]);
            }
        }
        break;

    case 0x66:  /* 当前页面 ID（sendme 的回复） */
        if (len >= 2) {
            ui_page_id = body[1];
            ESP_LOGI(TAG, "解析指令: 屏幕回报当前页面 ID=%u", body[1]);
            if (s_page_cb) {
                s_page_cb(body[1]);
            }
        }
        break;

    case 0x86:  /* 设备自动进入睡眠模式（比 exit 超时判定更可靠，优先采用） */
        ESP_LOGW(TAG, "解析指令: 屏幕自动进入睡眠模式（0x86）");
        if (s_event_cb) {
            s_event_cb(0x86);
        }
        break;

    case 0x87:  /* 设备自动唤醒 */
        ESP_LOGI(TAG, "解析指令: 屏幕自动唤醒（0x87）");
        if (s_event_cb) {
            s_event_cb(0x87);
        }
        break;

    case 0x88:  /* 屏幕上电/复位完成 */
        ESP_LOGI(TAG, "解析指令: 屏幕启动成功（0x88）");
        if (s_event_cb) {
            s_event_cb(0x88);
        }
        uart_send("sendme");        /* 问一下当前页面，便于补发该页状态 */
        break;

    case 0x70:
    case 0x71:  /* get 指令返回的变量数据（本工程暂不解析） */
        ESP_LOGW(TAG, "get 变量数据返回（0x%02X, len=%u），本工程暂未使用", body[0], (unsigned)len);
        break;

    default:
        ESP_LOGW(TAG, "TJC 返回码 0x%02X：%s（len=%u；串口屏工程自身事件代码里的错误也会回传，"
                      "不一定与本机发送的指令有关）",
                 body[0], tjc_code_desc(body[0]), (unsigned)len);
        break;
    }
}

static void handle_frame(const uint8_t *body, size_t len)
{
    /* 先剥掉尾部 \r\n（printh 0d 0a 的收尾） */
    len = frame_trim(body, len);
    if (len == 0) {
        return;     /* 光秃秃一个帧尾，忽略 */
    }

    /* 1) 全可打印 ASCII → 文本命令（页面名 / exit / ledon / n0.val=1 / R255 ...） */
    if (!frame_is_binary(body, len)) {
        post_text_cmd(body, len);
        return;
    }

    /* 2) "ASCII 属性名 + 小端二进制数值"（prints "h0.val=",0 + prints h0.val,0） */
    {
        char synth[UI_CMD_MAX];
        if (frame_try_attr_synth(body, len, synth, sizeof(synth))) {
            if (s_ui_cmd_queue &&
                xQueueSend(s_ui_cmd_queue, synth, pdMS_TO_TICKS(20)) != pdTRUE) {
                ESP_LOGW(TAG, "命令队列已满，丢弃: \"%s\"", synth);
            }
            return;
        }
    }

    /* 3) 标准二进制返回码帧 */
    if (frame_is_known_code_layout(body, len)) {
        handle_binary_code(body, len);
        return;
    }

    /* 4) 混合帧：**未带帧尾的文本命令** + 紧跟的二进制返回码（屏幕错误帧密集时的常态）。
     *    实测两种顺序都会出现：
     *        a) 文本在前： 6C 65 64 6F 6E 04 FF FF FF   → "ledon" + 0x04
     *        b) 返回码在前：04 6C 65 64 6F 6E FF FF FF   → 0x04 + "ledon"
     *    这里把帧体里的**所有可打印文本段**都取出来当命令分发，不可打印部分交给返回码处理
     *    （0x04 会被静默忽略），从而不会因为"粘连"把真正的命令吞掉。 */
    {
        size_t i = 0;
        bool posted_any = false;

        /* 4a) 帧体开头的不可打印段（通常就是一个返回码，如 04） */
        while (i < len && !(body[i] >= 0x20 && body[i] <= 0x7E)) {
            i++;
        }
        if (i > 0) {
            ESP_LOGD(TAG, "帧体前段是不可打印返回码（%u 字节），按返回码处理", (unsigned)i);
            handle_binary_code(body, i);
        }

        /* 4b) 取出所有可打印文本段（>=2 字节才算命令，避免把噪声单个字节当命令），
         *     夹在中间的不可打印段仍按返回码处理（0x04 静默，其它码照常提示）。 */
        while (i < len) {
            size_t start = i;
            while (i < len && body[i] >= 0x20 && body[i] <= 0x7E) {
                i++;
            }
            if (i - start >= 2) {
                post_text_cmd(&body[start], i - start);
                posted_any = true;
            }

            size_t bin_start = i;
            while (i < len && !(body[i] >= 0x20 && body[i] <= 0x7E)) {
                i++;    /* 跳过中间的不可打印噪声 */
            }
            if (i > bin_start) {
                handle_binary_code(&body[bin_start], i - bin_start);
            }
        }

        if (posted_any) {
            return;
        }
    }

    /* 5) 兜底：按返回码处理 */
    handle_binary_code(body, len);
}

/* 文本命令：只保留可打印 ASCII，去掉首尾空白后投入队列 */
static void post_text_cmd(const uint8_t *body, size_t len)
{
    char cmd[UI_CMD_MAX];
    size_t n = 0;

    for (size_t i = 0; i < len && n < sizeof(cmd) - 1; i++) {
        uint8_t c = body[i];
        if (c >= 0x20 && c <= 0x7E) {
            cmd[n++] = (char)c;
        }
    }
    cmd[n] = '\0';

    /* 去尾部空白（屏幕若用 printh 0d 0a 收尾会带 \r\n，虽不可打印已被剔除，
     * 但可能有空格） */
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\t')) {
        cmd[--n] = '\0';
    }
    if (n == 0) {
        return;
    }

    /* 这里只打 DEBUG：真正的"解析指令"日志由 ui.c 统一打印（含属性回传形式） */
    ESP_LOGD(TAG, "收到文本命令: \"%s\"", cmd);

    if (s_ui_cmd_queue) {
        if (xQueueSend(s_ui_cmd_queue, cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
            ESP_LOGW(TAG, "命令队列已满，丢弃: \"%s\"", cmd);
        }
    }
}
