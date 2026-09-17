#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bt_app_core.h"
#include "amp.h"
#include "audio_vol.h"
#include "uart.h"       /* ui_res：串口屏命令缓冲（bt_vo 解析音量时读取） */
#include "driver/i2s_std.h"
#include "freertos/ringbuf.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"


#define RINGBUF_HIGHEST_WATER_LEVEL    (32 * 1024)  //缓冲区最大容量阈值
#define RINGBUF_PREFETCH_WATER_LEVEL   (20 * 1024)  //开始播放的数据量阈值

extern esp_bd_addr_t g_connected_bda; // 初始化为全0（表示未连接）

enum {
    RINGBUFFER_MODE_PROCESSING,    //正常播放状态
    RINGBUFFER_MODE_PREFETCHING,   //缓冲不足，等待更多数据
    RINGBUFFER_MODE_DROPPING       //数据异常，丢弃无效数据
};

/*******************************
 * STATIC FUNCTION 声明
 ******************************/

/* 主应用程序任务 */
static void bt_app_task_handler(void *arg);
/* I2S 任务 */
static void bt_i2s_task_handler(void *arg);
/* 发送消息到队列的函数 */
static bool bt_app_send_msg(bt_app_msg_t *msg);
/* 专门工作分发函数 */
static void bt_app_work_dispatched(bt_app_msg_t *msg);

/*******************************
 * STATIC VARIABLE 定义
 ******************************/

static QueueHandle_t s_bt_app_task_queue = NULL;  /* 创建消息队列 */
TaskHandle_t s_bt_app_task_handle = NULL;  /* 创建应用任务句柄  */
TaskHandle_t s_bt_i2s_task_handle = NULL;  /* 创建I2S任务句柄  */
static RingbufHandle_t s_ringbuf_i2s = NULL;     // I2S环形缓冲区               （实时性，边写入边读取，做到只使用32KB内存）
static SemaphoreHandle_t s_i2s_write_semaphore = NULL;  // I2S写信号量
static uint16_t ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;   // 当前模式

/*********************************
 * EXTERNAL FUNCTION 声明
 ********************************/
#ifndef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
extern i2s_chan_handle_t tx_chan;   // 使用外部I2S编码器
#else
extern dac_continuous_handle_t tx_chan; //// 使用内部DAC
#endif

/*******************************
 * 自定义函数定义
 ******************************/
/* 播放/暂停（btpau/btpla）属于"调度级"开关，不是初始化/反初始化。
 *
 * 【本轮整改】按串口屏要求：这两个命令必须与手机用 AVRCP 同步，但**不再挂起蓝牙/I2S 任务**。
 *   原因：挂起后收不到手机端事件，用户在手机上点"播放"时音箱不会跟随（会静音不同步）；
 *         而且旧实现在未连接手机时 s_bt_i2s_task_handle == NULL，
 *         vTaskSuspend(NULL) 挂起的是**调用者自己**（分发串口屏命令的 ui_task），
 *         会导致"按一次暂停后串口屏全部命令都没反应"。
 *   现在只做两件事：① 发 AVRCP 暂停/播放；② 立即关/开功放。任务始终运行。
 */
static bool s_bt_paused = false;

void bt_sleep(void)
{
    s_bt_paused = true;
    amp_set_power(false);           // 暂停：立即关断功放（静音且省电）
    /* 保持接收音频（write_data_sleep_flag = 1）：手机上再点播放时音箱能立刻跟随 */
    write_data_sleep_flag = 1;
    esp_avrc_ct_send_passthrough_cmd(
            0,
        ESP_AVRC_PT_CMD_PAUSE,
        ESP_AVRC_PT_CMD_STATE_PRESSED
    );
    ESP_LOGI("bt_sleep", "已发送 AVRCP 暂停并关断功放（任务不挂起，保持与手机同步）");
}

void bt_work(void)
{
    s_bt_paused = false;
    amp_set_power(true);            // 播放：开启功放
    write_data_sleep_flag = 1;
    esp_avrc_ct_send_passthrough_cmd(
            1,
        ESP_AVRC_PT_CMD_PLAY,
        ESP_AVRC_PT_CMD_STATE_PRESSED
    );
    ESP_LOGI("bt_work", "已发送 AVRCP 播放并开启功放");
}

/* 供蓝牙反初始化（关蓝牙）复用：把可能被挂起的任务恢复回可调度状态 */
void bt_tasks_resume(void)
{
    if (s_bt_app_task_handle) {
        vTaskResume(s_bt_app_task_handle);
    }
    if (s_bt_i2s_task_handle) {
        vTaskResume(s_bt_i2s_task_handle);
    }
    s_bt_paused = false;
}

bool bt_is_paused(void)
{
    return s_bt_paused;
}

/* =====================================================================
 * 串口屏音量接口
 *   命令格式： ui_res = "V<0..127>"，例如 "V100" = 音量 100
 *   分发位置： main.c 的 process_command() → ui_res[0]=='V' 分支
 *   说明：     实际的增益换算与限幅在 audio_vol.c，本函数只负责解析与转交。
 * ===================================================================== */
void bt_vo(void)
{
    int vo = atoi(ui_res + 1);
    if (vo < 0) {
        vo = 0;
    }
    if (vo > AUDIO_VOL_MAX) {
        vo = AUDIO_VOL_MAX;
    }
    audio_vol_set((uint8_t)vo);
    ESP_LOGI("bt", "串口屏音量设置为: %d (总衰减 %.1fdB)", vo, (double)audio_vol_get_db());
}
void bt_ne(void)
{
    ESP_LOGI("bt","下一首");
    /* 通过 AVRCP 把"下一曲"透传给手机（事务标签 2，与暂停/播放的 0/1 区分开） */
    esp_avrc_ct_send_passthrough_cmd(2, ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
}
void bt_la(void)
{
    ESP_LOGI("bt","上一首");
    esp_avrc_ct_send_passthrough_cmd(3, ESP_AVRC_PT_CMD_BACKWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
}
/* 静音/开声：静音前记住当前音量，开声时恢复到该音量（而不是一律恢复到最大） */
static uint8_t s_vol_before_mute = AUDIO_VOL_DEFAULT;
void bt_sli(void)
{
    uint8_t cur = audio_vol_get();
    if (cur != 0) {
        s_vol_before_mute = cur;
    }
    audio_vol_set(0);
    ESP_LOGI("bt","静音");
}
void bt_noi(void)
{
    audio_vol_set(s_vol_before_mute ? s_vol_before_mute : AUDIO_VOL_DEFAULT);
    ESP_LOGI("bt","开声");
}

/*******************************
 * STATIC FUNCTION 定义
 ******************************/

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    //检查是否为空
    if (msg == NULL) {
        return false;
    }

    //将消息发送到工作队列
    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s 队列发送失败", __func__);
        return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;
    /*结构体含义
        uint8_t sig                信号类型
        uint16_t event             事件类型
        void *param                参数指针
        void (*cb)(uint16_t, void*) 回调函数
    */

    for (;;) {
        //工作队列接收消息并处理
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (TickType_t)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, 信号类型: 0x%x, 事件: 0x%x", __func__, msg.sig, msg.event);

            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled signal: %d", __func__, msg.sig);
                break;
            }

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

static void bt_i2s_task_handler(void *arg)
{
    uint8_t *data = NULL;   //数据指针
    size_t item_size = 0;   //数据总大小
    /**
     * 缓冲区总长度:
     * `dma_frame_num * dma_desc_num * i2s_channel_num * i2s_data_bit_width / 8
     * 即 每个DMA描述符包含的音频帧数量*DMA描述符的数量*音频声道数量*每个采样点的数据位宽/8
     */
    const size_t item_size_upto = 240 * 6;  //读取的最大数量
    size_t bytes_written = 0;   //实际写入的字节数

    for (;;) {
        if (pdTRUE == xSemaphoreTake(s_i2s_write_semaphore, portMAX_DELAY)) {
            for (;;) {
                item_size = 0;
                //从ringbuffer接收数据并将其写入I2S DMA传输缓冲区
                data = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size, (TickType_t)pdMS_TO_TICKS(20), item_size_upto);
                //检测下溢（数据不足）
                if (item_size == 0) {
                    ESP_LOGI(BT_APP_CORE_TAG, "环缓冲区下溢! 等待数据: RINGBUFFER_MODE_PREFETCHING");
                    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                    break;
                }

                /* ★ 消除炸音的关键一步 ★
                 * 数字音量级配 + 饱和限幅：把 PCM5102A 的 2.1Vrms 满刻度压进 PAM8403 的线性输入区
                 * （5V/4Ω 下不得超过约 0.19Vrms），等价于"把功放电位器扭到安全位置"。
                 * 16bit 交错立体声一帧 = 4 字节，先按 4 字节对齐裁剪，避免把一帧切两半。 */
                size_t gain_bytes = item_size & ~((size_t)0x3);
                if (gain_bytes) {
                    audio_apply_gain_i16((int16_t *)data, gain_bytes >> 1);
                }

            #ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC     //内部DAC
                dac_continuous_write(tx_chan, data, item_size, &bytes_written, -1);
            #else
                i2s_channel_write(tx_chan, data, item_size, &bytes_written, portMAX_DELAY);     //外部I2S
            #endif
                vRingbufferReturnItem(s_ringbuf_i2s, (void *)data);     //释放已处理的数据项
            }
        }
    }
}

/********************************
 * 回调函数 定义
 *******************************/

 //工作任务分发回调
bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s 事件: 0x%x,参数长度: %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    //无参数
    if (param_len == 0) {
        return bt_app_send_msg(&msg);   //直接发送到消息队列
    } 
    //有参数
    else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {  //分配内存
            memcpy(msg.param, p_params, param_len); //浅层复制
            //检查调用者是否提供了回调函数来执行深度复制
            if (p_copy_cback) {
                p_copy_cback(msg.param, p_params, param_len);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

//任务启动函数
void bt_app_task_start_up(void)
{
    /* 幂等：初始化只做一次（初始化已在上电由 bt_init()/bt_a2dp_work() 完成，
     * 重复创建会多出一个任务和一条队列，旧代码在"关蓝牙后再开"时就会重复创建） */
    if (s_bt_app_task_handle != NULL) {
        ESP_LOGI(BT_APP_CORE_TAG, "蓝牙应用任务已存在，跳过重复创建");
        return;
    }
    s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));   //创建消息队列：容量10条消息，每条消息大小= bt_app_msg_t
    xTaskCreate(bt_app_task_handler, "BtAppTask", 3072, NULL, 10, &s_bt_app_task_handle);
}

//任务关闭函数
void bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}

//I2S任务启动函数
void bt_i2s_task_start_up(void)
{
    /* 幂等：若上一次的任务/环形缓冲区还在（例如断开重连、关蓝牙后再开），
     * 先释放，避免任务与 32KB 环形缓冲区重复创建造成内存泄漏 */
    if (s_bt_i2s_task_handle != NULL || s_ringbuf_i2s != NULL) {
        bt_i2s_task_shut_down();
    }

    ESP_LOGI(BT_APP_CORE_TAG, "环缓冲区数据为空! 等待数据: RINGBUFFER_MODE_PREFETCHING");
    // 初始化模式为预取模式（等待数据）
    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    // 创建二进制信号量（用于控制I2S写入，无数据处理时休眠）
    if ((s_i2s_write_semaphore = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s, 信号量 create failed", __func__);
        return;
    }
    // 创建环形缓冲区（32KB大小，字节缓冲模式）
    if ((s_ringbuf_i2s = xRingbufferCreate(RINGBUF_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF)) == NULL) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s, ringbuffer create failed", __func__);
        return;
    }
    //创建I2S任务
    xTaskCreate(bt_i2s_task_handler, "BtI2STask", 2048, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle);
}

//I2S任务关闭函数
void bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }
    if (s_ringbuf_i2s) {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
    }
    if (s_i2s_write_semaphore) {
        vSemaphoreDelete(s_i2s_write_semaphore);
        s_i2s_write_semaphore = NULL;
    }
}

//环形缓冲区写入函数
size_t write_ringbuf(const uint8_t *data, size_t size)
{
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    //丢弃模式
    if (ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        ESP_LOGW(BT_APP_CORE_TAG, "数据异常，丢弃此数据包!");
        // 获取环形缓冲区的信息（主要是当前数据量）
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        // 检查缓冲区是否下降到可处理水平
        if (item_size <= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_APP_CORE_TAG, "环缓冲区数据正常!处理模式: RINGBUFFER_MODE_PROCESSING");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        return 0;
    }

    //尝试写入数据
    done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (TickType_t)0);

    // 写入失败，缓冲区溢出
    if (!done) {
        ESP_LOGW(BT_APP_CORE_TAG, "数据溢出，丢弃模式: RINGBUFFER_MODE_DROPPING");
        ringbuffer_mode = RINGBUFFER_MODE_DROPPING; //切换到丢弃模式
    }

    //模式切换：预取 → 处理
    if (ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        // 获取环形缓冲区的信息（主要是当前数据量）
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        // 检查缓冲区数据量是否达到预取水位线（20KB）
        if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_APP_CORE_TAG, "数据量达到水位线，处理模式: RINGBUFFER_MODE_PROCESSING");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;   //切换到处理模式
            if (pdFALSE == xSemaphoreGive(s_i2s_write_semaphore)) { //释放信号量，启动I2S任务
                ESP_LOGE(BT_APP_CORE_TAG, "semphore give failed");
            }
        }
    }

    return done ? size : 0;
}
