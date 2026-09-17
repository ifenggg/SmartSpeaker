#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include "bt_app_core.h"
#include "bt_app_av.h"
#include "amp.h"
#include "audio_vol.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"

#include "sys/lock.h"

#include "ui.h"     /* 串口屏界面状态：当前页判断 + 状态推送 */

//AVRCP（音视频远程控制协议）使用的事务标签
#define APP_RC_CT_TL_GET_CAPS            (0)    //标识 AVRCP 控制器发起的 “获取对方设备能力” 操作
#define APP_RC_CT_TL_GET_META_DATA       (1)    //获取当前播放音频的标题、艺术家等元数据
#define APP_RC_CT_TL_RN_TRACK_CHANGE     (2)    //曲目变化通知
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE  (3)    //播放状态变化通知
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE  (4)    //播放进度变化通知

//引脚宏定义（改用 Kconfig 配置，menuconfig 修改即可生效）
#define I2S_BCK_PIN     CONFIG_EXAMPLE_I2S_BCK_PIN
#define I2S_LRCK_PIN    CONFIG_EXAMPLE_I2S_LRCK_PIN
#define I2S_DATA_PIN    CONFIG_EXAMPLE_I2S_DATA_PIN

//应用层的延迟值
#define APP_DELAY_VALUE                  50  // 5ms

/*******************************
 * STATIC 任务函数 声明
 ******************************/

//分配新的元缓冲区
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
//新曲目加载处理程序
static void bt_av_new_track(void);
//通道状态变化的处理程序
static void bt_av_playback_changed(void);
//轨道播放进度变化的处理器
static void bt_av_play_pos_changed(void);
//通知事件处理程序
static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
//安装i2s
static void bt_i2s_driver_install(void);
//卸载i2s
void bt_i2s_driver_uninstall(void);
//通过遥控器设置音量
static void volume_set_by_controller(uint8_t volume);
//通过本地主机（esp）设置音量
static void volume_set_by_local_host(uint8_t volume);
//模拟音量变化
//static void volume_change_simulation(void *arg);
//a2dp事件处理程序
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
//avrc控制器事件处理器
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
//AVRC目标事件处理程序
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);

/*******************************
 * 自定义VARIABLE 定义
 ******************************/

uint8_t write_data_sleep_flag = 1;
uint8_t bt_con_flag;
esp_bd_addr_t g_connected_bda = {0}; // 初始化为全0（表示未连接）
extern TaskHandle_t s_bt_i2s_task_handle;  /* 创建I2S任务句柄  */

/*******************************
 * STATIC VARIABLE 定义
 ******************************/

static uint32_t s_pkt_cnt = 0;               //计数音频包
static uint32_t pos = 0;        //实时播放进度
uint32_t total_pos = 1;             //歌曲总时长（赋初值，避免除以零引起系统复位）
static uint8_t  s_progress = 0;     //播放进度百分比（0-100），供串口屏切页后补发
static esp_a2d_audio_state_t s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
                                             //音频流数据路径状态
static const char *s_a2d_conn_state_str[] = {"已断开连接","正在连接","已连接","正在断开连接"};
                                             //连接状态字符串
static const char *s_a2d_audio_state_str[] = {"暂停", "启动"};
                                             //音频流数据路径状态字符串
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
                                             //AVRC目标通知功能位掩码
static _lock_t s_volume_lock;
static TaskHandle_t s_vcs_task_hdl = NULL;    //处理音量变化模拟任务的句柄
uint8_t s_volume = 0x7f;              //手机绝对音量（AVRCP 上报值）；初值给最大，
                                      //避免向手机回送 0 时把手机侧音量一并清零
static bool s_volume_notify;                 //通知音量变化与否
#ifndef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
i2s_chan_handle_t tx_chan = NULL;
#else
dac_continuous_handle_t tx_chan;
#endif

/********************************
 * STATIC 任务函数 定义
 *******************************/

//分配合适的内存空间，并把数据转换为“\0”结束的字符串
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param); //转换指针类型
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1); //+1 是为了存储字符串结束符 \0

    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);    //把原始元数据文本复制到刚分配的内存中
    attr_text[rc->meta_rsp.attr_length] = 0;
    rc->meta_rsp.attr_text = attr_text;
}

static void bt_av_new_track(void)
{
    //请求元数据
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM |
                        ESP_AVRC_MD_ATTR_GENRE |
                        ESP_AVRC_MD_ATTR_PLAYING_TIME;
    esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    //远程设备如果支持新曲目加载事件，则注册通知
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_TRACK_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                                   ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

static void bt_av_playback_changed(void)
{
    //远程设备如果支持播放状态通知事件，则注册通知
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
                                                   ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

static void bt_av_play_pos_changed(void)
{
    //远程设备如果支持曲目进度变化事件，则注册通知
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAY_POS_CHANGE,
                                                   ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
    }
}

static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter)
{
    switch (event_id) {
    //新曲目加载
    case ESP_AVRC_RN_TRACK_CHANGE:
        bt_av_new_track();
        break;
    //曲目播放状态更改
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        ESP_LOGI(BT_AV_TAG, "播放状态变化: 0x%x", event_parameter->playback);
        bt_av_playback_changed();
        break;
    //曲目播放进度更改
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        ESP_LOGI(BT_AV_TAG, "播放进度变化: %"PRIu32"-ms", event_parameter->play_pos);
        pos = event_parameter->play_pos;
        if(total_pos)
        {
            s_progress = (uint8_t)((pos * 100) / total_pos);
            ESP_LOGI(BT_RC_CT_TAG, "播放进度：%"PRIu8, s_progress);
            /* 只有屏幕停在蓝牙页才推送（进度条控件是页面私有，切页后由 ui 层补发） */
            if (ui_get_page() == UI_PAGE_BT) {
                ui_push_bt_play_state();
            }
        }
        bt_av_play_pos_changed();
        break;
    //其他
    default:
        ESP_LOGI(BT_AV_TAG, "unhandled event: %d", event_id);
        break;
    }
}

void bt_i2s_driver_install(void)
{
    /* 幂等保护：已经安装过就不再重复创建（重复安装会失败并触发断言复位） */
    if (tx_chan != NULL) {
        ESP_LOGW(BT_AV_TAG, "I2S 驱动已安装，跳过重复安装");
        return;
    }
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = 8,
        .buf_size = 2048,
        .freq_hz = 44100,
        .offset = 127,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,   // Using APLL as clock source to get a wider frequency range
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
    };
    /* Allocate continuous channels */
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &tx_chan));
    /* Enable the continuous channels */
    ESP_ERROR_CHECK(dac_continuous_enable(tx_chan));
#else
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN,
            .ws = I2S_LRCK_PIN,
            .dout = I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* 使能 I2S */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
#endif
}

void bt_i2s_driver_uninstall(void)
{
    /* 幂等保护：未安装（或已卸载）时直接返回。
     * 旧代码在"断开连接"与"关蓝牙"两处都会卸载，第二次对已删除的通道操作会触发断言复位。 */
    if (tx_chan == NULL) {
        ESP_LOGW(BT_AV_TAG, "I2S 驱动未安装，跳过卸载");
        return;
    }
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
    ESP_ERROR_CHECK(dac_continuous_disable(tx_chan));
    ESP_ERROR_CHECK(dac_continuous_del_channels(tx_chan));
#else
    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
#endif
    tx_chan = NULL;
}

/* ============================ 状态查询（供串口屏） ============================ */

bool bt_a2d_is_playing(void)
{
    return (s_audio_state == ESP_A2D_AUDIO_STATE_STARTED);
}

uint8_t bt_a2d_get_progress(void)
{
    return s_progress;
}

static void volume_set_by_controller(uint8_t volume)
{
    //设置锁保护中的音量
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);

    /* 上报给软件音量模块：模式B（AUDIO_USE_REMOTE_VOL=1）下由本机代手机执行衰减，
     * 手机音量条才能生效；若该机型自己做数字衰减（模式A），把宏改回 0 即可，避免双重衰减。 */
    audio_vol_set_remote(volume);

    ESP_LOGI(BT_RC_TG_TAG, "AVRC 收到设置音量: %"PRIu32"%% -> 总衰减 %.1fdB (增益 %"PRIu32"/32768)",
             (uint32_t)volume * 100 / 0x7f, (double)audio_vol_get_db(), audio_vol_get_gain_q15());

    /* 静音判据统一改为"本机音量"：原实现用 s_volume 判断，而 s_volume 初值为 0，
     * 手机未上报绝对音量前会把功放永久静音（既有缺陷）。 */
    amp_set_mute(audio_vol_get() == 0);
}

static void volume_set_by_local_host(uint8_t volume)
{
    //将输入的音量值转换为百分比形式显示
    ESP_LOGI(BT_RC_TG_TAG, "音量设置为: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    //设置锁保护中的音量（用锁操作保护全局变量）
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);

    //本机音量 → 数字衰减（与串口屏 bt_vo() 走同一条通路）
    audio_vol_set(volume);

    //向远程AVRCP控制器发送通知响应
    if (s_volume_notify) {
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = s_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
    amp_set_mute(audio_vol_get() == 0);   // 音量联动：本机音量=0 自动静音
}

// static void volume_change_simulation(void *arg)
// {
//     ESP_LOGI(BT_RC_TG_TAG, "start volume change simulation");

//     for (;;) {
//         //每10秒本地音量增加一次
//         vTaskDelay(10000 / portTICK_PERIOD_MS);
//         uint8_t volume = (s_volume + 5) & 0x7f;
//         volume_set_by_local_host(volume);
//     }
// }

static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    esp_a2d_cb_param_t *a2d = NULL;

    switch (event) {
    //连接状态改变
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        memcpy(g_connected_bda, a2d->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "A2DP 连接状态: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
            s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        //断开连接
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);  //设置蓝牙为可连接和可发现模式
            if(ui_get_page() == UI_PAGE_BT)
                ui_push_bt_conn_state();
            // s_audio_state = ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND;
            // s_pkt_cnt = 0;  // 重置数据包计数
            // s_avrc_peer_rn_cap.bits = 0;    //清除对等方通知能力记录

            bt_i2s_driver_uninstall();  //卸载iis驱动器和相关任务
            bt_i2s_task_shut_down();
            amp_set_power(false);   // 蓝牙断开 → 关断功放
            bt_con_flag = 0;        // 【修复】断开后清零连接标志，否则后续"关蓝牙"会重复反初始化 I2S
        } 
        //连接成功
        else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){   
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);//设置蓝牙为不可连接和不可发现模式
            bt_i2s_task_start_up(); //启动iis任务
            vTaskResume(s_bt_i2s_task_handle);
            write_data_sleep_flag = 1;
            bt_con_flag = 1;
            if(ui_get_page() == UI_PAGE_BT)
                ui_push_bt_conn_state();
        } 
        //连接中。。。
        else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            bt_i2s_driver_install();    //安装iis驱动器
        }
        break;
    }
    //音频流传输状态发生变化
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP 音频状态: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
        s_audio_state = a2d->audio_stat.state;
        /* 播放/暂停图标只在屏幕停在蓝牙页时推送（切页后由 ui 层补发） */
        if (ui_get_page() == UI_PAGE_BT) {
            ui_push_bt_play_state();
        }
        //音频开始，重置音频包计数器
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            s_pkt_cnt = 0;
            amp_set_power(true);            // 音频开始 → 唤醒功放
            amp_set_mute(audio_vol_get() == 0);    // 音量联动：本机音量=0→静音
        } else {
            amp_power_off_delayed(AMP_PAUSE_OFF_DELAY_MS);  // 暂停/停止 → 延时关断
        }
        break;
    }
    //音频编解码器配置完成
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP音频流配置完成,编解码器类型: %d", a2d->audio_cfg.mcc.type);
        //官方示例仅支持SBC流媒体
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            int sample_rate = 16000;
            int ch_count = 2;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            //解析采样率
            if (oct0 & (0x01 << 6)) {
                sample_rate = 32000;
            } else if (oct0 & (0x01 << 5)) {
                sample_rate = 44100;
            } else if (oct0 & (0x01 << 4)) {
                sample_rate = 48000;
            }

            //解析声道数量
            if (oct0 & (0x01 << 3)) {
                ch_count = 1;
            }
        #ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
            dac_continuous_disable(tx_chan);
            dac_continuous_del_channels(tx_chan);
            dac_continuous_config_t cont_cfg = {
                .chan_mask = DAC_CHANNEL_MASK_ALL,
                .desc_num = 8,
                .buf_size = 2048,
                .freq_hz = sample_rate,
                .offset = 127,
                .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,   // Using APLL as clock source to get a wider frequency range
                .chan_mode = (ch_count == 1) ? DAC_CHANNEL_MODE_SIMUL : DAC_CHANNEL_MODE_ALTER,
            };
            /* Allocate continuous channels */
            dac_continuous_new_channels(&cont_cfg, &tx_chan);
            /* Enable the continuous channels */
            dac_continuous_enable(tx_chan);
        #else
        //根据蓝牙音频参数配置 I2S 接口
            i2s_channel_disable(tx_chan);
            i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
            i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, ch_count);
            i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
            i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg);
            i2s_channel_enable(tx_chan);
        #endif
            ESP_LOGI(BT_AV_TAG, "配置音频播放器: %x-%x-%x-%x",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            ESP_LOGI(BT_AV_TAG, "播放器已配置，采样率: %d", sample_rate);
        }
        break;
    }
    //a2dp初始化或卸载完成
    case ESP_A2D_PROF_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: 初始化完成");
        } else {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Deinit 完成");
        }
        break;
    }
    //协议服务功能被配置时候触发
    case ESP_A2D_SNK_PSC_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "协议服务功能配置: 0x%x ", a2d->a2d_psc_cfg_stat.psc_mask);
        if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT) {
            ESP_LOGI(BT_AV_TAG, "对等设备支持延迟报告");
        } else {
            ESP_LOGI(BT_AV_TAG, "对等设备不支持延迟报告");
        }
        break;
    }
    //设置延迟值完成
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_SET_INVALID_PARAMS == a2d->a2d_set_delay_value_stat.set_state) {
            ESP_LOGI(BT_AV_TAG, "设置延迟值失败");
        } else {
            ESP_LOGI(BT_AV_TAG, "设置延迟值成功, 延迟值为: %u * 1/10 ms", a2d->a2d_set_delay_value_stat.delay_value);
        }
        break;
    }
    //获取延迟值完成
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "Get delay report value: delay_value: %u * 1/10 ms", a2d->a2d_get_delay_value_stat.delay_value);
        /*默认延迟值 + 由应用层引起的延迟*/
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + APP_DELAY_VALUE);
        break;
    }
    /* 其他 */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_CT_TAG, "%s event: %d", __func__, event);

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event) {
    //连接状态改变
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_CT_TAG, "AVRC连接状态: %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            //获取远程设备支持的事件通知
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            //清除对等方通知能力记录
            s_avrc_peer_rn_cap.bits = 0;
        }
        break;
    }
    //处理遥控按键的响应
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC 按键响应: 按键ID 0x%x, 按键状态 %d, 响应内容 %d", rc->psth_rsp.key_code,
                    rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
        break;
    }
    //元数据的响应
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC 元数据响应: 属性ID 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        /* 旧实现里 medata_flag 恒为 0，导致任何属性都会覆盖 t2.txt；
         * 现在按 AVRCP 属性号分发：0x1=TITLE（曲名），0x2=ARTIST（歌手）。
         * ui 层会缓存这两个文本，屏幕切回蓝牙页时用缓存补发。 */
        if(rc->meta_rsp.attr_id == 0x1)
        {
            ui_push_bt_track_text((const char *)rc->meta_rsp.attr_text, NULL);
        }
        else if(rc->meta_rsp.attr_id == 0x2)
        {
            ui_push_bt_track_text(NULL, (const char *)rc->meta_rsp.attr_text);
        }
        else if(rc->meta_rsp.attr_id == 0x40)
        {
            total_pos = atoi((const char *)rc->meta_rsp.attr_text);
            // s_progress = (pos * 100)/total_pos;
            // ESP_LOGI(BT_RC_CT_TAG, "AVRC 元数据响应: 播放进度：%d", s_progress);
        }
        free(rc->meta_rsp.attr_text);
        break;
    }
    //处理远程设备的状态变化通知（播放状态、曲目、音量）
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC 事件通知: %d", rc->change_ntf.event_id);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    //了解对等设备支持哪些控制功能
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC 遥控器功能 %"PRIx32", 目标器功能 %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
        break;
    }
    //获取远程设备支持的通知类型
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "遥控 rn_cap: 计数 %d, 位掩码 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        bt_av_new_track();
        bt_av_playback_changed();
        bt_av_play_pos_changed();
        break;
    }

    default:
        ESP_LOGE(BT_RC_CT_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_TG_TAG, "%s event: %d", __func__, event);

    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);

    switch (event) {
    //连接状态改变
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_TG_TAG, "AVRC 连接状态 %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (rc->conn_stat.connected) {
            //创建任务以模拟音量值变化
            //xTaskCreate(volume_change_simulation, "vcsTask", 2048, NULL, 5, &s_vcs_task_hdl);
        } else {
            //vTaskDelete(s_vcs_task_hdl);
            ESP_LOGI(BT_RC_TG_TAG, "Stop volume change simulation");
        }
        break;
    }
    //接收来自控制器的遥控命令
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC 收到按键命令: 按键ID 0x%x, 按键状态 %d", rc->psth_cmd.key_code, rc->psth_cmd.key_state);
        break;
    }
    //从远程设备设置绝对音量命令
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC 收到设置音量: %d%%", (int)rc->set_abs_vol.volume * 100 / 0x7f);
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    }
    //处理控制器的通知注册请求
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC 注册事件通知%d, param: 0x%"PRIx32, rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = s_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    //获取远程控制器支持的AVRCP特性
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC 遥控器功能: %"PRIx32", 控制器功能: %x", rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
        break;
    }

    default:
        ESP_LOGE(BT_RC_TG_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/********************************
 * 回调函数 定义（不工作，分发到任务函数工作）
 *******************************/

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        //将事件分发到专门的工作任务中处理
        bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if(!write_data_sleep_flag)
    {
        return;
    } 
    //使用环形缓冲区存储音频数据
    write_ringbuf(data, len);
    //每发送100个数据包记录一次数量
    if (++s_pkt_cnt % 100 == 0) 
    {
        ESP_LOGI(BT_AV_TAG, "Audio packet count: %"PRIu32, s_pkt_cnt);
    }
}

void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);    // 专门处理元数据内存分配
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        //使用工作队列进行事件分发
        bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_RC_CT_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
        //使用工作队列进行事件分发
        bt_app_work_dispatch(bt_av_hdl_avrc_tg_evt, event, param, sizeof(esp_avrc_tg_cb_param_t), NULL);
        break;
    default:
        ESP_LOGE(BT_RC_TG_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}
