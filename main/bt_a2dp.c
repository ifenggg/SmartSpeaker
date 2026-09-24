#include <stdio.h>
#include <stdlib.h>
#include <bt_a2dp.h>
#include "ui.h"     /* 串口屏界面状态：判断当前是否停在蓝牙页、推送该页状态 */
#include "amp.h"    /* 关蓝牙时联动关断功放 amp_set_power() */
#include "ble_client.h" /* 整机休眠：BLE 主机链路的收尾/恢复 */
#include "ble_gap.h"    /* 整机休眠唤醒后重新注册 GAP 回调 */

#define LOCAL_DEVICE_NAME    "ESP_speaker"

/* 蓝牙"功能层"（A2DP/AVRCP + 可发现）是否已开启：
 * bt_init() 在上电执行的是**初始化**，本文件里的开关是**调度级**，两者不可混用 */
static bool s_bt_a2dp_on = false;

extern esp_bd_addr_t bda;
extern TaskHandle_t s_bt_app_task_handle;  /* 创建应用任务句柄  */
extern esp_bd_addr_t g_connected_bda; // 初始化为全0（表示未连接）

//标志蓝牙协议栈建立是否完成
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/********************************
 * STATIC FUNCTION 声明
 *******************************/

//设备回调函数
void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);
//GAP回调函数
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
//蓝牙堆栈启用事件的处理程序
void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/*******************************
 * STATIC FUNCTION 定义
 ******************************/

//将蓝牙设备地址转换为字符串形式
char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

//蓝牙设备相关的回调函数，核心作用是获取本地蓝牙设备名称
void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_DEV_NAME_RES_EVT: {
        if (param->name_res.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "Get local device name success: %s", param->name_res.name);
        } else {
            ESP_LOGE(BT_AV_TAG, "Get local device name failed, status: %d", param->name_res.status);
        }
        break;
    }
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    uint8_t *bda = NULL;

    switch (event) {
    //身份验证完成
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "身份验证成功: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
        }
        ESP_LOGI(BT_AV_TAG, "当前连接类型是: %d", param->auth_cmpl.lk_type);
        break;
    }
    //加密模式改变事件
    case ESP_BT_GAP_ENC_CHG_EVT: {
        char *str_enc[3] = {"OFF", "E0", "AES"};
        bda = (uint8_t *)param->enc_chg.bda;
        ESP_LOGI(BT_AV_TAG, "加密模式为[%02x:%02x:%02x:%02x:%02x:%02x] 改变成 %s",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], str_enc[param->enc_chg.enc_mode]);
        break;
    }

#if (CONFIG_EXAMPLE_A2DP_SINK_SSP_ENABLED == true)
    //请求用户确认安全简单配对
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT 请比较数值的大小: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    //安全简单配对密码通知
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT 密码: %"PRIu32, param->key_notif.passkey);
        break;
    //请求安全简单配对密钥
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT 请输入密码!");
        break;
#endif

    //GAP模式更改
    case ESP_BT_GAP_MODE_CHG_EVT:
        uint8_t btmode = param->mode_chg.mode;
        if(btmode == 0)
            ESP_LOGI(BT_AV_TAG, "全功率模式");
        else if(btmode == 2)
            ESP_LOGI(BT_AV_TAG, "播放暂停，低功耗模式");
        break;
    //ACL（可实时通信链路）连接完成
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_conn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT 已连接到 [%02x:%02x:%02x:%02x:%02x:%02x], 状态: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_conn_cmpl_stat.stat);
        /* 只有屏幕当前停在蓝牙页时才推送（切页后由 ui_refresh_page() 补发） */
        if (ui_get_page() == UI_PAGE_BT) {
            ui_push_bt_conn_state();
        }
        break;
    //ACL断开连接完成
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT 连接失败 from [%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_disconn_cmpl_stat.reason);
        break;
    /* 其他 */
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}

void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    //蓝牙协议栈完成
    case BT_APP_EVT_STACK_UP: {
        esp_bt_gap_set_device_name(LOCAL_DEVICE_NAME);
        /* 注意：BLE 侧名称由 ble_client.c 设置（BLE_CLIENT_LOCAL_NAME），
         * 这里只设经典蓝牙名称，避免两边互相覆盖。 */
        esp_bt_dev_register_callback(bt_app_dev_cb);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        //初始化 AVRCP 控制器，并注册回调函数
        assert(esp_avrc_ct_init() == ESP_OK);
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        //初始化 AVRCP 目标器（esp在A2DP中的角色；手机为AVRCP发起器），并注册回调函数
        assert(esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        //使能音量变化事件的远程通知能力
        esp_avrc_rn_evt_cap_mask_t evt_set = {0};   //音量变量
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK); //配置到目标器中

        //初始化 A2DP 接收端
        assert(esp_a2d_sink_init() == ESP_OK);
        esp_a2d_register_callback(bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);

        //获取延迟值的默认值（用于音频同步）
        esp_a2d_sink_get_delay_value();
        //获取本地设备名称
        esp_bt_gap_get_device_name();

        //设置可发现和可连接模式，等待被连接。
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/**
 * @brief 旧接口（保留兼容）：等价于 bt_a2dp_work()
 *
 * 【调试阶段代码，已停用】原先此处还有一段"蓝牙反初始化后手动重建协议栈"的示例
 * （esp_bt_controller_init/enable + esp_bluedroid_init/enable + 设置 PIN/SSP）。
 * 本工程 bt_init() 已在上电完成控制器与协议栈初始化，若在这里再初始化一次会与
 * 上电初始化重复；需要"彻底重建协议栈"时应改 bt_init()，故这段代码不再使用
 * （源码可从 git 历史取回）。
 */
void bt_app_start(void)
{
    bt_a2dp_work();
}

/**
 * @brief 关闭蓝牙功能（串口屏"蓝牙总开关"关 / 命令 btoff）
 *
 * 与 bt_a2dp_work() 成对：开=注册 A2DP/AVRCP 并进入可发现，关=反初始化它们。
 * （协议栈本身仍保持上电初始化的状态，即"双模蓝牙一直在"，只是不再提供 A2DP 功能）
 */
void bt_app_shutdown(void)
{
    if (!s_bt_a2dp_on) {
        ESP_LOGW(BT_AV_TAG, "蓝牙功能本就处于关闭状态，忽略重复关闭");
        return;
    }
    s_bt_a2dp_on = false;

    /* 先让对端暂停播放。**只在真的连着时才发**：没连的时候发会得到 BTC 的
     * "btc_avrc_ct_send_passthrough_cmd() called when RC is not connected" 警告，
     * 每条都把日志刷一行（整机休眠每轮都会走到这里），真问题反而被淹没。 */
    if (bt_con_flag)
        esp_avrc_ct_send_passthrough_cmd(
            0,
        ESP_AVRC_PT_CMD_PAUSE,
        ESP_AVRC_PT_CMD_STATE_PRESSED
    );

    if (bt_con_flag)
    {
        vTaskDelay(200 / portTICK_PERIOD_MS);   // 给对端一点时间处理 PAUSE
        bt_i2s_driver_uninstall();  //卸载iis驱动器
        bt_i2s_task_shut_down();    //关闭I2S任务并释放环形缓冲区（旧代码漏了这一步）
        bt_con_flag = 0;            //蓝牙连接标志清零（旧代码不清零，导致后续判断/重复反初始化出错）
    }

    /* 把可能被"暂停(btpau)"挂起的任务恢复回可调度状态：
     * 否则再次开启蓝牙（bton）时事件只能进队列却无人处理，界面表现为"完全没反应" */
    bt_tasks_resume();
    write_data_sleep_flag = 0;
    amp_set_power(false);           // 蓝牙关闭 → 关断功放

    /* ---------- 反初始化顺序：**AVRCP 必须先于 A2DP** ----------
     * ESP-IDF 的 esp_avrc_api.h 明确要求 "AVRC should be deinitialized before A2DP"；
     * 反过来写 Bluedroid 会报 "A2DP already deinit, AVRC CT should deinit in advance of
     * A2DP !!!"，并把 AVRC CT/TG 留在"已注册但底层已撤"的坏状态里，随后抛出一串
     * "Invalid AVRC event"、L2CAP PSM 注销失败，重开蓝牙时整栈状态错乱。
     * 旧代码顺序是反的（A2DP 在最前），整机休眠每轮都会踩一次，故在此修正。 */
    // ----------反初始化AVRCP控制器/目标器 ----------
    esp_err_t e_ct = esp_avrc_ct_deinit();
    esp_err_t e_tg = esp_avrc_tg_deinit();
    esp_err_t e_a2d = esp_a2d_sink_deinit();
    if (e_ct != ESP_OK || e_tg != ESP_OK || e_a2d != ESP_OK) {
        ESP_LOGW(BT_AV_TAG, "反初始化返回: avrc_ct=%s avrc_tg=%s a2dp=%s",
                 esp_err_to_name(e_ct), esp_err_to_name(e_tg), esp_err_to_name(e_a2d));
    }
    /*蓝牙反初始化示例*/
    // // ---------- 禁用并反初始化蓝牙协议栈 ----------
    // esp_bluedroid_disable();
    // esp_bluedroid_deinit();
    // // ---------- 禁用并反初始化蓝牙控制器 ----------
    // esp_bt_controller_disable();
    // esp_bt_controller_deinit();

    ESP_LOGI(BT_AV_TAG, "蓝牙功能已关闭（A2DP/AVRCP 已反初始化，协议栈保持上电初始化状态）");
}

/**
 * @brief 开启蓝牙功能（串口屏"蓝牙总开关"开 / 命令 bton）
 *
 * 注意：这里**不能**调用 bt_init()——初始化已在上电执行过，重复初始化控制器/协议栈
 * 会失败。本函数属于"额外的调度任务"：注册 A2DP/AVRCP 回调、恢复任务、进入可发现模式。
 */
void bt_a2dp_work(void)
{
    if (s_bt_a2dp_on) {
        ESP_LOGW(BT_AV_TAG, "蓝牙功能已开启，忽略重复开启");
        return;
    }
    s_bt_a2dp_on = true;

    //启动工作分发任务（内部幂等：已存在则直接复用，不会重复建任务/队列）
    bt_app_task_start_up();
    //若之前被"暂停/关蓝牙"挂起过，先恢复，否则下面分发的 STACK_UP 事件没人处理
    if (s_bt_app_task_handle) {
        vTaskResume(s_bt_app_task_handle);
    }
    //分发调度蓝牙应用的工作，处理蓝牙协议栈启动的事件
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

    ESP_LOGI(BT_AV_TAG, "蓝牙功能已开启（A2DP/AVRCP 已就绪，进入可发现/可连接模式）");
}

bool bt_a2dp_is_on(void)
{
    return s_bt_a2dp_on;
}

/*
 * 整机休眠：双模蓝牙整体开/关
 * ---------------------------------------------------------------------
 * 为什么不用 esp_bt_controller_deinit()/init() 重建协议栈：
 *   IDF 头文件写明控制器 init/deinit "should be called only once"，而
 *   disable → enable 是官方支持的反复循环（换 mode 就是这么做的）。
 * 为什么要连控制器一起关：
 *   控制器使能时 BT 会占用 modem/低功耗时钟，带着它进 light sleep 会把
 *   BT 时钟/状态搞坏；而"整机休眠"本来也要求关掉双模蓝牙。
 * 顺序：AVRCP 暂停（由 power.c 的 bt_sleep() 先发）→ 反初始化 A2DP/AVRCP + 释放 I2S
 *       → BLE 链路收尾 → Bluedroid disable → 控制器 disable。
 */
/**
 * @brief 设置 A2DP 配对用的 GAP 安全参数（SSP IO 能力 + 固定 PIN 1234）
 *
 * bt_init() 上电设置一次；整机休眠唤醒时 Bluedroid 走的是 disable→enable，
 * host 侧初始化会重跑一次（日志里的 "SMP_Register: duplicate registration" 就是它），
 * 所以唤醒路径按同一套参数再设一遍，避免"唤醒后新配对失败"。
 * 内容必须与 bt_init() 里的调用保持一致（只有这一份实现）。
 */
static void bt_apply_gap_security_settings(void)
{
#if (CONFIG_EXAMPLE_A2DP_SINK_SSP_ENABLED == true)
    //设置安全简单配对的默认参数
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    //设置PIN码为1234
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
}

esp_err_t bt_stack_sleep(void)
{
    /* ① 功能层反初始化（幂等：本就关着时只打一条警告）
     * I2S 的收尾顺序很重要：**先停 I2S 任务、再释放通道**。
     * 反过来的话，任务可能正好拿着（已释放的）tx_chan 去 i2s_channel_write，
     * 属于 use-after-free；而且"A2DP 正在连接但还没连上"时 bt_con_flag 仍为 0，
     * bt_app_shutdown() 不会停 I2S 任务，必须在这里兜底。 */
    bt_app_shutdown();
    bt_i2s_task_shut_down();
    bt_i2s_driver_uninstall();

    /* ② BLE 主机链路收尾（停扫描/断链路/注销 GATTC 应用） */
    ble_client_suspend_for_sleep();

    /* ③ 协议栈与控制器整体关闭 */
    esp_err_t err = esp_bluedroid_disable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(BT_AV_TAG, "esp_bluedroid_disable 返回 %s（继续尝试关控制器）", esp_err_to_name(err));
    }
    err = esp_bt_controller_disable();
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "esp_bt_controller_disable 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 保守等待：控制器 disable 会在控制器任务里异步做一次 BLE soft reset
     * （日志里 L2CAP PSM 注销、bta_dm_disable 的 200ms 延时也都是异步收尾），
     * 让控制器任务把收尾做完再进 light sleep，避免"CPU 停了但协议栈还没停干净"。 */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGW(BT_AV_TAG, "双模蓝牙已关闭（Bluedroid + 控制器均已 disable，可以进 light sleep）");
    return ESP_OK;
}

esp_err_t bt_stack_wake(bool a2dp_on)
{
    /* ① 控制器 → ② 协议栈（顺序与 bt_init() 一致；mode 必须与 bt_init() 相同） */
    esp_err_t err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "esp_bt_controller_enable 失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "esp_bluedroid_enable 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 控制器/协议栈刚 enable 完，稍等控制器任务把初始化收尾，
     * 再做 GAP 回调注册、A2DP/AVRCP 注册与 BLE 应用注册，避免与初始化过程打架。
     * 另外 Bluedroid 的 disable→enable 会重跑一次 host 侧初始化
     * （日志里的 "SMP_Register: duplicate registration" 就是它），
     * 所以配对用的 GAP 安全参数在这里按 bt_init() 那套再设一遍，避免配对参数丢失。 */
    vTaskDelay(pdMS_TO_TICKS(100));
    bt_apply_gap_security_settings();

    /* GAP 回调重新注册一次（覆盖式注册，安全）：disable/enable 后不保证还挂着 */
    ble_gap_reregister();

    /* ③ A2DP/AVRCP 功能层：按休眠前的"蓝牙总开关"状态恢复，不擅自替用户打开 */
    if (a2dp_on) {
        bt_a2dp_work();
    } else {
        ESP_LOGW(BT_AV_TAG, "整机唤醒：休眠前蓝牙总开关是关的 → 只恢复控制器/BLE，不注册 A2DP");
    }

    /* ④ BLE 主机链路：重新注册 GATTC 应用（或复用）并开始扫描 */
    ble_client_resume_after_sleep();

    ESP_LOGW(BT_AV_TAG, "双模蓝牙已恢复：控制器/BLE=开，A2DP/AVRCP=%s", a2dp_on ? "开" : "关");
    return ESP_OK;
}

void bt_init(void)
{
    char bda_str[18] = {0};
    //初始化NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));    //释放BLE的内存资源（不使用BLE的情况下）

    //初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize 控制器 失败: %s", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable 控制器 失败: %s", __func__, esp_err_to_name(err));
        return;
    }

    //初始化Bluedroid 协议栈
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        //设置安全简单配对
#if (CONFIG_EXAMPLE_A2DP_SINK_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((err = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize 协议栈 失败: %s", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable 协议栈 失败: %s", __func__, esp_err_to_name(err));
        return;
    }

    //设置安全简单配对 + PIN 码（与整机唤醒路径共用同一份实现）
    bt_apply_gap_security_settings();

    //获取蓝牙设备地址
    ESP_LOGI(BT_AV_TAG, "ESP自身地址:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
