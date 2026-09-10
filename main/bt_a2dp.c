#include <stdio.h>
#include <stdlib.h>
#include <bt_a2dp.h>

#define LOCAL_DEVICE_NAME    "ESP_speaker"

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
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
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
        uart_send("t1.txt=\"已连接\"");
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
        esp_ble_gap_set_device_name(LOCAL_DEVICE_NAME);
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

void bt_app_start(void)
{
    /*蓝牙反初始化后重建示例*/
    // esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    // esp_bt_controller_init(&bt_cfg);
    // esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    // esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    // esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    // esp_bluedroid_enable();

    // //设置安全简单配对的默认参数
    // esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    // esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    // esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    // //设置PIN码为1234
    // esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    // esp_bt_pin_code_t pin_code;
    // pin_code[0] = '1';
    // pin_code[1] = '2';
    // pin_code[2] = '3';
    // pin_code[3] = '4';
    // esp_bt_gap_set_pin(pin_type, 4, pin_code);
    //恢复任务
    bt_app_task_start_up();
    vTaskResume(s_bt_app_task_handle);
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
    
    ESP_LOGI("bt_work", "任务已恢复！");
    //bt_work();
}

void bt_app_shutdown(void)
{
    esp_avrc_ct_send_passthrough_cmd(
            0,
        ESP_AVRC_PT_CMD_PAUSE,
        ESP_AVRC_PT_CMD_STATE_PRESSED
    );
    if(bt_con_flag)
    {
        //挂起任务
        bt_sleep();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        bt_i2s_driver_uninstall();  //卸载iis驱动器
        bt_con_flag = 0; //蓝牙连接标志清零
    }
    // ---------- 反初始化A2DP Sink ----------
    esp_a2d_sink_deinit();
    // ----------反初始化AVRCP控制器/目标器 ----------
    esp_avrc_ct_deinit();
    esp_avrc_tg_deinit();
    /*蓝牙反初始化示例*/
    // // ---------- 禁用并反初始化蓝牙协议栈 ----------
    // esp_bluedroid_disable();
    // esp_bluedroid_deinit();
    // // ---------- 禁用并反初始化蓝牙控制器 ----------
    // esp_bt_controller_disable();
    // esp_bt_controller_deinit();
    
    ESP_LOGI(BT_AV_TAG, "蓝牙已反初始化");
}

void bt_a2dp_work(void)
{
    //启动工作分发任务
    bt_app_task_start_up();
    //分发调度蓝牙应用的工作，处理蓝牙协议栈启动的事件
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
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

    //获取蓝牙设备地址
    ESP_LOGI(BT_AV_TAG, "ESP自身地址:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
