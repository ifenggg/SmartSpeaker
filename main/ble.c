/**
 * @file ble.c
 * @brief 【已废弃 / 已移出编译】早期 BLE 服务端 + 客户端示例实现
 *
 * ⚠ 本文件当前**不参与编译**（见 main/CMakeLists.txt）。
 *
 * 废弃原因：
 *   1. 本文件内部的 ble_gatt_init() 会再次注册 BLE GAP 回调，而 Bluedroid 只保存一个 GAP 回调
 *      （esp_ble_gap_register_callback → btc_profile_cb_set，覆盖式赋值），会把 ble_client/ble_server
 *      注册的回调顶掉，造成"扫不到/收不到广播完成事件"这类隐蔽故障；
 *   2. 它会注册额外的 GATTS/GATTC 应用，并调用 esp_ble_gap_config_local_privacy(true) 把本地地址
 *      切成 RPA，与"显式使用公共地址 + 显式地址类型"的建链策略冲突（0x3e 排查项之一）；
 *   3. 功能已由下列模块替代：
 *        ble_gap.c    —— GAP 事件统一分发（唯一系统回调）
 *        ble_server.c —— BLE 从机（手机 BLE 调试助手接入，GATT Server）
 *        ble_client.c —— BLE 主机（扫描并连接手表，GATT Client）
 *
 * 如需查阅历史实现请使用 git 历史（提交 5285f9f 等），不要重新加入编译。
 */
#if 0   /* 整文件停用：保留源码便于对照，避免与 ble_gap/ble_server/ble_client 冲突 */

/* 本地设备名称 */
#define BLE_ADV_NAME                "speaker-BLE"

static const char remote_device_name[] = "KT6368A-BLE-2.1";//"Ace 2V";//"iQOO Neo8";      //要自动连接的远程ble服务端名字（双机通信）KT6368A-BLE-2.1

/* BLE defines */
#define GATTS_SERVICE_UUID_A        0x00FF
#define GATTS_CHAR_UUID_A           0xFF01
#define GATTS_DESCR_UUID_A          0x3333
#define GATTS_NUM_HANDLE_A          4

#define GATTS_SERVICE_UUID_B        0x00EE
#define GATTS_CHAR_UUID_B           0xEE01
#define GATTS_DESCR_UUID_B          0x2222
#define GATTS_NUM_HANDLE_B          4

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE        1024
#define PROFILE_NUM                 2
#define PROFILE_NUM1                1
#define PROFILE_A_APP_ID            0
#define PROFILE_B_APP_ID            1
#define PROFILE_C_APP_ID            0
#define INVALID_HANDLE              0

#define GATTC_TAG                   "GATTCli"
#define BT_BLE_COEX_TAG             "GATTSer"
#define REMOTE_SERVICE_UUID        0xFFF0
#define REMOTE_NOTIFY_CHAR_UUID    0xFFF1
#define REMOTE_WRITE_CHAR_UUID     0xFFF1

extern QueueHandle_t led_queue;     //灯带队列句柄
// extern uint8_t leds_flag;           //灯带初始化标志位

// 定义KT设备的特征值UUID（0000fff1-0000-1000-8000-00805f9b34fb）
// static esp_bt_uuid_t remote_filter_char_uuid = {
//     .len = ESP_UUID_LEN_128,  // 128位UUID
//     .uuid = {.uuid128 = {
//         0xfb, 0x34, 0x9b, 0x5f, 
//         0x80, 0x00, 0x00, 0x80, 
//         0x00, 0x10, 0x00, 0x00, 
//         0xf1, 0xff, 0x00, 0x00  // 上面转换的字节数组
//     }}
// };

/*ble客户端变量*/

uint16_t connid;
uint8_t sendflag;
esp_gatt_if_t gattcif;

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t remote_filter_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_WRITE_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_char_elem_t *char_elem   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst glcc_profile_tab[PROFILE_NUM1] = {
    [PROFILE_C_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* ---------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------
    *****************************BLE-服务端初始化部分 
    ---------------------------------------------------------------------------------------------------------------------------------------------
    ---------------------------------------------------------------------------------------------------------------------------------------------
*/

uint8_t ble_rec_buf[20];

/* event for stack up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;

//Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static uint8_t ble_char_value_str[] = {0x11, 0x22, 0x33};
esp_gatt_char_prop_t a_property = 0;
esp_gatt_char_prop_t b_property = 0;

esp_attr_value_t gatts_initial_char_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(ble_char_value_str),
    .attr_value   = ble_char_value_str,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x060,
    .adv_int_max        = 0x060,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_B_APP_ID] = {
        .gatts_cb = gatts_profile_b_event_handler,/* This demo does not implement, similar as profile A */
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }
};
static void ble_init_adv_data(const char *name)
{
    int len = strlen(name);
    uint8_t raw_adv_data[len+5];
    //flag
    raw_adv_data[0] = 2;
    raw_adv_data[1] = ESP_BT_EIR_TYPE_FLAGS;
    raw_adv_data[2] = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    //adv name
    raw_adv_data[3] = len + 1;
    raw_adv_data[4] = ESP_BLE_AD_TYPE_NAME_CMPL;
    for (int i = 0;i < len;i++)
    {
        raw_adv_data[i+5] = *(name++);
    }
    //The length of adv data must be less than 31 bytes
    esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (raw_adv_ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "config raw adv data failed, error code = 0x%x ", raw_adv_ret);
    }
    esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (raw_scan_ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "config raw scan rsp data failed, error code = 0x%x", raw_scan_ret);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        //esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BT_BLE_COEX_TAG, "Advertising start failed");
        }else {
            ESP_LOGI(BT_BLE_COEX_TAG, "开始广播，蓝牙可被连接");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BT_BLE_COEX_TAG, "Advertising stop failed");
        }
        else {
            ESP_LOGI(BT_BLE_COEX_TAG, "停止广播");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(BT_BLE_COEX_TAG, "更新连接参数状态 = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;

        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "开始扫描附近设备");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            //esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            //ESP_LOGI(GATTC_TAG, "搜索到的广播数据长度 %d, 扫描响应长度 %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            //ESP_LOGI(GATTC_TAG, "搜索到的设备名称长度 %d", adv_name_len);
            //esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
            //ESP_LOGI(GATTC_TAG, " ");

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    //ESP_LOGI(GATTC_TAG, "搜索到的设备： %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "连接到远程设备.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(glcc_profile_tab[PROFILE_C_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "停止扫描附近设备");
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "数据包长度已更新: rx = %d, tx = %d, status = %d",
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len,
                  param->pkt_data_length_cmpl.status);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep) {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_OFFSET;
            } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }

            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(BT_BLE_COEX_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK){
                    ESP_LOGE(BT_BLE_COEX_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            } else {
                ESP_LOGE(BT_BLE_COEX_TAG, "%s, malloc failed", __func__);
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(BT_BLE_COEX_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(BT_BLE_COEX_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        esp_ble_gap_config_local_privacy(true);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_A;
        //init BLE adv data and scan response data
        ble_init_adv_data(BLE_ADV_NAME);
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(BT_BLE_COEX_TAG, "GATT_READ_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(BT_BLE_COEX_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(BT_BLE_COEX_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex("BLE服务端收到数据", param->write.value, param->write.len);
            
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(BT_BLE_COEX_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(BT_BLE_COEX_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(BT_BLE_COEX_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(BT_BLE_COEX_TAG, "unknown descr value");
                    esp_log_buffer_hex(BT_BLE_COEX_TAG, param->write.value, param->write.len);
                }

            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_initial_char_val, NULL);
        if (add_char_ret){
            ESP_LOGE(BT_BLE_COEX_TAG, "add char failed, error code = 0x%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        ESP_LOGI(BT_BLE_COEX_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(BT_BLE_COEX_TAG, "add char descr failed, error code = 0x%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(BT_BLE_COEX_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "ESP_GATTS_DISCONNECT_EVT");
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(BT_BLE_COEX_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_B;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_B);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(BT_BLE_COEX_TAG, "GATT_READ_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        //ESP_LOGI(BT_BLE_COEX_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %"PRIu32", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            // ESP_LOGI(BT_BLE_COEX_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            // //esp_log_buffer_hex("服务端数据:", param->write.value, param->write.len);
            // for (int i = 0; i < param->write.len; i++) {
            //     // 以十进制格式打印每个字节，每个数值占3位（0-255），用空格分隔
            //     esp_log_write(ESP_LOG_INFO, "服务端数据", "%3d ", param->write.value[i]);
            // }
            // ESP_LOGI("data", "BLE服务端收到数据");
            //     memcpy(ble_rec_buf, param->write.value, param->write.len);
            // ESP_LOGI("data", "收到颜色数据= %d", ble_rec_buf[0]);
            // ESP_LOGI("data", "收到颜色数据= %d", ble_rec_buf[1]);
            // ESP_LOGI("data", "收到颜色数据= %d", ble_rec_buf[2]);
            uint8_t *recv_data = param->write.value;
            int recv_len = param->write.len;
            
            // 1. 将字节数组转换为字符串（添加结束符）
            char str_buf[recv_len + 1];
            memcpy(str_buf, recv_data, recv_len);
            str_buf[recv_len] = '\0'; // 字符串结束符
            ESP_LOGI("BLE服务端","接收到字符串");
            
            // 2. 按空格分割字符串，解析为整数
            int count = 0;       // 实际解析的数值数量
            char *token = strtok(str_buf, " "); // 按空格分割
            
            while (token != NULL && count < 3) {
                // 将字符串转换为十进制整数
                ble_rec_buf[count] = atoi(token);
                //ESP_LOGI("解析数值", "%d: %d", count, ble_rec_buf[count]);
                token = strtok(NULL, " "); // 下一个子串
                count++;
            }
            
            // 3. 此时 values[0]、values[1]、values[2] 即为解析后的 64、25、31
            if (count == 3) {
                ESP_LOGI("解析完成", "三个数值: %d, %d, %d", ble_rec_buf[0], ble_rec_buf[1], ble_rec_buf[2]);
            } else {
                ESP_LOGW("解析警告", "解析到 %d 个数值（预期3个）", count);
            }
            uint32_t gpio_num;
            xQueueSendFromISR(led_queue, &gpio_num, NULL);
            // if(leds_flag)
            //     ESP_LOGI("BLE服务端","灯带已响应"); 
            // else
            //     ESP_LOGI("BLE服务端","灯带未初始化");

            if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(BT_BLE_COEX_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(BT_BLE_COEX_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(BT_BLE_COEX_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(BT_BLE_COEX_TAG, "unknown value");
                }

            }
        }
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_B;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        b_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(BT_BLE_COEX_TAG, "add char failed, error code = 0x%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(BT_BLE_COEX_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BT_BLE_COEX_TAG, "ESP_GATTS_CONF_EVT status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(BT_BLE_COEX_TAG, param->conf.value, param->conf.len);
        }
    break;
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(BT_BLE_COEX_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------
    *****************************BLE-客户端初始化部分
    ---------------------------------------------------------------------------------------------------------------------------------------------
    ---------------------------------------------------------------------------------------------------------------------------------------------
*/

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        glcc_profile_tab[PROFILE_C_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(glcc_profile_tab[PROFILE_C_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "远程蓝牙设备地址:");
        esp_log_buffer_hex(GATTC_TAG, glcc_profile_tab[PROFILE_C_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "发现服务failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "发现服务完成连接ID %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"配置 mtu failed, error 状态 = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    //搜索服务结果事件
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        //检查发现的服务 UUID 是否与目标服务 UUID (REMOTE_SERVICE_UUID) 匹配
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            glcc_profile_tab[PROFILE_C_APP_ID].service_start_handle = p_data->search_res.start_handle;
            glcc_profile_tab[PROFILE_C_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "搜索服务失败, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "从远程设备获取服务信息");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "从闪存获取服务信息");
        } else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        /*发现该服务中的特征，并注册通知*/
        if (get_server) //若找到目标服务
        {
            //获取特征数量
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,    //查询类型为特征
                                                                     glcc_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                                     glcc_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            ESP_LOGI("ble","1获取特征数量");
            if (count > 0){
                //为特征元素数组分配内存
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    connid=p_data->search_cmpl.conn_id;
                    gattcif=gattc_if;
                    //根据 UUID 获取特征的详细信息
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             glcc_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                             glcc_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,   //要查找的特定特征的 UUID
                                                             char_elem_result,
                                                             &count);
                    //若找到，处理找到的特征
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }

                    ESP_LOGI("ble","count:%d",count);
                    ESP_LOGI("ble","count:%d",(char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY));
                    //检查第一个特征是否支持通知功能，若支持，保存特征句柄并注册通知
                    if (count > 0){
                        ESP_LOGI("ble","2-2注册通知");
                        glcc_profile_tab[PROFILE_C_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, glcc_profile_tab[PROFILE_C_APP_ID].remote_bda, char_elem_result[0].char_handle);
                        ESP_LOGI("ble","2-1注册通知");
                    }
                    ESP_LOGI("ble","2注册通知");
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    //注册通知事件
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "向服务器注册接收特征");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "注册特征失败: 错误状态 ： %d", p_data->reg_for_notify.status);
        }else{
            //获取描述符数量
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         glcc_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,    //查询类型为描述符
                                                                         glcc_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                                                         glcc_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                                                         glcc_profile_tab[PROFILE_C_APP_ID].char_handle,    //限定在特定特征句柄下查找描述符
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            //获取描述符详细信息
            if (count > 0){
                //为描述符分配内存
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }else{
                    //根据特征句柄和描述符的UUID获取描述符的详细信息
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         glcc_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid, //通常为0x2902（不变）
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        free(descr_elem_result);
                        descr_elem_result = NULL;
                        break;
                    }
                    //检查并写入客户端特征配置描述符（CCCD）
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        //向CCCD写入值来启用通知
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     glcc_profile_tab[PROFILE_C_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en), 
                                                                     (uint8_t *)&notify_en,     //写入的值（0x0001启用通知，0x0002启用指示）
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }
                    ESP_LOGI("ble","3写入");
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "描述符写入操作失败");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    //收到来自服务端的通知或指示
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "收到通知，值:");   //通知不需要客户端确认
        }else{
            ESP_LOGI(GATTC_TAG, "收到指示，值:");   //指示需要客户端确认
        }
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        uint8_t data_buf[10];  // 用于存储8字节数据
        
        // 复制8字节数据到缓冲区
        memcpy(data_buf, p_data->notify.value, p_data->notify.value_len);
        ESP_LOGI("data", "收到数据包头= %d", data_buf[0]);
        // 这里添加对8字节数据的处理逻辑

        sendflag=1;
        ESP_LOGI(GATTC_TAG, "BLE客户端收到数据包通知数据");
        break;
    //描述符写入完成事件
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI("ble","4描述符已写入");
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "描述符写入失败，错误状态 ： %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "描述符写入成功，向服务端发送数据 ");
        //向服务器的特征写入数据
        // uint8_t write_char_data[35];
        // for (int i = 0; i < sizeof(write_char_data); ++i)
        // {
        //     write_char_data[i] = i % 256;
        // }
        // esp_ble_gattc_write_char( gattc_if,
        //                           glcc_profile_tab[PROFILE_C_APP_ID].conn_id,
        //                           glcc_profile_tab[PROFILE_C_APP_ID].char_handle,   //写入的特征句柄
        //                           sizeof(write_char_data),
        //                           write_char_data,  //写入的数据   
        //                           ESP_GATT_WRITE_TYPE_RSP,
        //                           ESP_GATT_AUTH_REQ_NONE);  //要求服务端响应
        break;
    //服务变更事件
    case ESP_GATTC_SRVC_CHG_EVT: {
        //记录远程设备的蓝牙地址
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    //特征写入完成事件
    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGI("ble","5特征已写入");
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "特征写入失败，错误状态 ： %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "特征写入成功 ");
        break;
    // 断开连接事件
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "连接断开, 原因 ： %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

void ble_send(void)
{
    ESP_LOGI("ble", "向服务端发送数据 ");
    uint16_t cnt = 1;
    char_elem = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * cnt);
    esp_ble_gattc_get_char_by_uuid( gattcif,
                                    connid,
                                    glcc_profile_tab[PROFILE_C_APP_ID].service_start_handle,
                                    glcc_profile_tab[PROFILE_C_APP_ID].service_end_handle,
                                    remote_filter_uuid,   //要查找的特定特征的 UUID
                                    char_elem,
                                    &cnt);
    glcc_profile_tab[PROFILE_C_APP_ID].char_handle = char_elem[0].char_handle;
    //向服务器的特征写入数据
    uint8_t write_char_data[35];
    for (int i = 0; i < sizeof(write_char_data); ++i)
    {
        write_char_data[i] = i % 256;
    }
    esp_ble_gattc_write_char( gattcif,
                                glcc_profile_tab[PROFILE_C_APP_ID].conn_id,
                                glcc_profile_tab[PROFILE_C_APP_ID].char_handle,   //写入的特征句柄
                                sizeof(write_char_data),
                                write_char_data,  //写入的数据   
                                ESP_GATT_WRITE_TYPE_NO_RSP,
                                ESP_GATT_AUTH_REQ_NONE);  //要求服务端响应
    ESP_LOGI("ble", "已经向服务端发送数据 ");
}

//配置文件（Profile）的事件分发器
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* 注册事件处理 */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            //将分配到的 GATT 接口 ID (gattc_if) 存储到对应的配置文件表中
            glcc_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    //事件分发
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM1; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == glcc_profile_tab[idx].gattc_if) {
                if (glcc_profile_tab[idx].gattc_cb) {
                    glcc_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

/*------------------------------------------------------------------
        GATT最终初始化
    ----------------------------------------------------------------
*/

void ble_gatt_init(void)
{
    esp_err_t ret;
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "gap register error, error code = 0x%x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "set local  MTU failed, error code = 0x%x", local_mtu_ret);
    }
    ESP_LOGI("BLE配置:", "已完成");
}

void ble_gatts_init(void)
{
    esp_err_t ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "gatts register error, error code = 0x%x", ret);
        return;
    } 
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "gatts app register error, error code = 0x%x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        ESP_LOGE(BT_BLE_COEX_TAG, "gatts app register error, error code = 0x%x", ret);
        return;
    }
    ESP_LOGI("BLE服务端:", "已开启");
}

void ble_gattc_init(void)
{
    esp_err_t ret;
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_C_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    ESP_LOGI("BLE客户端:", "已开启");
}

#endif  /* 整文件停用（见文件头说明） */
