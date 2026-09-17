#include <adc.h>
#include "amp.h"
#include "ui.h"     /* 电量推送接口（本阶段已停用：串口屏电量固定 60，见下方 adc_task 注释） */

const static char *adc_TAG = "adc";

/*---------------------------------------------------------------
        ADC 宏
---------------------------------------------------------------*/
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_5   //Pin12

#define EXAMPLE_ADC2_CHAN1          ADC_CHANNEL_6   //Pin14

//衰减模式（最大衰减，即最大测量范围）
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12 

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

QueueHandle_t adc_queue = NULL;     //adc队列句柄
static void adc_task(void* arg);    //任务函数声明
uint8_t battery;

//-------------ADC1 Init---------------//
adc_oneshot_unit_handle_t adc_handle;
adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_2,
};
//-------------ADC1 Config---------------//
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};
//-------------ADC1 校准 Init---------------//
adc_cali_handle_t adc_cali_chan1_handle = NULL;

bool do_calibration_chan0;
bool do_calibration_chan1;
void adc_init(void)
{
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, EXAMPLE_ADC2_CHAN1, &config));

    do_calibration_chan1 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN1, EXAMPLE_ADC_ATTEN, &adc_cali_chan1_handle);

    adc_queue = xQueueCreate(1, sizeof(uint8_t));    //创建gpio队列

    xTaskCreate(adc_task, "adc_task", 4096, NULL, tskIDLE_PRIORITY+1, NULL);      //创建adc任务  (优先级设为仅比空闲任务大1,即最低)
    //        入口函数 函数名称(终端可看) 栈深 参数      优先级        句柄

}

//adc任务函数
static void adc_task(void* arg)
{
    //TaskStatus_t task_status;  // 保存任务状态的结构体
    bool first_round = true;    // 开机首轮：先快速测一次电量，让屏幕右上角尽快有数

    while (1) {
        /* 开机 3 秒后先测一次，之后每分钟测一次 */
        vTaskDelay(pdMS_TO_TICKS(first_round ? 3000 : 60000));
        first_round = false;

        // ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, EXAMPLE_ADC2_CHAN0, &adc_raw[0][0]));
        // ESP_LOGI(adc_TAG, "ADC%d 通道[%d]原始数据: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC2_CHAN0, adc_raw[0][0]);
        // if (do_calibration_chan0) {
        //     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        //     ESP_LOGI(adc_TAG, "ADC%d 通道[%d]校准电压: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC2_CHAN0, voltage[0][0]);
        // }
        // vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, EXAMPLE_ADC2_CHAN1, &adc_raw[0][1]));
        ESP_LOGI(adc_TAG, "ADC%d 通道[%d]原始数据: %d", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN1, adc_raw[0][1]);
        if (do_calibration_chan1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
            ESP_LOGI(adc_TAG, "ADC%d 通道[%d]校准电压: %d mV", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN1, voltage[0][1]);
        }
        if(voltage[0][1]>=3200)
            battery = 100;
        else if(voltage[0][1]>=3000)
            battery = 90;
        else if(voltage[0][1]>=2800)
            battery = 80;
        else if(voltage[0][1]>=2600)
            battery = 70;
        else if(voltage[0][1]>=2400)
            battery = 60;
        else if(voltage[0][1]>=2200)
            battery = 50;
        else if(voltage[0][1]>=2000)
            battery = 40;
        else if(voltage[0][1]>=1800)
            battery = 30;
        else if(voltage[0][1]>=1600)
            battery = 20;
        else
            battery = 0;
        ESP_LOGI("adc", "电量%d",battery);
        /* 【本阶段停用】按要求：不能初始化 ADC、也不能调用 ADC 电量采集。
         * 串口屏显示的电量固定为 60（见 main/ui.c 的 s_battery 初值），
         * 因此这里不再把 ADC 结果推给屏幕。
         * 硬件确认（电池分压接到 GPIO14 / ADC2_CH6）后，恢复下面这一行即可。 */
        // ui_set_battery(battery);
        // 低电量联动功放：低于阈值关断，恢复后开启
        if (battery <= AMP_LOW_BATT_OFF) {
            amp_set_low_batt_off(true);
        } else if (battery >= AMP_LOW_BATT_RECOVER) {
            amp_set_low_batt_off(false);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        // vTaskGetInfo(
        //     NULL,               // 传入NULL表示查询“当前任务”
        //     &task_status,       // 保存结果的结构体
        //     pdTRUE,             // pdTRUE表示需要获取栈信息
        //     eInvalid            // 不需要任务状态码
        // );
        // ESP_LOGI("stack_info", "剩余栈空间最小值（高水位线）: %lu 字", task_status.usStackHighWaterMark);
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    if (do_calibration_chan1) {
        example_adc_calibration_deinit(adc_cali_chan1_handle);
    }
}

/*---------------------------------------------------------------
        ADC 校准
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    //线性拟合
    if (!calibrated) {
        ESP_LOGI(adc_TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    //错误处理
    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(adc_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(adc_TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(adc_TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

//校准反初始化
static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(adc_TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
}
