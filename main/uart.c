#include "uart.h"

#define SEND_BUF_SIZE 256  // 格式化缓冲区大小

#define RX_BUF_SIZE  256

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

QueueHandle_t uart_queue;
void rx_task(void *arg);
extern uint8_t battery;

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
    uart_queue = xQueueCreate(10, sizeof(uint64_t));    //创建uart队列
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 20, &uart_queue, 0);
    //        UART编号     Rx缓冲区大小 Tx缓冲区大小 队列大小 句柄 分配中断的标志
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);   //最高任务优先级
    //          入口函数     函数名称      栈深      参数          优先级             句柄
}

void uart_send(const char *format, ...)
{
    const char data_end[] = "\xff\xff\xff";  // 包尾
    size_t end_len = strlen(data_end);       

    va_list args;                // 可变参数列表
    char formatted_buf[SEND_BUF_SIZE];  // 存储“格式化后的字符串”

    //初始化并解析可变参数，格式化到 formatted_buf
    va_start(args, format);
    vsnprintf(formatted_buf, SEND_BUF_SIZE, format, args);
    va_end(args);  // 结束可变参数解析

    size_t formatted_len = strlen(formatted_buf);  // 格式化后字符串的长度

    //动态分配内存：格式化内容长度 + 结尾标记长度 + 1（\0 终止符）
    char *temp = (char *)malloc(formatted_len + end_len + 1);
    if (temp != NULL) {
        // 拼接“格式化内容”和“结尾标记”
        snprintf(temp, formatted_len + end_len + 1, "%s%s", formatted_buf, data_end);
        
        //串口发送
        uart_write_bytes(UART_NUM_2, temp, strlen(temp));
        
        //释放动态内存，避免泄漏
        free(temp);
    } else {
        // （可选）内存分配失败的错误处理
        printf("uart_send: 内存分配失败\n");
    }
}

uint8_t rx[RX_BUF_SIZE + 1];
char ui_res[10];
int parsed_values[3];
void rx_task(void *arg)
{
    //uint8_t* rx = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    
    while(1)
    {
        memset(rx, 0, RX_BUF_SIZE + 1);  // 清空接收缓冲区（可选，若每次读取后处理，也可省略）
        // UART读取：超时10ms，快速响应
        const int length = uart_read_bytes(UART_NUM_2, rx, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (length > 0) {
            // 1. 过滤非打印字符，只保留有效ASCII
            int valid_len = 0;
            for (int i = 0; i < length; i++) {
                // 保留条件：数字 + 大写字母 + 小写字母 + 空格 + 正负符号
                if ((rx[i] >= '0' && rx[i] <= '9') || 
                    (rx[i] >= 'A' && rx[i] <= 'Z') ||  // 大写英文
                    (rx[i] >= 'a' && rx[i] <= 'z') ||  // 小写英文
                    rx[i] == ' ' || rx[i] == '-' || rx[i] == '+') {
                    rx[valid_len++] = rx[i];  // 保留有效字符
                }
            }
            
            // 2. 添加字符串结束符
            rx[valid_len] = '\0';
            
            // 3. 复制到结果缓冲区
            strncpy(ui_res, (const char*)rx, sizeof(ui_res) - 1);
            ui_res[sizeof(ui_res) - 1] = '\0';
            ESP_LOGI("TRUNK", "收到字符串: %s", ui_res);
            
            // // 4. 按空格分割字符串，解析为整数
            // int count = 0;
            // char *token = strtok(ui_res, " ");
            
            // while (token != NULL && count < 3) {  // 最多解析3个数值
            //     parsed_values[count] = atoi(token);
            //     ESP_LOGI("解析结果", "%d: %d", count, parsed_values[count]);
            //     token = strtok(NULL, " ");
            //     count++;
            // }
            
            // 在这里可以使用解析后的parsed_values数组
            // if (count > 0) {
            //     // 示例：处理解析后的数值
            //     ESP_LOGI("数据处理", "共解析到%d个数值", count);
            // }
        }
        uart_send("ba.val=%u",battery); //反馈最近一次检测电量值
        // 短暂延时，降低CPU占用
        vTaskDelay(200/ portTICK_PERIOD_MS);
    }
    //free(rx);
}
