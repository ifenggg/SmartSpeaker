#ifndef __BT_APP_CORE_H__
#define __BT_APP_CORE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define BT_APP_CORE_TAG    "BT_APP_CORE"

/* 事件分发的信号 */
#define BT_APP_SIG_WORK_DISPATCH    (0x01)

/**
 * @brief  处理已调度工作的函数
 *
 * @param [in] event  event id
 * @param [in] param  handler parameter
 */
typedef void (* bt_app_cb_t) (uint16_t event, void *param);

/* 将要发送的消息(函数) */
typedef struct {
    uint16_t       sig;      /*!< 消息发给 bt_app_task */
    uint16_t       event;    /*!< event id */
    bt_app_cb_t    cb;       /*!< 上下文切换回调 */
    void           *param;   /*!< 参数 */
} bt_app_msg_t;

/**
 * @brief  自定义的深度复制函数
 *
 * @param [out] p_dest  指向目标数据的指针
 * @param [in]  p_src   指向源数据的指针
 * @param [in]  len     数据长度以字节为单位
 */
typedef void (* bt_app_copy_cb_t) (void *p_dest, void *p_src, int len);

/**
 * @brief  应用程序任务的事件分发器
 *
 * @param [in] p_cback      回调函数
 * @param [in] event         event id
 * @param [in] p_params      回调的参数
 * @param [in] param_len     参数长度以字节为单位
 * @param [in] p_copy_cback  参数深拷贝函数
 * @return  果工作调度成功则为true，否则为false
 */
bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback);

/**
 * @brief  启动动应用程序任务
 */
void bt_app_task_start_up(void);

/**
 * @brief  关闭应用程序任务
 */
void bt_app_task_shut_down(void);

/**
 * @brief  启动i2s任务
 */
void bt_i2s_task_start_up(void);

/**
 * @brief  关闭I2S任务
 */
void bt_i2s_task_shut_down(void);

/**
 * @brief 将数据写入环缓冲区
 *
 * @param [in] data  指针指向数据流
 * @param [in] size  以字节为单位
 *
 * @return 如果写入成功则为环缓冲区大小，其他情况为0
 */
size_t write_ringbuf(const uint8_t *data, size_t size);

extern uint8_t write_data_sleep_flag;
void bt_page(void);
void bt_sleep(void);
void bt_work(void);
void bt_vo(void);
void bt_ne(void);
void bt_la(void);
void bt_sli(void);
void bt_noi(void);

#endif /* __BT_APP_CORE_H__ */
