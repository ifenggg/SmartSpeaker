/**
 * @file amp.h
 * @brief PAM8403 D 类功放驱动（静音 / 电源关断 / 功耗联动）
 *
 * 引脚逻辑（严格遵循）：
 *   MUTE：拉低 = 静音（无输出）；拉高 = 取消静音（正常输出）
 *   SHDN：拉高 = 正常工作；拉低 = 休眠关断（极低功耗）
 */
#pragma once

#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 引脚定义（可快速修改）
 * ===================================================================== */
#define AMP_MUTE_GPIO           GPIO_NUM_15   /* MUTE：拉低静音，拉高取消静音 */
#define AMP_SHDN_GPIO           GPIO_NUM_2    /* SHDN：拉高正常工作，拉低关断 */

/* =====================================================================
 * 参数（可调）
 * ===================================================================== */
#define AMP_POWER_ON_DELAY_MS   200    /* 上电软启动延时（ms），防开机爆音 */
#define AMP_PAUSE_OFF_DELAY_MS  30000  /* 暂停后自动关断延时（ms），0 = 立即关断 */

/* 低电量联动阈值（%） */
#define AMP_LOW_BATT_OFF        10     /* 电量 ≤10% 强制关断功放 */
#define AMP_LOW_BATT_RECOVER    30     /* 电量 ≥30% 恢复工作（带回滞） */

/* =====================================================================
 * 对外接口
 * ===================================================================== */

/**
 * @brief 初始化：上电默认 SHDN=1(开启)、MUTE=1(非静音)，带软启动防爆音
 */
void amp_init(void);

/**
 * @brief 静音控制
 * @param mute true=静音(MUTE 拉低) false=取消静音(MUTE 拉高)
 */
void amp_set_mute(bool mute);

/**
 * @brief 电源控制
 * @param on true=正常工作(SHDN 拉高) false=关断休眠(SHDN 拉低)
 */
void amp_set_power(bool on);

/**
 * @brief 延时关断（用于暂停超时；期间若调用 amp_set_power(true) 则取消）
 * @param delay_ms 延时毫秒；0 = 立即关断
 */
void amp_power_off_delayed(uint32_t delay_ms);

/**
 * @brief 低电量强制关断 / 恢复（优先级最高：关断期间禁止其他逻辑开启）
 * @param off true=强制关断 false=恢复
 */
void amp_set_low_batt_off(bool off);

/**
 * @brief 状态查询
 * @return 当前静音状态
 */
bool amp_is_muted(void);

/**
 * @brief 状态查询
 * @return 功放电源是否开启
 */
bool amp_is_powered(void);

#ifdef __cplusplus
}
#endif
