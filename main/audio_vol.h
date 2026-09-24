/**
 * @file audio_vol.h
 * @brief 纯软件音量级配与限幅（PCM5102A 满刻度 → PAM8403 线性输入区）
 *
 * 背景（简要）：
 *   - PAM8403 为固定 24dB 增益的 D 类功放，5V/4Ω 下输入超过约 0.19Vrms 即输出硬削波（炸音），
 *     数据手册明确警告输入过大会损坏芯片；
 *   - PCM5102A 满刻度输出 2.1Vrms，比功放线性上限高约 21dB；
 *   - 硬件电位器保留为"面向用户的主音量旋钮"，因此本模块定位为「安全上限保护」：
 *     标定基准 = 电位器扭到最大 + 手机音量 100%，此时刚好不炸音。
 *
 * 音量共三层相乘，职责分离：
 *   手机绝对音量(手机数字域) × 硬件电位器(模拟域) × 本模块(TRIM + 本机音量)
 *
 * 性能：Q15 定点乘 + 饱和限幅，44.1kHz 立体声约 8.8 万次乘加/秒，占用 <1% CPU。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 可调参数（量产标定只需改这里）
 * ===================================================================== */

/* 固定级配衰减(dB)。理论临界 -21dB（2.1Vrms → 0.19Vrms），
 * 默认 -24dB 作为起点并留约 3dB 余量。
 *
 * 标定方法（实测 5 分钟，无需示波器）：
 *   1) 把功放电位器扭到「最大」（用户能达到的最大位置）；
 *   2) 手机音量拉到 100%，播放低音最重的常用曲子；
 *   3) 仍炸音 → 往更负调（-28/-32）；完全不炸但明显偏小 → 往回调（-21/-22）；
 *   4) 取"再增加 2~3dB 就开始破"的临界值，再回退 3dB 作为最终值写回本宏。 */
#define AUDIO_TRIM_DB           (-24.0f)

/* 本机音量在 TRIM 基础上再往下可调的范围(dB)。
 * 本机音量满量程(127) 时不再额外衰减，只保留 TRIM 保护。 */
#define AUDIO_VOL_RANGE_DB      (-32.0f)

/* 手机绝对音量(AVRCP)映射到本机衰减的范围(dB)。
 *   手机 100%(0x7f) → 不额外衰减（此时总衰减 = AUDIO_TRIM_DB，即标定的安全上限）
 *   手机 0%        → 数字静音
 * 手感调整（手机拖到 50% 时看串口日志打印的"总衰减"）：
 *   总衰减 比最大值低约 11~12dB（即约 -35 ~ -36dB，约为最大功率的 1/12）→ 普通音量旋钮手感
 *   手机要开到很大才够响  → 绝对值调小(如 -18)
 *   最小档位仍然偏响      → 绝对值调大(如 -30) */
#define AUDIO_REMOTE_RANGE_DB   (-24.0f)

#define AUDIO_VOL_MAX           127     /* 音量满量程（与 AVRCP 0x7f 对齐） */
#define AUDIO_VOL_DEFAULT       100     /* 开机默认本机音量（串口屏音量条初始值，0-127） */

/* 减弱系数满量程（千分比）：1000 = 不减弱。睡眠模式（手表 SLEEP=1）用 */
#define AUDIO_DIM_FULL_PERMILLE 1000

/* 限幅器开关：
 *   0 = 仅固定 TRIM（默认，行为完全可预期，不引入动态特性）
 *   1 = 追加分块包络限幅，把平均响度顶到上限（可能有轻微"抽气感"）
 * 建议先跑 0 完成标定；若标定后仍在鼓点处偶发轻微破音再改为 1。 */
#define AUDIO_LIMITER_ENABLE    0

#if AUDIO_LIMITER_ENABLE
#define AUDIO_LIMITER_CEIL      29000   /* 限幅门限，≈ -1dBFS */
#define AUDIO_LIMITER_RELEASE   64      /* 释放系数，越大释放越快（1 阶） */
#endif

/* 手机绝对音量（AVRCP）是否参与增益计算：
 *   1 = 模式B（当前）：安卓在绝对音量模式下把自身数字增益置为满刻度、把衰减责任交给接收端，
 *                    串口日志中能看到随拖动变化的 "AVRC 收到设置音量: N%"。
 *                    本机已向手机声明支持音量变化通知，因此必须由本机执行该衰减，
 *                    否则表现为"手机音量 30% 与 100% 响度完全一样"。
 *   0 = 模式A：部分机型自己会做数字衰减，此时本机不可再叠加（会双重衰减导致声音过小）。
 * 判断方法：拖动手机音量条，30% 与 100% 响度不同→0；相同→1。 */
#define AUDIO_USE_REMOTE_VOL    1

/* =====================================================================
 * 对外接口
 * ===================================================================== */

/**
 * @brief 初始化（预生成增益表），必须在蓝牙起流之前调用
 */
void audio_vol_init(void);

/**
 * @brief 设置本机音量（串口屏音量条 / 本地控制走这里）
 * @param vol 0..127，0 = 数字静音（输出全零）
 */
void audio_vol_set(uint8_t vol);

/**
 * @brief 读取当前本机音量
 * @return 0..127
 */
uint8_t audio_vol_get(void);

/**
 * @brief 上报手机绝对音量（AVRCP SetAbsoluteVolume）
 * @param vol 0..0x7f
 * @note  仅在 AUDIO_USE_REMOTE_VOL = 1 时参与增益计算；始终可用于显示。
 */
void audio_vol_set_remote(uint8_t vol);

/**
 * @brief 读取手机最近一次上报的绝对音量
 * @return 0..0x7f
 */
uint8_t audio_vol_get_remote(void);

/**
 * @brief 读取当前线性增益(Q15)，32768 = 0dB
 */
uint32_t audio_vol_get_gain_q15(void);

/**
 * @brief 读取当前总衰减(dB)，负值
 */
float audio_vol_get_db(void);

/**
 * @brief 对 16bit 交错立体声 PCM 缓冲「就地」施加增益与饱和限幅
 *
 * @param pcm     样本指针（原地修改，调用后数据即被改写）
 * @param samples 样本个数（注意：不是字节数；立体声一帧 = 2 个样本）
 *
 * @note 必须在 i2s_channel_write() 之前调用；可在任意任务上下文调用。
 *       增益为 0dB 时直接返回，零额外开销；增益为 0 时整块写零。
 */
void audio_apply_gain_i16(int16_t *pcm, size_t samples);

/**
 * @brief 设置"减弱系数"（千分比，1000 = 不减弱）
 *
 * @param permille 0~1000；用于手表睡眠标志联动（SLEEP=1 时逐级降到 100 = 10%）
 *
 * @note 该系数乘在 TRIM / 本机音量 / 手机音量之外，**不改变用户设置的音量值**，
 *       因此串口屏音量条显示的仍是用户值；0 视为数字静音。
 */
void audio_vol_set_dim_permille(uint16_t permille);

/**
 * @brief 读取当前减弱系数（千分比）
 */
uint16_t audio_vol_get_dim_permille(void);

#ifdef __cplusplus
}
#endif
