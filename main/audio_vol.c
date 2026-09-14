/**
 * @file audio_vol.c
 * @brief 纯软件音量级配与限幅实现
 *
 * 实现要点：
 *   - 增益用 Q15 定点表示（32768 = 0dB），只在初始化/改音量时做一次浮点运算，
 *     音频路径上只做一次整数乘 + 饱和，适合在 160MHz 下实时运行；
 *   - 增益值用 32bit 原子读写（volatile uint32_t），音频路径无需加锁；
 *   - 带舍入的定点乘（+16384 >> 15），避免小音量时产生量化直流偏移；
 *   - 0dB 直通、0 增益整块写零，两条快速路径避免无谓运算。
 */
#include <math.h>
#include <string.h>

#include "audio_vol.h"
#include "sys/lock.h"
#include "esp_log.h"

static const char *TAG = "audio_vol";

/* 限幅器内部实现（仅在 AUDIO_LIMITER_ENABLE 时编译） */
#if AUDIO_LIMITER_ENABLE
static void audio_limit_block(int16_t *pcm, size_t samples)
{
    static int32_t s_env = 0;   /* 包络峰值（持久化在块之间） */

    int32_t peak = 0;
    for (size_t i = 0; i < samples; i++) {
        int32_t a = (pcm[i] < 0) ? -(int32_t)pcm[i] : (int32_t)pcm[i];
        if (a > peak) {
            peak = a;
        }
    }

    /* 快攻慢放：攻击即时，释放按 1 阶指数回落 */
    if (peak > s_env) {
        s_env = peak;
    } else {
        s_env -= (s_env - peak + AUDIO_LIMITER_RELEASE - 1) / AUDIO_LIMITER_RELEASE;
    }

    if (s_env > AUDIO_LIMITER_CEIL) {
        /* 整块按比例回退，保证峰值不越限 */
        int32_t g = (int32_t)(((int64_t)AUDIO_LIMITER_CEIL << 15) / s_env);
        for (size_t i = 0; i < samples; i++) {
            int32_t v = ((int32_t)pcm[i] * g) >> 15;
            if (v > 32767) {
                v = 32767;
            } else if (v < -32768) {
                v = -32768;
            }
            pcm[i] = (int16_t)v;
        }
    }
}
#endif /* AUDIO_LIMITER_ENABLE */

/* =====================================================================
 * 内部状态
 * ===================================================================== */

static _lock_t           s_lock;                       /* 保护多步更新（仅设置路径使用） */
static volatile uint32_t s_gain_q15   = 32768u;        /* Q15 线性增益，32768 = 0dB */
static volatile float    s_gain_db    = 0.0f;          /* 当前总衰减(dB)，仅用于显示 */
static volatile uint8_t  s_vol        = AUDIO_VOL_MAX; /* 本机音量 0..127 */
static volatile uint8_t  s_vol_remote = 0x7f;          /* 手机绝对音量 0..0x7f */

/* =====================================================================
 * 内部函数
 * ===================================================================== */

/**
 * @brief dB → Q15 线性增益
 * @param db 衰减量（负值）
 * @return Q15 增益；≤ -96dB 视为 0（静音）
 */
static uint32_t audio_db_to_q15(float db)
{
    if (db <= -96.0f) {
        return 0u;
    }
    float g = powf(10.0f, db / 20.0f) * 32768.0f;
    if (g >= 32768.0f) {
        g = 32768.0f;
    }
    return (uint32_t)(g + 0.5f);
}

/**
 * @brief 依据当前音量重算增益（调用者须持有 s_lock）
 */
static void audio_update_gain_locked(void)
{
    if (s_vol == 0) {
        /* 本机音量 0 → 数字静音 */
        s_gain_db = -96.0f;
        s_gain_q15 = 0u;
        return;
    }

    /* 基准衰减 + 本机音量贡献 */
    float db = AUDIO_TRIM_DB;
    db += AUDIO_VOL_RANGE_DB * (1.0f - (float)s_vol / (float)AUDIO_VOL_MAX);

#if AUDIO_USE_REMOTE_VOL
    /* 模式B：手机把衰减责任交给接收端（本机已声明支持音量变化通知），
     * 必须由本机代其衰减，否则手机音量条不起作用。
     * 手机 0% → 数字静音，与手机侧显示保持一致。 */
    if (s_vol_remote == 0) {
        s_gain_db = -96.0f;
        s_gain_q15 = 0u;
        return;
    }
    db += AUDIO_REMOTE_RANGE_DB * (1.0f - (float)s_vol_remote / 127.0f);
#endif

    s_gain_db  = db;
    s_gain_q15 = audio_db_to_q15(db);
}

/* =====================================================================
 * 对外接口
 * ===================================================================== */

void audio_vol_init(void)
{
    _lock_acquire(&s_lock);
    s_vol = AUDIO_VOL_DEFAULT;
    audio_update_gain_locked();
    _lock_release(&s_lock);

    ESP_LOGI(TAG, "音量级配就绪: TRIM=%.1fdB 本机音量=%u 手机绝对音量=%s 增益=%lu/32768 (总衰减 %.1fdB)",
             (double)AUDIO_TRIM_DB, (unsigned)s_vol,
#if AUDIO_USE_REMOTE_VOL
             "参与(模式B)",
#else
             "忽略(模式A)",
#endif
             (unsigned long)s_gain_q15, (double)s_gain_db);
}

void audio_vol_set(uint8_t vol)
{
    if (vol > AUDIO_VOL_MAX) {
        vol = AUDIO_VOL_MAX;
    }

    _lock_acquire(&s_lock);
    s_vol = vol;
    audio_update_gain_locked();
    _lock_release(&s_lock);

    ESP_LOGI(TAG, "本机音量 -> %u (增益 %lu/32768, 总衰减 %.1fdB)",
             (unsigned)vol, (unsigned long)s_gain_q15, (double)s_gain_db);
}

uint8_t audio_vol_get(void)
{
    return s_vol;
}

void audio_vol_set_remote(uint8_t vol)
{
    if (vol > 0x7f) {
        vol = 0x7f;
    }

    _lock_acquire(&s_lock);
    s_vol_remote = vol;
    audio_update_gain_locked();
    _lock_release(&s_lock);
}

uint8_t audio_vol_get_remote(void)
{
    return s_vol_remote;
}

uint32_t audio_vol_get_gain_q15(void)
{
    return s_gain_q15;
}

float audio_vol_get_db(void)
{
    return s_gain_db;
}

void audio_apply_gain_i16(int16_t *pcm, size_t samples)
{
    if (pcm == NULL || samples == 0) {
        return;
    }

    uint32_t g = s_gain_q15;   /* 32bit 原子读，音频路径不加锁 */

    if (g == 32768u) {
        return;                /* 0dB：直通，零开销 */
    }
    if (g == 0u) {
        /* 数字静音：整块写零（比逐样本乘更省，且保证绝对静音） */
        memset(pcm, 0, samples * sizeof(int16_t));
        return;
    }

    for (size_t i = 0; i < samples; i++) {
        /* 带舍入的定点乘；pcm[i]*g 最大 |−32768×32768| = 2^30，不溢出 int32 */
        int32_t v = ((int32_t)pcm[i] * (int32_t)g + 16384) >> 15;
        if (v > 32767) {
            v = 32767;         /* 饱和限幅：防溢出，同时防功放过载 */
        } else if (v < -32768) {
            v = -32768;
        }
        pcm[i] = (int16_t)v;
    }

#if AUDIO_LIMITER_ENABLE
    audio_limit_block(pcm, samples);
#endif
}
