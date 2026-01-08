/*
 * =============================================================================
 * TIM6 周期中断 = 1ms，是全系统的“节拍源”。
 *
 * 本回调内部做的事情可以按“任务调度”理解（虽然没用RTOS）：
 *   - 每 1ms 运行：Key_Tick()、360定时旋转计时
 *   - 每 10ms 运行：置位 g_data_update_flag（主循环刷新UI/输出）
 *   - 每 20ms 运行：180/270 舵机平滑推进（限速 + 死区 + 量化）
 *
 * 设计原则：
 *   1) 中断里不要做耗时操作（printf、长延时、大循环）
 *   2) 中断里只做“必须实时”的最小工作
 *   3) 共享变量要用 volatile；跨上下文写共享变量要有临界区
 */

#include "Timer.h"
#include "Key.h"
#include "Servo.h"
#include "UART.h"

/* main.c 里有 */
extern uint8_t g_data_update_flag;
extern uint8_t mode_flag;

/* ===================== 360 timed rotate（连续旋转“计时转角”） =====================
 * Servo_Rotate_Angle_Timed() 会把“角度”换算成“目标运行时间(ms)”并写入这些变量，
 * Timer 中断每 1ms 对 current_ms++，到点后输出 STOP。
 */
volatile uint16_t g_servo_rotating = 0;
uint16_t g_servo_target_ms = 0;
uint16_t g_servo_current_ms = 0;
uint16_t g_servo_timer_num = 0;

/* ===================== 180 claw smooth（夹爪平滑） ===================== */
volatile uint8_t  g_claw_moving = 0;
volatile uint16_t g_claw_current_pulse = 500;  /* 初值：约 15 度附近（按你公式） */
volatile uint16_t g_claw_target_pulse  = 500;

/* ===================== 270 joint2 smooth（关节平滑） ===================== */
volatile uint8_t  g_joint2_moving = 0;
volatile uint16_t g_joint2_current_pulse = 1500; /* 135度中心 = 1500us */
volatile uint16_t g_joint2_target_pulse  = 1500;

/* 手动方向（来自UART.c） */
extern volatile int8_t g_joint2_dir;
extern volatile int8_t g_claw_dir;

/* 270角度缓存（显示用） */
extern volatile float g_joint2_angle;

/* -----------------------------------------------------------------------------
 * QuantizeUS：输出脉宽量化
 * -----------------------------------------------------------------------------
 * 量化的意义：
 *   舵机内部对PWM脉宽解析并不是无限精度，再加上机械间隙/摩擦，
 *   1~2us 的变化通常只会导致“抖动而非真实位移”。
 *   因此把输出四舍五入到固定粒度 q 的倍数，可显著减少末端抖动。
 */
static inline uint16_t QuantizeUS(uint16_t x, uint16_t q)
{
    return (uint16_t)(((x + (q/2)) / q) * q);
}

/**
 * HAL_TIM_PeriodElapsedCallback：TIM6 的 1ms 周期中断
 * -----------------------------------------------------------------------------
 * 拆分逻辑：
 *   Part1：系统 tick（Key + UI flag）
 *   Part2：360 计时停止
 *   Part3：手动模式下的“方向 -> 端点目标”翻译
 *   Part4：180 平滑推进（20ms）
 *   Part5：270 平滑推进（20ms + 角度缓存）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    static uint16_t count10ms = 0;
    static uint16_t count100ms = 0;

    static uint16_t clawTick = 0;
    static uint16_t joint2Tick = 0;

    /* ---------------- Part1：按键扫描 & UI节拍 ---------------- */
    Key_Tick();

    /* 10ms 置位：主循环看到后可以刷新OLED/输出调试等（避免在中断里做耗时工作） */
    if (++count10ms >= 10)
    {
        count10ms = 0;
        g_data_update_flag = 1;
    }

    /* 100ms 计数器（目前没用，但保留做扩展节拍很方便） */
    if (++count100ms >= 100)
    {
        count100ms = 0;
    }

    /* ---------------- Part2：360“定时转角”到点自动 STOP ---------------- */
    if (g_servo_rotating == 1)
    {
        g_servo_current_ms++;
        if (g_servo_current_ms >= g_servo_target_ms)
        {
            g_servo_rotating = 0;
            Servo_360mode(g_servo_timer_num, STOP);
        }
    }

    /* ---------------- Part3：手动模式下的“方向 -> 端点目标” ----------------
     * 手动交互常见行为：
     *   - 按住按钮：持续朝某方向运动（这里实现为推到某安全端点）
     *   - 松开按钮：停止并保持当前位置
     *
     * 注意：这里不直接写 CCR 输出，而是写 target_pulse/moving，
     *       让后面的 20ms 平滑推进执行实际输出更新。
     */
    if (mode_flag == 2)
    {
        /* 270：方向->目标端点 */
        if (g_joint2_dir > 0)
        {
            /* 手动上推到 180 度 */
            uint16_t pulse = (uint16_t)(180.0f * 2000.0f / 270.0f + 500.0f);
            g_joint2_target_pulse = pulse;
            g_joint2_moving = 1;
        }
        else if (g_joint2_dir < 0)
        {
            /* 手动下推到 60 度（原代码的安全范围） */
            uint16_t pulse = (uint16_t)(60.0f * 2000.0f / 270.0f + 500.0f);
            g_joint2_target_pulse = pulse;
            g_joint2_moving = 1;
        }
        else
        {
            /* 松手：冻结当前位置（保持力矩） */
            g_joint2_target_pulse = g_joint2_current_pulse;
            g_joint2_moving = 0;
        }

        /* 180：方向->目标端点 */
        if (g_claw_dir > 0)
        {
            g_claw_target_pulse = 1833; /* 约 120deg（按 180 线性映射） */
            g_claw_moving = 1;
        }
        else if (g_claw_dir < 0)
        {
            g_claw_target_pulse = 500;  /* 0deg */
            g_claw_moving = 1;
        }
    }

    /* ---------------- Part4：180夹爪平滑推进（每20ms执行一次） ----------------
     * 典型一阶限速逻辑：
     *   - diff 小于死区：直接到位并停止 moving
     *   - 否则：每次向目标靠近 STEP_US
     *   - 最后：量化输出并写 CCR
     */
    if (++clawTick >= CLAW_SMOOTH_INTERVAL_MS)
    {
        clawTick = 0;

        if (g_claw_moving)
        {
            uint16_t diff = (g_claw_current_pulse > g_claw_target_pulse) ?
                            (g_claw_current_pulse - g_claw_target_pulse) :
                            (g_claw_target_pulse - g_claw_current_pulse);

            if (diff <= CLAW_DEADBAND_US)
            {
                g_claw_current_pulse = g_claw_target_pulse;
                g_claw_moving = 0;
            }
            else
            {
                if (g_claw_current_pulse < g_claw_target_pulse)
                {
                    uint16_t d = g_claw_target_pulse - g_claw_current_pulse;
                    g_claw_current_pulse += (d > CLAW_SMOOTH_STEP_US) ? CLAW_SMOOTH_STEP_US : d;
                }
                else
                {
                    uint16_t d = g_claw_current_pulse - g_claw_target_pulse;
                    g_claw_current_pulse -= (d > CLAW_SMOOTH_STEP_US) ? CLAW_SMOOTH_STEP_US : d;
                }
            }

            uint16_t out = QuantizeUS(g_claw_current_pulse, CLAW_QUANT_US);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, out);
        }
    }

    /* ---------------- Part5：270关节平滑推进（每20ms执行一次） ----------------
     * 同样的一阶限速逻辑，但量化粒度更细（2us）以获得更平滑视觉效果。
     * 另外把 out 转回角度缓存 g_joint2_angle，便于 UI 显示/调试。
     */
    if (++joint2Tick >= JOINT2_SMOOTH_INTERVAL_MS)
    {
        joint2Tick = 0;

        if (g_joint2_moving)
        {
            uint16_t diff = (g_joint2_current_pulse > g_joint2_target_pulse) ?
                            (g_joint2_current_pulse - g_joint2_target_pulse) :
                            (g_joint2_target_pulse - g_joint2_current_pulse);

            if (diff <= JOINT2_DEADBAND_US)
            {
                g_joint2_current_pulse = g_joint2_target_pulse;
                g_joint2_moving = 0;
            }
            else
            {
                if (g_joint2_current_pulse < g_joint2_target_pulse)
                {
                    uint16_t d = g_joint2_target_pulse - g_joint2_current_pulse;
                    g_joint2_current_pulse += (d > JOINT2_SMOOTH_STEP_US) ? JOINT2_SMOOTH_STEP_US : d;
                }
                else
                {
                    uint16_t d = g_joint2_current_pulse - g_joint2_target_pulse;
                    g_joint2_current_pulse -= (d > JOINT2_SMOOTH_STEP_US) ? JOINT2_SMOOTH_STEP_US : d;
                }
            }

            uint16_t out = QuantizeUS(g_joint2_current_pulse, JOINT2_QUANT_US);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, out);

            /* 同步角度缓存（显示用） */
            g_joint2_angle = ((float)(out - 500U)) * 270.0f / 2000.0f;
        }
        else
        {
            /* 不移动时也更新角度缓存（保持一致性） */
            uint16_t now = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
            g_joint2_current_pulse = now;
            g_joint2_target_pulse  = now;
            g_joint2_angle = ((float)(now - 500U)) * 270.0f / 2000.0f;
        }
    }
}
