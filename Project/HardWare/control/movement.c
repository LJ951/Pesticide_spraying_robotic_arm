/*
 * =============================================================================
 * 这个文件解决的问题是：把“动作语义”组织起来，让上层不用关心 PWM 细节。
 *
 * ---------------------------------------------------------------------------
 * 1) 模块职责边界（非常重要，写项目报告/答辩常问）
 * ---------------------------------------------------------------------------
 * movement.c 只做：
 *   - “动作脚本/编排”：先做什么、再做什么、持续多久
 *   - “动作意图设置”：关节目标角度、夹爪目标角度、底座转动角度
 *
 * movement.c 不做：
 *   - PWM 计算细节（交给 Servo.c）
 *   - 180/270 平滑推进（交给 Timer.c）
 *   - 串口协议解析（交给 UART.c）
 *
 * ---------------------------------------------------------------------------
 * 2) 自动流程可抢占（g_manual_override）
 * ---------------------------------------------------------------------------
 * 自动演示流程中会有较长的延时和循环。
 * 为了让用户一旦下发手动命令就能立即接管：
 *   - 自动流程中反复检查 g_manual_override
 *   - 一旦为 1：停止底座并立刻 return
 *
 * DelayAbortable() 就是把长延时拆成小片段（每 5ms 检查一次），
 */

#include "movement.h"

#include "Servo.h"
#include "Timer.h"
#include "UART.h"
#include "stm32f4xx_hal.h"

/* ================== 外部状态（来自UART模块） ================== */
extern volatile uint8_t g_manual_override;

/* ================== 内部小工具：clampf（浮点限幅） ==================
 * 解释：
 *  限幅是任何机器人/舵机系统的“安全第一步”，防止：
 *   - 协议数据异常
 *   - 计算误差
 *   - 机械干涉（超过物理行程）
 */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Auto_ShouldAbort：统一的“自动流程是否应中止”判断 */
static inline uint8_t Auto_ShouldAbort(void)
{
    if (g_manual_override)
    {
        base_stop();   /* 至少先停掉360，避免继续旋转造成危险 */
        return 1;
    }
    return 0;
}

/* DelayAbortable：可中断延时（长流程必备） */
static void DelayAbortable(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms)
    {
        if (Auto_ShouldAbort()) return;
        HAL_Delay(5);
    }
}

/* ================== 底座旋转控制（360°连续旋转舵机） ==================
 * 这里的 angle 参数是“希望旋转的角度”，底层通过 Servo_Rotate_Angle_Timed()
 * 换算成“运行时间”实现开环近似。
 */
void base_rotate_forward(float angle)
{
    Servo_Rotate_Angle_Timed(SERVO_BASE_360, angle, SPEED_SLOW_FORWARD);
}

void base_rotate_backward(float angle)
{
    Servo_Rotate_Angle_Timed(SERVO_BASE_360, angle, SPEED_SLOW_BACKWARD);
}

void base_stop(void)
{
    Servo_360mode(SERVO_BASE_360, STOP);
}

/* ================== 第二关节（270°舵机） ================== */
/**
 * joint2_set_angle（270°关节）：设置目标角度
 * -----------------------------------------------------------------------------
 * 逻辑链：
 *   angle(度) -> pulse(us) -> 写 g_joint2_target_pulse -> g_joint2_moving=1
 *   Timer.c 每 20ms 把 current_pulse 向 target_pulse 推近（平滑）
 *
 * 这样 movement 层不需要关心“每次推多少us、何时到位”，职责更清晰。
 */
void joint2_set_angle(float angle)
{
    /* 安全范围限制（你协议通常发 105~165，这里再兜底） */
    if (angle < 60.0f)  angle = 60.0f;
    if (angle > 180.0f) angle = 180.0f;

    /* 270°线性映射：0~270 -> 500~2500us */
    uint16_t pulse = (uint16_t)(angle * 2000.0f / 270.0f + 500.0f);

    /* 临界区：避免 Timer 中断同时读写共享变量 */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    g_joint2_target_pulse = pulse;
    g_joint2_moving = 1;
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ================== 180°夹爪舵机（TIM3-CH2 / Num=2） ================== */
/**
 * claw_set_angle（180°夹爪）：设置目标角度
 * -----------------------------------------------------------------------------
 * 同 joint2_set_angle：只设置 target，让 Timer 做平滑推进。
 */
void claw_set_angle(float angle)
{
    /* 范围限制：原注释写 0~140，但这里 clamp 到 0~120（保持你当前代码行为） */
    angle = clampf(angle, 0.0f, 120.0f);

    /* 180°线性映射：0~180 -> 500~2500us */
    uint16_t pulse = (uint16_t)(angle * 2000.0f / 180.0f + 500.0f);

    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    g_claw_target_pulse = pulse;
    g_claw_moving = 1;
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ================== 关节控制封装（语义化 API） ==================
 * 这些函数把“具体角度数值”封装成“动作语义”，提高可读性。
 */
void joint2_move_back_30(void)    { joint2_set_angle(ANGLE_270_BACK_30); }
void joint2_move_forward_30(void) { joint2_set_angle(ANGLE_270_FORWARD_30); }
void joint2_reset(void)           { joint2_set_angle(ANGLE_270_CENTER); }

void joint3_open(void)            { claw_set_angle(ANGLE_180_OPEN); }
void joint3_reset(void)           { claw_set_angle(ANGLE_180_CLOSE); }

/* ================== 动作插值（线性插值 lerp） ==================
 * 教材解释：
 *  progress 从 0->1，angle 从 A->B，形成平滑过渡：
 *     angle = A + (B-A)*progress
 */
static float lerp_angle(float a, float b, float t)
{
    if (t <= 0.0f) return a;
    if (t >= 1.0f) return b;
    return a + (b - a) * t;
}

/**
 * movement_startup：演示/启动动作脚本（可被手动接管中止）
 * -----------------------------------------------------------------------------
 * 可把它当成一个“显式状态机”：
 *   STATE_INIT_WAIT  ：安全等待（10s）
 *   STATE_RAMP_CW    ：底座正转软启动
 *   STATE_CW_LINK    ：底座正转 + joint2 从 center -> back 插值
 *   STATE_PAUSE      ：暂停（5s）
 *   STATE_RAMP_CCW   ：底座反转软启动
 *   STATE_CCW_LINK   ：底座反转 + joint2 从 back -> front 插值
 *   STATE_DONE       ：joint2 回到 center
 *
 * 关键工程点：
 *   - 在每个循环/延时段反复检查 Auto_ShouldAbort()，实现手动抢占。
 */
void movement_startup(void)
{
    uint32_t start_time = 0;
    uint32_t elapsed = 0;

    uint16_t current_speed = STOP;
    uint16_t target_speed = STOP;
    uint32_t last_ramp_time = 0;

    g_manual_override = 0;

    UART2_Printf("\r\n>>> Robot Arm Startup - v3.9 <<<\r\n");
    UART2_Printf(">>> Angle Config: Center=%d / Back=%d / Front=%d <<<\r\n",
                 (int)ANGLE_270_CENTER, (int)ANGLE_270_BACK, (int)ANGLE_270_FRONT);
    UART2_Printf(">>> Safety Wait 10 Seconds <<<\r\n\r\n");

    /* ========== STATE_INIT_WAIT：安全等待10秒（避免上电立刻大动作）========== */
    joint2_set_angle(ANGLE_270_CENTER);

    UART2_Printf("[Action] Claw servo move to 60 deg\r\n");
    claw_set_angle(ANGLE_180_OPEN);

    start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < DELAY_SAFETY)
    {
        if (Auto_ShouldAbort()) return;
        DelayAbortable(500);
        if (Auto_ShouldAbort()) return;
    }

    UART2_Printf("[State] STATE_RAMP_CW - Forward soft start\r\n");

    /* ========== STATE_RAMP_CW：正转软启动 ========== */
    current_speed = STOP;
    target_speed  = SPEED_SLOW_FORWARD;
    last_ramp_time = HAL_GetTick();

    while (current_speed < target_speed)
    {
        if (Auto_ShouldAbort()) return;

        if ((HAL_GetTick() - last_ramp_time) >= RAMP_INTERVAL)
        {
            last_ramp_time = HAL_GetTick();

            current_speed = (uint16_t)(current_speed + RAMP_STEP);
            if (current_speed > target_speed) current_speed = target_speed;

            Servo_360mode(SERVO_BASE_360, current_speed);
        }

        DelayAbortable(10);
        if (Auto_ShouldAbort()) return;
    }

    UART2_Printf("[State] STATE_CW_LINK - Enter forward linkage (PWM=%d)\r\n", target_speed);

    /* ========== STATE_CW_LINK：正转联动（底座保持转动 + Joint2同步）========== */
    start_time = HAL_GetTick();
    elapsed = 0;

    while (elapsed < DELAY_ROTATION)
    {
        if (Auto_ShouldAbort()) return;
        elapsed = HAL_GetTick() - start_time;

        /* A. 底座保持转动 */
        Servo_360mode(SERVO_BASE_360, SPEED_SLOW_FORWARD);

        /* B. Joint2同步：Center -> Back */
        float progress = (float)elapsed / (float)DELAY_ROTATION;
        joint2_set_angle(lerp_angle(ANGLE_270_CENTER, ANGLE_270_BACK, progress));

        DelayAbortable(50);
        if (Auto_ShouldAbort()) return;
    }

    UART2_Printf("[Action] Forward complete (%.0f -> %.0f deg)\r\n", ANGLE_270_CENTER, ANGLE_270_BACK);
    base_stop();
    joint2_set_angle(ANGLE_270_BACK);

    /* ========== STATE_PAUSE：休息5秒（原逻辑保留：这里使用阻塞延时）========== */
    UART2_Printf("[State] STATE_PAUSE - Rest 5 seconds\r\n");
    HAL_Delay(DELAY_PAUSE);

    UART2_Printf("[State] STATE_RAMP_CCW - Backward soft start\r\n");

    /* ========== STATE_RAMP_CCW：反转软启动 ========== */
    current_speed = STOP;
    target_speed  = SPEED_SLOW_BACKWARD;
    last_ramp_time = HAL_GetTick();

    while (current_speed > target_speed)
    {
        if (Auto_ShouldAbort()) return;

        if ((HAL_GetTick() - last_ramp_time) >= RAMP_INTERVAL)
        {
            last_ramp_time = HAL_GetTick();

            current_speed = (uint16_t)(current_speed - RAMP_STEP);
            if (current_speed < target_speed) current_speed = target_speed;

            Servo_360mode(SERVO_BASE_360, current_speed);
        }

        DelayAbortable(10);
        if (Auto_ShouldAbort()) return;
    }

    UART2_Printf("[State] STATE_CCW_LINK - Enter backward linkage (PWM=%d)\r\n", target_speed);

    /* ========== STATE_CCW_LINK：反转联动（Back -> Front）========== */
    start_time = HAL_GetTick();
    elapsed = 0;

    while (elapsed < DELAY_ROTATION)
    {
        if (Auto_ShouldAbort()) return;
        elapsed = HAL_GetTick() - start_time;

        Servo_360mode(SERVO_BASE_360, SPEED_SLOW_BACKWARD);

        float progress = (float)elapsed / (float)DELAY_ROTATION;
        joint2_set_angle(lerp_angle(ANGLE_270_BACK, ANGLE_270_FRONT, progress));

        DelayAbortable(50);
        if (Auto_ShouldAbort()) return;
    }

    UART2_Printf("[Action] Backward complete (%.0f -> %.0f deg)\r\n", ANGLE_270_BACK, ANGLE_270_FRONT);
    base_stop();
    joint2_set_angle(ANGLE_270_FRONT);

    /* ========== STATE_DONE：回中位（2秒插值）========== */
    UART2_Printf("[State] STATE_DONE - Return to center (%.0f -> %.0f deg)\r\n",
                 ANGLE_270_FRONT, ANGLE_270_CENTER);

    start_time = HAL_GetTick();
    elapsed = 0;

    while (elapsed < 2000)
    {
        if (Auto_ShouldAbort()) return;
        elapsed = HAL_GetTick() - start_time;

        float progress = (float)elapsed / 2000.0f;
        joint2_set_angle(lerp_angle(ANGLE_270_FRONT, ANGLE_270_CENTER, progress));

        DelayAbortable(50);
        if (Auto_ShouldAbort()) return;
    }

    joint2_set_angle(ANGLE_270_CENTER);
    UART2_Printf("[System] Return to center complete, current %.0f deg\r\n", ANGLE_270_CENTER);

    UART2_Printf("[Action] Claw servo reset to CLOSE deg\r\n");
    claw_set_angle(ANGLE_180_CLOSE);

    DelayAbortable(500);
    if (Auto_ShouldAbort()) return;

    /* ========== STATE_BT_IDLE：提示进入手动模式（仅提示，不自动切模式）========== */
    UART2_Printf("\r\n==================================\r\n");
    UART2_Printf(">>> Enter Manual Control Mode <<<\r\n");
    UART2_Printf(">>> Bluetooth Commands (mode_flag=2): <<<\r\n");
    UART2_Printf("==================================\r\n\r\n");
}

void ready_demo(void)
{
    UART2_Printf("\r\n>>> Start Ready Actions <<<\r\n");

    joint2_reset();
    DelayAbortable(2000);
    if (Auto_ShouldAbort()) return;

    joint3_reset();

    UART2_Printf("\r\n>>> Ready Actions Complete <<<\r\n");
}

void movement_demo(void)
{
    g_manual_override = 0;

    UART2_Printf("\r\n>>> Start Demo Actions <<<\r\n");

    UART2_Printf("[Test] Joint2: center -> back -> front -> center\r\n");
    joint2_reset();
    DelayAbortable(1000);
    if (Auto_ShouldAbort()) return;

    joint2_move_back_30();
    DelayAbortable(1000);
    if (Auto_ShouldAbort()) return;

    joint2_move_forward_30();
    DelayAbortable(1000);
    if (Auto_ShouldAbort()) return;

    joint2_reset();
    DelayAbortable(1000);
    if (Auto_ShouldAbort()) return;

    UART2_Printf("[Test] Base: forward 90deg -> stop -> backward 90deg -> stop\r\n");
    base_rotate_forward(90.0f);
    DelayAbortable(2000);
    if (Auto_ShouldAbort()) return;

    base_stop();
    DelayAbortable(1000);
    if (Auto_ShouldAbort()) return;

    base_rotate_backward(90.0f);
    DelayAbortable(2000);
    if (Auto_ShouldAbort()) return;

    base_stop();

    UART2_Printf(">>> Demo Actions Complete <<<\r\n\r\n");
}
