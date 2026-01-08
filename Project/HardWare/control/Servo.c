/*
 * =============================================================================
 * 本文件只做“硬件输出映射”：
 *   - __HAL_TIM_SET_COMPARE() 写 CCR 以改变 PWM 脉宽
 *   - 角度 -> 脉宽使用线性公式
 *   - 连续旋转的“定时转角”则把角度换算为时间，并写入 Timer.c 的共享变量
 *
 * 重要提醒：
 *   连续旋转舵机的“转速”不是常数：
 *     - 电压变化、负载变化都会影响速度
 *   因此 Servo_Rotate_Angle_Timed 属于开环近似方案：
 *     - 对“演示/粗略转角”足够
 *     - 对“精确定位”需要闭环（编码器/IMU/视觉反馈）
 */

#include "Servo.h"

/* 舵机旋转速率：这里按 60deg/s 的标定值换算（示例）
 * 60 deg / 1000 ms = 0.06 deg/ms
 * 该值若要更准，需要你对实际舵机做标定（同 PWM、不同负载下会变）
 */
#define ROTATION_SPEED_DEG_PER_MS_PROS (60.0f / 1000.0f)
#define ROTATION_SPEED_DEG_PER_MS_CONS (60.0f / 1000.0f)

/* Timer.c 的共享变量（用于定时停止） */
extern volatile uint16_t g_servo_rotating;
extern uint16_t g_servo_target_ms;
extern uint16_t g_servo_current_ms;
extern uint16_t g_servo_timer_num;

/**
 * Servo_360mode：连续旋转舵机速度输出
 * -----------------------------------------------------------------------------
 * 输入：mode = PWM 脉宽（us 计数值，实际与定时器配置有关）
 * 核心原理：
 *   - STOP=1500us 表示停止
 *   - 偏离 1500us 的方向与大小决定转动方向与速度
 *
 * Num 的含义是“你项目里约定的舵机编号”，映射到不同 TIM 通道。
 */
void Servo_360mode(uint8_t Num, uint16_t mode)
{
    if(Num == 2){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, mode);}
    if(Num == 3){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, mode);}
    if(Num == 4){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, mode);}
}

/**
 * Servo_270Angle：270°定位舵机角度输出（直接写CCR）
 * -----------------------------------------------------------------------------
 * pulse = 500 + Angle * (2000/270)
 * 注意：这里是“立即写输出”，不含平滑；你工程实际平滑是 Timer.c 完成的。
 */
void Servo_270Angle(uint8_t Num, float Angle)
{
    uint16_t pulse = (uint16_t)(Angle * 2000.0f / 270.0f + 500.0f);

    if(Num == 2){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);}
    if(Num == 3){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);}
    if(Num == 4){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);}
}

/**
 * Servo_180Angle：180°定位舵机角度输出（直接写CCR）
 * -----------------------------------------------------------------------------
 * pulse = 500 + Angle * (2000/180)
 */
void Servo_180Angle(uint8_t Num, float Angle)
{
    uint16_t pulse = (uint16_t)(Angle * 2000.0f / 180.0f + 500.0f);

    if(Num == 1){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);}
    if(Num == 2){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);}
    if(Num == 3){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);}
    if(Num == 4){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);}
}

/**
  * @brief 连续旋转舵机，通过计时控制角度 (支持抢占/打断)
  * @note  修复：兼容 SPEED_SLOW_FORWARD=1620 / SPEED_SLOW_BACKWARD=1400 等“非MAX档位”
  */

/**
 * Servo_Rotate_Angle_Timed：连续旋转舵机“按角度转动”的开环近似
 * -----------------------------------------------------------------------------
 * 目标：让连续旋转舵机大致转过 Angle（度）
 *
 * 方法：
 *   1) 选择一个转速常量 speed（deg/ms）
 *   2) target_ms = Angle / speed
 *   3) 把 target_ms 写入 Timer.c 的共享变量
 *   4) 立即输出 Pulse_Mode（让舵机开始转）
 *   5) Timer.c 每 1ms 计数，到点后自动 STOP
 *
 * 抢占/打断：
 *   新命令到来时可以 Servo_Cancel_Timed() 清掉旧的计时任务。
 */
void Servo_Rotate_Angle_Timed(uint8_t Num, float Angle, uint16_t Pulse_Mode)
{
    /* Angle 异常或命令就是 STOP：直接停 */
    if (Angle <= 0.0f || Pulse_Mode == STOP)
    {
        Servo_360mode(Num, STOP);
        return;
    }

    /* 根据 PWM 相对 STOP 的方向来选速度（结构保留，便于未来分别标定正反转速） */
    float speed = (Pulse_Mode > STOP) ? ROTATION_SPEED_DEG_PER_MS_CONS : ROTATION_SPEED_DEG_PER_MS_PROS;

    uint16_t new_target_ms = (uint16_t)(Angle / speed);
    if (new_target_ms == 0) new_target_ms = 1;

    /* 临界区保护：Timer 中断每 1ms 会访问这些变量 */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);

    g_servo_target_ms = new_target_ms;
    g_servo_current_ms = 0;
    g_servo_timer_num = Num;
    g_servo_rotating = 1;

    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* 立即更新 PWM，让舵机开始转 */
    Servo_360mode(Num, Pulse_Mode);
}

/**
 * Servo_Cancel_Timed：取消当前定时旋转任务
 * -----------------------------------------------------------------------------
 * 用途：当收到新的旋转/全停/模式切换指令时，清空计时状态，避免旧任务残留。
 */
void Servo_Cancel_Timed(void)
{
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    g_servo_rotating = 0;
    g_servo_target_ms = 0;
    g_servo_current_ms = 0;
    g_servo_timer_num = 0;
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
