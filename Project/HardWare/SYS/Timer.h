/*
 * =============================================================================
 * TIM6 在本工程中扮演“系统节拍器 / 轻量调度器”的角色：
 *
 *   1) 1ms：按键扫描 Key_Tick()
 *   2) 10ms：置位 g_data_update_flag（让主循环刷新显示/状态）
 *   3) 1ms：360° 连续旋转的“定时转角”计时（到点 STOP）
 *   4) 20ms：180° / 270° 舵机做“平滑推进”
 *
 * -----------------------------------------------------------------------------
 * 为什么 20ms 平滑推进最自然？
 * -----------------------------------------------------------------------------
 * 绝大多数舵机 PWM 的周期为 20ms（50Hz）。
 * 如果你每 20ms 更新一次占空比/脉宽，就与舵机内部采样节拍对齐，输出更稳定。
 *
 * -----------------------------------------------------------------------------
 * 平滑推进算法（极简一阶限速控制）
 * -----------------------------------------------------------------------------
 * 设 current_pulse 为当前输出脉宽，target_pulse 为目标脉宽：
 *   diff = target - current
 *   每 20ms 只允许改变最多 STEP_US
 *
 * 再加两种“抑抖技巧”（教材常见）：
 *   1) deadband（死区）：diff 小于阈值就直接到位并停止移动
 *   2) quantize（量化）：把输出脉宽四舍五入到 q 的倍数，减少无意义的微小抖动
 *
 * -----------------------------------------------------------------------------
 * 并发注意
 * -----------------------------------------------------------------------------
 * current/target/moving 这类变量会被：
 *   - Timer.c（中断）读写
 *   - UART.c / movement.c（主线程或其它回调）写
 * 因此这些变量都用 volatile，并且写 target/moving 时要进入临界区。
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include "main.h"
#include "tim.h"
#include <stdint.h>

/* ===================== 360° 连续旋转的“定时转角”共享变量 =====================
 * g_servo_rotating：1 表示正在计时旋转；0 表示没有任务
 * g_servo_target_ms/current_ms：目标持续时间 / 已运行时间（ms）
 * g_servo_timer_num：要控制的舵机编号（对应 Servo_360mode 的 Num）
 */
extern volatile uint16_t g_servo_rotating;
extern uint16_t g_servo_target_ms;
extern uint16_t g_servo_current_ms;
extern uint16_t g_servo_timer_num;

/* ===================== 180° 夹爪平滑参数（可调） ===================== */
#define CLAW_SMOOTH_INTERVAL_MS   20u    // 每隔 20ms 推进一步（匹配 50Hz 舵机节拍）
#define CLAW_SMOOTH_STEP_US       5u     // 每次最多改变 5us（越小越平滑、越慢）
#define CLAW_DEADBAND_US         10u     // 死区：小于 10us 直接到位
#define CLAW_QUANT_US             5u     // 输出量化粒度 5us（降低末端抖动）

extern volatile uint8_t  g_claw_moving;
extern volatile uint16_t g_claw_current_pulse;
extern volatile uint16_t g_claw_target_pulse;

/* ===================== 270° 关节平滑参数（可调） ===================== */
#define JOINT2_SMOOTH_INTERVAL_MS 20u
#define JOINT2_SMOOTH_STEP_US      4u
#define JOINT2_DEADBAND_US         6u
#define JOINT2_QUANT_US            2u

extern volatile uint8_t  g_joint2_moving;
extern volatile uint16_t g_joint2_current_pulse;
extern volatile uint16_t g_joint2_target_pulse;

#endif
