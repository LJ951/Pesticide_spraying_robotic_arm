/*
 * =============================================================================
 * 这一层是“动作语义层/动作脚本层”，负责：
 *   - 封装“动作意图”：例如底座转多少、关节到哪个角度、夹爪开合
 *   - 组织演示流程（movement_startup / movement_demo）
 *
 * 分层关系（非常建议按此理解）：
 *   UART(协议解析) -> movement(动作意图/脚本) -> Servo(写PWM/启动定时旋转) -> Timer(平滑/计时)
 *
 * 并发注意：
 *   movement 会写 g_joint2_target_pulse 等共享变量，
 *   Timer 中断会读写它们，因此 movement 中写这些变量要进入临界区。
 */

#ifndef __MOVEMENT_H_
#define __MOVEMENT_H_

#include <stdint.h>

/* ================== 机械臂舵机编号（按你的接线/定时器映射） ================== */
#define SERVO_BASE_360       2      // 底座舵机（360°连续旋转，TIM2-CH2）
#define SERVO_JOINT2_270     3      // 第二关节舵机（270°，TIM2-CH3）

/* ================== 270°舵机角度定义 ================== */
#define ANGLE_270_CENTER     135.0f  // 中位
#define ANGLE_270_BACK       165.0f  // 后仰 +30°
#define ANGLE_270_FRONT      105.0f  // 前趴 -30°

/* ================== 180°夹爪舵机配置 ================== */
#define SERVO_CLAW_180       2
#define ANGLE_180_OPEN       60.0f   // 开机动作角度
#define ANGLE_180_CLOSE      0.0f    // 下垂/闭合角度

/* 兼容旧代码别名 */
#define ANGLE_270_BACK_30    ANGLE_270_BACK
#define ANGLE_270_FORWARD_30 ANGLE_270_FRONT

/* ================== 速度参数（连续旋转舵机 PWM）================== */
#define SPEED_SLOW_FORWARD   1620   // 正转
#define SPEED_SLOW_BACKWARD  1400   // 反转

/* ================== 软启动参数 ================== */
#define RAMP_STEP            3
#define RAMP_INTERVAL        50     // ms

/* ================== 时间参数 ================== */
#define DELAY_SAFETY         10000  // 10s
#define DELAY_ROTATION       10000  // 10s
#define DELAY_PAUSE          5000   // 5s

/* ================== 对外接口 ================== */
void movement_startup(void);
void ready_demo(void);
void movement_demo(void);

/* 底座旋转控制（360°舵机） */
void base_rotate_forward(float angle);
void base_rotate_backward(float angle);
void base_stop(void);

/* 第二关节控制（270°舵机） */
void joint2_set_angle(float angle);
void joint2_move_back_30(void);
void joint2_move_forward_30(void);
void joint2_reset(void);

/* 第三关节（180°夹爪舵机） */
void joint3_open(void);
void joint3_reset(void);

void claw_set_angle(float angle);

#endif
