/*
 * UART.h / UART.c — 通信层（串口协议 + 模式切换入口）
 * =============================================================================
 * 本工程使用两路串口，分别承担不同“信息源”的输入：
 *
 *   USART1：视觉/上位机 —— 固定长度 7 字节帧（AA ... FF）
 *           典型特点：数据频率较高（可能连续输出），必须能自动同步、自动组帧。
 *
 *   USART2：蓝牙手柄/遥控 —— 单字节命令（0x01/0x02/...）
 *           典型特点：命令离散、交互式（按下/松开），用于“手动模式”。
 *
 * ---------------------------------------------------------------------------
 * 1) 软件分层（建议按“教材式”理解）
 * ---------------------------------------------------------------------------
 *   [硬件中断层] HAL_UART_RxCpltCallback()
 *        |
 *        |  (USART1) 每来 1 字节 -> Vision_FeedByte() 组帧状态机
 *        |  (USART2) 每来 1 字节 -> process_bluetooth_command() 命令译码
 *        v
 *   [协议解析层] Vision_HandleFrame() / process_bluetooth_command()
 *        |
 *        v
 *   [动作意图层] movement.c 里的 base_rotate_joint2_set_angle/claw_set_angle...
 *        |
 *        v
 *   [执行层] Servo.c(写CCR) + Timer.c(1ms节拍计时/20ms平滑推进)
 *
 * ---------------------------------------------------------------------------
 * 2) “1字节中断接收”模式的关键点（非常重要，考试常考）
 * ---------------------------------------------------------------------------
 * HAL_UART_Receive_IT(&huartX, buf, 1) 只会启动“一次接收”：
 *   - 当接收完成，会进 RxCpltCallback。
 *   - 若回调里不再次调用 HAL_UART_Receive_IT()，接收链路就断了（只收一次）。
 *
 * ---------------------------------------------------------------------------
 * 3) mode_flag（模式标志）
 * ---------------------------------------------------------------------------
 * main.c 会通过按键切换 mode_flag：
 *   mode_flag == 1 : 自动模式（视觉帧驱动）
 *   mode_flag == 2 : 手动模式（蓝牙命令驱动）
 *
 * ---------------------------------------------------------------------------
 * 4) g_manual_override（手动接管）
 * ---------------------------------------------------------------------------
 * 任何手动命令到来，都置 g_manual_override=1：
 *   - movement_startup() 等自动长流程会反复检查它，一旦接管就立刻中止自动动作。
 */

#ifndef __UART_H__
#define __UART_H__

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ========================= USART1（视觉帧）接收缓冲 =========================
 * 采用 1 字节缓冲的原因：
 *  - HAL 中断接收常用模式：每次收 1 字节，进回调后再挂下一次。
 *  - 这样可以在回调里写“流式协议解析”，无需等待整帧 DMA。
 */
extern uint8_t g_uart1_rx_buf[1];
extern uint8_t g_uart1_rx_data;

/* ========================= USART2（蓝牙命令）接收缓冲 ======================= */
extern uint8_t g_uart2_rx_buf[1];
extern uint8_t g_uart2_rx_data;

/* 模式标志：main.c 改它，UART.c/Timer.c/movement.c 读它 */
extern uint8_t mode_flag;

/* 手动接管标志：UART.c 写，movement.c 读 */
extern volatile uint8_t g_manual_override;

/* printf-like 输出（阻塞发送，适合调试；注意不要在高频中断里狂刷） */
void UART1_Printf(char *format, ...);
void UART2_Printf(char *format, ...);

/* 蓝牙单字节命令译码（手动模式入口） */
void process_bluetooth_command(uint8_t cmd);

#endif
