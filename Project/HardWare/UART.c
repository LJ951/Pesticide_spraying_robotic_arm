/*
 * UART.c — 自动帧解析（USART1）& 手动蓝牙命令（USART2）
 * =============================================================================
 * 本文件承担“通信输入 -> 动作意图”的第一跳转换：
 *
 *   USART1（自动模式）：视觉帧 7 字节，协议如下（固定长度，定界符 AA/FF）
 *     Byte0  Byte1   Byte2   Byte3     Byte4     Byte5      Byte6
 *     0xAA   cw      ccw     raw270    raw180     sel       0xFF
 *
 *   USART2（手动模式）：单字节命令 cmd（例如 0x01/0x02/0x03...）
 *
 * -----------------------------------------------------------------------------
 * A. 为什么要用“组帧状态机”？
 * -----------------------------------------------------------------------------
 * 串口是字节流：可能从任何位置开始读到（丢字节、错位、重启），因此必须：
 *   1) 先找到帧头（0xAA）完成同步；
 *   2) 再按固定长度收满 7 字节；
 *   3) 检查帧尾（0xFF）做校验；
 *   4) 通过后才交给 Vision_HandleFrame() 做动作译码。
 *
 * 这就是典型“同步字 + 定长帧”的教材级协议解析套路。
 *
 * -----------------------------------------------------------------------------
 * B. 并发/临界区（非常重要）
 * -----------------------------------------------------------------------------
 * 180/270 舵机的平滑推进由 Timer.c 的 TIM6 中断每 20ms 更新一次，
 * 因此 UART.c / movement.c 在修改以下共享变量时要避免“写一半被中断打断”：
 *   g_joint2_target_pulse, g_joint2_moving
 *   g_claw_target_pulse,   g_claw_moving
 *
 * 这里使用：
 *   HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
 *   ... 修改共享变量 ...
 *   HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
 *
 * 这是最直接的“关中断临界区保护”。
 *
 * -----------------------------------------------------------------------------
 * C. 调试输出注意
 * -----------------------------------------------------------------------------
 * BT_PrintVisionFrame() 会通过 UART2_Printf() 阻塞发送调试信息。
 * 如果在高频 USART1 回调里频繁打印，可能导致中断执行时间变长、影响实时性。
 * 教学建议：调试完成后可关闭或降低打印频率。
 */

#include "UART.h"
#include "Timer.h"
#include "movement.h"
#include "Servo.h"

/* ================== RX buffers ==================
 * 说明：
 *  - HAL_UART_Receive_IT 需要提供接收缓冲区指针
 *  - 这里使用 1 字节 buffer + 回调里重挂接，是典型“流式解析”写法
 */
uint8_t g_uart1_rx_buf[1] = {0};
uint8_t g_uart1_rx_data   = 0;

uint8_t g_uart2_rx_buf[1] = {0};
uint8_t g_uart2_rx_data   = 0;

/* main.c 会切换：1=Auto, 2=Manual */
uint8_t mode_flag = 0;

/* 手动模式方向（由蓝牙命令更新；Timer.c 会读取 joint2/claw 方向） */
volatile int8_t g_base_dir   = 0;
volatile int8_t g_joint2_dir = 0;
volatile int8_t g_claw_dir   = 0;

/* 手动接管：一旦收到任何手动命令 -> 置 1，用于打断自动长流程 */
volatile uint8_t g_manual_override = 0;

/* 270 当前角度缓存（显示/调试用；真实输出以 PWM 脉宽为准） */
volatile float g_joint2_angle = ANGLE_270_CENTER;

/* ================== 视觉帧解析 ==================
   协议：AA [1] [2] [3] [4] [5] FF  (固定7字节)

   Byte1/Byte2（360连续旋转）：
     - 约定：cw 表示某方向转动量，ccw 表示反方向转动量
     - 安全策略：只允许单方向非零（cw>0 XOR ccw>0），否则认为无效 -> stop

   Byte3（270关节）：
     - 协议映射：00~3C -> 105~165（这里用 105 + raw270）
     - 再做 clamp，防止越界/异常

   Byte4（180夹爪）：
     - 这里直接当作角度值使用（并做 0~140 限幅）

   Byte5（sel 选择位）：
     - 用于决定“本帧控制哪些舵机”
     - 兼容旧协议：sel=00 表示正常（以前只控制360）
================================================== */
#define V_FRAME_LEN 7
static uint8_t v_buf[V_FRAME_LEN];
static uint8_t v_sync = 0;  /* 0=未同步（还没找到AA），1=已同步开始收帧 */
static uint8_t v_idx  = 0;  /* 已收集到的字节数（0~6） */

/* 视觉帧调试输出到蓝牙（教材提醒：中断里大量printf会影响实时性） */
static void BT_PrintVisionFrame(const uint8_t f[V_FRAME_LEN])
{
    UART2_Printf("-> V: %02X %02X %02X %02X %02X %02X %02X\r\n",
                 f[0],f[1],f[2],f[3],f[4],f[5],f[6]);
}

/* -----------------------------------------------------------------------------
 * Hold_270 / Hold_180：冻结舵机当前位置
 * -----------------------------------------------------------------------------
 * “冻结”的做法不是停止PWM（舵机会失力），而是：
 *   target_pulse = current_pulse
 *   moving = 0
 *
 * 这样 Timer.c 的平滑推进逻辑就不会再改输出，PWM 仍保持当前位置。
 */
static void Hold_270(void)
{
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    g_joint2_target_pulse = g_joint2_current_pulse;
    g_joint2_moving = 0;
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

static void Hold_180(void)
{
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    g_claw_target_pulse = g_claw_current_pulse;
    g_claw_moving = 0;
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* -----------------------------------------------------------------------------
 * DecodeSelect：把 sel 翻译成“哪些舵机使能”
 * -----------------------------------------------------------------------------
 * 这是一个典型“控制组合码”的译码函数。
 *
 * sel 含义（按你 Excel 定义）：
 *   0x01 : 仅控制 360
 *   0x02 : 仅控制 270
 *   0x03 : 仅控制 180
 *   0x04 : 360 + 270
 *   0x05 : 360 + 180
 *   0x06 : 270 + 180
 *   0x07 : 全停（在上层处理）
 *
 * 兼容旧视觉：
 *   sel=0x00 过去表示“正常”（常用于只控制 360 的旧版本）
 */
static void DecodeSelect(uint8_t sel,
                         uint8_t *en360, uint8_t *en270, uint8_t *en180)
{
    *en360 = *en270 = *en180 = 0;

    /* 兼容旧视觉：sel=00 表示正常（以前只控制360） */
    if (sel == 0x00) { *en360 = 1; return; }

    if (sel == 0x01) { *en360 = 1; return; }
    if (sel == 0x02) { *en270 = 1; return; }
    if (sel == 0x03) { *en180 = 1; return; }
    if (sel == 0x04) { *en360 = 1; *en270 = 1; return; }
    if (sel == 0x05) { *en360 = 1; *en180 = 1; return; }
    if (sel == 0x06) { *en270 = 1; *en180 = 1; return; }
    /* sel==0x07 由上层处理为全停 */
}

/**
 * Vision_HandleFrame：把一帧视觉数据翻译为动作
 * -----------------------------------------------------------------------------
 * 输入：f[0..6] 为已通过 AA/FF 校验的帧
 * 输出：调用 movement/servo 接口，改变系统的“目标状态”
 *
 * 逻辑步骤（教材式）：
 *   1) 若不在自动模式 mode_flag!=1，则丢弃（防止自动帧干扰手动）
 *   2) 处理特殊“停止帧/全停码”（兼容旧协议 + 新协议）
 *   3) 根据 sel 译码出 en360/en270/en180
 *   4) 360：只允许单方向，且你反馈“方向反了”，这里做 cw/ccw 对调修正
 *   5) 270：raw270 -> deg -> clamp -> joint2_set_angle（交给 Timer 平滑）
 *   6) 180：raw180 -> deg -> clamp -> claw_set_angle（交给 Timer 平滑）
 */
static void Vision_HandleFrame(const uint8_t f[V_FRAME_LEN])
{
    uint8_t cw   = f[1];
    uint8_t ccw  = f[2];
    uint8_t raw270 = f[3];
    uint8_t raw180 = f[4];
    uint8_t sel  = f[5];

    BT_PrintVisionFrame(f);

    if (mode_flag != 1) return;   /* 只有自动模式才执行 */
    g_manual_override = 0;        /* 自动模式清除手动接管（让自动流程可继续） */

    /* ------------------ 兼容旧视觉：AA 00 00 00 00 01 FF 表示停止 ------------------ */
    if (sel == 0x01 && cw==0 && ccw==0 && raw270==0 && raw180==0)
    {
        Servo_Cancel_Timed();
        base_stop();
        Hold_270();
        Hold_180();
        return;
    }

    /* ------------------ 新协议：sel=0x07 所有舵机停止（全停） ------------------ */
    if (sel == 0x07)
    {
        Servo_Cancel_Timed();
        base_stop();
        Hold_270();
        Hold_180();
        return;
    }

    uint8_t en360, en270, en180;
    DecodeSelect(sel, &en360, &en270, &en180);

    /* 兼容策略：
     * 若 sel=00（旧版本“正常”）但视觉开始发送 270/180 非0，
     * 则自动放开控制，让 270/180 也能被驱动。
     */
    if (sel == 0x00)
    {
        if (raw270 != 0) en270 = 1;
        if (raw180 != 0) en180 = 1;
    }

    /* ================== 360 控制（方向修正） ==================
       你说“360运动是反的”，所以这里把 cw/ccw 映射对调：
       - cw>0  -> 调用 base_rotate_backward()
       - ccw>0 -> 调用 base_rotate_forward()
       若以后硬件接线/舵机定义改变，只需交换这两句。
    =========================================================== */
    if (!en360)
    {
        Servo_Cancel_Timed();
        base_stop();
    }
    else
    {
        /* 安全：只允许单方向（协议常用“二选一”避免歧义） */
        if (cw > 0 && ccw == 0)
        {
            Servo_Cancel_Timed();                 /* 抢占：清掉旧的定时转动任务 */
            base_rotate_backward((float)cw);      /* 方向对调修正 */
        }
        else if (ccw > 0 && cw == 0)
        {
            Servo_Cancel_Timed();
            base_rotate_forward((float)ccw);      /* 方向对调修正 */
        }
        else
        {
            /* 同时非零或都为 0：当作无效，停 */
            Servo_Cancel_Timed();
            base_stop();
        }
    }

    /* ================== 270 控制（105~165°） ==================
     * 协议映射：deg = 105 + raw270
     * 再做 clamp：防止 raw270 异常导致越界
     */
    if (!en270)
    {
        Hold_270();
    }
    else
    {
        float deg = 105.0f + (float)raw270;
        if (deg < 105.0f) deg = 105.0f;
        if (deg > 165.0f) deg = 165.0f;

        /* joint2_set_angle() 做的是“写目标”，具体平滑由 Timer.c 完成 */
        joint2_set_angle(deg);
    }

    /* ================== 180 控制（0~140°） ==================
     * 这里把 raw180 视为角度值（再限幅），再交给 claw_set_angle 平滑推进。
     */
    if (!en180)
    {
        Hold_180();
    }
    else
    {
        float deg = (float)raw180;
        if (deg < 0.0f)   deg = 0.0f;
        if (deg > 140.0f) deg = 140.0f;
        claw_set_angle(deg);
    }
}

/**
 * Vision_FeedByte：视觉帧组帧状态机（同步字 0xAA + 定长 7 字节）
 * -----------------------------------------------------------------------------
 * 状态机思想：
 *
 *   状态 S0（未同步）：
 *     - 丢弃所有字节，直到看到 0xAA -> 进入 S1（同步收帧）
 *
 *   状态 S1（同步中/收帧中）：
 *     - 把字节依次塞入 v_buf[]
 *     - 如果中途又遇到 0xAA，认为前一帧可能错位：立即重同步（从头收）
 *     - 收满 7 字节后，检查 v_buf[0]==AA 且 v_buf[6]==FF
 *       - 通过 -> Vision_HandleFrame()
 *       - 不通过 -> 丢弃
 */
static void Vision_FeedByte(uint8_t b)
{
    if (!v_sync)
    {
        if (b == 0xAA)
        {
            v_sync = 1;
            v_idx = 0;
            v_buf[v_idx++] = b;
        }
        return;
    }

    /* 中途遇到AA，直接重同步（非常常见的“自恢复”策略） */
    if (b == 0xAA)
    {
        v_idx = 0;
        v_buf[v_idx++] = b;
        return;
    }

    v_buf[v_idx++] = b;

    if (v_idx >= V_FRAME_LEN)
    {
        v_sync = 0;
        if (v_buf[0] == 0xAA && v_buf[6] == 0xFF)
        {
            Vision_HandleFrame(v_buf);
        }
        v_idx = 0;
    }
}

/* ================== 蓝牙手动命令 ==================
 * 设计思路：
 *  - “底座360”：直接输出速度 PWM（持续转，直到 STOP 命令）
 *  - “270/180”：只改方向变量 g_joint2_dir / g_claw_dir
 *     由 Timer.c 的 20ms 平滑推进把它推到端点，并减少抖动
 *  - 任意命令：置 g_manual_override=1，用于抢占自动流程
 */

/**
 * process_bluetooth_command：蓝牙手动命令解析
 * -----------------------------------------------------------------------------
 * cmd 是单字节命令码。
 * 这是“命令码 -> 行为”的查表/分支实现。
 */
void process_bluetooth_command(uint8_t cmd)
{
    if (mode_flag != 2) return;   /* 只在手动模式响应蓝牙 */

    switch (cmd)
    {
        case 0x01: /* base forward press：底座正向持续转 */
            g_base_dir = 1;
            Servo_Cancel_Timed(); /* 清掉自动模式可能遗留的定时旋转任务 */
            Servo_360mode(SERVO_BASE_360, SPEED_SLOW_FORWARD);
            break;

        case 0x02: /* base backward press：底座反向持续转 */
            g_base_dir = -1;
            Servo_Cancel_Timed();
            Servo_360mode(SERVO_BASE_360, SPEED_SLOW_BACKWARD);
            break;

        case 0x07: /* base stop release：底座停 */
            g_base_dir = 0;
            Servo_Cancel_Timed();
            base_stop();
            break;

        /* ==================== 第二关节（270°）方向控制 ==================== */
        case 0x03: g_joint2_dir = +1; break; /* 向上 */
        case 0x04: g_joint2_dir = -1; break; /* 向下 */
        case 0x08: g_joint2_dir =  0; break; /* 松手停止（Timer里会冻结当前位置） */

        /* ==================== 夹爪（180°）方向控制 ==================== */
        case 0x05: g_claw_dir = +1; break; /* 张开方向 */
        case 0x06: g_claw_dir = -1; break; /* 闭合方向 */

        case 0x09: /* claw stop release：冻结当前位置 */
            g_claw_dir = 0;
            Hold_180();
            break;

        /* 额外 GPIO 控制：外设开关(风扇) */
        case 0x10:
            HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_3);
            HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_4);
            break;

        default:
            break;
    }

    /* 一旦出现任何手动命令，视为“人工抢占自动控制权” */
    g_manual_override = 1;
}

/* ================== HAL 回调 ==================
 * HAL_UART_RxCpltCallback 是 HAL 层提供的“中断接收完成回调”。
 * 这里每次只收 1 字节，因此回调触发频率 = 字节到来频率。
 *
 * 教材关键点：
 *   回调里必须再次调用 HAL_UART_Receive_IT() 重挂接下一次接收。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        g_uart1_rx_data = g_uart1_rx_buf[0];
        Vision_FeedByte(g_uart1_rx_data);
        HAL_UART_Receive_IT(&huart1, g_uart1_rx_buf, 1);
        return;
    }

    if (huart->Instance == USART2)
    {
        g_uart2_rx_data = g_uart2_rx_buf[0];
        process_bluetooth_command(g_uart2_rx_data);
        HAL_UART_Receive_IT(&huart2, g_uart2_rx_buf, 1);
        return;
    }
}

/* ================== Printf ==================
 * UARTx_Printf：面向调试的 printf 封装
 * 原理：vsnprintf 把格式化字符串写入 buf，再 HAL_UART_Transmit 阻塞发送。
 *
 * 注意：
 *  - 阻塞发送在中断环境会拉长中断时间，不建议长期开启高频打印。
 *  - 这里保留原实现，用于调试阶段很方便。
 */
void UART1_Printf(char *format, ...)
{
    char buf[128];
    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}

void UART2_Printf(char *format, ...)
{
    char buf[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}
