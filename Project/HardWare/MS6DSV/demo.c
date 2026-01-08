/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6DSV模块测试实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "demo.h"
#include "atk_ms6dsv.h"
#include "usart.h"
#include "delay.h"
#include <stdio.h>
// #include "./BSP/LED/led.h"

// #define DEMO_READ_DATA_POLLING      /* 轮询读取加速度、角速度和温度数据 */
// #define DEMO_READ_DATA_FIFO_POLLING /* 轮询读取FIFO中的加速度、角速度和时间戳数据 */
// #define DEMO_QVAR_READ_DATA_POLLING /* 轮询读取静电电压数据 */
// #define DEMO_READ_DATA_INT          /* 中断读取加速度数据 */
// #define DEMO_READ_DATA_FIFO_INT     /* 中断读取FIFO中的加速度和时间戳数据 */
// #define DEMO_FREE_FALL              /* 中断读取自由落体状态 */
// #define DEMO_WAKEUP                 /* 中断读取唤醒状态 */
// #define DEMO_6D                     /* 中断读取6D方向状态 */
// #define DEMO_TAP                    /* 中断读取单击和双击识别状态 */
// #define DEMO_STEP_COUNTER           /* 步数检测功能 */
// #define DEMO_SENSOR_FUSION          /* 传感器融合功能 */
#define DEMO_SENSOR_FUSION_ANOTC /* 传感器融合功能上传匿名上位机 */
// #define DEMO_FSM_TOUCH              /* 状态机静电触摸检测功能 */
// #define DEMO_FSM_4D                 /* 状态机4D方向检测功能 */
//  #define RUN_YAW_ROLL_PITCH /				*USER CODE 获取yaw roll pitch*/

#ifdef DEMO_FSM_TOUCH
#include "./BSP/ATK_MS6DSV/lsm6dsv16x_fsm_touch.h"
#endif /* DEMO_FSM_TOUCH */
#ifdef DEMO_FSM_4D
#include "./BSP/ATK_MS6DSV/lsm6dsv16x_four_d.h"
#endif /* DEMO_FSM_4D */

#if defined(DEMO_SENSOR_FUSION) || defined(DEMO_SENSOR_FUSION_ANOTC)
static uint32_t npy_halfbits_to_floatbits(uint16_t h)
{
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (h & 0x7c00u);
    f_sgn = ((uint32_t)h & 0x8000u) << 16;
    switch (h_exp)
    {
    case 0x0000u: /* 0 or subnormal */
        h_sig = (h & 0x03ffu);
        /* Signed zero */
        if (h_sig == 0)
        {
            return f_sgn;
        }
        /* Subnormal */
        h_sig <<= 1;
        while ((h_sig & 0x0400u) == 0)
        {
            h_sig <<= 1;
            h_exp++;
        }
        f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
        f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
        return f_sgn + f_exp + f_sig;
    case 0x7c00u: /* inf or NaN */
        /* All-ones exponent and a copy of the significand */
        return f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
    default: /* normalized */
        /* Just need to adjust the exponent and shift */
        return f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
    }
}

static float_t npy_half_to_float(uint16_t h)
{
    union
    {
        float_t ret;
        uint32_t retbits;
    } conv;
    conv.retbits = npy_halfbits_to_floatbits(h);
    return conv.ret;
}

static void sflp2q(float quat[4], uint16_t sflp[3])
{
    float sumsq = 0;

    quat[0] = npy_half_to_float(sflp[0]);
    quat[1] = npy_half_to_float(sflp[1]);
    quat[2] = npy_half_to_float(sflp[2]);

    for (uint8_t i = 0; i < 3; i++)
        sumsq += quat[i] * quat[i];

    if (sumsq > 1.0f)
    {
        float n = sqrtf(sumsq);
        quat[0] /= n;
        quat[1] /= n;
        quat[2] /= n;
        sumsq = 1.0f;
    }

    quat[3] = sqrtf(1.0f - sumsq);
}
#endif /* DEMO_SENSOR_FUSION or DEMO_SENSOR_FUSION_ANOTC */
#ifdef DEMO_SENSOR_FUSION_ANOTC

/**
 * @brief       通过串口1发送数据至匿名地面站V4
 * @param       fun: 功能字
 *              dat: 待发送的数据（最多28字节）
 *              len: dat数据的有效位数
 * @retval      无
 */
static void demo_usart1_niming_report(uint8_t fun, uint8_t *dat, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;

    if (len > 28)
    {
        return;
    }

    send_buf[len + 4] = 0;    /* 校验位清零 */
    send_buf[0] = 0xAA;       /* 帧头为0xAAAA */
    send_buf[1] = 0xAA;       /* 帧头为0xAAAA */
    send_buf[2] = fun;        /* 功能字 */
    send_buf[3] = len;        /* 数据长度 */
    for (i = 0; i < len; i++) /* 复制数据 */
    {
        send_buf[4 + i] = dat[i];
    }
    for (i = 0; i < (len + 4); i++) /* 计算校验和 */
    {
        send_buf[len + 4] += send_buf[i];
    }

    /* 发送数据 */
    HAL_UART_Transmit(&huart1, send_buf, len + 5, HAL_MAX_DELAY);
}

/**
 * @brief       发送状态帧至匿名地面站V4
 * @param       rol     : 横滚角
 *              pit     : 俯仰角
 *              yaw     : 航向角
 *              alt     : 飞行高度，单位：cm
 *              fly_mode: 飞行模式
 *              armed   : 锁定状态，0xA0：加锁 0xA1：解锁
 * @retval      无
 */
static void demo_niming_report_status(int16_t rol, int16_t pit, int16_t yaw, uint32_t alt, uint8_t fly_mode, uint8_t armed)
{
    uint8_t send_buf[12];

    /* 横滚角 */
    send_buf[0] = (rol >> 8) & 0xFF;
    send_buf[1] = rol & 0xFF;
    /* 俯仰角 */
    send_buf[2] = (pit >> 8) & 0xFF;
    send_buf[3] = pit & 0xFF;
    /* 航向角 */
    send_buf[4] = (yaw >> 8) & 0xFF;
    send_buf[5] = yaw & 0xFF;
    /* 飞行高度 */
    send_buf[6] = (alt >> 24) & 0xFF;
    send_buf[7] = (alt >> 16) & 0xFF;
    send_buf[8] = (alt >> 8) & 0xFF;
    send_buf[9] = alt & 0xFF;
    /* 飞行模式 */
    send_buf[10] = fly_mode;
    /* 锁定状态 */
    send_buf[11] = armed;

    /* 状态帧的功能字为0x01 */
    demo_usart1_niming_report(0x01, send_buf, 12);
}
#endif /* DEMO_SENSOR_FUSION_ANOTC */

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t ret;
#ifdef DEMO_READ_DATA_POLLING
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    lsm6dsv16x_data_ready_t drdy;
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];
    int16_t data_raw_temperature;
    float acceleration_mg[3];
    float angular_rate_mdps[3];
    float temperature_degc;
    uint8_t times = 0;
#endif /* DEMO_READ_DATA_POLLING */
#ifdef DEMO_READ_DATA_FIFO_POLLING
    lsm6dsv16x_fifo_status_t fifo_status;
    lsm6dsv16x_fifo_out_raw_t data_raw_fifo;
    int16_t *datax;
    int16_t *datay;
    int16_t *dataz;
    int32_t *ts;
    float acceleration_mg[3];
    float angular_rate_mdps[3];
    int32_t timestamp;
    uint8_t times = 0;
#endif /* DEMO_READ_DATA_FIFO_POLLING */
#ifdef DEMO_QVAR_READ_DATA_POLLING
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    lsm6dsv16x_ah_qvar_mode_t qvar_mode;
    lsm6dsv16x_all_sources_t all_sources;
    int16_t data_raw_qvar;
    float qvar_mv;
    uint8_t times = 0;
#endif /* DEMO_QVAR_READ_DATA_POLLING */
#ifdef DEMO_READ_DATA_INT
    lsm6dsv16x_pin_int_route_t pin_int;
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    lsm6dsv16x_data_ready_t drdy;
    int16_t data_raw_acceleration[3];
    float acceleration_mg[3];
    uint8_t times = 0;
#endif /* DEMO_READ_DATA_INT */
#ifdef DEMO_READ_DATA_FIFO_INT
    lsm6dsv16x_pin_int_route_t pin_int;
    lsm6dsv16x_fifo_status_t fifo_status;
    uint16_t num;
    lsm6dsv16x_fifo_out_raw_t data_raw_fifo;
    int16_t *datax;
    int16_t *datay;
    int16_t *dataz;
    int32_t *ts;
    float acceleration_mg[3];
    int32_t timestamp;
    uint8_t times = 0;
#endif /* DEMO_READ_DATA_FIFO_INT */
#ifdef DEMO_FREE_FALL
    lsm6dsv16x_pin_int_route_t pin_int;
    lsm6dsv16x_interrupt_mode_t irq;
    lsm6dsv16x_all_sources_t status;
#endif /* DEMO_FREE_FALL */
#ifdef DEMO_WAKEUP
    lsm6dsv16x_pin_int_route_t pin_int;
    lsm6dsv16x_interrupt_mode_t irq;
    lsm6dsv16x_act_thresholds_t wu;
    lsm6dsv16x_all_sources_t status;
#endif /* DEMO_WAKEUP */
#ifdef DEMO_6D
    lsm6dsv16x_pin_int_route_t pin_int;
    lsm6dsv16x_interrupt_mode_t irq;
    lsm6dsv16x_all_sources_t status;
    uint8_t sixd_event;
#endif /* DEMO_6D */
#ifdef DEMO_TAP
    lsm6dsv16x_pin_int_route_t pin_int;
    lsm6dsv16x_interrupt_mode_t irq;
    lsm6dsv16x_tap_detection_t tap;
    lsm6dsv16x_tap_thresholds_t tap_ths;
    lsm6dsv16x_tap_time_windows_t tap_win;
    lsm6dsv16x_all_sources_t status;
#endif /* DEMO_TAP */
#ifdef DEMO_STEP_COUNTER
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    lsm6dsv16x_stpcnt_mode_t stpcnt_mode;
    lsm6dsv16x_all_sources_t status;
    uint16_t steps;
#endif /* DEMO_STEP_COUNTER */
#ifdef DEMO_SENSOR_FUSION
    lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
    lsm6dsv16x_sflp_gbias_t gbias;
    lsm6dsv16x_fifo_status_t fifo_status;
    uint16_t num;
    lsm6dsv16x_fifo_out_raw_t data_raw_fifo;
    int16_t *axis;
    float quat[4];
    float gravity_mg[3];
    float gbias_mdps[3];
    uint8_t times = 0;
#endif /* DEMO_SENSOR_FUSION */
#ifdef DEMO_SENSOR_FUSION_ANOTC
    lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
    lsm6dsv16x_fifo_status_t fifo_status;
    uint16_t num;
    lsm6dsv16x_fifo_out_raw_t data_raw_fifo;
    lsm6dsv16x_data_ready_t drdy;
    int16_t data_raw_angular_rate[3];
    float angular_rate_mdps_avg[3] = {0};
    uint32_t times;
    lsm6dsv16x_sflp_gbias_t gbias;
    float quat[4];
    float pitch;
    float roll;
    float yaw;
#endif /* DEMO_SENSOR_FUSION_ANOTC */
#ifdef DEMO_FSM_TOUCH
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    lsm6dsv16x_ah_qvar_mode_t qvar_mode;
    uint32_t reg_index;
    lsm6dsv16x_fsm_status_mainpage_t fsm_status;
#endif /* DEMO_FSM_TOUCH */
#ifdef DEMO_FSM_4D
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
    uint32_t reg_index;
    lsm6dsv16x_fsm_status_mainpage_t fsm_status;
    lsm6dsv16x_fsm_out_t fsm_out;
#endif /* DEMO_FSM_4D */

    /* 初始化ATK-MS6DSV模块 */
    ret = atk_ms6dsv_init();
    if (ret != 0)
    {
        printf("ATK-MS6DSV init failed!\r\n");
        while (1)
        {
            //            LED0_TOGGLE();
            delay_ms(200);
        }
    }

#ifdef DEMO_READ_DATA_POLLING
    /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);

    /* 配置滤波器 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
    lsm6dsv16x_filt_gy_lp1_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_GY_ULTRA_LIGHT);
    lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);

    while (1)
    {
        /* 获取数据就绪状态 */
        lsm6dsv16x_flag_data_ready_get(&atk_ms6dsv, &drdy);

        /* 加速度数据就绪 */
        if (drdy.drdy_xl)
        {
            lsm6dsv16x_acceleration_raw_get(&atk_ms6dsv, data_raw_acceleration);
            acceleration_mg[0] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[2]);
        }

        /* 角速度数据就绪 */
        if (drdy.drdy_gy)
        {
            lsm6dsv16x_angular_rate_raw_get(&atk_ms6dsv, data_raw_angular_rate);
            angular_rate_mdps[0] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[2]);
        }

        /* 温度数据就绪 */
        if (drdy.drdy_temp)
        {
            lsm6dsv16x_temperature_raw_get(&atk_ms6dsv, &data_raw_temperature);
            temperature_degc = lsm6dsv16x_from_lsb_to_celsius(data_raw_temperature);
        }

        if (++times == 20)
        {
            times = 0;
            printf("\r\n******************************************\r\n");
            printf("Acceleration[mg]: %4.2f %4.2f %4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            printf("Angular rate[mdps]: %4.2f %4.2f %4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            printf("Temperature[degC]: %6.2f\r\n", temperature_degc);
        }

        delay_ms(10);
    }
#endif /* DEMO_READ_DATA_POLLING */
#ifdef DEMO_READ_DATA_FIFO_POLLING
    /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);

    /* 使能时间戳计数器 */
    lsm6dsv16x_timestamp_set(&atk_ms6dsv, PROPERTY_ENABLE);

    /* 配置FIFO阈值 */
    lsm6dsv16x_fifo_watermark_set(&atk_ms6dsv, 64);

    /* 配置FIFO的传感器批处理数据率 */
    lsm6dsv16x_fifo_xl_batch_set(&atk_ms6dsv, LSM6DSV16X_XL_BATCHED_AT_60Hz);
    lsm6dsv16x_fifo_gy_batch_set(&atk_ms6dsv, LSM6DSV16X_GY_BATCHED_AT_60Hz);
    lsm6dsv16x_fifo_timestamp_batch_set(&atk_ms6dsv, LSM6DSV16X_TMSTMP_DEC_8);

    /* 配置FIFO模式 */
    lsm6dsv16x_fifo_mode_set(&atk_ms6dsv, LSM6DSV16X_STREAM_MODE);

    while (1)
    {
        /* 获取FIFO状态 */
        lsm6dsv16x_fifo_status_get(&atk_ms6dsv, &fifo_status);

        /* FIFO满 */
        if (fifo_status.fifo_th)
        {
            /* 获取FIFO中的数据 */
            lsm6dsv16x_fifo_out_raw_get(&atk_ms6dsv, &data_raw_fifo);
            datax = (int16_t *)&data_raw_fifo.data[0];
            datay = (int16_t *)&data_raw_fifo.data[2];
            dataz = (int16_t *)&data_raw_fifo.data[4];
            ts = (int32_t *)&data_raw_fifo.data[0];

            switch (data_raw_fifo.tag)
            {
            /* 加速度数据 */
            case LSM6DSV16X_XL_NC_TAG:
            {
                acceleration_mg[0] = lsm6dsv16x_from_fs2_to_mg(*datax);
                acceleration_mg[1] = lsm6dsv16x_from_fs2_to_mg(*datay);
                acceleration_mg[2] = lsm6dsv16x_from_fs2_to_mg(*dataz);
                break;
            }
            /* 角速度数据 */
            case LSM6DSV16X_GY_NC_TAG:
            {
                angular_rate_mdps[0] = lsm6dsv16x_from_fs2000_to_mdps(*datax);
                angular_rate_mdps[1] = lsm6dsv16x_from_fs2000_to_mdps(*datay);
                angular_rate_mdps[2] = lsm6dsv16x_from_fs2000_to_mdps(*dataz);
                break;
            }
            /* 时间戳数据 */
            case LSM6DSV16X_TIMESTAMP_TAG:
            {
                timestamp = *ts;
                break;
            }
            default:
            {
                break;
            }
            }
        }

        if (++times == 20)
        {
            times = 0;
            printf("\r\n******************************************\r\n");
            printf("Acceleration[mg]: %4.2f %4.2f %4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            printf("Angular rate[mdps]: %4.2f %4.2f %4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            printf("Timestamp[ms]: %d\r\n", timestamp);
        }

        delay_ms(10);
    }
#endif /* DEMO_READ_DATA_FIFO_POLLING */
#ifdef DEMO_QVAR_READ_DATA_POLLING
    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_15Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 配置滤波器 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
    lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);

    /* 使能静电传感器 */
    qvar_mode.ah_qvar_en = 1;
    lsm6dsv16x_ah_qvar_mode_set(&atk_ms6dsv, qvar_mode);

    while (1)
    {
        /* 获取状态 */
        lsm6dsv16x_all_sources_get(&atk_ms6dsv, &all_sources);

        /* 静电电压数据就绪 */
        if (all_sources.drdy_ah_qvar)
        {
            /* 获取静电电压数据 */
            lsm6dsv16x_ah_qvar_raw_get(&atk_ms6dsv, &data_raw_qvar);
            qvar_mv = lsm6dsv16x_from_lsb_to_mv(data_raw_qvar);
        }

        if (++times == 20)
        {
            times = 0;
            printf("\r\n******************************************\r\n");
            printf("QVAR[mV]: %6.2f\r\n", qvar_mv);
        }

        delay_ms(10);
    }
#endif /* DEMO_QVAR_READ_DATA_POLLING */
#ifdef DEMO_READ_DATA_INT
    /* 配置中断路由 */
    pin_int.drdy_xl = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int2_route_set(&atk_ms6dsv, &pin_int);

    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_120Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 配置滤波器 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
    lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);

    while (1)
    {
        /* 获取中断状态 */
        if (ATK_MS6DSV_READ_INT())
        {
            /* 获取数据就绪状态 */
            lsm6dsv16x_flag_data_ready_get(&atk_ms6dsv, &drdy);

            if (drdy.drdy_xl)
            {
                lsm6dsv16x_acceleration_raw_get(&atk_ms6dsv, data_raw_acceleration);
                acceleration_mg[0] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[0]);
                acceleration_mg[1] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[1]);
                acceleration_mg[2] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[2]);
            }

            if (++times == 24)
            {
                times = 0;
                printf("\r\n******************************************\r\n");
                printf("Acceleration[mg]: %4.2f %4.2f %4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            }
        }
    }
#endif /* DEMO_READ_DATA_INT */
#ifdef DEMO_READ_DATA_FIFO_INT
    /* 配置中断路由 */
    pin_int.fifo_th = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int2_route_set(&atk_ms6dsv, &pin_int);

    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 使能时间戳计数器 */
    lsm6dsv16x_timestamp_set(&atk_ms6dsv, PROPERTY_ENABLE);

    /* 配置FIFO阈值 */
    lsm6dsv16x_fifo_watermark_set(&atk_ms6dsv, 64);

    /* 配置FIFO的传感器批处理数据率 */
    lsm6dsv16x_fifo_xl_batch_set(&atk_ms6dsv, LSM6DSV16X_XL_BATCHED_AT_60Hz);
    lsm6dsv16x_fifo_timestamp_batch_set(&atk_ms6dsv, LSM6DSV16X_TMSTMP_DEC_32);

    /* 配置FIFO模式 */
    lsm6dsv16x_fifo_mode_set(&atk_ms6dsv, LSM6DSV16X_STREAM_MODE);

    while (1)
    {
        /* 获取中断状态 */
        if (ATK_MS6DSV_READ_INT())
        {
            /* 获取FIFO状态 */
            lsm6dsv16x_fifo_status_get(&atk_ms6dsv, &fifo_status);

            num = fifo_status.fifo_level;
            while (num--)
            {
                /* 获取FIFO中的数据 */
                lsm6dsv16x_fifo_out_raw_get(&atk_ms6dsv, &data_raw_fifo);
                datax = (int16_t *)&data_raw_fifo.data[0];
                datay = (int16_t *)&data_raw_fifo.data[2];
                dataz = (int16_t *)&data_raw_fifo.data[4];
                ts = (int32_t *)&data_raw_fifo.data[0];

                switch (data_raw_fifo.tag)
                {
                /* 加速度数据 */
                case LSM6DSV16X_XL_NC_TAG:
                {
                    acceleration_mg[0] = lsm6dsv16x_from_fs2_to_mg(*datax);
                    acceleration_mg[1] = lsm6dsv16x_from_fs2_to_mg(*datay);
                    acceleration_mg[2] = lsm6dsv16x_from_fs2_to_mg(*dataz);
                    break;
                }
                /* 时间戳数据 */
                case LSM6DSV16X_TIMESTAMP_TAG:
                {
                    timestamp = *ts;
                    break;
                }
                default:
                {
                    break;
                }
                }
            }
        }

        if (++times == 20)
        {
            times = 0;
            printf("\r\n******************************************\r\n");
            printf("Acceleration[mg]: %4.2f %4.2f %4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            printf("Timestamp[ms]: %d\r\n", timestamp);
        }

        delay_ms(10);
    }
#endif /* DEMO_READ_DATA_FIFO_INT */
#ifdef DEMO_FREE_FALL
    /* 配置中断路由 */
    pin_int.freefall = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int2_route_set(&atk_ms6dsv, &pin_int);

    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_120Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 使能中断 */
    irq.enable = PROPERTY_ENABLE;
    irq.lir = PROPERTY_ENABLE;
    lsm6dsv16x_interrupt_enable_set(&atk_ms6dsv, irq);

    /* 配置自由落体检测参数 */
    lsm6dsv16x_ff_time_windows_set(&atk_ms6dsv, 0);
    lsm6dsv16x_ff_thresholds_set(&atk_ms6dsv, LSM6DSV16X_312_mg);

    while (1)
    {
        /* 获取中断状态 */
        if (ATK_MS6DSV_READ_INT())
        {
            /* 获取状态 */
            lsm6dsv16x_all_sources_get(&atk_ms6dsv, &status);

            /* 自由落体事件 */
            if (status.free_fall)
            {
                printf("Free Fall event!\r\n");
            }
        }
    }
#endif /* DEMO_FREE_FALL */
#ifdef DEMO_WAKEUP
    /* 配置中断路由 */
    pin_int.wakeup = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int2_route_set(&atk_ms6dsv, &pin_int);

    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_480Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 使能中断 */
    irq.enable = PROPERTY_ENABLE;
    irq.lir = PROPERTY_ENABLE;
    lsm6dsv16x_interrupt_enable_set(&atk_ms6dsv, irq);

    /* 配置滤波器 */
    lsm6dsv16x_filt_xl_fast_settling_set(&atk_ms6dsv, 1);
    lsm6dsv16x_mask_trigger_xl_settl_set(&atk_ms6dsv, 1);
    lsm6dsv16x_filt_wkup_act_feed_set(&atk_ms6dsv, LSM6DSV16X_WK_FEED_HIGH_PASS);

    /* 配置唤醒检测参数 */
    wu.inactivity_cfg.inact_dur = 0;
    wu.inactivity_cfg.xl_inact_odr = 1;
    wu.inactivity_cfg.wu_inact_ths_w = 3;
    wu.inactivity_ths = 0;
    wu.threshold = 1;
    wu.duration = 0;
    lsm6dsv16x_act_thresholds_set(&atk_ms6dsv, &wu);

    while (1)
    {
        /* 获取中断状态 */
        if (ATK_MS6DSV_READ_INT())
        {
            /* 获取状态 */
            lsm6dsv16x_all_sources_get(&atk_ms6dsv, &status);

            /* 唤醒事件 */
            if (status.wake_up)
            {
                printf("Wakeup event!\r\n");
            }
        }
    }
#endif /* DEMO_WAKEUP */
#ifdef DEMO_6D
    /* 配置中断路由 */
    pin_int.sixd = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int2_route_set(&atk_ms6dsv, &pin_int);

    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_120Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 使能中断 */
    irq.enable = PROPERTY_ENABLE;
    irq.lir = PROPERTY_ENABLE;
    lsm6dsv16x_interrupt_enable_set(&atk_ms6dsv, irq);

    /* 配置滤波器 */
    lsm6dsv16x_filt_sixd_feed_set(&atk_ms6dsv, LSM6DSV16X_SIXD_FEED_LOW_PASS);

    /* 配置6D方向检测参数 */
    lsm6dsv16x_6d_threshold_set(&atk_ms6dsv, LSM6DSV16X_DEG_60);

    while (1)
    {
        /* 获取中断状态 */
        if (ATK_MS6DSV_READ_INT())
        {
            /* 获取状态 */
            lsm6dsv16x_all_sources_get(&atk_ms6dsv, &status);

            /* 6D事件 */
            if (status.six_d)
            {
                sixd_event = (status.six_d << 6) |
                             (status.six_d_zh << 5) |
                             (status.six_d_zl << 4) |
                             (status.six_d_yh << 3) |
                             (status.six_d_yl << 2) |
                             (status.six_d_xh << 1) |
                             (status.six_d_xl << 0);
                switch (sixd_event)
                {
                case 0x48:
                {
                    printf("6D Position A!\r\n");
                    break;
                }
                case 0x41:
                {
                    printf("6D Position B!\r\n");
                    break;
                }
                case 0x42:
                {
                    printf("6D Position C!\r\n");
                    break;
                }
                case 0x44:
                {
                    printf("6D Position D!\r\n");
                    break;
                }
                case 0x60:
                {
                    printf("6D Position E!\r\n");
                    break;
                }
                case 0x50:
                {
                    printf("6D Position F!\r\n");
                    break;
                }
                default:
                {
                    break;
                }
                }
            }
        }
    }
#endif /* DEMO_6D */
#ifdef DEMO_TAP
    /* 配置中断路由 */
    pin_int.double_tap = PROPERTY_ENABLE;
    pin_int.single_tap = PROPERTY_ENABLE;
    lsm6dsv16x_pin_int2_route_set(&atk_ms6dsv, &pin_int);

    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_480Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_8g);

    /* 使能中断 */
    irq.enable = PROPERTY_ENABLE;
    irq.lir = PROPERTY_DISABLE;
    lsm6dsv16x_interrupt_enable_set(&atk_ms6dsv, irq);

    /* 配置单击双击检测参数 */
    tap.tap_z_en = PROPERTY_ENABLE;
    lsm6dsv16x_tap_detection_set(&atk_ms6dsv, tap);
    tap_ths.z = 3;
    lsm6dsv16x_tap_thresholds_set(&atk_ms6dsv, tap_ths);
    tap_win.tap_gap = 7;
    tap_win.shock = 3;
    tap_win.quiet = 3;
    lsm6dsv16x_tap_time_windows_set(&atk_ms6dsv, tap_win);
    lsm6dsv16x_tap_mode_set(&atk_ms6dsv, LSM6DSV16X_BOTH_SINGLE_DOUBLE);

    while (1)
    {
        /* 获取中断状态 */
        if (ATK_MS6DSV_READ_INT())
        {
            /* 获取状态 */
            lsm6dsv16x_all_sources_get(&atk_ms6dsv, &status);

            /* 单击事件 */
            if (status.single_tap)
            {
                printf("Single TAP!\r\n");
            }

            /* 双击事件 */
            if (status.double_tap)
            {
                printf("Double TAP!\r\n");
            }
        }
    }
#endif /* DEMO_TAP */
#ifdef DEMO_STEP_COUNTER
    /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_30Hz);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_OFF);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);

    /* 配置滤波器 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
    lsm6dsv16x_filt_gy_lp1_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_GY_ULTRA_LIGHT);
    lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);

    /* 配置步数检测参数 */
    stpcnt_mode.step_counter_enable = PROPERTY_ENABLE;
    stpcnt_mode.false_step_rej = PROPERTY_ENABLE;
    lsm6dsv16x_stpcnt_mode_set(&atk_ms6dsv, stpcnt_mode);
    lsm6dsv16x_stpcnt_rst_step_set(&atk_ms6dsv, PROPERTY_ENABLE);

    while (1)
    {
        /* 获取状态 */
        lsm6dsv16x_all_sources_get(&atk_ms6dsv, &status);

        /* 步数检测事件 */
        if (status.step_detector)
        {
            lsm6dsv16x_stpcnt_steps_get(&atk_ms6dsv, &steps);
            printf("Steps: %d\r\n", steps);
        }
    }
#endif /* DEMO_STEP_COUNTER */
#ifdef DEMO_SENSOR_FUSION
    /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_30Hz);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_30Hz);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);

    /* 配置FIFO阈值 */
    lsm6dsv16x_fifo_watermark_set(&atk_ms6dsv, 32);

    /* 配置FIFO的传感器融合数据批处理 */
    fifo_sflp.game_rotation = PROPERTY_ENABLE;
    fifo_sflp.gravity = PROPERTY_ENABLE;
    fifo_sflp.gbias = PROPERTY_ENABLE;
    lsm6dsv16x_fifo_sflp_batch_set(&atk_ms6dsv, fifo_sflp);

    /* 配置FIFO模式 */
    lsm6dsv16x_fifo_mode_set(&atk_ms6dsv, LSM6DSV16X_STREAM_MODE);

    /* 配置传感器融合参数 */
    lsm6dsv16x_sflp_data_rate_set(&atk_ms6dsv, LSM6DSV16X_SFLP_30Hz);
    lsm6dsv16x_sflp_game_rotation_set(&atk_ms6dsv, PROPERTY_ENABLE);
    gbias.gbias_x = 0.0f;
    gbias.gbias_y = 0.0f;
    gbias.gbias_z = 0.0f;
    lsm6dsv16x_sflp_game_gbias_set(&atk_ms6dsv, &gbias);

    while (1)
    {
        /* 获取FIFO状态 */
        lsm6dsv16x_fifo_status_get(&atk_ms6dsv, &fifo_status);

        num = fifo_status.fifo_level;
        while (num--)
        {
            /* 获取FIFO中的数据 */
            lsm6dsv16x_fifo_out_raw_get(&atk_ms6dsv, &data_raw_fifo);

            switch (data_raw_fifo.tag)
            {
            /* 陀螺仪偏差 */
            case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
            {
                axis = (int16_t *)&data_raw_fifo.data[0];
                gbias_mdps[0] = lsm6dsv16x_from_fs125_to_mdps(axis[0]);
                gbias_mdps[1] = lsm6dsv16x_from_fs125_to_mdps(axis[1]);
                gbias_mdps[2] = lsm6dsv16x_from_fs125_to_mdps(axis[2]);
                break;
            }
            /* 重力向量 */
            case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
            {
                axis = (int16_t *)&data_raw_fifo.data[0];
                gravity_mg[0] = lsm6dsv16x_from_sflp_to_mg(axis[0]);
                gravity_mg[1] = lsm6dsv16x_from_sflp_to_mg(axis[1]);
                gravity_mg[2] = lsm6dsv16x_from_sflp_to_mg(axis[2]);
                break;
            }
            /* 游戏旋转矢量 */
            case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
            {
                sflp2q(quat, (uint16_t *)&data_raw_fifo.data[0]);
                break;
            }
            default:
            {
                break;
            }
            }
        }

        if (++times == 20)
        {
            times = 0;
            printf("\r\n******************************************\r\n");
            printf("GBIAS[mdps]: %4.2f %4.2f %4.2f\r\n", gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]);
            printf("Gravity[mg]: %4.2f %4.2f %4.2f\r\n", gravity_mg[0], gravity_mg[1], gravity_mg[2]);
            printf("Game Rotation: %2.3f %2.3f %2.3f %2.3f\r\n", quat[0], quat[1], quat[2], quat[3]);
        }

        delay_ms(10);
    }
#endif /* DEMO_SENSOR_FUSION */
#ifdef DEMO_SENSOR_FUSION_ANOTC
    /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_15Hz);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_15Hz);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_125dps);

    /* 计算陀螺仪零偏值 */
    {
        for (times = 0; times < 200;)
        {
            lsm6dsv16x_flag_data_ready_get(&atk_ms6dsv, &drdy);

            if (drdy.drdy_gy)
            {
                lsm6dsv16x_angular_rate_raw_get(&atk_ms6dsv, data_raw_angular_rate);
                angular_rate_mdps_avg[0] += lsm6dsv16x_from_fs125_to_mdps(data_raw_angular_rate[0]);
                angular_rate_mdps_avg[1] += lsm6dsv16x_from_fs125_to_mdps(data_raw_angular_rate[1]);
                angular_rate_mdps_avg[2] += lsm6dsv16x_from_fs125_to_mdps(data_raw_angular_rate[2]);
                times++;
            }
        }
        angular_rate_mdps_avg[0] /= 200;
        angular_rate_mdps_avg[1] /= 200;
        angular_rate_mdps_avg[2] /= 200;
    }

    /* 配置FIFO阈值 */
    lsm6dsv16x_fifo_watermark_set(&atk_ms6dsv, 32);

    /* 配置FIFO的传感器融合数据批处理 */
    fifo_sflp.game_rotation = PROPERTY_ENABLE;
    lsm6dsv16x_fifo_sflp_batch_set(&atk_ms6dsv, fifo_sflp);

    /* 配置FIFO模式 */
    lsm6dsv16x_fifo_mode_set(&atk_ms6dsv, LSM6DSV16X_STREAM_MODE);

    /* 配置传感器融合参数 */
    lsm6dsv16x_sflp_game_rotation_set(&atk_ms6dsv, PROPERTY_DISABLE);
    lsm6dsv16x_sflp_data_rate_set(&atk_ms6dsv, LSM6DSV16X_SFLP_15Hz);
    lsm6dsv16x_sflp_game_rotation_set(&atk_ms6dsv, PROPERTY_ENABLE);
    gbias.gbias_x = angular_rate_mdps_avg[0] / 1000.0;
    gbias.gbias_y = angular_rate_mdps_avg[1] / 1000.0;
    gbias.gbias_z = angular_rate_mdps_avg[2] / 1000.0;
    lsm6dsv16x_sflp_game_gbias_set(&atk_ms6dsv, &gbias);

    while (1)
    {
        /* 获取FIFO状态 */
        lsm6dsv16x_fifo_status_get(&atk_ms6dsv, &fifo_status);

        num = fifo_status.fifo_level;
        while (num--)
        {
            /* 获取FIFO中的数据 */
            lsm6dsv16x_fifo_out_raw_get(&atk_ms6dsv, &data_raw_fifo);

            switch (data_raw_fifo.tag)
            {
            /* 游戏旋转矢量 */
            case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
            {
                sflp2q(quat, (uint16_t *)&data_raw_fifo.data[0]);
                pitch = atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3])) * 57.3;
                if (pitch > 0)
                {
                    pitch = 180 - pitch;
                }
                else
                {
                    pitch = -pitch - 180;
                }
                roll = -asin(2 * (quat[0] * quat[2] - quat[3] * quat[1])) * 57.3;
                yaw = -atan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])) * 57.3;
                demo_niming_report_status((int16_t)(roll * 100), (int16_t)((pitch) * 100), (int16_t)(yaw * 100), 0, 0, 0);
                break;
            }
            default:
            {
                break;
            }
            }
        }
    }
#endif /* DEMO_SENSOR_FUSION_ANOTC */
#ifdef DEMO_FSM_TOUCH
    /* 配置加速度计的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_15Hz);

    /* 配置加速度计的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);

    /* 配置滤波器 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
    lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);

    /* 使能静电传感器 */
    qvar_mode.ah_qvar_en = 1;
    lsm6dsv16x_ah_qvar_mode_set(&atk_ms6dsv, qvar_mode);

    /* 配置状态机 */
    for (reg_index = 0; reg_index < (sizeof(lsm6dsv16x_fsm_touch) / sizeof(ucf_line_t)); reg_index++)
    {
        lsm6dsv16x_write_reg(&atk_ms6dsv, lsm6dsv16x_fsm_touch[reg_index].address, (uint8_t *)&lsm6dsv16x_fsm_touch[reg_index].data, 1);
    }

    while (1)
    {
        /* 获取状态机状态 */
        lsm6dsv16x_read_reg(&atk_ms6dsv, LSM6DSV16X_FSM_STATUS_MAINPAGE, (uint8_t *)&fsm_status, 1);

        if (fsm_status.is_fsm1 || fsm_status.is_fsm2 || fsm_status.is_fsm3 || fsm_status.is_fsm4)
        {
            /* 长按事件 */
            if (fsm_status.is_fsm4)
            {
                printf("Long Touch!\r\n");
            }
            /* 三连击事件 */
            if (fsm_status.is_fsm3)
            {
                printf("Triple Touch!\r\n");
            }
            /* 双击事件 */
            else if (fsm_status.is_fsm2)
            {
                printf("Double Touch!\r\n");
            }
            /* 单击事件 */
            else if (fsm_status.is_fsm1)
            {
                printf("Single Touch!\r\n");
            }

            HAL_Delay(100);
        }
    }
#endif /* DEMO_FSM_TOUCH */
#ifdef DEMO_FSM_4D
    /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_7Hz5);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_15Hz);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);

    /* 配置滤波器 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
    lsm6dsv16x_filt_gy_lp1_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_GY_ULTRA_LIGHT);
    lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);

    /* 配置状态机 */
    for (reg_index = 0; reg_index < (sizeof(lsm6dsv16x_four_d) / sizeof(ucf_line_t)); reg_index++)
    {
        lsm6dsv16x_write_reg(&atk_ms6dsv, lsm6dsv16x_four_d[reg_index].address, (uint8_t *)&lsm6dsv16x_four_d[reg_index].data, 1);
    }

    while (1)
    {
        /* 获取状态机状态 */
        lsm6dsv16x_read_reg(&atk_ms6dsv, LSM6DSV16X_FSM_STATUS_MAINPAGE, (uint8_t *)&fsm_status, 1);

        if (fsm_status.is_fsm1)
        {
            /* 获取状态机输出数据 */
            lsm6dsv16x_fsm_out_get(&atk_ms6dsv, &fsm_out);
            switch (fsm_out.fsm_outs1)
            {
            /* Y轴朝下事件 */
            case 0x10:
            {
                printf("Y down event!\r\n");
                break;
            }
            /* Y轴朝上事件 */
            case 0x20:
            {
                printf("Y up event!\r\n");
                break;
            }
            /* X轴朝下事件 */
            case 0x40:
            {
                printf("X down event!\r\n");
                break;
            }
            /* X轴朝上事件 */
            case 0x80:
            {
                printf("X up event!\r\n");
                break;
            }
            }

            HAL_Delay(100);
        }
    }
#endif /* DEMO_FSM_4D */
#ifdef RUN_YAW_ROLL_PITCH
#endif /*RUN_YAW_ROLL_PITCH*/
}
