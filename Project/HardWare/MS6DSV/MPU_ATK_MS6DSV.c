#include "MPU_ATK_MS6DSV.H"

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
float pitch,roll,yaw;


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

void USER_MS6DSV_INIT(void)
{
		unsigned char ret;
		 ret = atk_ms6dsv_init();
    if (ret != 0)
    {
//        UART_Printf("ATK-MS6DSV init failed!\r\n");
        while (1)
        {
            delay_ms(200);
        }
    }
//		UART_Printf("USER_MS6DSV_INIT.\r\n");
 /* 配置加速度计和陀螺仪的ODR */
    lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_480Hz);
    lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_480Hz);

    /* 配置加速度计和陀螺仪的量程 */
    lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_125dps);
//		UART_Printf("USER_MS6DSV_INIT..\r\n");
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
//		UART_Printf("USER_MS6DSV_INIT...\r\n");
		delay_ms(2000);
}

void USER_MS6DSV_YAW_ROLL_PITCH(void)
{

//    while (1)
//    {
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
                break;
            }
            default:
            {
                break;
            }
            }
        }
//    }
}
