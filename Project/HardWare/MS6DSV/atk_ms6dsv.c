/**
 ****************************************************************************************************
 * @file        atk_ms6dsv.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6DSV模块驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_ms6dsv.h"
#include "atk_ms6dsv_iic.h"
#include "lsm6dsv16x_reg.h"
#include "delay.h"
#include "usart.h"

/* ATK-MS6DSV模块IIC通讯地址 */
static uint8_t atk_ms6dsv_iic_addr;

/**
 * @brief       ATK-MS6DSV硬件初始化
 * @param       无
 * @retval      无
 */
static void atk_ms6dsv_hw_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能GPIO时钟 */
    ATK_MS6DSV_SA0_GPIO_CLK_ENABLE();
    ATK_MS6DSV_INT_GPIO_CLK_ENABLE();
    
    /* 初始化AD0引脚 */
    gpio_init_struct.Pin    = ATK_MS6DSV_SA0_GPIO_PIN;          /* AD0引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;          /* 推挽输出 */
    gpio_init_struct.Pull   = GPIO_PULLUP;                  /* 上拉 */
    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
    HAL_GPIO_Init(ATK_MS6DSV_SA0_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化INT引脚 */
    gpio_init_struct.Pin    = ATK_MS6DSV_INT_GPIO_PIN;          /* INT引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_INPUT;              /* 输入 */
    gpio_init_struct.Pull   = GPIO_PULLDOWN;                /* 下拉 */
    HAL_GPIO_Init(ATK_MS6DSV_INT_GPIO_PORT, &gpio_init_struct);
    
    /* 控制ATK-MS6DSV的SA0引脚为低电平
     * 设置其IIC的从机地址为0xD5(LSM6DSV16X_I2C_ADD_L)
     */
    ATK_MS6DSV_SA0(0);
    atk_ms6dsv_iic_addr = LSM6DSV16X_I2C_ADD_L >> 1;
}

/**
 * @brief       往ATK-MS6DSV的指定寄存器连续写入指定数据
 * @param       addr: ATK-MS6DSV的IIC通讯地址
 *              reg : ATK-MS6DSV寄存器地址
 *              len : 写入的长度
 *              dat : 写入的数据
 * @retval      ATK_MS6DSV_EOK : 函数执行成功
 *              ATK_MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6dsv_write(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *dat)
{
    uint8_t i;
    
    atk_ms6dsv_iic_start();
    atk_ms6dsv_iic_send_byte((addr << 1) | 0);
    if (atk_ms6dsv_iic_wait_ack() == 1)
    {
        atk_ms6dsv_iic_stop();
        return ATK_MS6DSV_EACK;
    }
    atk_ms6dsv_iic_send_byte(reg);
    if (atk_ms6dsv_iic_wait_ack() == 1)
    {
        atk_ms6dsv_iic_stop();
        return ATK_MS6DSV_EACK;
    }
    for (i=0; i<len; i++)
    {
        atk_ms6dsv_iic_send_byte(dat[i]);
        if (atk_ms6dsv_iic_wait_ack() == 1)
        {
            atk_ms6dsv_iic_stop();
            return ATK_MS6DSV_EACK;
        }
    }
    atk_ms6dsv_iic_stop();
    return ATK_MS6DSV_EOK;
}

/**
 * @brief       连续读取ATK-MS6DSV指定寄存器的值
 * @param       addr: ATK-MS6DSV的IIC通讯地址
 *              reg : ATK-MS6DSV寄存器地址
 *              len: 读取的长度
 *              dat: 存放读取到的数据的地址
 * @retval      ATK_MS6DSV_EOK : 函数执行成功
 *              ATK_MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6dsv_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dat)
{
    atk_ms6dsv_iic_start();
    atk_ms6dsv_iic_send_byte((addr << 1) | 0);
    if (atk_ms6dsv_iic_wait_ack() == 1)
    {
        atk_ms6dsv_iic_stop();
        return ATK_MS6DSV_EACK;
    }
    atk_ms6dsv_iic_send_byte(reg);
    if (atk_ms6dsv_iic_wait_ack() == 1)
    {
        atk_ms6dsv_iic_stop();
        return ATK_MS6DSV_EACK;
    }
    atk_ms6dsv_iic_start();
    atk_ms6dsv_iic_send_byte((addr << 1) | 1);
    if (atk_ms6dsv_iic_wait_ack() == 1)
    {
        atk_ms6dsv_iic_stop();
        return ATK_MS6DSV_EACK;
    }
    while (len)
    {
        *dat = atk_ms6dsv_iic_read_byte((len > 1) ? 1 : 0);
        len--;
        dat++;
    }
    atk_ms6dsv_iic_stop();
    return ATK_MS6DSV_EOK;
}

/**
 * @brief       ATK-MS6DSV初始化
 * @param       无
 * @retval      ATK_MS6DSV_EOK: 函数执行成功
 *              ATK_MS6DSV_EID: 获取ID错误，函数执行失败
 */
uint8_t atk_ms6dsv_init(void)
{
    uint8_t lsm6dsv16x_id;
    lsm6dsv16x_reset_t rst;
    
    /* 初始化硬件接口 */
    atk_ms6dsv_hw_init();
    atk_ms6dsv_iic_init();
    
    /* 校验ATK-MS6DSV模块ID */
    lsm6dsv16x_device_id_get(&atk_ms6dsv, &lsm6dsv16x_id);
    if (lsm6dsv16x_id != LSM6DSV16X_ID)
    {
        return ATK_MS6DSV_EID;
    }
    
    /* 复位ATK-MS6DSV模块 */
    lsm6dsv16x_reset_set(&atk_ms6dsv, LSM6DSV16X_RESTORE_CTRL_REGS);
    do {
        lsm6dsv16x_reset_get(&atk_ms6dsv, &rst);
    } while (rst != LSM6DSV16X_READY);
    
    /* 使能块数据更新功能 */
    lsm6dsv16x_block_data_update_set(&atk_ms6dsv, PROPERTY_ENABLE);
    
    return ATK_MS6DSV_EOK;
}

/**
 * @brief       LSM6DSV16X写寄存器函数
 * @param       handle: 未使用
 *              reg : LSM6DSV16X寄存器地址
 *              bufp: 数据
 *              len: 数据长度
 * @retval      ATK_MS6DSV_EOK : 函数执行成功
 *              ATK_MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
static int32_t atk_ms6dsv_i2c_write_reg(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    return atk_ms6dsv_write(atk_ms6dsv_iic_addr, reg, len, (uint8_t *)bufp);
}

/**
 * @brief       LSM6DSV16X读寄存器函数
 * @param       handle: 未使用
 *              reg : LSM6DSV16X寄存器地址
 *              bufp: 数据
 *              len: 数据长度
 * @retval      ATK_MS6DSV_EOK : 函数执行成功
 *              ATK_MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
static int32_t atk_ms6dsv_i2c_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    return atk_ms6dsv_read(atk_ms6dsv_iic_addr, reg, len, (uint8_t *)bufp);
}

/**
 * @brief       LSM6DSV16X延时函数
 * @param       millisec: 延时时间，单位：毫秒
 * @retval      无
 */
static void atk_ms6dsv_delay(uint32_t millisec)
{
    delay_ms(millisec);
}

/* 定义ATK-MD模块对象 */
stmdev_ctx_t atk_ms6dsv = {
    .write_reg = atk_ms6dsv_i2c_write_reg,
    .read_reg = atk_ms6dsv_i2c_read_reg,
    .mdelay = atk_ms6dsv_delay,
    .handle = (void *)0,
};
