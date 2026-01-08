/**
 ****************************************************************************************************
 * @file        atk_ms6dsv.h
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

#ifndef __ATK_MS6DSV_H
#define __ATK_MS6DSV_H

#include "sys.h"
#include "lsm6dsv16x_reg.h"

/* 引脚定义 */
#define ATK_MS6DSV_SA0_GPIO_PORT            GPIOC
#define ATK_MS6DSV_SA0_GPIO_PIN             GPIO_PIN_0
#define ATK_MS6DSV_SA0_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)
#define ATK_MS6DSV_INT_GPIO_PORT            GPIOC
#define ATK_MS6DSV_INT_GPIO_PIN             GPIO_PIN_1
#define ATK_MS6DSV_INT_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* IO操作 */
#define ATK_MS6DSV_SA0(x)                   do{ x ?                                                                             \
                                                HAL_GPIO_WritePin(ATK_MS6DSV_SA0_GPIO_PORT, ATK_MS6DSV_SA0_GPIO_PIN, GPIO_PIN_SET) :    \
                                                HAL_GPIO_WritePin(ATK_MS6DSV_SA0_GPIO_PORT, ATK_MS6DSV_SA0_GPIO_PIN, GPIO_PIN_RESET);   \
                                            }while(0)
#define ATK_MS6DSV_READ_INT()               HAL_GPIO_ReadPin(ATK_MS6DSV_INT_GPIO_PORT, ATK_MS6DSV_INT_GPIO_PIN)

/* 导出ATK-MS6DSV模块对象 */
extern stmdev_ctx_t atk_ms6dsv;

/* 函数错误代码 */
#define ATK_MS6DSV_EOK      0   /* 没有错误 */
#define ATK_MS6DSV_EID      1   /* ID错误 */
#define ATK_MS6DSV_EACK     2   /* IIC通讯ACK错误 */
#define ATK_MS6DSV_EINVAL   3   /* 传参错误 */

/* 操作函数 */
uint8_t atk_ms6dsv_init(void);  /* ATK-MS6DSV初始化 */

#endif
