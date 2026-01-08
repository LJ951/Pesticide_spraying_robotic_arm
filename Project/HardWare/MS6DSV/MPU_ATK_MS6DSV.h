#ifndef __MPU_ATK_MS6DSV_H_
#define __MPU_ATK_MS6DSV_H_

#include "atk_ms6dsv.h"
#include "UART.h"
#include "delay.h"
#include <stdio.h>

void USER_MS6DSV_YAW_ROLL_PITCH(void);
void USER_MS6DSV_INIT(void);

extern float pitch,roll,yaw;

#endif
