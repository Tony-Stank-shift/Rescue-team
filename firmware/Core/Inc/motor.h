#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* 初始化 TIM1 的 CH1/CH4 双路 PWM；初始化完成后仍保持 STBY=0。 */
HAL_StatusTypeDef Motor_Init(TIM_HandleTypeDef *pwm_timer);

/* 控制 TB6612 的 STBY。ESTOP 和故障处理应调用 Motor_Disable。 */
void Motor_Enable(void);
void Motor_Disable(void);
bool Motor_IsEnabled(void);

/* 开环 PWM 百分比，范围 -100～100，正数表示车轮向前。 */
void Motor_Set(int16_t left_percent, int16_t right_percent);
void Motor_SetLeft(int16_t percent);
void Motor_SetRight(int16_t percent);

/* 滑行停车与短刹车。 */
void Motor_Stop(void);
void Motor_Brake(void);

int16_t Motor_GetLeftCommand(void);
int16_t Motor_GetRightCommand(void);

#endif /* MOTOR_H */
