#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"

#include <stdint.h>

#define SERVO_MIN_PULSE_US       1000U
#define SERVO_MAX_PULSE_US       1778U
#define SERVO_MAX_ANGLE_DEG        70U

/*
 * 机械端点标定值。第一次装机时先卸下舵盘，小步调整这两个脉宽，
 * 确认不会顶住机械限位后再装回机构。
 */
#define SERVO_RAISED_ANGLE_DEG     70U
#define SERVO_LOWERED_ANGLE_DEG     0U
#define SERVO_RAISED_PULSE_US    1778U
#define SERVO_LOWERED_PULSE_US   1000U

/* 启动 TIM4_CH1 的 50 Hz PWM，并先转到安全的抬起位置。 */
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *pwm_timer);

/* 直接设置脉宽，超出 1000～2000 us 的值会被钳位。 */
void Servo_SetPulseUs(uint16_t pulse_us);

/* 舵机角度控制:0°=1000us(放下),70°=1778us(抬起),超出 0～70° 会被钳位。 */
void Servo_SetAngleDeg(uint16_t angle_deg);

/* 套取机构的三个动作接口。HOLD 会持续保持放下位置。 */
void Servo_Raise(void);
void Servo_Lower(void);
void Servo_Hold(void);

uint16_t Servo_GetAngleDeg(void);
uint16_t Servo_GetPulseUs(void);

#endif /* SERVO_H */
