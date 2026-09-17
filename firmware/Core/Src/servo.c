#include "servo.h"

#include <stddef.h>

#if (SERVO_RAISED_PULSE_US < SERVO_MIN_PULSE_US) || \
    (SERVO_RAISED_PULSE_US > SERVO_MAX_PULSE_US)
#error "SERVO_RAISED_PULSE_US is outside the configured servo pulse range"
#endif

#if (SERVO_LOWERED_PULSE_US < SERVO_MIN_PULSE_US) || \
    (SERVO_LOWERED_PULSE_US > SERVO_MAX_PULSE_US)
#error "SERVO_LOWERED_PULSE_US is outside the configured servo pulse range"
#endif

static TIM_HandleTypeDef *servo_timer;
static uint16_t servo_pulse_us = SERVO_RAISED_PULSE_US;
static uint16_t servo_angle_deg = SERVO_RAISED_ANGLE_DEG;

static uint16_t angle_to_pulse_us(uint16_t angle_deg)
{
    if (angle_deg > SERVO_MAX_ANGLE_DEG)
    {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }

    /* 0° = 1000us,70° = 1778us。 */
    return (uint16_t)(
        SERVO_MIN_PULSE_US +
        ((((uint32_t)angle_deg *
           (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) +
          (SERVO_MAX_ANGLE_DEG / 2U)) /
         SERVO_MAX_ANGLE_DEG));
}

HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *pwm_timer)
{
    if (pwm_timer == NULL || pwm_timer->Instance != TIM4)
    {
        return HAL_ERROR;
    }

    servo_timer = pwm_timer;
    servo_pulse_us = SERVO_RAISED_PULSE_US;
    servo_angle_deg = SERVO_RAISED_ANGLE_DEG;
    __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_1, servo_pulse_us);

    return HAL_TIM_PWM_Start(servo_timer, TIM_CHANNEL_1);
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    servo_pulse_us = pulse_us;
    servo_angle_deg = (uint16_t)(
        ((((uint32_t)pulse_us - SERVO_MIN_PULSE_US) *
          SERVO_MAX_ANGLE_DEG) +
         ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2U)) /
        (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));

    if (servo_timer != NULL)
    {
        __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_1, servo_pulse_us);
    }
}

void Servo_SetAngleDeg(uint16_t angle_deg)
{
    if (angle_deg > SERVO_MAX_ANGLE_DEG)
    {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }

    Servo_SetPulseUs(angle_to_pulse_us(angle_deg));
    servo_angle_deg = angle_deg;
}

void Servo_Raise(void)
{
    Servo_SetPulseUs(SERVO_RAISED_PULSE_US);
    servo_angle_deg = SERVO_RAISED_ANGLE_DEG;
}

void Servo_Lower(void)
{
    Servo_SetPulseUs(SERVO_LOWERED_PULSE_US);
    servo_angle_deg = SERVO_LOWERED_ANGLE_DEG;
}

void Servo_Hold(void)
{
    Servo_Lower();
}

uint16_t Servo_GetAngleDeg(void)
{
    return servo_angle_deg;
}

uint16_t Servo_GetPulseUs(void)
{
    return servo_pulse_us;
}
