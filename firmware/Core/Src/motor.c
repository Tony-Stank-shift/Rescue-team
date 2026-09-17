#include "motor.h"

#include "main.h"

#define MOTOR_LEFT_PWM_CHANNEL  TIM_CHANNEL_1
#define MOTOR_RIGHT_PWM_CHANNEL TIM_CHANNEL_4

typedef struct
{
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t pwm_channel;
    bool invert;
    int16_t command;
} MotorChannel;

static TIM_HandleTypeDef *motor_pwm_timer;
static bool motor_enabled;

static MotorChannel left_motor = {
    .in1_port = LAIN1_GPIO_Port,
    .in1_pin = LAIN1_Pin,
    .in2_port = LAIN2_GPIO_Port,
    .in2_pin = LAIN2_Pin,
    .pwm_channel = MOTOR_LEFT_PWM_CHANNEL,
    .invert = true,
};

static MotorChannel right_motor = {
    .in1_port = RAIN1_GPIO_Port,
    .in1_pin = RAIN1_Pin,
    .in2_port = RAIN2_GPIO_Port,
    .in2_pin = RAIN2_Pin,
    .pwm_channel = MOTOR_RIGHT_PWM_CHANNEL,
    .invert = false,
};

static int16_t Motor_ClampPercent(int16_t percent)
{
    if (percent > 100)
    {
        return 100;
    }
    if (percent < -100)
    {
        return -100;
    }
    return percent;
}

static uint32_t Motor_PercentToPulse(uint16_t percent)
{
    uint32_t period_counts;

    period_counts = __HAL_TIM_GET_AUTORELOAD(motor_pwm_timer) + 1U;
    return (period_counts * (uint32_t)percent + 50U) / 100U;
}

static void Motor_SetDirection(MotorChannel *motor, GPIO_PinState in1,
                               GPIO_PinState in2)
{
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, in1);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, in2);
}

static void Motor_CoastOne(MotorChannel *motor)
{
    uint32_t full_pulse;

    __HAL_TIM_SET_COMPARE(motor_pwm_timer, motor->pwm_channel, 0U);
    Motor_SetDirection(motor, GPIO_PIN_RESET, GPIO_PIN_RESET);

    /* TB6612: IN1=IN2=0 且 PWM=1 时为高阻滑行。 */
    full_pulse = __HAL_TIM_GET_AUTORELOAD(motor_pwm_timer) + 1U;
    __HAL_TIM_SET_COMPARE(motor_pwm_timer, motor->pwm_channel, full_pulse);
    motor->command = 0;
}

static void Motor_BrakeOne(MotorChannel *motor)
{
    __HAL_TIM_SET_COMPARE(motor_pwm_timer, motor->pwm_channel, 0U);
    Motor_SetDirection(motor, GPIO_PIN_SET, GPIO_PIN_SET);
    motor->command = 0;
}

static void Motor_SetOne(MotorChannel *motor, int16_t percent)
{
    uint16_t magnitude;
    int16_t logical_percent;

    percent = Motor_ClampPercent(percent);
    logical_percent = percent;
    if (motor->invert)
    {
        percent = (int16_t)(-percent);
    }
    if (percent == 0)
    {
        Motor_CoastOne(motor);
        motor->command = logical_percent;
        return;
    }

    __HAL_TIM_SET_COMPARE(motor_pwm_timer, motor->pwm_channel, 0U);
    if (percent > 0)
    {
        Motor_SetDirection(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
        magnitude = (uint16_t)percent;
    }
    else
    {
        Motor_SetDirection(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
        magnitude = (uint16_t)(-percent);
    }

    __HAL_TIM_SET_COMPARE(
        motor_pwm_timer,
        motor->pwm_channel,
        Motor_PercentToPulse(magnitude));
    motor->command = logical_percent;
}

HAL_StatusTypeDef Motor_Init(TIM_HandleTypeDef *pwm_timer)
{
    if (pwm_timer == NULL)
    {
        return HAL_ERROR;
    }

    motor_pwm_timer = pwm_timer;
    motor_enabled = false;
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
    Motor_SetDirection(&left_motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
    Motor_SetDirection(&right_motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_pwm_timer, MOTOR_LEFT_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(motor_pwm_timer, MOTOR_RIGHT_PWM_CHANNEL, 0U);

    if (HAL_TIM_PWM_Start(motor_pwm_timer, MOTOR_LEFT_PWM_CHANNEL) != HAL_OK)
    {
        motor_pwm_timer = NULL;
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(motor_pwm_timer, MOTOR_RIGHT_PWM_CHANNEL) != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(pwm_timer, MOTOR_LEFT_PWM_CHANNEL);
        motor_pwm_timer = NULL;
        return HAL_ERROR;
    }

    Motor_Stop();
    return HAL_OK;
}

void Motor_Enable(void)
{
    if (motor_pwm_timer == NULL)
    {
        return;
    }
    Motor_Stop();
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    motor_enabled = true;
}

void Motor_Disable(void)
{
    motor_enabled = false;
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

    if (motor_pwm_timer != NULL)
    {
        __HAL_TIM_SET_COMPARE(motor_pwm_timer, MOTOR_LEFT_PWM_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(motor_pwm_timer, MOTOR_RIGHT_PWM_CHANNEL, 0U);
        Motor_SetDirection(&left_motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
        Motor_SetDirection(&right_motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
        left_motor.command = 0;
        right_motor.command = 0;
    }
}

bool Motor_IsEnabled(void)
{
    return motor_enabled;
}

void Motor_Set(int16_t left_percent, int16_t right_percent)
{
    if ((motor_pwm_timer == NULL) || !motor_enabled)
    {
        return;
    }
    Motor_SetOne(&left_motor, left_percent);
    Motor_SetOne(&right_motor, right_percent);
}

void Motor_SetLeft(int16_t percent)
{
    if ((motor_pwm_timer != NULL) && motor_enabled)
    {
        Motor_SetOne(&left_motor, percent);
    }
}

void Motor_SetRight(int16_t percent)
{
    if ((motor_pwm_timer != NULL) && motor_enabled)
    {
        Motor_SetOne(&right_motor, percent);
    }
}

void Motor_Stop(void)
{
    if (motor_pwm_timer == NULL)
    {
        return;
    }
    Motor_CoastOne(&left_motor);
    Motor_CoastOne(&right_motor);
}

void Motor_Brake(void)
{
    if ((motor_pwm_timer == NULL) || !motor_enabled)
    {
        return;
    }
    Motor_BrakeOne(&left_motor);
    Motor_BrakeOne(&right_motor);
}

int16_t Motor_GetLeftCommand(void)
{
    return left_motor.command;
}

int16_t Motor_GetRightCommand(void)
{
    return right_motor.command;
}
