#include "speed_control.h"

#include "motor.h"

#include <stdint.h>

#define SPEED_CONTROL_FILTER_ALPHA     0.25f
#define SPEED_CONTROL_KP                0.10f
#define SPEED_CONTROL_KI                0.50f
#define SPEED_CONTROL_INTEGRAL_LIMIT   25.0f

typedef struct
{
    float forward_start_pwm;
    float reverse_start_pwm;
    float forward_max_rpm;
    float reverse_max_rpm;
    float target_rpm;
    float filtered_rpm;
    float integral;
    bool filter_initialized;
} WheelSpeedController;

/* 12 V 架空测试标定值；落地后需要复测 PI 参数。 */
static WheelSpeedController left_controller = {
    .forward_start_pwm = 3.0f,
    .reverse_start_pwm = 3.0f,
    .forward_max_rpm = 304.7f,
    .reverse_max_rpm = 309.6f,
};

static WheelSpeedController right_controller = {
    .forward_start_pwm = 3.0f,
    .reverse_start_pwm = 3.0f,
    .forward_max_rpm = 299.7f,
    .reverse_max_rpm = 304.4f,
};

static bool speed_control_enabled;

static float SpeedControl_Clip(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static int SpeedControl_Sign(float value)
{
    if (value > 0.0f) return 1;
    if (value < 0.0f) return -1;
    return 0;
}

static float SpeedControl_FeedForward(const WheelSpeedController *controller)
{
    float magnitude;
    float pwm;

    if (controller->target_rpm > 0.0f)
    {
        magnitude = controller->target_rpm;
        pwm = controller->forward_start_pwm +
            (100.0f - controller->forward_start_pwm) * magnitude /
            controller->forward_max_rpm;
        return pwm;
    }
    if (controller->target_rpm < 0.0f)
    {
        magnitude = -controller->target_rpm;
        pwm = controller->reverse_start_pwm +
            (100.0f - controller->reverse_start_pwm) * magnitude /
            controller->reverse_max_rpm;
        return -pwm;
    }
    return 0.0f;
}

static float SpeedControl_UpdateWheel(WheelSpeedController *controller,
                                      float sample_period_seconds)
{
    float error;
    float feed_forward;
    float candidate_integral;
    float unsaturated_output;
    float output;

    if (controller->target_rpm == 0.0f)
    {
        controller->integral = 0.0f;
        return 0.0f;
    }

    error = controller->target_rpm - controller->filtered_rpm;
    feed_forward = SpeedControl_FeedForward(controller);
    candidate_integral = controller->integral +
        SPEED_CONTROL_KI * error * sample_period_seconds;
    candidate_integral = SpeedControl_Clip(
        candidate_integral,
        -SPEED_CONTROL_INTEGRAL_LIMIT,
        SPEED_CONTROL_INTEGRAL_LIMIT);

    unsaturated_output = feed_forward +
        SPEED_CONTROL_KP * error + candidate_integral;
    output = SpeedControl_Clip(unsaturated_output, -100.0f, 100.0f);

    if ((unsaturated_output == output) ||
        ((unsaturated_output > 100.0f) && (error < 0.0f)) ||
        ((unsaturated_output < -100.0f) && (error > 0.0f)))
    {
        controller->integral = candidate_integral;
    }
    return output;
}

static int16_t SpeedControl_RoundToPercent(float value)
{
    value = SpeedControl_Clip(value, -100.0f, 100.0f);
    return (value >= 0.0f)
        ? (int16_t)(value + 0.5f)
        : (int16_t)(value - 0.5f);
}

void SpeedControl_Init(void)
{
    left_controller.target_rpm = 0.0f;
    right_controller.target_rpm = 0.0f;
    left_controller.filtered_rpm = 0.0f;
    right_controller.filtered_rpm = 0.0f;
    left_controller.integral = 0.0f;
    right_controller.integral = 0.0f;
    left_controller.filter_initialized = false;
    right_controller.filter_initialized = false;
    speed_control_enabled = false;
    Motor_Stop();
}

void SpeedControl_SetTarget(float left_rpm, float right_rpm)
{
    float clipped_left = SpeedControl_Clip(
        left_rpm, -SPEED_CONTROL_MAX_TARGET_RPM,
        SPEED_CONTROL_MAX_TARGET_RPM);
    float clipped_right = SpeedControl_Clip(
        right_rpm, -SPEED_CONTROL_MAX_TARGET_RPM,
        SPEED_CONTROL_MAX_TARGET_RPM);

    if ((SpeedControl_Sign(clipped_left) !=
         SpeedControl_Sign(left_controller.target_rpm)) ||
        (clipped_left == 0.0f))
    {
        left_controller.integral = 0.0f;
    }
    if ((SpeedControl_Sign(clipped_right) !=
         SpeedControl_Sign(right_controller.target_rpm)) ||
        (clipped_right == 0.0f))
    {
        right_controller.integral = 0.0f;
    }

    left_controller.target_rpm = clipped_left;
    right_controller.target_rpm = clipped_right;
    if ((clipped_left == 0.0f) && (clipped_right == 0.0f))
    {
        SpeedControl_Stop();
        return;
    }
    speed_control_enabled = true;
}

void SpeedControl_Update(float left_rpm, float right_rpm,
                         float sample_period_seconds)
{
    float left_output;
    float right_output;

    if (!left_controller.filter_initialized)
    {
        left_controller.filtered_rpm = left_rpm;
        left_controller.filter_initialized = true;
    }
    else
    {
        left_controller.filtered_rpm += SPEED_CONTROL_FILTER_ALPHA *
            (left_rpm - left_controller.filtered_rpm);
    }

    if (!right_controller.filter_initialized)
    {
        right_controller.filtered_rpm = right_rpm;
        right_controller.filter_initialized = true;
    }
    else
    {
        right_controller.filtered_rpm += SPEED_CONTROL_FILTER_ALPHA *
            (right_rpm - right_controller.filtered_rpm);
    }

    if (!speed_control_enabled || !Motor_IsEnabled())
    {
        return;
    }

    sample_period_seconds = SpeedControl_Clip(
        sample_period_seconds, 0.001f, 0.100f);
    left_output = SpeedControl_UpdateWheel(
        &left_controller, sample_period_seconds);
    right_output = SpeedControl_UpdateWheel(
        &right_controller, sample_period_seconds);
    Motor_Set(
        SpeedControl_RoundToPercent(left_output),
        SpeedControl_RoundToPercent(right_output));
}

void SpeedControl_Stop(void)
{
    speed_control_enabled = false;
    left_controller.target_rpm = 0.0f;
    right_controller.target_rpm = 0.0f;
    left_controller.integral = 0.0f;
    right_controller.integral = 0.0f;
    Motor_Stop();
}

bool SpeedControl_IsEnabled(void) { return speed_control_enabled; }
float SpeedControl_GetLeftTargetRpm(void) { return left_controller.target_rpm; }
float SpeedControl_GetRightTargetRpm(void) { return right_controller.target_rpm; }
float SpeedControl_GetLeftMeasuredRpm(void) { return left_controller.filtered_rpm; }
float SpeedControl_GetRightMeasuredRpm(void) { return right_controller.filtered_rpm; }
