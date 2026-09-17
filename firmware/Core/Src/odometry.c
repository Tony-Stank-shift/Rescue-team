#include "odometry.h"

#include "chassis_config.h"

#include <stddef.h>

#define ODOMETRY_PI 3.14159265358979323846f
#define ODOMETRY_TWO_PI (2.0f * ODOMETRY_PI)
#define ODOMETRY_MM_PER_COUNT \
    (ODOMETRY_PI * CHASSIS_WHEEL_DIAMETER_MM / \
     CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV)

static OdometryState odometry_state;
static float heading_cos;
static float heading_sin;

/*
 * 编码器单次增量对应的转角小于 1 rad，用 7/6 阶展开可避免在 C6
 * 中链接体积较大的软件浮点三角函数。正常 10 ms 周期下误差远小于
 * 轮胎打滑和机械尺寸误差。
 */
static void Odometry_SinCosIncrement(float angle_rad,
                                     float *sine, float *cosine)
{
    float angle_squared = angle_rad * angle_rad;
    float angle_fourth = angle_squared * angle_squared;
    float angle_sixth = angle_fourth * angle_squared;

    *sine = angle_rad *
        (1.0f - angle_squared / 6.0f + angle_fourth / 120.0f -
         angle_sixth / 5040.0f);
    *cosine =
        1.0f - angle_squared / 2.0f + angle_fourth / 24.0f -
        angle_sixth / 720.0f;
}

static float Odometry_NormalizeAngle(float angle_rad)
{
    while (angle_rad > ODOMETRY_PI)
    {
        angle_rad -= ODOMETRY_TWO_PI;
    }
    while (angle_rad < -ODOMETRY_PI)
    {
        angle_rad += ODOMETRY_TWO_PI;
    }
    return angle_rad;
}

void Odometry_Init(void)
{
    Odometry_Reset();
}

void Odometry_Reset(void)
{
    odometry_state.x_mm = 0.0f;
    odometry_state.y_mm = 0.0f;
    odometry_state.theta_rad = 0.0f;
    odometry_state.linear_mm_s = 0.0f;
    odometry_state.angular_rad_s = 0.0f;
    heading_cos = 1.0f;
    heading_sin = 0.0f;
}

void Odometry_Update(int16_t left_delta, int16_t right_delta,
                     float sample_period_seconds)
{
    float left_distance_mm;
    float right_distance_mm;
    float center_distance_mm;
    float delta_theta_rad;
    float half_sine;
    float half_cosine;
    float delta_sine;
    float delta_cosine;
    float middle_cosine;
    float middle_sine;
    float new_heading_cos;
    float new_heading_sin;
    float heading_norm_correction;

    if (sample_period_seconds <= 0.0f)
    {
        return;
    }

    left_distance_mm = (float)left_delta * ODOMETRY_MM_PER_COUNT;
    right_distance_mm = (float)right_delta * ODOMETRY_MM_PER_COUNT;
    center_distance_mm = (left_distance_mm + right_distance_mm) * 0.5f;
    delta_theta_rad =
        (right_distance_mm - left_distance_mm) / CHASSIS_TRACK_WIDTH_MM;

    /* 用本周期中点航向积分，直行和弧线运动都适用。 */
    Odometry_SinCosIncrement(
        delta_theta_rad * 0.5f, &half_sine, &half_cosine);
    middle_cosine = heading_cos * half_cosine - heading_sin * half_sine;
    middle_sine = heading_sin * half_cosine + heading_cos * half_sine;
    odometry_state.x_mm += center_distance_mm * middle_cosine;
    odometry_state.y_mm += center_distance_mm * middle_sine;

    Odometry_SinCosIncrement(
        delta_theta_rad, &delta_sine, &delta_cosine);
    new_heading_cos = heading_cos * delta_cosine - heading_sin * delta_sine;
    new_heading_sin = heading_sin * delta_cosine + heading_cos * delta_sine;
    /* 一步牛顿修正，使累计的航向单位向量始终保持长度约为 1。 */
    heading_norm_correction = 0.5f *
        (3.0f - new_heading_cos * new_heading_cos -
         new_heading_sin * new_heading_sin);
    heading_cos = new_heading_cos * heading_norm_correction;
    heading_sin = new_heading_sin * heading_norm_correction;
    odometry_state.theta_rad = Odometry_NormalizeAngle(
        odometry_state.theta_rad + delta_theta_rad);
    odometry_state.linear_mm_s = center_distance_mm / sample_period_seconds;
    odometry_state.angular_rad_s = delta_theta_rad / sample_period_seconds;
}

void Odometry_GetState(OdometryState *state)
{
    if (state != NULL)
    {
        *state = odometry_state;
    }
}
