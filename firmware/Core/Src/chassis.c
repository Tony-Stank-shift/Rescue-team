#include "chassis.h"

#include "chassis_config.h"
#include "speed_control.h"

#define CHASSIS_PI                 3.14159265358979323846f
#define CHASSIS_MM_S_PER_RPM \
    (CHASSIS_PI * CHASSIS_WHEEL_DIAMETER_MM / 60.0f)

static float Chassis_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void Chassis_SetVelocity(float linear_mm_s, float angular_rad_s)
{
    float half_track_mm = CHASSIS_TRACK_WIDTH_MM * 0.5f;
    float left_rpm =
        (linear_mm_s - angular_rad_s * half_track_mm) /
        CHASSIS_MM_S_PER_RPM;
    float right_rpm =
        (linear_mm_s + angular_rad_s * half_track_mm) /
        CHASSIS_MM_S_PER_RPM;
    float largest_rpm = Chassis_Abs(left_rpm);

    if (Chassis_Abs(right_rpm) > largest_rpm)
    {
        largest_rpm = Chassis_Abs(right_rpm);
    }
    if (largest_rpm > SPEED_CONTROL_MAX_TARGET_RPM)
    {
        float scale = SPEED_CONTROL_MAX_TARGET_RPM / largest_rpm;
        left_rpm *= scale;
        right_rpm *= scale;
    }
    SpeedControl_SetTarget(left_rpm, right_rpm);
}

void Chassis_Stop(void)
{
    SpeedControl_Stop();
}
