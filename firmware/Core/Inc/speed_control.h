#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdbool.h>

#define SPEED_CONTROL_MAX_TARGET_RPM 250.0f

void SpeedControl_Init(void);
void SpeedControl_SetTarget(float left_rpm, float right_rpm);
void SpeedControl_Update(float left_rpm, float right_rpm,
                         float sample_period_seconds);
void SpeedControl_Stop(void);

bool SpeedControl_IsEnabled(void);
float SpeedControl_GetLeftTargetRpm(void);
float SpeedControl_GetRightTargetRpm(void);
float SpeedControl_GetLeftMeasuredRpm(void);
float SpeedControl_GetRightMeasuredRpm(void);

#endif /* SPEED_CONTROL_H */
