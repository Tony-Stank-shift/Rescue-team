#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct
{
    float x_mm;
    float y_mm;
    float theta_rad;
    float linear_mm_s;
    float angular_rad_s;
} OdometryState;

void Odometry_Init(void);
void Odometry_Reset(void);
void Odometry_Update(int16_t left_delta, int16_t right_delta,
                     float sample_period_seconds);
void Odometry_GetState(OdometryState *state);

#endif /* ODOMETRY_H */
