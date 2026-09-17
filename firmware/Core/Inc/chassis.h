#ifndef CHASSIS_H
#define CHASSIS_H

/* 线速度 mm/s，角速度 rad/s；正角速度表示向左转。 */
void Chassis_SetVelocity(float linear_mm_s, float angular_rad_s);
void Chassis_Stop(void);

#endif /* CHASSIS_H */
