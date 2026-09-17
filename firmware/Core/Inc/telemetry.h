#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stm32f1xx_hal.h"

#include <stdint.h>

void Telemetry_Init(UART_HandleTypeDef *uart);

/*
 * TEL,tick_ms,left_target,left_rpm,right_target,right_rpm,left_pwm,right_pwm
 * ODOM,x_m,y_m,theta_rad,encL,encR,vL_m_s,vR_m_s
 * IMU,tick_ms,seq,ax_mg,ay_mg,az_mg,gx_mrad_s,gy_mrad_s,gz_mrad_s,temp_cC
 */
void Telemetry_Update(int16_t left_target_rpm, float left_rpm,
                      int16_t right_target_rpm, float right_rpm,
                      int16_t left_command, int16_t right_command);

#endif /* TELEMETRY_H */
