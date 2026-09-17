#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t tick_ms;
    uint32_t sequence;
} MPU6050_Sample;

/* 初始化地址为 0x68 的 MPU6050，并开启 100 Hz DATA_RDY 中断。 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *i2c);

/* 在主循环调用；仅在 PB5 的 DATA_RDY 中断到来后执行一次 I2C 读取。 */
HAL_StatusTypeDef MPU6050_Update(void);

/* 由 HAL_GPIO_EXTI_Callback 调用；该函数不执行 I2C 操作。 */
void MPU6050_NotifyDataReadyFromIsr(void);

bool MPU6050_IsReady(void);
bool MPU6050_GetLatestSample(MPU6050_Sample *sample);
HAL_StatusTypeDef MPU6050_GetLastStatus(void);

#endif /* MPU6050_H */
