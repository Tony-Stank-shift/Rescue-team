#include "mpu6050.h"

#include <stddef.h>

#define MPU6050_I2C_ADDRESS       (0x68U << 1)
#define MPU6050_I2C_TIMEOUT_MS    20U

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_INT_PIN_CFG   0x37U
#define MPU6050_REG_INT_ENABLE    0x38U
#define MPU6050_REG_INT_STATUS    0x3AU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_PWR_MGMT_2    0x6CU
#define MPU6050_REG_WHO_AM_I      0x75U

static I2C_HandleTypeDef *mpu_i2c;
static volatile bool data_ready_pending;
static bool initialized;
static bool sample_valid;
static MPU6050_Sample latest_sample;
static HAL_StatusTypeDef last_status = HAL_ERROR;

static int16_t decode_i16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static HAL_StatusTypeDef write_register(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(mpu_i2c,
                             MPU6050_I2C_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_registers(uint8_t reg, uint8_t *data, uint16_t size)
{
    return HAL_I2C_Mem_Read(mpu_i2c,
                            MPU6050_I2C_ADDRESS,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            size,
                            MPU6050_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *i2c)
{
    uint8_t who_am_i = 0U;
    uint8_t int_status = 0U;

    if (i2c == NULL || i2c->Instance != I2C2)
    {
        return HAL_ERROR;
    }

    mpu_i2c = i2c;
    initialized = false;
    sample_valid = false;
    data_ready_pending = false;

    last_status = HAL_I2C_IsDeviceReady(mpu_i2c,
                                        MPU6050_I2C_ADDRESS,
                                        3U,
                                        MPU6050_I2C_TIMEOUT_MS);
    if (last_status != HAL_OK)
    {
        return last_status;
    }

    last_status = read_registers(MPU6050_REG_WHO_AM_I, &who_am_i, 1U);
    if (last_status != HAL_OK || who_am_i != 0x68U)
    {
        last_status = HAL_ERROR;
        return last_status;
    }

    last_status = write_register(MPU6050_REG_PWR_MGMT_1, 0x80U);
    if (last_status != HAL_OK)
    {
        return last_status;
    }
    HAL_Delay(100U);

    if ((last_status = write_register(MPU6050_REG_PWR_MGMT_1, 0x01U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_PWR_MGMT_2, 0x00U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_SMPLRT_DIV, 9U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_CONFIG, 0x03U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_GYRO_CONFIG, 0x00U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_ACCEL_CONFIG, 0x00U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_INT_ENABLE, 0x00U)) != HAL_OK ||
        (last_status = write_register(MPU6050_REG_INT_PIN_CFG, 0x00U)) != HAL_OK)
    {
        return last_status;
    }

    /* 读取一次状态以清除上电期间可能遗留的 DATA_RDY 标志。 */
    last_status = read_registers(MPU6050_REG_INT_STATUS, &int_status, 1U);
    if (last_status != HAL_OK)
    {
        return last_status;
    }

    last_status = write_register(MPU6050_REG_INT_ENABLE, 0x01U);
    if (last_status != HAL_OK)
    {
        return last_status;
    }

    initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Update(void)
{
    uint8_t raw[14];
    MPU6050_Sample sample;
    uint32_t primask;

    if (!initialized)
    {
        return HAL_ERROR;
    }
    if (!data_ready_pending)
    {
        return HAL_OK;
    }

    data_ready_pending = false;
    last_status = read_registers(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (last_status != HAL_OK)
    {
        return last_status;
    }

    sample.accel_x = decode_i16(&raw[0]);
    sample.accel_y = decode_i16(&raw[2]);
    sample.accel_z = decode_i16(&raw[4]);
    sample.temperature = decode_i16(&raw[6]);
    sample.gyro_x = decode_i16(&raw[8]);
    sample.gyro_y = decode_i16(&raw[10]);
    sample.gyro_z = decode_i16(&raw[12]);
    sample.tick_ms = HAL_GetTick();
    sample.sequence = latest_sample.sequence + 1U;

    primask = __get_PRIMASK();
    __disable_irq();
    latest_sample = sample;
    sample_valid = true;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return HAL_OK;
}

void MPU6050_NotifyDataReadyFromIsr(void)
{
    data_ready_pending = true;
}

bool MPU6050_IsReady(void)
{
    return initialized;
}

bool MPU6050_GetLatestSample(MPU6050_Sample *sample)
{
    bool valid;
    uint32_t primask;

    if (sample == NULL)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    valid = sample_valid;
    if (valid)
    {
        *sample = latest_sample;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return valid;
}

HAL_StatusTypeDef MPU6050_GetLastStatus(void)
{
    return last_status;
}
