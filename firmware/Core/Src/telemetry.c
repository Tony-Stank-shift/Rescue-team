#include "telemetry.h"

#include "chassis_config.h"
#include "encoder.h"
#include "mpu6050.h"
#include "odometry.h"

#include <stdint.h>
#include <stdio.h>

#define TELEMETRY_ODOM_PERIOD_MS   50U
#define TELEMETRY_IMU_PERIOD_MS    20U
#define TELEMETRY_DEBUG_PERIOD_MS 100U
#define TELEMETRY_UART_TIMEOUT_MS 20U
#define TELEMETRY_LINE_SIZE       160U
#define TELEMETRY_PI              3.14159265358979323846f
#define TELEMETRY_MM_S_PER_RPM \
    (TELEMETRY_PI * CHASSIS_WHEEL_DIAMETER_MM / 60.0f)
#define TELEMETRY_ACCEL_LSB_PER_G  16384.0f
#define TELEMETRY_GYRO_LSB_PER_DPS 131.0f

static UART_HandleTypeDef *telemetry_uart;
static uint32_t telemetry_last_odom_tick;
static uint32_t telemetry_last_imu_tick;
static uint32_t telemetry_last_debug_tick;

static int32_t Telemetry_RoundToInt32(float value)
{
    return (value >= 0.0f)
        ? (int32_t)(value + 0.5f)
        : (int32_t)(value - 0.5f);
}

static void Telemetry_Send(const char *line, int length)
{
    if ((length > 0) && ((size_t)length < TELEMETRY_LINE_SIZE))
    {
        (void)HAL_UART_Transmit(
            telemetry_uart,
            (uint8_t *)line,
            (uint16_t)length,
            TELEMETRY_UART_TIMEOUT_MS);
    }
}

static int32_t Telemetry_RpmToMilli(float rpm)
{
    return (rpm >= 0.0f)
        ? (int32_t)(rpm * 1000.0f + 0.5f)
        : (int32_t)(rpm * 1000.0f - 0.5f);
}

static int32_t Telemetry_AccelToMilliG(int16_t raw)
{
    return Telemetry_RoundToInt32(
        (float)raw * (1000.0f / TELEMETRY_ACCEL_LSB_PER_G));
}

static int32_t Telemetry_GyroToMilliRad(int16_t raw)
{
    return Telemetry_RoundToInt32(
        (float)raw * (TELEMETRY_PI * 1000.0f) /
        (TELEMETRY_GYRO_LSB_PER_DPS * 180.0f));
}

static int32_t Telemetry_TempToCentiC(int16_t raw)
{
    /* MPU6050: T[°C] = 36.53 + raw / 340 */
    return 3653 + Telemetry_RoundToInt32((float)raw * (100.0f / 340.0f));
}

static uint32_t Telemetry_AbsInt32(int32_t value)
{
    return (value < 0)
        ? (uint32_t)(-(int64_t)value)
        : (uint32_t)value;
}

static const char *Telemetry_Sign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

void Telemetry_Init(UART_HandleTypeDef *uart)
{
    telemetry_uart = uart;
    telemetry_last_odom_tick = HAL_GetTick();
    telemetry_last_imu_tick = telemetry_last_odom_tick;
    telemetry_last_debug_tick = telemetry_last_odom_tick;
}

void Telemetry_Update(int16_t left_target_rpm, float left_rpm,
                      int16_t right_target_rpm, float right_rpm,
                      int16_t left_command, int16_t right_command)
{
    uint32_t now;
    int32_t left_milli;
    int32_t right_milli;
    uint32_t left_abs;
    uint32_t right_abs;
    int32_t x_micro_m;
    int32_t y_micro_m;
    int32_t theta_micro_rad;
    int32_t left_micro_m_s;
    int32_t right_micro_m_s;
    OdometryState odometry;
    char line[TELEMETRY_LINE_SIZE];
    int length;

    if (telemetry_uart == NULL)
    {
        return;
    }
    now = HAL_GetTick();

    if ((now - telemetry_last_odom_tick) >= TELEMETRY_ODOM_PERIOD_MS)
    {
        telemetry_last_odom_tick = now;
        Odometry_GetState(&odometry);

        /*
         * 手册规定 ODOM 对外使用 m、rad、m/s，并保留六位小数。
         * 这里手动拆分定点数，避免 C6 因 printf 浮点支持显著增加 Flash。
         */
        x_micro_m = Telemetry_RoundToInt32(odometry.x_mm * 1000.0f);
        y_micro_m = Telemetry_RoundToInt32(odometry.y_mm * 1000.0f);
        theta_micro_rad =
            Telemetry_RoundToInt32(odometry.theta_rad * 1000000.0f);
        left_micro_m_s = Telemetry_RoundToInt32(
            left_rpm * TELEMETRY_MM_S_PER_RPM * 1000.0f);
        right_micro_m_s = Telemetry_RoundToInt32(
            right_rpm * TELEMETRY_MM_S_PER_RPM * 1000.0f);

        length = snprintf(
            line,
            sizeof(line),
            "ODOM,%s%lu.%06lu,%s%lu.%06lu,%s%lu.%06lu,%ld,%ld,"
            "%s%lu.%06lu,%s%lu.%06lu\r\n",
            Telemetry_Sign(x_micro_m),
            (unsigned long)(Telemetry_AbsInt32(x_micro_m) / 1000000U),
            (unsigned long)(Telemetry_AbsInt32(x_micro_m) % 1000000U),
            Telemetry_Sign(y_micro_m),
            (unsigned long)(Telemetry_AbsInt32(y_micro_m) / 1000000U),
            (unsigned long)(Telemetry_AbsInt32(y_micro_m) % 1000000U),
            Telemetry_Sign(theta_micro_rad),
            (unsigned long)(Telemetry_AbsInt32(theta_micro_rad) / 1000000U),
            (unsigned long)(Telemetry_AbsInt32(theta_micro_rad) % 1000000U),
            (long)Encoder_GetLeftTotal(),
            (long)Encoder_GetRightTotal(),
            Telemetry_Sign(left_micro_m_s),
            (unsigned long)(Telemetry_AbsInt32(left_micro_m_s) / 1000000U),
            (unsigned long)(Telemetry_AbsInt32(left_micro_m_s) % 1000000U),
            Telemetry_Sign(right_micro_m_s),
            (unsigned long)(Telemetry_AbsInt32(right_micro_m_s) / 1000000U),
            (unsigned long)(Telemetry_AbsInt32(right_micro_m_s) % 1000000U));
        Telemetry_Send(line, length);
    }

    if ((now - telemetry_last_imu_tick) >= TELEMETRY_IMU_PERIOD_MS)
    {
        MPU6050_Sample sample;

        telemetry_last_imu_tick = now;
        if (MPU6050_GetLatestSample(&sample))
        {
            length = snprintf(
                line,
                sizeof(line),
                "IMU,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                (unsigned long)sample.tick_ms,
                (unsigned long)sample.sequence,
                (long)Telemetry_AccelToMilliG(sample.accel_x),
                (long)Telemetry_AccelToMilliG(sample.accel_y),
                (long)Telemetry_AccelToMilliG(sample.accel_z),
                (long)Telemetry_GyroToMilliRad(sample.gyro_x),
                (long)Telemetry_GyroToMilliRad(sample.gyro_y),
                (long)Telemetry_GyroToMilliRad(sample.gyro_z),
                (long)Telemetry_TempToCentiC(sample.temperature));
            Telemetry_Send(line, length);
        }
    }

    if ((now - telemetry_last_debug_tick) < TELEMETRY_DEBUG_PERIOD_MS)
    {
        return;
    }
    telemetry_last_debug_tick = now;

    /* 手动输出三位小数，避免链接 printf 浮点格式化。 */
    left_milli = Telemetry_RpmToMilli(left_rpm);
    right_milli = Telemetry_RpmToMilli(right_rpm);
    left_abs = (left_milli < 0)
        ? (uint32_t)(-(int64_t)left_milli)
        : (uint32_t)left_milli;
    right_abs = (right_milli < 0)
        ? (uint32_t)(-(int64_t)right_milli)
        : (uint32_t)right_milli;

    length = snprintf(
        line,
        sizeof(line),
        "TEL,%lu,%d,%s%lu.%03lu,%d,%s%lu.%03lu,%d,%d\r\n",
        (unsigned long)now,
        (int)left_target_rpm,
        (left_milli < 0) ? "-" : "",
        (unsigned long)(left_abs / 1000U),
        (unsigned long)(left_abs % 1000U),
        (int)right_target_rpm,
        (right_milli < 0) ? "-" : "",
        (unsigned long)(right_abs / 1000U),
        (unsigned long)(right_abs % 1000U),
        (int)left_command,
        (int)right_command);

    Telemetry_Send(line, length);
}
