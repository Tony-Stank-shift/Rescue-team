#include "command.h"

#include "chassis.h"
#include "encoder.h"
#include "main.h"
#include "motor.h"
#include "odometry.h"
#include "servo.h"
#include "speed_control.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define COMMAND_RX_RING_SIZE      128U

/* RX 自愈看门狗周期。取 500ms：故障时接收最多静默半个周期，
   而上位机探测超时 1.5~2s、且带重试，因此足够覆盖。
   不宜再短：每次强制重新武装都有极小概率丢掉恰好在途的 1 个字节。 */
#define RX_HEALTH_PERIOD_MS       500U
#define COMMAND_LINE_SIZE          64U
#define COMMAND_MAX_FIELDS          5U
#define COMMAND_TX_LINE_SIZE       64U
#define COMMAND_TX_TIMEOUT_MS      20U
#define COMMAND_TEST_MAX_MS      5000U
#define COMMAND_BUTTON_DEBOUNCE_MS 30U
#define COMMAND_WATCHDOG_HOLD_MS  300U
#define COMMAND_WATCHDOG_STOP_MS  800U
#define COMMAND_WATCHDOG_RECOVERY_GAP_MS 100U
#define COMMAND_WATCHDOG_RECOVERY_FRAMES 3U

typedef enum
{
    COMMAND_STATE_WAIT_START = 0,
    COMMAND_STATE_RUNNING,
    COMMAND_STATE_ESTOPPED,
} CommandState;

static UART_HandleTypeDef *command_uart;
static uint8_t rx_ring[COMMAND_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_overflow;
static volatile bool rx_restart_pending;
static uint8_t rx_byte;
/*
 * RX 健康看门狗（2026-09-16 现场新增）。
 *
 * 现象：跑一段时间后**命令接收静默死亡** —— 遥测 TX 照发（主循环活着），
 *       但下位机对 PING/START/VEL/SERVO **一律不回**（连 ERR,FORMAT 都没有），
 *       上位机侧表现为"PING 收不到 PONG、车完全不受控"，只能靠复位恢复。
 *       实测：刷完固件复位后 hw_selftest 的 serial 是 PASS，跑约 10 分钟后复现。
 *
 * 成因：STM32 HAL 在 ORE(溢出)/帧错误后走 UART_EndRxTransfer → 关掉 RXNE/EIE
 *       并把 RxState 置 READY；但恢复路径依赖 HAL 内部状态能正确回到可武装态，
 *       一旦没回去，HAL_UART_Receive_IT 就永远返回 HAL_BUSY →
 *       接收永不再武装，而 TX 完全不受影响，所以故障极难察觉。
 *
 * 兜底：每秒检查一次"接收是否仍处于 BUSY_RX"（= 正在等字节）。
 *       只要不是 BUSY_RX，就说明接收没在武装 → 强制复位 HAL 接收状态并重新武装。
 *       这样无论 HAL 把状态留在哪，都能在 1 秒内自愈，不必再靠人工复位。
 */
static uint32_t rx_health_tick;

static char command_line[COMMAND_LINE_SIZE];
static uint16_t command_line_length;
static bool command_line_too_long;
static CommandState command_state;
static bool timed_test_active;
static uint32_t timed_test_start_tick;
static uint32_t timed_test_duration_ms;

static bool velocity_mode_active;
static bool watchdog_stopped;
static uint8_t watchdog_recovery_frames;
static uint32_t last_valid_velocity_tick;
static float last_linear_mm_s;
static float last_angular_rad_s;

static bool button_raw_pressed;
static bool button_stable_pressed;
static uint32_t button_change_tick;

static void Command_SetStatusLed(bool on)
{
    /* 本板 PC13 LED 高电平点亮。 */
    HAL_GPIO_WritePin(
        STATUS_LED_GPIO_Port,
        STATUS_LED_Pin,
        on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Command_Send(const char *text)
{
    size_t length;

    if ((command_uart == NULL) || (text == NULL))
    {
        return;
    }
    length = strlen(text);
    if ((length == 0U) || (length > UINT16_MAX))
    {
        return;
    }
    (void)HAL_UART_Transmit(
        command_uart,
        (uint8_t *)text,
        (uint16_t)length,
        COMMAND_TX_TIMEOUT_MS);
}

static void Command_SendMotorAck(const char *name, int16_t left,
                                 int16_t right)
{
    char response[COMMAND_TX_LINE_SIZE];
    int length = snprintf(
        response,
        sizeof(response),
        "ACK,%s,%d,%d\r\n",
        name,
        (int)left,
        (int)right);

    if ((length > 0) && ((size_t)length < sizeof(response)))
    {
        Command_Send(response);
    }
}

static int16_t Command_ClipPercent(long value)
{
    if (value > 100L) return 100;
    if (value < -100L) return -100;
    return (int16_t)value;
}

static int16_t Command_ClipRpm(long value)
{
    if (value > (long)SPEED_CONTROL_MAX_TARGET_RPM)
        return (int16_t)SPEED_CONTROL_MAX_TARGET_RPM;
    if (value < -(long)SPEED_CONTROL_MAX_TARGET_RPM)
        return (int16_t)(-SPEED_CONTROL_MAX_TARGET_RPM);
    return (int16_t)value;
}

static bool Command_ParseLong(const char *text, long *value)
{
    char *end;
    long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return false;
    }
    errno = 0;
    parsed = strtol(text, &end, 10);
    if ((end == text) || (*end != '\0'))
    {
        return false;
    }
    if (errno == ERANGE)
    {
        parsed = (*text == '-') ? LONG_MIN : LONG_MAX;
    }
    *value = parsed;
    return true;
}

static char *Command_Trim(char *text)
{
    char *end;

    while ((*text == ' ') || (*text == '\t')) text++;
    end = text + strlen(text);
    while ((end > text) && ((end[-1] == ' ') || (end[-1] == '\t'))) end--;
    *end = '\0';
    return text;
}

static size_t Command_Split(char *line, char **fields, size_t capacity)
{
    size_t count = 0U;
    char *cursor;

    fields[count++] = line;
    for (cursor = line; *cursor != '\0'; cursor++)
    {
        if (*cursor != ',') continue;
        *cursor = '\0';
        if (count >= capacity) return capacity + 1U;
        fields[count++] = cursor + 1;
    }
    for (size_t i = 0U; i < count; i++) fields[i] = Command_Trim(fields[i]);
    return count;
}

static void Command_ResetVelocityWatchdog(void)
{
    velocity_mode_active = false;
    watchdog_stopped = false;
    watchdog_recovery_frames = 0U;
    last_valid_velocity_tick = HAL_GetTick();
    last_linear_mm_s = 0.0f;
    last_angular_rad_s = 0.0f;
}

static void Command_EnterRunning(bool from_button)
{
    if (command_state == COMMAND_STATE_ESTOPPED)
    {
        return;
    }
    if (command_state != COMMAND_STATE_RUNNING)
    {
        timed_test_active = false;
        Command_ResetVelocityWatchdog();
        Chassis_Stop();
        Encoder_Reset();
        Odometry_Reset();
        Motor_Enable();
        command_state = COMMAND_STATE_RUNNING;
        Command_SetStatusLed(true);
        if (from_button)
        {
            Command_Send("EVENT,START_BUTTON\r\n");
        }
    }
}

static bool Command_CanMove(void)
{
    if (command_state == COMMAND_STATE_ESTOPPED)
    {
        Command_Send("ERR,LOCKED\r\n");
        return false;
    }
    if ((command_state != COMMAND_STATE_RUNNING) || !Motor_IsEnabled())
    {
        Command_Send("ERR,NOT_STARTED\r\n");
        return false;
    }
    return true;
}

static void Command_HandlePwm(char **fields, size_t count, bool timed)
{
    long left_value;
    long right_value;
    long duration_value = 0L;
    int16_t left_percent;
    int16_t right_percent;
    char response[COMMAND_TX_LINE_SIZE];
    int length;

    if (count != (timed ? 4U : 3U))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }
    if (!Command_ParseLong(fields[1], &left_value) ||
        !Command_ParseLong(fields[2], &right_value))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }
    if (timed &&
        (!Command_ParseLong(fields[3], &duration_value) ||
         (duration_value <= 0L) ||
         (duration_value > (long)COMMAND_TEST_MAX_MS)))
    {
        Command_Send("ERR,DURATION\r\n");
        return;
    }
    if (!Command_CanMove()) return;

    Command_ResetVelocityWatchdog();
    left_percent = Command_ClipPercent(left_value);
    right_percent = Command_ClipPercent(right_value);
    SpeedControl_Stop();
    Motor_Set(left_percent, right_percent);

    if (!timed)
    {
        timed_test_active = false;
        Command_SendMotorAck("PWM", left_percent, right_percent);
        return;
    }

    timed_test_start_tick = HAL_GetTick();
    timed_test_duration_ms = (uint32_t)duration_value;
    timed_test_active = true;
    length = snprintf(
        response,
        sizeof(response),
        "ACK,TESTPWM,%d,%d,%lu\r\n",
        (int)left_percent,
        (int)right_percent,
        (unsigned long)timed_test_duration_ms);
    if ((length > 0) && ((size_t)length < sizeof(response)))
    {
        Command_Send(response);
    }
}

static void Command_HandleRpm(char **fields, size_t count, bool timed)
{
    long left_value;
    long right_value;
    long duration_value = 0L;
    int16_t left_rpm;
    int16_t right_rpm;
    char response[COMMAND_TX_LINE_SIZE];
    int length;

    if (count != (timed ? 4U : 3U))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }
    if (!Command_ParseLong(fields[1], &left_value) ||
        !Command_ParseLong(fields[2], &right_value))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }
    if (timed &&
        (!Command_ParseLong(fields[3], &duration_value) ||
         (duration_value <= 0L) ||
         (duration_value > (long)COMMAND_TEST_MAX_MS)))
    {
        Command_Send("ERR,DURATION\r\n");
        return;
    }
    if (!Command_CanMove()) return;

    Command_ResetVelocityWatchdog();
    left_rpm = Command_ClipRpm(left_value);
    right_rpm = Command_ClipRpm(right_value);
    SpeedControl_SetTarget((float)left_rpm, (float)right_rpm);
    if (!timed)
    {
        timed_test_active = false;
        Command_SendMotorAck("RPM", left_rpm, right_rpm);
        return;
    }

    timed_test_start_tick = HAL_GetTick();
    timed_test_duration_ms = (uint32_t)duration_value;
    timed_test_active = true;
    length = snprintf(
        response,
        sizeof(response),
        "ACK,TESTRPM,%d,%d,%lu\r\n",
        (int)left_rpm,
        (int)right_rpm,
        (unsigned long)timed_test_duration_ms);
    if ((length > 0) && ((size_t)length < sizeof(response)))
    {
        Command_Send(response);
    }
}

static void Command_HandleVelocity(char **fields, size_t count)
{
    long linear_mm_s;
    long angular_mrad_s;
    uint32_t now;
    float linear_command;
    float angular_command;

    if ((count != 3U) ||
        !Command_ParseLong(fields[1], &linear_mm_s) ||
        !Command_ParseLong(fields[2], &angular_mrad_s))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }
    if (!Command_CanMove()) return;

    timed_test_active = false;
    now = HAL_GetTick();
    linear_command = (float)linear_mm_s;
    angular_command = (float)angular_mrad_s / 1000.0f;

    if (velocity_mode_active && watchdog_stopped)
    {
        if ((watchdog_recovery_frames > 0U) &&
            ((now - last_valid_velocity_tick) <=
             COMMAND_WATCHDOG_RECOVERY_GAP_MS))
        {
            watchdog_recovery_frames++;
        }
        else
        {
            watchdog_recovery_frames = 1U;
        }

        last_valid_velocity_tick = now;
        last_linear_mm_s = linear_command;
        last_angular_rad_s = angular_command;
        if (watchdog_recovery_frames < COMMAND_WATCHDOG_RECOVERY_FRAMES)
        {
            return;
        }

        watchdog_stopped = false;
        watchdog_recovery_frames = 0U;
        Chassis_SetVelocity(last_linear_mm_s, last_angular_rad_s);
        return;
    }

    velocity_mode_active = true;
    watchdog_recovery_frames = 0U;
    last_valid_velocity_tick = now;
    last_linear_mm_s = linear_command;
    last_angular_rad_s = angular_command;
    Chassis_SetVelocity(linear_command, angular_command);
}

static void Command_UpdateVelocityWatchdog(void)
{
    uint32_t elapsed_ms;
    float scale;

    if (!velocity_mode_active || watchdog_stopped ||
        (command_state != COMMAND_STATE_RUNNING))
    {
        return;
    }

    elapsed_ms = HAL_GetTick() - last_valid_velocity_tick;
    if (elapsed_ms <= COMMAND_WATCHDOG_HOLD_MS)
    {
        return;
    }
    if (elapsed_ms < COMMAND_WATCHDOG_STOP_MS)
    {
        scale = (float)(COMMAND_WATCHDOG_STOP_MS - elapsed_ms) /
            (float)(COMMAND_WATCHDOG_STOP_MS - COMMAND_WATCHDOG_HOLD_MS);
        Chassis_SetVelocity(
            last_linear_mm_s * scale,
            last_angular_rad_s * scale);
        return;
    }

    Chassis_Stop();
    watchdog_stopped = true;
    watchdog_recovery_frames = 0U;
    Command_Send("EVENT,WATCHDOG_STOP\r\n");
}

static void Command_HandleServo(char **fields, size_t count)
{
    long angle_deg;
    char response[COMMAND_TX_LINE_SIZE];
    int length;

    if ((count != 2U) && (count != 3U))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }
    if (!Command_CanMove()) return;

    if ((count == 2U) && (strcmp(fields[1], "RAISE") == 0))
    {
        Servo_Raise();
        Command_Send("ACK,SERVO,RAISE\r\n");
        return;
    }
    if ((count == 2U) && (strcmp(fields[1], "LOWER") == 0))
    {
        Servo_Lower();
        Command_Send("ACK,SERVO,LOWER\r\n");
        return;
    }
    if ((count == 2U) && (strcmp(fields[1], "HOLD") == 0))
    {
        Servo_Hold();
        Command_Send("ACK,SERVO,HOLD\r\n");
        return;
    }
    if ((count == 3U) && (strcmp(fields[1], "ANGLE") == 0))
    {
        if (!Command_ParseLong(fields[2], &angle_deg) ||
            (angle_deg < 0L) ||
            (angle_deg > (long)SERVO_MAX_ANGLE_DEG))
        {
            Command_Send("ERR,SERVO_ANGLE\r\n");
            return;
        }

        Servo_SetAngleDeg((uint16_t)angle_deg);
        length = snprintf(
            response,
            sizeof(response),
            "ACK,SERVO,ANGLE,%ld\r\n",
            angle_deg);
        if ((length > 0) && ((size_t)length < sizeof(response)))
        {
            Command_Send(response);
        }
        return;
    }

    Command_Send("ERR,FORMAT\r\n");
}

static void Command_HandleLine(char *line)
{
    char *fields[COMMAND_MAX_FIELDS];
    size_t count = Command_Split(line, fields, COMMAND_MAX_FIELDS);

    if ((count == 0U) || (count > COMMAND_MAX_FIELDS) ||
        (fields[0][0] == '\0'))
    {
        Command_Send("ERR,FORMAT\r\n");
        return;
    }

    if (strcmp(fields[0], "PING") == 0)
    {
        Command_Send((count == 1U) ? "PONG\r\n" : "ERR,FORMAT\r\n");
        return;
    }
    if (strcmp(fields[0], "START") == 0)
    {
        if (count != 1U) Command_Send("ERR,FORMAT\r\n");
        else if (command_state == COMMAND_STATE_ESTOPPED)
            Command_Send("ERR,LOCKED\r\n");
        else
        {
            Command_EnterRunning(false);
            Command_Send("ACK,START\r\n");
        }
        return;
    }
    if (strcmp(fields[0], "VEL") == 0)
    {
        Command_HandleVelocity(fields, count);
        return;
    }
    if (strcmp(fields[0], "SERVO") == 0)
    {
        Command_HandleServo(fields, count);
        return;
    }
    if (strcmp(fields[0], "PWM") == 0)
    {
        Command_HandlePwm(fields, count, false);
        return;
    }
    if (strcmp(fields[0], "TESTPWM") == 0)
    {
        Command_HandlePwm(fields, count, true);
        return;
    }
    if (strcmp(fields[0], "RPM") == 0)
    {
        Command_HandleRpm(fields, count, false);
        return;
    }
    if (strcmp(fields[0], "TESTRPM") == 0)
    {
        Command_HandleRpm(fields, count, true);
        return;
    }
    if (strcmp(fields[0], "ODOM_RESET") == 0)
    {
        if (count != 1U) Command_Send("ERR,FORMAT\r\n");
        else
        {
            Encoder_Reset();
            Odometry_Reset();
            Command_Send("ACK,ODOM_RESET\r\n");
        }
        return;
    }
    if (strcmp(fields[0], "STOP") == 0)
    {
        if (count != 1U) Command_Send("ERR,FORMAT\r\n");
        else
        {
            timed_test_active = false;
            Command_ResetVelocityWatchdog();
            Chassis_Stop();
            Command_Send("ACK,STOP\r\n");
        }
        return;
    }
    if (strcmp(fields[0], "ESTOP") == 0)
    {
        if (count != 1U) Command_Send("ERR,FORMAT\r\n");
        else
        {
            timed_test_active = false;
            Command_ResetVelocityWatchdog();
            command_state = COMMAND_STATE_ESTOPPED;
            Chassis_Stop();
            Motor_Brake();
            Motor_Disable();
            Command_SetStatusLed(false);
            Command_Send("ACK,ESTOP\r\n");
        }
        return;
    }
    Command_Send("ERR,UNKNOWN\r\n");
}

static void Command_ProcessByte(uint8_t byte)
{
    if (byte == '\r') return;
    if (byte == '\n')
    {
        if (command_line_too_long)
        {
            command_line_too_long = false;
            command_line_length = 0U;
            Command_Send("ERR,TOO_LONG\r\n");
            return;
        }
        if (command_line_length == 0U) return;
        command_line[command_line_length] = '\0';
        Command_HandleLine(command_line);
        command_line_length = 0U;
        return;
    }
    if (command_line_too_long) return;
    if (command_line_length >= (COMMAND_LINE_SIZE - 1U))
    {
        command_line_too_long = true;
        return;
    }
    command_line[command_line_length++] = (char)byte;
}

static void Command_UpdateButton(void)
{
    uint32_t now = HAL_GetTick();
    bool pressed =
        (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET);

    if (pressed != button_raw_pressed)
    {
        button_raw_pressed = pressed;
        button_change_tick = now;
    }
    if ((button_raw_pressed != button_stable_pressed) &&
        ((now - button_change_tick) >= COMMAND_BUTTON_DEBOUNCE_MS))
    {
        button_stable_pressed = button_raw_pressed;
        if (button_stable_pressed)
        {
            if (command_state == COMMAND_STATE_RUNNING)
            {
                /* 自锁开关拨到 ON:仅点亮状态灯,不动电机状态。 */
                Command_SetStatusLed(true);
                Command_Send("EVENT,BUTTON_LED_ON\r\n");
            }
            else
            {
                Command_EnterRunning(true);
            }
        }
        else if (command_state == COMMAND_STATE_RUNNING)
        {
            /* 自锁开关拨回 OFF:仅熄灭状态灯,不动电机状态。 */
            Command_SetStatusLed(false);
            Command_Send("EVENT,BUTTON_LED_OFF\r\n");
        }
    }
}

HAL_StatusTypeDef Command_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL) return HAL_ERROR;

    command_uart = uart;
    rx_head = 0U;
    rx_tail = 0U;
    rx_overflow = false;
    rx_restart_pending = false;
    rx_health_tick = HAL_GetTick();
    command_line_length = 0U;
    command_line_too_long = false;
    command_state = COMMAND_STATE_WAIT_START;
    timed_test_active = false;
    Command_ResetVelocityWatchdog();
    Motor_Disable();
    Command_SetStatusLed(false);

    button_raw_pressed =
        (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET);
    button_stable_pressed = button_raw_pressed;
    button_change_tick = HAL_GetTick();
    return HAL_UART_Receive_IT(command_uart, &rx_byte, 1U);
}

void Command_Update(void)
{
    uint16_t head_snapshot;

    if (command_uart == NULL) return;
    Command_UpdateButton();

    if (rx_restart_pending)
    {
        /* 不再要求 RxState == HAL_UART_STATE_READY：HAL 出错后可能把状态留在
           BUSY_RX，那个旧条件就永远不成立 → 接收永久失效而 TX 照发。
           这里强制把接收状态机拉回可用，再重新武装。 */
        command_uart->RxState = HAL_UART_STATE_READY;
        command_uart->ErrorCode = HAL_UART_ERROR_NONE;
        if (HAL_UART_Receive_IT(command_uart, &rx_byte, 1U) == HAL_OK)
        {
            rx_restart_pending = false;
        }
    }

    /* RX 健康看门狗（详见 rx_health_tick 处说明）——
       **无条件**强制重新武装接收，每 RX_HEALTH_PERIOD_MS 一次。

       为什么不能"先判断再决定"：本函数第一版写的是
           if (command_uart->RxState != HAL_UART_STATE_BUSY_RX) { ...重新武装... }
       而实测故障态恰恰是 **RxState 停在 BUSY_RX、但 RXNE 中断再也不来**。
       STM32 HAL 在 ORE(溢出) 处理里会走 UART_EndRxTransfer()，
       它会清掉 CR1 的 RXNEIE/PEIE 与 CR3 的 EIE；一旦 EIE 被清，
       后续错误连 HAL_UART_ErrorCallback 都不会再触发 —— 于是
       RxState 留在 BUSY_RX 且没有任何中断来推动状态机，
       上面那个判断就会**永远跳过**，接收永久死亡。
       现场实测：连续 3 次探测（≥6 秒）全部无应答，只能靠复位恢复。

       因此这里不看状态、直接强制：
         1) 把接收状态机拉回 READY（HAL_UART_Receive_IT 的唯一前置条件）
         2) 清错误标志与 ErrorCode（ORE 不清则 RXNE 不会正常产生中断）
         3) 重新武装（内部会重新使能 RXNEIE/PEIE/EIE）
       代价：极小概率丢掉"恰好在这一刻到达的 1 个字节"
       （VEL 以 50Hz 下发，丢 1 帧无影响；PING/START 等有重试）。
       这远小于"整机收不到任何指令"的代价。 */
    if ((HAL_GetTick() - rx_health_tick) >= RX_HEALTH_PERIOD_MS)
    {
        rx_health_tick = HAL_GetTick();
        command_uart->RxState = HAL_UART_STATE_READY;
        command_uart->ErrorCode = HAL_UART_ERROR_NONE;
        __HAL_UART_CLEAR_PEFLAG(command_uart);
        if (HAL_UART_Receive_IT(command_uart, &rx_byte, 1U) == HAL_OK)
        {
            rx_restart_pending = false;
        }
        else
        {
            rx_restart_pending = true;
        }
    }
    if (rx_overflow)
    {
        head_snapshot = rx_head;
        rx_tail = head_snapshot;
        rx_overflow = false;
        command_line_length = 0U;
        command_line_too_long = false;
        Command_Send("ERR,OVERFLOW\r\n");
    }
    while (rx_tail != rx_head)
    {
        uint8_t byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % COMMAND_RX_RING_SIZE);
        Command_ProcessByte(byte);
    }
    Command_UpdateVelocityWatchdog();
    if (timed_test_active &&
        ((HAL_GetTick() - timed_test_start_tick) >= timed_test_duration_ms))
    {
        timed_test_active = false;
        SpeedControl_Stop();
        Command_Send("EVENT,TEST_DONE\r\n");
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next_head;

    if ((command_uart == NULL) || (uart != command_uart)) return;
    next_head = (uint16_t)((rx_head + 1U) % COMMAND_RX_RING_SIZE);
    if (next_head == rx_tail) rx_overflow = true;
    else
    {
        rx_ring[rx_head] = rx_byte;
        rx_head = next_head;
    }
    if (HAL_UART_Receive_IT(command_uart, &rx_byte, 1U) != HAL_OK)
    {
        rx_restart_pending = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if ((command_uart == NULL) || (uart != command_uart)) return;

    /* ORE/NE/FE/PE 之后 HAL 会走 UART_EndRxTransfer：关掉 RXNE/EIE、
       并把 RxState 置 READY。这里额外显式清错误标志（F1 序列：读 SR 再读 DR）
       并把接收状态机强制拉回可用，保证 Command_Update 一定能重新武装成功 ——
       否则接收会永久死亡，而遥测 TX 完全正常，现场极难定位。 */
    __HAL_UART_CLEAR_PEFLAG(command_uart);
    command_uart->ErrorCode = HAL_UART_ERROR_NONE;
    command_uart->RxState = HAL_UART_STATE_READY;
    rx_restart_pending = true;
}
