#include "main.h"
#include "robot.h"
#include "robot_internal.h"

#include <stdio.h>
#include <string.h>

#define BASE_SPEED_MPS 0.25f
#define WHEEL_SPACING_M 0.174f
#define WHEEL_DIAMETER_M 0.080f
#define MOTOR_MAX_RPM 333.0f
#define PI_F 3.14159265f

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;

typedef enum
{
  ROBOT_STATUS_INIT,
  ROBOT_STATUS_NO_LINE,
  ROBOT_STATUS_RUNNING,
  ROBOT_STATUS_ERROR
} RobotStatus;

static volatile uint32_t controlTicks;
static uint32_t processedTicks;
static uint32_t lastControlMs;
static uint32_t lastLogMs;
static volatile RobotStatus robotStatus;
static uint8_t ledTicks;

static void Robot_Print(const char *message)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)message,
                          (uint16_t)strlen(message), 20U);
}

static void Robot_PrintLogHeader(void)
{
  Robot_Print("      TCRT5000 ADC       |  Error |  SET RPM  | MEAS RPM  | STATUS\r\n");
  Robot_Print("  T1   T2   T3   T4   T5 | 0.1 mm |    L    R |    L    R |\r\n");
  Robot_Print("------------------------------------------------------------------------\r\n");
}

static void Robot_LogStatus(const char *state,
                            const uint16_t values[LINE_SENSOR_COUNT],
                            float errorMeters, float leftSetRpm,
                            float rightSetRpm, float leftRpm, float rightRpm)
{
  uint32_t nowMs = HAL_GetTick();
  if (nowMs - lastLogMs < 500U)
  {
    return;
  }
  lastLogMs = nowMs;

  char message[120];
  int length;
  if (values == NULL)
  {
    length = snprintf(message, sizeof(message),
                      "%24s | %6s | %9s | %9s | %-12s\r\n",
                      "-", "-", "-", "-", state);
  }
  else
  {
    length = snprintf(message, sizeof(message),
                      "%4u %4u %4u %4u %4u | %6d | %4d %4d | %4d %4d | %-12s\r\n",
                      (unsigned)values[0], (unsigned)values[1],
                      (unsigned)values[2], (unsigned)values[3],
                      (unsigned)values[4],
                      (int)(errorMeters * 10000.0f),
                      (int)leftSetRpm, (int)rightSetRpm,
                      (int)leftRpm, (int)rightRpm, state);
  }
  if (length > 0 && length < (int)sizeof(message))
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)message,
                            (uint16_t)length, 20U);
  }
}

bool Robot_Init(void)
{
  controlTicks = 0U;
  processedTicks = 0U;
  robotStatus = ROBOT_STATUS_INIT;
  ledTicks = 0U;
  LinePd_Reset();
  SpeedPi_Reset();
  Robot_Print("ROBOT INIT MOTOR DIR CHECK 1\r\n");

  if (!Motor_Init())
  {
    Motor_Stop();
    Robot_Print("MOTOR INIT FAIL\r\n");
    return false;
  }
  if (!LineSensor_Init())
  {
    Motor_Stop();
    Robot_Print("ADC INIT FAIL\r\n");
    return false;
  }
  Robot_Print("ROBOT READY ADC CAL\r\n");
  Robot_PrintLogHeader();
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Motor_Stop();
    Robot_Print("TIM2 START FAIL\r\n");
    return false;
  }
  lastControlMs = HAL_GetTick();
  lastLogMs = lastControlMs;
  robotStatus = ROBOT_STATUS_NO_LINE;
  return true;
}

void Robot_Task(void)
{
  uint32_t currentTicks = controlTicks;
  if (currentTicks == processedTicks)
  {
    if (HAL_GetTick() - lastControlMs >= 100U)
    {
      Robot_LogStatus("NO TIM2 TICK", NULL, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f);
    }
    return;
  }
  lastControlMs = HAL_GetTick();

  uint32_t elapsedTicks = currentTicks - processedTicks;
  processedTicks = currentTicks;

  uint16_t lineValues[LINE_SENSOR_COUNT];
  float errorMeters;
  float leftMeasuredRpm;
  float rightMeasuredRpm;
  float leftDutyPercent;
  float rightDutyPercent;
  float periodSeconds = CONTROL_PERIOD_S * (float)elapsedTicks;

  Motor_ReadSpeed(periodSeconds, &leftMeasuredRpm, &rightMeasuredRpm);
  if (elapsedTicks != 1U)
  {
    Motor_Stop();
    LinePd_Reset();
    SpeedPi_Reset();
    robotStatus = ROBOT_STATUS_ERROR;
    Robot_LogStatus("MISSED TICK", NULL, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f);
    return;
  }

  if (!LineSensor_Read(lineValues))
  {
    Motor_Stop();
    LinePd_Reset();
    SpeedPi_Reset();
    robotStatus = ROBOT_STATUS_ERROR;
    Robot_LogStatus("ADC READ FAIL", NULL, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f);
    return;
  }

  if (!LineError_Calculate(lineValues, &errorMeters))
  {
    Motor_Stop();
    LinePd_Reset();
    SpeedPi_Reset();
    robotStatus = ROBOT_STATUS_NO_LINE;
    Robot_LogStatus("NO LINE", lineValues, 0.0f, 0.0f, 0.0f,
                    leftMeasuredRpm, rightMeasuredRpm);
    return;
  }

  float yawRate = LinePd_Update(errorMeters, periodSeconds);
  float leftSpeed = BASE_SPEED_MPS - 0.5f * WHEEL_SPACING_M * yawRate;
  float rightSpeed = BASE_SPEED_MPS + 0.5f * WHEEL_SPACING_M * yawRate;
  if (leftSpeed < 0.0f)
  {
    leftSpeed = 0.0f;
  }
  if (rightSpeed < 0.0f)
  {
    rightSpeed = 0.0f;
  }
  if (leftSpeed > BASE_SPEED_MPS)
  {
    leftSpeed = BASE_SPEED_MPS;
  }
  if (rightSpeed > BASE_SPEED_MPS)
  {
    rightSpeed = BASE_SPEED_MPS;
  }

  float rpmPerMeterPerSecond = 60.0f / (PI_F * WHEEL_DIAMETER_M);
  float leftSetRpm = leftSpeed * rpmPerMeterPerSecond;
  float rightSetRpm = rightSpeed * rpmPerMeterPerSecond;
  if (leftSetRpm > MOTOR_MAX_RPM)
  {
    leftSetRpm = MOTOR_MAX_RPM;
  }
  if (rightSetRpm > MOTOR_MAX_RPM)
  {
    rightSetRpm = MOTOR_MAX_RPM;
  }

  SpeedPi_Update(leftSetRpm, rightSetRpm,
                 leftMeasuredRpm, rightMeasuredRpm, periodSeconds,
                 &leftDutyPercent, &rightDutyPercent);
  Motor_SetDuty(leftDutyPercent, rightDutyPercent);
  robotStatus = ROBOT_STATUS_RUNNING;
  Robot_LogStatus("RUN", lineValues, errorMeters, leftSetRpm,
                  rightSetRpm, leftMeasuredRpm,
                  rightMeasuredRpm);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer == &htim2)
  {
    ++controlTicks;
    /* Fast blink: no line; slow blink: running; LED off: control fault. */
    if (robotStatus == ROBOT_STATUS_ERROR)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      ledTicks = 0U;
    }
    else if (robotStatus == ROBOT_STATUS_NO_LINE ||
             robotStatus == ROBOT_STATUS_RUNNING)
    {
      uint8_t halfPeriodTicks = robotStatus == ROBOT_STATUS_NO_LINE ? 10U : 50U;
      if (++ledTicks >= halfPeriodTicks)
      {
        ledTicks = 0U;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
    }
  }
}
