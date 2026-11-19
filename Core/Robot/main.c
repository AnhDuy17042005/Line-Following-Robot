#include "main.h"
#include "robot.h"
#include "robot_internal.h"
#include "color_sensor.h"
#include "calibration.h"

#include <stdio.h>
#include <string.h>

#define BASE_SPEED_MPS 0.25f
#define WHEEL_SPACING_M 0.174f
#define MOTOR_MAX_RPM 333.0f
#define PI_F 3.14159265f

/* Measure these offsets on the assembled robot before relying on the stop.
   A positive sensor-to-tray offset means the tray is behind the line probe. */
#define LOAD_ZONE_CENTER_MM 2000.0f
#define LOAD_SENSOR_TO_TRAY_MM 100.0f
#define LOAD_BRAKE_LEAD_MM 0.0f
#define LOAD_SLOWDOWN_MM 200.0f
#define LOAD_APPROACH_SPEED_MPS 0.10f

#define FORK_PATTERN_TICKS 3U
#define FORK_SPEED_MPS 0.25f
#define FORK_MAX_YAW_RAD_S 0.30f
#define FORK_ACQUIRED_TICKS 5U
#define FORK_TIMEOUT_MS 12000U

#define END_MIN_AFTER_FORK_MM 1800.0f
#define END_LINE_LOST_TICKS 8U

#define LINE_LOST_CONFIRM_TICKS 3U
#define LOAD_MIN_SETTLE_MS 150U
#define LOAD_STABLE_TICKS 10U
#define ENCODER_STALL_MS 500U

#define COLOR_SAMPLE_INTERVAL_MS 120U
#define COLOR_CONFIRM_SAMPLES 10U

#define STATUS_HEARTBEAT_MS 1000U

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;

typedef enum
{
  ROBOT_STATUS_INIT,
  ROBOT_STATUS_NO_LINE,
  ROBOT_STATUS_RUNNING,
  ROBOT_STATUS_STOPPED,
  ROBOT_STATUS_ERROR
} RobotStatus;

typedef enum
{
  RUN_TO_LOAD,
  STOP_LOAD,
  READ_COLOR,
  RUN_TO_FORK,
  FORK,
  RUN_TO_END,
  STOP_END
} RobotPhase;

typedef struct
{
  RobotPhase phase;
  ColorSensorColor cargoColor;
  ColorSensorColor candidateColor;
  uint8_t colorStreak;
  uint8_t stoppedTicks;
  uint32_t phaseStartedMs;
  uint32_t lastColorSampleMs;
  uint32_t lastEncoderMotionMs;
  float lastDistanceMm;
  float loadStopDistanceMm;
  float forkDetectedDistanceMm;
  uint8_t forkPatternTicks;
  uint8_t forkAcquiredTicks;
  bool forkSideSeen;
  bool distanceStarted;
  bool fault;
} RobotProcess;

static volatile uint32_t controlTicks;
static uint32_t processedTicks;
static uint32_t lastControlMs;
static uint32_t lastLogMs;
static uint32_t lastHeartbeatMs;
static volatile RobotStatus robotStatus;
static uint8_t ledTicks;
static RobotProcess process;
static char faultMessage[128];
static bool calibrationMode;
static bool calibrationReported;
static uint8_t lineLostTicks;

static void Robot_Print(const char *message)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)message,
                          (uint16_t)strlen(message), 20U);
}

static const char *Robot_PhaseName(RobotPhase phase)
{
  switch (phase)
  {
    case RUN_TO_LOAD: return "RUN_TO_LOAD";
    case STOP_LOAD: return "STOP_LOAD";
    case READ_COLOR: return "READ_COLOR";
    case RUN_TO_FORK: return "RUN_TO_FORK";
    case FORK: return "FORK";
    case RUN_TO_END: return "RUN_TO_END";
    case STOP_END: return "STOP_END";
    default: return "UNKNOWN";
  }
}

static void Robot_EnterPhase(RobotPhase phase)
{
  char message[48];
  process.phase = phase;
  process.phaseStartedMs = HAL_GetTick();
  (void)snprintf(message, sizeof(message), "STATE %s\r\n",
                 Robot_PhaseName(phase));
  Robot_Print(message);
}

static void Robot_Fault(const char *message)
{
  Motor_Stop();
  LinePd_Reset();
  SpeedPi_Reset();
  process.fault = true;
  robotStatus = ROBOT_STATUS_ERROR;
  (void)snprintf(faultMessage, sizeof(faultMessage), "%s", message);
  Robot_Print(faultMessage);
}

static void Robot_LogHeartbeat(void)
{
  if (!process.fault && process.phase != STOP_LOAD &&
      process.phase != READ_COLOR && process.phase != STOP_END)
  {
    return;
  }

  uint32_t nowMs = HAL_GetTick();
  if (nowMs - lastHeartbeatMs < STATUS_HEARTBEAT_MS)
  {
    return;
  }
  lastHeartbeatMs = nowMs;
  if (process.fault)
  {
    Robot_Print(faultMessage);
    return;
  }

  const char *cargoName = process.cargoColor == COLOR_SENSOR_RED ? "RED" :
                          process.cargoColor == COLOR_SENSOR_BLUE ? "BLUE" :
                          "NONE";
  char message[96];
  (void)snprintf(message, sizeof(message),
                 "STATE %s D=%d mm CARGO=%s COLOR=%u/%u STOP=%u/%u\r\n",
                 Robot_PhaseName(process.phase), (int)Motor_GetDistanceMm(),
                 cargoName, (unsigned)process.colorStreak,
                 (unsigned)COLOR_CONFIRM_SAMPLES,
                 (unsigned)process.stoppedTicks,
                 (unsigned)LOAD_STABLE_TICKS);
  Robot_Print(message);
}

static void Robot_PrintLogHeader(void)
{
  Robot_Print("      TCRT5000 ADC       |  Error |  SET RPM  | MEAS RPM  | DIST mm | STATUS\r\n");
  Robot_Print("  T1   T2   T3   T4   T5 | 0.1 mm |    L    R |    L    R |         |\r\n");
  Robot_Print("----------------------------------------------------------------------------------\r\n");
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
  int distanceMm = process.distanceStarted ? (int)Motor_GetDistanceMm() : 0;
  if (values == NULL)
  {
    length = snprintf(message, sizeof(message),
                      "%24s | %6s | %9s | %9s | %7d | %-12s\r\n",
                      "-", "-", "-", "-", distanceMm, state);
  }
  else
  {
    length = snprintf(message, sizeof(message),
                      "%4u %4u %4u %4u %4u | %6d | %4d %4d | %4d %4d | %7d | %-12s\r\n",
                      (unsigned)values[0], (unsigned)values[1],
                      (unsigned)values[2], (unsigned)values[3],
                      (unsigned)values[4],
                      (int)(errorMeters * 10000.0f),
                      (int)leftSetRpm, (int)rightSetRpm,
                      (int)leftRpm, (int)rightRpm, distanceMm, state);
  }
  if (length > 0 && length < (int)sizeof(message))
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)message,
                            (uint16_t)length, 20U);
  }
}

static void Robot_ReadColorTask(void)
{
  uint32_t nowMs = HAL_GetTick();
  if (nowMs - process.lastColorSampleMs < COLOR_SAMPLE_INTERVAL_MS)
  {
    return;
  }
  process.lastColorSampleMs = nowMs;

  ColorSensorSample sample;
  ColorSensorStatus result = ColorSensor_Read(&sample);
  if (result == COLOR_SENSOR_NOT_READY)
  {
    return; /* No new conversion yet; do not count the same sample twice. */
  }
  if (result != COLOR_SENSOR_OK)
  {
    process.candidateColor = COLOR_SENSOR_NONE;
    process.colorStreak = 0U;
    Robot_Print("COLOR READ ERROR; streak reset\r\n");
    return;
  }

  ColorSensorColor color = ColorSensor_Detect(&sample);
  if (color == COLOR_SENSOR_NONE)
  {
    process.candidateColor = COLOR_SENSOR_NONE;
    process.colorStreak = 0U;
  }
  else if (color == process.candidateColor)
  {
    ++process.colorStreak;
  }
  else
  {
    process.candidateColor = color;
    process.colorStreak = 1U;
  }

  char message[96];
  const char *colorName = color == COLOR_SENSOR_RED ? "RED" :
                          color == COLOR_SENSOR_BLUE ? "BLUE" : "NONE";
  (void)snprintf(message, sizeof(message),
                 "COLOR R=%u G=%u B=%u C=%u %s %u/%u\r\n",
                 (unsigned)sample.red, (unsigned)sample.green,
                 (unsigned)sample.blue, (unsigned)sample.clear,
                 colorName, (unsigned)process.colorStreak,
                 (unsigned)COLOR_CONFIRM_SAMPLES);
  Robot_Print(message);

  if (process.colorStreak >= COLOR_CONFIRM_SAMPLES)
  {
    process.cargoColor = color;
    Robot_Print(color == COLOR_SENSOR_RED ? "CARGO RED\r\n" :
                "CARGO BLUE\r\n");
    /* Restart the movement watchdog after the deliberate stop for color. */
    LinePd_Reset();
    SpeedPi_Reset();
    process.lastDistanceMm = Motor_GetDistanceMm();
    process.lastEncoderMotionMs = HAL_GetTick();
    process.forkPatternTicks = 0U;
    process.forkAcquiredTicks = 0U;
    process.forkSideSeen = false;
    lineLostTicks = 0U;
    Robot_EnterPhase(RUN_TO_FORK);
    processedTicks = controlTicks;
    lastControlMs = HAL_GetTick();
    Robot_Print("RESUME LINE FOLLOWING; CARGO SAVED\r\n");
  }
}

static void Robot_StopLoadTask(float leftRpm, float rightRpm)
{
  Motor_Stop();
  if (leftRpm > -4.6f && leftRpm < 4.6f &&
      rightRpm > -4.6f && rightRpm < 4.6f)
  {
    if (process.stoppedTicks < LOAD_STABLE_TICKS)
    {
      ++process.stoppedTicks;
    }
  }
  else
  {
    process.stoppedTicks = 0U;
  }

  if (process.stoppedTicks < LOAD_STABLE_TICKS ||
      HAL_GetTick() - process.phaseStartedMs < LOAD_MIN_SETTLE_MS)
  {
    return;
  }

  char message[64];
  (void)snprintf(message, sizeof(message), "LOAD STOP AT %d mm\r\n",
                 (int)Motor_GetDistanceMm());
  Robot_Print(message);

  ColorSensorStatus result = ColorSensor_Init(NULL);
  if (result != COLOR_SENSOR_OK)
  {
    (void)snprintf(message, sizeof(message),
                   "COLOR INIT FAIL status=%d\r\n", (int)result);
    Robot_Fault(message);
    return;
  }
  process.lastColorSampleMs = HAL_GetTick() - COLOR_SAMPLE_INTERVAL_MS;
  Robot_EnterPhase(READ_COLOR);
}

bool Robot_Init(void)
{
  memset(&process, 0, sizeof(process));
  process.cargoColor = COLOR_SENSOR_NONE;
  process.candidateColor = COLOR_SENSOR_NONE;
  process.loadStopDistanceMm = LOAD_ZONE_CENTER_MM +
                               LOAD_SENSOR_TO_TRAY_MM - LOAD_BRAKE_LEAD_MM;
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
  {
    char message[40];
    (void)snprintf(message, sizeof(message), "BOOT RCC_CSR=%08lX\r\n",
                   (unsigned long)Calibration_GetBootResetFlags());
    Robot_Print(message);
  }
  calibrationMode = !Calibration_Init();
  calibrationReported = false;
  if (calibrationMode)
  {
    Motor_Stop();
    Robot_Print("LINE CAL: MOVE ALL 5 SENSORS OVER WHITE AND BLACK FOR 20 S\r\n");
    return true;
  }
  Robot_Print("LINE CAL: LOADED FROM FLASH; ROBOT READY\r\n");
  Robot_PrintLogHeader();
  Robot_EnterPhase(RUN_TO_LOAD);
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Motor_Stop();
    Robot_Print("TIM2 START FAIL\r\n");
    return false;
  }
  lastControlMs = HAL_GetTick();
  lastLogMs = lastControlMs;
  lastHeartbeatMs = lastControlMs;
  process.lastEncoderMotionMs = lastControlMs;
  robotStatus = ROBOT_STATUS_NO_LINE;
  return true;
}

void Robot_Task(void)
{
  if (calibrationMode)
  {
    CalibrationStatus result = Calibration_Task();
    if (result != CALIBRATION_IN_PROGRESS && !calibrationReported)
    {
      calibrationReported = true;
      Robot_Print(result == CALIBRATION_SAVED ?
                  "LINE CAL SAVED; POWER OFF CONTROL, PLACE AT START, POWER ON\r\n" :
                  "LINE CAL FAILED; CHECK ALL 5 SENSORS, THEN PRESS RESET\r\n");
    }
    return;
  }
  Robot_LogHeartbeat();
  if (process.fault)
  {
    return;
  }
  if (process.phase == READ_COLOR)
  {
    Robot_ReadColorTask();
    return;
  }
  if (process.phase != RUN_TO_LOAD && process.phase != STOP_LOAD &&
      process.phase != RUN_TO_FORK && process.phase != FORK &&
      process.phase != RUN_TO_END)
  {
    return;
  }

  uint32_t currentTicks = controlTicks;
  if (currentTicks == processedTicks)
  {
    if (HAL_GetTick() - lastControlMs >= 100U)
    {
      Robot_Fault("NO TIM2 TICK; MOTORS STOPPED\r\n");
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

  if (process.phase == STOP_LOAD)
  {
    Robot_StopLoadTask(leftMeasuredRpm, rightMeasuredRpm);
    return;
  }

  float distanceMm = Motor_GetDistanceMm();
  if (process.phase == RUN_TO_LOAD && process.distanceStarted &&
      distanceMm >= process.loadStopDistanceMm)
  {
    Motor_Stop();
    LinePd_Reset();
    SpeedPi_Reset();
    process.stoppedTicks = 0U;
    robotStatus = ROBOT_STATUS_STOPPED;
    Robot_EnterPhase(STOP_LOAD);
    return;
  }

  if (!LineSensor_Read(lineValues))
  {
    Motor_Stop();
    LinePd_Reset();
    SpeedPi_Reset();
    process.lastEncoderMotionMs = HAL_GetTick();
    robotStatus = ROBOT_STATUS_ERROR;
    Robot_LogStatus("ADC READ FAIL", NULL, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f);
    return;
  }

  if (process.phase == RUN_TO_FORK)
  {
    if (LineError_ForkPattern(lineValues))
    {
      if (process.forkPatternTicks < FORK_PATTERN_TICKS)
      {
        ++process.forkPatternTicks;
      }
    }
    else
    {
      process.forkPatternTicks = 0U;
    }
    if (process.forkPatternTicks >= FORK_PATTERN_TICKS)
    {
      if (process.cargoColor != COLOR_SENSOR_RED &&
          process.cargoColor != COLOR_SENSOR_BLUE)
      {
        Robot_Fault("FORK COLOR MISSING; MOTORS STOPPED\r\n");
        return;
      }
      process.forkAcquiredTicks = 0U;
      process.forkSideSeen = false;
      process.forkDetectedDistanceMm = distanceMm;
      lineLostTicks = 0U;
      LinePd_Reset();
      Robot_EnterPhase(FORK);
      char message[128];
      (void)snprintf(message, sizeof(message),
                     "FORK SELECT %s D=%d mm A=%u,%u,%u,%u,%u\r\n",
                     process.cargoColor == COLOR_SENSOR_RED ? "RED LEFT" :
                     "BLUE RIGHT", (int)distanceMm,
                     (unsigned)lineValues[0], (unsigned)lineValues[1],
                     (unsigned)lineValues[2], (unsigned)lineValues[3],
                     (unsigned)lineValues[4]);
      Robot_Print(message);
    }
  }

  bool selectedSideSeen = false;
  bool branchAcquired = false;
  bool lineValid = process.phase == FORK ?
                   LineError_CalculateFork(lineValues,
                                           process.cargoColor == COLOR_SENSOR_RED,
                                           &errorMeters, &selectedSideSeen,
                                           &branchAcquired) :
                   LineError_Calculate(lineValues, &errorMeters);
  if (!lineValid)
  {
    bool endAreaEligible = process.phase == RUN_TO_END &&
                           distanceMm - process.forkDetectedDistanceMm >=
                           END_MIN_AFTER_FORK_MM;
    uint8_t lostLimit = endAreaEligible ? END_LINE_LOST_TICKS :
                        LINE_LOST_CONFIRM_TICKS;
    if (lineLostTicks < lostLimit)
    {
      ++lineLostTicks;
    }
    if (process.distanceStarted && lineLostTicks < lostLimit)
    {
      /* Keep the previous PWM briefly across a gap in the painted line. */
      LinePd_Reset();
      return;
    }
    Motor_Stop();
    LinePd_Reset();
    SpeedPi_Reset();
    if (process.phase == FORK)
    {
      char message[112];
      (void)snprintf(message, sizeof(message),
                     "FORK LINE LOST D=%d A=%u,%u,%u,%u,%u; STOPPED\r\n",
                     (int)distanceMm, (unsigned)lineValues[0],
                     (unsigned)lineValues[1], (unsigned)lineValues[2],
                     (unsigned)lineValues[3], (unsigned)lineValues[4]);
      Robot_Fault(message);
      return;
    }
    if (endAreaEligible)
    {
      robotStatus = ROBOT_STATUS_STOPPED;
      Robot_EnterPhase(STOP_END);
      char message[64];
      (void)snprintf(message, sizeof(message),
                     "END LINE EXIT D=%d mm; MOTORS STOPPED\r\n",
                     (int)distanceMm);
      Robot_Print(message);
      return;
    }
    process.lastEncoderMotionMs = HAL_GetTick();
    robotStatus = ROBOT_STATUS_NO_LINE;
    Robot_LogStatus("NO LINE", lineValues, 0.0f, 0.0f, 0.0f,
                    leftMeasuredRpm, rightMeasuredRpm);
    return;
  }
  lineLostTicks = 0U;

  if (process.phase == FORK)
  {
    if (HAL_GetTick() - process.phaseStartedMs >= FORK_TIMEOUT_MS)
    {
      Robot_Fault("FORK TIMEOUT; BRANCH NOT ACQUIRED; MOTORS STOPPED\r\n");
      return;
    }
    if (selectedSideSeen)
    {
      process.forkSideSeen = true;
    }
    if (process.forkSideSeen && branchAcquired)
    {
      if (process.forkAcquiredTicks < FORK_ACQUIRED_TICKS)
      {
        ++process.forkAcquiredTicks;
      }
    }
    else
    {
      process.forkAcquiredTicks = 0U;
    }
    if (process.forkAcquiredTicks >= FORK_ACQUIRED_TICKS)
    {
      uint32_t forkDurationMs = HAL_GetTick() - process.phaseStartedMs;
      LinePd_Reset();
      Robot_EnterPhase(RUN_TO_END);
      char message[128];
      (void)snprintf(message, sizeof(message),
                     "FORK BRANCH ACQUIRED T=%lu ms D=%d mm A=%u,%u,%u,%u,%u; TO_END\r\n",
                     (unsigned long)forkDurationMs, (int)distanceMm,
                     (unsigned)lineValues[0], (unsigned)lineValues[1],
                     (unsigned)lineValues[2], (unsigned)lineValues[3],
                     (unsigned)lineValues[4]);
      Robot_Print(message);
      if (!LineError_Calculate(lineValues, &errorMeters))
      {
        Robot_Fault("FORK EXIT LINE LOST; MOTORS STOPPED\r\n");
        return;
      }
    }
  }

  if (!process.distanceStarted)
  {
    /* The first valid line marks the start of motion and the 2000 mm route. */
    Motor_ResetDistance();
    process.distanceStarted = true;
    process.lastDistanceMm = 0.0f;
    process.lastEncoderMotionMs = HAL_GetTick();
    distanceMm = 0.0f;
    Robot_Print("DISTANCE START 0 mm\r\n");
  }

  if (distanceMm > process.lastDistanceMm + 0.05f)
  {
    process.lastEncoderMotionMs = HAL_GetTick();
  }
  process.lastDistanceMm = distanceMm;
  if (HAL_GetTick() - process.lastEncoderMotionMs >= ENCODER_STALL_MS)
  {
    int32_t leftCounts;
    int32_t rightCounts;
    uint32_t leftPwmPercent;
    uint32_t rightPwmPercent;
    bool standbyEnabled;
    char message[128];
    Motor_GetTravelCounts(&leftCounts, &rightCounts);
    Motor_GetDriveState(&leftPwmPercent, &rightPwmPercent, &standbyEnabled);
    (void)snprintf(message, sizeof(message),
                   "ENCODER STALL D=%d L=%ld R=%ld PWM=%lu/%lu STBY=%u A=%u,%u,%u,%u,%u\r\n",
                   (int)distanceMm, (long)leftCounts, (long)rightCounts,
                   (unsigned long)leftPwmPercent,
                   (unsigned long)rightPwmPercent,
                   standbyEnabled ? 1U : 0U,
                   (unsigned)lineValues[0], (unsigned)lineValues[1],
                   (unsigned)lineValues[2], (unsigned)lineValues[3],
                   (unsigned)lineValues[4]);
    Robot_Fault(message);
    return;
  }

  float yawRate = LinePd_Update(errorMeters, periodSeconds);
  if (process.phase == FORK)
  {
    if (yawRate > FORK_MAX_YAW_RAD_S)
    {
      yawRate = FORK_MAX_YAW_RAD_S;
    }
    else if (yawRate < -FORK_MAX_YAW_RAD_S)
    {
      yawRate = -FORK_MAX_YAW_RAD_S;
    }
  }
  float baseSpeed = BASE_SPEED_MPS;
  if (process.phase == RUN_TO_LOAD &&
      distanceMm >= process.loadStopDistanceMm - LOAD_SLOWDOWN_MM)
  {
    baseSpeed = LOAD_APPROACH_SPEED_MPS;
  }
  else if (process.phase == FORK)
  {
    baseSpeed = FORK_SPEED_MPS;
  }
  float leftSpeed = baseSpeed - 0.5f * WHEEL_SPACING_M * yawRate;
  float rightSpeed = baseSpeed + 0.5f * WHEEL_SPACING_M * yawRate;
  if (leftSpeed < 0.0f)
  {
    leftSpeed = 0.0f;
  }
  if (rightSpeed < 0.0f)
  {
    rightSpeed = 0.0f;
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
  const char *state = process.phase == RUN_TO_FORK ? "TO_FORK" :
                      process.phase == FORK ?
                      (process.cargoColor == COLOR_SENSOR_RED ?
                       "FORK_RED" : "FORK_BLUE") :
                      process.phase == RUN_TO_END ? "TO_END" : "TO_LOAD";
  Robot_LogStatus(state,
                  lineValues, errorMeters, leftSetRpm,
                  rightSetRpm, leftMeasuredRpm,
                  rightMeasuredRpm);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer == &htim2)
  {
    ++controlTicks;
    /* Fast: no line; medium: stopped; slow: running; off: control fault. */
    if (robotStatus == ROBOT_STATUS_ERROR)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      ledTicks = 0U;
    }
    else if (robotStatus == ROBOT_STATUS_NO_LINE ||
             robotStatus == ROBOT_STATUS_RUNNING ||
             robotStatus == ROBOT_STATUS_STOPPED)
    {
      uint8_t halfPeriodTicks = robotStatus == ROBOT_STATUS_NO_LINE ? 10U :
                                robotStatus == ROBOT_STATUS_STOPPED ? 25U : 50U;
      if (++ledTicks >= halfPeriodTicks)
      {
        ledTicks = 0U;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
    }
  }
}
