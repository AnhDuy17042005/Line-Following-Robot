#include "robot_internal.h"

typedef struct
{
  float kp;
  float ki;
  float integral;
} SpeedPiState;

static SpeedPiState leftPi = {0.72892f, 6.3088f, 0.0f};
static SpeedPiState rightPi = {0.73493f, 6.5438f, 0.0f};

static float SpeedPi_Step(SpeedPiState *state, float setRpm,
                          float measuredRpm, float periodSeconds)
{
  if (setRpm <= 0.0f)
  {
    state->integral = 0.0f;
    return 0.0f;
  }

  float error = setRpm - measuredRpm;
  float nextIntegral = state->integral + error * periodSeconds;
  float nextDuty = state->kp * error + state->ki * nextIntegral;

  if (!((nextDuty > 100.0f && error > 0.0f) ||
        (nextDuty < 0.0f && error < 0.0f)))
  {
    state->integral = nextIntegral;
  }

  float duty = state->kp * error + state->ki * state->integral;
  if (duty < 0.0f)
  {
    return 0.0f;
  }
  if (duty > 100.0f)
  {
    return 100.0f;
  }
  return duty;
}

void SpeedPi_Reset(void)
{
  leftPi.integral = 0.0f;
  rightPi.integral = 0.0f;
}

void SpeedPi_Update(float leftSetRpm, float rightSetRpm,
                    float leftMeasuredRpm, float rightMeasuredRpm,
                    float periodSeconds, float *leftDutyPercent,
                    float *rightDutyPercent)
{
  *leftDutyPercent = SpeedPi_Step(&leftPi, leftSetRpm,
                                  leftMeasuredRpm, periodSeconds);
  *rightDutyPercent = SpeedPi_Step(&rightPi, rightSetRpm,
                                   rightMeasuredRpm, periodSeconds);
}
