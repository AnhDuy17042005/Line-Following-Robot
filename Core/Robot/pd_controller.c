#include "robot_internal.h"

#define LINE_KP 30.3636f
#define LINE_KD -1.5199f

static float previousErrorMeters;
static bool hasPreviousError;

void LinePd_Reset(void)
{
  previousErrorMeters = 0.0f;
  hasPreviousError = false;
}

float LinePd_Update(float errorMeters, float periodSeconds)
{
  float errorRate = 0.0f;
  if (hasPreviousError)
  {
    errorRate = (errorMeters - previousErrorMeters) / periodSeconds;
  }
  previousErrorMeters = errorMeters;
  hasPreviousError = true;
  return LINE_KP * errorMeters + LINE_KD * errorRate;
}
