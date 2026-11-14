#include "robot_internal.h"

/* Initial lost-line threshold; tune it against measured white noise. */
#define MIN_LINE_SIGNAL 0.5f

/* White and black ADC values measured on this robot. */
static const uint16_t adcWhite[LINE_SENSOR_COUNT] = {
  93U, 91U, 45U, 98U, 71U
};
static const uint16_t adcBlack[LINE_SENSOR_COUNT] = {
  140U, 157U, 159U, 180U, 140U
};
static const float sensorPositionMm[LINE_SENSOR_COUNT] = {
  50.0f, 30.0f, 0.0f, -30.0f, -50.0f
};

bool LineError_Calculate(const uint16_t values[LINE_SENSOR_COUNT],
                         float *errorMeters)
{
  float signalSum = 0.0f;
  float weightedPositionMm = 0.0f;

  for (uint32_t index = 0; index < LINE_SENSOR_COUNT; ++index)
  {
    float signal = (float)((int32_t)values[index] - adcWhite[index]) /
                   (float)(adcBlack[index] - adcWhite[index]);
    if (signal < 0.0f)
    {
      signal = 0.0f;
    }
    else if (signal > 1.0f)
    {
      signal = 1.0f;
    }
    signalSum += signal;
    weightedPositionMm += sensorPositionMm[index] * signal;
  }

  if (signalSum < MIN_LINE_SIGNAL)
  {
    return false;
  }

  *errorMeters = (weightedPositionMm / signalSum) * 0.001f;
  return true;
}
