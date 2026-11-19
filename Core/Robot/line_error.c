#include "robot_internal.h"

#include <stddef.h>

/* A wide calibration span can make a clear partial line read below 0.5. */
#define MIN_LINE_SIGNAL 0.5f
#define MIN_PARTIAL_LINE_SIGNAL 0.10f
#define MIN_PARTIAL_LINE_ADC 100

static uint16_t adcWhite[LINE_SENSOR_COUNT];
static uint16_t adcBlack[LINE_SENSOR_COUNT];
static bool calibrationReady;
/* On this robot T1 is on the right and T5 is on the left. */
static const float sensorPositionMm[LINE_SENSOR_COUNT] = {
  -30.0f, -15.0f, 0.0f, 15.0f, 30.0f
};

bool LineError_SetCalibration(const uint16_t white[LINE_SENSOR_COUNT],
                              const uint16_t black[LINE_SENSOR_COUNT])
{
  if (white == NULL || black == NULL)
  {
    return false;
  }
  for (uint32_t index = 0; index < LINE_SENSOR_COUNT; ++index)
  {
    if (white[index] > 4095U || black[index] > 4095U ||
        black[index] < white[index] + 20U)
    {
      return false;
    }
  }
  for (uint32_t index = 0; index < LINE_SENSOR_COUNT; ++index)
  {
    adcWhite[index] = white[index];
    adcBlack[index] = black[index];
  }
  calibrationReady = true;
  return true;
}

bool LineError_Calculate(const uint16_t values[LINE_SENSOR_COUNT],
                         float *errorMeters)
{
  if (!calibrationReady || values == NULL || errorMeters == NULL)
  {
    return false;
  }
  float signalSum = 0.0f;
  float weightedPositionMm = 0.0f;
  bool partialLineSeen = false;

  for (uint32_t index = 0; index < LINE_SENSOR_COUNT; ++index)
  {
    int32_t contrast = (int32_t)values[index] - adcWhite[index];
    float signal = (float)contrast /
                   (float)(adcBlack[index] - adcWhite[index]);
    if (signal < 0.0f)
    {
      signal = 0.0f;
    }
    else if (signal > 1.0f)
    {
      signal = 1.0f;
    }
    if (signal >= MIN_PARTIAL_LINE_SIGNAL &&
        contrast >= MIN_PARTIAL_LINE_ADC)
    {
      partialLineSeen = true;
    }
    signalSum += signal;
    weightedPositionMm += sensorPositionMm[index] * signal;
  }

  if (signalSum < MIN_LINE_SIGNAL && !partialLineSeen)
  {
    return false;
  }

  *errorMeters = (weightedPositionMm / signalSum) * 0.001f;
  return true;
}
