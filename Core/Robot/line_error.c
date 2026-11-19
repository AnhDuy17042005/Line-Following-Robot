#include "robot_internal.h"

#include <stddef.h>

/* A wide calibration span can make a clear partial line read below 0.5. */
#define MIN_LINE_SIGNAL 0.5f
#define MIN_PARTIAL_LINE_SIGNAL 0.10f
#define MIN_PARTIAL_LINE_ADC 100
#define FORK_SIDE_SIGNAL 0.35f
#define FORK_CENTER_WHITE_MAX 0.20f
#define FORK_BRANCH_CENTER_SIGNAL 0.35f
#define FORK_OPPOSITE_WHITE_MAX 0.20f

static uint16_t adcWhite[LINE_SENSOR_COUNT];
static uint16_t adcBlack[LINE_SENSOR_COUNT];
static bool calibrationReady;
/* On this robot T1 is on the right and T5 is on the left. */
static const float sensorPositionMm[LINE_SENSOR_COUNT] = {
  -30.0f, -15.0f, 0.0f, 15.0f, 30.0f
};

static float LineError_Signal(uint16_t value, uint32_t index)
{
  float signal = (float)((int32_t)value - adcWhite[index]) /
                 (float)(adcBlack[index] - adcWhite[index]);
  if (signal < 0.0f)
  {
    return 0.0f;
  }
  return signal > 1.0f ? 1.0f : signal;
}

static bool LineError_CalculateRange(const uint16_t values[LINE_SENSOR_COUNT],
                                     uint32_t first, uint32_t last,
                                     float *errorMeters)
{
  if (!calibrationReady || values == NULL || errorMeters == NULL)
  {
    return false;
  }

  float signalSum = 0.0f;
  float weightedPositionMm = 0.0f;
  bool partialLineSeen = false;
  for (uint32_t index = first; index <= last; ++index)
  {
    int32_t contrast = (int32_t)values[index] - adcWhite[index];
    float signal = LineError_Signal(values[index], index);
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
  return LineError_CalculateRange(values, 0U, LINE_SENSOR_COUNT - 1U,
                                  errorMeters);
}

bool LineError_ForkPattern(const uint16_t values[LINE_SENSOR_COUNT])
{
  if (!calibrationReady || values == NULL)
  {
    return false;
  }
  float right = LineError_Signal(values[0], 0U);
  float left = LineError_Signal(values[4], 4U);
  float innerRight = LineError_Signal(values[1], 1U);
  float innerLeft = LineError_Signal(values[3], 3U);
  if (innerRight > right)
  {
    right = innerRight;
  }
  if (innerLeft > left)
  {
    left = innerLeft;
  }
  /* The split is recognized only when both sides see black and T3 sees white. */
  float center = LineError_Signal(values[2], 2U);
  return left >= FORK_SIDE_SIGNAL && right >= FORK_SIDE_SIGNAL &&
         center <= FORK_CENTER_WHITE_MAX;
}

bool LineError_CalculateFork(const uint16_t values[LINE_SENSOR_COUNT],
                             bool turnLeft, float *errorMeters,
                             bool *selectedSideSeen, bool *branchAcquired)
{
  if (!calibrationReady || values == NULL || errorMeters == NULL ||
      selectedSideSeen == NULL || branchAcquired == NULL)
  {
    return false;
  }

  uint32_t innerIndex = turnLeft ? 3U : 1U;
  uint32_t outerIndex = turnLeft ? 4U : 0U;
  float selectedInner = LineError_Signal(values[innerIndex], innerIndex);
  float selectedOuter = LineError_Signal(values[outerIndex], outerIndex);
  float oppositeInner = LineError_Signal(values[turnLeft ? 1U : 3U],
                                         turnLeft ? 1U : 3U);
  float oppositeOuter = LineError_Signal(values[turnLeft ? 0U : 4U],
                                         turnLeft ? 0U : 4U);
  *selectedSideSeen = selectedInner >= 0.25f || selectedOuter >= 0.25f;
  bool centerSeen = LineError_Signal(values[2], 2U) >=
                    FORK_BRANCH_CENTER_SIGNAL;
  bool oppositeClear = oppositeInner < FORK_OPPOSITE_WHITE_MAX &&
                       oppositeOuter < FORK_OPPOSITE_WHITE_MAX;

  /* During the turn only the two sensors on the chosen side steer the robot.
     The inner sensor is the zero-error reference; the outer sensor steers
     farther into the chosen branch. */
  bool selectedLineValid = LineError_CalculateRange(values,
                                                    turnLeft ? 3U : 0U,
                                                    turnLeft ? 4U : 1U,
                                                    errorMeters);
  /* Once the unselected branch leaves the probe, the normal five-sensor
     follower can take over even if the chosen line has not reached T3. */
  *branchAcquired = oppositeClear && (selectedLineValid || centerSeen);
  if (selectedLineValid)
  {
    *errorMeters -= turnLeft ? 0.015f : -0.015f;
    return true;
  }

  /* The branch can move from the side pair onto T3 before confirmation ends.
     T3 is used only for the handoff; opposite-side sensors never steer. */
  if (centerSeen && oppositeClear)
  {
    *errorMeters = 0.0f;
    return true;
  }
  return false;
}
