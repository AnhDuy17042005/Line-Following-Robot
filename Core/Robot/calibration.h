#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "robot_internal.h"

typedef enum
{
  CALIBRATION_IN_PROGRESS,
  CALIBRATION_SAVED,
  CALIBRATION_FAILED
} CalibrationStatus;

/* Call before HAL_Init so the reset cause cannot be lost during startup. */
void Calibration_CaptureBootReason(void);
uint32_t Calibration_GetBootResetFlags(void);

/* Returns true when a valid stored calibration was loaded for this firmware. */
bool Calibration_Init(void);
CalibrationStatus Calibration_Task(void);

#endif
