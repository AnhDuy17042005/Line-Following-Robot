#ifndef ROBOT_INTERNAL_H
#define ROBOT_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 5U
#define CONTROL_PERIOD_S 0.01f
#define WHEEL_DIAMETER_M 0.080f

bool LineSensor_Init(void);
bool LineSensor_Read(uint16_t values[LINE_SENSOR_COUNT]);

bool LineError_Calculate(const uint16_t values[LINE_SENSOR_COUNT],
                         float *errorMeters);
bool LineError_SetCalibration(const uint16_t white[LINE_SENSOR_COUNT],
                              const uint16_t black[LINE_SENSOR_COUNT]);

void LinePd_Reset(void);
float LinePd_Update(float errorMeters, float periodSeconds);

void SpeedPi_Reset(void);
void SpeedPi_Update(float leftSetRpm, float rightSetRpm,
                    float leftMeasuredRpm, float rightMeasuredRpm,
                    float periodSeconds, float *leftDutyPercent,
                    float *rightDutyPercent);

bool Motor_Init(void);
void Motor_Stop(void);
void Motor_ReadEncoderCounts(uint16_t *leftCount, uint16_t *rightCount);
void Motor_ReadSpeed(float periodSeconds, float *leftRpm, float *rightRpm);
void Motor_ResetDistance(void);
float Motor_GetDistanceMm(void);
void Motor_GetTravelCounts(int32_t *leftCounts, int32_t *rightCounts);
void Motor_GetDriveState(uint32_t *leftPwmPercent,
                         uint32_t *rightPwmPercent, bool *standbyEnabled);
void Motor_SetDuty(float leftDutyPercent, float rightDutyPercent);

#endif
