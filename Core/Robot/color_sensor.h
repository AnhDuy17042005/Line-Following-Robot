#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

typedef struct
{
  uint16_t clear;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
} ColorSensorSample;

typedef enum
{
  COLOR_SENSOR_NONE,
  COLOR_SENSOR_RED,
  COLOR_SENSOR_BLUE
} ColorSensorColor;

typedef enum
{
  COLOR_SENSOR_OK,
  COLOR_SENSOR_NO_DEVICE,
  COLOR_SENSOR_ID_READ_ERROR,
  COLOR_SENSOR_UNSUPPORTED_ID,
  COLOR_SENSOR_CONFIG_ERROR,
  COLOR_SENSOR_ENABLE_ERROR,
  COLOR_SENSOR_NOT_INITIALIZED,
  COLOR_SENSOR_NOT_READY,
  COLOR_SENSOR_STATUS_READ_ERROR,
  COLOR_SENSOR_DATA_READ_ERROR,
  COLOR_SENSOR_INVALID_ARGUMENT
} ColorSensorStatus;

/* Call while the motors are stopped: initialization waits for the first color
   integration to finish. deviceId may be NULL. */
ColorSensorStatus ColorSensor_Init(uint8_t *deviceId);
/* Blocking I2C read with a 100 ms timeout per transfer. Schedule outside
   the robot's 10 ms control step until an asynchronous read is implemented. */
ColorSensorStatus ColorSensor_Read(ColorSensorSample *sample);
ColorSensorColor ColorSensor_Detect(const ColorSensorSample *sample);

#endif
