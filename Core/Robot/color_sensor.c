#include "main.h"
#include "color_sensor.h"

#include <stdbool.h>
#include <stddef.h>

/* TCS34725/TCS34727: HAL expects the 7-bit I2C address shifted left. */
#define TCS_ADDR            (0x29U << 1)
#define TCS_CMD             0x80U
#define TCS_AUTO_INCREMENT  0x20U
#define TCS_REG_ENABLE      0x00U
#define TCS_REG_ATIME       0x01U
#define TCS_REG_CONTROL     0x0FU
#define TCS_REG_ID          0x12U
#define TCS_REG_STATUS      0x13U
#define TCS_REG_CDATA       0x14U

#define TCS_PON             0x01U
#define TCS_AEN             0x02U
#define TCS_AVALID          0x01U
#define I2C_TIMEOUT_MS      100U

extern I2C_HandleTypeDef hi2c1;

static bool initialized;

static HAL_StatusTypeDef Tcs_Write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, TCS_ADDR, TCS_CMD | reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Tcs_Read(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c1, TCS_ADDR, TCS_CMD | reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1U, I2C_TIMEOUT_MS);
}

ColorSensorStatus ColorSensor_Init(uint8_t *deviceId)
{
  uint8_t id = 0U;
  initialized = false;
  if (deviceId != NULL)
  {
    *deviceId = 0U;
  }

  if (HAL_I2C_IsDeviceReady(&hi2c1, TCS_ADDR, 2U, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return COLOR_SENSOR_NO_DEVICE;
  }
  if (Tcs_Read(TCS_REG_ID, &id) != HAL_OK)
  {
    return COLOR_SENSOR_ID_READ_ERROR;
  }
  if (deviceId != NULL)
  {
    *deviceId = id;
  }
  if (id != 0x44U && id != 0x4DU)
  {
    return COLOR_SENSOR_UNSUPPORTED_ID;
  }

  /* 0xD5 = 42 * 2.4 ms (~101 ms); CONTROL=1 gives 4x gain. */
  if (Tcs_Write(TCS_REG_ATIME, 0xD5U) != HAL_OK ||
      Tcs_Write(TCS_REG_CONTROL, 0x01U) != HAL_OK ||
      Tcs_Write(TCS_REG_ENABLE, TCS_PON) != HAL_OK)
  {
    return COLOR_SENSOR_CONFIG_ERROR;
  }
  HAL_Delay(3U); /* Oscillator start after PON. */
  if (Tcs_Write(TCS_REG_ENABLE, TCS_PON | TCS_AEN) != HAL_OK)
  {
    return COLOR_SENSOR_ENABLE_ERROR;
  }

  HAL_Delay(110U); /* Wait for the first complete integration. */
  initialized = true;
  return COLOR_SENSOR_OK;
}

ColorSensorStatus ColorSensor_Read(ColorSensorSample *sample)
{
  uint8_t status = 0U;
  uint8_t raw[8];

  if (sample == NULL)
  {
    return COLOR_SENSOR_INVALID_ARGUMENT;
  }
  if (!initialized)
  {
    return COLOR_SENSOR_NOT_INITIALIZED;
  }
  if (Tcs_Read(TCS_REG_STATUS, &status) != HAL_OK)
  {
    return COLOR_SENSOR_STATUS_READ_ERROR;
  }
  if ((status & TCS_AVALID) == 0U)
  {
    return COLOR_SENSOR_NOT_READY;
  }

  /* One auto-increment transfer reads C, R, G, B as little-endian words. */
  if (HAL_I2C_Mem_Read(&hi2c1, TCS_ADDR,
                       TCS_CMD | TCS_AUTO_INCREMENT | TCS_REG_CDATA,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                       I2C_TIMEOUT_MS) != HAL_OK)
  {
    return COLOR_SENSOR_DATA_READ_ERROR;
  }

  sample->clear = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
  sample->red   = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
  sample->green = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
  sample->blue  = (uint16_t)raw[6] | ((uint16_t)raw[7] << 8);
  return COLOR_SENSOR_OK;
}

/* Starting thresholds for red/blue cargo testing; tune on the robot. */
ColorSensorColor ColorSensor_Detect(const ColorSensorSample *sample)
{
  if (sample == NULL || sample->clear < 100U || sample->clear > 40000U)
  {
    return COLOR_SENSOR_NONE;
  }
  if ((uint32_t)sample->red * 100U > (uint32_t)sample->green * 135U &&
      (uint32_t)sample->red * 100U > (uint32_t)sample->blue * 135U)
  {
    return COLOR_SENSOR_RED;
  }
  if ((uint32_t)sample->blue * 100U > (uint32_t)sample->red * 140U &&
      (uint32_t)sample->blue * 100U > (uint32_t)sample->green * 115U)
  {
    return COLOR_SENSOR_BLUE;
  }
  return COLOR_SENSOR_NONE;
}
