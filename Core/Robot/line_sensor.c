#include "main.h"
#include "robot_internal.h"

extern ADC_HandleTypeDef hadc1;

#define LINE_ADC_TIMEOUT_MS 2U

static uint16_t adcValues[LINE_SENSOR_COUNT];
static volatile bool sampleReady;

bool LineSensor_Init(void)
{
  sampleReady = false;
  return HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK;
}

bool LineSensor_Read(uint16_t values[LINE_SENSOR_COUNT])
{
  sampleReady = false;
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcValues,
                        LINE_SENSOR_COUNT) != HAL_OK)
  {
    return false;
  }

  uint32_t startMs = HAL_GetTick();
  while (!sampleReady && HAL_GetTick() - startMs < LINE_ADC_TIMEOUT_MS)
  {
  }

  bool ready = sampleReady;
  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK || !ready)
  {
    return false;
  }

  for (uint32_t index = 0; index < LINE_SENSOR_COUNT; ++index)
  {
    values[index] = adcValues[index];
  }
  return true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
  if (adc == &hadc1)
  {
    sampleReady = true;
  }
}
