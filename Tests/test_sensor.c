#include "main.h"
#include "color_sensor.h"

#include <stdio.h>
#include <string.h>

#define LINE_SENSOR_COUNT    5U
#define LINE_TIMEOUT_MS      10U

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

static uint8_t sensor_ready;
static uint8_t header_printed;
static uint16_t line_dma_values[LINE_SENSOR_COUNT];
static volatile uint8_t line_scan_ready;
static uint8_t line_adc_ready;

static void Uart_Print(const char *message)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)message,
                          (uint16_t)strlen(message), 100U);
}

void TestLine_Init(void)
{
  line_adc_ready = 0U;
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Uart_Print("LINE: ADC calibration failed\r\n");
    return;
  }
  line_adc_ready = 1U;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
  if (adc == &hadc1)
  {
    line_scan_ready = 1U;
  }
}

static uint8_t TestLine_Read(uint16_t values[LINE_SENSOR_COUNT])
{
  if (line_adc_ready == 0U)
  {
    return 0U;
  }

  line_scan_ready = 0U;
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)line_dma_values,
                        LINE_SENSOR_COUNT) != HAL_OK)
  {
    Uart_Print("LINE: ADC start failed\r\n");
    return 0U;
  }

  uint32_t start_ms = HAL_GetTick();
  while (line_scan_ready == 0U && HAL_GetTick() - start_ms < LINE_TIMEOUT_MS)
  {
  }

  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
  {
    Uart_Print("LINE: ADC stop failed\r\n");
    return 0U;
  }
  if (line_scan_ready == 0U)
  {
    Uart_Print("LINE: ADC scan timeout\r\n");
    return 0U;
  }

  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    values[index] = line_dma_values[index];
  }
  return 1U;
}

static const char *Color_Name(ColorSensorColor color)
{
  if (color == COLOR_SENSOR_RED)
  {
    return "RED";
  }
  if (color == COLOR_SENSOR_BLUE)
  {
    return "BLUE";
  }
  return "NONE";
}

void TestColor_Init(void)
{
  uint8_t id = 0U;
  char line[64];

  /* Keep both motors in standby while running this sensor-only test. */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  sensor_ready = 0U;

  ColorSensorStatus result = ColorSensor_Init(&id);
  if (result == COLOR_SENSOR_NO_DEVICE)
  {
    Uart_Print("TCS3472: I2C device 0x29 not found\r\n");
    return;
  }
  if (result == COLOR_SENSOR_ID_READ_ERROR)
  {
    Uart_Print("TCS3472: failed to read ID\r\n");
    return;
  }
  if (result == COLOR_SENSOR_UNSUPPORTED_ID)
  {
    (void)snprintf(line, sizeof(line),
                   "TCS3472: ID=0x%02X, expected 0x44 or 0x4D\r\n", id);
    Uart_Print(line);
    return;
  }
  if (result == COLOR_SENSOR_CONFIG_ERROR)
  {
    Uart_Print("TCS3472: configuration failed\r\n");
    return;
  }
  if (result == COLOR_SENSOR_ENABLE_ERROR)
  {
    Uart_Print("TCS3472: failed to enable color measurement\r\n");
    return;
  }
  if (result != COLOR_SENSOR_OK)
  {
    Uart_Print("TCS3472: initialization failed\r\n");
    return;
  }
  sensor_ready = 1U;
  if (header_printed == 0U)
  {
    Uart_Print("              TCRT5000               |||         TCS34725\r\n");
    Uart_Print("-------------------------------------------------------------------------------\r\n");
    Uart_Print("    T1     T2     T3     T4     T5   |||      R      G      B  Clear  Detect\r\n");
    header_printed = 1U;
  }
}

void TestColor_Task(void)
{
  uint16_t line_adc[5];
  ColorSensorSample sample;
  char line[112];

  /* Heartbeat: if the LED toggles, the application reached its main loop. */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  if (sensor_ready == 0U)
  {
    HAL_Delay(1000U);
    TestColor_Init();
    return;
  }

  ColorSensorStatus result = ColorSensor_Read(&sample);
  if (result == COLOR_SENSOR_NOT_READY)
  {
    Uart_Print("TCS3472: waiting for valid color data\r\n");
    HAL_Delay(100U);
    return;
  }
  if (result != COLOR_SENSOR_OK)
  {
    Uart_Print(result == COLOR_SENSOR_STATUS_READ_ERROR ?
               "TCS3472: failed to read STATUS\r\n" :
               "TCS3472: failed to read RGBC\r\n");
    sensor_ready = 0U;
    return;
  }

  if (TestLine_Read(line_adc) == 0U)
  {
    HAL_Delay(300U);
    return;
  }

  (void)snprintf(line, sizeof(line),
                 "%6u %6u %6u %6u %6u   ||| %6u %6u %6u %6u  %s\r\n",
                 (unsigned)line_adc[0], (unsigned)line_adc[1],
                 (unsigned)line_adc[2], (unsigned)line_adc[3],
                 (unsigned)line_adc[4],
                 (unsigned)sample.red, (unsigned)sample.green,
                 (unsigned)sample.blue, (unsigned)sample.clear,
                 Color_Name(ColorSensor_Detect(&sample)));
  Uart_Print(line);
  HAL_Delay(300U);
}
