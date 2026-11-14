#include "main.h"

#include <stdio.h>
#include <string.h>

/* TCS34725/TCS34727: 7-bit I2C address 0x29. HAL expects it shifted left. */
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
#define LINE_SENSOR_COUNT    5U
#define LINE_TIMEOUT_MS      10U

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
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

/* These thresholds are only starting points for red/blue cargo testing. */
static const char *Color_Name(uint16_t red, uint16_t green,
                              uint16_t blue, uint16_t clear)
{
  if (clear < 100U || clear > 40000U)
  {
    return "NONE";
  }
  if ((uint32_t)red * 100U > (uint32_t)green * 135U &&
      (uint32_t)red * 100U > (uint32_t)blue * 135U)
  {
    return "RED";
  }
  if ((uint32_t)blue * 100U > (uint32_t)red * 140U &&
      (uint32_t)blue * 100U > (uint32_t)green * 115U)
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

  if (HAL_I2C_IsDeviceReady(&hi2c1, TCS_ADDR, 2U, I2C_TIMEOUT_MS) != HAL_OK)
  {
    Uart_Print("TCS3472: I2C device 0x29 not found\r\n");
    return;
  }
  if (Tcs_Read(TCS_REG_ID, &id) != HAL_OK)
  {
    Uart_Print("TCS3472: failed to read ID\r\n");
    return;
  }
  if (id != 0x44U && id != 0x4DU)
  {
    (void)snprintf(line, sizeof(line),
                   "TCS3472: ID=0x%02X, expected 0x44 or 0x4D\r\n", id);
    Uart_Print(line);
    return;
  }

  /* 0xD5 = 42 * 2.4 ms (~101 ms); CONTROL=1 gives 4x gain. */
  if (Tcs_Write(TCS_REG_ATIME, 0xD5U) != HAL_OK ||
      Tcs_Write(TCS_REG_CONTROL, 0x01U) != HAL_OK ||
      Tcs_Write(TCS_REG_ENABLE, TCS_PON) != HAL_OK)
  {
    Uart_Print("TCS3472: configuration failed\r\n");
    return;
  }
  HAL_Delay(3U); /* Oscillator start after PON. */
  if (Tcs_Write(TCS_REG_ENABLE, TCS_PON | TCS_AEN) != HAL_OK)
  {
    Uart_Print("TCS3472: failed to enable color measurement\r\n");
    return;
  }

  HAL_Delay(110U); /* Wait for the first complete integration. */
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
  uint8_t status = 0U;
  uint8_t raw[8];
  uint16_t line_adc[5];
  uint16_t clear, red, green, blue;
  char line[112];

  /* Heartbeat: if the LED toggles, the application reached its main loop. */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  if (sensor_ready == 0U)
  {
    HAL_Delay(1000U);
    TestColor_Init();
    return;
  }

  if (Tcs_Read(TCS_REG_STATUS, &status) != HAL_OK)
  {
    Uart_Print("TCS3472: failed to read STATUS\r\n");
    sensor_ready = 0U;
    return;
  }
  if ((status & TCS_AVALID) == 0U)
  {
    Uart_Print("TCS3472: waiting for valid color data\r\n");
    HAL_Delay(100U);
    return;
  }

  /* One auto-increment transfer reads C, R, G, B as little-endian words. */
  if (HAL_I2C_Mem_Read(&hi2c1, TCS_ADDR,
                       TCS_CMD | TCS_AUTO_INCREMENT | TCS_REG_CDATA,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                       I2C_TIMEOUT_MS) != HAL_OK)
  {
    Uart_Print("TCS3472: failed to read RGBC\r\n");
    sensor_ready = 0U;
    return;
  }

  clear = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
  red   = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
  green = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
  blue  = (uint16_t)raw[6] | ((uint16_t)raw[7] << 8);

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
                 (unsigned)red, (unsigned)green, (unsigned)blue,
                 (unsigned)clear, Color_Name(red, green, blue, clear));
  Uart_Print(line);
  HAL_Delay(300U);
}
