#include "calibration.h"
#include "main.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CAL_PAGE_A 0x0800F800UL
#define CAL_PAGE_B 0x0800FC00UL
#define CAL_FIRMWARE_END CAL_PAGE_A
#define CAL_MAGIC 0x4C494E45UL
#define CAL_FORMAT_VERSION 1UL
#define CAL_COMMITTED 0xA55AU
#define CAL_DURATION_MS 20000U
#define CAL_SAMPLE_INTERVAL_MS 2U
#define CAL_LED_INTERVAL_MS 250U
#define CAL_PROGRESS_INTERVAL_MS 1000U
#define CAL_GROUP_SAMPLES 10U

typedef struct
{
  uint32_t magic;
  uint32_t formatVersion;
  uint32_t firmwareCrc;
  uint32_t sequence;
  uint16_t white[LINE_SENSOR_COUNT];
  uint16_t black[LINE_SENSOR_COUNT];
  uint32_t checksum;
  uint16_t committed;
} CalibrationRecord;

_Static_assert(offsetof(CalibrationRecord, committed) % 2U == 0U,
               "Flash record must use halfwords");
_Static_assert(sizeof(CalibrationRecord) <= 1024U,
               "Calibration record must fit in one Flash page");

static uint32_t bootResetFlags;
static uint32_t firmwareCrc;
static uint32_t startMs;
static uint32_t nextSampleMs;
static uint32_t lastLedMs;
static uint32_t lastProgressMs;
static uint16_t groupMin[LINE_SENSOR_COUNT];
static uint16_t groupMax[LINE_SENSOR_COUNT];
static uint16_t calLow[LINE_SENSOR_COUNT];
static uint16_t calHigh[LINE_SENSOR_COUNT];
static uint8_t groupCount;
static uint16_t completedGroups;
static CalibrationStatus status;

extern UART_HandleTypeDef huart3;

static void Calibration_PrintValues(const char *label)
{
  uint32_t validSensors = 0U;
  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    if (calLow[index] <= 4095U && calHigh[index] <= 4095U &&
        calHigh[index] >= calLow[index] + 20U)
    {
      ++validSensors;
    }
  }
  char message[128];
  int length = snprintf(message, sizeof(message),
                        "CAL %s OK=%lu/5 WHITE=%u,%u,%u,%u,%u BLACK=%u,%u,%u,%u,%u\r\n",
                        label, (unsigned long)validSensors,
                        (unsigned)calLow[0], (unsigned)calLow[1],
                        (unsigned)calLow[2], (unsigned)calLow[3],
                        (unsigned)calLow[4], (unsigned)calHigh[0],
                        (unsigned)calHigh[1], (unsigned)calHigh[2],
                        (unsigned)calHigh[3], (unsigned)calHigh[4]);
  if (length > 0 && length < (int)sizeof(message))
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)message,
                            (uint16_t)length, 20U);
  }
}

static uint32_t Calibration_Crc32(const uint8_t *bytes, size_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t index = 0U; index < length; ++index)
  {
    crc ^= bytes[index];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
    }
  }
  return ~crc;
}

static bool Calibration_ValuesValid(const CalibrationRecord *record)
{
  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    if (record->white[index] > 4095U || record->black[index] > 4095U ||
        record->black[index] < record->white[index] + 20U)
    {
      return false;
    }
  }
  return true;
}

static bool Calibration_RecordValid(const CalibrationRecord *record)
{
  return record->committed == CAL_COMMITTED &&
         record->magic == CAL_MAGIC &&
         record->formatVersion == CAL_FORMAT_VERSION &&
         record->checksum == Calibration_Crc32((const uint8_t *)record,
                                               offsetof(CalibrationRecord,
                                                        checksum)) &&
         Calibration_ValuesValid(record);
}

static const CalibrationRecord *Calibration_LatestRecord(void)
{
  const CalibrationRecord *a = (const CalibrationRecord *)CAL_PAGE_A;
  const CalibrationRecord *b = (const CalibrationRecord *)CAL_PAGE_B;
  bool aValid = Calibration_RecordValid(a);
  bool bValid = Calibration_RecordValid(b);
  if (aValid && bValid)
  {
    return (int32_t)(a->sequence - b->sequence) > 0 ? a : b;
  }
  return aValid ? a : (bValid ? b : NULL);
}

static bool Calibration_Save(void)
{
  const CalibrationRecord *previous = Calibration_LatestRecord();
  uint32_t targetAddress = previous == (const CalibrationRecord *)CAL_PAGE_A ?
                           CAL_PAGE_B : CAL_PAGE_A;
  CalibrationRecord record = {0};
  record.magic = CAL_MAGIC;
  record.formatVersion = CAL_FORMAT_VERSION;
  record.firmwareCrc = firmwareCrc;
  record.sequence = previous == NULL ? 1U : previous->sequence + 1U;
  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    record.white[index] = calLow[index];
    record.black[index] = calHigh[index];
  }
  record.checksum = Calibration_Crc32((const uint8_t *)&record,
                                      offsetof(CalibrationRecord, checksum));
  record.committed = CAL_COMMITTED;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }
  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = targetAddress;
  erase.NbPages = 1U;
  uint32_t pageError = 0U;
  bool saved = HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;
  const uint8_t *bytes = (const uint8_t *)&record;
  for (uint32_t offset = 0U;
       saved && offset < offsetof(CalibrationRecord, committed);
       offset += 2U)
  {
    uint16_t halfword = (uint16_t)bytes[offset] |
                        ((uint16_t)bytes[offset + 1U] << 8U);
    saved = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              targetAddress + offset, halfword) == HAL_OK;
  }
  if (saved)
  {
    saved = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              targetAddress + offsetof(CalibrationRecord,
                                                        committed),
                              CAL_COMMITTED) == HAL_OK;
  }
  (void)HAL_FLASH_Lock();
  return saved && Calibration_RecordValid(
                    (const CalibrationRecord *)targetAddress) &&
         ((const CalibrationRecord *)targetAddress)->firmwareCrc == firmwareCrc;
}

static void Calibration_ResetGroup(void)
{
  groupCount = 0U;
  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    groupMin[index] = UINT16_MAX;
    groupMax[index] = 0U;
  }
}

void Calibration_CaptureBootReason(void)
{
  bootResetFlags = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint32_t Calibration_GetBootResetFlags(void)
{
  return bootResetFlags;
}

bool Calibration_Init(void)
{
  firmwareCrc = Calibration_Crc32((const uint8_t *)FLASH_BASE,
                                   CAL_FIRMWARE_END - FLASH_BASE);
  const CalibrationRecord *record = Calibration_LatestRecord();
  bool powerReset = (bootResetFlags & RCC_CSR_PORRSTF) != 0U;
  bool pinReset = (bootResetFlags & RCC_CSR_PINRSTF) != 0U;
  bool softwareReset = (bootResetFlags & RCC_CSR_SFTRSTF) != 0U;
  bool watchdogReset = (bootResetFlags &
                        (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0U;

  /* POR takes priority because PINRSTF may also be set at power-on. */
  if (record != NULL && record->firmwareCrc == firmwareCrc &&
      !(pinReset && !powerReset && !softwareReset && !watchdogReset) &&
      LineError_SetCalibration(record->white, record->black))
  {
    status = CALIBRATION_SAVED;
    return true;
  }

  status = CALIBRATION_IN_PROGRESS;
  startMs = HAL_GetTick();
  nextSampleMs = startMs;
  lastLedMs = startMs;
  lastProgressMs = startMs;
  completedGroups = 0U;
  Calibration_ResetGroup();
  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    calLow[index] = UINT16_MAX;
    calHigh[index] = 0U;
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  return false;
}

CalibrationStatus Calibration_Task(void)
{
  if (status != CALIBRATION_IN_PROGRESS)
  {
    return status;
  }

  uint32_t nowMs = HAL_GetTick();
  if (nowMs - lastLedMs >= CAL_LED_INTERVAL_MS)
  {
    lastLedMs = nowMs;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  if (nowMs - startMs >= CAL_DURATION_MS)
  {
    CalibrationRecord candidate = {0};
    for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
    {
      candidate.white[index] = calLow[index];
      candidate.black[index] = calHigh[index];
    }
    if (completedGroups == 0U || !Calibration_ValuesValid(&candidate))
    {
      status = CALIBRATION_FAILED;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      Calibration_PrintValues("FAILED: SPAN < 20");
      return status;
    }
    if (!Calibration_Save() || !LineError_SetCalibration(calLow, calHigh))
    {
      status = CALIBRATION_FAILED;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      Calibration_PrintValues("FAILED: FLASH SAVE");
      return status;
    }
    status = CALIBRATION_SAVED;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    Calibration_PrintValues("DONE");
    return status;
  }
  if (nowMs - lastProgressMs >= CAL_PROGRESS_INTERVAL_MS)
  {
    lastProgressMs = nowMs;
    char label[16];
    (void)snprintf(label, sizeof(label), "%lus/%lus",
                   (unsigned long)((nowMs - startMs) / 1000U),
                   (unsigned long)(CAL_DURATION_MS / 1000U));
    Calibration_PrintValues(label);
  }
  if ((int32_t)(nowMs - nextSampleMs) < 0)
  {
    return status;
  }
  nextSampleMs = nowMs + CAL_SAMPLE_INTERVAL_MS;

  uint16_t values[LINE_SENSOR_COUNT];
  if (!LineSensor_Read(values))
  {
    Calibration_ResetGroup();
    return status;
  }
  for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
  {
    if (values[index] < groupMin[index])
    {
      groupMin[index] = values[index];
    }
    if (values[index] > groupMax[index])
    {
      groupMax[index] = values[index];
    }
  }
  if (++groupCount == CAL_GROUP_SAMPLES)
  {
    for (uint32_t index = 0U; index < LINE_SENSOR_COUNT; ++index)
    {
      if (groupMax[index] < calLow[index])
      {
        calLow[index] = groupMax[index];
      }
      if (groupMin[index] > calHigh[index])
      {
        calHigh[index] = groupMin[index];
      }
    }
    ++completedGroups;
    Calibration_ResetGroup();
  }
  return status;
}

