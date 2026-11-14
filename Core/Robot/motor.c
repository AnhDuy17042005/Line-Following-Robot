#include "main.h"
#include "robot_internal.h"

#define ENCODER_COUNTS_PER_REVOLUTION 1320.0f
/* M1 is on AO2/BO2; right encoder C1 is on TIM3 CH2. */
/* Forward must move both wheels forward and report positive encoder RPM. */
#define LEFT_FORWARD_IN1_STATE GPIO_PIN_SET
#define RIGHT_FORWARD_IN1_STATE GPIO_PIN_RESET
#define LEFT_ENCODER_FORWARD_SIGN 1.0f
#define RIGHT_ENCODER_FORWARD_SIGN 1.0f

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static uint16_t previousLeftCount;
static uint16_t previousRightCount;
static int32_t leftTravelCounts;
static int32_t rightTravelCounts;

bool Motor_Init(void)
{
  Motor_Stop();
  if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
    Motor_Stop();
    return false;
  }

  Motor_ResetDistance();
  return true;
}

void Motor_Stop(void)
{
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin | BIN1_Pin | BIN2_Pin,
                    GPIO_PIN_RESET);
}

void Motor_ReadEncoderCounts(uint16_t *leftCount, uint16_t *rightCount)
{
  *leftCount = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
  *rightCount = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void Motor_ResetDistance(void)
{
  Motor_ReadEncoderCounts(&previousLeftCount, &previousRightCount);
  leftTravelCounts = 0;
  rightTravelCounts = 0;
}

float Motor_GetDistanceMm(void)
{
  const float mmPerCount = 3.14159265f * WHEEL_DIAMETER_M * 1000.0f /
                           ENCODER_COUNTS_PER_REVOLUTION;
  return 0.5f * (float)(leftTravelCounts + rightTravelCounts) * mmPerCount;
}

void Motor_GetTravelCounts(int32_t *leftCounts, int32_t *rightCounts)
{
  *leftCounts = leftTravelCounts;
  *rightCounts = rightTravelCounts;
}

void Motor_GetDriveState(uint32_t *leftPwmPercent,
                         uint32_t *rightPwmPercent, bool *standbyEnabled)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  *leftPwmPercent = 100U * __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4) /
                    period;
  *rightPwmPercent = 100U * __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3) /
                     period;
  *standbyEnabled = HAL_GPIO_ReadPin(STBY_GPIO_Port, STBY_Pin) == GPIO_PIN_SET;
}

void Motor_ReadSpeed(float periodSeconds, float *leftRpm, float *rightRpm)
{
  uint16_t leftCount;
  uint16_t rightCount;
  Motor_ReadEncoderCounts(&leftCount, &rightCount);
  int16_t leftDelta = (int16_t)(leftCount - previousLeftCount);
  int16_t rightDelta = (int16_t)(rightCount - previousRightCount);
  float rpmPerCount = 60.0f / (ENCODER_COUNTS_PER_REVOLUTION * periodSeconds);

  previousLeftCount = leftCount;
  previousRightCount = rightCount;
  leftTravelCounts += (int32_t)((float)leftDelta * LEFT_ENCODER_FORWARD_SIGN);
  rightTravelCounts += (int32_t)((float)rightDelta * RIGHT_ENCODER_FORWARD_SIGN);
  *leftRpm = (float)leftDelta * rpmPerCount * LEFT_ENCODER_FORWARD_SIGN;
  *rightRpm = (float)rightDelta * rpmPerCount * RIGHT_ENCODER_FORWARD_SIGN;
}

void Motor_SetDuty(float leftDutyPercent, float rightDutyPercent)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  uint32_t leftPulse = (uint32_t)(period * leftDutyPercent / 100.0f + 0.5f);
  uint32_t rightPulse = (uint32_t)(period * rightDutyPercent / 100.0f + 0.5f);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, leftPulse);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, rightPulse);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, LEFT_FORWARD_IN1_STATE);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin,
                    LEFT_FORWARD_IN1_STATE == GPIO_PIN_SET ?
                    GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, RIGHT_FORWARD_IN1_STATE);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,
                    RIGHT_FORWARD_IN1_STATE == GPIO_PIN_SET ?
                    GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
}
