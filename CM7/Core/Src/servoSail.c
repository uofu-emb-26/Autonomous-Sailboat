
#include "cmsis_os2.h"
#include "main.h"
#include "servoSail.h"

#define TASK_NAME "ServoSailTask"
#define TASK_STACK_SIZE 128
#define TASK_PRIORITY osPriorityAboveNormal

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

TaskHandle_t task_servoSail;
TIM_HandleTypeDef servo_tim1;

void servoSail_handler(void *argument);

/**
  * Initialize the hardware.
  */
void servoSail_hardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull =  GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // The boards clock rate is 240MHz, and we want a 50Hz signal, 
  // so we set the prescaler to 239 (240MHz/240 = 1MHz) and the 
  // autoreload to 19999 (1MHz/20,000 = 50Hz)
  __HAL_TIM_SET_PRESCALER(&servo_tim1, 239);
  __HAL_TIM_SET_AUTORELOAD(&servo_tim1, 19999);
  HAL_TIM_PWM_Start(&servo_tim1, TIM_CHANNEL_1);
}

/**
  * Initialize the RTOS components.
  */
void servoSail_rtosInit()
{
  if (xTaskCreate(servoSail_handler, TASK_NAME, TASK_STACK_SIZE, NULL, TASK_PRIORITY, &task_servoSail) != pdPASS) { Error_Handler(); }
}

/**
  * Handler for the task.
  */
void servoSail_handler(void *argument)
{
  for(;;)
  {
    servoSail_setAngle(90);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servoSail_setAngle(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servoSail_setAngle(180);
    vTaskDelay(pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
    vTaskDelay(100 * portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  }
}

void servoSail_setAngle(uint16_t angle)
{
    uint16_t pulse_length = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    __HAL_TIM_SET_COMPARE(&servo_tim1, TIM_CHANNEL_1, pulse_length);
}