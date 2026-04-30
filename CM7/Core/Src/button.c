#include "main.h"
#include "button.h"
#include "sensorMagnetometer.h"
#include "sensorWind.h"
#include "servoRudder.h"
#include "servoSail.h"

SemaphoreHandle_t semphr_button;
TaskHandle_t task_button = NULL;

/**
  * @brief Configure the user button interrupt input and its status LED output.
  */
void button_hardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // PB14 drives the LED that acknowledges a mode change.
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PC13 is the pushbutton input and generates an interrupt on the rising edge.
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief Wait for button events and cycle to the next control mode.
  * @param argument Unused RTOS task argument.
  */
void button_handler(void *argument)
{
  for(;;)
  {
    if (xSemaphoreTake(semphr_button, portMAX_DELAY) == pdTRUE)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
      // Hold the LED on briefly so the mode change is visible on the board.
      vTaskDelay(pdMS_TO_TICKS(1000));
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
  }
}
