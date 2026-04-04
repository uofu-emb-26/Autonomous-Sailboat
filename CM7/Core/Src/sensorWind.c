
#include "cmsis_os2.h"
#include "main.h"
#include "sensorWind.h"

#include "stm32h755xx.h"
#include "stm32h7xx_hal_uart.h"

#define TASK_NAME "SensorWindTask"
#define TASK_STACK_SIZE 128
#define TASK_PRIORITY osPriorityAboveNormal

TaskHandle_t task_sensorWind;

void sensorWind_usart3Init();
void sensorWind_handler(void *argument);

/**
  * Initialize the hardware.
  */
void sensorWind_hardwareInit()
{
  // USART3 initialization for wind sensor communication
  sensorWind_usart3Init();
}

void sensorWind_usart3Init(void) {
  // Setting up PB10 (USART3_TX)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull =  GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3; // have to change bit mask if not right
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Changing to PB11 (USART3_RX)
  GPIO_InitStruct.Pin = GPIO_PIN_11; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Setting USART setting
  UART_InitTypeDef UART_InitStruct = {0};
  UART_InitStruct.BaudRate = 96000;
  UART_InitStruct.WordLength = UART_WORDLENGTH_8B;
  UART_InitStruct.StopBits = UART_STOPBITS_1;
  UART_InitStruct.Parity = UART_PARITY_NONE;
  UART_InitStruct.Mode = UART_MODE_TX_RX;

  UART_HandleTypeDef UART3_Handler = {0};
  UART3_Handler.Init = UART_InitStruct;
  UART3_Handler.Instance = USART3;

  HAL_UART_Init(&UART3_Handler);
}

/**
  * Initialize the RTOS components.
  */
void sensorWind_rtosInit()
{
  if (xTaskCreate(sensorWind_handler, TASK_NAME, TASK_STACK_SIZE, NULL, TASK_PRIORITY, &task_sensorWind) != pdPASS) { Error_Handler(); }
}

/**
  * Handler for the task.
  */
void sensorWind_handler(void *argument)
{
  for(;;)
  {
    // Read wind sensor data via USART3 and process it
    // This is a placeholder for the actual sensor reading and processing logic
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
  }
}