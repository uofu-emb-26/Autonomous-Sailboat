#include "cmsis_os2.h"
#include "main.h"
#include "sensorWind.h"

#include "stm32h755xx.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_gpio_ex.h"
#include "stm32h7xx_hal_uart.h"
#include <stdint.h>

#define TASK_NAME "SensorWindTask"
#define TASK_STACK_SIZE 128
#define TASK_PRIORITY osPriorityAboveNormal

TaskHandle_t task_sensorWind;

void sensorWind_uart4Init();
void sensorWind_handler(void *argument);
UART_HandleTypeDef UART4_Handler = {0};


/**
  * Initialize the hardware.
  */
void sensorWind_hardwareInit()
{
  // UART4 initialization for wind sensor communication
  sensorWind_uart4Init();
}

void sensorWind_uart4Init(void) {
  // Setting up PA0 (UART4 TX)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull =  GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Changing to PA1 (UART4 RX)
  GPIO_InitStruct.Pin = GPIO_PIN_1; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Setting USART setting
  UART_InitTypeDef UART_InitStruct = {0};
  UART_InitStruct.BaudRate = 9600;
  UART_InitStruct.WordLength = UART_WORDLENGTH_8B;
  UART_InitStruct.StopBits = UART_STOPBITS_1;
  UART_InitStruct.Parity = UART_PARITY_NONE;
  UART_InitStruct.Mode = UART_MODE_TX_RX;

  UART4_Handler.Init = UART_InitStruct;
  UART4_Handler.Instance = UART4;

  HAL_UART_Init(&UART4_Handler);
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
  uint8_t txBuffer[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39}; // protocol for asking windvane for wind direction in degrees
  uint8_t angleBuffer[7];
  uint16_t rawAngle;
  for(;;)
  {
    if (HAL_UART_Transmit(&UART4_Handler, txBuffer, 8, 1000) != HAL_OK) {
      printf("HAL transmit failed");
    } 
    if (HAL_UART_Receive(&UART4_Handler, angleBuffer, 7, 1000) != HAL_OK) {
      printf("HAL receivefailed");
    }
    rawAngle = (uint16_t)(((angleBuffer[3] << 8) | angleBuffer[4]) / 10); 
    servoSail_setAngle(rawAngle);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
  }
}

