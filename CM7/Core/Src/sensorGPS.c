

// #include "cmsis_os2.h"
#include "main.h"
// #include "servoSail.h"

#define TASK_NAME "magnetometerTask"
#define TASK_STACK_SIZE 128
#define TASK_PRIORITY osPriorityAboveNormal

TaskHandle_t task_sensorGPS

void magnetometer_handle(void *argument);

/**
  * Initialize the hardware.
  */
void magnometer_hardwareInit()
{
    // Page 65 of the chip datasheet says pf0 and pf1 are I2c_SDA and I2c_SCL
    // added  __HAL_RCC_GPIOF_CLK_ENABLE(); to the main.c
    // added __HAL_RCC_I2C2_CLK_ENABLE(); to the main.c
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}