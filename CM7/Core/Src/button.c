#include "main.h"
#include "button.h"
#include "sensorMagnetometer.h"
#include "sensorWind.h"
#include "servoRudder.h"
#include "servoSail.h"

TaskHandle_t task_button;
SemaphoreHandle_t semphr_button;
static volatile controlMode_t currentControlMode = CONTROL_MODE_SERVO_SAIL;

/**
  * @brief Convert a control mode enum into the name shown on the debug console.
  * @param mode Control mode to describe.
  * @return Human-readable mode name.
  */
static const char *button_modeName(controlMode_t mode)
{
  switch (mode)
  {
    case CONTROL_MODE_SERVO_SAIL: return "servo sail";
    case CONTROL_MODE_SERVO_RUDDER: return "servo rudder";
    case CONTROL_MODE_SENSOR_WIND: return "sensor wind";
    case CONTROL_MODE_MAG_YAW: return "magnetometer yaw";
    case CONTROL_MODE_MAG_PITCH: return "magnetometer pitch";
    case CONTROL_MODE_MAG_ROLL: return "magnetometer roll";
    default: return "unknown";
  }
}

/**
  * @brief Return the control mode currently selected by the button task.
  * @return The active control mode.
  */
controlMode_t button_getCurrentControlMode(void)
{
  return currentControlMode;
}

/**
  * @brief Activate a specific control mode and update any dependent subsystems.
  * @param mode Control mode to activate.
  */
void button_activateControlMode(controlMode_t mode)
{
  currentControlMode = mode;

  switch (mode)
  {
    case CONTROL_MODE_SERVO_SAIL:
      printf("MODE: Servo Sail\r\n");
      break;

    case CONTROL_MODE_SERVO_RUDDER:
      printf("MODE: Servo Rudder\r\n");
      break;

    case CONTROL_MODE_SENSOR_WIND:
      printf("MODE: Sensor Wind\r\n");
      break;

    case CONTROL_MODE_MAG_YAW:
      printf("MODE: Magnetometer Yaw\r\n");
      sensorMagnetometer_setMode(SENSOR_MAG_MODE_YAW);
      break;

    case CONTROL_MODE_MAG_PITCH:
      printf("MODE: Magnetometer Pitch\r\n");
      sensorMagnetometer_setMode(SENSOR_MAG_MODE_PITCH);
      break;

    case CONTROL_MODE_MAG_ROLL:
      printf("MODE: Magnetometer Roll\r\n");
      sensorMagnetometer_setMode(SENSOR_MAG_MODE_ROLL);
      break;

    default:
      break;
  }

  printf("Active control mode: %s\r\n", button_modeName(mode));
}

/**
  * @brief Advance to the next control mode in the button rotation.
  */
void button_activateNextControlMode(void)
{
  controlMode_t nextMode = (controlMode_t)((currentControlMode + 1) % CONTROL_MODE_COUNT);
  button_activateControlMode(nextMode);
}

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
      button_activateNextControlMode();
      // Hold the LED on briefly so the mode change is visible on the board.
      vTaskDelay(pdMS_TO_TICKS(1000));
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
  }
}
