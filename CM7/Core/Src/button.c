#include "main.h"
#include "button.h"
#include "sensorMagnetometer.h"
#include "sensorWind.h"
#include "servoRudder.h"
#include "servoSail.h"

TaskHandle_t task_button;
SemaphoreHandle_t semphr_button;
static controlMode_t currentControlMode = CONTROL_MODE_SERVO_SAIL;

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

void button_activateControlMode(controlMode_t mode)
{
  TaskHandle_t taskToResume = NULL;

  currentControlMode = mode;

  vTaskSuspend(task_servoSail);
  vTaskSuspend(task_servoRudder);
  vTaskSuspend(task_sensorWind);
  vTaskSuspend(task_sensorMagnetometer);

  switch (mode)
  {
    case CONTROL_MODE_SERVO_SAIL:
      taskToResume = task_servoSail;
      break;

    case CONTROL_MODE_SERVO_RUDDER:
      taskToResume = task_servoRudder;
      break;

    case CONTROL_MODE_SENSOR_WIND:
      taskToResume = task_sensorWind;
      break;

    case CONTROL_MODE_MAG_YAW:
      sensorMagnetometer_setMode(SENSOR_MAG_MODE_YAW);
      taskToResume = task_sensorMagnetometer;
      break;

    case CONTROL_MODE_MAG_PITCH:
      sensorMagnetometer_setMode(SENSOR_MAG_MODE_PITCH);
      taskToResume = task_sensorMagnetometer;
      break;

    case CONTROL_MODE_MAG_ROLL:
      sensorMagnetometer_setMode(SENSOR_MAG_MODE_ROLL);
      taskToResume = task_sensorMagnetometer;
      break;

    default:
      break;
  }

  printf("Active control mode: %s\r\n", button_modeName(mode));

  if (taskToResume != NULL)
  {
    vTaskResume(taskToResume);
  }
}

void button_activateNextControlMode(void)
{
  controlMode_t nextMode = (controlMode_t)((currentControlMode + 1) % CONTROL_MODE_COUNT);
  button_activateControlMode(nextMode);
}

/**
  * Initialize the hardware.
  */
void button_hardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * Handler for the task.
  */
void button_handler(void *argument)
{
  for(;;)
  {
    if (xSemaphoreTake(semphr_button, portMAX_DELAY) == pdTRUE)
    {
      printf("Button pressed!\r\n");
      button_activateNextControlMode();
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
      vTaskDelay(1000 * portTICK_PERIOD_MS);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
  }
}
