#include "main.h"
#include "servoSail.h"
#include "servoRudder.h"

TaskHandle_t task_servoRudder;

/**
  * Initialize the hardware.
  */
void servoRudder_hardwareInit() {}

/**
  * Handler for the task.
  */
void servoRudder_handler(void *argument) {
  for(;;) {
    servoSail_setAngle(-135);
    vTaskDelay(2000);
    servoSail_setAngle(0);
    vTaskDelay(2000);
    servoSail_setAngle(135);
    vTaskDelay(2000);
  }
}