#include "main.h"
#include "button.h"
#include "servoSail.h"
#include "servoRudder.h"

#define SERVO_RUDDER_IDLE_PERIOD_MS 20

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
    vTaskDelay(pdMS_TO_TICKS(2000));

    servoSail_setAngle(0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    servoSail_setAngle(135);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
