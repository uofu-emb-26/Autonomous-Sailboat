#ifndef SERVO_SAIL_H
#define SERVO_SAIL_H

#include <stdint.h>
#include "cmsis_os2.h"

extern TaskHandle_t task_servoSail;

void servoSail_hardwareInit(void);
void servoSail_handler(void *argument);
void servoSail_setAngle(int angle);

#endif