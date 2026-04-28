#ifndef SERVO_SAIL_H
#define SERVO_SAIL_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

extern TaskHandle_t task_servoSail;

/**
 * @brief Configure the PWM output used to drive the sail servo.
 */
void servoSail_hardwareInit(void);

/**
 * @brief FreeRTOS task that updates the sail servo when sail control mode is active.
 * @param argument Unused RTOS task argument.
 */
void servoSail_handler(void *argument);

/**
 * @brief Command the sail servo to a target angle in degrees.
 * @param angle Desired servo angle. Values outside the supported range are clamped.
 */
void servoSail_setAngle(int16_t angle);

#endif
