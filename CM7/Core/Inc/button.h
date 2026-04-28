#ifndef BUTTON_H
#define BUTTON_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern TaskHandle_t task_button;
extern SemaphoreHandle_t semphr_button;

/* Control modes cycled by the user pushbutton. */
typedef enum
{
    CONTROL_MODE_SERVO_SAIL = 0,
    CONTROL_MODE_SERVO_RUDDER,
    CONTROL_MODE_SENSOR_WIND,
    CONTROL_MODE_MAG_YAW,
    CONTROL_MODE_MAG_PITCH,
    CONTROL_MODE_MAG_ROLL,
    CONTROL_MODE_COUNT
} controlMode_t;

/**
 * @brief Configure the pushbutton input and status LED output.
 */
void button_hardwareInit(void);

/**
 * @brief FreeRTOS task that waits for button presses and advances the control mode.
 * @param argument Unused RTOS task argument.
 */
void button_handler(void *argument);

/**
 * @brief Switch the system into a specific control mode.
 * @param mode Control mode to activate.
 */
void button_activateControlMode(controlMode_t mode);

/**
 * @brief Advance to the next control mode in the button cycle.
 */
void button_activateNextControlMode(void);

/**
 * @brief Get the control mode currently selected by the button task.
 * @return The active control mode.
 */
controlMode_t button_getCurrentControlMode(void);

#endif
