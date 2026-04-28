#ifndef SENSOR_WIND_H
#define SENSOR_WIND_H

#include "FreeRTOS.h"
#include "task.h" 
#include <stdint.h>
#include "cmsis_os2.h"  

#define SENSOR_ADDRESS 0x02  // Modbus address configured on the wind sensor.

extern TaskHandle_t task_sensorWind;

/**
 * @brief Configure the UART interface used by the wind sensor.
 */
void  sensorWind_hardwareInit(void);

/**
 * @brief FreeRTOS task that reads wind data when wind-sensor mode is active.
 * @param argument Unused RTOS task argument.
 */
void  sensorWind_handler(void *argument);

/**
 * @brief Read the 0-360 degree wind angle from the sensor over Modbus.
 * @param addr Modbus address of the wind sensor.
 * @return Angle in degrees with 0.1 degree resolution, or `-1.0f` on error.
 */
float read_wind_angle_360(uint8_t addr);

#endif
