#ifndef SENSOR_WIND_H
#define SENSOR_WIND_H

#include "FreeRTOS.h"
#include "task.h" 
#include <stdint.h>
#include "cmsis_os2.h"  

#define SENSOR_ADDRESS 0x02  // defined once here, remove from .c

extern TaskHandle_t task_sensorWind;

void  sensorWind_hardwareInit(void);
void  sensorWind_handler(void *argument);
float read_wind_angle_360(uint8_t addr); // public — used by servoSail.c

#endif