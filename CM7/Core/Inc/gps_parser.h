#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"



extern GPS_Data_t myGPS;

typedef struct{
  double latitude;
  double longitude;
  float altitude;
  uint8_t satellites;
  uint8_t fix_valid; // There are 3 types
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} GPS_Data_t;

void GPS_ProcessChar(uint8_t rx_byte);
void GPS_Parse_GGA(char* nmea_str, GPS_Data_t* gps_struct); // (Global Positioning System Fix Data)
// void GPS_Parse_GLL(char* nmea_str, GPS_Data_t* gps_struct); not sure if we need GLL
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif