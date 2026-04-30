extern TaskHandle_t task_sensorMagnetometer;

/**
 * @brief Configure I2C2 and initialize the BNO055 magnetometer/IMU.
 */
void sensorMagnetometer_hardwareInit();

/**
 * @brief FreeRTOS task that reads orientation data when a magnetometer mode is active.
 * @param argument Unused RTOS task argument.
 */
void sensorMagnetometer_handler(void *argument);