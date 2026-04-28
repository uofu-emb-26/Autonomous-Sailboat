
extern TaskHandle_t task_sensorMagnetometer;

/* Output axis selected for the magnetometer task. */
typedef enum
{
    SENSOR_MAG_MODE_YAW = 0,
    SENSOR_MAG_MODE_PITCH,
    SENSOR_MAG_MODE_ROLL
} sensorMagnetometerMode_t;

/**
 * @brief Configure I2C2 and initialize the BNO055 magnetometer/IMU.
 */
void sensorMagnetometer_hardwareInit();

/**
 * @brief FreeRTOS task that reads orientation data when a magnetometer mode is active.
 * @param argument Unused RTOS task argument.
 */
void sensorMagnetometer_handler(void *argument);

/**
 * @brief Select which Euler axis the magnetometer task reports.
 * @param mode Axis mode to use for subsequent task updates.
 */
void sensorMagnetometer_setMode(sensorMagnetometerMode_t mode);

/**
 * @brief Get the Euler axis currently selected for the magnetometer task.
 * @return The active magnetometer mode.
 */
sensorMagnetometerMode_t sensorMagnetometer_getMode(void);
