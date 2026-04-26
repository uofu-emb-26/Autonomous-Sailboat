
extern TaskHandle_t task_sensorMagnetometer;

typedef enum
{
    SENSOR_MAG_MODE_YAW = 0,
    SENSOR_MAG_MODE_PITCH,
    SENSOR_MAG_MODE_ROLL
} sensorMagnetometerMode_t;

void sensorMagnetometer_hardwareInit();
void sensorMagnetometer_handler(void *argument);
void sensorMagnetometer_setMode(sensorMagnetometerMode_t mode);
sensorMagnetometerMode_t sensorMagnetometer_getMode(void);
