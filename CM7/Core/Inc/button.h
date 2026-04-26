
extern TaskHandle_t task_button;
extern SemaphoreHandle_t semphr_button;

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

void button_hardwareInit();
void button_handler(void *argument);
void button_activateControlMode(controlMode_t mode);
void button_activateNextControlMode(void);
