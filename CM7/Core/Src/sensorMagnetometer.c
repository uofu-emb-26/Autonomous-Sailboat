

// #include "cmsis_os2.h"
#include "main.h"
// #include "servoSail.h"

#define TASK_NAME "magnetometerTask"
#define TASK_STACK_SIZE 128
#define TASK_PRIORITY osPriorityAboveNormal
#define BNO055_ADDR  0x29  // default, COM3 high Try 0x28 if this doesn't work
#define BNO055_WHO_AM_I 0x00  // chip ID register

TaskHandle_t task_sensorGPS

void magnetometer_handle(void *argument);
I2C_HandleTypeDef I2C_BNO055_Handle;

/**
  * Initialize the hardware.
  */
void magnometer_hardwareInit()
{
    // Page 65 of the chip datasheet says pf0 and pf1 are I2c_SDA and I2c_SCL
    // added  __HAL_RCC_GPIOF_CLK_ENABLE(); to the main.c
    // added __HAL_RCC_I2C2_CLK_ENABLE(); to the main.c
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // using PF0 for I2C2_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // using PF1 for I2C2_SCL
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);                      line 601 of Stm32h7xx_hal_i2c.h
    // I2C_TypeDef                *Instance;      /*!< I2C registers base address    line 186 of Stm32h7xx_hal_i2c.h
    // I2C_InitTypeDef            Init;           /*!< I2C communication parameters  line 187 of Stm32h7xx_hal_i2c.h
    I2C_BNO055_Handle.Instance = I2C2; 
    // The board's clock rate is 64MHz accoding to servoSail.c

    // 0x60201E28 for 100khz I2C signal

    // PRESC = 6: Divides 64MHz down to a manageable internal frequency.

    // SCLH/SCLL: Set to create a symmetrical 100kHz square wave.
    // (1 + 1)/ 64MHZ = 31.25 ns t_{prescaler} - so prescaler is 0x1

    // SCLDEL = 7, 31.25 * 7 = 250ns of set up time

    // t_{sclh} - 125 (really is 126) - 

    // t_{scll + 1} - 188 (really is 189) - 

    // t_{SCLH} = t_{sclh + 1} * t_{prescaler} = 3937.5 ns

    // t_{SCLL} = t_{scll + 1} * t_{prescaler} = 5906.25 ns

    // 3937.5 + 5906.25 = 9843.75 - The period

    // freq = 1/ period = 101khz-ish ~= basically 100khz (really is 101khz) we can do more specific math later

    // The Total = t_{SCLL} + t_{SCLH} high and low durations
    I2C_BNO055_Handle.Init.Timing = 0x107DBC; // <- need to see if this is correct.
    // (1+1)/64MHZ = 31.25 NS
    // 
    I2C_BNO055_Handle.Init.I2C_ADDRESSINGMODE_7BIT
    I2C_BNO055_Handle.Init.OwnAddress1 = 0;
    I2C_BNO055_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_BNO055_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_BNO055_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&I2C_BNO055_Handle);
    // then we need to do the write transaciton
    uint8_t reg = BNO055_WHO_AM_I; // Pretty sure this is right
    uint8_t chip_id = 0;
    // the chip is 0xA0  on https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
    // HAL_StatusTypeDef HAL_I2C_Master_Receive and HAL_I2C_Master_Receive parameters:
    // I2C_HandleTypeDef *hi2c, - I2C_BNO055_Handle,
    // uint16_t DevAddress, - BNO055_ADDR The header above,
    // uint8_t *pData, - &chip_id,
    // uint16_t Size, - The amount of bytes we are reading,
    // uint32_t Timeout); - The delay,
    HAL_I2C_Master_Transmit(&I2C_BNO055_Handle, BNO055_ADDR << 1, &reg, 1, HAL_MAX_DELAY);

    HAL_I2C_Master_Receive(&I2C_BNO055_Handle, BNO055_ADDR << 1, &chip_id, 1, HAL_MAX_DELAY);
    // we right shift by 1 cause we want to write/read data to there, 8 bit address total
    if(chip_id == 0xA0) // The value we read: uint8_t reg = BNO055_WHO_AM_I;
    {
        // we know we're talking to the sensor once this is true
        button_handler();
    }
}

// 0xAA is the start byte
