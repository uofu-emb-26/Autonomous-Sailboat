#include "main.h"
#include "sensorMagnetometer.h"

#define BNO055_ADDR  0x29  // default, COM3 high Try 0x28 if this doesn't work
#define BNO055_WHO_AM_I 0x00  // chip ID register

TaskHandle_t task_sensorMagnetometer;

void sensorMagnetometer_readWhoAmI();

I2C_HandleTypeDef I2C_BNO055_Handle;

/**
  * Initialize the hardware.
  */
void sensorMagnetometer_hardwareInit()
{
    // Page 65 of the chip datasheet says pf0 and pf1 are I2c_SDA and I2c_SCL
    // added  __HAL_RCC_GPIOF_CLK_ENABLE(); to the main.c
    // added __HAL_RCC_I2C2_CLK_ENABLE(); to the main.c
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // using PB11 for I2C2_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // using PB10 for I2C2_SCL
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    //I2C_BNO055_Handle.Init.Timing = 0x107DBC; // <- need to see if this is correct.
    // (1+1)/64MHZ = 31.25 NS
    // 


    //<---------------Connor and Charbel Timing Setup----------------->
    // Use table example in reference manual (use 64Mhz base clock divide by 16 to get 4MHz)
    // For each frequency the tables prescales each frequency into 4Mhz and uses 4Mhz for every other settings
    // That's why prescale is 15 since 64MHz / 16 = 4MHz
    //
    // I2C TIMINGR register layout (RM0433 reference manual):
    // [31:28] PRESC  - Prescaler: divides I2C kernel clock. tick = 1/(f_i2cclk / (PRESC+1))
    // [27:24] (reserved, must be 0)
    // [23:20] SCLDEL - SCL data setup delay (in prescaled ticks)
    // [19:16] SDADEL - SDA data hold delay  (in prescaled ticks)
    // [15:8]  SCLH   - SCL high period      (in prescaled ticks, actual = SCLH+1)
    // [7:0]   SCLL   - SCL low period       (in prescaled ticks, actual = SCLL+1)
    //
    // With 64MHz kernel clock and PRESC=15: tick = 1/(64MHz/16) = 250ns
    // SCLDEL=4 -> setup  = 5   * 250ns = 1250ns
    // SDADEL=2 -> hold   = 2   * 250ns =  500ns
    // SCLH=15  -> high   = 16  * 250ns = 4000ns
    // SCLL=19  -> low    = 20  * 250ns = 5000ns
    // f_SCL = 1 / (4000ns + 5000ns) ~= 111kHz (standard-mode 100kHz, rise/fall times account for the rest)
    I2C_BNO055_Handle.Init.Timing =
        (0xFU << 28) |  // PRESC  = 15 : 64MHz / 16 = 4MHz (250ns per tick)
        (0x0U << 24) |  // reserved
        (0x4U << 20) |  // SCLDEL =  4 : SCL data setup  = 5   ticks = 1.25us
        (0x2U << 16) |  // SDADEL =  2 : SDA data hold   = 2   ticks = 500ns
        (0x0FU << 8) |  // SCLH   = 15 : SCL high period = 16  ticks = 4us
        (0x13U << 0);   // SCLL   = 19 : SCL low  period = 20  ticks = 5us

    I2C_BNO055_Handle.Init.AddressingMode =  I2C_ADDRESSINGMODE_7BIT;
    I2C_BNO055_Handle.Init.OwnAddress1 = 0;
    I2C_BNO055_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_BNO055_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_BNO055_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&I2C_BNO055_Handle) != HAL_OK) {
        printf("I2C Init Error");
        Error_Handler();
    }
}

// 0xAA is the start byte

/**
  * Handler for the task.
  */
void sensorMagnetometer_handler(void *argument)
{
    for(;;)
    {
        sensorMagnetometer_readWhoAmI();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}

void sensorMagnetometer_readWhoAmI()
{   
    // then we need to do the write transaciton
    uint8_t sendBuff = BNO055_WHO_AM_I; // Pretty sure this is right
    uint8_t receiveBuff = 0;
    // the chip is 0xA0  on https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
    // HAL_StatusTypeDef HAL_I2C_Master_Receive and HAL_I2C_Master_Receive parameters:
    // I2C_HandleTypeDef *hi2c, - I2C_BNO055_Handle,
    // uint16_t DevAddress, - BNO055_ADDR The header above,
    // uint8_t *pData, - &chip_id,
    // uint16_t Size, - The amount of bytes we are reading,
    // uint32_t Timeout); - The delay,
    HAL_StatusTypeDef info;
    info = HAL_I2C_Master_Transmit(&I2C_BNO055_Handle, BNO055_ADDR << 1, &sendBuff, 1, 5000);
    
    if (info != HAL_OK){
        printf("Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }



    info = HAL_I2C_Master_Receive(&I2C_BNO055_Handle, BNO055_ADDR << 1, &receiveBuff, 1, 5000);

    if (info != HAL_OK) {
        printf("Receive FAILED, HAL status: %d, I2C error: 0x%lX\r\n", info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

    // we right shift by 1 cause we want to write/read data to there, 8 bit address total
    if(receiveBuff == 0xA0) // The value we read: uint8_t reg = BNO055_WHO_AM_I;
    {
        printf("BNO055 WHO_AM_I OK: 0x%02X\r\n", receiveBuff);
    }
    else
    {
        printf("BNO055 WHO_AM_I FAILED: expected 0xA0, got 0x%02X\r\n", receiveBuff);
    }
}
