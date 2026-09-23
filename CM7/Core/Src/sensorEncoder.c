#include "main.h"
#include "stm32h7xx_hal_i2c.h"
#include <stdint.h>
#include "sensorMagnetometer.h"
#include <math.h>

#ifndef AS5600_REGS_H
#define AS5600_REGS_H

/* 7-bit I2C address */
#define AS5600_ADDR             0x36

/* Configuration registers */
#define AS5600_ZMCO             0x00
#define AS5600_ZPOS_H           0x01  /* ZPOS[11:8] */
#define AS5600_ZPOS_L           0x02  /* ZPOS[7:0]  */
#define AS5600_MPOS_H           0x03  /* MPOS[11:8] */
#define AS5600_MPOS_L           0x04  /* MPOS[7:0]  */
#define AS5600_MANG_H           0x05  /* MANG[11:8] */
#define AS5600_MANG_L           0x06  /* MANG[7:0]  */
#define AS5600_CONF_H           0x07  /* WD, FTH[2:0], SF[1:0] */
#define AS5600_CONF_L           0x08  /* PWMF, OUTS, HYST, PM  */

/* Output registers */
#define AS5600_RAW_ANGLE_H      0x0C  /* RAW_ANGLE[11:8] */
#define AS5600_RAW_ANGLE_L      0x0D  /* RAW_ANGLE[7:0]  */
#define AS5600_ANGLE_H          0x0E  /* ANGLE[11:8] */
#define AS5600_ANGLE_L          0x0F  /* ANGLE[7:0]  */

/* Status registers */
#define AS5600_STATUS           0x0B
#define AS5600_AGC              0x1A
#define AS5600_MAGNITUDE_H      0x1B  /* MAGNITUDE[11:8] */
#define AS5600_MAGNITUDE_L      0x1C  /* MAGNITUDE[7:0]  */

/* Burn command */
#define AS5600_BURN             0xFF
#define AS5600_BURN_ANGLE           0x80
#define AS5600_BURN_SETTING         0x40

/* STATUS bits */
#define AS5600_STATUS_MH            (1U << 3)  /* magnet too strong */
#define AS5600_STATUS_ML            (1U << 4)  /* magnet too weak   */
#define AS5600_STATUS_MD            (1U << 5)  /* magnet detected   */

/* 12-bit value mask for H/L register pairs */
#define AS5600_12BIT_MASK           0x0FFF

#endif /* AS5600_REGS_H */

#define TRUE 0x01
#define FALSE 0x00

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Helper macro to split a float into printable integer parts
#define FLOAT_INT(f)  ((int)(f))
#define FLOAT_FRAC(f) ((int)(fabsf((f) - (int)(f)) * 10000))

void setOperationMode(uint8_t mode);
static void readChip(uint8_t regADDR, const char *name);
void readVectorDynamic(uint8_t startReg, uint8_t bytes, const char *name, uint8_t *vectorData);
void fillStruct();
void printIMU();

#pragma pack(1) // ensure no padding between fields
typedef struct {
    // Raw sensor vectors
    int16_t acc_x,  acc_y,  acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x,  mag_y,  mag_z;

    //Quaternion Fusion Mode Values
    int16_t w, x, y, z;

    // Claude said to have ths for when we transfer between cores
    uint32_t update_count;
} Encoder_Data;
#pragma pack()

Encoder_Data IMU = {0};
TaskHandle_t task_sensorMagnetometer;
I2C_HandleTypeDef I2C_BNO055_Handle;
uint8_t currentMode;

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
    I2C_BNO055_Handle.Init.OwnAddress1 = 0x1;
    I2C_BNO055_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_BNO055_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_BNO055_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&I2C_BNO055_Handle) != HAL_OK) {
        printf("I2C Init Error");
        Error_Handler();
    }

    // BNO055 requires up to 650ms after power-on before it responds to I2C.
    // Without this delay the first transaction gets a NACK (error 0x2) which
    // can then leave the peripheral in a stuck state (error 0x20).
    HAL_Delay(700);

    readWhoAmI();
    readSelfTest();
    currentMode = BNO055_OPR_MODE_NDOF; // USER: only change this line to set mode

    if (!(currentMode >= BNO055_OPR_MODE_IMUPLUS && currentMode <= BNO055_OPR_MODE_NDOF))
    {
        // Non-fusion mode — just set it, no calibration needed
        setOperationMode(currentMode);
    }
    if (isCalibrated) {
        // Step 1: Must be in CONFIG mode to write offsets (chip powers on here anyway)
        setOperationMode(BNO055_OPR_MODE_CONFIG);

        // Step 2: Write saved offsets — gives fusion algorithm a head start
        loadCalibrationData();

        // Step 3: Enter fusion mode — algorithm starts running and will
        setOperationMode(BNO055_OPR_MODE_NDOF);

        // Step 4: Still poll CALIB_STAT — but with good offsets loaded
        // this should reach 3/3/3 in seconds, not minutes
        printf("Offsets loaded — waiting for fusion algorithm to confirm calibration...\r\n");
        while (checkCalibration(1) == 0)
        {
            HAL_Delay(500);
        }
        printf("Calibration confirmed\r\n");
    }
    else
    {
        // First boot in fusion mode — must set fusion mode FIRST so the calibration
        // algorithm runs, then poll until all three sensors reach 3/3

        setOperationMode(currentMode);
        HAL_Delay(20);

        printf("Move sensor in figure-8 for mag, hold 6 orientations for acc, keep still for gyro\r\n");
        int count = 0;
        while (checkCalibration(0) == 0 && count < 90)
        {
            count++;
            HAL_Delay(2000);
        }

        // saveCalibrationOffsets switches to CONFIG_MODE internally to read offsets
        saveCalibrationData();
        HAL_Delay(10000);
        isCalibrated = TRUE;
        printf("BNO055 fully calibrated\r\n");

        // Restore fusion mode (save left chip in CONFIG_MODE)
        setOperationMode(currentMode);
    }
}

void setOperationMode(uint8_t mode)
{
    HAL_I2C_Mem_Write(&I2C_BNO055_Handle, BNO055_ADDR << 1, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, 1000);
    HAL_Delay(25); // small delay to allow mode switch to take effect
    currentMode = mode;
}

// 0xAA is the start byte

/**
  * Handler for the task.
  */
void sensorMagnetometer_handler(void *argument)
{
    for(;;)
    {
        fillStruct();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
        printIMU();
    }
}

void readWhoAmI() {
    readChip(BNO055_WHO_AM_I, "Who Am I");
}

static void readChip(uint8_t regADDR, const char *name)
{   
    uint8_t receiveBuff = 0;
    uint8_t expected = 0;

    HAL_StatusTypeDef info;

    info = HAL_I2C_Mem_Read(&I2C_BNO055_Handle, BNO055_ADDR << 1, regADDR,
                            I2C_MEMADD_SIZE_8BIT, &receiveBuff, 1, 5000);

    if (info != HAL_OK) {
        //printf("%s FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        HAL_I2C_DeInit(&I2C_BNO055_Handle);
        HAL_I2C_Init(&I2C_BNO055_Handle);
        return;
    }

    switch (regADDR) {
        case BNO055_WHO_AM_I: expected = 0xA0; break;
        case BNO055_ACC: expected = 0xFB; break;
        case BNO055_MAG: expected = 0x32; break;
        case BNO055_GYRO: expected = 0x0F; break;
        case BNO055_ST_RESULT: expected = 0x0F; break;
        default: expected = 0xff;
    }

    if(receiveBuff == expected)
    {
        printf("BNO055 %s OK: 0x%02X\r\n", name, receiveBuff);
    }
    else
    {
        printf("BNO055 %s FAILED: expected 0x%02X, got 0x%02X\r\n", name, expected, receiveBuff);
    }
}

static void BNO055_readVector(uint8_t startReg, const char *name, int16_t *xData, int16_t *yData, int16_t *zData)
{
    uint8_t data[6] = {0xF, 0xF, 0xF, 0xF, 0xF, 0xF}; // Initialize with invalid data for easier debugging
    HAL_StatusTypeDef info;

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,       // 7-bit addr shifted for HAL
        startReg,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        data,                   // output buffer
        6,                      // read 6 bytes (LSB+MSB for X, Y, Z)
        5000                    // timeout ms
    )) != HAL_OK) {
        //printf("%s Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
    }

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);
    *xData = x; *yData = y; *zData = z;
}

void readVectorDynamic(uint8_t startReg, uint8_t bytes, const char *name, uint8_t *vectorData)
{
    HAL_StatusTypeDef info;

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,       // 7-bit addr shifted for HAL
        startReg,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        vectorData,                 // output buffer
        bytes,                   // number of bytes to read
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("%s Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

    printf("\r\n");
}

void readQuaternion() {
    uint8_t quat[8] = {0};
    readVectorDynamic(BNO055_Quaternion_LSB, 8, "Quaternion Data", quat);
    IMU.w = (quat[1] << 8) | quat[0];
    IMU.x = (quat[3] << 8) | quat[2];
    IMU.y = (quat[5] << 8) | quat[4];
    IMU.z = (quat[7] << 8) | quat[6];
}

void fillStruct() {
    readACC_Vector();
    readMAG_Vector();
    readGYRO_Vector();
    readQuaternion();
}

