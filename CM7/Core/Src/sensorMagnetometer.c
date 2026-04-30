#include "main.h"
#include "button.h"
#include "flash.h"
#include "servoSail.h"
#include "stm32h7xx_hal_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "sensorMagnetometer.h"

/* BNO055 device and register addresses. */
#define ADDR  0x28  // ADR pin LOW (default Adafruit breakout). Use 0x29 if ADR pin is HIGH.
#define WHO_AM_I 0x00
#define ACC 0x01
#define MAG 0x02
#define GYRO 0x03
#define ACCX_LSB 0x08
#define ACCX_MSB 0x09
#define ACCY_LSB 0x0A
#define ACCY_MSB 0x0B
#define ACCZ_LSB 0x0C
#define ACCZ_MSB 0x0D
#define MAGX_LSB 0x0E
#define MAGX_MSB 0x0F
#define MAGY_LSB 0x10
#define MAGY_MSB 0x11
#define MAGZ_LSB 0x12
#define MAGZ_MSB 0x13
#define GYRX_LSB 0x14
#define GYRX_MSB 0x15
#define GYRY_LSB 0x16
#define GYRY_MSB 0x17
#define GYRZ_LSB 0x18
#define GYRZ_MSB 0x19
#define EUL_HEADING_LSB 0x1A
#define EUL_HEADING_MSB 0x1B
#define EUL_ROLL_LSB 0x1C   // Roll (heel angle)
#define EUL_PITCH_LSB 0x1E   // Pitch (trim angle)
#define QUATERNION_LSB 0x20
#define LIA_DATA_X_LSB   0x28   // Linear Accel X (gravity removed)
#define LIA_DATA_Y_LSB   0x2A   // Linear Accel Y
#define LIA_DATA_Z_LSB   0x2C   // Linear Accel Z
#define ST_RESULT 0x36
#define SYS_STAT 0x39
#define SYS_ERROR 0x3A
#define OPR_MODE 0x3D
#define CALIB_STATE 0x35
#define CALIB 0x55
#define CALIB_OFFSET_SIZE 22
#define SICMATRIX 0x43
#define CALIB_SICMATRIX_SIZE 18

#define OPR_MODE_CONFIG 0x00
#define OPR_MODE_ACCONLY 0x01
#define OPR_MODE_MAGONLY 0x02
#define OPR_MODE_GYROONLY 0x03
#define OPR_MODE_ACCMAG 0x04
#define OPR_MODE_ACCGYRO 0x05
#define OPR_MODE_MAGGYRO 0x06
#define OPR_MODE_AMG 0x07
#define OPR_MODE_IMUPLUS 0x08
#define OPR_MODE_COMPASS 0x09
#define OPR_MODE_M4G 0x0A
#define OPR_MODE_NDOF_FMC_OFF 0x0B
#define OPR_MODE_NDOF 0x0C

#define TRUE 0x01
#define FALSE 0x00

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Helper macros for printing fixed-point style debug values without %f support.
#define FLOAT_INT(f)  ((int)(f))
#define FLOAT_FRAC(f) ((int)(fabsf((f) - (int)(f)) * 10000))

TaskHandle_t task_sensorMagnetometer = NULL;

/* Internal helpers used during initialization, sampling, and calibration. */
void setOperationMode(uint8_t mode);
static void readChip(uint8_t regADDR, const char *name);
void readWhoAmI();
void readMAG();
void readACC();
void readGYRO();
void readSelfTest();
void readGYRO_Vector();
void readQuaternion();
void readVectorDynamic(uint8_t startReg, uint8_t bytes, const char *name, uint8_t *vectorData);
static uint8_t calibrationProcedure(uint8_t targetMode, uint8_t startMode, bool recalibrate, bool sicMatrix);
uint8_t checkCalibration(int onlySysGyro);
void saveCalibrationData(bool sicMatrix);
void loadCalibrationData(bool sicMatrix);
void fillStruct();
void printIMU();

typedef struct {
    // Euler angles direct from chip. 1 degree = 16 LSB.
    // heading: 0–5759 (0–359.9°), roll: -2879–2879, pitch: -1439–1439
    int16_t heading;     // Compass heading (yaw), 0–360°
    int16_t roll;        // Heel angle, port(-) / starboard(+)
    int16_t pitch;       // Trim angle, bow-up(+) / bow-down(-)

    // --- Fusion: Quaternion (for control math, SLERP, etc.) ---
    int16_t quat_w, quat_x, quat_y, quat_z;

    // --- Fusion: Linear Acceleration (gravity already removed by BNO055) ---
    // 1 m/s² = 100 LSB
    int16_t lia_x, lia_y, lia_z;

    // --- Raw: Angular Rate (rate of turn — useful for autopilot) ---
    // 1 dps = 16 LSB (default), or 1 rps = 900 LSB
    int16_t gyro_x, gyro_y, gyro_z;

    // --- Status ---
    uint8_t calib_stat;   // bits[7:6]=sys, [5:4]=gyro, [3:2]=acc, [1:0]=mag (3=fully calib)
    uint32_t update_count;
} IMU_Data;

/* Shared IMU sample data, I2C handle, and task state. */
IMU_Data IMU = {0};
I2C_HandleTypeDef I2C_BNO055_Handle;
uint8_t currentMode;
int16_t servoAngle;
static uint8_t isCalibrated = FALSE;

static uint8_t savedOffsets[CALIB_OFFSET_SIZE] = {0};
static uint8_t savedSicMatrix[CALIB_SICMATRIX_SIZE] = {0};

// User Calculated sicMatrix to be saved to flash memory and written to IMU
static uint8_t UserMadeSicMatrix[CALIB_SICMATRIX_SIZE] = {0x1,0x2,0x3,0x4,0x5,0x6,
    0x7,0x8,0x8,0xA,0xB,0xC,0xD,0xE,0xF,0x10,0x11,0x12};

/**
  * @brief Configure I2C2 and initialize the BNO055 sensor.
  */
void sensorMagnetometer_hardwareInit()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PB11 and PB10 are routed to I2C2 on this board.
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
    
    // Args: desired fusion mode, starting mode, recalibrate, save/load SIC Matrix.
    calibrationProcedure(OPR_MODE_NDOF, OPR_MODE_CONFIG, false, false);
}

/**
  * @brief Run the active magnetometer mode and mirror the selected angle onto the sail servo.
  * @param argument Unused RTOS task argument.
  */
void sensorMagnetometer_handler(void *argument)
{
    int32_t count = 0;
    for(;;)
    {
        fillStruct();
        count++;
        if (count == 1000) {
            printIMU();
            count = 0;
        }
    }
}

/**
  * @brief Write a new BNO055 operating mode and wait for the change to settle.
  * @param mode BNO055 operating mode value to write.
  */
void setOperationMode(uint8_t mode)
{
    HAL_I2C_Mem_Write(&I2C_BNO055_Handle, ADDR << 1, OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, 1000);
    HAL_Delay(25); // small delay to allow mode switch to take effect
    currentMode = mode;
}

// Read Euler angles directly from BNO055 (heading, roll, pitch in one 6-byte burst)
void readEuler_Direct() {
    uint8_t data[6] = {0};
    readVectorDynamic(EUL_HEADING_LSB, 6, "Euler", data);
    IMU.heading = (int16_t)((data[1] << 8) | data[0]);  // Heading/Yaw
    IMU.roll    = (int16_t)((data[3] << 8) | data[2]);  // Roll
    IMU.pitch   = (int16_t)((data[5] << 8) | data[4]);  // Pitch
}

// Linear acceleration with gravity removed (fusion output)
void readLinearAccel() {
    uint8_t data[6] = {0};
    readVectorDynamic(LIA_DATA_X_LSB, 6, "LIA", data);
    IMU.lia_x = (int16_t)((data[1] << 8) | data[0]);
    IMU.lia_y = (int16_t)((data[3] << 8) | data[2]);
    IMU.lia_z = (int16_t)((data[5] << 8) | data[4]);
}

void readCalibStatus() {
    HAL_I2C_Mem_Read(&I2C_BNO055_Handle, ADDR << 1,
        CALIB_STATE, I2C_MEMADD_SIZE_8BIT, &IMU.calib_stat, 1, 1000);
}

void fillStruct() {
    readEuler_Direct();       // Heading, roll, pitch — direct from fusion engine
    readQuaternion();         // For accurate attitude math
    readLinearAccel();        // Gravity-free acceleration
    readGYRO_Vector();        // Rate of turn
    readCalibStatus();        // Calibration health
    IMU.update_count++;
}

/**
  * @brief Read and verify the BNO055 chip ID register.
  */
void readWhoAmI() {
    readChip(WHO_AM_I, "Who Am I");
}

/**
  * @brief Read a one-byte identification or status register and compare it to the expected value.
  * @param regADDR Register address to read.
  * @param name Human-readable label used in debug output.
  */
static void readChip(uint8_t regADDR, const char *name)
{   
    uint8_t receiveBuff = 0;
    uint8_t expected = 0;

    HAL_StatusTypeDef info;

    info = HAL_I2C_Mem_Read(&I2C_BNO055_Handle, ADDR << 1, regADDR,
                            I2C_MEMADD_SIZE_8BIT, &receiveBuff, 1, 5000);

    if (info != HAL_OK) {
        //printf("%s FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        HAL_I2C_DeInit(&I2C_BNO055_Handle);
        HAL_I2C_Init(&I2C_BNO055_Handle);
        return;
    }

    switch (regADDR) {
        case WHO_AM_I: expected = 0xA0; break;
        case ACC: expected = 0xFB; break;
        case MAG: expected = 0x32; break;
        case GYRO: expected = 0x0F; break;
        case ST_RESULT: expected = 0x0F; break;
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

/**
  * @brief Read and verify the magnetometer ID register.
  */
void readMAG()
{   
    readChip(MAG, "MAG");
}

/**
  * @brief Read and verify the accelerometer ID register.
  */
void readACC()
{   
    readChip(ACC, "ACC");
}

/**
  * @brief Read and verify the gyroscope ID register.
  */
void readGYRO()
{   
    readChip(GYRO, "GYRO");
}

/**
  * @brief Read and verify the BNO055 self-test result register.
  */
void readSelfTest()
{   
    readChip(ST_RESULT, "Self-Test Result");
}

/**
  * @brief Read a three-axis sensor vector from six consecutive BNO055 registers.
  * @param startReg First register in the vector block.
  * @param name Label used in debug output.
  * @param xData Destination for the X-axis sample.
  * @param yData Destination for the Y-axis sample.
  * @param zData Destination for the Z-axis sample.
  */
static void BNO055_readVector(uint8_t startReg, const char *name, int16_t *xData, int16_t *yData, int16_t *zData)
{
    uint8_t data[6] = {0xF, 0xF, 0xF, 0xF, 0xF, 0xF}; // Initialize with invalid data for easier debugging
    HAL_StatusTypeDef info;

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        ADDR << 1,              // 7-bit addr shifted for HAL
        startReg,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        data,                   // output buffer
        6,                      // read 6 bytes (LSB+MSB for X, Y, Z)
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("%s Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
    }

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);
    *xData = x; *yData = y; *zData = z;

//    printf("%s: X=%d, Y=%d, Z=%d\r\n", name, x, y, z);
}

/**
  * @brief Refresh the gyroscope sample stored in the IMU struct.
  */
void readGYRO_Vector()
{
    BNO055_readVector(GYRX_LSB, "GYRO",  &IMU.gyro_x, &IMU.gyro_y, &IMU.gyro_z);
}

/**
  * @brief Read an arbitrary block of bytes from the BNO055.
  * @param startReg First register to read.
  * @param bytes Number of bytes to read.
  * @param name Label used in debug output.
  * @param vectorData Destination buffer.
  */
void readVectorDynamic(uint8_t startReg, uint8_t bytes, const char *name, uint8_t *vectorData)
{
    HAL_StatusTypeDef info;

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        ADDR << 1,              // 7-bit addr shifted for HAL
        startReg,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        vectorData,                 // output buffer
        bytes,                   // number of bytes to read
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("%s Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

//    printf("%s: ", name);
    // for (uint8_t i = 0; i < bytes; i++) {
    //     // printf("%02X ", vectorData[i]);
    // }
    // printf("\r\n");
}

/**
  * @brief Refresh the quaternion sample stored in the IMU struct.
  */
void readQuaternion() {
    uint8_t quat[8] = {0};
    readVectorDynamic(QUATERNION_LSB, 8, "Quaternion Data", quat);
    IMU.quat_w = (quat[1] << 8) | quat[0];
    IMU.quat_x = (quat[3] << 8) | quat[2];
    IMU.quat_y = (quat[5] << 8) | quat[4];
    IMU.quat_z = (quat[7] << 8) | quat[6];
}

/**
  * @brief Print the current raw and fused IMU values to the debug console.
  */
void printIMU()
{
    // Scale Euler: 1 deg = 16 LSB
    float heading = IMU.heading / 16.0f;
    float roll    = IMU.roll    / 16.0f;
    float pitch   = IMU.pitch   / 16.0f;

    // Scale linear accel: 1 m/s² = 100 LSB
    float lia_x = IMU.lia_x / 100.0f;
    float lia_y = IMU.lia_y / 100.0f;
    float lia_z = IMU.lia_z / 100.0f;
    float lia_mag = sqrtf(lia_x*lia_x + lia_y*lia_y + lia_z*lia_z);

    // Gyro: 1 dps = 16 LSB
    float gyro_z = IMU.gyro_z / 16.0f;  // yaw rate (rate of turn)

    // Calibration bits
    uint8_t cal_sys  = (IMU.calib_stat >> 6) & 0x03;
    uint8_t cal_gyro = (IMU.calib_stat >> 4) & 0x03;
    uint8_t cal_acc  = (IMU.calib_stat >> 2) & 0x03;
    uint8_t cal_mag  = (IMU.calib_stat >> 0) & 0x03;

    printf("\r\n===== SAILBOAT IMU =====\r\n");

    // Navigation heading (most important for autonomous sailing)
    printf("Heading:  %d.%02d deg  (Compass)\r\n",
        FLOAT_INT(heading), (int)(fabsf(heading - (int)heading) * 100));

    // Heel and trim
    printf("Roll:     %d.%02d deg  (+ = starboard heel)\r\n",
        FLOAT_INT(roll),  (int)(fabsf(roll  - (int)roll)  * 100));
    printf("Pitch:    %d.%02d deg  (+ = bow up)\r\n",
        FLOAT_INT(pitch), (int)(fabsf(pitch - (int)pitch) * 100));

    // Rate of turn (autopilot damping)
    printf("Yaw rate: %d.%02d dps\r\n",
        FLOAT_INT(gyro_z), (int)(fabsf(gyro_z - (int)gyro_z) * 100));

    // Linear acceleration magnitude (wave/impact detection)
    printf("Accel:    %d.%02d m/s2 (no gravity, |total|=%d.%02d)\r\n",
        FLOAT_INT(lia_x), (int)(fabsf(lia_x - (int)lia_x) * 100),
        FLOAT_INT(lia_mag), (int)(fabsf(lia_mag - (int)lia_mag) * 100));

    // Calibration health — critical for trusting heading
    printf("Calib:    Sys=%d/3 Gyro=%d/3 Acc=%d/3 Mag=%d/3  %s\r\n",
        cal_sys, cal_gyro, cal_acc, cal_mag,
        (cal_sys == 3 && cal_mag == 3) ? "[OK]" : "[WARN: heading unreliable]");

    printf("==================================\r\n");
}

/**
 * @brief Restore saved calibration data when available, or run a new BNO055 calibration pass.
 * @param targetMode Operating mode to use after calibration handling completes.
 * @param startMode Caller-supplied current operating mode state before transitions begin.
 * @param recalibrate When true, force a new calibration pass even if flash data exists.
 * @param sicMatrix When true, load/save the SIC matrix alongside the standard offsets.
 * @return Non-zero when the procedure reaches its configured end state.
 */
static uint8_t calibrationProcedure(uint8_t targetMode, uint8_t startMode, bool recalibrate, bool sicMatrix) {
    // initialize calibrationData
    FlashCalibData_t flashCalibData = {0};

    currentMode = startMode;

    // Loads flash calibration data from flash memory
    if (flashStorage_read(&flashCalibData)) {
        isCalibrated = flashCalibData.isCalibrated;
        memcpy(savedOffsets, flashCalibData.offsets, sizeof(savedOffsets));
        memcpy(savedSicMatrix, flashCalibData.sicMatrix, sizeof(savedSicMatrix));
        printf("Flash calibration loaded: calibrated=%u offset[0..1]=%02X %02X sic[0..1]=%02X %02X\r\n",
               isCalibrated,
               savedOffsets[0], savedOffsets[1],
               savedSicMatrix[0], savedSicMatrix[1]);
    } else {
        isCalibrated = FALSE;
        memset(savedOffsets, 0, sizeof(savedOffsets));
    }

    // If non-fusion mode, just sets operation mode and return
    if (!(targetMode >= OPR_MODE_IMUPLUS && targetMode <= OPR_MODE_NDOF))
    {
        setOperationMode(targetMode);
        return TRUE;
    }

    // If already calibrated and not recalirating, load caliration data and return
    if (isCalibrated && !recalibrate) {
        loadCalibrationData(sicMatrix);
        setOperationMode(targetMode);
        return TRUE;
    }

    // Go through full calibration procedure
    setOperationMode(targetMode);
    HAL_Delay(20);

    printf("%s: Move sensor in figure-8 for mag, hold 6 orientations for acc, keep still for gyro\r\n",
         recalibrate ? "Recalibrating: " : "Calibrating: ");
    HAL_Delay(1000);
    int count = 0;
    while (checkCalibration(0) == 0 && count < 90)
    {
        count++;
        HAL_Delay(2000);
    }

    saveCalibrationData(sicMatrix);
    HAL_Delay(1000);
    isCalibrated = TRUE;
    printf("BNO055 fully calibrated\r\n");

    // saveCalibrationData() leaves the chip in CONFIG mode.
    setOperationMode(targetMode);
    return TRUE;
}

/**
  * @brief Read and report the current BNO055 calibration state.
  * @param onlySysGyro When non-zero, only require system and gyro calibration to pass.
  * @return Non-zero when the requested calibration criteria are satisfied.
  */
uint8_t checkCalibration(int onlySysGyro) {

    HAL_StatusTypeDef info;
    uint8_t calBuffer = 0;
    
    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        ADDR << 1,              // 7-bit addr shifted for HAL
        CALIB_STATE,            // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        &calBuffer,                 // output buffer
        1,                   // number of bytes to read
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("Calibration Register Read FAILED, HAL status: %d, I2C error: 0x%lX\r\n", info, HAL_I2C_GetError(&I2C_BNO055_Handle));
    }

    uint8_t sys_cal  = (calBuffer >> 6) & 0x03;
    uint8_t gyro_cal = (calBuffer >> 4) & 0x03;
    uint8_t acc_cal  = (calBuffer >> 2) & 0x03;
    uint8_t mag_cal  = (calBuffer >> 0) & 0x03;

    printf("System: %d/3 | Accel: %d/3 | Mag: %d/3 | Gyro: %d/3\r\n",
           sys_cal, acc_cal, mag_cal, gyro_cal);

    // All three must be fully calibrated
    if (onlySysGyro) {
        return (sys_cal == 3 && gyro_cal == 3);
    }
    return (sys_cal == 3 && acc_cal == 3 && mag_cal == 3 && gyro_cal == 3);
}

/**
  * @brief Read the current calibration offsets from the BNO055 into `savedOffsets`.
  */
void saveCalibrationData(bool sicMatrix)
{
    HAL_StatusTypeDef info;
    FlashCalibData_t flashCalibData = {0};

    setOperationMode(OPR_MODE_CONFIG);
    HAL_Delay(20);

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        ADDR << 1,
        CALIB,
        I2C_MEMADD_SIZE_8BIT,
        savedOffsets,
        CALIB_OFFSET_SIZE,
        5000
    )) != HAL_OK) {
        printf("Offset save FAILED, status: %d, err: 0x%lX\r\n",
               info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

    if (sicMatrix) {
        // Copy User made SIC Matrix into flash memory struct
        memcpy(flashCalibData.sicMatrix, UserMadeSicMatrix, sizeof(savedSicMatrix));
    }

    flashCalibData.magic = FLASH_STORAGE_MAGIC;
    flashCalibData.isCalibrated = TRUE;
    memcpy(flashCalibData.offsets, savedOffsets, sizeof(savedOffsets));

    if (flashStorage_write(&flashCalibData) != HAL_OK) {
        printf("Calibration save to flash FAILED\r\n");
        return;
    }

    printf("Calibration saved to flash\r\n");
}

/**
  * @brief Write the cached calibration offsets back into the BNO055.
  */
void loadCalibrationData(bool sicMatrix)
{
    HAL_StatusTypeDef info;

    setOperationMode(OPR_MODE_CONFIG);
    HAL_Delay(20);

    if ((info = HAL_I2C_Mem_Write(
        &I2C_BNO055_Handle,
        ADDR << 1,
        CALIB,
        I2C_MEMADD_SIZE_8BIT,
        savedOffsets,
        CALIB_OFFSET_SIZE,
        5000
    )) != HAL_OK) {
        printf("Offset load FAILED, status: %d, err: 0x%lX\r\n",
               info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

    if (sicMatrix) {
        if ((info = HAL_I2C_Mem_Write(
        &I2C_BNO055_Handle,
        ADDR << 1,
        SICMATRIX,
        I2C_MEMADD_SIZE_8BIT,
        savedSicMatrix,
        CALIB_SICMATRIX_SIZE,
        5000
        )) != HAL_OK) {
            printf("SIC Matrix load FAILED, status: %d, err: 0x%lX\r\n",
                info, HAL_I2C_GetError(&I2C_BNO055_Handle));
            return;
        }
    }

    printf("Calibration offsets loaded from flash copy\r\n");
}
