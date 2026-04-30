#include "flash.h"

#include <stdio.h>
#include <string.h>

#define FLASH_STORAGE_TEST_STRING "FLASH_RW_OK"

static void flashStorage_copyRaw(FlashCalibData_t *data)
{
    memcpy(data, (const void *)FLASH_STORAGE_ADDR, sizeof(*data));
}

uint8_t flashStorage_read(FlashCalibData_t *data)
{
    if (data == NULL) {
        return 0U;
    }

    flashStorage_copyRaw(data);
    if (data->magic != FLASH_STORAGE_MAGIC) {
        printf("Flash: no calibration record (magic=0x%08lX \r\n expected=0x0%081X)\r\n",
               (unsigned long)data->magic, FLASH_STORAGE_MAGIC);
        return 0U;
    }

    printf("Flash: calibration record found (isCalibrated=%u)\r\n",
           data->isCalibrated);
    return 1U;
}

HAL_StatusTypeDef flashStorage_write(const FlashCalibData_t *data)
{
    FlashCalibData_t staging __attribute__((aligned(32)));
    FlashCalibData_t verify;
    HAL_StatusTypeDef status;

    if (data == NULL) {
        return HAL_ERROR;
    }

    memcpy(&staging, data, sizeof(staging));
    staging.magic = FLASH_STORAGE_MAGIC;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        printf("Flash: unlock failed (0x%02X)\r\n", status);
        return status;
    }

    {
        FLASH_EraseInitTypeDef eraseInit = {
            .TypeErase = FLASH_TYPEERASE_SECTORS,
            .Banks = FLASH_STORAGE_BANK,
            .Sector = FLASH_STORAGE_SECTOR,
            .NbSectors = 1,
            .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        };
        uint32_t sectorError = 0;

        status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
        if (status != HAL_OK) {
            printf("Flash: erase failed (sectorError=0x%08lX, flashErr=0x%08lX)\r\n",
                   (unsigned long)sectorError,
                   (unsigned long)HAL_FLASH_GetError());
            HAL_FLASH_Lock();
            return status;
        }
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                               FLASH_STORAGE_ADDR,
                               (uint32_t)(uintptr_t)&staging);
    if (status != HAL_OK) {
        printf("Flash: first 32-byte write failed (err=0x%08lX)\r\n",
               (unsigned long)HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                               FLASH_STORAGE_ADDR + 32U,
                               (uint32_t)(uintptr_t)((const uint8_t *)&staging + 32U));
    if (status != HAL_OK) {
        printf("Flash: second 32-byte write failed (err=0x%08lX)\r\n",
               (unsigned long)HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return status;
    }

    if (HAL_FLASH_Lock() != HAL_OK) {
        printf("Flash: relock failed after write\r\n");
        return HAL_ERROR;
    }

    flashStorage_copyRaw(&verify);
    if (memcmp(&verify, &staging, sizeof(staging)) != 0) {
        printf("Flash: write/read verify failed\r\n");
        return HAL_ERROR;
    }

    printf("Flash: write/read verify passed\r\n");
    return HAL_OK;
}

void flashStorage_runMagicStringTest(void)
{
    FlashCalibData_t record = {0};
    static const char testString[] = FLASH_STORAGE_TEST_STRING;

    flashStorage_copyRaw(&record);

    if (memcmp(record.sicMatrix, testString, sizeof(testString)) == 0) {
        printf("Flash test: magic string already present: %s\r\n", record.sicMatrix);
        return;
    }

    if (record.magic == FLASH_STORAGE_MAGIC) {
        printf("Flash test: magic string missing, preserving existing record and writing test string\r\n");
    } else {
        printf("Flash test: no valid record, writing test string\r\n");
        memset(&record, 0, sizeof(record));
    }

    memset(record.sicMatrix, 0, sizeof(record.sicMatrix));
    memcpy(record.sicMatrix, testString, sizeof(testString));

    if (flashStorage_write(&record) != HAL_OK) {
        printf("Flash test: failed to write magic string\r\n");
        return;
    }

    flashStorage_copyRaw(&record);
    if (memcmp(record.sicMatrix, testString, sizeof(testString)) != 0) {
        printf("Flash test: readback mismatch after write\r\n");
        return;
    }

    printf("Flash test: magic string written and verified: %s\r\n", record.sicMatrix);
}
