#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "main.h"
#include <stdint.h>

#define FLASH_STORAGE_ADDR        0x081E0000U
#define FLASH_STORAGE_SECTOR      FLASH_SECTOR_7
#define FLASH_STORAGE_BANK        FLASH_BANK_2
#define FLASH_STORAGE_SECTOR_SIZE (128U * 1024U)
#define FLASH_STORAGE_MAGIC       0xCAFEBABEU

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint8_t  isCalibrated;
    uint8_t  offsets[22];
    uint8_t  padding1[5];
    uint8_t  sicMatrix[18];
    uint8_t  padding2[14];
} FlashCalibData_t;
#pragma pack(pop)

_Static_assert(sizeof(FlashCalibData_t) == 64U, "FlashCalibData_t must stay 64 bytes");

HAL_StatusTypeDef flashStorage_write(const FlashCalibData_t *data);
uint8_t flashStorage_read(FlashCalibData_t *data);
void flashStorage_runMagicStringTest(void);

#endif
