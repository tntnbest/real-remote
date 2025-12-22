#pragma once
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "main.h"

#define MAX_SAVE    100
#define NAME_LEN    16
#define MAX_PULSES  100

#define STORAGE_BASE 0x08070000U
#define STORAGE_END  0x08080000U
#define STORAGE_SIZE    (STORAGE_END - STORAGE_BASE)
#define STORAGE_SECTOR  FLASH_SECTOR_7
#define VOLTAGE_RANGE   FLASH_VOLTAGE_RANGE_3

HAL_StatusTypeDef SaveIRToFlash(const char *name, uint16_t *pulses, uint8_t count);
HAL_StatusTypeDef Storage_Erase(void);
HAL_StatusTypeDef Storage_Write(uint32_t offset, const uint8_t *data, uint32_t len);
void LoadAllIRFromFlash(void);
