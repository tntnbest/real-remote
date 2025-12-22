#include "flash_storage.h"
#include "main.h"        
#include <string.h>

char     save_names   [MAX_SAVE][NAME_LEN];
uint16_t save_ir      [MAX_SAVE][MAX_PULSES];
uint8_t  save_counts  [MAX_SAVE];
uint8_t  save_count = 0;
uint32_t flash_next_offset = 0;

HAL_StatusTypeDef SaveIRToFlash(const char *name, uint16_t *pulses, uint8_t count) {
    uint8_t save_data[1 + NAME_LEN + 1 + MAX_PULSES * 2];
    memset(save_data, 0xFF, sizeof(save_data));      // ★ 패딩 초기화

    uint8_t namelen = strlen(name);
    if (namelen > NAME_LEN) namelen = NAME_LEN;

    uint32_t idx = 0;
    save_data[idx++] = namelen;

    memcpy(&save_data[idx], name, namelen);
    idx += NAME_LEN;                                 // 고정 길이 슬롯 건너뜀

    save_data[idx++] = count;

    for (uint8_t i = 0; i < count; i++) {
        uint16_t v = pulses[i];
        save_data[idx++] = (uint8_t)(v & 0xFF);
        save_data[idx++] = (uint8_t)(v >> 8);
    }

    uint32_t write_len = (idx + 1) & ~1U;           // 2바이트 정렬
    HAL_StatusTypeDef st = Storage_Write(flash_next_offset, save_data, write_len);
    if (st == HAL_OK) flash_next_offset += write_len;
    return st;
}


HAL_StatusTypeDef Storage_Write(uint32_t offset, const uint8_t *data, uint32_t len) {
    if (offset + len > STORAGE_SIZE) return HAL_ERROR;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_OK;
    for (uint32_t i = 0; i < len; i += 2) {
        uint16_t half = data[i];
			
        if (i + 1 < len) half |= (uint16_t)data[i+1] << 8;
        else half |= 0xFF00;
			
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, STORAGE_BASE + offset + i, half);
                               
        if (st != HAL_OK) break;
    }
    HAL_FLASH_Lock();
    return st;
}


HAL_StatusTypeDef Storage_Erase(void) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = VOLTAGE_RANGE;
    eraseInit.Sector = STORAGE_SECTOR;
    eraseInit.NbSectors = 1;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();
    return status;
}

void LoadAllIRFromFlash(void) {
    uint32_t offset = 0;
    save_count = 0;

    while (offset < STORAGE_SIZE) {
        uint8_t *base = (uint8_t*)(STORAGE_BASE + offset);
        uint8_t namelen = base[0];
        if (namelen == 0xFF || namelen == 0 || save_count >= MAX_SAVE)
            break;
        offset += 1;

        memcpy(save_names[save_count], (void*)(STORAGE_BASE + offset), namelen);
        save_names[save_count][namelen] = '\0';
        offset += NAME_LEN;

        uint8_t cnt = *(uint8_t*)(STORAGE_BASE + offset);
        offset += 1;
        save_counts[save_count] = cnt;

        for (uint8_t i = 0; i < cnt; i++) {
            uint8_t lo = *(uint8_t*)(STORAGE_BASE + offset++);
            uint8_t hi = *(uint8_t*)(STORAGE_BASE + offset++);
            save_ir[save_count][i] = (uint16_t)(lo | (hi << 8));
        }

        if (offset & 1) offset++;
        save_count++;
    }
    flash_next_offset = offset;
}
