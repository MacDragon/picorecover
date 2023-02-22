#pragma once

#define BOOTMAGIC (0xDEADBEEF)

// from micropython rp2 port
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)

#ifndef MICROPY_HW_FLASH_STORAGE_BYTES
#define MICROPY_HW_FLASH_STORAGE_BYTES (1408 * 1024)
#endif
#ifndef MICROPYW_HW_FLASH_STORAGE_BYTES
#define MICROPYW_HW_FLASH_STORAGE_BYTES (848 * 1024)
#endif

static_assert(MICROPY_HW_FLASH_STORAGE_BYTES % 4096 == 0, "Flash storage size must be a multiple of 4K");

#ifndef MICROPY_HW_FLASH_STORAGE_BASE
#define MICROPY_HW_FLASH_STORAGE_BASE (PICO_FLASH_SIZE_BYTES - MICROPY_HW_FLASH_STORAGE_BYTES)
#endif

#ifndef MICROPYW_HW_FLASH_STORAGE_BASE
#define MICROPYW_HW_FLASH_STORAGE_BASE (PICO_FLASH_SIZE_BYTES - MICROPYW_HW_FLASH_STORAGE_BYTES)
#endif

typedef struct shareddata {
    uint32_t magic;
    uint32_t inst;
    int32_t res;
    uint32_t addr;
    uint32_t size;
    uint32_t crc;
    uint8_t data[1024*4]; // 1kB buffer. Max for one write sequence?
} shareddata_t;

// EOF