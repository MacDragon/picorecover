#pragma once

#define BOOTMAGIC (0xDEADBEEF)

typedef struct shareddata {
    uint32_t magic;
    uint32_t inst;
    int32_t res;
    uint32_t addr;
    uint32_t size;
    uint32_t crc32;
    uint8_t data[1024*4]; // 1kB buffer. Max for one write sequence?
} shareddata_t;

// EOF