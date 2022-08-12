#pragma once

#define BOOTMAGIC (0xDEADBEEF)

typedef struct shareddata {
    uint32_t magic;
    uint32_t inst;
    int32_t res;
    uint32_t rdy;
    uint32_t addr;
    uint32_t crc32;
    uint32_t data[1024];
} shareddata_t;

// EOF