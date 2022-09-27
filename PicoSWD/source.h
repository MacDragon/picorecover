#include <stdio.h>
#include "pico/stdlib.h"
#include "uf2.h"

uint32_t crc32b(unsigned char *data, uint32_t size);
void drawstatus(connectionstatus_t status, const char * statusstr);
