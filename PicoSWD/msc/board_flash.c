/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "board_api.h"
#include "tusb.h" // for logging
#include "hardware/flash.h"
#include "uf2.h"

#define FLASH_STORAGEBLOCKS (384) // 1.5MB. 6144 256byte pages   6144*4 = 24576bytes for addresses. = 6 4096k pages.
#define FLASH_ADDRESSBLOCKS (6+1) //  enough blocks to store all addresses + a header for metadata.

#define FLASH_STORAGE_SIZE (4096*(FLASH_STORAGEBLOCKS+FLASH_ADDRESSBLOCKS)) // must be flash sector size 256 aligned and whole number of flash erase sectors for simplicity

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_STORAGE_SIZE)

static bool erased[FLASH_ADDRESSBLOCKS+FLASH_STORAGEBLOCKS];
static bool firsterased;

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
void board_flash_init(void)
{
  // init is already performed by bootloader.
  printf("Resetting flash storage for new upload\n");
  memset(erased, 0, sizeof erased);
  firsterased = false;
  uint8_t *data=(uint8_t*)XIP_BASE + PICO_FLASH_SIZE_BYTES - FLASH_STORAGE_SIZE;
  #if 0
  printf("Header data\n");
  DumpHex(data+4096*6, 256);
  printf("Block address data\n");
  DumpHex(data, 384);
  #endif
  #if 0
  printf("Setting test data\n");
  flash_range_erase(FLASH_TARGET_OFFSET, FLASH_STORAGE_SIZE);

  for ( int i=0;i<16;i++)
  {
    char data[256] = { 0 };
    sprintf(data, "Testing %d", i);
    flash_range_program(FLASH_TARGET_OFFSET + FLASH_PAGE_SIZE*i, data, 256);
  }
  #endif
}

uint32_t board_flash_size(void)
{
  uf2_reset_namestate();
  printf("Getting flash size:%lu\n", FLASH_STORAGE_SIZE);
  return FLASH_STORAGE_SIZE;
}

void board_flash_read(uint32_t addr, void* buffer, uint32_t len, bool header)
{
  printf("Read from %lu:%lu\n", addr, len);
  addr = XIP_BASE + PICO_FLASH_SIZE_BYTES - FLASH_STORAGE_SIZE + addr;
  if ( !header )
    addr+=(FLASH_ADDRESSBLOCKS*4096);
  memcpy(buffer, (void*)addr, len);
}

void board_flash_flush(void)
{
  //na, nothing cashed.
}

void board_flash_write(uint32_t addr, void const *data, uint32_t len, bool header)
{
    uint32_t block = addr >>12;

    if ( ( !header && block >= FLASH_STORAGEBLOCKS ) || ( header && block >= FLASH_ADDRESSBLOCKS) )
    {
        printf("write outside storage area!\n");
        return;
    }

    if (!header)
    {
      block+=FLASH_ADDRESSBLOCKS;
      addr+=(FLASH_ADDRESSBLOCKS*4096);
    }

    if ( !erased[block] )
    {
      // if first erase, clear the header address data ready, also flags data as invalid upon first write.
      // else, if quite, data is still intact.
      if ( !firsterased )
      {
        printf("erasing metadata storage\n");
        flash_range_erase(FLASH_TARGET_OFFSET, 4096*FLASH_ADDRESSBLOCKS);
        firsterased = true;
      }

      uint32_t eraseaddr = FLASH_TARGET_OFFSET + 4096*block;
      printf("erasing block %d before write at %08x\n", block, eraseaddr);
      erased[block] = true;
      flash_range_erase(FLASH_TARGET_OFFSET + 4096*block, 4096);
    }

    printf("Write to %08x:%lu, block %d\n", addr, len, block);
    {
        flash_range_program(FLASH_TARGET_OFFSET + addr, data, header?len:256);
    }
}

void board_flash_erase_app(void)
{
  // TODO implement later
}

bool board_flash_protect_bootloader(bool protect)
{
  // TODO implement later
  (void) protect;
  return false;
}

#ifdef TINYUF2_SELF_UPDATE
void board_self_update(const uint8_t * bootloader_bin, uint32_t bootloader_len)
{
  (void) bootloader_bin;
  (void) bootloader_len;
}
#endif
