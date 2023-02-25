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

// total 512 blocks.  reserve 64 for program., leaves 1.75MB for storage. 448 blocks.

// pico image max 1024*1024*2-(1408*1024) = 655360 bytes. =  160 blocks allow 150 = 150 = 614400 bytes max image for non W

// pico w image max 1024*1024*2-(848*1024) = 1228800 bytes. = 300 blocks allow 290 = 1187840 bytes max image for W

//  leaves 8 total free

// 150 blocks = 2400 sectors = 4800bytes of address storage, 3 blocks enough for addresses and filename.
// 300 blocks = 4800 sectors = 19200 bytes of address storage. 5 blocks enough.

// all storage allocated.


// to fit both, will need to reduce each by ~ 100k to leave enough room for FW.

#define FLASH_PROGRAMAREA    (64)
#define FLASH_PROGRAMAREABYTES    (FLASH_PROGRAMAREA*4096)

#define FLASH_STORAGE1BLOCKS (150)
#define FLASH_HEADER1BLOCKS (3)
#define FLASH_FILE1STORAGEBYTES (FLASH_STORAGE1BLOCKS*4096)
#define FLASH_FILE1HEADERBYTES    (FLASH_HEADER1BLOCKS*4096)
#define FLASH_FILE1BYTES ((FLASH_STORAGE1BLOCKS+FLASH_HEADER1BLOCKS)*4096)

#define FLASH_STORAGE2BLOCKS (288)
#define FLASH_HEADER2BLOCKS (5)
#define FLASH_FILE2STORAGEBYTES (FLASH_STORAGE2BLOCKS*4096)
#define FLASH_FILE2HEADERBYTES    (FLASH_HEADER2BLOCKS*4096)
#define FLASH_FILE2BYTES ((FLASH_STORAGE2BLOCKS+FLASH_HEADER2BLOCKS)*4096)

static bool erased[FLASH_STORAGE2BLOCKS+FLASH_HEADER2BLOCKS]; // big enough array to hold either.
static bool firsterased;

typedef struct {
  uint8_t * dataread;
  uint8_t * datawrite;
  uint32_t header;
  uint32_t storage;
} filedata_t;

static filedata_t files[2] = {
  {
    .dataread=(uint8_t*)XIP_BASE + FLASH_PROGRAMAREABYTES,
    .header=FLASH_FILE1HEADERBYTES,
    .storage=FLASH_FILE1STORAGEBYTES,
    .datawrite=FLASH_PROGRAMAREABYTES,
  },
  {
    .dataread=(uint8_t*)XIP_BASE + (FLASH_PROGRAMAREABYTES+FLASH_FILE1BYTES),
    .header=FLASH_FILE2HEADERBYTES,
    .storage=FLASH_FILE2STORAGEBYTES,
    .datawrite=FLASH_PROGRAMAREABYTES+FLASH_FILE1BYTES,
  }
};

static filedata_t * activefile = &files[0];

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
void board_flash_init(uint8_t fileno)
{
  // init is already performed by bootloader.
  printf("Resetting flash storage for new upload to file %d\n", fileno);
  activefile = &files[fileno];
  memset(erased, 0, sizeof erased);
  firsterased = false;
  #if 1
  printf("Header data %d\n", fileno);
  DumpHex(activefile->dataread+activefile->header-256, 256);
  printf("Block address data\n");
  DumpHex(activefile->dataread, 384);
  #endif
}

uint32_t board_flash_size(void)
{
  uf2_reset_namestate();
  printf("Getting flash size:%lu\n", activefile->storage);
  return activefile->storage;
}

void board_flash_read(uint32_t addr, void* buffer, uint32_t len, datatype_t area)
{
  char * areatype;
  uint32_t addrint = (uint32_t)activefile->dataread + addr;
  switch ( area )
  {
    case dataarea: addrint+=activefile->header; areatype = "data"; break;
    case headerarea: addrint+=activefile->header-256;  areatype = "header"; break;
    case addressarea: areatype = "address"; break;
  }
  memcpy(buffer, (void*)addrint, len);

#ifdef DEBUGMSG
  uint32_t crc = crc32b(buffer, len);
  printf("Read %s from %08x:%lu addrin %08x crc %08x\n", areatype, addrint, len, addr, crc);
#endif
}

void board_flash_flush(void)
{
  //na, nothing cashed.
}

uint8_t tempdata[4096*4] = {0};

void board_flash_write(uint32_t addr, void const *buffer, uint32_t len, datatype_t area)
{
  uint32_t block = addr >>12;

  if ( area == dataarea && block >= (activefile->storage >>12) ) 
  {
      printf("write data outside storage area %lu %08x!\n", block, addr);
      return;
  }

  if ( area == addressarea && block > 0 ) // >= ( activefile->header >> 12))
  {
      printf("write address outside storage area %lu %08x!\n", block, addr);
      return;
  }

  if ( area == headerarea && block > 0 )
  {
      printf("write header outside storage area %lu %08x!\n", block, addr);
      return;
  }

  switch ( area )
  {
    case dataarea: 
      addr+=activefile->header;
      block+=activefile->header>>12;
      break;
    case headerarea:
      addr+=activefile->header-256;
      break;
    case addressarea: // adjust address block write size to correct size.
      if ( len > activefile->header - 256 )
      {
        len = activefile->header - 256;
      }
    break;
  }

  if ( !erased[block] )
  {
    // if first erase, clear the header address data ready, also flags data as invalid upon first write.
    // else, if quite, data is still intact.
    if ( !firsterased )
    {
      printf("erasing metadata storage\n");
      flash_range_erase(activefile->datawrite, activefile->header);
      firsterased = true;
    }

    uint32_t eraseaddr = activefile->datawrite + 4096*block;
    printf("erasing block %d before write at %08x\n", block, eraseaddr);
    
    erased[block] = true;
    flash_range_erase(eraseaddr, 4096);
  }

  uint32_t crc1=crc32b(buffer, headerarea?len:256);

  flash_range_program(activefile->datawrite + addr, buffer, headerarea?len:256);
  
#ifdef DEBUGMSG
  uint32_t addrread = activefile->dataread + addr;

  if ( len < 4096*4 )
    memcpy(tempdata, (void*)addrread, headerarea?len:256);

  uint32_t crc2=crc32b(tempdata, headerarea?len:256);
  printf("Write to %08x -> %08x %lu, block %d crc in %08x read %08x\n", addr, activefile->datawrite + addr, len, block, crc1, crc2);
#endif
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
