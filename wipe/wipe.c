/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Obliterate the contents of flash. This is a silly thing to do if you are
// trying to run this program from flash, so you should really load and run
// directly from SRAM. You can enable RAM-only builds for all targets by doing:
//
// cmake -DPICO_NO_FLASH=1 ..
//
// in your build directory. We've also forced no-flash builds for this app in
// particular by adding:
//
// pico_set_binary_type(flash_nuke no_flash)
//
// To the CMakeLists.txt app for this file. Just to be sure, we can check the
// define:
#if !PICO_NO_FLASH
#error "This example must be built to run from SRAM!"
#endif

#if PICO_NO_FLASH
#include "pico/bootrom.h"
#include "pico/stdlib.h"

#include "hardware/structs/ioqspi.h"
#include "hardware/structs/ssi.h"
#else
#include "hardware/flash.h"
#endif

//#include "pico/stdio.h"

//#include "stdio.h"

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/bootrom.h"
#include "pico_hal.h"
#include "wipe.h"

#include "boot2.h"


// https://web.archive.org/web/20190108202303/http://www.hackersdelight.org/hdcodetxt/crc.c.txt

// ----------------------------- crc32b --------------------------------

/* This is the basic CRC-32 calculation with some optimization but no
table lookup. The the byte reversal is avoided by shifting the crc reg
right instead of left and by using a reversed 32-bit word to represent
the polynomial.
   When compiled to Cyclops with GCC, this function executes in 8 + 72n
instructions, where n is the number of bytes in the input message. It
should be doable in 4 + 61n instructions.
   If the inner loop is strung out (approx. 5*8 = 40 instructions),
it would take about 6 + 46n instructions. */

uint32_t crc32b(unsigned char *data, uint32_t size) {
   int i, j;
   unsigned int byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (i < size) {
      byte = data[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}


// from micropython rp2 port
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)

#ifndef MICROPY_HW_FLASH_STORAGE_BYTES
#define MICROPY_HW_FLASH_STORAGE_BYTES (1408 * 1024)
#endif
static_assert(MICROPY_HW_FLASH_STORAGE_BYTES % 4096 == 0, "Flash storage size must be a multiple of 4K");

#ifndef MICROPY_HW_FLASH_STORAGE_BASE
#define MICROPY_HW_FLASH_STORAGE_BASE (PICO_FLASH_SIZE_BYTES - MICROPY_HW_FLASH_STORAGE_BYTES)
#endif

// copy paste in direct snipped of sdk code, via https://forums.raspberrypi.com/viewtopic.php?t=332590

#if PICO_NO_FLASH
static void flash_start( void )
{
    rom_connect_internal_flash_fn connect_internal_flash =
        (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip =
        (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flash_flush_cache =
        (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    rom_flash_enter_cmd_xip_fn flash_enter_cmd_xip =
        (rom_flash_enter_cmd_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_ENTER_CMD_XIP);
    connect_internal_flash();
    flash_exit_xip();
    flash_flush_cache();
    flash_enter_cmd_xip();
}
#endif

shareddata_t exchange __attribute__((section(".shared")));

int recoverfile( char * filename )
{
    int res = 0;
    char filename1[32]="/boot.py";
    char filename2[32]="/boot_recover.py";

    for ( int i=0;i<4;i++)
    {
        filename1[i+1] = filename[i];
        filename2[i+1] = filename[i];
    }

    int file = pico_open(filename1, 0);
    int file2 = pico_open(filename2, 0);

    exchange.data[1] = file;
    exchange.data[2] = file2;

    if ( file > 0 ) pico_close(file); // only used to check if recovery exists.

    if ( file2 > 0 ) pico_close(file2);

    bool delete = false;

    if ( file2 > 0 )
        delete = true;

    if ( file > 0 )
    {
        if ( file2 < 0 )
        {
           // printf("boot.py found but no boot_recover, renaming\n");
            if( pico_rename(filename1, filename2) < 0 )
            {
               // printf("boot.py couldn't be renamed, deleting\n");
                delete = true;
                // rename failed, just delete the file instead.
            } else
                res = 1;
            //else
              //  printf("boot.py renamed to boot_recover.py\n");
        }

        if ( delete )
        {
            if ( pico_remove(filename1) < 0 )
                res = -1;
            else
                res = 2;
        }
    } else
        res = -2; 
    //else
        //  printf("boot.py not found %d\n", file);
    return res;
}

void flasherror( void )
{
    for ( int i=0;i<3;i++)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
    }
}

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    flasherror();

    flash_start();

    exchange.magic = 0;

    memset(exchange.data, 0, sizeof exchange.data);

    // check for and run boot2 if it exists in flash?

    // can't read flash from a NO FLASH program, but you can write and erase it.

    // else assume standard pico and run our own.

    // W25Q16JV

    exchange.inst = 0;

    bool filesystemok = false;

    int res = pico_mount(false);

    if ( res == LFS_ERR_OK)
    {
        exchange.res = 0xabcd; // flag that filesystem was found.
        filesystemok = true;
    } else
    {
        flasherror();
        exchange.res = res;
    }

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    exchange.magic = BOOTMAGIC;

    while ( 1 )
    {
        if ( exchange.inst != 0 )
        {
            uint32_t current_inst = exchange.inst;
            exchange.inst = 0;
            exchange.res = 0;

            switch ( current_inst )
            {
                case 1 : // boot.py recover
                    if ( !filesystemok )
                    {
                        flasherror();
                        exchange.res = -1; 
                    } else
                        exchange.res = recoverfile("boot");
                    break;
                case 2 : // main.py recover
                    if ( !filesystemok )
                    {
                        flasherror();
                        exchange.res = -1; 
                    } else
                        exchange.res = recoverfile("main");
                    break;
                    break;
                case 3 : // wipe filesystem area
                    uint flash_size_bytes;
                    #ifndef PICO_FLASH_SIZE_BYTES
                    #warning PICO_FLASH_SIZE_BYTES not set, assuming 16M
                        flash_size_bytes = 16 * 1024 * 1024;
                    #else
                        flash_size_bytes = MICROPY_HW_FLASH_STORAGE_BYTES;
                    #endif
                        flash_range_erase(MICROPY_HW_FLASH_STORAGE_BASE, flash_size_bytes);
                    break;
                case 4 : 
                    // Pop back up as an MSD drive
                    reset_usb_boot(0, 0);
                    break;
                case 5 :
                    flash_range_erase(0, 4096); // erase boot sector only.
                    exchange.res = 1;
                    break;
                case 6 :
                    // receive data.
                    if ( exchange.size > 1024 || exchange.size == 0 )
                        exchange.res = -2;
                    else 
                    {
                        uint32_t crc32 = crc32b(exchange.data, exchange.size);
                        if ( crc32 == exchange.crc32 )
                        {
                            exchange.crc32 = ~crc32;
                            exchange.res = 1;
                        }
                        else
                        {
                            exchange.crc32 = ~crc32;
                            exchange.res = -1;
                        }
                    }
                    break;
                case 0xff :
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                    sleep_ms(100);
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    sleep_ms(100);
                    break;
            }
        }
    }
}
