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


//CRC-16 (Modbus) table

//https://community.nxp.com/t5/Kinetis-Microcontrollers/Cortex-A-vs-Cortex-M-Differents-CRC-output-calculation/m-p/484486/highlight/true

static short crc_tab16[256] =

{

        0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,

        0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,

        0xC481,0x0440,0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,

        0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,

        0xC901,0x09C0,0x0880,0xC841,0xD801,0x18C0,0x1980,

        0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,

        0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,0x1400,

        0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,

        0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,

        0x1040,0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,

        0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,

        0x35C0,0x3480,0xF441,0x3C00,0xFCC1,0xFD81,0x3D40,

        0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,

        0xFB41,0x3900,0xF9C1,0xF881,0x3840,0x2800,0xE8C1,

        0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,

        0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,

        0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,

        0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,

        0x2080,0xE041,0xA001,0x60C0,0x6180,0xA141,0x6300,

        0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,

        0xA501,0x65C0,0x6480,0xA441,0x6C00,0xACC1,0xAD81,

        0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,

        0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,0x7800,

        0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,

        0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,

        0x7C40,0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,

        0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,

        0x71C0,0x7080,0xB041,0x5000,0x90C1,0x9181,0x5140,

        0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,

        0x9741,0x5500,0x95C1,0x9481,0x5440,0x9C01,0x5CC0,

        0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,

        0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,

        0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,

        0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,

        0x4C80,0x8C41,0x4400,0x84C1,0x8581,0x4540,0x8701,

        0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,

        0x4100,0x81C1,0x8081,0x4040

};


unsigned short update_crc16( unsigned short crc, char c ) {

  unsigned short tmp, short_c;

  short_c = 0x00ff & (unsigned short) c;

  tmp =  crc       ^ short_c;

  crc = (crc >> 8) ^ crc_tab16[ tmp & 0xff ];

  return crc;

}

unsigned short crc16(char* data, int len)

{

    unsigned short crc=0xFFFF;// crc_16_modbus;

    int i;

    for(i=0;i<len;i++)

    {

        crc = update_crc16(crc, data[i] );

    }

    return crc;

}

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
                {
                    if ( exchange.addr < 0x10000000 || exchange.addr + exchange.size > 0x10000000 + 2048*1024 || exchange.size % 256 != 0 || exchange.addr % 256 != 0)
                    {
                        exchange.res = -3;
                    }

                    uint32_t addr = exchange.addr - 0x10000000;

                    // receive data.
                    if ( exchange.size > 1024 || exchange.size == 0 )
                        exchange.res = -2;
                    else 
                    {
                        uint32_t crc = crc32b(exchange.data, exchange.size);
                        //uint32_t crc =  crc16(exchange.data, exchange.size);
                        if ( crc == exchange.crc )
                        {
                            exchange.crc = ~crc;
                            flash_range_program( addr, exchange.data, exchange.size);
                            // write flash!
                            exchange.res = 1;
                        }
                        else
                        {
                            exchange.crc = ~crc;
                            exchange.res = -1;
                        }
                    }
                    break;
                }
                case 7 :
                {
                    if ( exchange.addr < 0x10000000 || exchange.addr + exchange.size > 0x10000000 + 2048*1024 || exchange.size % 4096 != 0 || exchange.addr % 4096 != 0)
                    {
                        exchange.res = -2;
                    }
                    uint32_t addr = exchange.addr - 0x10000000;
                    flash_range_erase(addr, exchange.size); // erase boot sector only.
                    exchange.res = 1;
                    break;
                }
                case 0xff :
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                    sleep_ms(100);
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    sleep_ms(100);
                    exchange.res = 1;
                    break;
            }
        }
    }
}
