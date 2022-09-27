#include <string.h>
#include <ctype.h>
#include "ring_buffer.hpp"

extern "C" {
#include "source.h"
#include "SEGGER_RTT.h"
#include "hardware/structs/systick.h"
#include "pico_hal.h"
#include "uf2.h"
#include "../wipe/wipe.h"
}
#include "pico_display.hpp"
#include "drivers/st7789/st7789.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"
#include "rgbled.hpp"
#include "button.hpp"
#include "swd.hpp"
#include "source.h"


int32_t probe_flash_uf2( void )
{
    UF2_Block uf2data;

    uint32_t uf2blocks = uf2_get_uf2blockcount();
    uint32_t highaddr = 0;
    uint32_t lowaddr = 0xffffffff;

    printf("Got uf2 data size %d in %d blocks\n", uf2blocks*512, uf2blocks);

    bool baddata = false;
    if (uf2blocks == 0)
    {
        return -1;
    } else for ( int i=0;i<uf2blocks;i++)
    {
        uf2_get_uf2block(i, (uint8_t*)&uf2data);
        if ( uf2data.magicStart0 != UF2_MAGIC_START0
        || uf2data.magicStart1 != UF2_MAGIC_START1
        || uf2data.magicEnd != UF2_MAGIC_END 
        || uf2data.numBlocks != uf2blocks 
        || uf2data.payloadSize > 256 // only allow blocks smaller than a flash write page.
        || uf2data.blockNo != i 
        || ( uf2data.flags & 0x2000 == 0x2000 && uf2data.familyID != PICOFAMILYID )
        )
        {
            printf("Bad data in block %d\n", i);
#if 0
            printf("uf2data.magicStart0 %08x ", uf2data.magicStart0);
        || uf2data.magicStart1 != UF2_MAGIC_START1
        || uf2data.magicEnd != UF2_MAGIC_END 
        || uf2data.numBlocks != uf2blocks 
        || uf2data.payloadSize > 256 // only allow blocks smaller than a flash write page.
        || uf2data.blockNo != i 
        || ( uf2data.flags & 0x2000 == 0x2000 && uf2data.familyID != PICOFAMILYID )
            )
#endif
            return -2;
        }
        uint32_t curaddr = uf2data.targetAddr;
        if ( curaddr > highaddr )
        {
            highaddr = curaddr;
        }
        if ( curaddr < lowaddr )
        {
            lowaddr = curaddr;
        }
    }

    highaddr = (highaddr+256); // actual end of data will be a page later.
    if ( highaddr % 4096 != 0)
        highaddr = highaddr + ( 4096 - highaddr % 4096 ); // pad upto next 4k boundary.
    
    {
        // data verified, start sending!
        printf("UF2 data ok %d blocks, erasing from %08x to %08x\n", uf2blocks, lowaddr, highaddr);

        drawstatus(connected, "Erasing");

        uint32_t erasesize = highaddr-lowaddr;
        
        probe_write_memory(0x20038000+offsetof(shareddata_t, addr), (uint8_t*)&lowaddr, 4);
        probe_write_memory(0x20038000+offsetof(shareddata_t, size), (uint8_t*)&erasesize, 4);
        probe_send_instruction(7);
        uint32_t start = time_us_32();
        int32_t res = probe_wait_reply(20000); // erase could take a while.

        if ( res != 1 )
        {
            printf("data erase failed %d\n", res);
            drawstatus(connected, "erase fail");
            return -3;
        } else
        {
            printf("erase took %dms\n", (time_us_32()-start)/1000);
            int32_t lastperc = -1;
            bool error = false;

            for ( int i=0;i<uf2blocks;i++)
            {
                uf2_get_uf2block(i, (uint8_t*)&uf2data);
                int32_t perc = ( ( 100000/(uf2blocks-1)) * i) / 1000;

                if ( perc != lastperc )
                {
                    lastperc = perc;
                    char str[32];
                    snprintf(str, sizeof str, "flash %d%%", perc);
                    // display % status here.
                    drawstatus(connected, str);
                    printf("Progress %d%%\n", perc);
                }

                if ( uf2data.flags & 0x1 != 0 )
                {
                    printf("UF2 block %d not for flash, ignoring", i);
                    continue;
                }
                //printf("UF2 block %d, %08x:%d(%03x)\n", i, uf2data[i].targetAddr, uf2data[i].payloadSize, uf2data[i].payloadSize);

                uint32_t sendsize = uf2data.payloadSize; // uf2 block size.
                uint32_t crccalc = crc32b(uf2data.data, sendsize);
                //printf("Data send %dB crc32 %08x\n", sendsize, crc);
                probe_write_memory(0x20038000+offsetof(shareddata_t, addr), (uint8_t*)&uf2data.targetAddr, 4);
                probe_write_memory(0x20038000+offsetof(shareddata_t, data), uf2data.data, sendsize);
                probe_write_memory(0x20038000+offsetof(shareddata_t, size), (uint8_t*)&sendsize, 4);
                probe_write_memory(0x20038000+offsetof(shareddata_t, crc), (uint8_t*)&crccalc, 4);
                probe_send_instruction(6);
                int32_t res = probe_wait_reply(1000);
                int32_t crcread = 0;
                probe_read_memory( 0x20038000+offsetof(shareddata_t, crc), (uint8_t*)&crcread, sizeof crcread);
                crcread = ~crcread;
                if ( res == 1 && crcread == crccalc )
                    continue; //printf("data sent ok\n");
                else
                {
                    printf("data send error %d, %04x %04x on block %d\n", res, crcread, crccalc, i);
                    error = true;
                    return -4;
                }
                printf("unknown state\n");
                break;
            }  

            if ( lastperc != 100 && !error )
            {
                drawstatus(connected, "flash 100%");
                printf("Progress 100%%\n");
                printf("data sent ok took %d\n", 0);
                return 1;
            }
        }
    }
    return -5; // shouldn't be here, unknown failure.
}