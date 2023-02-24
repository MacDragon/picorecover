#include <string.h>
#include <ctype.h>
#include "ring_buffer.hpp"

extern "C" {
#include "source.h"
//#include "SEGGER_RTT.h"
#include "hardware/structs/systick.h"
#include "../wipe/littlefs-lib/pico_hal.h"
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
        
        probe_write_memory(0x20038000+offsetof(shareddata_t, inst[0])+offsetof(instruction_t, addr), (uint8_t*)&lowaddr, 4);
        probe_write_memory(0x20038000+offsetof(shareddata_t, inst[0])+offsetof(instruction_t, size), (uint8_t*)&erasesize, 4);
        probe_send_instruction(7, 0);
        uint32_t start = time_us_32();
        int32_t res = probe_wait_reply(20000, 0); // erase could take a while.

        if ( res != 1 )
        {
            printf("data erase failed %d\n", res);
            drawstatus(connected, "erase fail");
            return -3;
        } else
        {
            printf("erase took %dms\n", (time_us_32()-start)/1000);
            start = time_us_32();
            int32_t lastperc = -1;
            bool error = false;

            uint32_t startwrite = 0;
            uint32_t startblock = 0;
            uint32_t writeend = 0;

            uint32_t crccalc[2] = {0};
#define INTERLEAVE

#ifdef INTERLEAVE
            for ( int i=0;i<=uf2blocks;i++)
#else   
            for ( int i=0;i<uf2blocks;i++)
#endif
            {
                #ifdef INTERLEAVE
                if ( i < uf2blocks ) // send a block if last not sent.
                #endif
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
                        printf("Progress %d%% transfer %dus write %dus\n", perc,(startwrite-startblock),(writeend-startwrite));
                    }

                    if ( uf2data.flags & 0x1 != 0 )
                    {
                        printf("UF2 block %d not for flash, ignoring", i+1);
                        continue;
                    }

                    uint32_t sendsize = uf2data.payloadSize; // uf2 block size.

                    #ifdef INTERLEAVE
                    uint8_t instr = i % 2;
                    #else
                    uint8_t instr = 0;
                    #endif
                    crccalc[instr] = crc32b(uf2data.data, sendsize);
                    //printf("Data send %dB crc32 %08x at \n", sendsize, crccalc[instr]);
#ifdef DEBUGMSG
                    printf("UF2 block %d, %08x:%d crc %08x\n", i+1, uf2data.targetAddr, uf2data.payloadSize, crccalc[instr] );
#endif
                    startblock = time_us_32();

                    sleep_us(100);

                    instruction_t cmd;
                    uint32_t instroffset = offsetof(shareddata_t, inst) + sizeof cmd*instr;
                    //printf("writing instruction %d\n", instr);
                    probe_write_memory(0x20038000+instroffset+offsetof(instruction_t, addr), (uint8_t*)&uf2data.targetAddr, 4);
                    probe_write_memory_slow(0x20038000+instroffset+offsetof(instruction_t, data), uf2data.data, sendsize);
                    probe_write_memory(0x20038000+instroffset+offsetof(instruction_t, size), (uint8_t*)&sendsize, 4);
                    probe_write_memory(0x20038000+instroffset+offsetof(instruction_t, crc), (uint8_t*)&crccalc[instr], 4);
                    probe_send_instruction(6, instr);
                    startwrite = time_us_32();
                }
#ifdef INTERLEAVE
                if ( i > 0 ) // check previous block result if not first write
                {
                    uint8_t checkinstr = (i-1) % 2;
#else
                {
                    uint8_t checkinstr = 0;
#endif
                    //printf("waiting on instruction %d\n", checkinstr);
                    int32_t res = probe_wait_reply(1000, checkinstr);
                    writeend = time_us_32();
                    
                    instruction_t cmd;
                    uint32_t instroffset = offsetof(shareddata_t, inst) + sizeof cmd*checkinstr;
                    
                    int32_t crcread = 0;
                    probe_read_memory( 0x20038000+instroffset+offsetof(instruction_t, crc), (uint8_t*)&crcread, sizeof crcread);
                    crcread = ~crcread;
                    if ( res == 1 && crcread == crccalc[checkinstr] )
                    {
                        if ( i == uf2blocks ) // last block was checked.
                        {
                            drawstatus(connected, "flash 100%");
                            printf("Progress 100%%\n");
                            printf("data sent ok took %dms\n", (time_us_32()-start)/1000);
                            return 1;
                        }
                        continue; //printf("data sent ok\n");
                    }
                    else
                    {
                        printf("data send error %d, %04x %04x on block %d\n", res, crcread, crccalc, i);
                        error = true;
                        return -4;
                    }
                    printf("unknown state\n");
                    break;
                }
                printf("First block no check\n");
                continue;
            }  

            if ( lastperc != 100 && !error )
            {
                drawstatus(connected, "flash 100%");
                printf("Progress 100%%\n");
                printf("data sent ok took %dms\n", (time_us_32()-start)/1000);
                return 1;
            }
        }
    }
    return -5; // shouldn't be here, unknown failure.
}