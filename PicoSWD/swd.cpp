#include <string.h>
#include <ctype.h>
#include "ring_buffer.hpp"

extern "C" {
#include "source.h"
//#include "SEGGER_RTT.h"
#include "hardware/structs/systick.h"
#include "../wipe/helper.h"
#include "../wipe/littlefs-lib/pico_hal.h"
#include "../wipe/wipe.h"
#include "uf2.h"
}

#define PROBE_PIN_OFFSET 2
#define PROBE_PIN_SWCLK PROBE_PIN_OFFSET + 0 // 2
#define PROBE_PIN_SWDIO PROBE_PIN_OFFSET + 1 // 3
#define PROBE_PIN_DIR   PROBE_PIN_OFFSET + 2 // 3

const uint PROBE_SWDCLK = PROBE_PIN_SWCLK;
const uint PROBE_SWDIO = PROBE_PIN_SWDIO;
const uint PROBE_DIR = PROBE_PIN_DIR;

#define CYCLES1 1

inline void cycledelay()
{
    //for ( int i=0;i<cycles;i++)
        asm("NOP"); // two nops appears to give a big enough delay to work. approx 3x speedup to uploading.
        asm("NOP");
}

void probe_high(void)
{
    gpio_put(PROBE_SWDIO, 1);
    //cycledelay();  
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay();  
    gpio_put(PROBE_SWDCLK, 0); 
    cycledelay();     
}

void probe_low(void)
{
    gpio_put(PROBE_SWDIO, 0);
    //cycledelay(); 
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay(); 
    gpio_put(PROBE_SWDCLK, 0); 
    cycledelay();  
}

void probe_turn(void)
{
    //cycledelay(); 
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay(); 
    gpio_put(PROBE_SWDCLK, 0); 
    cycledelay(); 
}

bool probe_read(void)
{
    cycledelay();
    bool res = gpio_get(PROBE_SWDIO);
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay();
    gpio_put(PROBE_SWDCLK, 0);
    //cycledelay();
    return res;  
}

void probe_sendbits(const uint8_t * data, int bits)
{
    int bit = 0;
    int j=0;
    gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
    for ( int i=0;i<bits;i++)
    {
        uint output = data[j]>>bit & 1;
        gpio_put(PROBE_SWDIO, output);
        sleep_us(1);
        gpio_put(PROBE_SWDCLK, 1);
        sleep_us(1);
        //printf("Outputting bit %d of %d\n", i, output);
        bit++;
        if ( bit == 8 )
        {
            j++;
            bit = 0;
        }
        gpio_put(PROBE_SWDCLK, 0); // clock out the bit.
        sleep_us(1);
    }
    //gpio_put(PROBE_SWDCLK, 0); // clock out the bit.
    //gpio_put(PROBE_SWDIO, 0); 
}


// sends 32bit data value + parity. No turns.
void probe_send32bit(const uint32_t data )
{
    uint32_t parity = 0;
    gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
    for ( int bit=0;bit<32;bit++)
    {
        uint output = data>>bit & 1;
        parity += output;
        gpio_put(PROBE_SWDIO, output);
        sleep_us(1);
        gpio_put(PROBE_SWDCLK, 1);
        sleep_us(1);
        //printf("Outputting bit %d of %d\n", i, output);
        gpio_put(PROBE_SWDCLK, 0); // clock out the bit.
        sleep_us(1);
    }
    gpio_put(PROBE_SWDIO, parity & 1);
    sleep_us(1);
    // calculate the parity bit and write it
    gpio_put(PROBE_SWDCLK, 1);
    sleep_us(1);
    //printf("Outputting bit %d of %d\n", i, output);
    gpio_put(PROBE_SWDCLK, 0); // clock out the bit.
    sleep_us(1);
}

void probe_readbits(int bits)
{
    gpio_set_dir(PROBE_SWDIO, GPIO_IN);

    int bit = 0;
    int j=0;
    for ( int i=0;i<bits;i++)
    {
        gpio_put(PROBE_SWDCLK, 1);
        gpio_get(PROBE_SWDIO);
        sleep_us(1);
        //printf("Outputting bit %d of %d\n", i, output);
        bit++;
        if ( bit == 8 )
        {
            j++;
            bit = 0;
        }
        gpio_put(PROBE_SWDCLK, 0); // clock out the bit.
        sleep_us(1);
    }
    gpio_put(PROBE_SWDIO, 0); 
}

const uint8_t dormantwake[] = {
    0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
    0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19
};

const uint32_t dp_core0 = 0x01002927;  //  # ID to select core 0 and 1
const uint32_t dp_core1 = 0x11002927;
const uint32_t dp_rescue = 0xf1002927;

typedef enum _resultcode { E_FAULT, E_OK, E_PARITY, E_WAIT } resultcode;

typedef enum _AHBAPRegs
{
    ahbap_rw_CSW = 0x00,
    ahbap_rw_TAR = 0x04,
    ahbap_rw_DRW = 0x0C,
    ahbap_rw_BANKED0 = 0x10,
    ahbap_rw_BANKED1 = 0x14,
    ahbap_rw_BANKED2 = 0x18,
    ahbap_rw_BANKED3 = 0x1C,
    ahbap_ro_DbgROMAddr= 0xF8,
    ahbap_ro_ID=  0xFC,
} AHBAPRegs;

typedef enum _SWDPRegs
{
    swdpreg_ro_IDCODE= 0, 
    swdpreg_wo_ABORT= 0,
    swdpreg_rw_CSR= 4, 
    swdpreg_rw_WCR= 5,
    swdpreg_ro_RESEND= 8, 
    swdpreg_wo_SELECT= 8,
    swdpreg_ro_RDBUFF = 12,
} SWDPRegs;

void probe_setdir(int dir)
{
    if ( dir )
    {
        gpio_put(PROBE_DIR, dir);
        gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
        while (!gpio_is_dir_out(PROBE_SWDIO));
    }
    else
    {
        gpio_put(PROBE_DIR, dir);
        gpio_set_dir(PROBE_SWDIO, GPIO_IN);
        while (gpio_is_dir_out(PROBE_SWDIO));
    }
}

#define CDRADDR_DHCSR
#define CDRADDR_DCRSR
#define CDRADDR_DCRDR
#define CDRADDR_DEMCR
typedef enum _ES_CrDbg_CrRegs
{
   crreg_R0 = 0,
   crreg_R1 = 1,
   crreg_R2 = 2,
   crreg_R3 = 3,
   crreg_R4 = 4,
   crreg_R5 = 5,
   crreg_R6 = 6,
   crreg_R7 = 7,
   crreg_R8 = 8,
   crreg_R9 = 9,
   crreg_R10 = 10,
   crreg_R11 = 11,
   crreg_R12 = 12,
   crreg_SP = 13,
   crreg_LR = 14,
   crreg_DbgRetAddr = 15,
   crreg_xPSR = 16,
   crreg_MSP = 17,
   crreg_PSP = 18,
   crreg_SFR4in1 = 20,
} ES_CrDbg_CrRegs;


typedef enum _CSRbits
{
    ORUNDETECT = 0,
    STICKYORUN = 1,
    TRNMODE = 2, // 2 bits
    STICKYCMP = 4,
    STICKYERR = 5,
    READOK = 6,
    WDATAERR = 7,
    MASKLANE = 8,
    TRNCNT = 12, // [23:12] 12 bits
    CDBGRSTREQ = 26,
    CDBGRSTACK = 27,
    CDBGPWRUPREQ = 28,
    CDBGPWRUPACK = 29,
    CSYSPWRUPREQ = 30,
    CSYSPWRUPACK = 31,  
} CSRbits;

typedef enum _CSWbits
{
    MEMAPSIZE = 0, // 3 bits
    ADDRINC = 4, // 2 bits
    APDEVICEEN = 6,
    APPROT = 24, // 7 bits, check RPI datasheet.
    DBGSWENABLE = 31,

    APSIZE32BIT = 2,
    ADDRINCPACKETEN = 2,
    ADDRINCSINGLEEN = 1,
} CSWbits;

typedef enum _REGISTERS 
{
    DHCSR = 0xE000EDF0,
    DCRSR = 0xE000EDF4,
    DCRDR = 0xE000EDF8,
} REGISTERS;

resultcode probe_read_reg(bool dp, uint32_t reg, uint32_t * readval)
{
    gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
    gpio_put(PROBE_DIR, 0);
    uint32_t parity = 1;
    probe_high(); // start
    if ( dp )
    {
        probe_low(); // APnDP DP
    } else
    {
        probe_high();
        parity++;
    }
    probe_high(); // RnW R

    if ( reg & ( 1 << 2 ) )// A[2:3] // reg 0 IDR
    {
        probe_high();
        parity++;
    } else
        probe_low(); 
    if ( reg & ( 1 << 3 ) )// A[2:3] // reg 0 IDR
    {
        probe_high();
        parity++;
    } else
        probe_low(); 

    if(parity & 0x01) // parity write
        probe_high();
    else
        probe_low();

    probe_low();  // stop
    probe_high(); // park

    gpio_set_dir(PROBE_SWDIO, GPIO_IN);
    gpio_put(PROBE_DIR, 0);
    probe_turn();

    bool ok = probe_read();
    bool wait = probe_read();
    bool fault = probe_read();

    if ( !ok && wait && !fault )
    {
        printf("waitr\n");
        return E_WAIT;
    }

    if ( ok && !wait && !fault )
    {
        //probe_turn();

        *readval = 0;
        uint32_t parity = 0;
        for ( int i=0;i<32;i++){
            bool val = probe_read();
            *readval |= (val << i);
            parity += val;
        }

        parity+=probe_read();

        gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
        gpio_put(PROBE_DIR, 0);
        probe_turn();

        if (parity & 1)
        {
           printf("parity error\n"); 
           return E_PARITY;
        }

        for ( int i=0;i<8;i++)
            probe_low();  // idle
        return E_OK;
    } else
        printf("read error: %d %d %d\n", ok, wait, fault); 
    return E_FAULT;
}

bool probe_read_dp(uint32_t reg, uint32_t * readval)
{
    return probe_read_reg(true, reg, readval) == E_OK?true:false;
}

bool probe_read_ap(uint32_t reg, uint32_t * readval)
{
    return probe_read_reg(false, reg, readval) == E_OK?true:false;
}

resultcode probe_write_reg(bool dp, uint32_t reg, uint32_t writeval)
{
    gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
    gpio_put(PROBE_DIR, 0);
    uint32_t parity = 0;
    probe_high(); // start
    if ( dp )
    {
        probe_low(); // APnDP DP
    } else
    {
        probe_high();
        parity += 1;
    }
    probe_low(); // RnW W

    if ( reg & ( 1 << 2 ) )// A[2:3] // reg 0 IDR
    {
        probe_high();
        parity += 1;
    } else
        probe_low(); 
    if ( reg & ( 1 << 3 ) )// A[2:3] // reg 0 IDR
    {
        probe_high();
        parity++;
    } else
        probe_low(); 

    if(parity & 0x01) // parity write
        probe_high();
    else
        probe_low();

    probe_low();  // stop
    probe_high(); // park

    gpio_set_dir(PROBE_SWDIO, GPIO_IN);
    gpio_put(PROBE_DIR, 0);
    probe_turn();

    bool ok = probe_read();
    bool wait = probe_read();
    bool fault = probe_read();

    if ( !ok && wait && !fault )
    {
        printf("waitw\n");
        return E_WAIT;
    }

    if ( ok && !wait && !fault )
    {
        gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
        gpio_put(PROBE_DIR, 1);
        probe_turn();

        uint32_t parity = 0;
        for ( int i=0;i<32;i++){
            if ( writeval & (1 << i) )
            {
                parity++;
                probe_high();
            }
            else
                probe_low();
        }

        if(parity & 0x01) // parity write
            probe_high();
        else
            probe_low();

        for ( int i=0;i<8;i++)
            probe_low();  // idle
        return E_OK;
    } else
        printf("error: %d %d %d\n", ok, wait, fault); 
    return E_FAULT;
}

bool probe_write_dp(uint32_t reg, uint32_t writeval)
{
    resultcode res;

    res = probe_write_reg(true, reg, writeval);

    if ( res == E_WAIT){
        //probe_clear_error();
        printf("Wait error on write, abort and resend");
        probe_write_dp(swdpreg_wo_ABORT, 0x1E);
        res = probe_write_reg(true, reg, writeval);
    }

    return res = E_OK?E_OK:E_FAULT;
}


bool probe_write_ap(uint32_t reg, uint32_t writeval)
{
    resultcode res = probe_write_reg(false, reg, writeval);

    if ( res == E_OK)
        return true;
    else if ( res == E_WAIT )
    {
        // wait a bit for bus to hopefully clear
        sleep_us(1000);
        res = probe_write_reg(false, reg, writeval);
        if ( res == E_OK)
            return true;
        printf("write resend failed\n");
    }

    return false;
}

/*
https://forums.raspberrypi.com/viewtopic.php?t=301893
 I think our assumption was that people would use the B4.3.4 sequence, which you have tried:
1.Perform a line reset. See Figure B4-9 on page B4-125.
2.Write to DP register 0xC, TARGETSEL, where the data indicates the selected target.
The target response must be ignored. See Figure B4-9 on page B4-125.
3.Read from the DP register 0x0, DPIDR, to verify that the target has been successfully selected
*/

bool initDebug( uint32_t core )
{
    // wake dormant

    for ( int i=0;i<8;i++) // high 8
        probe_high();

    probe_sendbits(dormantwake, 128);

    for ( int i=0;i<4;i++) // 4 idle bits.
        probe_low();

    gpio_put(PROBE_DIR, 0);

    uint8_t activation_code = 0x1a; //Select SWD protocol.
    probe_sendbits(&activation_code, 8); // looks correct

    gpio_put(PROBE_DIR, 1);

    // clock 50+ high

    for ( int i=0;i<56;i++)
        probe_high();

    for ( int i=0;i<8;i++)
        probe_low();

//B4-9 DP targetsel write, ignoring ok/ack/fault, will be no reply.
    uint8_t targetselwrite = 0b10011001;
    probe_sendbits(&targetselwrite, 8);

    gpio_set_dir(PROBE_SWDIO, GPIO_IN); // set undriven for 5 cycles.
    gpio_put(PROBE_DIR, 0);
    probe_turn();
    probe_turn();
    probe_turn();
    probe_turn();
    probe_turn(); // one sent by SWDWrDataOnly, was causing no reply

    // write device ID + parity.
    probe_send32bit(core);
    probe_turn();

    for ( int i=0;i<8;i++) // send idle.
        probe_low();

    // req accessport read idr register
    probe_high(); // start
    probe_low(); // APnDP DP
    probe_high(); // RnW R
    probe_low(); // A[2:3] // reg 0 IDR
    probe_low();
    probe_high(); // parity
    probe_low();  // stop
    probe_high(); // park

    gpio_set_dir(PROBE_SWDIO, GPIO_IN);
    gpio_put(PROBE_DIR, 0);
    probe_turn();

    bool ok = probe_read();
    bool wait = probe_read();
    bool fault = probe_read();

    if ( ok && !wait && !fault )
    {
        uint32_t readval = 0;
        uint32_t parity = 0;
        for ( int i=0;i<32;i++){
            bool val = probe_read();
            readval |= (val << i);
            parity += val;
        }

        parity+=probe_read();

        gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
        gpio_put(PROBE_DIR, 0);
        probe_turn();

        if (parity & 1)
        {
           printf("parity error\n"); 
           return false;
        }

        for ( int i=0;i<8;i++) // send idle.
            probe_low();

        if ( readval == 0x10212927 )
        {
            printf("Got Rescue DP\n"); 
            return probe_write_dp(swdpreg_wo_ABORT, 0x1E);
        } else if  ( readval == 0x0BC12477 )
        {
            printf("Got Core DP, writing abort\n"); 
        } else
        {
            printf("Got unexpected init reply\n"); 
            return false;
        }

        return probe_write_dp(swdpreg_wo_ABORT, 0x1E);
    }  else
    {
        printf("Init err\n");
    }

    return false;
}

bool probe_rescue_reset( void )
{
    if ( initDebug(dp_rescue) )
    {
        printf("Got rescue, resetting\n"); 

        uint32_t reg;

        if ( probe_write_dp(swdpreg_rw_CSR, 1 << 28) )
        {
            printf("Written reset\n"); 
        } else
        {
            printf("fail1\n"); 
            return false;
        }

        // read it back to verify.
        if ( probe_read_dp(swdpreg_rw_CSR, &reg) )
        {
            printf("Read %08X\n", reg); 
        } else
        {
            printf("fail2\n"); 
            return false; 
        }

        if ( probe_write_dp(swdpreg_rw_CSR, 0) )
        {
            printf("Written reset\n"); 
        } else
        {
            printf("fail3\n"); 
            return false;
        }

        sleep_ms(5);

        // read it back to verify.
        if ( probe_read_dp(swdpreg_rw_CSR, &reg) )
        {
            printf("Read %08X\n", reg); 
            if ( reg == 0 )
                return initDebug(dp_core0);
        } else
        {
            printf("Read error got %08X\n", reg);
            return false;
        }
    }

    return false;

}


bool probe_write_memory( uint32_t addr, uint8_t *data, uint32_t count )
{
    if ( addr & 0b11 )
    {
        printf("Bad address! %08x\n", addr);
    }
    //probe_write_ap(ahbap_rw_TAR, addr);
    uint32_t writeval;
    uint32_t reg;
    for ( int i=0;i<count;i+=4)
    {
        if ( i % 256 == 0 )
        {
            //printf("TAR write at position %d to %08x\n", i, addr+i);
            if (!probe_write_ap(ahbap_rw_TAR, addr+i))
            {
                printf("Write error at %d\n", i);
                return false;
            }
        }
        //reg = ((uint32_t*)data)[i];
        reg = 0;
        reg = data[i]; // crude way to write only as much as given even if not an evenly aligned amount.
        if ( i+1 < count )
            reg |= data[i+1] << 8;
        if ( i+2 < count )
            reg |= data[i+2] << 16;
        if ( i+3 < count )
            reg |= data[i+3] << 24;

        //printf("TAR write %08x, %02x %02x %02x %02x\n", reg, reg&0xff, (reg>>8)&0xff, (reg>>16)&0xff, (reg>>24)&0xff);
        if ( ! probe_write_ap(ahbap_rw_DRW,reg) ) // * 256 to program 1kB
        {
            printf("Write error 2 at %d\n", i);
            return false;
        }
    }

    return true;
}

bool probe_write_memory_slow( uint32_t addr, uint8_t *data, uint32_t count )
{
    if ( addr & 0b11 )
    {
        printf("Bad address! %08x\n", addr);
    }
    //probe_write_ap(ahbap_rw_TAR, addr);
    uint32_t writeval;
    uint32_t reg;
    for ( int i=0;i<count;i+=4)
    {
        if ( i % 256 == 0 )
        {
            //printf("TAR write at position %d to %08x\n", i, addr+i);
            if (!probe_write_ap(ahbap_rw_TAR, addr+i))
            {
                printf("Write error at %d\n", i);
                return false;
            }
        }
        //reg = ((uint32_t*)data)[i];
        reg = 0;
        reg = data[i]; // crude way to write only as much as given even if not an evenly aligned amount.
        if ( i+1 < count )
            reg |= data[i+1] << 8;
        if ( i+2 < count )
            reg |= data[i+2] << 16;
        if ( i+3 < count )
            reg |= data[i+3] << 24;

        //printf("TAR write %08x, %02x %02x %02x %02x\n", reg, reg&0xff, (reg>>8)&0xff, (reg>>16)&0xff, (reg>>24)&0xff);
        if ( ! probe_write_ap(ahbap_rw_DRW,reg) ) // * 256 to program 1kB
        {
            printf("Write error 2 at %d\n", i);
            return false;
        }
#ifdef DEBUGMSG
        sleep_us(10); // with debug messages on, this seems to be needed, unsure why.
#endif
    }

    return true;
}


// count must be multiple of 4
bool probe_read_memory( uint32_t addr, uint8_t *data, uint32_t count)
{
    if ( addr & 0b11 )
    {
        printf("Bad address! %08x\n", addr);
        return false;
    }
    uint32_t reg = 0;
    //probe_write_ap(ahbap_rw_TAR, addr);
    //probe_read_ap(ahbap_rw_DRW, &reg); //throw away read
    for ( int i=0;i<count;i+=4) // need to execute one more read instruction than count indicates due to first result returning non defined
    {
        if ( i % 256 == 0 )
        {
            //printf("TAR read at position %d to %08x\n", i, 0x20000000+i);
            if ( ! probe_write_ap(ahbap_rw_TAR, addr+i) )
            {
                printf("Write error at %d\n", i);
                return false;
            }
            if ( ! probe_read_ap(ahbap_rw_DRW, &reg) ) //throw away read
            {
                printf("Read error throwaway at %d\n", i);
                return false;
            }
            //printf("TAR throwaway read %02x %02x %02x %02x\n", reg&0xff, (reg>>8)&0xff, (reg>>16)&0xff, (reg>>24)&0xff);
        }
        reg = 0;
        if ( !probe_read_ap(ahbap_rw_DRW, &reg) ) // * 256 to program 1kB
        {
            if ( ! probe_read_ap(ahbap_rw_DRW, &reg) ) //throw away read
            {
                printf("Read error at %d %08x\n", i, addr);
                return false;
            }  
        }
        
        uint32_t writepos = i; // account for first read being garbage
        //printf("TAR read %02x %02x %02x %02x\n", reg&0xff, (reg>>8)&0xff, (reg>>16)&0xff, (reg>>24)&0xff);
        data[writepos+0] = (reg>>0)&0xff;
        if ( writepos+1 < count )
            data[writepos+1] = (reg>>8)&0xff;
        if ( writepos+2 < count )
            data[writepos+2] = (reg>>16)&0xff;
        if ( writepos+3 < count )
            data[writepos+3] = (reg>>24)&0xff;

        //printf("TAR read %08x\n", reg);
    }

    return true;
}

bool probe_verify_memory( uint32_t addr, uint8_t *data, uint32_t count)
{
    uint8_t buffer[1024];
    uint32_t read = 0;

    while ( read < count ){
        uint32_t readamount = 1024;
        if ( read + 1024 > count )
            readamount = count-read;

        if (!probe_read_memory( addr+read, buffer, readamount) )
        {
            printf("verify read failed at %d\n", read);
            return false;
        }

        if ( memcmp(buffer, &data[read], readamount) != 0 )
        {
            printf("verify check failed at %d %d\n", read, readamount);
            return false;
        }
        read += readamount;
    }

    return true;
}

bool probe_sendhelper( void )
{
    // send the recovery program
    probe_write_dp(swdpreg_wo_SELECT, 0);
    probe_write_dp(swdpreg_rw_CSR,
                      ( 1 << ORUNDETECT )
                    //| ( 1 << READOK )
                    | ( 1 << CDBGPWRUPREQ )
                    | ( 1 << CSYSPWRUPREQ ) );

    uint32_t reg = 0;
    probe_read_dp(swdpreg_rw_CSR, &reg);
    if ( reg ==       ( 1 << ORUNDETECT )
                    | ( 1 << CDBGPWRUPREQ )
                    | ( 1 << CDBGPWRUPACK )
                    | ( 1 << CSYSPWRUPREQ ) 
                    | ( 1 << CSYSPWRUPACK ) )
    {
        printf("CSR ok\n");
    }

    if ( ! probe_write_ap(ahbap_rw_CSW, 
                      ( APSIZE32BIT << MEMAPSIZE )
                    | ( ADDRINCSINGLEEN << ADDRINC )  
                    | ( 1 << APDEVICEEN )
                    | ( 0b0100010 << APPROT )
                    | ( 1 << DBGSWENABLE) ) )
    {
       printf("Setup error 1\n"); 
    }

    if ( ! probe_write_ap(ahbap_rw_TAR, DHCSR) ) // Debug Halting Control and Status Register
    {
        printf("Setup error 2\n");
    }
    if (! probe_write_ap(ahbap_rw_DRW, 0xA05F0003) ) // set debug enable and halt CPU.
    {
        printf("Setup error 3\n");
    }

    printf("Send helper %dB\n", sizeof helper);
    uint32_t start = time_us_32();
    probe_write_memory(0x20000000, helper, sizeof helper);
    printf("Send took %dms, Checking helper %dB\n", (time_us_32() - start)/1000, sizeof helper);
    start = time_us_32();

    printf("Verify data:\n"); 
    if ( !probe_verify_memory(0x20000000, helper, sizeof helper) )
    {
       printf("Verify failed, Bad data send\n"); 
    }

    printf("Read took %dms\n", (time_us_32() - start)/1000);

    bool waiting = true;
    uint32_t magiccheck = 0;

    uint8_t empty[12] = {0};
    if ( ! probe_write_memory(0x20038000, empty, 12) )
    {
        printf("Clear error\n");
    }
    magiccheck = 0;
    if ( ! probe_read_memory( 0x20038000, (uint8_t*)&magiccheck, sizeof magiccheck) || magiccheck != 0)
    {
        printf("Initial magic error\n");
    }

    // start execution.
    probe_write_ap(ahbap_rw_TAR, DCRDR); // Debug Core Register Data Register
    probe_write_ap(ahbap_rw_DRW, 0x20000000);
    probe_write_ap(ahbap_rw_TAR, DCRSR); // Debug Core Register Selector Register
    probe_write_ap(ahbap_rw_DRW, 0x0001000F);
    // DebugReturnAddress << REGSEL Sets execution address on leaving debug state.
    //    1 << DCRSRWRITE
        // bit 16 -- write access
    // 0b1 0000 0000 0000 1111
    probe_write_ap(ahbap_rw_TAR, DHCSR); // DHCSR	RW	0x00000000	Debug Halting Control and Status Register
    probe_write_ap(ahbap_rw_DRW, 0xA05F0001); // remove halt bit.

    uint32_t magic = 0xDEADBEEF;

    printf("Waiting magic to confirm execution started\n");

    sleep_ms(50); // seems to sometimes need a small delay after execution starts to get a non error read?

    start = time_us_32();
    do
    {
        magiccheck = 0;
        if ( !probe_read_memory( 0x20038000, (uint8_t*)&magiccheck, sizeof magiccheck) )
        {
            printf("magic read error\n");
            //return false;
        }
        if ( magiccheck == magic )
        {
            printf("Got confirmation: %08x\n", magiccheck);
            return true;
        }
        sleep_ms(100);
    } while ( time_us_32() - start < 3000*1000 );

    printf("Timeout waiting for magic.\n");

    return false;

#if 0
    ( 1<<  C_DEBUGEN )
    ( 0xA05F << DBGKEY )
typedef enum _DHCSRbits
{
   C_DEBUGEN = 0,
   C_HALT = 1,
 DBGKEY = 16

}
    // 0b1010 0000 0101 1111 0000 0000 0000 0001
#endif

 // should now be sat at ready to write halted pico

/*
R/W	AP/DP	Register	Request byte	WData

write	DebugPort	TARGETSEL	0x99	0x01002927
read	DebugPort	IDCODE	0xA5	0x0BC12477
write	DebugPort	ABORT	0x81	0x0000001E
write	DebugPort	SELECT	0xB1	0x00000000
write	DebugPort	CTRL/STAT	0xA9	0x50000021
read	DebugPort	CTRL/STAT	0x8D	0xF0000001
write	AccessPort	CSW	0xA3	0xA2000052
write	AccessPort	TAR	0x8B	0xE000EDF0
write	AccessPort	DRW	0xBB	0xA05F0003
write	AccessPort	TAR	0x8B	0x20000000
write	AccessPort	DRW	0xBB	program 1st 32-bit word
write	AccessPort	DRW	0xBB	program 2nd 32-bit word
...				
write	AccessPort	DRW	0xBB	program last 32-bit word
write	AccessPort	TAR	0x8B	0xE000EDF8
write	AccessPort	DRW	0xBB	0x20000000
write	AccessPort	TAR	0x8B	0xE000EDF4
write	AccessPort	DRW	0xBB	0x0001000F
write	AccessPort	TAR	0x8B	0xE000EDF0
write	AccessPort	DRW	0xBB	0xA05F0001
*/

}


int32_t probe_send_instruction( uint8_t instruction, uint8_t instno )
{
    instruction_t cmd;
    uint32_t instroffset = offsetof(shareddata_t, inst) + sizeof cmd*instno;
    // check current instruction field is 0, signifying 
    uint8_t ins[4] = {instruction};
    if ( probe_write_memory(0x20038000+instroffset+offsetof(instruction_t, inst), ins, 4) )
        return true;
    return false;
    #if 0
    uint32_t start = 0;
    int32_t result = 0;
    while ( result == 0 && count < 10)
    {
        sleep_ms(20);
        probe_read_memory( 0x20038000+instroffset+offsetof(instruction_t, inst), (uint8_t*)&result, sizeof result);
    }

    return result;
    #endif
    // wait until it's been read, should be near instant
}


int32_t probe_wait_reply( uint32_t timeout, uint8_t instno )
{
    int32_t result = 0;
    int32_t clear = 0;
    timeout = timeout*1000;
    uint32_t start = time_us_32();

    instruction_t cmd;
    uint32_t instroffset = offsetof(shareddata_t, inst) + sizeof cmd*instno;

    for ( int i=0;i<8;i++)
        probe_low();  // idle
    while ( result == 0 && time_us_32()-start < timeout )
    {
        if ( !probe_read_memory( 0x20038000+instroffset+offsetof(instruction_t, res), (uint8_t*)&result, sizeof result) )
        {
            printf("error reading address %08x\n", 0x20038000+instroffset+offsetof(instruction_t, res));
            break;
        }
        sleep_us(600); // smaller values lead to read fail which would require reinit of connection.
    }

    if ( result == 0 )
    {
        printf("reply timeout\n");
    }

    return result;
}


void probe_init( void )
{
    gpio_init(PROBE_DIR);
    gpio_set_dir(PROBE_DIR, GPIO_OUT);

    gpio_init(PROBE_SWDCLK);
    gpio_set_dir(PROBE_SWDCLK, GPIO_OUT);
    gpio_put(PROBE_SWDCLK, 0); // start with clock line low.
    gpio_init(PROBE_SWDIO);
    gpio_set_dir(PROBE_SWDIO, GPIO_OUT);
    gpio_pull_up(PROBE_SWDIO);
    gpio_set_input_hysteresis_enabled(PROBE_SWDIO, 0);
    gpio_pull_up(PROBE_SWDCLK);

    gpio_set_slew_rate(PROBE_SWDCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PROBE_SWDIO, GPIO_SLEW_RATE_FAST);

    gpio_put(PROBE_DIR, 1);
    gpio_put(PROBE_SWDIO, 0);
}
