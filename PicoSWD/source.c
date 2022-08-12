#include <string.h>
#include <ctype.h>
#include "source.h"
#include "SEGGER_RTT.h"
#include "hardware/structs/systick.h"
#include "helper.h"
#include "pico_hal.h"
#include "../wipe/wipe.h"
//#include "picoprobe_config.h"
//#include "probe.h"

#define PROBE_PIN_OFFSET 2
#define PROBE_PIN_SWCLK PROBE_PIN_OFFSET + 0 // 2
#define PROBE_PIN_SWDIO PROBE_PIN_OFFSET + 1 // 3
#define PROBE_PIN_DIR   PROBE_PIN_OFFSET + 2 // 3

const uint PROBE_SWDCLK = PROBE_PIN_SWCLK;
const uint PROBE_SWDIO = PROBE_PIN_SWDIO;
const uint PROBE_DIR = PROBE_PIN_DIR;

const 

#define CYCLES1 3
#define CYCLES2 1

inline void cycledelay(int cycles)
{
    for ( int i=0;i<cycles;i++)
        asm("NOP");
}

void probe_high(void)
{
    gpio_put(PROBE_SWDIO, 1);
    cycledelay(CYCLES1);  
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay(CYCLES1);  
    gpio_put(PROBE_SWDCLK, 0); 
    cycledelay(CYCLES1);     
}

void probe_low(void)
{
    gpio_put(PROBE_SWDIO, 0);
    cycledelay(CYCLES1); 
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay(CYCLES1); 
    gpio_put(PROBE_SWDCLK, 0); 
    cycledelay(CYCLES1);  
}

void probe_turn(void)
{
    cycledelay(CYCLES1); 
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay(CYCLES1); 
    gpio_put(PROBE_SWDCLK, 0); 
    cycledelay(CYCLES1); 
}

bool probe_read(void)
{
    cycledelay(CYCLES1);
    bool res = gpio_get(PROBE_SWDIO);
    gpio_put(PROBE_SWDCLK, 1);
    cycledelay(CYCLES1);
    gpio_put(PROBE_SWDCLK, 0);
    cycledelay(CYCLES1);
    return res;  
}

#if 0
def parity32(i):
    i = i - ((i >> 1) & 0x55555555)
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333)
    i = (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24
    return i & 1
#endif

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

#define E_OK 0
#define E_SWD_PARITY 1
#define E_SWD_WAIT 2
#define E_SWD_FAULT 3
#define E_SWD_SWDERR 4
#define E_SWD_NOT_SUPPORTED 5

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

#define SWDIO_H gpio_put(PROBE_SWDIO, 1)
#define SWDIO_L gpio_put(PROBE_SWDIO, 0)

#define SWCLK_H gpio_put(PROBE_SWDCLK, 1)
#define SWCLK_L gpio_put(PROBE_SWDCLK, 0)

#define SWDIO_IS_H (gpio_get(PROBE_SWDIO) == 1)

void _prv_SWDDelay(uint32_t n)
{
    while (n--)
        asm("NOP");
}



#if 0
ERRCODE SWDWrDPReg(uint32_t regId, uint32_t u32Val)
{
    ERRCODE err = E_OK;
    uint32_t i;
    // Access to WRC register is deprecated and we
    // don't support.
    if (regId == swdpreg_rw_WCR)
        return E_SWD_NOT_SUPPORTED;
   // try 3 times in case we got errors.
    for (i=0; i<3; i++)
    {
        // if we got error on the last try, then we
        // clear all error flags or reinitialize.
        if (0 != i)
        {
            printf("write fault %d\n", err); 
            if (err == E_SWD_FAULT)
            {
                SWDWrDPReg(swdpreg_wo_ABORT, 0x1E);
            } else
                DAPInits(PP_NULL);
        }
        err = SWDWr(0, regId >> 2, u32Val);
        if (err >= E_OK)
            break;
    }
    return err;
}

typedef struct {
    uint8_t b4APBankSel;
} DS_SWDP_SELECT;

ERRCODE SWDWrAPReg(uint32_t apRegAddr, uint32_t val)
{
/*
    ahbap_rw_TAR = 0x04,
    ahbap_rw_DRW = 0x0C,
    swdpreg_wo_SELECT= 8,
    */

    ERRCODE err = E_OK;
    uint32_t i;
    DS_SWDP_SELECT apSel;
    ((U32*)&apSel)[0] = 0;
    // First set the high 4 bits address of AHB-AP by
    // writing SW-DP's SELECT register
    apSel.b4APBankSel = apRegAddr >> 4;
    err = SWDWrDPReg(swdpreg_wo_SELECT, ((U32*)&apSel)[0]);
    if (err == E_SWD_FAULT)
    {
        SWDWrDPReg(swdpreg_wo_ABORT, 0x1E);
    }
    err = SWDWr(1, (apRegAddr >> 2) & 3, val);
    return err;
}

ERRCODE SWDRdAPReg(uint32_t apRegAddr, OUT uint32_t *pVal)
{
    ERRCODE err = E_OK;
    DS_SWDP_SELECT apSel;
    ((U32*)&apSel)[0] = 0;
    // First set the high 4 bits address of AHB-AP by
    // writing SW-DP's SELECT register
    apSel.b4APBankSel = apRegAddr >> 4;
    err = SWDWrDPReg(swdpreg_wo_SELECT, ((U32*)&apSel)[0]);
    // Read AHB-AP, the first read just issues the read operation, we
    // then read the SW-DP's RDBUFF register to fetch the result
    err = SWDRd(1, (apRegAddr >> 2) & 3, pVal);
    err = SWDRd(0, swdpreg_ro_RDBUFF >> 2, pVal);
    return err;
}

ERRCODE DAPInits(OUT uint32_t *pIDR)
{
    ERRCODE err = E_OK;
    uint32_t tmp;
    #if 1
    // Set SWDIO line output
    probe_setdir(1);

    err = SWDRd(0, 0, pIDR);
    if (err < E_OK)
        return err;
    #endif
    // Clear any sticky bits in the SW-DP
    SWDWr(0, swdpreg_wo_ABORT >> 2, 0x1E);
    // Request system power up, debug component power up
    SWDWr(0, swdpreg_rw_CSR >> 2, 0x50000001);
    // Set AHB-AP 32 bit access, automatic increment, access as debug
    SWDWrAPReg(ahbap_rw_CSW, 2 | 1<<4 | 1<<24 | 1<<25 | 1<<29);
    return err;
}
#endif

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

bool probe_read_reg(bool dp, uint32_t reg, uint32_t * readval)
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
           return false;
        }

        for ( int i=0;i<8;i++)
            probe_low();  // idle
        return true;
    } else
        printf("read error: %d %d %d\n", ok, wait, fault); 
    return false;
}

bool probe_read_dp(uint32_t reg, uint32_t * readval)
{
    return probe_read_reg(true, reg, readval);
}

bool probe_read_ap(uint32_t reg, uint32_t * readval)
{
    return probe_read_reg(false, reg, readval);
}

bool probe_write_reg(bool dp, uint32_t reg, uint32_t writeval)
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
        return true;
    } else
        printf("error: %d %d %d\n", ok, wait, fault); 
    return false;
}

bool probe_write_dp(uint32_t reg, uint32_t writeval)
{
    return probe_write_reg(true, reg, writeval);
}

bool probe_write_ap(uint32_t reg, uint32_t writeval)
{
    return probe_write_reg(false, reg, writeval);
}


/*
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
            return false;
        }

        // read it back to verify.
        if ( probe_read_dp(swdpreg_rw_CSR, &reg) )
        {
            printf("Read %08X\n", reg); 
        } else
        {
            return false; 
        }

        if ( probe_write_dp(swdpreg_rw_CSR, 0) )
        {
            printf("Written reset\n"); 
        } else
        {
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
            probe_write_ap(ahbap_rw_TAR, addr+i);
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
        probe_write_ap(ahbap_rw_DRW,reg); // * 256 to program 1kB
    }


}


// count must be multiple of 4
bool probe_read_memory( uint32_t addr, uint8_t *data, uint32_t count)
{
    if ( addr & 0b11 )
    {
        printf("Bad address! %08x\n", addr);
    }
    uint32_t reg = 0;
    //probe_write_ap(ahbap_rw_TAR, addr);
    //probe_read_ap(ahbap_rw_DRW, &reg); //throw away read
    for ( int i=0;i<count+4;i+=4) // need to execute one more read instruction than count indicates due to first result returning non defined
    {
        if ( i % 256 == 0 )
        {
            //printf("TAR read at position %d to %08x\n", i, 0x20000000+i);
            probe_write_ap(ahbap_rw_TAR, addr+i);
            probe_read_ap(ahbap_rw_DRW, &reg); //throw away read
            //printf("TAR throwaway read %02x %02x %02x %02x\n", reg&0xff, (reg>>8)&0xff, (reg>>16)&0xff, (reg>>24)&0xff);
        }
        reg = 0;
        probe_read_ap(ahbap_rw_DRW, &reg); // * 256 to program 1kB
        
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

}

void dump_dir(void) {
	// display each directory entry name
	printf("File list\n");
    int dir = pico_dir_open("/");
    if (dir < 0)
        return;
    struct lfs_info info;
    while (pico_dir_read(dir, &info) > 0)
        printf("%s\n", info.name);
    pico_dir_close(dir);
	printf("End of list\n");
}

bool recoverboot( void )
{
    int file = pico_open("/boot.py", 0);
    int file2 = pico_open("/boot_recover.py", 0);

    pico_close(file); // only used to check if recovery exists.
    pico_close(file2);

    bool delete = false;

    if ( file2 > 0 )
        delete = true;

    if ( file > 0 )
    {
        if ( file2 < 0 )
        {
            printf("boot.py found but no boot_recover, renaming\n");
            if( pico_rename("/boot.py", "/boot_recover.py") < 0 )
            {
                printf("boot.py couldn't be renamed, deleting\n");
                delete = true;
                // rename failed, just delete the file instead.
            } else
                printf("boot.py renamed to boot_recover.py\n");
        }

        if ( delete )
            pico_remove("/boot.py");

    } else
          printf("boot.py not found %d\n", file);
}

bool recovermain( void )
{
    int file = pico_open("/main.py", 0);
    int file2 = pico_open("/main_recover.py", 0);

    pico_close(file); // only used to check if recovery exists.
    pico_close(file2);

    bool delete = false;

    if ( file2 > 0 )
        delete = true;

    if ( file > 0 )
    {
        if ( file2 < 0 )
        {
            printf("main.py found but no main_recover, renaming\n");
            if( pico_rename("/main.py", "/main_recover.py") < 0 )
            {
                printf("main.py couldn't be renamed, deleting\n");
                delete = true;
                // rename failed, just delete the file instead.
            } else
                printf("main.py renamed to main_recover.py\n");
        }

        if ( delete )
            pico_remove("/main.py");

    } else
          printf("main.py not found %d\n", file);

    return true;
}

bool sendhelper( void )
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

    probe_write_ap(ahbap_rw_CSW, 
                      ( APSIZE32BIT << MEMAPSIZE )
                    | ( ADDRINCSINGLEEN << ADDRINC )  
                    | ( 1 << APDEVICEEN )
                    | ( 0b0100010 << APPROT )
                    | ( 1 << DBGSWENABLE) );

    probe_write_ap(ahbap_rw_TAR, DHCSR); // Debug Halting Control and Status Register
    probe_write_ap(ahbap_rw_DRW, 0xA05F0003); // set debug enable and halt CPU.

    uint8_t data[sizeof _Users_visa_Code_pico_wipe_build_wipe_bin] = {0};
    printf("Send helper %dB\n", sizeof _Users_visa_Code_pico_wipe_build_wipe_bin);
    probe_write_memory(0x20000000, _Users_visa_Code_pico_wipe_build_wipe_bin, sizeof _Users_visa_Code_pico_wipe_build_wipe_bin);
    printf("Check helper %dB\n", sizeof _Users_visa_Code_pico_wipe_build_wipe_bin);
    probe_read_memory( 0x20000000, data, sizeof _Users_visa_Code_pico_wipe_build_wipe_bin);
    printf("Verify written memory: %d\n", memcmp(_Users_visa_Code_pico_wipe_build_wipe_bin, data, sizeof _Users_visa_Code_pico_wipe_build_wipe_bin));

    bool waiting = true;
    uint32_t magiccheck = 0;

    uint8_t empty[12] = {0};
    probe_write_memory(0x20038000, empty, 12);
    magiccheck = 0;
    probe_read_memory( 0x20038000, (uint8_t*)&magiccheck, sizeof magiccheck);

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

    printf("Waiting magic\n");

    uint32_t count = 0;

    while ( count < 10 )
    {
        magiccheck = 0;
        probe_read_memory( 0x20038000, (uint8_t*)&magiccheck, sizeof magiccheck);
        printf("Got magic: %08x\n", magiccheck);
        if ( magiccheck == magic ) 
            return true;
        sleep_ms(100);
    }

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


int32_t probe_send_insruction( uint8_t instruction )
{
    // check current instruction field is 0, signifying 
    uint8_t ins[4] = {instruction};
    probe_write_memory(0x20038000+offsetof(shareddata_t, inst), ins, 4);
    // wait until it's been read, should be near instant
}


int32_t probe_wait_reply( uint32_t timeout )
{
    uint32_t start = 0;
    int32_t result = 0;
    while ( result == 0 )
    {
        sleep_ms(20);
        probe_read_memory( 0x20038000+offsetof(shareddata_t, res), (uint8_t*)&result, sizeof result);
    }

    return result;
}

bool streql( const char * str1, const char * str2 )
{
	return ! ( strcmp(str1, str2) );
}

int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    stdio_init_all();

    SEGGER_RTT_Init();

#if 0
    if (pico_mount(false) == LFS_ERR_OK) {
        if ( !recoverfiles() )
            printf("Error recovering files\n");
    } else
    {
        printf("Error mounting FS, deleting file system area instead.\n");
    }

    printf("FS mounted\n");
#endif

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

    volatile uint32_t rtt = (uint32_t) &_SEGGER_RTT;
    volatile uint32_t rttsize = sizeof _SEGGER_RTT;

    gpio_put(LED_PIN, 1);

#if 0
    uint8_t eraseboot[4] = {5};
    probe_write_memory(0x20038004, eraseboot, 4);

    int32_t result = 0;
    while ( result == 0 )
    {
        sleep_ms(20);
        probe_read_memory( 0x20038008, (uint8_t*)&result, sizeof result);
    }
    printf("Boot block erased, sending blink command\n");

#endif

    bool connected = false;
    bool filesystem = false;

    printf("Enter Command ( connect to start ):\n");

	uint8_t charcount = 0;
    char str[61] = { 0 };

    while ( true )
    {
        int read;
        bool endline = false;
        read = SEGGER_RTT_GetKey();

		if ( read == 0 )
		{
			continue; // nothing to do this loop, return to start.
		}

		if ( read == 8 || read == 127)
		{
			if ( charcount > 0 )
			{
				--charcount;
				str[charcount] = 0;
				str[charcount+1] = 0;
			}
		} else
		if ( !( read == '\n' || read == '\r') )
		{
			if ( read >= 32 && read <= 128) // only process printable charecters.
			{
				str[charcount] = read;
				str[charcount+1] = 0;
				printf("%c", read);
				++charcount;
			}
		} else
		{
			endline = true;
			printf("\r\n");
		}

#define TOKENLENGTH   12

		if ( charcount == 60 || endline )
		{
            // lowercase the input string.
            for ( int i=0;str[i];i++)
                str[i]=tolower(str[i]);

            char tkn1[TOKENLENGTH] = "";
            char tkn2[TOKENLENGTH] = "";
            char tkn3[TOKENLENGTH] = "";
            char tkn4[TOKENLENGTH] = "";
            char tkn5[TOKENLENGTH] = "";

            uint8_t tokens = 0;

            int val1;
            int val2;
            int val3;
            int val4;

            // parse the input string into tokens to be processed.

            char *s=str;

            if (*(s += strspn(s, " ")) != '\0') { // find the first non space, and move string pointer to it. Check if we have reached end of string.
                size_t tknlen = strcspn(s, " ");  // if not at end of string, find the next space, getting the span of token.
                if ( tknlen < TOKENLENGTH )
                {
                    strncpy(tkn1, s, tknlen);
                    tkn1[tknlen] = '\0';

                    s += tknlen;
                    tokens++;
                } else
                {
                    strncpy(tkn1, "too long", tknlen);
                }
            }

            if (*(s += strspn(s, " ")) != '\0') {
                size_t tknlen = strcspn(s, " ");
                if ( tknlen < TOKENLENGTH )
                {
                    strncpy(tkn2, s, tknlen);
                    tkn2[tknlen] = '\0';

                    s += tknlen;
                    tokens++;
                }
            }

            if ( strlen(tkn2) >0 )
            {
                val1 = strtol(tkn2, NULL, 10);
            } else val1 = 0;


            if (*(s += strspn(s, " ")) != '\0') {
                size_t tknlen = strcspn(s, " ");
                if ( tknlen < TOKENLENGTH )
                {
                    strncpy(tkn3, s, tknlen);
                    tkn3[tknlen] = '\0';
                    s += tknlen;
                    tokens++;
                }
            }

            if ( strlen(tkn3)>0 )
            {
                val2 = strtol(tkn3, NULL, 10);
            } else val2 = 0;

            if (*(s += strspn(s, " ")) != '\0') {
                size_t tknlen = strcspn(s, " ");
                if ( tknlen < TOKENLENGTH )
                {
                    strncpy(tkn4, s, tknlen);
                    tkn4[tknlen] = '\0';
                    s += tknlen;
                    tokens++;
                }
            }

            if ( strlen(tkn4)>0 )
            {
                val3 = strtol(tkn4, NULL, 10);
            } else val3 = 0;


            if (*(s += strspn(s, " ")) != '\0') {
                size_t tknlen = strcspn(s, " ");
                if ( tknlen < TOKENLENGTH )
                {
                    strncpy(tkn5, s, tknlen);
                    tkn5[tknlen] = '\0';
                    tokens++;
                }
            }

            if ( strlen(tkn5)>0 )
            {
                val4 = strtol(tkn5, NULL, 10);
            } else val4 = 0;

            gpio_put(LED_PIN, 0);
            sleep_ms(250);
            gpio_put(LED_PIN, 1);

            bool processed = false;

            if ( connected )
            {
                processed = true;
                if ( streql(tkn1, "blink" ) )
                {
                    printf("Blink\n");
                    uint8_t blink[4] = {0xff};
                    probe_write_memory(0x20038000+offsetof(shareddata_t, inst), blink, 4);

                    probe_wait_reply(1000);

                } else if ( streql(tkn1, "boot" ) )
                {
                    if ( filesystem )
                    {
                        printf("boot.py recovery\n");
                        uint8_t boot[4] = {0x1};
                        probe_write_memory(0x20038000+offsetof(shareddata_t, inst), boot, 4);
                    } else
                    {
                       printf("boot: no file system found\n"); 
                    }
                } else if ( streql(tkn1, "main" ) )
                {                    
                    if ( filesystem )
                    {
                        printf("main.py recovery\n");
                        uint8_t main[4] = {0x2};
                        probe_write_memory(0x20038000+offsetof(shareddata_t, inst), main, 4);
                    } else
                    {
                       printf("main: no file system found\n"); 
                    }
                } else if ( streql(tkn1, "wipefiles" ) )
                {
                    if ( filesystem )
                    {
                        printf("files\n");
                        uint8_t files[4] = {0x3};
                        probe_write_memory(0x20038000+offsetof(shareddata_t, inst), files, 4);
                    } else
                    {
                       printf("wipefiles: no file system found\n"); 
                    }
                } else if ( streql(tkn1, "usb" ) )
                {
                    printf("usb load requested\n");
                    uint8_t usb[4] = {0x4};
                    probe_write_memory(0x20038000+offsetof(shareddata_t, inst), usb, 4);

                    connected = false;
                } else 
                {
                    processed = false;
                }
            }

            if ( !processed ) // command not yet processed, check all state commands.
            {
                processed = true;
                if ( streql(tkn1, "connect" ) )
                {
                    printf("Connecting to pico:\n");

                    printf("Wake pico to rescue core\n");
                    probe_rescue_reset();

                    if ( sendhelper() )
                    {
                        int32_t result = 0;
                        probe_read_memory( 0x20038000+offsetof(shareddata_t, res), (uint8_t*)&result, sizeof result);
                                printf("Helper uploaded, checking state: %08x -> ", result);
                        if ( result == 0xabcd )
                        {
                            printf("Filesystem found\n");
                            filesystem = true;
                        }
                        else
                        {
                            printf("No filesystem\n");
                            filesystem = false;
                        }
                        connected = true;
                    } else
                        connected = false;

                    gpio_put(LED_PIN, 0);
                    sleep_ms(200);
                    gpio_put(LED_PIN, 1);
                } else
                {
                    processed = false;
                }
            }
            
            if ( !processed )
            {
                printf("Got unavailable command: %s\n", str);
            }

            charcount = 0;
			str[0] = 0;
            printf("(%s)Enter Command:\n", connected?"connected":"disconnected");
        }
    }
#endif
}
