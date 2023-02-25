#include <string.h>
#include <ctype.h>
#include "ring_buffer.hpp"

extern "C" {
#include "source.h"
#include "SEGGER_RTT.h"
#include "stdio_rtt.h"
#include "hardware/structs/systick.h"
#include "hardware/adc.h"
#include "../wipe/littlefs-lib/pico_hal.h"
#include "uf2.h"
#include "../wipe/wipe.h"
#include "hardware/watchdog.h"
}
#include "pico_display.hpp"
#include "drivers/st7789/st7789.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"
#include "rgbled.hpp"
#include "button.hpp"
#include "swd.hpp"

int usbload(uint8_t file);

using namespace pimoroni;
using namespace std;

// Display driver
ST7789 st7789(PicoDisplay::WIDTH, PicoDisplay::HEIGHT, ROTATE_0, false, get_spi_pins(BG_SPI_FRONT));

// Graphics library - in RGB332 mode you get 256 colours and optional dithering for ~32K RAM.
//PicoGraphics_PenRGB332 graphics(st7789.width, st7789.height, nullptr);

PicoGraphics_PenRGB565 graphics(st7789.width, st7789.height, nullptr);

// RGB LED
RGBLED led(PicoDisplay::LED_R, PicoDisplay::LED_G, PicoDisplay::LED_B);

// And each button
Button button_a(PicoDisplay::A);
Button button_b(PicoDisplay::B);
Button button_x(PicoDisplay::X);
Button button_y(PicoDisplay::Y);

uint32_t crc32b(unsigned char *data, uint32_t size) {
    int i, j;
    unsigned int byte, crc, mask;

    if ( data == NULL )
        return 0;

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

bool streql( const char * str1, const char * str2 )
{
	return ! ( strcmp(str1, str2) );
}

ring_buffer<string> output(9);

void logstrmulti(const char * str, bool multiline)
{
    string line = "";
    int pos = 0;
    int len = strlen(str);

    graphics.set_font(&font8);

    while ( pos < len )
    {
        if ( graphics.measure_text(line+str[pos], 1) > 240) // find out how much of line will fit
        {
            output.push_back(line);
            line.clear();
            if ( !multiline )
            break;
        }
        line.push_back(str[pos++]);
    }
    if ( line.length()>0 )
        output.push_back(line);

    graphics.remove_clip();
    graphics.set_pen(0, 0, 0);
    Rect clear_rect(0, 0, 240, 13*9);
    graphics.rectangle(clear_rect);
    for ( int i = 0;i<output.size();i++ )
    {
        graphics.set_pen(0, 0, 0);
        Rect text_rect(0, i*13, 240, 13);
        graphics.rectangle(text_rect);
        text_rect.deflate(1);
        graphics.set_pen(110, 120, 130);
        char str[40];
        graphics.set_clip(text_rect);
        graphics.text(output[i].c_str(), Point(text_rect.x, text_rect.y), text_rect.w, 1);
        graphics.remove_clip();
    }

    // now we've done our drawing let's update the screen
    st7789.update(&graphics);
}

void logstr(const char * str)
{
    logstrmulti(str, false);
}

void buttonguide(void)
{
    //graphics.set_font(&font8);
    graphics.set_font(&hershey::timesrb);

    graphics.remove_clip();
    graphics.set_pen(0, 0, 0);
    Rect clear_rect(0, 0, 240, 13*9);
    graphics.rectangle(clear_rect);

    graphics.set_pen(0, 120, 120);
    graphics.text("Recover", Point(0, 25), 120, 0.66);
    graphics.set_pen(120, 120, 0);
    graphics.text("Reflash", Point(0, 105), 120, 0.66);
    graphics.set_pen(120, 0, 120);
    graphics.text("Erase FS", Point(240-graphics.measure_text("Erase FS", 0.66), 25), 120, 0.66);
    graphics.set_pen(120, 60, 60);
    graphics.text("USB Load", Point(240-graphics.measure_text("USB Load", 0.66), 105), 120, 0.66);

    // now we've done our drawing let's update the screen
    st7789.update(&graphics);
}

void storagechoiceguide(void)
{
    //graphics.set_font(&font8);
    graphics.set_font(&hershey::timesrb);

    graphics.remove_clip();
    graphics.set_pen(0, 0, 0);
    Rect clear_rect(0, 0, 240, 13*9);
    graphics.rectangle(clear_rect);

    graphics.set_pen(0, 120, 120);
    graphics.text("Pico", Point(0, 25), 120, 0.66);
    graphics.set_pen(120, 120, 0);
    graphics.text("Pico W", Point(0, 105), 120, 0.66);
    graphics.set_pen(120, 0, 120);
    graphics.text("cancel", Point(240-graphics.measure_text("cancel", 0.66), 25), 120, 0.66);

    graphics.set_pen(120, 60, 60);
    graphics.text("Rcv Image for?", Point(30, 65), 200, 0.8);

    // now we've done our drawing let's update the screen
    st7789.update(&graphics);
}

uint8_t picoconnection = false;
bool filesystem = false;
uint32_t lastseen = 0;

void drawstatus(connectionstatus_t status, const char * statusstr)
{
    static connectionstatus_t laststatus = none;

    if ( status == connected && filesystem )
    {
        status = connectedwithfs;
    }

    if ( status != laststatus || strlen (statusstr) > 0 )
    {
        string str;
        laststatus = status;
        graphics.remove_clip();
        graphics.set_font(&font8);
        graphics.set_pen(0, 0, 0);
        Rect clear_rect(0, 13*9+1, 240, 134);
        graphics.rectangle(clear_rect);
        switch ( status )
        {
            case connected:
            case connectednofs:
                graphics.set_pen(0, 120, 0);
                if ( picoconnection == 2)
                    str = "con W";
                else
                    str = "connected";
                break;
            case connectedwithfs:
                graphics.set_pen(0, 120, 0);
                if ( picoconnection == 2)
                    str = "con W+gotfs";
                else
                    str = "con+gotfs";
                break;
            case notconnected:
                graphics.set_pen(120, 0, 0);
                str = "no device";
                break;
            case usbconnected:
            graphics.set_pen(0, 120, 0);
            str = "usb con";
            break;
            case usbnotconnected:
            graphics.set_pen(120, 0, 0);
            str = "no usb";
            break;
            case booting:
            graphics.set_pen(120, 120, 0);
            str = "booting";
            break;
            default:
            graphics.set_pen(120, 120, 120);
            str = "unknown";
        }

        graphics.text(str, Point(0, 13*9+1), 120, 2);
        if ( strlen(statusstr) )
            graphics.text(statusstr, Point(120, 13*9+1), 120, 2);

        st7789.update(&graphics);
    }  
}

void openconnection(void)
{
    printf("Connecting to pico:\n");

    printf("Wake pico to rescue core\n");
    if ( !probe_rescue_reset() )
    {
        drawstatus(notconnected, "");
        picoconnection = 0;
        return;
    }

    if ( probe_sendhelper() )
    {
        int32_t result = 0;
        probe_read_memory( 0x20038000+offsetof(shareddata_t, inst[0])+offsetof(instruction_t, res), (uint8_t*)&result, sizeof result);
        printf("Helper uploaded, checking state: %08x -> ", result);
        
        switch ( ( abs(result) & 0xf00 ) >> 8 )
        {
            case 1:
                printf("Found pico\n");
                picoconnection = 1;
                break;
            case 2:
                printf("Found pico W\n");
                picoconnection = 2;
                break;
            default:
                picoconnection = 0;
                filesystem = false;
                printf("Pico version unidentified\n");
                return;
        }

        result = result & 0xff;
        
        if ( result == 0xab )
        {
            printf("Filesystem found\n");
            filesystem = true;
            drawstatus(connectedwithfs, "");
        } else
        {
            printf("No filesystem\n");
            filesystem = false;
            drawstatus(connectednofs, "");
        }

        lastseen = time_us_32();
    } else
    {
        drawstatus(notconnected, "");
        picoconnection = 0;
    }
#if 0
    gpio_put(LED_PIN, 0);
    sleep_ms(200);
    gpio_put(LED_PIN, 1);
#endif
}

void __attribute__((noreturn)) _exit(__unused int status)
{
    watchdog_reboot(0, 0, 0);
    while ( 1 );
}

int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning requires a board with a regular LED
#else
    set_sys_clock_khz(250000, true); //250mhz seems to work
    //stdio_init_all();
    stdio_rtt_init();

    SEGGER_RTT_Init();

    const uint SET3V3EN_PIN = 28;
    gpio_init(SET3V3EN_PIN);
    gpio_set_dir(SET3V3EN_PIN, GPIO_OUT);
    gpio_put(SET3V3EN_PIN, 0);

    const uint INTERLOCKOUT = 27;
    gpio_init(INTERLOCKOUT);
    gpio_set_dir(INTERLOCKOUT, GPIO_OUT);
    gpio_put(INTERLOCKOUT, 1);

    const uint INTERLOCKIN = 26;
    gpio_init(INTERLOCKIN);
    gpio_set_dir(INTERLOCKIN, GPIO_IN);
    gpio_pull_down(INTERLOCKIN);
    bool interlockclosed = gpio_get(INTERLOCKIN);
    bool powered = false;


    adc_init();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // detect pico version here.

    int picoversion = 1;

// https://forums.raspberrypi.com/viewtopic.php?t=336775
    #define VSYS_ADC_GPIO  29
    #define VSYS_ADC_CHAN   3
    #define HALF_VOLT     204 // 0.5V divide by 3, as 12-bit with 3V3 Vref
   
    adc_gpio_init(VSYS_ADC_GPIO); // vsys input
    adc_select_input(VSYS_ADC_CHAN); // VSYS channel

    uint16_t adcval = adc_read();

    // with LED pin off this should read near 0
    if (adcval< HALF_VOLT)
        picoversion = 2;                                 // Pico-W


    printf("got adc reading %u, pico version %d\n", adcval, picoversion);


    // turn on led
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    st7789.set_backlight(255);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    graphics.set_pen(0, 0, 0);
    graphics.clear();
    st7789.update(&graphics);
    led.set_rgb(0,0,0);

    logstr("startup.");

    drawstatus(notconnected, "startup");
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    probe_init();

  //  volatile uint32_t rtt = (uint32_t) &_SEGGER_RTT;
  //  volatile uint32_t rttsize = sizeof _SEGGER_RTT;

    gpio_put(LED_PIN, 1);

    //gpio_put(LED3V3EN_PIN, 1);

    char filename[139];

    bool gotfile[2] = { false };

    if ( uf2_get_uf2filename(0, filename, sizeof filename) )
    {
        logstrmulti(filename, true);
        gotfile[0] = true;
    } else
    {
        logstr("No file for pico");
    }

    if ( uf2_get_uf2filename(1, filename, sizeof filename) )
    {
        logstrmulti(filename, true);
        gotfile[1] = true;
    } else
    {
        logstr("No file for pico W");
    }

    //openconnection();
#ifdef RTTCONSOLE
    printf("Enter Command (connect to start) :\n");
#endif
    uint32_t starttime = time_us_32();
    bool shownchoices = false;

    //logstr("Press button");

	uint8_t charcount = 0;
    char str[61] = { 0 };

    while ( true )
    {
#ifdef RTTCONSOLE
        int read;
        bool endline = false;
        read = SEGGER_RTT_GetKey();
#endif
        interlockclosed = gpio_get(INTERLOCKIN);

        if ( interlockclosed )
        {
            if ( !powered )
            {
                gpio_put(SET3V3EN_PIN, 1);
                lastseen = time_us_32();
                powered = true;
                drawstatus(notconnected, "");
            }
        }
        else
        {
            if ( powered )
            {
                gpio_put(SET3V3EN_PIN, 0);
                logstr("interlock opened");
                filesystem = false;
                picoconnection = 0;
                powered = false;
                drawstatus(notconnected, "");
            }
        }

        uint32_t curtick = time_us_32();

        if ( !shownchoices && time_us_32() > starttime + 1000*1000*3)
        {
            shownchoices = true;
            buttonguide();
            //logstr("Press button");
        }

        if ( interlockclosed && curtick > lastseen + 1000*1000 ) // every second of not doing something check if pico still there.
        {
            int32_t result = 0;
            if ( picoconnection && probe_read_memory( 0x20038000+offsetof(shareddata_t, inst[0])+offsetof(instruction_t, data), (uint8_t*)&result, sizeof result) )
            {
                lastseen = time_us_32();
                drawstatus(connected, "");
            } else
            {
                openconnection();
            }
        } 
#ifdef RTTCONSOLE
        else if ( read == 0 )
		{
			continue; // nothing to do this loop, return to start.
		} else if ( read == 8 || read == 127)
		{
			if ( charcount > 0 )
			{
				--charcount;
				str[charcount] = 0;
				str[charcount+1] = 0;
			}
		} else if ( !( read == '\n' || read == '\r') )
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
#endif

        // check button presses for possible commands.

        if ( picoconnection )
        {
            if(button_a.read())
            {
                shownchoices = true;
                if ( filesystem )
                {
                    drawstatus(connectedwithfs, "Recovering");
                    sleep_ms(200);
                    printf("boot.py recovery\n");
                    logstr("boot.py recovery");
                    probe_send_instruction(1,0);
                    int32_t res;
                    char str[32];
                    sleep_ms(20);
                    bool recovered = false;
                    printf("recovery wait\n");
                    res = probe_wait_reply(5000, 0);
                    switch ( res )
                    {
                        case 1 : logstr("boot.py renamed"); drawstatus(connectedwithfs, "Recovered"); recovered = true; break;
                        case 2 : logstr("boot.py deleted"); drawstatus(connectedwithfs, "Recovered"); recovered = true; break;
                        case -1 : logstr("boot.py error recovering"); drawstatus(connectedwithfs, "Recovery Err"); break;
                        case -2 : logstr("boot.py not found"); break;
                        default :
                        snprintf(str, 32, "unknown res %d", res);
                        logstr(str);
                    }

                    printf("main.py recovery\n");
                    logstr("main.py recovery");
                    probe_send_instruction(2,0);
                    sleep_ms(20);
                    printf("recovery wait\n");
                    res = probe_wait_reply(5000,0);
                    switch ( res )
                    {
                        case 1 : logstr("main.py renamed"); drawstatus(connectedwithfs, "Recovered"); recovered = true; break;
                        case 2 : logstr("main.py deleted"); drawstatus(connectedwithfs, "Recovered"); recovered = true; break;
                        case -1 : logstr("main.py error recovering"); drawstatus(connectedwithfs, "Recovery Err"); break;
                        case -2 : logstr("main.py not found"); break;
                        default :
                        snprintf(str, 32, "unknown res %d", res);
                        logstr(str);
                    }

                    if ( !recovered )
                    {
                        drawstatus(connectedwithfs, "Recovery N/A");
                        sleep_ms(500);
                    }
                } else
                {
                    logstr("no FS to recover");
                    drawstatus(connectedwithfs, "No FS to recover");
                }
            }

            if(button_b.read())
            {
                shownchoices = true;
                if ( gotfile[picoconnection-1] )
                {
                    drawstatus(filesystem?connectedwithfs:connected, picoconnection==2?"Flash Pico W":"Flash Pico");
                    sleep_ms(500);
                    char str[32];
                    snprintf(str, sizeof str, "Writing uf2 to pico%s", picoconnection==2?" W":"");
                    logstr(str);
                    uf2_init(picoconnection-1); // setup file ready to flash.
                    int res = probe_flash_uf2();
                    switch ( res )
                    {
                        case 1: logstr("flash complete");
                        logstr("booting board");
                        gpio_put(SET3V3EN_PIN, 0);
                        drawstatus(booting, "");
                        gpio_put(SET3V3EN_PIN, 1);
                        sleep_ms(2000);
                        openconnection();
                        break;
                        default:
                            logstr("flash failed");
                            printf("flash failed %d\n", res);
                    }
                } else
                {
                    drawstatus(filesystem?connectedwithfs:connected, picoconnection==2?"No Pico W UF2":"No Pico UF2");
                    logstr("no uf2 to flash");
                }
            }

            if(button_x.read())
            {
                printf("clearing file area\n");
                drawstatus(filesystem?connectedwithfs:connected, "Clearing FS area");
                logstr("clearing file area");
                probe_send_instruction(3,0);
                sleep_ms(20);
                printf("recovery wait\n");
                int32_t res = probe_wait_reply(20000,0);
                switch ( res )
                {
                    case 1 : logstr("file area erased"); break;
                    default :
                            logstr("erase failed?"); break;
                    break;
                }
                openconnection(); // file system should be nulled now, reopen connection.
            }
        } else
        {
            if ( button_a.read() || button_b.read() || button_x.read() )
                logstr("not connected");
        }

        if(button_y.read())
        {
            storagechoiceguide();
            uint8_t choice = 2;

            uint32_t qstart = time_us_32();
            while ( time_us_32() < qstart + 1000*1000*5) // give 5 seconds to make choice, or backs out.
            {
                if ( button_x.read() )
                {
                    choice = 2;
                    break;
                }

                button_y.read();

                if ( button_a.read() )
                {
                    choice = 0;
                    break;
                }

                if ( button_b.read() )
                {
                    choice = 1;
                    break;
                }
            }

            switch ( choice )
            {
                case 0:
                case 1:
                    {
                    char str[32];
                    snprintf(str, sizeof str, "usbload pico%s", choice==1?" W":"");
                    logstr(str);
                    powered = false;
                    gpio_put(SET3V3EN_PIN, 0);
                    picoconnection = false;
                    drawstatus(usbnotconnected, "");
                    usbload(choice);
                    logstr("usb exit.");
                    if ( uf2_get_uf2filename(choice, filename, sizeof filename) )
                    {
                        logstrmulti(filename, true);
                        gotfile[choice] = true;
                    } else
                    {
                        logstr("No file");
                        gotfile[choice] = false;
                    }
                    }
                break;

                default:
                    logstr("usbload choice timeout");
            }
        }
#ifdef RTTCONSOLE
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
#if 1
            if ( streql(tkn1, "send" ) )
            {
                processed = true;
                {
                    logstr("Writing uf2 to pico.");
                    switch ( probe_flash_uf2() )
                    {
                        case 1:
                            logstr("flash complete");
                            sleep_ms(250);
                            drawstatus(connected, "");
                            break;

                        default:
                            logstr("flash failed");
                            printf("flash failed\n");
                    }
                }
            }
#endif

            if ( streql(tkn1, "usbload" ) )
            {
                logstr("usbload.");
                gpio_put(LED3V3EN_PIN, 0);
                picoconnection = false;
                drawstatus(usbnotconnected, "");
                usbload(0);
                processed = true;
                logstr("usb exit.");
                gpio_put(LED3V3EN_PIN, 1);
                openconnection();
                drawstatus(picoconnection?connected:notconnected, "");
            } 
            
            if ( !processed && picoconnection )
            {
                processed = true;
                if ( streql(tkn1, "boot" ) )
                {
                    if ( filesystem )
                    {
                        printf("boot.py recovery\n");
                        probe_send_instruction(1,0);
                        //sleep_ms(20);
                        int32_t res = probe_wait_reply(5000,0);
                        printf("result %d\n", res);
                    } else
                    {
                       printf("boot: no file system found\n"); 
                    }
                } else if ( streql(tkn1, "main" ) )
                {                    
                    if ( filesystem )
                    {
                        printf("main.py recovery\n");
                        probe_send_instruction(2,0);
                        sleep_ms(20);
                        int32_t res = probe_wait_reply(5000,0);
                        printf("result %d\n", res);
                    } else
                    {
                       printf("main: no file system found\n"); 
                    }
                } else if ( streql(tkn1, "wipefiles" ) )
                {
                    if ( filesystem )
                    {
                        printf("clearing file area\n");
                        probe_send_instruction(3,0);
                        sleep_ms(20);
                        int32_t res = probe_wait_reply(20000,0);
                        printf("result %d\n", res);
                    } else
                    {
                       printf("wipefiles: no file system found\n"); 
                    }
                } else if ( streql(tkn1, "wipenofiles" ) )
                {
                    printf("wiping program region only\n");
                    uint32_t lowaddr = 0x10000000;
                    uint32_t erasesize = PICO_FLASH_SIZE_BYTES - MICROPY_HW_FLASH_STORAGE_BYTES;

                    probe_write_memory(0x20038000+offsetof(shareddata_t, addr), (uint8_t*)&lowaddr, 4);
                    probe_write_memory(0x20038000+offsetof(shareddata_t, size), (uint8_t*)&erasesize, 4);
                    probe_send_instruction(7,0);
                    uint32_t start = time_us_32();
                    int32_t res = probe_wait_reply(20000,0); // erase could take a while.

                    if ( res != 1 )
                    {
                        printf("data erase failed %d\n", res);
                    } else
                    {
                        printf("data erase succeeded %d\n", res);
                    }
                } else if ( streql(tkn1, "usb" ) )
                {
                    printf("usb load requested\n");
                    probe_send_instruction(4,0);
                    picoconnection = false;
                    drawstatus(notconnected, "");
                } else 
                {
                    processed = false;
                }
            }

            if ( !processed ) // command not yet processed, check all state commands.
            {
                processed = true;
                if ( streql(tkn1, "blink" ) )
                {
                    printf("Blink\n");
                    probe_send_instruction(0xff,0);
                } else if ( streql(tkn1, "connect" ) )
                {
                    openconnection();
                }
                else
                {
                    processed = false;
                }
            }
            
            
            if ( !processed )
            {
                printf("Got unavailable command: %s\n", str);
                char log[128];
                snprintf(log, sizeof log, "Unknown: %s", str);
                logstr(log);
            }

            charcount = 0;
			str[0] = 0;
            printf("Enter Command (%s) :\n", connected?"connected":"disconnected");
        }
        #endif
    }
#endif

// if somehow end up here, reboot.
    watchdog_reboot(0, 0, 0);
}
