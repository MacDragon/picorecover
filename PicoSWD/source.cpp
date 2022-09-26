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


extern "C" {
    int usbload(void);
}

using namespace pimoroni;
using namespace std;

// Display driver
ST7789 st7789(PicoDisplay::WIDTH, PicoDisplay::HEIGHT, ROTATE_0, false, get_spi_pins(BG_SPI_FRONT));

// Graphics library - in RGB332 mode you get 256 colours and optional dithering for ~32K RAM.
PicoGraphics_PenRGB332 graphics(st7789.width, st7789.height, nullptr);

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
    graphics.text("Blink", Point(240-graphics.measure_text("Blink", 0.66), 25), 120, 0.66);
    graphics.set_pen(120, 60, 60);
    graphics.text("USB Load", Point(240-graphics.measure_text("USB Load", 0.66), 105), 120, 0.66);

    // now we've done our drawing let's update the screen
    st7789.update(&graphics);
}

bool picoconnection = false;
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
                str = "connected";
                break;
            case connectedwithfs:
                graphics.set_pen(0, 120, 0);
                str = "connected+fs";
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
            default:
            graphics.set_pen(120, 120, 120);
            str = "unknown";
        }

        graphics.text(str, Point(0, 13*9+1), 120, 2);

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
        picoconnection = false;
        return;
    }

    if ( probe_sendhelper() )
    {
        int32_t result = 0;
        probe_read_memory( 0x20038000+offsetof(shareddata_t, res), (uint8_t*)&result, sizeof result);
        printf("Helper uploaded, checking state: %08x -> ", result);
        if ( result == 0xabcd )
        {
            printf("Filesystem found\n");
            filesystem = true;
            drawstatus(connectedwithfs, "");
        }
        else
        {
            printf("No filesystem\n");
            filesystem = false;
            drawstatus(connectednofs, "");
        }

        picoconnection = true;
        lastseen = time_us_32();
    } else
    {
        drawstatus(notconnected, "");
        picoconnection = false;
    }
#if 0
    gpio_put(LED_PIN, 0);
    sleep_ms(200);
    gpio_put(LED_PIN, 1);
#endif
}

int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning requires a board with a regular LED
#else
    set_sys_clock_khz(250000, true); //250mhz seems to work
    stdio_init_all();

    SEGGER_RTT_Init();

    const uint LED3V3EN_PIN = 28;
    gpio_init(LED3V3EN_PIN);
    gpio_set_dir(LED3V3EN_PIN, GPIO_OUT);
    gpio_put(LED3V3EN_PIN, 0);

    st7789.set_backlight(255);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    graphics.set_pen(0, 0, 0);
    graphics.clear();
    st7789.update(&graphics);
    led.set_rgb(0,0,0);

    logstr("startup.");

    drawstatus(none, "startup");
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    probe_init();

    volatile uint32_t rtt = (uint32_t) &_SEGGER_RTT;
    volatile uint32_t rttsize = sizeof _SEGGER_RTT;

    gpio_put(LED_PIN, 1);

    gpio_put(LED3V3EN_PIN, 1);

    char filename[139];

    if ( uf2_get_uf2filename(filename, sizeof filename) )
    {
        logstrmulti(filename, true);
    } else
    {
        logstr("No file");
    }

    openconnection();

    printf("Enter Command (connect to start) :\n");

    uint32_t starttime = time_us_32();
    bool shownchoices = false;

    //logstr("Press button");

	uint8_t charcount = 0;
    char str[61] = { 0 };

    while ( true )
    {
        int read;
        bool endline = false;
        read = SEGGER_RTT_GetKey();

        uint32_t curtick = time_us_32();

        if ( !shownchoices && time_us_32() > starttime + 1000*1000*3)
        {
            shownchoices = true;
            buttonguide();
            //logstr("Press button");
        }

        if ( curtick > lastseen + 1000*1000 )
        {
            int32_t result = 0;
            if ( picoconnection && probe_read_memory( 0x20038000+offsetof(shareddata_t, data), (uint8_t*)&result, sizeof result) )
            {
                lastseen = time_us_32();
                drawstatus(connected, "");
            } else
            {
                openconnection();
                //connected = false;
            }
        } else if ( read == 0 )
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

        // check button presses for possible commands.

        if(button_a.read())
        {
            strcpy(str, "recover");
            charcount = strlen(str);
            endline = true;
        }

        if(button_b.read())
        {
            strcpy(str, "reflash");
            charcount = strlen(str);
            endline = true;
        }

        if(button_x.read())
        {
            strcpy(str, "blink");
            charcount = strlen(str);
            endline = true;
        }

        if(button_y.read())
        {
            strcpy(str, "usbload");
            charcount = strlen(str);
            endline = true;
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
#if 1
            if ( streql(tkn1, "send" ) )
            {
                processed = true;
                {
                    probe_flash_uf2();
                }
            }
#endif

            if ( streql(tkn1, "usbload" ) )
            {
                logstr("usbload.");
                gpio_put(LED3V3EN_PIN, 0);
                picoconnection = false;
                drawstatus(usbnotconnected, "");
                usbload();
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
                        probe_send_instruction(1);
                        //sleep_ms(20);
                        int32_t res = probe_wait_reply(5000);
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
                        probe_send_instruction(2);
                        //sleep_ms(20);
                        int32_t res = probe_wait_reply(5000);
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
                        probe_send_instruction(3);
                        //sleep_ms(20);
                        int32_t res = probe_wait_reply(20000);
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
                    probe_send_instruction(7);
                    uint32_t start = time_us_32();
                    int32_t res = probe_wait_reply(20000); // erase could take a while.

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
                    probe_send_instruction(4);
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
                    probe_send_instruction(0xff);
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
    }
#endif
}
