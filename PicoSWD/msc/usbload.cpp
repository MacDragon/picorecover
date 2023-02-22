/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "button.hpp"
extern "C" {
#include "board_api.h"
#include "uf2.h"
#include "tusb.h"
}

static volatile uint32_t _timer_count = 0;
volatile bool closeusb = false;

extern pimoroni::Button button_y;

int usbload(uint8_t file)
{
  uf2_init(file);
  tusb_init();

  printf("Start usb drive\n"); 

  tud_connect();

  bool run = true;
  bool connected = false;
  closeusb = false;

#if (CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_PICO)
  while(run)
  {
    tud_task();

    if ( !connected && tud_connected() )
    {
      connected = true;
    }

    if ( (connected && !tud_connected() )|| closeusb || button_y.read() )
    {
      run = false;
    }
  }
  tud_disconnect();

#endif

  printf("usb exit\n");
  return 1;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is plugged and configured
void tud_mount_cb(void)
{
  printf("Mounted\n"); 
  drawstatus(usbconnected, "");
  //indicator_set(STATE_USB_PLUGGED);
}

// Invoked when device is unplugged
void tud_umount_cb(void)
{
  printf("Unmounted\n"); 
  drawstatus(usbnotconnected, "");
  //indicator_set(STATE_USB_UNPLUGGED);
}

#if 0
//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}


// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // This example doesn't use multiple report and report ID
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}
#endif