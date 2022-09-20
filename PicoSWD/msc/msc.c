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

#include "tusb.h"
#include "uf2.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define DEBUG_SPEED_TEST  0

#if DEBUG_SPEED_TEST
#include "esp_log.h"
static uint32_t _write_ms;
#endif

static WriteState _wr_state = { 0 };

void msc_reset_write( void )
{
    memset(&_wr_state, 0, sizeof _wr_state); // reset write state so if opened again will complete.
}

//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;

  const char vid[] = "Adafruit";
  const char pid[] = "UF2 Bootloader";
  const char rev[] = "1.0";

  memcpy(vendor_id  , vid, strlen(vid));
  memcpy(product_id , pid, strlen(pid));
  memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  void const* response = NULL;
  uint16_t resplen = 0;

  // most scsi handled is input
  bool in_xfer = true;

  switch (scsi_cmd[0])
  {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      // Host is about to read/write etc ... better not to disconnect disk
      resplen = 0;
    break;

    default:
      // Set Sense = Invalid Command Operation
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

      // negative means error -> tinyusb could stall and/or response with failed status
      resplen = -1;
    break;
  }

  // return resplen must not larger than bufsize
  if ( resplen > bufsize ) resplen = bufsize;

  if ( response && (resplen > 0) )
  {
    if(in_xfer)
    {
      memcpy(buffer, response, resplen);
    }else
    {
      // SCSI output
    }
  }

  return resplen;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb (uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) lun;
  memset(buffer, 0, bufsize);

  // since we return block size each, offset should always be zero
  TU_ASSERT(offset == 0, -1);

  uint32_t count = 0;

  while ( count < bufsize )
  {
    uf2_read_block(lba, buffer);

    lba++;
    buffer += 512;
    count  += 512;
  }

  return count;
}

void DumpHex(const void* data, size_t size) {
	char ascii[17];
	size_t i, j;
	ascii[16] = '\0';
	for (i = 0; i < size; ++i) {
		printf("%02X ", ((unsigned char*)data)[i]);
		if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char*)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i+1) % 8 == 0 || i+1 == size) {
			printf(" ");
			if ((i+1) % 16 == 0) {
				printf("|  %s \n", ascii);
			} else if (i+1 == size) {
				ascii[(i+1) % 16] = '\0';
				if ((i+1) % 16 <= 8) {
					printf(" ");
				}
				for (j = (i+1) % 16; j < 16; ++j) {
					printf("   ");
				}
				printf("|  %s \n", ascii);
			}
		}
	}
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb (uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  (void) lun;
  (void) offset;

  printf("Usb write to %08x of %d\n", lba, bufsize);

  if ( lba == 0x205 ) // root file table being writte, we can get file name from here.
  {
    uf2_get_filename(buffer, bufsize, 0, &_wr_state);
    //DumpHex(buffer, bufsize);
  }

  if ( lba == 0x206 ) // root file table being writte, we can get file name from here.
  {
    uf2_get_filename(buffer, bufsize, 1, &_wr_state);
    //DumpHex(buffer, bufsize);
  }

  uint32_t count = 0;
  while ( count < bufsize )
  {
    // Consider non-uf2 block write as successful
    // only break if write_block is busy with flashing (return 0)
    if ( 0 == uf2_write_block(lba, buffer, &_wr_state) ) break;

    // write block to flash data area.
    // keep list of erased area.
    // erase as go, mark as erased to avoid processing again.

    lba++;
    buffer += 512;
    count  += 512;
  }

  return count;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
void tud_msc_write10_complete_cb(uint8_t lun)
{
  (void) lun;
  static bool first_write = true;

  // abort the DFU, uf2 block failed integrity check
  if ( _wr_state.aborted )
  {
    // aborted and reset
    printf("write aborted\n");
    //indicator_set(STATE_WRITING_FINISHED);
  }
  else if ( _wr_state.numBlocks )
  {
    // Start LED writing pattern with first write
    if (first_write)
    {
      #if DEBUG_SPEED_TEST
      _write_ms = esp_log_timestamp();
      #endif

      first_write = false;
      printf("write started\n");
      //indicator_set(STATE_WRITING_STARTED);
    }

    // All block of uf2 file is complete --> complete DFU process
    if (_wr_state.numWritten >= _wr_state.numBlocks && _wr_state.gotname )
    {
      #if DEBUG_SPEED_TEST
      uint32_t const wr_byte = _wr_state.numWritten*256;
      _write_ms = esp_log_timestamp()-_write_ms;
      printf("written %u bytes in %.02f seconds.\r\n", wr_byte, _write_ms / 1000.0F);
      printf("Speed : %.02f KB/s\r\n", (wr_byte / 1000.0F) / (_write_ms / 1000.0F));
      #endif

      printf("uf2 write complete\n");
      tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
      //indicator_set(STATE_WRITING_FINISHED);
      //board_dfu_complete();

      // board_dfu_complete() should not return
      // getting here is an indicator of error
      //while(1) {}
      tud_disconnect();
    }
  }
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;

  *block_count = CFG_UF2_NUM_BLOCKS;
  *block_size  = 512;

   printf("Returning size %d:%d->%dB\n", *block_count, *block_size, *block_count * *block_size);
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;

  if ( load_eject )
  {
    if (start)
    {
       printf("Load requested\n");
      // load disk storage
    }else
    {
      printf("Eject requested\n");
      tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
      // unload disk storage
      return false;
    }
  }

  return true;
}
