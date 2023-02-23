/**

Microsoft UF2

The MIT License (MIT)

Copyright (c) Microsoft Corporation

All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#ifndef UF2FORMAT_H
#define UF2FORMAT_H 1

#include <stdint.h>
#include <stdbool.h>

// All entries are little endian.

#define UF2_VERSION_BASE "v0.1.0"

#define UF2_VERSION UF2_VERSION_BASE " F"

// The largest flash size that is supported by the board, in bytes, default is 4MB
// Flash size is constrained by RAM, a 4MB size requires 2kB RAM, see MAX_BLOCKS
// Largest tested is 256MB, with 0x300000 blocks (1.5GB), 64 sectors per cluster
#ifndef CFG_UF2_FLASH_SIZE
    #define CFG_UF2_FLASH_SIZE          (2*1024*1024)
#endif

// Number of 512-byte blocks in the exposed filesystem, default is just under 32MB
// The filesystem needs space for the current file, text files, uploaded file, and FAT
#ifndef CFG_UF2_NUM_BLOCKS
    #define CFG_UF2_NUM_BLOCKS          (0x10109)
#endif

// Sectors per FAT cluster, must be increased proportionally for larger filesystems
#ifndef CFG_UF2_SECTORS_PER_CLUSTER
    #define CFG_UF2_SECTORS_PER_CLUSTER (1)
#endif

#define UF2_MAGIC_START0 0x0A324655UL // "UF2\n"
#define UF2_MAGIC_START1 0x9E5D5157UL // Randomly selected
#define UF2_MAGIC_END 0x0AB16F30UL    // Ditto

// If set, the block is "comment" and should not be flashed to the device
#define UF2_FLAG_NOFLASH 0x00000001
#define UF2_FLAG_FAMILYID 0x00002000

#define PICOFAMILYID 0xe48bff56

#define MAX_BLOCKS (CFG_UF2_FLASH_SIZE / 256 + 100)

typedef struct {
    uint32_t numBlocks;
    uint32_t numWritten;

    bool aborted;             // aborting update and reset

    bool gotname;
    uint8_t percent;

    uint8_t writtenMask[MAX_BLOCKS / 8 + 1];
} WriteState;

typedef struct {
    // 32 byte header
    uint32_t magicStart0;
    uint32_t magicStart1;
    uint32_t flags;
    uint32_t targetAddr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t familyID;

    // raw data;
    uint8_t data[476];

    // store magic also at the end to limit damage from partial block reads
    uint32_t magicEnd;
} UF2_Block;

typedef enum { none, booting, connected, connectednofs, connectedwithfs, notconnected, usbconnected, usbnotconnected } connectionstatus_t;

typedef enum { dataarea, addressarea, headerarea } datatype_t;

void drawstatus(connectionstatus_t status, const char * statusstr);
void logstrmulti(const char * str, bool multiline);
void logstr(const char * str);


void uf2_init(uint8_t file);
uint32_t uf2_get_uf2blockcount(void);
bool uf2_get_uf2filename( uint8_t fileno, char *str, uint32_t strlen);
void uf2_get_uf2block(uint32_t block_no, uint8_t *data);
void uf2_read_fsblock(uint32_t block_no, uint8_t *data);
void uf2_get_filename(uint8_t *data, uint32_t datalen, uint8_t block, WriteState *state);
void uf2_reset_namestate(void);
int  uf2_write_block(uint32_t block_no, uint8_t *data, WriteState *state);
int  uf2_write_header(void);

int32_t probe_flash_uf2(void);

void msc_reset_write( void );

void DumpHex(const void* data, size_t size);

#endif
