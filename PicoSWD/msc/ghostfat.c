/*
 * The MIT License (MIT)
 *
 * Copyright (c) Microsoft Corporation
 * Copyright (c) Ha Thach for Adafruit Industries
 * Copyright (c) Henry Gabryjelski
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

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <inttypes.h>

#include "compile_date.h"
#include "board_api.h"
#include "uf2.h"



//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// ota0 partition size 
static uint32_t _flash_size;

#define STATIC_ASSERT(_exp) _Static_assert(_exp, "static assert failed")

#define STR0(x) #x
#define STR(x) STR0(x)

#define UF2_ARRAY_SIZE(_arr)    ( sizeof(_arr) / sizeof(_arr[0]) )
#define UF2_DIV_CEIL(_v, _d)    ( ((_v) / (_d)) + ((_v) % (_d) ? 1 : 0) )

typedef struct {
    uint8_t JumpInstruction[3];
    uint8_t OEMInfo[8];
    uint16_t SectorSize;
    uint8_t SectorsPerCluster;
    uint16_t ReservedSectors;
    uint8_t FATCopies;
    uint16_t RootDirectoryEntries;
    uint16_t TotalSectors16;
    uint8_t MediaDescriptor;
    uint16_t SectorsPerFAT;
    uint16_t SectorsPerTrack;
    uint16_t Heads;
    uint32_t HiddenSectors;
    uint32_t TotalSectors32;
    uint8_t PhysicalDriveNum;
    uint8_t Reserved;
    uint8_t ExtendedBootSig;
    uint32_t VolumeSerialNumber;
    uint8_t VolumeLabel[11];
    uint8_t FilesystemIdentifier[8];
} __attribute__((packed)) FAT_BootBlock;

typedef struct {
    char name[8];
    char ext[3];
    uint8_t attrs;
    uint8_t reserved;
    uint8_t createTimeFine;
    uint16_t createTime;
    uint16_t createDate;
    uint16_t lastAccessDate;
    uint16_t highStartCluster;
    uint16_t updateTime;
    uint16_t updateDate;
    uint16_t startCluster;
    uint32_t size;
} __attribute__((packed)) DirEntry;
STATIC_ASSERT(sizeof(DirEntry) == 32);

//https://www.kernel.org/doc/html/latest/filesystems/vfat.html

typedef struct { // Up to 13 characters of a long name
        unsigned char id;               // sequence number for slot
        unsigned char name0_4[10];      // first 5 characters in name
        unsigned char attr;             // attribute byte
        unsigned char reserved;         // always 0
        unsigned char alias_checksum;   // checksum for 8.3 alias
        unsigned char name5_10[12];     // 6 more characters in name
        unsigned char start[2];         // starting cluster number
        unsigned char name11_12[4];     // last 2 characters in name
} __attribute__((packed)) LFNDirEntry;


typedef struct FileContent {
  char const name[11];
  char const longname[234]; // 9 * 26 for a decent length lfn that fits in under 256bytes
  void const * content;
  uint32_t size;       // OK to use uint32_T b/c FAT32 limits filesize to (4GiB - 2)

  // computing fields based on index and size
  uint16_t cluster_start;
  uint16_t cluster_end;
} FileContent_t;


//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

#define BPB_SECTOR_SIZE           ( 512)
#define BPB_SECTORS_PER_CLUSTER   (CFG_UF2_SECTORS_PER_CLUSTER)
#define BPB_RESERVED_SECTORS      (   1)
#define BPB_NUMBER_OF_FATS        (   2)
#define BPB_ROOT_DIR_ENTRIES      (  64)
#define BPB_TOTAL_SECTORS         CFG_UF2_NUM_BLOCKS
#define BPB_MEDIA_DESCRIPTOR_BYTE (0xF8)
#define FAT_ENTRY_SIZE            (2)
#define FAT_ENTRIES_PER_SECTOR    (BPB_SECTOR_SIZE / FAT_ENTRY_SIZE)
#define FAT_END_OF_CHAIN          (0xFFFF)

// NOTE: MS specification explicitly allows FAT to be larger than necessary
#define TOTAL_CLUSTERS_ROUND_UP   UF2_DIV_CEIL(BPB_TOTAL_SECTORS, BPB_SECTORS_PER_CLUSTER)
#define BPB_SECTORS_PER_FAT       UF2_DIV_CEIL(TOTAL_CLUSTERS_ROUND_UP, FAT_ENTRIES_PER_SECTOR)
#define DIRENTRIES_PER_SECTOR     (BPB_SECTOR_SIZE/sizeof(DirEntry))
#define ROOT_DIR_SECTOR_COUNT     UF2_DIV_CEIL(BPB_ROOT_DIR_ENTRIES, DIRENTRIES_PER_SECTOR)
#define BPB_BYTES_PER_CLUSTER     (BPB_SECTOR_SIZE * BPB_SECTORS_PER_CLUSTER)

STATIC_ASSERT((BPB_SECTORS_PER_CLUSTER & (BPB_SECTORS_PER_CLUSTER-1)) == 0); // sectors per cluster must be power of two
STATIC_ASSERT(BPB_SECTOR_SIZE                              ==       512); // GhostFAT does not support other sector sizes (currently)
STATIC_ASSERT(BPB_NUMBER_OF_FATS                           ==         2); // FAT highest compatibility
STATIC_ASSERT(sizeof(DirEntry)                             ==        32); // FAT requirement
STATIC_ASSERT(BPB_SECTOR_SIZE % sizeof(DirEntry)           ==         0); // FAT requirement
STATIC_ASSERT(BPB_ROOT_DIR_ENTRIES % DIRENTRIES_PER_SECTOR ==         0); // FAT requirement
STATIC_ASSERT(BPB_BYTES_PER_CLUSTER                        <= (32*1024)); // FAT requirement (64k+ has known compatibility problems)
STATIC_ASSERT(FAT_ENTRIES_PER_SECTOR                       ==       256); // FAT requirement

#define UF2_FIRMWARE_BYTES_PER_SECTOR   256
#define UF2_SECTOR_COUNT                (_flash_size / UF2_FIRMWARE_BYTES_PER_SECTOR)
#define UF2_BYTE_COUNT                  (UF2_SECTOR_COUNT * BPB_SECTOR_SIZE) // always a multiple of sector size, per UF2 spec


const char infoUf2File[] =
    "PicoSWD recover " UF2_VERSION "\r\n"
    "Model: " UF2_PRODUCT_NAME "\r\n"
    "Board-ID: " UF2_BOARD_ID "\r\n";

const char indexFile[] =
    "<!doctype html>\n"
    "<html>"
    "<body>"
    "<script>\n"
    "location.replace(\"" UF2_INDEX_URL "\");\n"
    "</script>"
    "</body>"
    "</html>\n";

#ifdef TINYUF2_FAVICON_HEADER
#include TINYUF2_FAVICON_HEADER
const char autorunFile[] = "[Autorun]\r\nIcon=FAVICON.ICO\r\n";
#endif

// size of CURRENT.UF2:
static FileContent_t info[] = {
    {.name = "INFO_UF2TXT", .content = infoUf2File , .size = sizeof(infoUf2File) - 1},
    {.name = "INDEX   HTM", .content = indexFile   , .size = sizeof(indexFile  ) - 1},
#ifdef TINYUF2_FAVICON_HEADER
    {.name = "AUTORUN INF", .content = autorunFile , .size = sizeof(autorunFile) - 1},
    {.name = "FAVICON ICO", .content = favicon_data, .size = favicon_len            },
#endif
    // current.uf2 must be the last element and its content must be NULL
    {.name = "NODATA     ", .content = NULL       , .size = 0                       },
};

uint32_t blockaddresses[((384*4096)/256)]; // size has to be multiple of 256
typedef union {
  uint8_t writeblock[4096]; // make the structure be 4k
  struct {
    char shortname[11];
    char longname[234];
    uint32_t blocks;
    uint32_t crc32;
    uint32_t familyid;
  };
} headerdata_t;

headerdata_t headerdata;

enum {
  NUM_FILES = sizeof(info) / sizeof(info[0]),
  FID_UF2 = NUM_FILES-1,
  NUM_DIRENTRIES = NUM_FILES + 1 // including volume label as first root directory entry
};

STATIC_ASSERT(NUM_DIRENTRIES < BPB_ROOT_DIR_ENTRIES);  // FAT requirement -- Ensures BPB reserves sufficient entries for all files
STATIC_ASSERT(NUM_DIRENTRIES < DIRENTRIES_PER_SECTOR); // GhostFAT bug workaround -- else, code overflows buffer

#define NUM_SECTORS_IN_DATA_REGION (BPB_TOTAL_SECTORS - BPB_RESERVED_SECTORS - (BPB_NUMBER_OF_FATS * BPB_SECTORS_PER_FAT) - ROOT_DIR_SECTOR_COUNT)
#define CLUSTER_COUNT              (NUM_SECTORS_IN_DATA_REGION / BPB_SECTORS_PER_CLUSTER)

// Ensure cluster count results in a valid FAT16 volume!
STATIC_ASSERT( CLUSTER_COUNT >= 0x0FF5 && CLUSTER_COUNT < 0xFFF5 );

// Many existing FAT implementations have small (1-16) off-by-one style errors
// So, avoid being within 32 of those limits for even greater compatibility.
STATIC_ASSERT( CLUSTER_COUNT >= 0x1015 && CLUSTER_COUNT < 0xFFD5 );


#define FS_START_FAT0_SECTOR      BPB_RESERVED_SECTORS
#define FS_START_FAT1_SECTOR      (FS_START_FAT0_SECTOR + BPB_SECTORS_PER_FAT)
#define FS_START_ROOTDIR_SECTOR   (FS_START_FAT1_SECTOR + BPB_SECTORS_PER_FAT)
#define FS_START_CLUSTERS_SECTOR  (FS_START_ROOTDIR_SECTOR + ROOT_DIR_SECTOR_COUNT)

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static FAT_BootBlock const BootBlock = {
    .JumpInstruction      = {0xeb, 0x3c, 0x90},
    .OEMInfo              = "UF2 UF2 ",
    .SectorSize           = BPB_SECTOR_SIZE,
    .SectorsPerCluster    = BPB_SECTORS_PER_CLUSTER,
    .ReservedSectors      = BPB_RESERVED_SECTORS,
    .FATCopies            = BPB_NUMBER_OF_FATS,
    .RootDirectoryEntries = BPB_ROOT_DIR_ENTRIES,
    .TotalSectors16       = (BPB_TOTAL_SECTORS > 0xFFFF) ? 0 : BPB_TOTAL_SECTORS,
    .MediaDescriptor      = BPB_MEDIA_DESCRIPTOR_BYTE,
    .SectorsPerFAT        = BPB_SECTORS_PER_FAT,
    .SectorsPerTrack      = 1,
    .Heads                = 1,
    .TotalSectors32       = (BPB_TOTAL_SECTORS > 0xFFFF) ? BPB_TOTAL_SECTORS : 0,
    .PhysicalDriveNum     = 0x80, // to match MediaDescriptor of 0xF8
    .ExtendedBootSig      = 0x29,
    .VolumeSerialNumber   = 0x00420042,
    .VolumeLabel          = UF2_VOLUME_LABEL,
    .FilesystemIdentifier = "FAT16   ",
};

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static inline bool is_uf2_block (UF2_Block const *bl)
{
  return (bl->magicStart0 == UF2_MAGIC_START0) &&
         (bl->magicStart1 == UF2_MAGIC_START1) &&
         (bl->magicEnd == UF2_MAGIC_END) &&
         (bl->flags & UF2_FLAG_FAMILYID) &&
         !(bl->flags & UF2_FLAG_NOFLASH);
}

// cache the cluster start offset for each file
// this allows more flexible algorithms w/o O(n) time
static void init_starting_clusters(void)
{
  // +2 because FAT decided first data sector would be in cluster number 2, rather than zero
  uint16_t start_cluster = 2;

  for (uint16_t i = 0; i < NUM_FILES; i++)
  {
    uint32_t clustersize = BPB_SECTOR_SIZE*BPB_SECTORS_PER_CLUSTER;
    printf("File %d start cluster %d addr %08x\n", i, start_cluster, start_cluster*clustersize);
    info[i].cluster_start = start_cluster;
    info[i].cluster_end = start_cluster + UF2_DIV_CEIL(info[i].size, BPB_SECTOR_SIZE*BPB_SECTORS_PER_CLUSTER) - 1;

    start_cluster = info[i].cluster_end + 1;

  }

  return;
}

// get file index for file that uses the cluster
// if cluster is past last file, returns ( NUM_FILES-1 ).
//
// Caller must still check if a particular *sector*
// contains data from the file's contents, as there
// are often padding sectors, including all the unused
// sectors past the end of the media.
static uint32_t info_index_of(uint32_t cluster)
{
  // default results for invalid requests is the index of the last file (CURRENT.UF2)
  if (cluster >= 0xFFF0) return FID_UF2;

  for (uint32_t i = 0; i < NUM_FILES; i++)
  {
    if ( (info[i].cluster_start <= cluster) && (cluster <= info[i].cluster_end) )
    {
      return i;
    }
  }

  return FID_UF2;
}

void uf2_init(void)
{
  // TODO maybe limit to application size only if possible board_flash_app_size()
  _flash_size = board_flash_size();

  // read in possible header data.
  board_flash_read(4096*6, &headerdata, 4096, true);

  if ( headerdata.shortname[0] != 0 && headerdata.shortname[0] != 0xff)
  {
    // we have a header!
    info[FID_UF2].size = headerdata.blocks*512;
    printf("Setting filesize %lu", info[FID_UF2].size);
    memcpy(info[FID_UF2].name, headerdata.shortname, 11);
    if ( headerdata.longname[0] != 0 && headerdata.longname[0] != 0xff)
    {
      memcpy(info[FID_UF2].longname, headerdata.longname, sizeof headerdata.longname);
    }
  }
  else
  {
    // update CURRENT.UF2 file size
    info[FID_UF2].size = 0;//UF2_BYTE_COUNT;
  }

  init_starting_clusters();
}

/*------------------------------------------------------------------*/
/* Read CURRENT.UF2
 *------------------------------------------------------------------*/
void padded_memcpy (char *dst, char const *src, int len)
{
  for ( int i = 0; i < len; ++i )
  {
    if ( *src ) {
      *dst = *src++;
    } else {
      *dst = ' ';
    }
    dst++;
  }
}

typedef enum { nolfn, gotlfn, fulllfn } lfnstate;

static uint8_t dirbuffer[2048];
static uint8_t blockswritten;

static bool gotname = false;


void uf2_reset_namestate(void)
{
  memset(dirbuffer, 0, sizeof dirbuffer);
  // blockswritten = 0;
}

void uf2_get_filename(uint8_t *data, uint32_t datalen, uint8_t block, WriteState *state)
{
    if ( state->gotname )
      return;

    memcpy(dirbuffer+block*512, data, 512);

#if 0
    blockswritten |= ( 1 << block );

    for ( int i=0;i<block;i++)
    {
      if ( ( blockswritten & ( 1 < i ) ) == 0 )
      {
        printf("missing block %d %02x\n", i, blockswritten);
        return;
      }
    }
  #endif

    char longname[234];
    uint8_t longnameposition = 0;
    uint8_t longnamechecksum = 0;
    lfnstate lfnstate = nolfn;

    DirEntry *d = (void*) dirbuffer;                   // pointer to next free DirEntry this sector
    int remainingEntries = DIRENTRIES_PER_SECTOR; // remaining count of DirEntries this sector
    int count = 0;
    while ( d->name[0] != 0 && count*32 < sizeof dirbuffer )
    {
      if ( d->attrs == 0x0f )
      {
          LFNDirEntry * lfn = (LFNDirEntry*)d;

          if ( lfn->id & 0x40 )
          {
            longname[0] = 0;
            longnameposition = lfn->id & 0b1111;

            if ( longnameposition <= (sizeof longname)/26 ) // maximum allowable length to scan in as LFN.
            {
              //printf("Longname entry start %d parts\n", longnameposition);
              lfnstate = gotlfn;
              memset(longname, 0, 4);
              memset(longname+4, 0xff, sizeof longname-4);
            }
            else
            {
              //printf("Longname entry start %d parts is too long, ignoring\n", longnameposition);
              lfnstate = nolfn;
            }
            longnamechecksum = lfn->alias_checksum;
          }

          if ( lfnstate == gotlfn ) // currently processing a lfn, add or abandon.
          {
            if ( ( lfn->id & 0b1111 ) == longnameposition ) // expected value, continue processing.
            {
                //printf("adding to filename at position %d\n", (longnameposition-1)*26);
                // copy the parts of the filename to final.
                memcpy(longname+(longnameposition-1)*26, lfn->name0_4, 10);
                memcpy(longname+(longnameposition-1)*26+10, lfn->name5_10, 12);
                memcpy(longname+(longnameposition-1)*26+10+12, lfn->name11_12, 4);
                if (longnameposition == 1)
                {
                  //printf("Longname entry complete\n");
                  lfnstate = fulllfn;
                } else
                {
                  //printf("Longname entry %d\n", lfn->id & 0b1111);
                  longnameposition--;
                }
            } else
            {
              printf("Wrong lfn entry\n");
              lfnstate = nolfn;
            }
          }

      } else if ( d->attrs & 0x08 )
      {
        #if 0
        char volume[12] = "";

        memcpy(volume, d->name, 11);
        for ( int i=0;i<11;i++)
          if ( volume[i] == 0x20 ) volume[i] = 0;
        printf("Volume label entry : %s\n", volume);
        #endif
        lfnstate = nolfn;
      }
      else //if ( d->attrs & 0x02 == 0 )// regular file name, not hidden.
      {
        // check if name matches an existing one, if so ignore it.
        bool existing = false;
        for ( int i=0;i<NUM_FILES;i++)
        {
            if ( memcmp(info[i].name,d->name, 11) == 0 )
            {
              if ( d->updateTime == COMPILE_DOS_TIME && d->updateDate == COMPILE_DOS_DATE )
              {
                existing = true;
              } else
              {
                printf("Existing file, but timestamp changed, overwritten?\n");
              }
            }
        }

        if ( !existing )
        {
          char name[9] = "";
          char extention[4] = "";
          memcpy(name, d->name, 8);
          memcpy(extention, d->name+8, 3);
          for ( int i=0;i<8;i++)
            if ( name[i] == 0x20 ) name[i] = 0;
          for ( int i=0;i<3;i++)
            if ( extention[i] == 0x20 ) extention[i] = 0;
          
          uint8_t sum;
          uint8_t i;
          // https://www.kernel.org/doc/html/latest/filesystems/vfat.html
          for (sum = i = 0; i < 11; i++) {
            sum = (((sum&1)<<7)|((sum&0xfe)>>1)) + d->name[i];
          }

          if ( sum != longnamechecksum )
          {
            lfnstate == nolfn;
          }

          // realistically a uf2 file is not going to be smaller than 4k, eliminates mac directory helper files.
          if ( strcmp(extention, "UF2") == 0 && !(d->attrs & 0x02) && d->size > 0 )
          {
            printf("UF2 File name entry %s.%s size %d\n", name, extention, d->size);
            memcpy(info[NUM_FILES-1].name, d->name, 11);
            memcpy(headerdata.shortname, d->name, 11);
            if ( lfnstate == fulllfn )
            {
              memcpy(info[NUM_FILES-1].longname, longname, sizeof longname);
              memcpy(headerdata.longname, longname, sizeof longname);
            }

            // do we have a long name?
            state->gotname = true;
          } else
          {
            //printf("File name entry %s.%s size %d lfn:%c\n", name, extention, d->size, lfnstate == fulllfn?'y':'n');
          }
          #if 0
          if ( lfnstate == fulllfn )
          {
              printf("Longname : ");
              int i;
              for ( i = 0; i<sizeof longname; i+=2 )
              {
                if ( longname[i] == 0 || longname[i] == 0xff )
                  break;
                printf("%c", longname[i]);
              }
              printf(" length %d\n", i);
          }
          #endif

          if (state->gotname)
          {
            printf("Name set\n");
            return; // we got out name get out of here!
          }
        }
        lfnstate = nolfn; // reset long name
      }

      count++;
      d++;
    } 
}

bool showndump = false;

void uf2_read_block (uint32_t block_no, uint8_t *data)
{
  memset(data, 0, BPB_SECTOR_SIZE);
  uint32_t sectionRelativeSector = block_no;

  if ( block_no == 0 )
  {
    // Request was for the Boot block
    memcpy(data, &BootBlock, sizeof(BootBlock));
    data[510] = 0x55;    // Always at offsets 510/511, even when BPB_SECTOR_SIZE is larger
    data[511] = 0xaa;    // Always at offsets 510/511, even when BPB_SECTOR_SIZE is larger
  }
  else if ( block_no < FS_START_ROOTDIR_SECTOR )
  {
    // Request was for a FAT table sector
    sectionRelativeSector -= FS_START_FAT0_SECTOR;

    // second FAT is same as the first... use sectionRelativeSector to write data
    if ( sectionRelativeSector >= BPB_SECTORS_PER_FAT )
    {
      sectionRelativeSector -= BPB_SECTORS_PER_FAT;
    }

    uint16_t* data16 = (uint16_t*) (void*) data;

    uint32_t sectorFirstCluster = sectionRelativeSector * FAT_ENTRIES_PER_SECTOR;
    uint32_t firstUnusedCluster = info[FID_UF2].cluster_end + 1;

    // OPTIMIZATION:
    // Because all files are contiguous, the FAT CHAIN entries
    // are all set to (cluster+1) to point to the next cluster.
    // All clusters past the last used cluster of the last file
    // are set to zero.
    //
    // EXCEPTIONS:
    // 1. Clusters 0 and 1 require special handling
    // 2. Final cluster of each file must be set to END_OF_CHAIN
    // 

    // Set default FAT values first.
    for (uint16_t i = 0; i < FAT_ENTRIES_PER_SECTOR; i++)
    {
      uint32_t cluster = i + sectorFirstCluster;
      if (cluster >= firstUnusedCluster)
      {
        data16[i] = 0;
      }
      else
      {
        data16[i] = cluster + 1;
      }
    }

    // Exception #1: clusters 0 and 1 need special handling
    if (sectionRelativeSector == 0)
    {
      data[0] = BPB_MEDIA_DESCRIPTOR_BYTE;
      data[1] = 0xff;
      data16[1] = FAT_END_OF_CHAIN; // cluster 1 is reserved
    }

    // Exception #2: the final cluster of each file must be set to END_OF_CHAIN
    for (uint32_t i = 0; i < NUM_FILES; i++)
    {
      uint32_t lastClusterOfFile = info[i].cluster_end;
      if (lastClusterOfFile >= sectorFirstCluster)
      {
        uint32_t idx = lastClusterOfFile - sectorFirstCluster;
        if (idx < FAT_ENTRIES_PER_SECTOR)
        {
          // that last cluster of the file is in this sector
          data16[idx] = FAT_END_OF_CHAIN;
        }
      }
    }
  }
  else if ( block_no < FS_START_CLUSTERS_SECTOR )
  {
    // Request was for a (root) directory sector .. root because not supporting subdirectories (yet)
    sectionRelativeSector -= FS_START_ROOTDIR_SECTOR;

    DirEntry *d = (void*) data;                   // pointer to next free DirEntry this sector
    int remainingEntries = DIRENTRIES_PER_SECTOR; // remaining count of DirEntries this sector

    //printf("remainingdirEntries %d\n", remainingEntries);

    uint32_t startingFileIndex;

    if ( sectionRelativeSector == 0 )
    {
      // volume label is first directory entry
      padded_memcpy(d->name, (char const*) BootBlock.VolumeLabel, 11);
      d->attrs = 0x28;
      d++;
      remainingEntries--;

      startingFileIndex = 0;
    }else
    {
      // -1 to account for volume label in first sector
      startingFileIndex = DIRENTRIES_PER_SECTOR * sectionRelativeSector - 1;
    }

    for ( uint32_t fileIndex = startingFileIndex;
          remainingEntries > 0 && fileIndex < NUM_FILES; // while space remains in buffer and more files to add...
          fileIndex++, d++ )
    {
      // WARNING -- code presumes all files take exactly one directory entry (no long file names!)
#if 0
      if ( strlen(info[fileIndex].longname) > 0 )
      {
        // we have a long filename, lets store it.
      }
#endif
      uint32_t const startCluster = info[fileIndex].cluster_start;

      FileContent_t const *inf = &info[fileIndex];

      if ( fileIndex == NUM_FILES-1 && info[fileIndex].longname[0] != 0 ) // if our custom file has a long filename, add it.
      {
          uint8_t sum;
          uint8_t i;
          // https://www.kernel.org/doc/html/latest/filesystems/vfat.html
          for (sum = i = 0; i < 11; i++) {
            sum = (((sum&1)<<7)|((sum&0xfe)>>1)) + info[fileIndex].name[i];
          }

          int namelen=0;
          //printf("Adding Longname : ");
          for ( ; namelen<sizeof info[fileIndex].longname; namelen+=2 )
          {
            if ( info[fileIndex].longname[namelen] == 0 || info[fileIndex].longname[namelen] == 0xff )
              break;
            //printf("%c", info[fileIndex].longname[namelen]);
          }
          uint8_t entries = (namelen+25)/26;
          //printf(" length %d in %d entries, setting SFN\n", namelen, entries);

          bool first = true;
          while ( entries > 0 )
          {
            LFNDirEntry * lfn = (void*)d;

            lfn->id = entries;
            if ( first )
            {
              lfn->id |= (1 << 6);
              first = false;
            }
            lfn->alias_checksum = sum;
            lfn->attr = 0xf;
            lfn->reserved = 0;
            lfn->start[0] = 0;
            lfn->start[1] = 0;

            char *namepos = &info[fileIndex].longname[(entries-1)*26];

            memcpy(lfn->name0_4, namepos, 10);
            memcpy(lfn->name5_10, namepos+10, 12);
            memcpy(lfn->name11_12, namepos+10+12, 4);
            d++;
            entries--;

            // if saving a LFN, then set the SFN to something the OS will overwrite so we can catch it.
            // set a fake SFN to get OS to overwrite it so we can detect the UF2 filename more easily later.
            padded_memcpy(d->name, info[fileIndex].name, 11);

          } 
      } else
        padded_memcpy(d->name, inf->name, 11);
      d->createTimeFine   = COMPILE_SECONDS_INT % 2 * 100;
      d->createTime       = COMPILE_DOS_TIME;
      d->createDate       = COMPILE_DOS_DATE;
      d->lastAccessDate   = COMPILE_DOS_DATE;
      d->highStartCluster = startCluster >> 16;
      d->updateTime       = COMPILE_DOS_TIME;
      d->updateDate       = COMPILE_DOS_DATE;
      d->startCluster     = startCluster & 0xFFFF;
      printf("file %d size %d\n", fileIndex, inf->size);
      d->size             = inf->size; //(inf->content ? inf->size : UF2_BYTE_COUNT);
    }

#if 0
    if ( !showndump )
    {
      showndump = true;
      printf("Generated dir entry block\n");
      DumpHex(data, 512);
    }
#endif

  }
  else if ( block_no < BPB_TOTAL_SECTORS )
  {
    // Request was to read from the data area (files, unused space, ...)
    sectionRelativeSector -= FS_START_CLUSTERS_SECTOR;

    // plus 2 for first data cluster offset
    uint32_t fid = info_index_of(2 + sectionRelativeSector / BPB_SECTORS_PER_CLUSTER);
    FileContent_t const * inf = &info[fid];

    uint32_t fileRelativeSector = sectionRelativeSector - (info[fid].cluster_start-2) * BPB_SECTORS_PER_CLUSTER;
    
    if ( fid != FID_UF2 )
    {
      // Handle all files other than CURRENT.UF2
      size_t fileContentStartOffset = fileRelativeSector * BPB_SECTOR_SIZE;
      size_t fileContentLength = inf->size;

      // nothing to copy if already past the end of the file (only when >1 sector per cluster)
      if (fileContentLength > fileContentStartOffset) {
        // obviously, 2nd and later sectors should not copy data from the start
        const void * dataStart = (inf->content) + fileContentStartOffset;
        // limit number of bytes of data to be copied to remaining valid bytes
        size_t bytesToCopy = fileContentLength - fileContentStartOffset;
        // and further limit that to a single sector at a time
        if (bytesToCopy > BPB_SECTOR_SIZE) {
          bytesToCopy = BPB_SECTOR_SIZE;
        }
        memcpy(data, dataStart, bytesToCopy);
      }
    }
    else
    {
      // CURRENT.UF2: generate data on-the-fly
      uint32_t addr = BOARD_FLASH_APP_START + (fileRelativeSector * UF2_FIRMWARE_BYTES_PER_SECTOR);
      if ( addr < _flash_size ) // TODO abstract this out
      {
        UF2_Block *bl = (void*) data;
        bl->magicStart0 = UF2_MAGIC_START0;
        bl->magicStart1 = UF2_MAGIC_START1;
        bl->magicEnd = UF2_MAGIC_END;
        bl->blockNo = fileRelativeSector;
        bl->numBlocks = UF2_SECTOR_COUNT;
        bl->targetAddr = addr;
        bl->payloadSize = UF2_FIRMWARE_BYTES_PER_SECTOR;
        bl->flags = UF2_FLAG_FAMILYID;
        bl->familyID = BOARD_UF2_FAMILY_ID;

        board_flash_read(addr, bl->data, bl->payloadSize, false);
      }
    }
  }
}

/*------------------------------------------------------------------*/
/* Write UF2
 *------------------------------------------------------------------*/

int uf2_write_header(void)
{
    printf("Storing header data\n");
    board_flash_write(0, blockaddresses, sizeof blockaddresses, true);
    board_flash_write(4096*6, &headerdata, sizeof headerdata, true);
    //wDumpHex(blockaddresses, 384);
}

/**
 * Write an uf2 block wrapped by 512 sector.
 * @return number of bytes processed, only 3 following values
 *  -1 : if not an uf2 block
 * 512 : write is successful (BPB_SECTOR_SIZE == 512)
 *   0 : is busy with flashing, tinyusb stack will call write_block again with the same parameters later on
 */
int uf2_write_block (uint32_t block_no, uint8_t *data, WriteState *state)
{
  (void) block_no;
  UF2_Block *bl = (void*) data;

  if ( !is_uf2_block(bl) )
  {
      return -1; // ignore anything that\s not a uf2 block in input stream.
  }

  if (bl->familyID == BOARD_UF2_FAMILY_ID)
  {
    // generic family ID
    if (bl->blockNo ==1)
    {
      printf("Writing UF2 block %d %d/%d %dB target addr %08x\n", block_no, bl->blockNo+1, bl->numBlocks, bl->payloadSize, bl->targetAddr);

      board_flash_write(bl->blockNo*256, bl->data, bl->payloadSize, false);
    }
    // also store address in header block
    blockaddresses[bl->blockNo] = bl->targetAddr;
  }else
  {
    // TODO family matches VID/PID
    return -1;
  }

  //------------- Update written blocks -------------//
  if ( bl->numBlocks )
  {
    // Update state num blocks if needed
    if ( state->numBlocks != bl->numBlocks )
    {
      if ( bl->numBlocks >= MAX_BLOCKS || state->numBlocks )
        state->numBlocks = 0xffffffff;
      else
        state->numBlocks = bl->numBlocks;

        headerdata.blocks = bl->numBlocks; // store the number of uf2 blocks to header data.
        printf("Storing %d blocks to header\n", headerdata.blocks);
    }

    if ( bl->blockNo < MAX_BLOCKS )
    {
      uint8_t const mask = 1 << (bl->blockNo % 8);
      uint32_t const pos = bl->blockNo / 8;

      // only increase written number with new write (possibly prevent overwriting from OS)
      if ( !(state->writtenMask[pos] & mask) )
      {
        state->writtenMask[pos] |= mask;
        state->numWritten++;
      }

      // flush last blocks
      // TODO numWritten can be smaller than numBlocks if return early
      if ( state->numWritten >= state->numBlocks )
      {
        board_flash_flush();
      }
    }
  }

  return BPB_SECTOR_SIZE;
}
