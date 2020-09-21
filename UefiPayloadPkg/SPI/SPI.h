/** @file  BlSMMStoreDxe.h

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#ifndef __SPI_H__
#define __SPI_H__


#include <Base.h>
#include <PiDxe.h>

#include <Guid/EventGroup.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DiskIo.h>
#include <Protocol/FirmwareVolumeBlock.h>

#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeLib.h>

#define SMMSTORE_SIGNATURE                       SIGNATURE_32('S', 'M', 'M', 'S')
#define INSTANCE_FROM_FVB_THIS(a)                CR(a, SMMSTORE_INSTANCE, FvbProtocol, SMMSTORE_SIGNATURE)

typedef struct _SMMSTORE_INSTANCE                SMMSTORE_INSTANCE;

#pragma pack (1)
typedef struct {
  VENDOR_DEVICE_PATH                  Vendor;
  UINT8                               Index;
  EFI_DEVICE_PATH_PROTOCOL            End;
} NOR_FLASH_DEVICE_PATH;
#pragma pack ()

struct _SMMSTORE_INSTANCE {
  UINT32                              Signature;
  EFI_HANDLE                          Handle;
  EFI_BLOCK_IO_MEDIA                  Media;

  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol;

  NOR_FLASH_DEVICE_PATH               DevicePath;
};

//
// BlSMMStoreFvbDxe.c
//

EFI_STATUS
EFIAPI
SMMStoreFvbInitialize (
  IN SMMSTORE_INSTANCE*                            Instance
  );

EFI_STATUS
EFIAPI
FvbGetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  OUT       EFI_FVB_ATTRIBUTES_2                    *Attributes
  );

EFI_STATUS
EFIAPI
FvbSetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN OUT    EFI_FVB_ATTRIBUTES_2                    *Attributes
  );

EFI_STATUS
EFIAPI
FvbGetPhysicalAddress(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  OUT       EFI_PHYSICAL_ADDRESS                    *Address
  );

EFI_STATUS
EFIAPI
FvbGetBlockSize(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN        EFI_LBA                                 Lba,
  OUT       UINTN                                   *BlockSize,
  OUT       UINTN                                   *NumberOfBlocks
  );

EFI_STATUS
EFIAPI
FvbRead(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN        EFI_LBA                                 Lba,
  IN        UINTN                                   Offset,
  IN OUT    UINTN                                   *NumBytes,
  IN OUT    UINT8                                   *Buffer
  );

EFI_STATUS
EFIAPI
FvbWrite(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN        EFI_LBA                                 Lba,
  IN        UINTN                                   Offset,
  IN OUT    UINTN                                   *NumBytes,
  IN        UINT8                                   *Buffer
  );

EFI_STATUS
EFIAPI
FvbEraseBlocks(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  ...
  );

#define SMMSTORE_COMBUF_SIZE 16

/**
  Read from SMMStore

  @param[in] Lba      The starting logical block index to read from.
  @param[in] Offset   Offset into the block at which to begin reading.
  @param[in] NumBytes On input, indicates the requested read size. On
                      output, indicates the actual number of bytes read
  @param[in] Buffer   Pointer to the buffer to read into.

**/
EFI_STATUS
SMMStoreRead (
  IN        EFI_LBA                              Lba,
  IN        UINTN                                Offset,
  IN        UINTN                                *NumBytes,
  IN        UINT8                                *Buffer
  );


/**
  Write to SMMStore

  @param[in] Lba      The starting logical block index to write to.
  @param[in] Offset   Offset into the block at which to begin writing.
  @param[in] NumBytes On input, indicates the requested write size. On
                      output, indicates the actual number of bytes written
  @param[in] Buffer   Pointer to the data to write.

**/
EFI_STATUS
SMMStoreWrite (
  IN        EFI_LBA                              Lba,
  IN        UINTN                                Offset,
  IN        UINTN                                *NumBytes,
  IN        UINT8                                *Buffer
  );


/**
  Erase a block using the SMMStore

  @param Lba    The logical block index to erase.

**/
EFI_STATUS
SMMStoreEraseBlock (
  IN         EFI_LBA                              Lba
  );


/**
  Notify the SMMStore Library about a VirtualNotify

**/

VOID
EFIAPI
SMMStoreVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  );

/**
  Initializes SMMStore support

  @param[in] Ptr                  A runtime buffer where arguments are stored
                                  for SMM communication
  @param[in] SmmStoreInfoHob      A runtime buffer with a copy of the
                                  SmmStore Info Hob

  @retval EFI_WRITE_PROTECTED   The SMMSTORE is not present.
  @retval EFI_SUCCESS           The SMMSTORE is supported.

**/

typedef struct {
  UINT64    ComBuffer;
  UINT32    ComBufferSize;
  UINT32    NumBlocks;
  UINT32    BlockSize;
  UINT64    MmioAddress;
  UINT8     ApmCmd;
  UINT8     Reserved0[3];
} SMMSTORE_INFO;

EFI_STATUS
SMMStoreInitialize (
    IN         VOID                      *Ptr,
    IN         SMMSTORE_INFO             *SmmStoreInfoHob
  );


#endif /* __SPI_H__ */
