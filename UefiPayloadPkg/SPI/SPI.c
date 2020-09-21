/** @file  BlSMMStoreDxe.c

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include "SPI.h"

EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol = {
    FvbGetAttributes, // FvbGetAttributes, // GetAttributes
    FvbGetAttributes, // FvbSetAttributes, // SetAttributes
    FvbGetAttributes, // FvbGetPhysicalAddress,  // GetPhysicalAddress
    FvbGetAttributes, // FvbGetBlockSize,  // GetBlockSize
    FvbGetAttributes, // FvbRead,  // Read
    FvbGetAttributes, // FvbWrite, // Write
    FvbGetAttributes, // FvbEraseBlocks, // EraseBlocks
    NULL, //ParentHandle
  };

EFI_STATUS
EFIAPI
FvbGetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL    *This,
  OUT       EFI_FVB_ATTRIBUTES_2                   *Attributes
  )
{
  EFI_FVB_ATTRIBUTES_2  FlashFvbAttributes;
  SMMSTORE_INSTANCE *Instance;

  Instance = INSTANCE_FROM_FVB_THIS(This);

  FlashFvbAttributes = (EFI_FVB_ATTRIBUTES_2) (

      EFI_FVB2_READ_ENABLED_CAP | // Reads may be enabled
      EFI_FVB2_READ_STATUS      | // Reads are currently enabled
      EFI_FVB2_STICKY_WRITE     | // A block erase is required to flip bits into EFI_FVB2_ERASE_POLARITY
      EFI_FVB2_MEMORY_MAPPED    | // It is memory mapped
      EFI_FVB2_ERASE_POLARITY     // After erasure all bits take this value (i.e. '1')

      );

  // Check if it is write protected
  if (Instance->Media.ReadOnly != TRUE) {

    FlashFvbAttributes = FlashFvbAttributes         |
                         EFI_FVB2_WRITE_STATUS      | // Writes are currently enabled
                         EFI_FVB2_WRITE_ENABLED_CAP;  // Writes may be enabled
  }

  *Attributes = FlashFvbAttributes;

  DEBUG ((DEBUG_BLKIO, "FvbGetAttributes(0x%X)\n", *Attributes));

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SPIInitialize (
  IN EFI_HANDLE                        ImageHandle,
  IN EFI_SYSTEM_TABLE                  *SystemTable
  )
{
  EFI_STATUS Status;
  Status = gBS->InstallProtocolInterface(
    ImageHandle,
    &gEfiDevicePathProtocolGuid,
    EFI_NATIVE_INTERFACE,
    &FvbProtocol);
  DEBUG((EFI_D_INFO, "SPI\n"));
  if(EFI_ERROR (Status)) {
    DEBUG((
      EFI_D_INFO, "%a Error during protocol installation\n", __FUNCTION__));
  }
  return EFI_SUCCESS;
}
