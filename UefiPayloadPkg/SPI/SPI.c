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
    NULL, // FvbGetAttributes, // GetAttributes
    NULL, // FvbSetAttributes, // SetAttributes
    NULL, // FvbGetPhysicalAddress,  // GetPhysicalAddress
    NULL, // FvbGetBlockSize,  // GetBlockSize
    NULL, // FvbRead,  // Read
    NULL, // FvbWrite, // Write
    NULL, // FvbEraseBlocks, // EraseBlocks
    NULL, //ParentHandle
  };

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
  DEBUG((EFI_D_INFO, "%i\n", Status));
  DEBUG((EFI_D_INFO, "%i\n", EFI_ERROR (Status)));
  return EFI_SUCCESS;
}
