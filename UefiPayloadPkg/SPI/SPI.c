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
    NULL, // GetAttributes
    NULL, // SetAttributes
    NULL,  // GetPhysicalAddress
    NULL,  // GetBlockSize
    NULL,  // Read
    NULL, // Write
    NULL, // EraseBlocks
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
  if(EFI_ERROR (Status)) {
    DEBUG((
      EFI_D_INFO, "%a Error during protocol installation\n", __FUNCTION__));
  } else {
    DEBUG((
      EFI_D_INFO, "%a Successfull protocol installation\n", __FUNCTION__));
  }
  return EFI_SUCCESS;
}
