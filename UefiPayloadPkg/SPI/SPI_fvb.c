/** @file  BlSMMStoreDxe.c

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Include/Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include "fch_spi_ctrl.h"
#include "Fvb.h"

EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol = {
  NULL,             // GetAttributes
  NULL,             // SetAttributes
  NULL,             // GetPhysicalAddress
  FvbGetBlockSize,  // GetBlockSize
  FvbRead,          // Read
  FvbWrite,         // Write
  FvbEraseBlocks,   // EraseBlocks
  NULL              // ParentHandle
};

EFI_STATUS EFIAPI SPIInitialize(
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
) {
  EFI_STATUS Status;
  Status = gBS->InstallMultipleProtocolInterfaces(
    NULL,
    &gEfiFirmwareVolumeBlockProtocolGuid, &FvbProtocol,
    NULL
  );
  if(EFI_ERROR (Status)) {
    DEBUG((EFI_D_INFO, "%a Error during protocol installation\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  spi_init();
  return EFI_SUCCESS;
}
