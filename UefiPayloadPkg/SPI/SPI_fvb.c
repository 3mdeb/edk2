/** @file  BlSMMStoreDxe.c

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

// #include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/FirmwareVolumeBlock.h>
//#include <Guid/SMMSTOREInfoGuid.h>
#include <Library/HobLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/PcdLib.h>
//#include <Library/SMMStoreLib.h>
#include "SPIgeneric.h"
#include "fch_spi_ctrl.h"
#include "Fvb.h"
#include "spi_flash_internal.h"

EFI_HANDLE Handle = NULL;
EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol = {
  FvbGetAttributes, // GetAttributes
  FvbSetAttributes, // SetAttributes
  FvbGetPhysicalAddress,  // GetPhysicalAddress
  FvbGetBlockSize,  // GetBlockSize
  FvbRead, // Read
  FvbWrite,// FvbWrite, // Write
  FvbEraseBlocks, // EraseBlocks
  NULL
};

EFI_STATUS checkBusyBit(UINT8 *Busy);

EFI_STATUS EFIAPI SPIInitialize (
  IN EFI_HANDLE                        ImageHandle,
  IN EFI_SYSTEM_TABLE                  *SystemTable
) {
  EFI_STATUS Status;
  Status = gBS->InstallMultipleProtocolInterfaces (
              &Handle,
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
