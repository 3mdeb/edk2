/** @file  BlSMMStoreDxe.c

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Include/Library/DebugLib.h>
#include <Include/Library/SMMStoreLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/HobLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include "FchSPICtrl.h"
#include "Fvb.h"
#include "GenericSPI.h"
#include "SPIFlashInternal.h"

EFI_HANDLE Handle = NULL;
EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol = {
  FvbGetAttributes, // GetAttributes
  FvbSetAttributes, // SetAttributes
  FvbGetPhysicalAddress,  // GetPhysicalAddress
  FvbGetBlockSize,  // GetBlockSize
  FvbRead,          // Read
  FvbWrite,         // Write
  FvbEraseBlocks,   // EraseBlocks
  NULL
};

EFI_STATUS checkBusyBit(UINT8 *Busy);

EFI_STATUS EFIAPI SPIInitialize (
  IN EFI_HANDLE                        ImageHandle,
  IN EFI_SYSTEM_TABLE                  *SystemTable
) {
    FVBSPI_INFO fvbSPIInfo = {
    .ComBuffer = 0,
    .ComBufferSize = 0,
    .NumBlocks = 4,
    .BlockSize = BLOCK_SIZE,
    .MmioAddress = 0,
    .ApmCmd = 0
  };
  if (PcdGetBool(PcdEmuVariableNvModeEnable)) {
    DEBUG ((
      DEBUG_WARN,
      "Variable emulation is active! Skipping driver init.\n"));
    while(1);
    return EFI_SUCCESS;
  }
  EFI_STATUS Status;
  // Update PCDs for Variable/RuntimeDxe
  PcdSet32S(PcdFlashNvStorageVariableBase,
      PcdGet32(PcdFlashNvStorageVariableBase) + fvbSPIInfo.MmioAddress);
  PcdSet32S(PcdFlashNvStorageFtwWorkingBase,
     PcdGet32(PcdFlashNvStorageFtwWorkingBase) + fvbSPIInfo.MmioAddress);
  PcdSet32S(PcdFlashNvStorageFtwSpareBase,
      PcdGet32(PcdFlashNvStorageFtwSpareBase) + fvbSPIInfo.MmioAddress);
  spi_init();

  Status = InitializeFvAndVariableStoreHeaders(&fvbSPIInfo);
  if(Status != EFI_SUCCESS) {
    DEBUG((DEBUG_WARN, "%a InitializeFvAndVariableStoreHeaders() failed\n", __FUNCTION__));
  }
  Status = ValidateFvHeader(&fvbSPIInfo);
  if(Status != EFI_SUCCESS) {
    DEBUG((DEBUG_WARN, "%a ValidateFvHeader() failed\n", __FUNCTION__));
  }
  //
  // Install the protocol
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
    &Handle,
    &gEfiFirmwareVolumeBlockProtocolGuid, &FvbProtocol,
    NULL);
  if(EFI_ERROR (Status)) {
    DEBUG((
      EFI_D_INFO,
      "%a Error during protocol installation.\n", __FUNCTION__));
    PcdSetBoolS(PcdEmuVariableNvModeEnable, TRUE);
    while(1);
    return EFI_PROTOCOL_ERROR;
  }
  return EFI_SUCCESS;
}
