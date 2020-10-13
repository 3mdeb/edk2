/** @file  BlSMMStoreDxe.c

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include <Guid/SMMSTOREInfoGuid.h>
#include <Library/HobLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/PcdLib.h>
#include <Library/SMMStoreLib.h>
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

void * memset (void *dest, int ch, __SIZE_TYPE__ count)
{
  for(__SIZE_TYPE__ offset = 0; offset < count; ++offset) {
    ((CHAR8 *)dest)[offset] = ch;
  }
  return dest;
}

EFI_STATUS checkBusyBit(UINT8 *Busy);

EFI_STATUS EFIAPI SPIInitialize (
  IN EFI_HANDLE                        ImageHandle,
  IN EFI_SYSTEM_TABLE                  *SystemTable
  )
{
  EFI_STATUS Status;
  struct spi_slave slave;
  DEBUG((EFI_D_INFO, "SPI IS HERE\n"));

  DEBUG((EFI_D_INFO, "sizeof(unsigned int) 0x%X\n", sizeof(unsigned int)));
  DEBUG((EFI_D_INFO, "sizeof(void *) 0x%X\n", sizeof(VOID *)));
  Status = gBS->InstallMultipleProtocolInterfaces (
              &Handle,
              &gEfiFirmwareVolumeBlockProtocolGuid, &FvbProtocol,
              NULL
              );
  if(EFI_ERROR (Status)) {
    DEBUG((EFI_D_INFO, "%a Error during protocol installation\n", __FUNCTION__));
  }
  spi_init();
  DEBUG((EFI_D_INFO, "spi_setup_slave() returned 0x%X\n", spi_setup_slave(0, 0, &slave)));
  char fill[300] = {};
  UINTN length = 4;
  FvbRead(&FvbProtocol, 0, 0, &length, (VOID *)fill);
  DEBUG((EFI_D_INFO, "TRIALS\n"));
  for(int i = 0; i < 10; i++) {
    DEBUG((EFI_D_INFO, "%X %X\n", i, fill[i]));
  }
  DEBUG((EFI_D_INFO, "ERASING\n"));
  EFI_STATUS result = FvbEraseBlocks(&FvbProtocol, 0, 1, EFI_LBA_LIST_TERMINATOR);
  if(result == EFI_SUCCESS)
    DEBUG((EFI_D_INFO, "EFI_SUCCESS\n"));
  if(result == EFI_ACCESS_DENIED)
    DEBUG((EFI_D_INFO, "EFI_ACCESS_DENIED\n"));
  if(result == EFI_DEVICE_ERROR)
    DEBUG((EFI_D_INFO, "EFI_DEVICE_ERROR\n"));
  if(result == EFI_INVALID_PARAMETER)
    DEBUG((EFI_D_INFO, "EFI_INVALID_PARAMETER\n"));
  UINT8 busy = 0xFF;
  checkBusyBit(&busy);
  DEBUG((EFI_D_INFO, "BUSY: 0x%X\n", busy));
  FvbRead(&FvbProtocol, 0, 0, &length, (VOID *)fill);
  for(int i = 0; i < 10; i++) {
    DEBUG((EFI_D_INFO, "%X %X\n", i, fill[i]));
  }
  return EFI_SUCCESS;
}
