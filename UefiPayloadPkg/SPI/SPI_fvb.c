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
  unsigned char fill[300] = {};
  UINTN length = 30;
  FvbRead(&FvbProtocol, 0, 0, &length, (VOID *)fill);
  DEBUG((EFI_D_INFO, "TRIALS\n"));
  for(int i = 0; i < 10; i++) {
    DEBUG((EFI_D_INFO, "%X %X\n", i, (UINTN)(UINT8)fill[i]));
  }
  DEBUG((EFI_D_INFO, "ERASING\n"));
  FvbEraseBlocks(&FvbProtocol, 0, 2, 4, 5, EFI_LBA_LIST_TERMINATOR);
  FvbRead(&FvbProtocol, 0, 0, &length, (VOID *)fill);
  for(int i = 0; i < 10; i++) {
    DEBUG((EFI_D_INFO, "%X %X\n", i, (UINTN)(UINT8)fill[i]));
  }
  DEBUG((EFI_D_INFO, "WRITING\n"));
  UINT8 toWrite[] = {01, 0x01, 0x01, 0x77};
  UINTN writeLen = 4;
  EFI_STATUS status = FvbWrite(&FvbProtocol, 0, 1, &writeLen, toWrite);
  DEBUG((EFI_D_INFO, "Written: 0x%X\n", writeLen));
  if(status == EFI_SUCCESS) {
    DEBUG((EFI_D_INFO, "EFI_SUCCESS\n"));
  } else if(status == EFI_BAD_BUFFER_SIZE) {
    DEBUG((EFI_D_INFO, "EFI_BAD_BUFFER_SIZE\n"));
  } else if(status == EFI_ACCESS_DENIED) {
    DEBUG((EFI_D_INFO, "EFI_ACCESS_DENIED\n"));
  } else {
    DEBUG((EFI_D_INFO, "It's really bad\n"));
  }
  unsigned char newFill[300] = {};
  UINTN newLength = 30;
  FvbRead(&FvbProtocol, 0, 0, &newLength, (VOID *)newFill);
  for(int i = 0; i < 10; i++) {
    DEBUG((EFI_D_INFO, "%X %X\n", i, (UINTN)(UINT8)newFill[i]));
  }
  while(TRUE);
  return EFI_SUCCESS;
}
