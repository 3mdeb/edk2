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
#include "SPIgeneric.h"
#include "SPI.h"

EFI_HANDLE Handle = NULL;
EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol = {
    FvbGetAttributes, // GetAttributes
    FvbSetAttributes, // SetAttributes
    FvbGetPhysicalAddress,  // GetPhysicalAddress
    FvbGetBlockSize,  // GetBlockSize
    FvbRead,  // Read
    FvbWrite, // Write
    FvbEraseBlocks, // EraseBlocks
    NULL, //ParentHandle
  };

void * memset (void *dest, int ch, __SIZE_TYPE__ count)
{
  for(__SIZE_TYPE__ offset = 0; offset < count; ++offset) {
    ((CHAR8 *)dest)[offset] = ch;
  }
  return dest;
}

EFI_STATUS EFIAPI SPIInitialize (
  IN EFI_HANDLE                        ImageHandle,
  IN EFI_SYSTEM_TABLE                  *SystemTable
  )
{
  EFI_STATUS Status;
  struct spi_slave slave;
  Status = gBS->InstallProtocolInterface(
    ImageHandle,
    &gEfiDevicePathProtocolGuid,
    EFI_NATIVE_INTERFACE,
    &FvbProtocol);
  if(EFI_ERROR (Status)) {
    DEBUG((EFI_D_INFO, "%a Error during single protocol installation\n", __FUNCTION__));
  } else {
    DEBUG((EFI_D_INFO, "%a Successfull signgle protocol installation\n", __FUNCTION__));
  }
  DEBUG((EFI_D_INFO, "SPI IS HERE\n"));
  DEBUG((EFI_D_INFO, "sizeof(unsigned int) 0x%X\n", sizeof(unsigned int)));
  DEBUG((EFI_D_INFO, "sizeof(void *) 0x%X\n", sizeof(void *)));
  DEBUG((EFI_D_INFO, "spi_setup_slave() returned %i", spi_setup_slave(0, 0, &slave)));
  Status = gBS->InstallMultipleProtocolInterfaces (
              &Handle,
              &gEfiFirmwareVolumeBlockProtocolGuid, &FvbProtocol,
              NULL
              );
  if(EFI_ERROR (Status)) {
    DEBUG((EFI_D_INFO, "%a Error during protocol installation\n", __FUNCTION__));
  } else {
    DEBUG((EFI_D_INFO, "%a Successfull protocol installation\n", __FUNCTION__));
  }
  return EFI_SUCCESS;
}
