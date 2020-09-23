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

UINT32 spi_setup_slave(UINT32 bus, UINT32 cs, struct spi_slave *slave)
{
	__SIZE_TYPE__ i;

	memset(slave, 0, sizeof(*slave));

	for (i = 0; i < spi_ctrlr_bus_map_count; i++) {
		if ((spi_ctrlr_bus_map[i].bus_start <= bus) &&
		    (spi_ctrlr_bus_map[i].bus_end >= bus)) {
			slave->ctrlr = spi_ctrlr_bus_map[i].ctrlr;
			break;
		}
	}

	if (slave->ctrlr == NULL)
		return -1;

	slave->bus = bus;
	slave->cs = cs;

	if (slave->ctrlr->setup)
		return slave->ctrlr->setup(slave);

	return 0;
}

EFI_STATUS EFIAPI SPIInitialize (
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
  if(EFI_ERROR (Status)) {
    DEBUG((EFI_D_INFO, "%a Error during single protocol installation\n", __FUNCTION__));
  } else {
    DEBUG((EFI_D_INFO, "%a Successfull signgle protocol installation\n", __FUNCTION__));
  }
  DEBUG((EFI_D_INFO, "SPI IS HERE\n"));
  DEBUG((EFI_D_INFO, "sizeof(unsigned int) 0x%X\n", sizeof(unsigned int)));
  DEBUG((EFI_D_INFO, "sizeof(void *) 0x%X\n", sizeof(void *)));
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
