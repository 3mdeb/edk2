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
#include "BlSMMStoreDxe.h"
#include "Fvb.h"

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

EFI_STATUS EFIAPI SPIInitialize (
  IN EFI_HANDLE                        ImageHandle,
  IN EFI_SYSTEM_TABLE                  *SystemTable
  )
{
  EFI_STATUS Status;
  struct spi_slave slave;
  DEBUG((EFI_D_INFO, "SPI IS HERE 2\n"));

  DEBUG((EFI_D_INFO, "sizeof(unsigned int) 0x%X\n", sizeof(unsigned int)));
  DEBUG((EFI_D_INFO, "sizeof(void *) 0x%X\n", sizeof(VOID *)));
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
  DEBUG((EFI_D_INFO, "calling spi_init()\n"));
  spi_init();
  DEBUG((EFI_D_INFO, "spi_init() was called\n"));
  DEBUG((EFI_D_INFO, "spi_setup_slave() returned 0x%X\n", spi_setup_slave(0, 0, &slave)));
  DEBUG((EFI_D_INFO, "0x%X\n", slave.ctrlr->xfer));
  /*-----------------------------------------------------------------------
 * SPI transfer
 *
 * spi_xfer() interface:
 *   slave:	The SPI slave which will be sending/receiving the data.
 *   dout:	Pointer to a string of bytes to send out.
 *   bytesout:	How many bytes to write.
 *   din:	Pointer to a string of bytes that will be filled in.
 *   bytesin:	How many bytes to read.
 *
 * Note that din and dout are transferred simultaneously in a full duplex
 * transaction. The number of clocks within one transaction is calculated
 * as: MAX(bytesout*8, bytesin*8).
 *
 *   Returns: 0 on success, not 0 on failure
 */
  struct spi_op vector = {
    .dout = "asdf",
    .bytesout = 4,
  };
  char fill[255];
  DEBUG((EFI_D_INFO, "spi_xfer() returned 0x%X\n", spi_xfer(&slave, "asdf", 4, &fill, 0)));
  DEBUG((EFI_D_INFO, "spi_xfer_vectors() returned 0x%X\n", spi_xfer_vector(&slave, &vector, 1)));
  return EFI_SUCCESS;
}
