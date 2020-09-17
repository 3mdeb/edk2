/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <PiDxe.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <SPI.h>

EFI_STATUS EFIAPI SPIInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  DEBUG ((DEBUG_WARN, "SPI\n"));
  EFI_STATUS Status = EFI_SUCCESS;
  // EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *FvbProtocol;

  // Status = gBS->LocateProtocol (
  //   &gEfiFirmwareVolumeBlockProtocolGuid,
  //   NULL,
  //   (VOID**)&FvbProtocol);
  DEBUG((EFI_D_ERROR, "%a: Statasus %i\n", __FUNCTION__, Status));
if (EFI_ERROR (Status)) {
    DEBUG((EFI_D_ERROR, "%a: ERRRRORRRR %i\n", __FUNCTION__, Status));
  }


  return EFI_SUCCESS;
}
