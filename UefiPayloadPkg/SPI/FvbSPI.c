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
  // IN SMMSTORE_INFO             *FvbSPIInfoHob
) { EFI_STATUS                              Status;
  VOID                                    *ComBuf;
  VOID                                    *GuidHob;
  SMMSTORE_INFO                           *SMMStoreInfoHob;
  EFI_GCD_MEMORY_SPACE_DESCRIPTOR         GcdDescriptor;

  if (PcdGetBool (PcdEmuVariableNvModeEnable)) {
    DEBUG ((DEBUG_WARN, "Variable emulation is active! Skipping driver init.\n"));
    while(1);
    return EFI_SUCCESS;
  }

  //
  // Find the SMMSTORE information guid hob
  //
  GuidHob = GetFirstGuidHob (&gEfiSMMSTOREInfoHobGuid);
  // GuidHob = NULL;
  if (GuidHob == NULL) {
    DEBUG ((DEBUG_WARN, "SMMSTORE not supported! Skipping driver init.\n"));
    PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
    while(1);
    return EFI_SUCCESS;
  }

  //
  // Allocate Communication Buffer for arguments to pass to SMM
  //
  ComBuf = AllocateRuntimePool (SMMSTORE_COMBUF_SIZE);
  if (!ComBuf) {
    PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
    while(1);
    return EFI_OUT_OF_RESOURCES;
  }

  //
  // Place SMMSTORE information hob in a runtime buffer
  //
  SMMStoreInfoHob = AllocateRuntimePool (GET_GUID_HOB_DATA_SIZE(GuidHob));
  if (!SMMStoreInfoHob) {
    FreePool(ComBuf);
    PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
    while(1);
    return EFI_OUT_OF_RESOURCES;
  }

  CopyMem(SMMStoreInfoHob, GET_GUID_HOB_DATA (GuidHob), GET_GUID_HOB_DATA_SIZE(GuidHob));

  if (!SMMStoreInfoHob->MmioAddress ||
      !SMMStoreInfoHob->ComBuffer ||
      !SMMStoreInfoHob->BlockSize ||
      !SMMStoreInfoHob->NumBlocks) {
    DEBUG((EFI_D_ERROR, "%a: Invalid data in SMMStore Info hob\n", __FUNCTION__));
    FreePool(ComBuf);
    FreePool(SMMStoreInfoHob);
    while(1);
    return EFI_WRITE_PROTECTED;
  }

  Status = SMMStoreInitialize(ComBuf, SMMStoreInfoHob);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR,"%a: Failed to initialize SMMStore\n",
      __FUNCTION__));
    PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
    FreePool(ComBuf);
    FreePool(SMMStoreInfoHob);
    while(1);
    return Status;
  }

  // // Update PCDs for Variable/RuntimeDxe
  // PcdSet32S (PcdFlashNvStorageVariableBase,
  //     PcdGet32 (PcdFlashNvStorageVariableBase) + SMMStoreInfoHob->MmioAddress);
  // PcdSet32S (PcdFlashNvStorageFtwWorkingBase,
  //     PcdGet32 (PcdFlashNvStorageFtwWorkingBase) + SMMStoreInfoHob->MmioAddress);
  // PcdSet32S (PcdFlashNvStorageFtwSpareBase,
  //     PcdGet32 (PcdFlashNvStorageFtwSpareBase) + SMMStoreInfoHob->MmioAddress);

  // mSMMStoreInstance = AllocateRuntimePool (sizeof(SMMSTORE_INSTANCE*));
  // if (!mSMMStoreInstance) {
  //   DEBUG((EFI_D_ERROR, "%a: Out of resources\n", __FUNCTION__));
  //   FreePool(ComBuf);
  //   FreePool(SMMStoreInfoHob);
  //   PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
  //   return EFI_OUT_OF_RESOURCES;
  // }

  // Status = SMMStoreCreateInstance (
  //   SMMStoreInfoHob->NumBlocks,
  //   SMMStoreInfoHob->BlockSize,
  //   &mSMMStoreInstance
  // );
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "%a: Fail to create instance for SMMStore\n",
      __FUNCTION__));
    PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
    FreePool(ComBuf);
    FreePool(SMMStoreInfoHob);
    while(1);
    return Status;
  }

  //
  // Register for the virtual address change event
  //
  // Status = gBS->CreateEventEx (
  //                 EVT_NOTIFY_SIGNAL,
  //                 TPL_NOTIFY,
  //                 BlSMMStoreVirtualNotifyEvent,
  //                 NULL,
  //                 &gEfiEventVirtualAddressChangeGuid,
  //                 &mSMMStoreVirtualAddrChangeEvent
  //                 );
  ASSERT_EFI_ERROR (Status);

  //
  // Finally mark the SMM communication buffer provided by CB or SBL as runtime memory
  //
  Status      = gDS->GetMemorySpaceDescriptor (SMMStoreInfoHob->ComBuffer, &GcdDescriptor);
  if (EFI_ERROR (Status) || GcdDescriptor.GcdMemoryType != EfiGcdMemoryTypeReserved) {
    DEBUG((EFI_D_INFO, "%a: No memory space descriptor for com buffer found\n",
      __FUNCTION__));

    //
    // Add a new entry if not covered by existing mapping
    //
    Status = gDS->AddMemorySpace (
        EfiGcdMemoryTypeReserved,
        SMMStoreInfoHob->ComBuffer, SMMStoreInfoHob->ComBufferSize,
        EFI_MEMORY_WB | EFI_MEMORY_RUNTIME
        );
    ASSERT_EFI_ERROR (Status);
  }

  //
  // Mark as runtime service
  //
  Status = gDS->SetMemorySpaceAttributes (
                  SMMStoreInfoHob->ComBuffer,
                  SMMStoreInfoHob->ComBufferSize,
                  EFI_MEMORY_RUNTIME
                  );
  ASSERT_EFI_ERROR (Status);

  if (!SMMStoreInfoHob->MmioAddress){
    while(1);
    return Status;
  }

  //
  // Mark the memory mapped store as MMIO memory
  //
  Status      = gDS->GetMemorySpaceDescriptor (SMMStoreInfoHob->MmioAddress, &GcdDescriptor);
  if (EFI_ERROR (Status) || GcdDescriptor.GcdMemoryType != EfiGcdMemoryTypeMemoryMappedIo) {
    DEBUG((EFI_D_INFO, "%a: No memory space descriptor for com buffer found\n",
      __FUNCTION__));

    //
    // Add a new entry if not covered by existing mapping
    //
    Status = gDS->AddMemorySpace (
        EfiGcdMemoryTypeMemoryMappedIo,
        SMMStoreInfoHob->MmioAddress,
        SMMStoreInfoHob->NumBlocks * SMMStoreInfoHob->BlockSize,
        EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
        );
    ASSERT_EFI_ERROR (Status);
  }

  //
  // Mark as runtime service
  //
  Status = gDS->SetMemorySpaceAttributes (
                  SMMStoreInfoHob->MmioAddress,
                  SMMStoreInfoHob->NumBlocks * SMMStoreInfoHob->BlockSize,
                  EFI_MEMORY_RUNTIME
                  );
  ASSERT_EFI_ERROR (Status);

  while(1);
  return Status;
  ///////////////////////////////////////////////////////////////////////////////
  // DEBUG((DEBUG_WARN, "SPI IS HERE.\n"));
  // SMMSTORE_INFO *FvbSPIInfoHob;
  // SMMSTORE_INFO *NewFvbSPIInfoHob;
  // VOID *GuidHob;
  // if (PcdGetBool(PcdEmuVariableNvModeEnable)) {
  //   DEBUG ((
  //     DEBUG_WARN,
  //     "Variable emulation is active! Skipping driver init.\n"));
  //   while(1);
  //   return EFI_SUCCESS;
  // }

  // //
  // // Create guid hob for SMMSTORE
  // //
  // // Status = ParseSMMSTOREInfo (&SMMSTOREInfo);
  // // if (!EFI_ERROR (Status)) {
  // //   NewSMMSTOREInfo = BuildGuidHob (&gEfiSMMSTOREInfoHobGuid, sizeof (SMMSTOREInfo));
  // //   ASSERT (NewSMMSTOREInfo != NULL);
  // //   CopyMem (NewSMMSTOREInfo, &SMMSTOREInfo, sizeof (SMMSTOREInfo));
  // //   DEBUG ((DEBUG_INFO, "Created SMMSTORE info hob\n"));
  // // }

  // FVBSPI_INFO fvbSPIInfo = {
  //   .ComBuffer = 0,
  //   .ComBufferSize = 0,
  //   .NumBlocks = 0,
  //   .BlockSize = 0,
  //   .MmioAddress = 0,
  //   .ApmCmd = 0
  // };
  // DEBUG((DEBUG_WARN, "&gUefiFvbSPIInfoGuid = 0x%X, sizeof(FVBSPI_INFO) = 0x%X\n", &gUefiFvbSPIInfoGuid, sizeof(FVBSPI_INFO)));
  // NewFvbSPIInfoHob = BuildGuidHob(&gUefiFvbSPIInfoGuid, sizeof(FVBSPI_INFO));
  // CopyMem(NewFvbSPIInfoHob, &fvbSPIInfo, sizeof(FVBSPI_INFO));
  // DEBUG((DEBUG_INFO, "Created hob\n"));

  // FvbSPIInfoHob = GetFirstGuidHob(&gUefiFvbSPIInfoGuid);
  // DEBUG((DEBUG_WARN, "FvbSPIInfoHob = 0x%X\n", FvbSPIInfoHob));
  // if (GuidHob == NULL) {
  //   DEBUG ((DEBUG_WARN, "FvbSPI not supported! Skipping driver init.\n"));
  //   PcdSetBoolS (PcdEmuVariableNvModeEnable, TRUE);
  //   while(1);
  //   return EFI_SUCCESS;
  // }

  // //
  // // Place FvbSPI information hob in a runtime buffer
  // //
  // FvbSPIInfoHob = AllocateRuntimePool(GET_GUID_HOB_DATA_SIZE(GuidHob));
  // if (!FvbSPIInfoHob) {
  //   DEBUG((
  //     EFI_D_INFO,
  //     "%a Memory allocation failed.\n", __FUNCTION__));
  //   PcdSetBoolS(PcdEmuVariableNvModeEnable, TRUE);
  //   while(1);
  //   return EFI_OUT_OF_RESOURCES;
  // }
  // EFI_STATUS Status;
  // Status = gBS->InstallMultipleProtocolInterfaces (
  //   &Handle,
  //   &gEfiFirmwareVolumeBlockProtocolGuid, &FvbProtocol,
  //   NULL);
  // if(EFI_ERROR (Status)) {
  //   DEBUG((
  //     EFI_D_INFO,
  //     "%a Error during protocol installation.\n", __FUNCTION__));
  //   FreePool(FvbSPIInfoHob);
  //   PcdSetBoolS(PcdEmuVariableNvModeEnable, TRUE);
  //   while(1);
  //   return EFI_PROTOCOL_ERROR;
  // }
  // // Update PCDs for Variable/RuntimeDxe
  // // PcdSet32S (PcdFlashNvStorageVariableBase,
  // //     PcdGet32 (PcdFlashNvStorageVariableBase) + FvbSPIInfoHob->MmioAddress);
  // // PcdSet32S (PcdFlashNvStorageFtwWorkingBase,
  // //     PcdGet32 (PcdFlashNvStorageFtwWorkingBase) + FvbSPIInfoHob->MmioAddress);
  // // PcdSet32S (PcdFlashNvStorageFtwSpareBase,
  // //     PcdGet32 (PcdFlashNvStorageFtwSpareBase) + FvbSPIInfoHob->MmioAddress);
  // spi_init();
  // while(1);
  // return EFI_SUCCESS;
}
