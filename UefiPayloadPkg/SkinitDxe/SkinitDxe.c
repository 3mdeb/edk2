/** @file

  Copyright (c) 2022, 3mdeb Sp. z o.o.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Library/PciLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>

/* TODO: move this to external function, compilable by tools other than GCC */
static void SetGIF(void)
{
  asm volatile ("stgi");
}

EFI_STATUS
EFIAPI
Entry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  DEBUG ((DEBUG_INFO, "SKINIT Driver entry\n"));

  /* TODO: check if AMD and SKINIT available before reading MSR */
  UINT64 VmCr = AsmReadMsr64(0xc0010114);

  if (VmCr & (1 << 1)) {
    DEBUG ((DEBUG_INFO, "SKINIT detected\n"));
    SetGIF();
    /* TODO: install protocol/set PCD for later */
    return EFI_SUCCESS;
  }

  DEBUG ((DEBUG_INFO, "SKINIT not detected\n"));
  return EFI_UNSUPPORTED;
}
