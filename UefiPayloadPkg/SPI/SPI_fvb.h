/** @file  BlSMMStoreDxe.h

  Copyright (c) 2020, 9elements Agency GmbH<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#ifndef __SPI_H__
#define __SPI_H__


#include <Base.h>
#include <PiDxe.h>

#include <Guid/EventGroup.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DiskIo.h>
#include <Protocol/FirmwareVolumeBlock.h>

#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeLib.h>

#include "SPIgeneric.h"

#define SMMSTORE_SIGNATURE                       SIGNATURE_32('S', 'M', 'M', 'S')
#define INSTANCE_FROM_FVB_THIS(a)                CR(a, SMMSTORE_INSTANCE, FvbProtocol, SMMSTORE_SIGNATURE)

typedef struct _SMMSTORE_INSTANCE                SMMSTORE_INSTANCE;

/*
 * Representation of SPI flash operations:
 * read:	Flash read operation.
 * write:	Flash write operation.
 * erase:	Flash erase operation.
 * status:	Read flash status register.
 */
struct spi_flash_ops {
	int (*read)(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
			VOID *buf);
	int (*write)(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
			CONST VOID *buf);
	int (*erase)(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len);
	int (*status)(CONST struct spi_flash *flash, UINT8 *reg);
};

#pragma pack (1)
typedef struct {
  VENDOR_DEVICE_PATH                  Vendor;
  UINT8                               Index;
  EFI_DEVICE_PATH_PROTOCOL            End;
} NOR_FLASH_DEVICE_PATH;
#pragma pack ()

struct _SMMSTORE_INSTANCE {
  UINT32                              Signature;
  EFI_HANDLE                          Handle;
  EFI_BLOCK_IO_MEDIA                  Media;

  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol;

  NOR_FLASH_DEVICE_PATH               DevicePath;
};

enum optype {
	READ_NO_ADDR = 0,
	WRITE_NO_ADDR = 1,
	READ_WITH_ADDR = 2,
	WRITE_WITH_ADDR = 3
};

struct intel_spi_op {
	UINT8 op;
	enum optype type;
};

struct intel_swseq_spi_config {
	UINT8 opprefixes[2];
	struct intel_spi_op ops[8];
};

struct spi_flash_protection_ops {
	/*
	 * Returns 1 if the whole region is software write protected.
	 * Hardware write protection mechanism aren't accounted.
	 * If the write protection could be changed, due to unlocked status
	 * register for example, 0 should be returned.
	 * Returns 0 on success.
	 */
	int (*get_write)(const struct spi_flash *flash,
				    const struct region *region);
	/*
	 * Enable the status register write protection, if supported on the
	 * requested region, and optionally enable status register lock-down.
	 * Returns 0 if the whole region was software write protected.
	 * Hardware write protection mechanism aren't accounted.
	 * If the status register is locked and the requested configuration
	 * doesn't match the selected one, return an error.
	 * Only a single region is supported !
	 *
	 * @return 0 on success
	 */
	int
	(*set_write)(const struct spi_flash *flash,
				const struct region *region,
				const enum spi_flash_status_reg_lockdown mode);

};

void * memset (void *dest, int ch, __SIZE_TYPE__ count);

//
// BlSMMStoreFvbDxe.c
//

EFI_STATUS
EFIAPI
SMMStoreFvbInitialize (
  IN SMMSTORE_INSTANCE*                            Instance
  );

EFI_STATUS
EFIAPI
FvbGetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  OUT       EFI_FVB_ATTRIBUTES_2                    *Attributes
  );

EFI_STATUS
EFIAPI
FvbSetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN OUT    EFI_FVB_ATTRIBUTES_2                    *Attributes
  );

EFI_STATUS
EFIAPI
FvbGetPhysicalAddress(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  OUT       EFI_PHYSICAL_ADDRESS                    *Address
  );

EFI_STATUS
EFIAPI
FvbGetBlockSize(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN        EFI_LBA                                 Lba,
  OUT       UINTN                                   *BlockSize,
  OUT       UINTN                                   *NumberOfBlocks
  );

EFI_STATUS
EFIAPI
FvbRead(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN        EFI_LBA                                 Lba,
  IN        UINTN                                   Offset,
  IN OUT    UINTN                                   *NumBytes,
  IN OUT    UINT8                                   *Buffer
  );

EFI_STATUS
EFIAPI
FvbWrite(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  IN        EFI_LBA                                 Lba,
  IN        UINTN                                   Offset,
  IN OUT    UINTN                                   *NumBytes,
  IN        UINT8                                   *Buffer
  );

EFI_STATUS
EFIAPI
FvbEraseBlocks(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL     *This,
  ...
  );

#endif /* __SPI_H__ */
