#include <Include/PiDxe.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PciLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiRuntimeLib.h>
#include "GenericSPI.h"
#include "SPIFlashInternal.h"
#include "Winbond.h"

#define LPC_DEV		0x14
#define LPC_FUNC	0x03

#define SPIROM_BASE_ADDRESS_REGISTER 0xa0
#define SPI_BASE_ALIGNMENT 0x00000040
#define ALIGN_DOWN(x,a) ((x) & ~((typeof(x))(a)-1UL))

#define BLOCK_SIZE 0x10000
#define ADDRESS(Lba, Offset) (UINT32)(((BLOCK_SIZE) * (Lba)) + (Offset) + 0x120000)

STATIC struct spi_flash *flash = NULL;
STATIC VOID *spi_base = NULL;
STATIC VOID *fch_pci_addr = (VOID *)PCI_LIB_ADDRESS(0, LPC_DEV, LPC_FUNC, SPIROM_BASE_ADDRESS_REGISTER);

STATIC UINTN lpc_get_spibase(VOID)
{
	UINT32 base;
	base = PciRead32((UINTN)fch_pci_addr);
	base = ALIGN_DOWN(base, SPI_BASE_ALIGNMENT);
	return (UINTN)base;
}

STATIC VOID spi_set_base(UINTN base)
{
	spi_base = (VOID *)base;
}

UINTN spi_get_bar(VOID)
{
	if (spi_base == 0) {
		spi_set_base(lpc_get_spibase());
	}
	return (UINTN)spi_base;
}

EFI_STATUS
AmdSpiRead (
  IN        EFI_LBA                              Lba,
  IN        UINTN                                Offset,
  IN        UINTN                                *NumBytes,
  IN        UINT8                                *Buffer
  )
{
  UINT32 address = ADDRESS(Lba, Offset);

  if (*NumBytes == 0 || Buffer == NULL)
    return EFI_INVALID_PARAMETER;

  return spi_flash_read(flash, address, *NumBytes, Buffer);
}

EFI_STATUS
AmdSpiWrite (
  IN        EFI_LBA                              Lba,
  IN        UINTN                                Offset,
  IN        UINTN                                *NumBytes,
  IN        UINT8                                *Buffer
  )
{
  UINT32 address = ADDRESS(Lba, Offset);

  if (*NumBytes == 0 || Buffer == NULL)
    return EFI_INVALID_PARAMETER;

  return spi_flash_write(flash, address, *NumBytes, Buffer);
}

/**
  Erase a block using the AMD SPI

  @param Lba    The logical block index to erase.

**/
EFI_STATUS
AmdSpiEraseBlock (
  IN    EFI_LBA     Lba
  )
{
  UINT32 address = ADDRESS(Lba, 0);

  return spi_flash_erase(flash, address, BLOCK_SIZE);
}

EFI_STATUS
AmdSpiInitialize (VOID)
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));

  spi_base = (VOID *)spi_get_bar();

  flash = AllocateRuntimeZeroPool (sizeof(struct spi_flash));
  if (!flash) {
    DEBUG((EFI_D_ERROR, "%a: Out of resources\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  flash->ops = AllocateRuntimeZeroPool (sizeof(struct spi_flash_ops));
  if (!flash->ops) {
    DEBUG((EFI_D_ERROR, "%a: Out of resources\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  flash->spi.ctrlr = AllocateRuntimeZeroPool (sizeof(struct spi_ctrlr));
  if (!flash->spi.ctrlr) {
    DEBUG((EFI_D_ERROR, "%a: Out of resources\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  PcdSet32S (PcdFchSpiBar, (UINT32)(UINTN)spi_base);

  return spi_flash_probe(0, 0, flash);
}

VOID
EFIAPI
AmdSpiVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  EfiConvertPointer (0x0, &spi_base);
  EfiConvertPointer (0x0, &fch_pci_addr);
  EfiConvertPointer (0x0, (VOID **)&flash);
  EfiConvertPointer (0x0, (VOID **)&flash->spi.ctrlr);
  EfiConvertPointer (0x0, (VOID **)&flash->spi.ctrlr->setup);
  EfiConvertPointer (0x0, (VOID **)&flash->spi.ctrlr->xfer);
  EfiConvertPointer (0x0, (VOID **)&flash->spi.ctrlr->xfer_vector);
  EfiConvertPointer (0x0, (VOID **)&flash->spi.ctrlr->xfer_dual);
  EfiConvertPointer (0x0, (VOID **)&flash->spi.ctrlr->flash_probe);
  EfiConvertPointer (0x0, (VOID **)&flash->ops);
  EfiConvertPointer (0x0, (VOID **)&flash->ops->read);
  EfiConvertPointer (0x0, (VOID **)&flash->ops->write);
  EfiConvertPointer (0x0, (VOID **)&flash->ops->erase);
  EfiConvertPointer (0x0, (VOID **)&flash->ops->status);
  EfiConvertPointer (0x0, (VOID **)&flash->part);
}
