/* SPDX-License-Identifier: GPL-2.0-only */
#include <Include/Library/DebugLib.h>
#include <Include/Library/PciLib.h>
#include <Include/PiDxe.h>
#include <Library/IoLib.h>
#include "FchSPICtrl.h"
#include "FchSPIUtil.h"
#include "SPI.h"

#define _LPCB_DEV PCI_DEV(0, 0x14, 0x3)
#define SPIROM_BASE_ADDRESS_REGISTER 0xa0
#define SPI_BASE_ALIGNMENT 0x00000040
#define ALIGN_DOWN(x,a) ((x) & ~((typeof(x))(a)-1UL))

static UINTN spi_base = 0;

UINTN lpc_get_spibase(VOID)
{
	UINT32 base;
	base = PciRead32(
		PCI_LIB_ADDRESS(PCU_BUS, PCU_DEV, LPC_FUNC, SPIROM_BASE_ADDRESS_REGISTER));
	base = ALIGN_DOWN(base, SPI_BASE_ALIGNMENT);
	return (unsigned long int)base;
}

VOID spi_set_base(UINTN base)
{
	spi_base = base;
}

//static //__attribute__ ((__always_inline__))
union pci_bank *pcicfg(pci_devfn_t dev)
{
	// FIXME this should not be a magic constant
	UINT8 *pci_mmconf =(void *)((UINTN)(0xF8000000));
	return (void *)&pci_mmconf[PCI_DEVFN_OFFSET(dev)];
}

UINTN spi_get_bar(VOID)
{
	if (spi_base == 0) {
		spi_set_base(lpc_get_spibase());
	}
	return spi_base;
}

UINT8 spi_read8(UINT8 reg)
{
	return MmioRead8((spi_get_bar() + reg));
}

UINT16 spi_read16(UINT8 reg)
{
	return MmioRead16((spi_get_bar() + reg));
}

UINT32 spi_read32(UINT8 reg)
{
	return MmioRead32((spi_get_bar() + reg));
}

VOID spi_write8(UINT8 reg, UINT8 val)
{
	MmioWrite8((spi_get_bar() + reg), val);
}

VOID spi_write16(UINT8 reg, UINT16 val)
{
	MmioWrite16((spi_get_bar() + reg), val);
}

VOID spi_write32(UINT8 reg, UINT32 val)
{
	MmioWrite32((spi_get_bar() + reg), val);
}
