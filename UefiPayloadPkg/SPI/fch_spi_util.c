/* SPDX-License-Identifier: GPL-2.0-only */
#include <Include/PiDxe.h>
#include "mmio.h"
#include "fch_spi_util.h"
#include "lpc.h"

#include <Include/Library/DebugLib.h>
#include <Include/PiDxe.h>
#include "fch_spi_ctrl.h"
#include "pci_mmio_cfg.h"
#include "device.h"

static unsigned long int spi_base = 0xF8000000;

union pci_bank {
	UINT8 reg8[4096];
	UINT16 reg16[4096 / sizeof(UINT16)];
	UINT32 reg32[4096 / sizeof(UINT32)];
};

VOID spi_set_base(VOID *base)
{
	DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
	spi_base = (unsigned long int)base;
}

static //__attribute__ ((__always_inline__))
union pci_bank *pcicfg(pci_devfn_t dev)
{
	UINT8 *pci_mmconf = (void *)(unsigned long int)spi_base;
	return (void *)&pci_mmconf[PCI_DEVFN_OFFSET(dev)];
}

static //__attribute__ ((__always_inline__))
UINT32 pci_mmio_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg32[reg / sizeof(UINT32)];
}

static //__attribute__ ((__always_inline__))
UINT32 pci_s_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pci_mmio_read_config32(dev, reg);
}

static //__attribute__ ((__always_inline__))
pci_devfn_t pcidev_bdf(CONST struct device *dev)
{
	return (dev->path.pci.devfn << 12) | (dev->bus->secondary << 20);
}

static //__attribute__ ((__always_inline__))
UINT32 pci_read_config32(CONST struct device *dev, UINT16 reg)
{
	return pci_s_read_config32(pcidev_bdf(dev), reg);
}

unsigned long int spi_get_bar(VOID)
{
	UINT32 base;

	base = pci_read_config32((VOID *)_LPCB_DEV, SPIROM_BASE_ADDRESS_REGISTER);
	base = ALIGN_DOWN(base, SPI_BASE_ALIGNMENT);
	return (unsigned long int)base;
	// DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
	// if (!spi_base)
	// 	spi_set_base((VOID *)lpc_get_spibase());

	// return spi_base;
}

UINT8 spi_read8(UINT8 reg)
{
	return read8((VOID *)(spi_get_bar() + reg));
}

UINT16 spi_read16(UINT8 reg)
{
	return read8((VOID *)(spi_get_bar() + reg));
}

UINT32 spi_read32(UINT8 reg)
{
	return read32((VOID *)(spi_get_bar() + reg));
}

VOID spi_write8(UINT8 reg, UINT8 val)
{
	write8((VOID *)(spi_get_bar() + reg), val);
}

VOID spi_write16(UINT8 reg, UINT16 val)
{
	write16((VOID *)(spi_get_bar() + reg), val);
}

VOID spi_write32(UINT8 reg, UINT32 val)
{
	write32((VOID *)(spi_get_bar() + reg), val);
}
