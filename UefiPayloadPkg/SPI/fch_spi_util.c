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

static unsigned long int spi_base = 0;

VOID spi_set_base(VOID *base)
{
	DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
	spi_base = (unsigned long int)base;
}

//static //__attribute__ ((__always_inline__))
union pci_bank *pcicfg(pci_devfn_t dev)
{
	UINT8 *pci_mmconf =(void *)((unsigned long int)(0xF8000000));
	DEBUG((EFI_D_INFO, "%a pci_mmconf = 0x%X, dev = 0x%X, PCI_DEVFN_OFFSET(dev) = 0x%X\n", __FUNCTION__, pci_mmconf, dev, PCI_DEVFN_OFFSET(dev)));
	DEBUG((EFI_D_INFO, "%a returned address = 0x%X\n", __FUNCTION__, (void *)&pci_mmconf[PCI_DEVFN_OFFSET(dev)]));
	return (void *)&pci_mmconf[PCI_DEVFN_OFFSET(dev)];
}

unsigned long int spi_get_bar(VOID)
{
	if (spi_base == 0)
		DEBUG((EFI_D_INFO, "%a calling lpc_get_spibase()\n", __FUNCTION__));
	 	spi_set_base((VOID *)lpc_get_spibase());
	DEBUG((EFI_D_INFO, "%a spi_base = 0x%X\n", __FUNCTION__, spi_base));
	return spi_base;
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
