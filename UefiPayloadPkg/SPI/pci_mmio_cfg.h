#ifndef PCI_MMIO_CFG_H
#define PCI_MMIO_CFG_H

#include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>
#include "pci_type.h"

/* By not assigning this to CONFIG_MMCONF_BASE_ADDRESS here we
 * prevent some sub-optimal constant folding. */
//UINT8 *const pci_mmconf = (VOID *)(unsigned long int)0xBAADF00D; //CONFIG_MMCONF_BASE_ADDRESS;
//extern UINT8 *const pci_mmconf;

/* Using a unique datatype for MMIO writes makes the pointers to _not_
 * qualify for pointer aliasing with any other objects in memory.
 *
 * MMIO offset is a value originally derived from 'struct device *'
 * in ramstage. For the compiler to not discard this MMIO offset value
 * from CPU registers after any MMIO writes, -fstrict-aliasing has to
 * be also set for the build.
 *
 * Bottom 12 bits (4 KiB) are reserved to address the registers of a
 * single PCI function. Declare the bank as a union to avoid some casting
 * in the functions below.
 */
union pci_bank {
	UINT8 reg8[4096];
	UINT16 reg16[4096 / sizeof(UINT16)];
	UINT32 reg32[4096 / sizeof(UINT32)];
};

static __attribute__ ((__always_inline__)) inline
volatile union pci_bank *pcicfg(pci_devfn_t dev)
{
	UINT8 *const spi_bar = (VOID *)(unsigned long int)0xF8000000;
	return (void *)&spi_bar[PCI_DEVFN_OFFSET(dev)];
}

static __attribute__ ((__always_inline__)) inline
UINT32 pci_mmio_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg32[reg / sizeof(UINT32)];
}

static __attribute__ ((__always_inline__)) inline
void pci_mmio_write_config32(pci_devfn_t dev, UINT16 reg, UINT32 value)
{
	pcicfg(dev)->reg32[reg / sizeof(UINT32)] = value;
}

#endif /* PCI_MMIO_CFG_H */
