#ifndef PCI_OPS_H
#define PCI_OPS_H

// #include <Include/PiDxe.h>
// #include "device.h"
// #include "pci_type.h"

#include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>

#include "pci_ops.h"
#include "pci_type.h"
#include "device.h"
#include "pci_mmio_cfg.h"

static __attribute__ ((__always_inline__)) inline
UINT32 pci_s_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pci_mmio_read_config32(dev, reg);
}

static __attribute__ ((__always_inline__)) inline
pci_devfn_t pcidev_bdf(CONST struct device *dev)
{
	return (dev->path.pci.devfn << 12) | (dev->bus->secondary << 20);
}

// __attribute__ ((noreturn))
// VOID pcidev_die(VOID)
// {
// 	//die("PCI: dev is NULL!\n");
//   DEBUG((EFI_D_INFO, "%a: PCI: dev is NULL!", __FUNCTION__));
// 	while(1);
// }

static __attribute__ ((__always_inline__)) inline
pci_devfn_t pcidev_assert(CONST struct device *dev)
{
	if (!dev) {
    DEBUG((EFI_D_INFO, "%a: PCI: dev is NULL!!!", __FUNCTION__));
	  while(1);
  }
	return pcidev_bdf(dev);
}

static __attribute__ ((__always_inline__)) inline
UINT32 pci_read_config32(CONST struct device *dev, UINT16 reg)
{
	return pci_s_read_config32(PCI_BDF(dev), reg);
}

static __attribute__ ((__always_inline__)) inline
VOID pci_s_write_config32(pci_devfn_t dev, UINT16 reg, UINT32 value)
{
	pci_mmio_write_config32(dev, reg, value);
}

static __attribute__ ((__always_inline__)) inline
VOID pci_write_config32(CONST struct device *dev, UINT16 reg, UINT32 val)
{
	pci_s_write_config32(PCI_BDF(dev), reg, val);
}

#endif /* PCI_OPS_H */
