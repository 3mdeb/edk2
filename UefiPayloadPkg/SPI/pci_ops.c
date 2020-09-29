#include <Include/PiDxe.h>

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
pci_devfn_t pcidev_bdf(const struct device *dev)
{
	return (dev->path.pci.devfn << 12) | (dev->bus->secondary << 20);
}

__attribute__ ((noreturn))
VOID pcidev_die(VOID)
{
	die("PCI: dev is NULL!\n");
}

static __attribute__ ((__always_inline__)) inline
pci_devfn_t pcidev_assert(CONST struct device *dev)
{
	if (!dev)
		pcidev_die();
	return pcidev_bdf(dev);
}

static __attribute__ ((__always_inline__)) inline
UINT32 pci_read_config32(CONST struct device *dev, UINT16 reg)
{
	return pci_s_read_config32(PCI_BDF(dev), reg);
}
