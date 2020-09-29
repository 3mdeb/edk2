#ifndef PCI_OPS_H
#define PCI_OPS_H

#include <Include/PiDxe.h>
#include "device.h"

UINT32 pci_s_read_config32(pci_devfn_t dev, UINT16 reg);
pci_devfn_t pcidev_bdf(const struct device *dev);
VOID pcidev_die(VOID);
pci_devfn_t pcidev_assert(CONST struct device *dev);
UINT32 pci_read_config32(CONST struct device *dev, UINT16 reg);

#endif /* PCI_OPS_H */
