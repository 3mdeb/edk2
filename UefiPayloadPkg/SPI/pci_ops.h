#ifndef PCI_OPS_H
#define PCI_OPS_H

#include <Include/PiDxe.h>
#include "device.h"
#include "pci_type.h"

UINT32 pci_s_read_config32(pci_devfn_t dev, UINT16 reg);
pci_devfn_t pcidev_bdf(const struct device *dev);
VOID pcidev_die(VOID);
pci_devfn_t pcidev_assert(CONST struct device *dev);
UINT32 pci_read_config32(CONST struct device *dev, UINT16 reg);
void pci_s_write_config32(pci_devfn_t dev, uint16_t reg, UINT32 value);
void pci_write_config32(const struct device *dev, UINT16 reg, UINT32 val);

#endif /* PCI_OPS_H */
