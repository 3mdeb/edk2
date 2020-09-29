#ifndef FCH_SPI_UTIL_H
#define FCH_SPI_UTIL_H

#include <Include/PiDxe.h>
#include "pci_type.h"

VOID spi_set_base(VOID *base);
unsigned long int spi_get_bar(VOID);
UINT8 spi_read8(UINT8 reg);
UINT16 spi_read16(UINT8 reg);
UINT32 spi_read32(UINT8 reg);
VOID spi_write8(UINT8 reg, UINT8 val);
VOID spi_write16(UINT8 reg, UINT16 val);
VOID spi_write32(UINT8 reg, UINT32 val);

#endif /* FCH_SPI_UTIL_H */
