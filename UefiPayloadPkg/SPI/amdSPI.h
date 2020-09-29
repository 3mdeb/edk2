#ifndef AMD_SPI_H
#define AMD_SPI_H

#include <Include/PiDxe.h>

VOID spi_init(VOID);
UINT32 *spi_get_bar(void);
void spi_set_base(void *base);

#endif /* AMD_SPI_H */
