#ifndef SPI_FLASH_H
#define  SPI_FLASH_H

#include <Include/PiDxe.h>
#include "SPIgeneric.h"

int spi_flash_vector_helper(const struct spi_slave *slave,
	struct spi_op vectors[], __SIZE_TYPE__ count,
	int (*func)(const struct spi_slave *slave, const void *dout,
		    __SIZE_TYPE__ bytesout, void *din, __SIZE_TYPE__ bytesin));


#endif /* SPI_FLASH_H */
