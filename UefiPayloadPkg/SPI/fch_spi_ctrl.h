#ifndef FCH_SPI_CTRL_H
#define FCH_SPI_CTRL_H

#include <Include/PiDxe.h>

#include "kconfig.h"
#include "SPI.h"
#include "utils.h"
#include "stopwatch.h"
#include "fch_spi_util.h"
#include "SPIgeneric.h"

union pci_bank {
	UINT8 reg8[4096];
	UINT16 reg16[4096 / sizeof(UINT16)];
	UINT32 reg32[4096 / sizeof(UINT32)];
};

union pci_bank *pcicfg(pci_devfn_t dev);
VOID dump_state(CONST char *str, UINT8 phase);
int wait_for_ready(VOID);
int execute_command(VOID);
VOID spi_init(VOID);
int spi_ctrlr_xfer(CONST struct spi_slave *slave, CONST VOID *dout,
			__SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin);
int xfer_vectors(CONST struct spi_slave *slave,
			struct spi_op vectors[], __SIZE_TYPE__ count);
int protect_a_range(UINT32 value);

/*
 * Protect range of SPI flash defined by region using the SPI flash controller.
 *
 * Note: Up to 4 ranges can be protected, though if a particular region requires more than one
 * range, total number of regions decreases accordingly. Each range can be programmed to 4KiB or
 * 64KiB granularity.
 *
 * Warning: If more than 1 region needs protection, and they need mixed protections (read/write)
 * than start with the region that requires the most protection. After the restricted commands
 * have been written, they can't be changed (write once). So if first region is write protection
 * and second region is read protection, it's best to define first region as read and write
 * protection.
 */
int fch_spi_flash_protect(CONST struct spi_flash *flash, CONST struct region *region,
				 CONST enum ctrlr_prot_type type);

#endif /* FCH_SPI_CTRL_H */
