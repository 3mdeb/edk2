#include <Include/PiDxe.h>

#ifndef BIT
#define BIT(x)				(1ul << (x))
#endif

#define SPIROM_BASE_ADDRESS_REGISTER	0xa0
#define SPI_BASE_ALIGNMENT	BIT(6)
#define SPI_BASE_RESERVED		(BIT(4) | BIT(5))
#define ROUTE_TPM_2_SPI		  BIT(3)
#define SPI_ABORT_ENABLE		BIT(2)
#define SPI_ROM_ENABLE		  BIT(1)
#define SPI_ROM_ALT_ENABLE	BIT(0)
#define SPI_PRESERVE_BITS		(BIT(0) | BIT(1) | BIT(2) | BIT(3))

#define _LPCB_DEV PCI_DEV(0, 0x14, 0x3)

UINT32 *lpc_get_spibase(void);
