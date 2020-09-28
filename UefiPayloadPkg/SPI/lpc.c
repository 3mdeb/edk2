#include <Include/PiDxe.h>

#include "lpc.h"

UINT32 *lpc_get_spibase(void)
{
	UINT32 base;

	base = pci_read_config32(_LPCB_DEV, SPIROM_BASE_ADDRESS_REGISTER);
	base = ALIGN_DOWN(base, SPI_BASE_ALIGNMENT);
	return (UINT32)base;
}
