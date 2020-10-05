#include <Include/PiDxe.h>

#include "lpc.h"
#include "pci_ops.h"
#include "utils.h"

unsigned long int lpc_get_spibase(void)
{
	DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
	UINT32 base;

	base = pci_read_config32((struct device *)_LPCB_DEV, SPIROM_BASE_ADDRESS_REGISTER);
	base = ALIGN_DOWN(base, SPI_BASE_ALIGNMENT);
	return (unsigned long int)base;
}
