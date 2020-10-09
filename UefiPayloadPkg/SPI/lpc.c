#include <Include/PiDxe.h>

#include "lpc.h"
#include "pci_ops.h"
#include "utils.h"

unsigned long int lpc_get_spibase(void)
{
	DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
	UINT32 base;
	DEBUG((EFI_D_INFO, "%a _LPCB_DEV = 0x%X\n", __FUNCTION__, _LPCB_DEV));
	DEBUG((EFI_D_INFO, "%a SPIROM_BASE_ADDRESS_REGISTER = 0x%X\n", __FUNCTION__, SPIROM_BASE_ADDRESS_REGISTER));
	base = pci_read_config32((struct device *)_LPCB_DEV, SPIROM_BASE_ADDRESS_REGISTER);
	DEBUG((EFI_D_INFO, "%a base before align 0x%X\n", __FUNCTION__, base));
	base = ALIGN_DOWN(base, SPI_BASE_ALIGNMENT);
	DEBUG((EFI_D_INFO, "%a base after align 0x%X\n", __FUNCTION__, base));
	return (unsigned long int)base;
}
