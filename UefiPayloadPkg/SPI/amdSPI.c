#include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>

#include "amdSPI.h"

void spi_init(void)
{
	DEBUG((EFI_D_INFO, "%a: SPI BAR at 0x%08lx\n", __FUNCTION__, spi_get_bar()));
}

static UINT32 *spi_base;

void spi_set_base(void *base)
{
	spi_base = (UINT32 *)base;
}

UINT32 *spi_get_bar(void)
{
	if (!spi_base)
		spi_set_base((void *)lpc_get_spibase());

	return spi_base;
}
