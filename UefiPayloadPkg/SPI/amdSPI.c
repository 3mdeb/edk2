#include <Include/PiDxe.h>
#include <Include/Library/DebugLib.h>

void spi_init(void)
{
	DEBUG((EFI_D_INFO, "%a: SPI BAR at 0x%08lx\n", __FUNCTION__, spi_get_bar()));
}

static uintptr_t spi_base;

void spi_set_base(void *base)
{
	spi_base = (uintptr_t)base;
}

uintptr_t spi_get_bar(void)
{
	if (!spi_base)
		spi_set_base((void *)lpc_get_spibase());

	return spi_base;
}
