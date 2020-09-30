/* SPDX-License-Identifier: GPL-2.0-only */
#include <Include/PiDxe.h>
#include "mmio.h"
#include "fch_spi_util.h"
#include "lpc.h"

static unsigned long int spi_base;

VOID spi_set_base(VOID *base)
{
	spi_base = (unsigned long int)base;
}

unsigned long int spi_get_bar(VOID)
{
	if (!spi_base)
		spi_set_base((VOID *)lpc_get_spibase());

	return spi_base;
}

UINT8 spi_read8(UINT8 reg)
{
	return read8((VOID *)(spi_get_bar() + reg));
}

UINT16 spi_read16(UINT8 reg)
{
	return read8((VOID *)(spi_get_bar() + reg));
}

UINT32 spi_read32(UINT8 reg)
{
	return read32((VOID *)(spi_get_bar() + reg));
}

VOID spi_write8(UINT8 reg, UINT8 val)
{
	write8((VOID *)(spi_get_bar() + reg), val);
}

VOID spi_write16(UINT8 reg, UINT16 val)
{
	write16((VOID *)(spi_get_bar() + reg), val);
}

VOID spi_write32(UINT8 reg, UINT32 val)
{
	write32((VOID *)(spi_get_bar() + reg), val);
}
