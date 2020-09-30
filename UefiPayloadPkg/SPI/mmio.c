/* SPDX-License-Identifier: GPL-2.0-only */
/* Originally imported from linux/include/asm-arm/io.h. This file has changed
 * substantially since then.
 */

#include <Include/PiDxe.h>
#include "mmio.h"

static inline UINT8 read8(CONST VOID *addr)
{
	return *(volatile uint8_t *)addr;
}

static inline UINT16 read16(CONST VOID *addr)
{
	return *(volatile UINT16 *)addr;
}

static inline UINT32 read32(CONST VOID *addr)
{
	return *(volatile UINT32 *)addr;
}

static inline VOID write8(VOID *addr, uint8_t val)
{
	*(volatile uint8_t *)addr = val;
}

static inline VOID write16(VOID *addr, UINT16 val)
{
	*(volatile UINT16 *)addr = val;
}

static inline VOID write32(VOID *addr, UINT32 val)
{
	*(volatile UINT32 *)addr = val;
}
