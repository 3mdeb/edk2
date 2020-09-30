/* SPDX-License-Identifier: GPL-2.0-only */
/* Originally imported from linux/include/asm-arm/io.h. This file has changed
 * substantially since then.
 */

#include <Include/PiDxe.h>
#include "endian.h"
#include "mmio.h"

static inline UINT8 read8(CONST VOID *addr)
{
	return *(volatile uint8_t *)addr;
}

static inline uint16_t read16(CONST VOID *addr)
{
	return *(volatile uint16_t *)addr;
}

static inline uint32_t read32(CONST VOID *addr)
{
	return *(volatile uint32_t *)addr;
}

static inline VOID write8(VOID *addr, uint8_t val)
{
	*(volatile uint8_t *)addr = val;
}

static inline VOID write16(VOID *addr, uint16_t val)
{
	*(volatile uint16_t *)addr = val;
}

static inline VOID write32(VOID *addr, uint32_t val)
{
	*(volatile uint32_t *)addr = val;
}
