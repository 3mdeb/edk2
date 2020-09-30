#ifndef MMIO_H
#define MMIO_H

#include <Include/PiDxe.h>

static inline UINT8 read8(CONST VOID *addr);
static inline UINT16 read16(CONST VOID *addr);
static inline UINT32 read32(CONST VOID *addr);
static inline VOID write8(VOID *addr, UINT8 val);
static inline VOID write16(VOID *addr, UINT16 val);
static inline VOID write32(VOID *addr, UINT32 val);

#endif /* MMIO_H */
