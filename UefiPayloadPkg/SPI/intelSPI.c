/* SPDX-License-Identifier: GPL-2.0-or-later */

#define __SIMPLE_DEVICE__

/* This file is derived from the flashrom project. */

// #include <stdint.h>
// #include <string.h>
// #include <bootstate.h>
// #include <commonlib/helpers.h>
// #include <delay.h>
// #include <device/mmio.h>
// #include <device/pci_ops.h>
// #include <console/console.h>
// #include <device/device.h>
// #include <device/pci.h>
// #include <spi_flash.h>
// #include <SPIgeneric.h>
// #include <timer.h>

#include <Include/PiDxe.h>
#include <Include/Base.h>
#include "SPI.h"
#include "SPIgeneric.h"
#include "helpers.h"

/* SPDX-License-Identifier: GPL-2.0-or-later */

#define __SIMPLE_DEVICE__

#define HSFC_FCYCLE_OFF		1	/* 1-2: FLASH Cycle */
#define HSFC_FCYCLE		(0x3 << HSFC_FCYCLE_OFF)
#define HSFC_FDBC_OFF		8	/* 8-13: Flash Data Byte Count */
#define HSFC_FDBC		(0x3f << HSFC_FDBC_OFF)

/* SPI Flash opcodes */
#define SPI_OPCODE_WREN 0x06
#define SPI_OPCODE_FAST_READ 0x0b

#define NSECS_PER_SEC 1000000000
#define USECS_PER_SEC 1000000
#define MSECS_PER_SEC 1000
#define USECS_PER_MSEC (USECS_PER_SEC / MSECS_PER_SEC)

#define __ARG_PLACEHOLDER_1 0,
#define config_enabled(cfg) _config_enabled(cfg)
#define _config_enabled(value) __config_enabled(__ARG_PLACEHOLDER_##value)
#define __config_enabled(arg1_or_junk) ___config_enabled(arg1_or_junk 1, 0, 0)
#define ___config_enabled(__ignored, val, ...) val
#define CONFIG(option) config_enabled(CONFIG_##option)

#if CONFIG(SOUTHBRIDGE_INTEL_I82801GX)
#define MENU_BYTES member_size(struct ich7_spi_regs, opmenu)
#else
#define MENU_BYTES member_size(struct ich9_spi_regs, opmenu)
#endif

#define pci_read_config8 pci_s_read_config8
#define pci_read_config16 pci_s_read_config16
#define pci_read_config32 pci_s_read_config32
#define pci_write_config8 pci_s_write_config8
#define pci_write_config16 pci_s_write_config16
#define pci_write_config32 pci_s_write_config32

typedef UINT32 pci_devfn_t;

/* Convert pci_devfn_t to offset in MMCONF space.
 * As it is one-to-one,  nothing needs to be done. */
#define PCI_DEVFN_OFFSET(x) ((x))

#define PCI_DEV(SEGBUS, DEV, FN) ( \
	(((SEGBUS) & 0xFFF) << 20) | \
	(((DEV) & 0x1F) << 15) | \
	(((FN)  & 0x07) << 12))

#define PCI_DEV_INVALID (0xffffffffU)

typedef enum {
	BS_PRE_DEVICE,
	BS_DEV_INIT_CHIPS,
	BS_DEV_ENUMERATE,
	BS_DEV_RESOURCES,
	BS_DEV_ENABLE,
	BS_DEV_INIT,
	BS_POST_DEVICE,
	BS_OS_RESUME_CHECK,
	BS_OS_RESUME,
	BS_WRITE_TABLES,
	BS_PAYLOAD_LOAD,
	BS_PAYLOAD_BOOT,
} boot_state_t;

typedef enum {
	BS_ON_ENTRY,
	BS_ON_EXIT
} boot_state_sequence_t;

struct boot_state_callback {
	void *arg;
	void (*callback)(void *arg);
	/* For use internal to the boot state machine. */
	struct boot_state_callback *next;
};

struct boot_state_init_entry {
	boot_state_t state;
	boot_state_sequence_t when;
	struct boot_state_callback bscb;
};

#define BOOT_STATE_CALLBACK_INIT(func_, arg_)		\
	{						\
		.arg = arg_,				\
		.callback = func_,			\
		.next = NULL,				\
	}


#define BOOT_STATE_INIT_ENTRY(state_, when_, func_, arg_)		\
	static struct boot_state_init_entry func_ ##_## state_ ##_## when_ = \
	{								\
		.state = state_,					\
		.when = when_,						\
		.bscb = BOOT_STATE_CALLBACK_INIT(func_, arg_),		\
	};								\
	static struct boot_state_init_entry *				\
		bsie_ ## func_ ##_## state_ ##_## when_; \
		BOOT_STATE_INIT_ATTR = \
		&func_ ##_## state_ ##_## when_;

static INT32 spi_is_multichip(VOID);

typedef UINT32 pci_devfn_t;

/*
 * Representation of SPI flash operations:
 * read:	Flash read operation.
 * write:	Flash write operation.
 * erase:	Flash erase operation.
 * status:	Read flash status register.
 */
struct spi_flash_ops {
	int (*read)(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
			VOID *buf);
	int (*write)(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
			CONST VOID *buf);
	int (*erase)(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len);
	int (*status)(CONST struct spi_flash *flash, UINT8 *reg);
};

/* By not assigning this to CONFIG_MMCONF_BASE_ADDRESS here we
 * prevent some sub-optimal constant folding. */
extern UINT8 *const pci_mmconf;

/* Using a unique datatype for MMIO writes makes the pointers to _not_
 * qualify for pointer aliasing with any other objects in memory.
 *
 * MMIO offset is a value originally derived from 'struct device *'
 * in ramstage. For the compiler to not discard this MMIO offset value
 * from CPU registers after any MMIO writes, -fstrict-aliasing has to
 * be also set for the build.
 *
 * Bottom 12 bits (4 KiB) are reserved to address the registers of a
 * single PCI function. Declare the bank as a union to avoid some casting
 * in the functions below.
 */
union pci_bank {
	UINT8 reg8[4096];
	UINT16 reg16[4096 / sizeof(UINT16)];
	UINT32 reg32[4096 / sizeof(UINT32)];
};

static  __attribute__((always_inline))
volatile union pci_bank *pcicfg(pci_devfn_t dev)
{
	return (void *)&pci_mmconf[PCI_DEVFN_OFFSET(dev)];
}

static  __attribute__((always_inline))
UINT8 pci_mmio_read_config8(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg8[reg];
}

static  __attribute__((always_inline))
UINT16 pci_mmio_read_config16(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg16[reg / sizeof(UINT16)];
}

static  __attribute__((always_inline))
UINT32 pci_mmio_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg32[reg / sizeof(UINT32)];
}

static __attribute__((always_inline))
void pci_mmio_write_config8(pci_devfn_t dev, UINT16 reg, UINT8 value)
{
	pcicfg(dev)->reg8[reg] = value;
}

static  __attribute__((always_inline))
void pci_mmio_write_config16(pci_devfn_t dev, UINT16 reg, UINT16 value)
{
	pcicfg(dev)->reg16[reg / sizeof(UINT16)] = value;
}

static  __attribute__((always_inline))
void pci_mmio_write_config32(pci_devfn_t dev, UINT16 reg, UINT32 value)
{
	pcicfg(dev)->reg32[reg / sizeof(UINT32)] = value;
}

/*
 * The functions pci_mmio_config*_addr provide a way to determine the MMIO address of a PCI
 * config register. The address returned is dependent of both the MMCONF base address and the
 * assigned PCI bus number of the requested device, which both can change during the boot
 * process. Thus, the pointer returned here must not be cached!
 */
static __always_inline
UINT8 *pci_mmio_config8_addr(pci_devfn_t dev, UINT16 reg)
{
	return (UINT8 *)&pcicfg(dev)->reg8[reg];
}

static __always_inline
UINT16 *pci_mmio_config16_addr(pci_devfn_t dev, UINT16 reg)
{
	return (UINT16 *)&pcicfg(dev)->reg16[reg / sizeof(UINT16)];
}

static __always_inline
UINT32 *pci_mmio_config32_addr(pci_devfn_t dev, UINT16 reg)
{
	return (UINT32 *)&pcicfg(dev)->reg32[reg / sizeof(UINT32)];
}

struct ich7_spi_regs {
	UINT16 spis;
	UINT16 spic;
	UINT32 spia;
	UINT64 spid[8];
	UINT64 _pad;
	UINT32 bbar;
	UINT16 preop;
	UINT16 optype;
	UINT8 opmenu[8];
	UINT32 pbr[3];
} __attribute__((packed));

struct ich9_spi_regs {
	UINT32 bfpr;
	UINT16 hsfs;
	UINT16 hsfc;
	UINT32 faddr;
	UINT32 _reserved0;
	UINT32 fdata[16];
	UINT32 frap;
	UINT32 freg[5];
	UINT32 _reserved1[3];
	UINT32 pr[5];
	UINT32 _reserved2[2];
	UINT8 ssfs;
	UINT8 ssfc[3];
	UINT16 preop;
	UINT16 optype;
	UINT8 opmenu[8];
	UINT32 bbar;
	UINT8 _reserved3[12];
	UINT32 fdoc;
	UINT32 fdod;
	UINT8 _reserved4[8];
	UINT32 afc;
	UINT32 lvscc;
	UINT32 uvscc;
	UINT8 _reserved5[4];
	UINT32 fpb;
	UINT8 _reserved6[28];
	UINT32 srdl;
	UINT32 srdc;
	UINT32 srd;
} __attribute__((packed));

struct ich_spi_controller {
	INT32 locked;
	UINT32 flmap0;
	UINT32 flcomp;
	UINT32 hsfs;

	union {
		struct ich9_spi_regs *ich9_spi;
		struct ich7_spi_regs *ich7_spi;
	};
	UINT8 *opmenu;
	INT32 menubytes;
	UINT16 *preop;
	UINT16 *optype;
	UINT32 *addr;
	UINT8 *data;
	UINT32 databytes;
	UINT8 *status;
	UINT16 *control;
	UINT32 *bbar;
	UINT32 *fpr;
	UINT8 fpr_max;
};

static struct ich_spi_controller cntlr;

enum {
	SPIS_SCIP =						0x0001,
	SPIS_GRANT =					0x0002,
	SPIS_CDS =						0x0004,
	SPIS_FCERR =					0x0008,
	SSFS_AEL =						0x0010,
	SPIS_LOCK =						0x8000,
	SPIS_RESERVED_MASK =	0x7ff0,
	SSFS_RESERVED_MASK =	0x7fe2
};

enum {
	SPIC_SCGO =				0x000002,
	SPIC_ACS =				0x000004,
	SPIC_SPOP =				0x000008,
	SPIC_DBC =				0x003f00,
	SPIC_DS =					0x004000,
	SPIC_SME =				0x008000,
	SSFC_SCF_MASK =		0x070000,
	SSFC_RESERVED =		0xf80000
};

enum {
	HSFS_FDONE =				0x0001,
	HSFS_FCERR =				0x0002,
	HSFS_AEL =					0x0004,
	HSFS_BERASE_MASK =	0x0018,
	HSFS_BERASE_SHIFT =	3,
	HSFS_SCIP =					0x0020,
	HSFS_FDOPSS =				0x2000,
	HSFS_FDV =					0x4000,
	HSFS_FLOCKDN =			0x8000
};

enum {
	HSFC_FGO =					0x0001,
	HSFC_FCYCLE_MASK =	0x0006,
	HSFC_FCYCLE_SHIFT =	1,
	HSFC_FDBC_MASK =		0x3f00,
	HSFC_FDBC_SHIFT =		8,
	HSFC_FSMIE =				0x8000
};

enum {
	SPI_OPCODE_TYPE_READ_NO_ADDRESS =			0,
	SPI_OPCODE_TYPE_WRITE_NO_ADDRESS =		1,
	SPI_OPCODE_TYPE_READ_WITH_ADDRESS =		2,
	SPI_OPCODE_TYPE_WRITE_WITH_ADDRESS =	3
};

static inline UINT8 read8(const void *addr)
{
	return *(volatile UINT8 *)addr;
}

static inline UINT16 read16(const void *addr)
{
	return *(volatile UINT16 *)addr;
}

static inline UINT32 read32(const void *addr)
{
	return *(volatile UINT32 *)addr;
}

static inline void write8(void *addr, UINT8 val)
{
	*(volatile UINT8 *)addr = val;
}

static inline void write16(void *addr, UINT16 val)
{
	*(volatile UINT16 *)addr = val;
}

static inline void write32(void *addr, UINT32 val)
{
	*(volatile UINT32 *)addr = val;
}

#define DEBUG_SPI_FLASH

#ifdef DEBUG_SPI_FLASH

static UINT8 readb_(CONST VOID *addr)
{
	UINT8 v = read8(addr);

	DEBUG((EFI_D_INFO, "%a read %2.2x from %4.4x\n",
	       __FUNCTION__, v, ((UINT64) addr & 0xffff) - 0xf020));
	return v;
}

static UINT16 readw_(CONST VOID *addr)
{
	UINT16 v = read16(addr);

	DEBUG((EFI_D_INFO, "%a read %4.4x from %4.4x\n",
	       __FUNCTION__, v, ((UINT64) addr & 0xffff) - 0xf020));
	return v;
}

static UINT32 readl_(CONST VOID *addr)
{
	UINT32 v = read32(addr);

	DEBUG((EFI_D_INFO, "%a read %8.8x from %4.4x\n",
	       __FUNCTION__, v, ((UINT64) addr & 0xffff) - 0xf020));
	return v;
}

static VOID writeb_(UINT8 b, VOID *addr)
{
	write8(addr, b);
	DEBUG((EFI_D_INFO, "%a wrote %2.2x to %4.4x\n",
	       __FUNCTION__, b, ((UINT64) addr & 0xffff) - 0xf020));
}

static VOID writew_(UINT16 b, VOID *addr)
{
	write16(addr, b);
	DEBUG((EFI_D_INFO, "%a wrote %4.4x to %4.4x\n",
	       __FUNCTION__, b, ((UINT64) addr & 0xffff) - 0xf020));
}

static VOID writel_(UINT32 b, VOID *addr)
{
	write32(addr, b);
	DEBUG((EFI_D_INFO, "%a wrote %8.8x to %4.4x\n",
	       __FUNCTION__, b, ((UINT64) addr & 0xffff) - 0xf020));
}

#else /* CONFIG_DEBUG_SPI_FLASH ^^^ enabled  vvv NOT enabled */

#define readb_(a) read8(a)
#define readw_(a) read16(a)
#define readl_(a) read32(a)
#define writeb_(val, addr) write8(addr, val)
#define writew_(val, addr) write16(addr, val)
#define writel_(val, addr) write32(addr, val)

#endif  /* CONFIG_DEBUG_SPI_FLASH ^^^ NOT enabled */

static VOID write_reg(CONST VOID *value, VOID *dest, UINT32 size)
{
	CONST UINT8 *bvalue = value;
	UINT8 *bdest = dest;

	while (size >= 4) {
		writel_(*(CONST UINT32 *)bvalue, bdest);
		bdest += 4; bvalue += 4; size -= 4;
	}
	while (size) {
		writeb_(*bvalue, bdest);
		bdest++; bvalue++; size--;
	}
}

static VOID read_reg(CONST VOID *src, VOID *value, UINT32 size)
{
	CONST UINT8 *bsrc = src;
	UINT8 *bvalue = value;

	while (size >= 4) {
		*(UINT32 *)bvalue = readl_(bsrc);
		bsrc += 4; bvalue += 4; size -= 4;
	}
	while (size) {
		*bvalue = readb_(bsrc);
		bsrc++; bvalue++; size--;
	}
}

static VOID ich_set_bbar(UINT32 minaddr)
{
	CONST UINT32 bbar_mask = 0x00ffff00;
	UINT32 ichspi_bbar;

	minaddr &= bbar_mask;
	ichspi_bbar = readl_(cntlr.bbar) & ~bbar_mask;
	ichspi_bbar |= minaddr;
	writel_(ichspi_bbar, cntlr.bbar);
}

#if CONFIG(SOUTHBRIDGE_INTEL_I82801GX)
#define MENU_BYTES member_size(struct ich7_spi_regs, opmenu)
#else
#define MENU_BYTES member_size(struct ich9_spi_regs, opmenu)
#endif

#define RCBA 0xf0
#define SBASE 0x54

static VOID *get_spi_bar(pci_devfn_t dev)
{
	uintptr_t rcba; /* Root Complex Register Block */
	uintptr_t sbase;

	if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX)) {
		rcba = pci_read_config32(dev, RCBA);
		return (VOID *)((rcba & 0xffffc000) + 0x3020);
	}
	if (CONFIG(SOUTHBRIDGE_INTEL_COMMON_SPI_SILVERMONT)) {
		sbase = pci_read_config32(dev, SBASE);
		sbase &= ~0x1ff;
		return (VOID *)sbase;
	}
	if (CONFIG(SOUTHBRIDGE_INTEL_COMMON_SPI_ICH9)) {
		rcba = pci_read_config32(dev, RCBA);
		return (VOID *)((rcba & 0xffffc000) + 0x3800);
	}
}

VOID spi_init(VOID)
{
	UINT8 bios_cntl;
	struct ich9_spi_regs *ich9_spi;
	struct ich7_spi_regs *ich7_spi;
	UINT16 hsfs;

	pci_devfn_t dev = PCI_DEV(0, 31, 0);

	if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX)) {
		ich7_spi = get_spi_bar(dev);
		cntlr.ich7_spi = ich7_spi;
		cntlr.opmenu = ich7_spi->opmenu;
		cntlr.menubytes = sizeof(ich7_spi->opmenu);
		cntlr.optype = &ich7_spi->optype;
		cntlr.addr = &ich7_spi->spia;
		cntlr.data = (UINT8 *)ich7_spi->spid;
		cntlr.databytes = sizeof(ich7_spi->spid);
		cntlr.status = (UINT8 *)&ich7_spi->spis;
		cntlr.control = &ich7_spi->spic;
		cntlr.bbar = &ich7_spi->bbar;
		cntlr.preop = &ich7_spi->preop;
		cntlr.fpr = &ich7_spi->pbr[0];
		cntlr.fpr_max = 3;
	} else {
		ich9_spi = get_spi_bar(dev);
		cntlr.ich9_spi = ich9_spi;
		hsfs = readw_(&ich9_spi->hsfs);
		cntlr.hsfs = hsfs;
		cntlr.opmenu = ich9_spi->opmenu;
		cntlr.menubytes = sizeof(ich9_spi->opmenu);
		cntlr.optype = &ich9_spi->optype;
		cntlr.addr = &ich9_spi->faddr;
		cntlr.data = (UINT8 *)ich9_spi->fdata;
		cntlr.databytes = sizeof(ich9_spi->fdata);
		cntlr.status = &ich9_spi->ssfs;
		cntlr.control = (UINT16 *)ich9_spi->ssfc;
		cntlr.bbar = &ich9_spi->bbar;
		cntlr.preop = &ich9_spi->preop;
		cntlr.fpr = &ich9_spi->pr[0];
		cntlr.fpr_max = 5;

		if (cntlr.hsfs & HSFS_FDV) {
			writel_(4, &ich9_spi->fdoc);
			cntlr.flmap0 = readl_(&ich9_spi->fdod);
			writel_(0x1000, &ich9_spi->fdoc);
			cntlr.flcomp = readl_(&ich9_spi->fdod);
		}
	}

	ich_set_bbar(0);

	if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX) || CONFIG(SOUTHBRIDGE_INTEL_COMMON_SPI_ICH9)) {
		/* Disable the BIOS write protect so write commands are allowed. */
		bios_cntl = pci_read_config8(dev, 0xdc);
		/* Deassert SMM BIOS Write Protect Disable. */
		bios_cntl &= ~(1 << 5);
		pci_write_config8(dev, 0xdc, bios_cntl | 0x1);
	}
}

static INT32 spi_locked(VOID)
{
	if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX)) {
		return !!(readw_(&cntlr.ich7_spi->spis) & HSFS_FLOCKDN);
	} else {
		return !!(readw_(&cntlr.ich9_spi->hsfs) & HSFS_FLOCKDN);
	}
}

static VOID spi_init_cb(VOID *unused)
{
	spi_init();
}

BOOT_STATE_INIT_ENTRY(BS_DEV_INIT, BS_ON_ENTRY, spi_init_cb, NULL);

typedef struct spi_transaction {
	CONST UINT8 *out;
	UINT32 bytesout;
	UINT8 *in;
	UINT32 bytesin;
	UINT8 type;
	UINT8 opcode;
	UINT32 offset;
} spi_transaction;

static inline VOID spi_use_out(spi_transaction *trans, UINT32 bytes)
{
	trans->out += bytes;
	trans->bytesout -= bytes;
}

static inline VOID spi_use_in(spi_transaction *trans, UINT32 bytes)
{
	trans->in += bytes;
	trans->bytesin -= bytes;
}

static VOID spi_setup_type(spi_transaction *trans)
{
	trans->type = 0xFF;

	/* Try to guess spi type from read/write sizes. */
	if (trans->bytesin == 0) {
		if (trans->bytesout > 4)
			/*
			 * If bytesin = 0 and bytesout > 4, we presume this is
			 * a write data operation, which is accompanied by an
			 * address.
			 */
			trans->type = SPI_OPCODE_TYPE_WRITE_WITH_ADDRESS;
		else
			trans->type = SPI_OPCODE_TYPE_WRITE_NO_ADDRESS;
		return;
	}

	if (trans->bytesout == 1) { /* and bytesin is > 0 */
		trans->type = SPI_OPCODE_TYPE_READ_NO_ADDRESS;
		return;
	}

	if (trans->bytesout == 4) { /* and bytesin is > 0 */
		trans->type = SPI_OPCODE_TYPE_READ_WITH_ADDRESS;
	}

	/* Fast read command is called with 5 bytes instead of 4 */
	if (trans->out[0] == SPI_OPCODE_FAST_READ && trans->bytesout == 5) {
		trans->type = SPI_OPCODE_TYPE_READ_WITH_ADDRESS;
		--trans->bytesout;
	}
}

static INT32 spi_setup_opcode(spi_transaction *trans)
{
	UINT16 optypes;
	UINT8 opmenu[MENU_BYTES];

	trans->opcode = trans->out[0];
	spi_use_out(trans, 1);
	if (!spi_locked()) {
		/* The lock is off, so just use index 0. */
		writeb_(trans->opcode, cntlr.opmenu);
		optypes = readw_(cntlr.optype);
		optypes = (optypes & 0xfffc) | (trans->type & 0x3);
		writew_(optypes, cntlr.optype);
		return 0;
	}

	/* The lock is on. See if what we need is on the menu. */
	UINT8 optype;
	UINT16 opcode_index;

	/* Write Enable is handled as atomic prefix */
	if (trans->opcode == SPI_OPCODE_WREN)
		return 0;

	read_reg(cntlr.opmenu, opmenu, sizeof(opmenu));
	for (opcode_index = 0; opcode_index < ARRAY_SIZE(opmenu); opcode_index++) {
		if (opmenu[opcode_index] == trans->opcode)
			break;
	}

	if (opcode_index == ARRAY_SIZE(opmenu)) {
		DEBUG((EFI_D_INFO, "%a ICH SPI: Opcode %x not found\n",
			__FUNCTION__, trans->opcode));
		return -1;
	}

	optypes = readw_(cntlr.optype);
	optype = (optypes >> (opcode_index * 2)) & 0x3;
	if (trans->type == SPI_OPCODE_TYPE_WRITE_NO_ADDRESS &&
		optype == SPI_OPCODE_TYPE_WRITE_WITH_ADDRESS &&
		trans->bytesout >= 3) {
		/* We guessed wrong earlier. Fix it up. */
		trans->type = optype;
	}
	if (optype != trans->type) {
		DEBUG((EFI_D_INFO, "%a ICH SPI: Transaction doesn't fit type %d\n",
			__FUNCTION__, optype));
		return -1;
	}
	return opcode_index;
}

static INT32 spi_setup_offset(spi_transaction *trans)
{
	/* Separate the SPI address and data. */
	switch (trans->type) {
	case SPI_OPCODE_TYPE_READ_NO_ADDRESS:
	case SPI_OPCODE_TYPE_WRITE_NO_ADDRESS:
		return 0;
	case SPI_OPCODE_TYPE_READ_WITH_ADDRESS:
	case SPI_OPCODE_TYPE_WRITE_WITH_ADDRESS:
		trans->offset = ((UINT32)trans->out[0] << 16) |
				((UINT32)trans->out[1] << 8) |
				((UINT32)trans->out[2] << 0);
		spi_use_out(trans, 3);
		return 1;
	default:
		DEBUG((EFI_D_INFO, "%a Unrecognized SPI transaction type %#x\n", __FUNCTION__, trans->type));
		return -1;
	}
}

/*
 * Wait for up to 6s til status register bit(s) turn 1 (in case wait_til_set
 * below is True) or 0. In case the wait was for the bit(s) to set - write
 * those bits back, which would cause resetting them.
 *
 * Return the last read status value on success or -1 on failure.
 */
static INT32 ich_status_poll(UINT16 bitmask, INT32 wait_til_set)
{
	INT32 timeout = 600000; /* This will result in 6 seconds */
	UINT16 status = 0;

	while (timeout--) {
		status = readw_(cntlr.status);
		if (wait_til_set ^ ((status & bitmask) == 0)) {
			if (wait_til_set)
				writew_((status & bitmask), cntlr.status);
			return status;
		}
		udelay(10);
	}

	DEBUG((EFI_D_INFO, "%a ICH SPI: SCIP timeout, read %x, bitmask %x\n",
		__FUNCTION__, status, bitmask));
	return -1;
}

static INT32 spi_is_multichip(VOID)
{
	if (!(cntlr.hsfs & HSFS_FDV))
		return 0;
	return !!((cntlr.flmap0 >> 8) & 3);
}

static INT32 spi_ctrlr_xfer(CONST struct spi_slave *slave, CONST VOID *dout,
		__SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin)
{
	UINT16 control;
	int16_t opcode_index;
	INT32 with_address;
	INT32 status;

	spi_transaction trans = {
		dout, bytesout,
		din, bytesin,
		0xff, 0xff, 0
	};

	/* There has to always at least be an opcode. */
	if (!bytesout || !dout) {
		DEBUG((EFI_D_INFO, "%a ICH SPI: No opcode for transfer\n", __FUNCTION__));;
		return -1;
	}
	/* Make sure if we read something we have a place to put it. */
	if (bytesin != 0 && !din) {
		DEBUG((EFI_D_INFO, "%a ICH SPI: Read but no target buffer\n", __FUNCTION__));;
		return -1;
	}

	if (ich_status_poll(SPIS_SCIP, 0) == -1)
		return -1;

	writew_(SPIS_CDS | SPIS_FCERR, cntlr.status);

	spi_setup_type(&trans);
	if ((opcode_index = spi_setup_opcode(&trans)) < 0)
		return -1;
	if ((with_address = spi_setup_offset(&trans)) < 0)
		return -1;

	if (trans.opcode == SPI_OPCODE_WREN) {
		/*
		 * Treat Write Enable as Atomic Pre-Op if possible
		 * in order to prevent the Management Engine from
		 * issuing a transaction between WREN and DATA.
		 */
		if (!spi_locked())
			writew_(trans.opcode, cntlr.preop);
		return 0;
	}

	/* Preset control fields */
	control = SPIC_SCGO | ((opcode_index & 0x07) << 4);

	/* Issue atomic preop cycle if needed */
	if (readw_(cntlr.preop))
		control |= SPIC_ACS;

	if (!trans.bytesout && !trans.bytesin) {
		/* SPI addresses are 24 bit only */
		if (with_address)
			writel_(trans.offset & 0x00FFFFFF, cntlr.addr);

		/*
		 * This is a 'no data' command (like Write Enable), its
		 * bitesout size was 1, decremented to zero while executing
		 * spi_setup_opcode() above. Tell the chip to send the
		 * command.
		 */
		writew_(control, cntlr.control);

		/* wait for the result */
		status = ich_status_poll(SPIS_CDS | SPIS_FCERR, 1);
		if (status == -1)
			return -1;

		if (status & SPIS_FCERR) {
			DEBUG((EFI_D_INFO, "%a ICH SPI: Command transaction error\n", __FUNCTION__));;
			return -1;
		}

		goto spi_xfer_exit;
	}

	/*
	 * Check if this is a write command attempting to transfer more bytes
	 * than the controller can handle. Iterations for writes are not
	 * supported here because each SPI write command needs to be preceded
	 * and followed by other SPI commands, and this sequence is controlled
	 * by the SPI chip driver.
	 */
	if (trans.bytesout > cntlr.databytes) {
		DEBUG((EFI_D_INFO, "%a ICH SPI: Too much to write. Does your SPI chip driver use"
		     " spi_crop_chunk()?\n", __FUNCTION__));
		return -1;
	}

	/*
	 * Read or write up to databytes bytes at a time until everything has
	 * been sent.
	 */
	while (trans.bytesout || trans.bytesin) {
		UINT32 data_length;

		/* SPI addresses are 24 bit only */
		writel_(trans.offset & 0x00FFFFFF, cntlr.addr);

		if (trans.bytesout)
			data_length = MIN(trans.bytesout, cntlr.databytes);
		else
			data_length = MIN(trans.bytesin, cntlr.databytes);

		/* Program data into FDATA0 to N */
		if (trans.bytesout) {
			write_reg(trans.out, cntlr.data, data_length);
			spi_use_out(&trans, data_length);
			if (with_address)
				trans.offset += data_length;
		}

		/* Add proper control fields' values */
		control &= ~((cntlr.databytes - 1) << 8);
		control |= SPIC_DS;
		control |= (data_length - 1) << 8;

		/* write it */
		writew_(control, cntlr.control);

		/* Wait for Cycle Done Status or Flash Cycle Error. */
		status = ich_status_poll(SPIS_CDS | SPIS_FCERR, 1);
		if (status == -1)
			return -1;

		if (status & SPIS_FCERR) {
			DEBUG((EFI_D_INFO, "%a ICH SPI: Data transaction error\n", __FUNCTION__));;
			return -1;
		}

		if (trans.bytesin) {
			read_reg(cntlr.data, trans.in, data_length);
			spi_use_in(&trans, data_length);
			if (with_address)
				trans.offset += data_length;
		}
	}

spi_xfer_exit:
	/* Clear atomic preop now that xfer is done */
	writew_(0, cntlr.preop);

	return 0;
}

/* Sets FLA in FADDR to (addr & 0x01FFFFFF) without touching other bits. */
static VOID ich_hwseq_set_addr(UINT32 addr)
{
	UINT32 addr_old = readl_(&cntlr.ich9_spi->faddr) & ~0x01FFFFFF;

	writel_((addr & 0x01FFFFFF) | addr_old, &cntlr.ich9_spi->faddr);
}

/* Polls for Cycle Done Status, Flash Cycle Error or timeout in 8 us intervals.
   Resets all error flags in HSFS.
   Returns 0 if the cycle completes successfully without errors within
   timeout us, 1 on errors. */
static INT32 ich_hwseq_wait_for_cycle_complete(UINT32 timeout,
					     UINT32 len)
{
	UINT16 hsfs;
	UINT32 addr;

	timeout /= 8; /* scale timeout duration to counter */
	while ((((hsfs = readw_(&cntlr.ich9_spi->hsfs)) &
		 (HSFS_FDONE | HSFS_FCERR)) == 0) &&
	       --timeout) {
		udelay(8);
	}
	writew_(readw_(&cntlr.ich9_spi->hsfs), &cntlr.ich9_spi->hsfs);

	if (!timeout) {
		UINT16 hsfc;
		addr = readl_(&cntlr.ich9_spi->faddr) & 0x01FFFFFF;
		hsfc = readw_(&cntlr.ich9_spi->hsfc);
		DEBUG((EFI_D_INFO, "%a Transaction timeout between offset 0x%08x and "
		       "0x%08x (= 0x%08x + %d) HSFC=%x HSFS=%x!\n",
					 __FUNCTION__,
		       addr, addr + len - 1, addr, len - 1,
		       hsfc, hsfs));
		return 1;
	}

	if (hsfs & HSFS_FCERR) {
		UINT16 hsfc;
		addr = readl_(&cntlr.ich9_spi->faddr) & 0x01FFFFFF;
		hsfc = readw_(&cntlr.ich9_spi->hsfc);
		DEBUG((EFI_D_INFO, "%a Transaction error between offset 0x%08x and "
		       "0x%08x (= 0x%08x + %d) HSFC=%x HSFS=%x!\n",
					 __FUNCTION__,
		       addr, addr + len - 1, addr, len - 1,
		       hsfc, hsfs));
		return 1;
	}
	return 0;
}


static INT32 ich_hwseq_erase(CONST struct spi_flash *flash, UINT32 offset,
			__SIZE_TYPE__ len)
{
	UINT32 start, end, erase_size;
	INT32 ret;
	UINT16 hsfc;
	UINT32 timeout = 1000 * USECS_PER_MSEC; /* 1 second timeout */

	erase_size = flash->sector_size;
	if (offset % erase_size || len % erase_size) {
		DEBUG((EFI_D_INFO, "%a SF: Erase offset/length not multiple of erase size\n", __FUNCTION__));
		return -1;
	}

	ret = spi_claim_bus(&flash->spi);
	if (ret) {
		DEBUG((EFI_D_INFO, "%a SF: Unable to claim SPI bus\n", __FUNCTION__));
		return ret;
	}

	start = offset;
	end = start + len;

	while (offset < end) {
		/* make sure FDONE, FCERR, AEL are cleared by writing 1 to them */
		writew_(readw_(&cntlr.ich9_spi->hsfs), &cntlr.ich9_spi->hsfs);

		ich_hwseq_set_addr(offset);

		offset += erase_size;

		hsfc = readw_(&cntlr.ich9_spi->hsfc);
		hsfc &= ~HSFC_FCYCLE; /* clear operation */
		hsfc |= (0x3 << HSFC_FCYCLE_OFF); /* set erase operation */
		hsfc |= HSFC_FGO; /* start */
		writew_(hsfc, &cntlr.ich9_spi->hsfc);
		if (ich_hwseq_wait_for_cycle_complete(timeout, len)) {
			DEBUG((EFI_D_INFO, "%a SF: Erase failed at %x\n", __FUNCTION__, offset - erase_size));
			ret = -1;
			goto out;
		}
	}

	DEBUG((EFI_D_INFO, "%a SF: Successfully erased %zu bytes @ %#x\n", __FUNCTION__, len, start));

out:
	spi_release_bus(&flash->spi);
	return ret;
}

static VOID ich_read_data(UINT8 *data, INT32 len)
{
	INT32 i;
	UINT32 temp32 = 0;

	for (i = 0; i < len; i++) {
		if ((i % 4) == 0)
			temp32 = readl_(cntlr.data + i);

		data[i] = (temp32 >> ((i % 4) * 8)) & 0xff;
	}
}

static INT32 ich_hwseq_read(CONST struct spi_flash *flash, UINT32 addr, __SIZE_TYPE__ len,
			VOID *buf)
{
	UINT16 hsfc;
	UINT16 timeout = 100 * 60;
	UINT8 block_len;

	if (addr + len > flash->size) {
		DEBUG((EFI_D_INFO, "%a Attempt to read %x-%x which is out of chip\n",
			__FUNCTION__,
			(UINT32) addr,
			(UINT32) addr+(UINT32) len));
		return -1;
	}

	/* clear FDONE, FCERR, AEL by writing 1 to them (if they are set) */
	writew_(readw_(&cntlr.ich9_spi->hsfs), &cntlr.ich9_spi->hsfs);

	while (len > 0) {
		block_len = MIN(len, cntlr.databytes);
		if (block_len > (~addr & 0xff))
			block_len = (~addr & 0xff) + 1;
		ich_hwseq_set_addr(addr);
		hsfc = readw_(&cntlr.ich9_spi->hsfc);
		hsfc &= ~HSFC_FCYCLE; /* set read operation */
		hsfc &= ~HSFC_FDBC; /* clear byte count */
		/* set byte count */
		hsfc |= (((block_len - 1) << HSFC_FDBC_OFF) & HSFC_FDBC);
		hsfc |= HSFC_FGO; /* start */
		writew_(hsfc, &cntlr.ich9_spi->hsfc);

		if (ich_hwseq_wait_for_cycle_complete(timeout, block_len))
			return 1;
		ich_read_data(buf, block_len);
		addr += block_len;
		buf += block_len;
		len -= block_len;
	}
	return 0;
}

/* Fill len bytes from the data array into the fdata/spid registers.
 *
 * Note that using len > flash->pgm->spi.max_data_write will trash the registers
 * following the data registers.
 */
static VOID ich_fill_data(CONST UINT8 *data, INT32 len)
{
	UINT32 temp32 = 0;
	INT32 i;

	if (len <= 0)
		return;

	for (i = 0; i < len; i++) {
		if ((i % 4) == 0)
			temp32 = 0;

		temp32 |= ((UINT32) data[i]) << ((i % 4) * 8);

		if ((i % 4) == 3) /* 32 bits are full, write them to regs. */
			writel_(temp32, cntlr.data + (i - (i % 4)));
	}
	i--;
	if ((i % 4) != 3) /* Write remaining data to regs. */
		writel_(temp32, cntlr.data + (i - (i % 4)));
}

static INT32 ich_hwseq_write(CONST struct spi_flash *flash, UINT32 addr, __SIZE_TYPE__ len,
			CONST VOID *buf)
{
	UINT16 hsfc;
	UINT16 timeout = 100 * 60;
	UINT8 block_len;
	UINT32 start = addr;

	if (addr + len > flash->size) {
		DEBUG((EFI_D_INFO, "%a Attempt to write 0x%x-0x%x which is out of chip\n",
			__FUNCTION__, (UINT32)addr, (UINT32) (addr+len)));
		return -1;
	}

	/* clear FDONE, FCERR, AEL by writing 1 to them (if they are set) */
	writew_(readw_(&cntlr.ich9_spi->hsfs), &cntlr.ich9_spi->hsfs);

	while (len > 0) {
		block_len = MIN(len, cntlr.databytes);
		if (block_len > (~addr & 0xff))
			block_len = (~addr & 0xff) + 1;

		ich_hwseq_set_addr(addr);

		ich_fill_data(buf, block_len);
		hsfc = readw_(&cntlr.ich9_spi->hsfc);
		hsfc &= ~HSFC_FCYCLE; /* clear operation */
		hsfc |= (0x2 << HSFC_FCYCLE_OFF); /* set write operation */
		hsfc &= ~HSFC_FDBC; /* clear byte count */
		/* set byte count */
		hsfc |= (((block_len - 1) << HSFC_FDBC_OFF) & HSFC_FDBC);
		hsfc |= HSFC_FGO; /* start */
		writew_(hsfc, &cntlr.ich9_spi->hsfc);

		if (ich_hwseq_wait_for_cycle_complete(timeout, block_len)) {
			DEBUG((EFI_D_INFO, "%a SF: write failure at %x\n",
				__FUNCTION__, addr));
			return -1;
		}
		addr += block_len;
		buf += block_len;
		len -= block_len;
	}
	DEBUG((EFI_D_INFO, "%a SF: Successfully written %u bytes @ %#x\n",
	       __FUNCTION__, (UINT32) (addr - start), start));
	return 0;
}

static CONST struct spi_flash_ops spi_flash_ops = {
	.read = ich_hwseq_read,
	.write = ich_hwseq_write,
	.erase = ich_hwseq_erase,
};

static INT32 spi_flash_programmer_probe(CONST struct spi_slave *spi,
					struct spi_flash *flash)
{

	if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX))
		return spi_flash_generic_probe(spi, flash);

	/* Try generic probing first if spi_is_multichip returns 0. */
	if (!spi_is_multichip() && !spi_flash_generic_probe(spi, flash))
		return 0;

	memcpy(&flash->spi, spi, sizeof(*spi));

	ich_hwseq_set_addr(0);
	switch ((cntlr.hsfs >> 3) & 3) {
	case 0:
		flash->sector_size = 256;
		break;
	case 1:
		flash->sector_size = 4096;
		break;
	case 2:
		flash->sector_size = 8192;
		break;
	case 3:
		flash->sector_size = 65536;
		break;
	}

	flash->size = 1 << (19 + (cntlr.flcomp & 7));

	flash->ops = &spi_flash_ops;

	if ((cntlr.hsfs & HSFS_FDV) && ((cntlr.flmap0 >> 8) & 3))
		flash->size += 1 << (19 + ((cntlr.flcomp >> 3) & 7));
	DEBUG((EFI_D_INFO, "%a flash size 0x%x bytes\n", __FUNCTION__, flash->size));

	return 0;
}

static INT32 xfer_vectors(CONST struct spi_slave *slave,
			struct spi_op vectors[], __SIZE_TYPE__ count)
{
	return spi_flash_vector_helper(slave, vectors, count, spi_ctrlr_xfer);
}

#define SPI_FPR_SHIFT			12
#define ICH7_SPI_FPR_MASK		0xfff
#define ICH9_SPI_FPR_MASK		0x1fff
#define SPI_FPR_BASE_SHIFT		0
#define ICH7_SPI_FPR_LIMIT_SHIFT	12
#define ICH9_SPI_FPR_LIMIT_SHIFT	16
#define ICH9_SPI_FPR_RPE		(1 << 15) /* Read Protect */
#define SPI_FPR_WPE			(1 << 31) /* Write Protect */

static UINT32 spi_fpr(UINT32 base, UINT32 limit)
{
	UINT32 ret;
	UINT32 mask, limit_shift;

	if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX)) {
		mask = ICH7_SPI_FPR_MASK;
		limit_shift = ICH7_SPI_FPR_LIMIT_SHIFT;
	} else {
		mask = ICH9_SPI_FPR_MASK;
		limit_shift = ICH9_SPI_FPR_LIMIT_SHIFT;
	}
	ret = ((limit >> SPI_FPR_SHIFT) & mask) << limit_shift;
	ret |= ((base >> SPI_FPR_SHIFT) & mask) << SPI_FPR_BASE_SHIFT;
	return ret;
}

/*
 * Protect range of SPI flash defined by [start, start+size-1] using Flash
 * Protected Range (FPR) register if available.
 * Returns 0 on success, -1 on failure of programming fpr registers.
 */
static INT32 spi_flash_protect(CONST struct spi_flash *flash,
			CONST struct region *region,
			CONST enum ctrlr_prot_type type)
{
	UINT32 start = region_offset(region);
	UINT32 end = start + region_sz(region) - 1;
	UINT32 reg;
	UINT32 protect_mask = 0;
	INT32 fpr;
	UINT32 *fpr_base;

	fpr_base = cntlr.fpr;

	/* Find first empty FPR */
	for (fpr = 0; fpr < cntlr.fpr_max; fpr++) {
		reg = read32(&fpr_base[fpr]);
		if (reg == 0)
			break;
	}

	if (fpr == cntlr.fpr_max) {
		DEBUG((EFI_D_INFO, "%a ERROR: No SPI FPR free!\n", __FUNCTION__));
		return -1;
	}

	switch (type) {
	case WRITE_PROTECT:
		protect_mask |= SPI_FPR_WPE;
		break;
	case READ_PROTECT:
		if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX))
			return -1;
		protect_mask |= ICH9_SPI_FPR_RPE;
		break;
	case READ_WRITE_PROTECT:
		if (CONFIG(SOUTHBRIDGE_INTEL_I82801GX))
			return -1;
		protect_mask |= (ICH9_SPI_FPR_RPE | SPI_FPR_WPE);
		break;
	default:
		DEBUG((EFI_D_INFO, "%a ERROR: Seeking invalid protection!\n", __FUNCTION__));
		return -1;
	}

	/* Set protected range base and limit */
	reg = spi_fpr(start, end) | protect_mask;

	/* Set the FPR register and verify it is protected */
	write32(&fpr_base[fpr], reg);
	if (reg != read32(&fpr_base[fpr])) {
		DEBUG((EFI_D_INFO, "%a ERROR: Unable to set SPI FPR %d\n", __FUNCTION__, fpr));
		return -1;
	}

	DEBUG((EFI_D_INFO, "%a: FPR %d is enabled for range 0x%08x-0x%08x\n",
	       __FUNCTION__, fpr, start, end));
	return 0;
}

VOID spi_finalize_ops(VOID)
{
	UINT16 spi_opprefix;
	UINT16 optype = 0;
	struct intel_swseq_spi_config spi_config_default = {
		{0x06, 0x50},  /* OPPREFIXES: EWSR and WREN */
		{ /* OPCODE and OPTYPE */
			{0x01, WRITE_NO_ADDR},		/* WRSR: Write Status Register */
			{0x02, WRITE_WITH_ADDR},	/* BYPR: Byte Program */
			{0x03, READ_WITH_ADDR},		/* READ: Read Data */
			{0x05, READ_NO_ADDR},		/* RDSR: Read Status Register */
			{0x20, WRITE_WITH_ADDR},	/* SE20: Sector Erase 0x20 */
			{0x9f, READ_NO_ADDR},		/* RDID: Read ID */
			{0xd8, WRITE_WITH_ADDR},	/* BED8: Block Erase 0xd8 */
			{0x0b, READ_WITH_ADDR},		/* FAST: Fast Read */
		}
	};
	struct intel_swseq_spi_config spi_config_aai_write = {
		{0x06, 0x50}, /* OPPREFIXES: EWSR and WREN */
		{ /* OPCODE and OPTYPE */
			{0x01, WRITE_NO_ADDR},		/* WRSR: Write Status Register */
			{0x02, WRITE_WITH_ADDR},	/* BYPR: Byte Program */
			{0x03, READ_WITH_ADDR},		/* READ: Read Data */
			{0x05, READ_NO_ADDR},		/* RDSR: Read Status Register */
			{0x20, WRITE_WITH_ADDR},	/* SE20: Sector Erase 0x20 */
			{0x9f, READ_NO_ADDR},		/* RDID: Read ID */
			{0xad, WRITE_NO_ADDR},		/* Auto Address Increment Word Program */
			{0x04, WRITE_NO_ADDR}		/* Write Disable */
		}
	};
	CONST struct spi_flash *flash = boot_device_spi_flash();
	struct intel_swseq_spi_config *spi_config = &spi_config_default;
	INT32 i;

	/*
	 * Some older SST SPI flashes support AAI write but use 0xaf opcde for
	 * that. Flashrom uses the byte program opcode to write those flashes,
	 * so this configuration is fine too. SST25VF064C (id = 0x4b) is an
	 * exception.
	 */
	if (flash && flash->vendor == VENDOR_ID_SST && (flash->model & 0x00ff) != 0x4b)
		spi_config = &spi_config_aai_write;

	if (spi_locked())
		return;

	intel_southbridge_override_spi(spi_config);

	spi_opprefix = spi_config->opprefixes[0]
		| (spi_config->opprefixes[1] << 8);
	writew_(spi_opprefix, cntlr.preop);
	for (i = 0; i < ARRAY_SIZE(spi_config->ops); i++) {
		optype |= (spi_config->ops[i].type & 3) << (i * 2);
		writeb_(spi_config->ops[i].op, &cntlr.opmenu[i]);
	}
	writew_(optype, cntlr.optype);
}

__attribute__((__weak__))
VOID intel_southbridge_override_spi(struct intel_swseq_spi_config *spi_config)
{
}

static CONST struct spi_ctrlr spi_ctrlr = {
	.xfer_vector = xfer_vectors,
	.max_xfer_size = member_size(struct ich9_spi_regs, fdata),
	.flash_probe = spi_flash_programmer_probe,
	.flash_protect = spi_flash_protect,
};

CONST struct spi_ctrlr_buses spi_ctrlr_bus_map[] = {
	{
		.ctrlr = &spi_ctrlr,
		.bus_start = 0,
		.bus_end = 0,
	},
};

CONST __SIZE_TYPE__ spi_ctrlr_bus_map_count = ARRAY_SIZE(spi_ctrlr_bus_map);


/*-----------------------------------------------------------------------
 * Representation of a SPI controller. Note the xfer() and xfer_vector()
 * callbacks are meant to process full duplex transactions. If the
 * controller cannot handle these transactions then return an error when
 * din and dout are both set. See spi_xfer() below for more details.
 *
 * claim_bus:		Claim SPI bus and prepare for communication.
 * release_bus:	Release SPI bus.
 * setup:		Setup given SPI device bus.
 * xfer:		Perform one SPI transfer operation.
 * xfer_vector:	Vector of SPI transfer operations.
 * xfer_dual:		(optional) Perform one SPI transfer in Dual SPI mode.
 * max_xfer_size:	Maximum transfer size supported by the controller
 *			(0 = invalid,
 *			 SPI_CTRLR_DEFAULT_MAX_XFER_SIZE = unlimited)
 * flags:		See SPI_CNTRLR_* enums above.
 *
 * Following member is provided by specialized SPI controllers that are
 * actually SPI flash controllers.
 *
 * flash_probe:	Specialized probe function provided by SPI flash
 *			controllers.
 * flash_protect: Protect a region of flash using the SPI flash controller.
 */

/*
struct spi_ctrlr {
	INT32 (*claim_bus)(CONST struct spi_slave *slave);
	VOID (*release_bus)(CONST struct spi_slave *slave);
	INT32 (*setup)(CONST struct spi_slave *slave);
	INT32 (*xfer)(CONST struct spi_slave *slave, CONST VOID *dout,
		    __SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin);
	INT32 (*xfer_vector)(CONST struct spi_slave *slave,
			struct spi_op vectors[], __SIZE_TYPE__ count);
	INT32 (*xfer_dual)(CONST struct spi_slave *slave, CONST VOID *dout,
			 __SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin);
	UINT32 max_xfer_size;
	UINT32 flags;
	INT32 (*flash_probe)(CONST struct spi_slave *slave,
				struct spi_flash *flash);
	INT32 (*flash_protect)(CONST struct spi_flash *flash,
				CONST struct region *region,
				CONST enum ctrlr_prot_type type);
};
*/


static CONST struct spi_ctrlr spi_ctrlr = {
	.claim_bus = NULL,
	.release_bus = NULL,
	.setup = NULL,
	.xfer = spi_ctrlr_xfer,
	.xfer_vector = NULL,
	.xfer_dual  = NULL,
	.max_xfer_size = 0x40,
	.flags = 0,
	.flash_probe = NULL,
	.flash_protect = NULL
};

CONST struct spi_ctrlr_buses spi_ctrlr_bus_map[] = {
	{
		.ctrlr = &spi_ctrlr,
		.bus_start = 0,
		.bus_end = 0,
	},
};

CONST __SIZE_TYPE__ spi_ctrlr_bus_map_count = ARRAY_SIZE(spi_ctrlr_bus_map);
