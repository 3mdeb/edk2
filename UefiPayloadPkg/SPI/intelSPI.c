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
#include "spi_winbond.h"
#include "spi_flash_internal.h"

/* SPDX-License-Identifier: GPL-2.0-or-later */

#define __SIMPLE_DEVICE__

#define HSFC_FCYCLE_OFF		1	/* 1-2: FLASH Cycle */
#define HSFC_FCYCLE		(0x3 << HSFC_FCYCLE_OFF)
#define HSFC_FDBC_OFF		8	/* 8-13: Flash Data Byte Count */
#define HSFC_FDBC		(0x3f << HSFC_FDBC_OFF)

/* SPI Flash opcodes */
#define SPI_OPCODE_WREN 0x06
#define SPI_OPCODE_FAST_READ 0x0b

/* Common commands */
#define CMD_READ_ID			0x9f

#define CMD_READ_ARRAY_SLOW		0x03
#define CMD_READ_ARRAY_FAST		0x0b
#define CMD_READ_ARRAY_LEGACY		0xe8

#define CMD_READ_FAST_DUAL_OUTPUT	0x3b

#define CMD_READ_STATUS			0x05
#define CMD_WRITE_ENABLE		0x06

#define CMD_BLOCK_ERASE			0xD8

/* Common status */
#define STATUS_WIP			0x01

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

/* Convert pci_devfn_t to offset in MMCONF space.
 * As it is one-to-one,  nothing needs to be done. */
#define PCI_DEVFN_OFFSET(x) ((x))

#define PCI_DEV(SEGBUS, DEV, FN) ( \
	(((SEGBUS) & 0xFFF) << 20) | \
	(((DEV) & 0x1F) << 15) | \
	(((FN)  & 0x07) << 12))

#define PCI_DEV_INVALID (0xffffffffU)

/* By not assigning this to CONFIG_MMCONF_BASE_ADDRESS here we
 * prevent some sub-optimal constant folding. */
extern UINT8 *CONST pci_mmconf;

/* Using a unique datatype for MMIO writes makes the pointers to _not_
 * qualify for pointer aliasing with any other objects in memory.
 *
 * MMIO offset is a value originally derived from 'struct device *'
 * in ramstage. For the compiler to not discard this MMIO offset value
 * from CPU registers after any MMIO writes, -fstrict-aliasing has to
 * be also set for the build.
 *
 * Bottom 12 bits (4 KiB) are reserved to address the registers of a
 * single PCI function. Declare the bank as a union to aVOID some casting
 * in the functions below.
 */
union pci_bank {
	UINT8 reg8[4096];
	UINT16 reg16[4096 / sizeof(UINT16)];
	UINT32 reg32[4096 / sizeof(UINT32)];
};

typedef UINT32 pci_devfn_t;

static  __attribute__((always_inline)) inline
volatile union pci_bank *pcicfg(pci_devfn_t dev)
{
	return (VOID *)&pci_mmconf[PCI_DEVFN_OFFSET(dev)];
}

static  __attribute__((always_inline)) inline
UINT8 pci_mmio_read_config8(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg8[reg];
}

static  __attribute__((always_inline)) inline
UINT16 pci_mmio_read_config16(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg16[reg / sizeof(UINT16)];
}

static  __attribute__((always_inline)) inline
UINT32 pci_mmio_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pcicfg(dev)->reg32[reg / sizeof(UINT32)];
}

static __attribute__((always_inline)) inline
VOID pci_mmio_write_config8(pci_devfn_t dev, UINT16 reg, UINT8 value)
{
	pcicfg(dev)->reg8[reg] = value;
}

static  __attribute__((always_inline)) inline
VOID pci_mmio_write_config16(pci_devfn_t dev, UINT16 reg, UINT16 value)
{
	pcicfg(dev)->reg16[reg / sizeof(UINT16)] = value;
}

static  __attribute__((always_inline)) inline
VOID pci_mmio_write_config32(pci_devfn_t dev, UINT16 reg, UINT32 value)
{
	pcicfg(dev)->reg32[reg / sizeof(UINT32)] = value;
}

static __attribute__((always_inline)) inline
UINT8 pci_s_read_config8(pci_devfn_t dev, UINT16 reg)
{
	return pci_mmio_read_config8(dev, reg);
}

static __attribute__((always_inline)) inline
UINT16 pci_s_read_config16(pci_devfn_t dev, UINT16 reg)
{
	return pci_mmio_read_config16(dev, reg);
}

static __attribute__((always_inline)) inline
UINT32 pci_s_read_config32(pci_devfn_t dev, UINT16 reg)
{
	return pci_mmio_read_config32(dev, reg);
}

static __attribute__((always_inline)) inline
VOID pci_s_write_config8(pci_devfn_t dev, UINT16 reg, UINT8 value)
{
	pci_mmio_write_config8(dev, reg, value);
}

static __attribute__((always_inline)) inline
VOID pci_s_write_config16(pci_devfn_t dev, UINT16 reg, UINT16 value)
{
	pci_mmio_write_config16(dev, reg, value);
}

static __attribute__((always_inline)) inline
VOID pci_s_write_config32(pci_devfn_t dev, UINT16 reg, UINT32 value)
{
	pci_mmio_write_config32(dev, reg, value);
}

/* Function unit addresses. */
enum {
	UP_TAG_BASE = 0x60000000,
	TIMER_BASE = 0x60005000,
	CLK_RST_BASE = 0x60006000,
	FLOW_CTLR_BASE = 0x60007000,
	SECURE_BOOT_BASE = 0x6000C200,
	TEGRA_EVP_BASE = 0x6000f000,
	APB_MISC_BASE = 0x70000000,
	PINMUX_BASE = 0x70003000,
	PMC_CTLR_BASE = 0x7000e400,
	MC_CTLR_BASE = 0x70019000,
	FUSE_BASE = 0x7000F800,
	TEGRA_SDMMC1_BASE = 0x700b0000,
	TEGRA_SDMMC3_BASE = 0x700b0400,
	EMC_BASE = 0x7001B000,
	I2C5_BASE = 0x7000D000,
	I2S_BASE = 0x702d1000
};

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
	VOID *arg;
	VOID (*callback)(VOID *arg);
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

#define BOOT_STATE_INIT_ATTR  __attribute__((unused))

#define BOOT_STATE_INIT_ENTRY(state_, when_, func_, arg_)		\
	static struct boot_state_init_entry func_ ##_## state_ ##_## when_ = \
	{								\
		.state = state_,					\
		.when = when_,						\
		.bscb = BOOT_STATE_CALLBACK_INIT(func_, arg_),		\
	};								\
	static struct boot_state_init_entry *				\
		bsie_ ## func_ ##_## state_ ##_## when_ \
		BOOT_STATE_INIT_ATTR = \
		&func_ ##_## state_ ##_## when_;

typedef UINT32 pci_devfn_t;

/*
 * The functions pci_mmio_config*_addr provide a way to determine the MMIO address of a PCI
 * config register. The address returned is dependent of both the MMCONF base address and the
 * assigned PCI bus number of the requested device, which both can change during the boot
 * process. Thus, the pointer returned here must not be cached!
 */
static __attribute__((always_inline)) inline
UINT8 *pci_mmio_config8_addr(pci_devfn_t dev, UINT16 reg)
{
	return (UINT8 *)&pcicfg(dev)->reg8[reg];
}

static __attribute__((always_inline)) inline
UINT16 *pci_mmio_config16_addr(pci_devfn_t dev, UINT16 reg)
{
	return (UINT16 *)&pcicfg(dev)->reg16[reg / sizeof(UINT16)];
}

static __attribute__((always_inline)) inline
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

static inline UINT8 read8(CONST VOID *addr)
{
	return *(volatile UINT8 *)addr;
}

static inline UINT16 read16(CONST VOID *addr)
{
	return *(volatile UINT16 *)addr;
}

static inline UINT32 read32(CONST VOID *addr)
{
	return *(volatile UINT32 *)addr;
}

static inline VOID write8(VOID *addr, UINT8 val)
{
	*(volatile UINT8 *)addr = val;
}

static inline VOID write16(VOID *addr, UINT16 val)
{
	*(volatile UINT16 *)addr = val;
}

static inline VOID write32(VOID *addr, UINT32 val)
{
	*(volatile UINT32 *)addr = val;
}

#define IDCODE_LEN 5

static UINT32 *timer_us_ptr = (VOID *)(TIMER_BASE + 0x10);
static VOID udelay(UINT64 usecs)
{
	UINT32 start = read32(timer_us_ptr);
	while (read32(timer_us_ptr) - start < usecs)
		;
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
	unsigned long int rcba; /* Root Complex Register Block */
	unsigned long int sbase;

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
	return NULL;
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
		DEBUG((EFI_D_INFO, "------\n"
		"%a SETTING UP CONTROLLER\n"
		"SOUTHBRIDGE_INTEL_I82801GX: 0x%X\n"
		"ich7_spi: 0x%X\n"
		"ich9_spi: 0x%X\n"
		"hsfs: 0x%X\n"
		"opmenu: 0x%X\n"
		"menubytes: 0x%X\n"
		"optype: 0x%X\n"
		"addr: 0x%X\n"
		"data: 0x%X\n"
		"databytes: 0x%X\n"
		"status: 0x%X\n"
		"control: 0x%X\n"
		"bbar: 0x%X\n"
		"preop: 0x%X\n"
		"fpr: 0x%X\n"
		"fpr_max: 0x%X\n"
		"------\n",
	  __FUNCTION__, cntlr.ich7_spi, cntlr.ich9_spi, cntlr.hsfs, cntlr.opmenu,
		cntlr.menubytes, cntlr.optype, cntlr.addr, cntlr.data, cntlr.databytes,
		cntlr.status, cntlr.control, cntlr.bbar, cntlr.preop, cntlr.fpr,
		cntlr.fpr_max));
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

static int spi_ctrlr_xfer(CONST struct spi_slave *slave, CONST VOID *dout,
		__SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin)
{
	UINT16 control;
	INT16 opcode_index;
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

	DEBUG((EFI_D_INFO, "---\n%a\ntrans.bytesout: 0x%X\ncntlr.bytesout\n---\n",
		__FUNCTION__, trans.bytesout, cntlr.databytes));
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

static int do_spi_flash_cmd(CONST struct spi_slave *spi, CONST VOID *dout,
			    __SIZE_TYPE__ bytes_out, VOID *din, __SIZE_TYPE__ bytes_in)
{
	int ret;
	/*
	 * SPI flash requires command-response kind of behavior. Thus, two
	 * separate SPI vectors are required -- first to transmit dout and other
	 * to receive in din. If some specialized SPI flash controllers
	 * (e.g. x86) can perform both command and response together, it should
	 * be handled at SPI flash controller driver level.
	 */
	struct spi_op vectors[] = {
		[0] = { .dout = dout, .bytesout = bytes_out,
			.din = NULL, .bytesin = 0, },
		[1] = { .dout = NULL, .bytesout = 0,
			.din = din, .bytesin = bytes_in },
	};
	__SIZE_TYPE__ count = ARRAY_SIZE(vectors);
	if (!bytes_in)
		count = 1;

	ret = spi_claim_bus(spi);
	if (ret)
		return ret;

	ret = spi_xfer_vector(spi, vectors, count);

	spi_release_bus(spi);
	return ret;
}

int spi_flash_cmd(CONST struct spi_slave *spi, UINT8 cmd, VOID *response, __SIZE_TYPE__ len)
{
	int ret = do_spi_flash_cmd(spi, &cmd, sizeof(cmd), response, len);
	if (ret)
		DEBUG((EFI_D_INFO, "%a SF: Failed to send command %02x: %d\n",
		__FUNCTION__, cmd, ret));

	return ret;
}

/* M25Pxx-specific commands */
#define CMD_M25PXX_WREN		0x06	/* Write Enable */
#define CMD_M25PXX_WRDI		0x04	/* Write Disable */
#define CMD_M25PXX_RDSR		0x05	/* Read Status Register */
#define CMD_M25PXX_WRSR		0x01	/* Write Status Register */
#define CMD_M25PXX_READ		0x03	/* Read Data Bytes */
#define CMD_M25PXX_FAST_READ	0x0b	/* Read Data Bytes at Higher Speed */
#define CMD_M25PXX_PP		0x02	/* Page Program */
#define CMD_M25PXX_SSE		0x20	/* Subsector Erase */
#define CMD_M25PXX_SE		0xd8	/* Sector Erase */
#define CMD_M25PXX_BE		0xc7	/* Bulk Erase */
#define CMD_M25PXX_DP		0xb9	/* Deep Power-down */
#define CMD_M25PXX_RES		0xab	/* Release from DP, and Read Signature */

int stmicro_release_deep_sleep_identify(CONST struct spi_slave *spi, UINT8 *idcode)
{
	if (spi_flash_cmd(spi, CMD_M25PXX_RES, idcode, 4))
		return -1;

	/* Assuming ST parts identify with 0x1X to release from deep
	   power down and read electronic signature. */
	if ((idcode[3] & 0xf0) != 0x10)
		return -1;

	/* Fix up the idcode to mimic rdid jedec instruction. */
	idcode[0] = 0x20;
	idcode[1] = 0x20;
	idcode[2] = idcode[3] + 1;

	return 0;
}

static CONST struct spi_flash_vendor_info *spi_flash_vendors[] = {
#if CONFIG(SPI_FLASH_ADESTO)
	&spi_flash_adesto_vi,
#endif
#if CONFIG(SPI_FLASH_AMIC)
	&spi_flash_amic_vi,
#endif
#if CONFIG(SPI_FLASH_ATMEL)
	&spi_flash_atmel_vi,
#endif
#if CONFIG(SPI_FLASH_EON)
	&spi_flash_eon_vi,
#endif
#if CONFIG(SPI_FLASH_GIGADEVICE)
	&spi_flash_gigadevice_vi,
#endif
#if CONFIG(SPI_FLASH_MACRONIX)
	&spi_flash_macronix_vi,
#endif
#if CONFIG(SPI_FLASH_SPANSION)
	&spi_flash_spansion_ext1_vi,
	&spi_flash_spansion_ext2_vi,
	&spi_flash_spansion_vi,
#endif
#if CONFIG(SPI_FLASH_SST)
	&spi_flash_sst_ai_vi,
	&spi_flash_sst_vi,
#endif
#if CONFIG(SPI_FLASH_STMICRO)
	&spi_flash_stmicro1_vi,
	&spi_flash_stmicro2_vi,
	&spi_flash_stmicro3_vi,
	&spi_flash_stmicro4_vi,
#endif
#if CONFIG(SPI_FLASH_WINBOND)
	&spi_flash_winbond_vi,
#endif
};

static CONST struct spi_flash_part_id *find_part(CONST struct spi_flash_vendor_info *vi,
						UINT16 id[2])
{
	__SIZE_TYPE__ i;
	CONST UINT16 lid[2] = {
		[0] = id[0] & vi->match_id_mask[0],
		[1] = id[1] & vi->match_id_mask[1],
	};


	for (i = 0; i < vi->nr_part_ids; i++) {
		CONST struct spi_flash_part_id *part = &vi->ids[i];

		if (part->id[0] == lid[0] && part->id[1] == lid[1])
			return part;
	}

	return NULL;
}

VOID *memcpy (VOID *destination, CONST VOID *source, __SIZE_TYPE__ num ) {
	for(__SIZE_TYPE__ index = 0; index < num; ++index) {
		((char *)destination)[index] = ((char *)source)[index];
	}
	return destination;
}

static int fill_spi_flash(CONST struct spi_slave *spi, struct spi_flash *flash,
	CONST struct spi_flash_vendor_info *vi,
	CONST struct spi_flash_part_id *part)
{
	memcpy(&flash->spi, spi, sizeof(*spi));
	flash->vendor = vi->id;
	flash->model = part->id[0];

	flash->page_size = 1U << vi->page_size_shift;
	flash->sector_size = (1U << vi->sector_size_kib_shift) * KiB;
	flash->size = flash->sector_size * (1U << part->nr_sectors_shift);
	flash->erase_cmd = vi->desc->erase_cmd;
	flash->status_cmd = vi->desc->status_cmd;
	flash->pp_cmd = vi->desc->pp_cmd;
	flash->wren_cmd = vi->desc->wren_cmd;

	flash->flags.dual_spi = part->fast_read_dual_output_support;

	flash->ops = &vi->desc->ops;
	flash->prot_ops = vi->prot_ops;
	flash->part = part;

	if (vi->after_probe)
		return vi->after_probe(flash);

	return 0;
}

static int find_match(CONST struct spi_slave *spi, struct spi_flash *flash,
			UINT8 manuf_id, UINT16 id[2])
{
	int i;

	for (i = 0; i < (int)ARRAY_SIZE(spi_flash_vendors); i++) {
		CONST struct spi_flash_vendor_info *vi;
		CONST struct spi_flash_part_id *part;

		vi = spi_flash_vendors[i];

		if (manuf_id != vi->id)
			continue;

		part = find_part(vi, id);

		if (part == NULL)
			continue;

		return fill_spi_flash(spi, flash, vi, part);
	}

	return -1;
}

INT64 spi_flash_generic_probe(CONST struct spi_slave *spi,
				struct spi_flash *flash)
{
	INT64 ret, i;
	UINT8 idcode[IDCODE_LEN];
	UINT8 manuf_id;
	UINT16 id[2];

	/* Read the ID codes */
	ret = spi_flash_cmd(spi, CMD_READ_ID, idcode, sizeof(idcode));
	if (ret)
		return -1;

	if (CONFIG(DEBUG_SPI_FLASH)) {
		DEBUG((EFI_D_INFO, "%a SF: Got idcode: ", __FUNCTION__));
		for (i = 0; i < sizeof(idcode); i++)
			DEBUG((EFI_D_INFO, "%a %02x ", __FUNCTION__, idcode[i]));
		DEBUG((EFI_D_INFO, "%a \n", __FUNCTION__));
	}

	manuf_id = idcode[0];

	DEBUG((EFI_D_INFO, "%a Manufacturer: %02x\n", __FUNCTION__, manuf_id));

	/* If no result from RDID command and STMicro parts are enabled attempt
	   to wake the part from deep sleep and obtain alternative id info. */
	if (CONFIG(SPI_FLASH_STMICRO) && manuf_id == 0xff) {
		if (stmicro_release_deep_sleep_identify(spi, idcode))
			return -1;
		manuf_id = idcode[0];
	}

	id[0] = (idcode[1] << 8) | idcode[2];
	id[1] = (idcode[3] << 8) | idcode[4];

	return find_match(spi, flash, manuf_id, id);
}

INT64 spi_flash_vector_helper(CONST struct spi_slave *slave,
	struct spi_op vectors[], __SIZE_TYPE__ count,
	int (*func)(CONST struct spi_slave *slave, CONST VOID *dout,
		    __SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin))
{
	INT64 ret;
	VOID *din;
	__SIZE_TYPE__ bytes_in;

	if (count < 1 || count > 2)
		return -1;

	/* SPI flash commands always have a command first... */
	if (!vectors[0].dout || !vectors[0].bytesout)
		return -1;
	/* And not read any data during the command. */
	if (vectors[0].din || vectors[0].bytesin)
		return -1;

	if (count == 2) {
		/* If response bytes requested ensure the buffer is valid. */
		if (vectors[1].bytesin && !vectors[1].din)
			return -1;
		/* No sends can accompany a receive. */
		if (vectors[1].dout || vectors[1].bytesout)
			return -1;
		din = vectors[1].din;
		bytes_in = vectors[1].bytesin;
	} else {
		din = NULL;
		bytes_in = 0;
	}

	ret = func(slave, vectors[0].dout, vectors[0].bytesout, din, bytes_in);

	if (ret) {
		vectors[0].status = SPI_OP_FAILURE;
		if (count == 2)
			vectors[1].status = SPI_OP_FAILURE;
	} else {
		vectors[0].status = SPI_OP_SUCCESS;
		if (count == 2)
			vectors[1].status = SPI_OP_SUCCESS;
	}

	return ret;
}

#define SPI_FPR_SHIFT			12
#define ICH7_SPI_FPR_MASK		0xfff
#define ICH9_SPI_FPR_MASK		0x1fff
#define SPI_FPR_BASE_SHIFT		0
#define ICH7_SPI_FPR_LIMIT_SHIFT	12
#define ICH9_SPI_FPR_LIMIT_SHIFT	16
#define ICH9_SPI_FPR_RPE		(1 << 15) /* Read Protect */
#define SPI_FPR_WPE			(1 << 31) /* Write Protect */

static inline __SIZE_TYPE__ region_offset(CONST struct region *r)
{
	return r->offset;
}

static inline __SIZE_TYPE__ region_sz(CONST struct region *r)
{
	return r->size;
}

static struct spi_flash spi_flash_info;
static BOOLEAN spi_flash_init_done;

int spi_flash_probe(unsigned int bus, unsigned int cs, struct spi_flash *flash)
{
	struct spi_slave spi;
	int ret = -1;

	if (spi_setup_slave(bus, cs, &spi)) {
		DEBUG((EFI_D_INFO, "%a SF: Failed to set up slave\n", __FUNCTION__));
		return -1;
	}

	/* Try special programmer probe if any. */
	if (spi.ctrlr->flash_probe)
		ret = spi.ctrlr->flash_probe(&spi, flash);

	/* If flash is not found, try generic spi flash probe. */
	if (ret)
		ret = spi_flash_generic_probe(&spi, flash);

	/* Give up -- nothing more to try if flash is not found. */
	if (ret) {
		DEBUG((EFI_D_INFO, "%a SF: Unsupported manufacturer!\n", __FUNCTION__));
		return -1;
	}

	#define CONFIG_BOOT_DEVICE_SPI_FLASH_BUS 0
	#define CONFIG_ROM_SIZE 8196

	CONST char *mode_string = "";
	if (flash->flags.dual_spi && spi.ctrlr->xfer_dual)
		mode_string = " (Dual SPI mode)";
	DEBUG((EFI_D_INFO,
		"%a SF: Detected %02x %04x with sector size 0x%x, total 0x%x%s\n",
		__FUNCTION__, flash->vendor, flash->model,
		flash->sector_size, flash->size, mode_string));
	if (bus == CONFIG_BOOT_DEVICE_SPI_FLASH_BUS
			&& flash->size != CONFIG_ROM_SIZE) {
		DEBUG((EFI_D_INFO, "%a SF size 0x%x does not correspond to"
			" CONFIG_ROM_SIZE 0x%x!!\n", __FUNCTION__, flash->size,
			CONFIG_ROM_SIZE));
	}
	return 0;
}

struct mem_pool {
	UINT8 *buf;
	__SIZE_TYPE__ size;
	UINT8 *last_alloc;
	__SIZE_TYPE__ free_offset;
};

#define MEM_POOL_INIT(buf_, size_)	\
	{				\
		.buf = (buf_),		\
		.size = (size_),	\
		.last_alloc = NULL,	\
		.free_offset = 0,	\
	}

static inline VOID mem_pool_reset(struct mem_pool *mp)
{
	mp->last_alloc = NULL;
	mp->free_offset = 0;
}

/* Initialize a memory pool. */
static inline VOID mem_pool_init(struct mem_pool *mp, VOID *buf, __SIZE_TYPE__ sz)
{
	mp->buf = buf;
	mp->size = sz;
	mem_pool_reset(mp);
}

/* A region_device operations. */
struct region_device;
struct region_device_ops {
	VOID *(*mmap)(CONST struct region_device *, __SIZE_TYPE__, __SIZE_TYPE__);
	int (*munmap)(CONST struct region_device *, VOID *);
	__SIZE_TYPE__ (*readat)(CONST struct region_device *, VOID *, __SIZE_TYPE__, __SIZE_TYPE__);
	__SIZE_TYPE__ (*writeat)(CONST struct region_device *, CONST VOID *, __SIZE_TYPE__,
		__SIZE_TYPE__);
	__SIZE_TYPE__ (*eraseat)(CONST struct region_device *, __SIZE_TYPE__, __SIZE_TYPE__);
};

struct region_device {
	CONST struct region_device *root;
	CONST struct region_device_ops *ops;
	struct region region;
};

struct mmap_helper_region_device {
	struct mem_pool pool;
	struct region_device rdev;
};

#define container_of(ptr, type, member) ({			\
	CONST __typeof__(((type *)0)->member) *__mptr = (ptr);	\
	(type *)((char *)__mptr - offsetof(type, member)); })

VOID mem_pool_free(struct mem_pool *mp, VOID *p)
{
	/* Determine if p was the most recent allocation. */
	if (p == NULL || mp->last_alloc != p)
		return;

	mp->free_offset = mp->last_alloc - mp->buf;
	/* No way to track allocation before this one. */
	mp->last_alloc = NULL;
}

VOID mmap_helper_device_init(struct mmap_helper_region_device *mdev,
				VOID *cache, __SIZE_TYPE__ cache_size)
{
	mem_pool_init(&mdev->pool, cache, cache_size);
}

VOID *mem_pool_alloc(struct mem_pool *mp, __SIZE_TYPE__ sz)
{
	VOID *p;

	/* Make all allocations be at least 8 byte aligned. */
	sz = ALIGN_UP(sz, 8);

	/* Determine if any space available. */
	if ((mp->size - mp->free_offset) < sz)
		return NULL;

	p = &mp->buf[mp->free_offset];

	mp->free_offset += sz;
	mp->last_alloc = p;

	return p;
}

VOID *mmap_helper_rdev_mmap(CONST struct region_device *rd, __SIZE_TYPE__ offset,
				__SIZE_TYPE__ size)
{
	struct mmap_helper_region_device *mdev;
	VOID *mapping;

	mdev = container_of((VOID *)rd, __typeof__(*mdev), rdev);

	mapping = mem_pool_alloc(&mdev->pool, size);

	if (mapping == NULL)
		return NULL;

	if (rd->ops->readat(rd, mapping, offset, size) != size) {
		mem_pool_free(&mdev->pool, mapping);
		return NULL;
	}

	return mapping;
}

int mmap_helper_rdev_munmap(CONST struct region_device *rd, VOID *mapping)
{
	struct mmap_helper_region_device *mdev;

	mdev = container_of((VOID *)rd, __typeof__(*mdev), rdev);

	mem_pool_free(&mdev->pool, mapping);

	return 0;
}

static UINT32 volatile_group_count;

int spi_flash_volatile_group_begin(const struct spi_flash *flash)
{
	UINT32 count;
	int ret = 0;

	if (!CONFIG(SPI_FLASH_HAS_VOLATILE_GROUP))
		return ret;

	count = volatile_group_count;
	if (count == 0)
		ret = 0;

	count++;
	volatile_group_count = count;
	return ret;
}

int spi_flash_volatile_group_end(const struct spi_flash *flash)
{
	UINT32 count;
	int ret = 0;

	if (!CONFIG(SPI_FLASH_HAS_VOLATILE_GROUP))
		return ret;

	count = volatile_group_count;
	//assert(count == 0);
	count--;
	volatile_group_count = count;

	if (count == 0)
		ret = 0;
		//ret = chipset_volatile_group_end(flash);

	return ret;
}

int spi_flash_write(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
		CONST VOID *buf)
{
	int ret;

	if (spi_flash_volatile_group_begin(flash))
		return -1;

	ret = flash->ops->write(flash, offset, len, buf);

	if (spi_flash_volatile_group_end(flash))
		return -1;

	return ret;
}

int spi_flash_read(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
		void *buf)
{
	return flash->ops->read(flash, offset, len, buf);
}

int spi_flash_erase(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len)
{
	int ret;

	if (spi_flash_volatile_group_begin(flash))
		return -1;

	ret = flash->ops->erase(flash, offset, len);

	if (spi_flash_volatile_group_end(flash))
		return -1;

	return ret;
}

static struct spi_flash sfg;

static __SIZE_TYPE__ spi_writeat(CONST struct region_device *rd, CONST VOID *b,
				__SIZE_TYPE__ offset, __SIZE_TYPE__ size)
{
	if (spi_flash_write(&sfg, offset, size, b))
		return -1;

	return size;
}

static __SIZE_TYPE__ spi_readat(CONST struct region_device *rd, VOID *b,
				__SIZE_TYPE__ offset, __SIZE_TYPE__ size)
{
	if (spi_flash_read(&sfg, offset, size, b))
		return -1;

	return size;
}

static __SIZE_TYPE__ spi_eraseat(CONST struct region_device *rd,
				__SIZE_TYPE__ offset, __SIZE_TYPE__ size)
{
	if (spi_flash_erase(&sfg, offset, size))
		return -1;

	return size;
}

/* Provide all operations on the same device. */
static CONST struct region_device_ops spi_ops = {
	.mmap = mmap_helper_rdev_mmap,
	.munmap = mmap_helper_rdev_munmap,
	.readat = spi_readat,
	.writeat = spi_writeat,
	.eraseat = spi_eraseat,
};

#define REGION_DEV_INIT(ops_, offset_, size_)		\
	{						\
		.root = NULL,				\
		.ops = (ops_),				\
		.region = {				\
			.offset = (offset_),		\
			.size = (size_),		\
		},					\
	}

#define MMAP_HELPER_REGION_INIT(ops_, offset_, size_)			\
	{								\
		.rdev = REGION_DEV_INIT((ops_), (offset_), (size_)),	\
	}

#define REGION_SIZE(name) (_e##name - _##name)

#define DECLARE_REGION(name)	\
	extern UINT8 _##name[];	\
	extern UINT8 _e##name[];

/*
 * Regions can be declared optional if not all configurations provide them in
 * memlayout and you want code to be able to check for their existence at
 * runtime. Not every region that is architecture or platform-specific should
 * use this -- only declare regions optional if the code *accessing* them runs
 * both on configurations that have the region and those that don't.  That code
 * should then check (REGION_SIZE(name) != 0) before accessing it.
 */
#define DECLARE_OPTIONAL_REGION(name)	\
	__attribute__((__weak__)) extern UINT8 _##name[];	\
	__attribute__((__weak__)) extern UINT8 _e##name[];

DECLARE_REGION(sram)
DECLARE_OPTIONAL_REGION(timestamp)
DECLARE_REGION(preram_cbmem_console)
DECLARE_REGION(cbmem_init_hooks)
DECLARE_REGION(stack)
DECLARE_REGION(preram_cbfs_cache)
DECLARE_REGION(postram_cbfs_cache)
DECLARE_REGION(cbfs_cache)
DECLARE_REGION(fmap_cache)
DECLARE_REGION(tpm_tcpa_log)

static struct mmap_helper_region_device mdev =
	MMAP_HELPER_REGION_INIT(&spi_ops, 0, CONFIG_ROM_SIZE);

VOID boot_device_init(VOID)
{
	int bus = CONFIG_BOOT_DEVICE_SPI_FLASH_BUS;
	int cs = 0;

	if (spi_flash_init_done == TRUE)
		return;

	if (spi_flash_probe(bus, cs, &spi_flash_info))
		return;

	spi_flash_init_done = TRUE;

	mmap_helper_device_init(&mdev, _cbfs_cache, REGION_SIZE(cbfs_cache));
}

CONST struct spi_flash *boot_device_spi_flash(VOID)
{
	boot_device_init();

	if (spi_flash_init_done != TRUE)
		return NULL;

	return &spi_flash_info;
}

__attribute__((__weak__))
VOID intel_southbridge_override_spi(struct intel_swseq_spi_config *spi_config)
{
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
	.max_xfer_size = 0xFFFFFFFF,
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
