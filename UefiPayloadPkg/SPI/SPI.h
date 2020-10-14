/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __AMDBLOCKS_SPI_H__
#define __AMDBLOCKS_SPI_H__

#include <Include/PiDxe.h>

#define SPI_CNTRL0			0x00
#define   SPI_BUSY			BIT31

enum spi_read_mode {
	SPI_READ_MODE_NORMAL33M = 0,
	/* 1 is reserved. */
	SPI_READ_MODE_DUAL112 = 2,
	SPI_READ_MODE_QUAD114 = 3,
	SPI_READ_MODE_DUAL122 = 4,
	SPI_READ_MODE_QUAD144 = 5,
	SPI_READ_MODE_NORMAL66M = 6,
	SPI_READ_MODE_FAST_READ = 7,
};
/*
 * SPI read mode is split into bits 18, 29, 30 such that [30:29:18] correspond to bits [2:0] for
 * SpiReadMode.
 */
#define   SPI_READ_MODE_MASK		(BIT30 | BIT29 | BIT18)
#define   SPI_READ_MODE_UPPER_BITS(x)	((((x) >> 1) & 0x3) << 29)
#define   SPI_READ_MODE_LOWER_BITS(x)	(((x) & 0x1) << 18)
#define   SPI_READ_MODE(x)		(SPI_READ_MODE_UPPER_BITS(x) | \
					 SPI_READ_MODE_LOWER_BITS(x))
#define   SPI_ACCESS_MAC_ROM_EN		BIT22

#define SPI100_ENABLE			0x20
#define   SPI_USE_SPI100		BIT0

/* Use SPI_SPEED_16M-SPI_SPEED_66M below for the southbridge */
#define SPI100_SPEED_CONFIG		0x22
enum spi100_speed {
	SPI_SPEED_66M = 0,
	SPI_SPEED_33M = 1,
	SPI_SPEED_22M = 2,
	SPI_SPEED_16M = 3,
	SPI_SPEED_100M = 4,
	SPI_SPEED_800K = 5,
};

#define   SPI_SPEED_MASK		0xf
#define   SPI_SPEED_MODE(x, shift)	(((x) & SPI_SPEED_MASK) << shift)
#define   SPI_NORM_SPEED(x)		SPI_SPEED_MODE(x, 12)
#define   SPI_FAST_SPEED(x)		SPI_SPEED_MODE(x, 8)
#define   SPI_ALT_SPEED(x)		SPI_SPEED_MODE(x, 4)
#define   SPI_TPM_SPEED(x)		SPI_SPEED_MODE(x, 0)

#define   SPI_SPEED_CFG(n, f, a, t)	(SPI_NORM_SPEED(n) | SPI_FAST_SPEED(f) | \
					 SPI_ALT_SPEED(a) | SPI_TPM_SPEED(t))

#define SPI100_HOST_PREF_CONFIG		0x2c
#define   SPI_RD4DW_EN_HOST		BIT15

#define SPI_FIFO			0x80
#define SPI_FIFO_LAST_BYTE		0xc7
#define SPI_FIFO_DEPTH			(SPI_FIFO_LAST_BYTE - SPI_FIFO)

struct spi_config {
	/*
	 * Default values if not overridden by mainboard:
	 * Read mode - Normal 33MHz
	 * Normal speed - 66MHz
	 * Fast speed - 66MHz
	 * Alt speed - 66MHz
	 * TPM speed - 66MHz
	 */
	enum spi_read_mode read_mode;
	enum spi100_speed normal_speed;
	enum spi100_speed fast_speed;
	enum spi100_speed altio_speed;
	enum spi100_speed tpm_speed;
};

/* Set the SPI base address variable */
VOID spi_set_base(VOID *base);

#endif /* __AMDBLOCKS_SPI_H__ */
