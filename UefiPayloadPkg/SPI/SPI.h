/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef SPI_H
#define SPI_H

#include <Include/PiDxe.h>

#define PCU_BUS		0x00
#define PCU_DEV		0x14
#define LPC_FUNC	0x03

#define SPI_BUSY	BIT31

#define SPI_FIFO	0x80
#define SPI_FIFO_LAST_BYTE	0xc7
#define SPI_FIFO_DEPTH	(SPI_FIFO_LAST_BYTE - SPI_FIFO)

#endif /* SPI_H */
