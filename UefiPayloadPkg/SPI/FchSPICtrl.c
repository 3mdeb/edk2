#include <Include/Library/DebugLib.h>
#include <Include/Library/PciLib.h>
#include <Include/Library/TimerLib.h>
#include <Include/PiDxe.h>
#include "FchSPIUtil.h"
#include "GenericSPI.h"
#include "SPI.h"
#include "SPIFlashInternal.h"

/* SPDX-License-Identifier: GPL-2.0-only */

#define GRANULARITY_TEST_4k			0x0000f000		/* bits 15-12 */
#define WORD_TO_DWORD_UPPER(x)	((x << 16) & 0xffff0000)

/* SPI MMIO registers */
#define SPI_RESTRICTED_CMD1				0x04
#define SPI_RESTRICTED_CMD2				0x08
#define SPI_CNTRL1								0x0c
#define SPI_CMD_CODE							0x45
#define SPI_CMD_TRIGGER						0x47
#define SPI_CMD_TRIGGER_EXECUTE		0x80
#define SPI_TX_BYTE_COUNT					0x48
#define SPI_RX_BYTE_COUNT					0x4b
#define SPI_STATUS								0x4c
#define SPI_DONE_BYTE_COUNT_SHIFT	0
#define SPI_DONE_BYTE_COUNT_MASK	0xff
#define SPI_FIFO_WR_PTR_SHIFT			8
#define SPI_FIFO_WR_PTR_MASK			0x7f
#define SPI_FIFO_RD_PTR_SHIFT			16
#define SPI_FIFO_RD_PTR_MASK			0x7f

#define MAX_ROM_PROTECT_RANGES		4
#define ROM_PROTECT_RANGE0				0x50
#define ROM_PROTECT_RANGE_REG(n)	(ROM_PROTECT_RANGE0 + (4 * n))

static int wait_for_ready(VOID)
{
	CONST UINT64 timeoutMilisecond = 500;
	CONST UINT64 nanoToMiliDivider = 1000000;
	CONST UINT64 startTimeMilisec =
		GetTimeInNanoSecond(GetPerformanceCounter()) / nanoToMiliDivider;
	CONST UINT64 endTimeMilisec = startTimeMilisec + timeoutMilisecond;
	UINT64 timeNow = startTimeMilisec;

	do {
		if (!(spi_read32(SPI_STATUS) & SPI_BUSY))
			return 0;
		timeNow = GetTimeInNanoSecond(GetPerformanceCounter()) / nanoToMiliDivider;
	} while (
				timeNow<startTimeMilisec
		|| (timeNow>startTimeMilisec && timeNow>endTimeMilisec));

	return -1;
}

int execute_command(VOID)
{
	spi_write8(SPI_CMD_TRIGGER, SPI_CMD_TRIGGER_EXECUTE);
	if(wait_for_ready())
		DEBUG((EFI_D_INFO,
      "%a: FCH_SC Error: Timeout executing command\n", __FUNCTION__));

	return 0;
}

VOID spi_init(VOID)
{
	spi_get_bar();
}

int spi_ctrlr_xfer(CONST struct spi_slave *slave, CONST VOID *dout,
			__SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin)
{
	__SIZE_TYPE__ count;
	UINT8 cmd;
	UINT8 *bufin = din;
	CONST UINT8 *bufout = dout;

	/* First byte is cmd which cannot be sent through FIFO */
	cmd = bufout[0];
	bufout++;
	bytesout--;

	/*
	 * Check if this is a write command attempting to transfer more bytes
	 * than the controller can handle.  Iterations for writes are not
	 * supported here because each SPI write command needs to be preceded
	 * and followed by other SPI commands.
	 */
	if (bytesout + bytesin > SPI_FIFO_DEPTH) {
		DEBUG((EFI_D_INFO,
      "%a: FCH_SC: Too much to transfer, code error!\n", __FUNCTION__));
		return -1;
	}

	if (wait_for_ready())
		return -1;

	spi_write8(SPI_CMD_CODE, cmd);
	spi_write8(SPI_TX_BYTE_COUNT, bytesout);
	spi_write8(SPI_RX_BYTE_COUNT, bytesin);

	for (count = 0; count < bytesout; count++)
		spi_write8(SPI_FIFO + count, bufout[count]);

	if (execute_command())
		return -1;

	for (count = 0; count < bytesin; count++)
		bufin[count] = spi_read8(SPI_FIFO + count + bytesout);

	return 0;
}

int xfer_vectors(CONST struct spi_slave *slave,
			struct spi_op vectors[], __SIZE_TYPE__ count)
{
	return spi_flash_vector_helper(slave, vectors, count, spi_ctrlr_xfer);
}

int protect_a_range(UINT32 value)
{
	UINT32 reg32;
	UINT8 n;

	/* find a free protection register */
	for (n = 0; n < MAX_ROM_PROTECT_RANGES; n++) {
		reg32 = PciRead32(
			PCI_LIB_ADDRESS(PCU_BUS, PCU_DEV, LPC_FUNC, ROM_PROTECT_RANGE_REG(n)));
		if (!reg32)
			break;
	}
	if (n == MAX_ROM_PROTECT_RANGES)
		return -1; /* no free range */

	PciWrite32(
		PCI_LIB_ADDRESS(PCU_BUS, PCU_DEV, LPC_FUNC, ROM_PROTECT_RANGE_REG(n)),
		value);
	return 0;
}

CONST struct spi_ctrlr fch_spi_flash_ctrlr = {
	.xfer = spi_ctrlr_xfer,
	.xfer_vector = xfer_vectors,
	.max_xfer_size = SPI_FIFO_DEPTH,
	.flags = SPI_CNTRLR_DEDUCT_CMD_LEN | SPI_CNTRLR_DEDUCT_OPCODE_LEN,
};

CONST struct spi_ctrlr_buses spi_ctrlr_bus_map[] = {
	{
		.ctrlr = &fch_spi_flash_ctrlr,
		.bus_start = 0,
		.bus_end = 0,
	},
};

CONST __SIZE_TYPE__ spi_ctrlr_bus_map_count = ARRAY_SIZE(spi_ctrlr_bus_map);
