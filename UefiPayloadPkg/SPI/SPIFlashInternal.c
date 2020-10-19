/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <Include/Library/DebugLib.h>
#include <Include/Library/TimerLib.h>
#include <Include/PiDxe.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include "FvbSPI.h"
#include "GenericSPI.h"
#include "SPIFlashInternal.h"

static VOID spi_flash_addr(UINT32 addr, UINT8 *cmd)
{
	/* cmd[0] is actual command */
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr >> 0;
}

static int do_spi_flash_cmd(const struct spi_slave *spi, const VOID *dout,
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

static int do_dual_read_cmd(const struct spi_slave *spi, const VOID *dout,
			    __SIZE_TYPE__ bytes_out, VOID *din, __SIZE_TYPE__ bytes_in)
{
	int ret;

	/*
	 * spi_xfer_vector() will automatically fall back to .xfer() if
	 * .xfer_vector() is unimplemented. So using vector API here is more
	 * flexible, even though a controller that implements .xfer_vector()
	 * and (the non-vector based) .xfer_dual() but not .xfer() would be
	 * pretty odd.
	 */
	struct spi_op vector = { .dout = dout, .bytesout = bytes_out,
				 .din = NULL, .bytesin = 0 };

	ret = spi_claim_bus(spi);
	if (ret)
		return ret;

	ret = spi_xfer_vector(spi, &vector, 1);

	if (!ret)
		ret = spi->ctrlr->xfer_dual(spi, NULL, 0, din, bytes_in);

	spi_release_bus(spi);
	return ret;
}

int spi_flash_cmd(const struct spi_slave *spi, UINT8 cmd, VOID *response, __SIZE_TYPE__ len)
{
	int ret = do_spi_flash_cmd(spi, &cmd, sizeof(cmd), response, len);
	if (ret)
		DEBUG((EFI_D_INFO, "%a SF: Failed to send command %02x: %d\n", __FUNCTION__, cmd, ret));

	return ret;
}

/* TODO: This code is quite possibly broken and overflowing stacks. Fix ASAP! */
#pragma GCC diagnostic push
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wstack-usage="
#endif
#pragma GCC diagnostic ignored "-Wvla"
int spi_flash_cmd_write(const struct spi_slave *spi, const UINT8 *cmd,
			__SIZE_TYPE__ cmd_len, const VOID *data, __SIZE_TYPE__ data_len)
{
	int ret;
	UINT8 buff[cmd_len + data_len];
	InternalMemCopyMem(buff, cmd, cmd_len);
	InternalMemCopyMem(buff + cmd_len, data, data_len);

	ret = do_spi_flash_cmd(spi, buff, cmd_len + data_len, NULL, 0);
	if (ret) {
		DEBUG((EFI_D_INFO, "%a SF: Failed to send write command (%zu bytes): %d\n",
				__FUNCTION__, data_len, ret));
	}

	return ret;
}
#pragma GCC diagnostic pop

/* Perform the read operation honoring spi controller fifo size, reissuing
 * the read command until the full request completed. */
int spi_flash_cmd_read(const struct spi_flash *flash, UINT32 offset,
				  __SIZE_TYPE__ len, VOID *buf)
{
	UINT8 cmd[5];
	int ret, cmd_len;
	int (*do_cmd)(const struct spi_slave *spi, const VOID *din,
		      __SIZE_TYPE__ in_bytes, VOID *out, __SIZE_TYPE__ out_bytes);
#ifdef SPI_FLASH_NO_FAST_READ
		cmd_len = 4;
		cmd[0] = CMD_READ_ARRAY_SLOW;
		do_cmd = do_spi_flash_cmd;
#else
	if (flash->flags.dual_spi && flash->spi.ctrlr->xfer_dual) {
		cmd_len = 5;
		cmd[0] = CMD_READ_FAST_DUAL_OUTPUT;
		cmd[4] = 0;
		do_cmd = do_dual_read_cmd;
	} else {
		cmd_len = 5;
		cmd[0] = CMD_READ_ARRAY_FAST;
		cmd[4] = 0;
		do_cmd = do_spi_flash_cmd;
	}
#endif

	UINT8 *data = buf;
	while (len) {
		__SIZE_TYPE__ xfer_len = spi_crop_chunk(&flash->spi, cmd_len, len);
		spi_flash_addr(offset, cmd);
		ret = do_cmd(&flash->spi, cmd, cmd_len, data, xfer_len);
		if (ret) {
			DEBUG((EFI_D_INFO,
						"%a SF: Failed to send read command %#.2x(%#x, %#zx): %d\n",
			       __FUNCTION__, cmd[0], offset, xfer_len, ret));
			return ret;
		}
		offset += xfer_len;
		data += xfer_len;
		len -= xfer_len;
	}

	return 0;
}

int spi_flash_cmd_poll_bit(const struct spi_flash *flash, unsigned long timeout,
			   UINT8 cmd, UINT8 poll_bit)
{
	const struct spi_slave *spi = &flash->spi;
	int ret;
	UINT8 status;

	CONST UINT64 nanoToMiliDivider = 1000000;
	CONST UINT64 startTimeMilisec =
		GetTimeInNanoSecond(GetPerformanceCounter()) / nanoToMiliDivider;
	CONST UINT64 endTimeMilisec = startTimeMilisec + timeout;
	UINT64 timeNow = startTimeMilisec;

	do {
		ret = do_spi_flash_cmd(spi, &cmd, 1, &status, 1);
		if (ret)
			return -1;
		if ((status & poll_bit) == 0)
			return 0;
		timeNow = GetTimeInNanoSecond(GetPerformanceCounter()) / nanoToMiliDivider;
	} while (
				timeNow<startTimeMilisec
		|| (timeNow>startTimeMilisec && timeNow>endTimeMilisec));

	DEBUG((EFI_D_INFO, "%a SF: timeout at %ld msec\n", __FUNCTION__, timeout));
	return -1;
}

int spi_flash_cmd_wait_ready(const struct spi_flash *flash,
			unsigned long timeout)
{
	return spi_flash_cmd_poll_bit(flash, timeout,
		CMD_READ_STATUS, STATUS_WIP);
}

int spi_flash_cmd_erase(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len)
{
	UINT32 start, end, erase_size;
	int ret = -1;
	UINT8 cmd[4];

	erase_size = flash->sector_size;
	if (offset % erase_size || len % erase_size) {
		DEBUG((EFI_D_INFO, "%a SF: Erase offset/length not multiple of erase size\n", __FUNCTION__));
		return -1;
	}
	if (len == 0) {
		DEBUG((EFI_D_INFO, "%a SF: Erase length cannot be 0\n", __FUNCTION__));
		return -1;
	}

	cmd[0] = flash->erase_cmd;
	start = offset;
	end = start + len;

	while (offset < end) {
		spi_flash_addr(offset, cmd);
		offset += erase_size;

#ifdef DEBUG_SPI_FLASH
		DEBUG((EFI_D_INFO, "%a SF: erase %2x %2x %2x %2x (%x)\n", __FUNCTION,
			cmd[0], cmd[1], cmd[2], cmd[3], offset));
#endif
		ret = spi_flash_cmd(&flash->spi, CMD_WRITE_ENABLE, NULL, 0);
		if (ret)
			goto out;

		ret = spi_flash_cmd_write(&flash->spi, cmd, sizeof(cmd), NULL, 0);
		if (ret)
			goto out;

		ret = spi_flash_cmd_wait_ready(flash,
				SPI_FLASH_PAGE_ERASE_TIMEOUT_MS);
		if (ret)
			goto out;
	}

	DEBUG((EFI_D_INFO, "%a SF: Successfully erased %zu bytes @ %#x\n",
		__FUNCTION__, len, start));

out:
	return ret;
}

int spi_flash_cmd_status(const struct spi_flash *flash, UINT8 *reg)
{
	return spi_flash_cmd(&flash->spi, flash->status_cmd, reg, sizeof(*reg));
}

int spi_flash_cmd_write_page_program(const struct spi_flash *flash, UINT32 offset,
				__SIZE_TYPE__ len, const VOID *buf)
{
	unsigned long byte_addr;
	unsigned long page_size;
	__SIZE_TYPE__ chunk_len;
	__SIZE_TYPE__ actual;
	int ret = 0;
	UINT8 cmd[4];

	page_size = flash->page_size;
	cmd[0] = flash->pp_cmd;

	for (actual = 0; actual < len; actual += chunk_len) {
		byte_addr = offset % page_size;
		chunk_len = MIN(len - actual, page_size - byte_addr);
		chunk_len = spi_crop_chunk(&flash->spi, sizeof(cmd), chunk_len);

		spi_flash_addr(offset, cmd);
#ifdef DEBUG_SPI_FLASH
			DEBUG((EFI_D_INFO, "%a PP: %p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %zu\n",
				__FUNCTION__, buf + actual, cmd[0], cmd[1], cmd[2], cmd[3],
				chunk_len));
#endif

		ret = spi_flash_cmd(&flash->spi, flash->wren_cmd, NULL, 0);
		if (ret < 0) {
			DEBUG((EFI_D_INFO, "%a SF: Enabling Write failed\n", __FUNCTION__));
			goto out;
		}

		ret = spi_flash_cmd_write(&flash->spi, cmd, sizeof(cmd),
				buf + actual, chunk_len);
		if (ret < 0) {
			DEBUG((EFI_D_INFO, "%a SF: Page Program failed\n", __FUNCTION__));
			goto out;
		}

		ret = spi_flash_cmd_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT_MS);
		if (ret)
			goto out;

		offset += chunk_len;
	}
	ret = 0;

out:
	return ret;
}

int spi_flash_read(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
		VOID *buf)
{
	return flash->ops->read(flash, offset, len, buf);
}

int spi_flash_write(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len,
		const VOID *buf)
{
	int ret;

	if (spi_flash_volatile_group_begin(flash))
		return -1;

	ret = flash->ops->write(flash, offset, len, buf);

	if (spi_flash_volatile_group_end(flash))
		return -1;

	return ret;
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

int spi_flash_status(const struct spi_flash *flash, UINT8 *reg)
{
	if (flash->ops->status)
		return flash->ops->status(flash, reg);

	return -1;
}

int spi_flash_is_write_protected(const struct spi_flash *flash,
				 const struct region *region)
{
	// struct region flash_region = { 0 };

	if (!flash || !region)
		return -1;

	// flash_region.size = flash->size;

	// if (!region_is_subregion(&flash_region, region))
	// 	return -1;

	if (!flash->prot_ops) {
		DEBUG((EFI_D_INFO, "%a SPI: Write-protection gathering not "
		       "implemented for this vendor.\n", __FUNCTION__));
		return -1;
	}

	return flash->prot_ops->get_write(flash, region);
}

int spi_flash_set_write_protected(const struct spi_flash *flash,
				  const struct region *region,
				  const enum spi_flash_status_reg_lockdown mode)
{
	// struct region flash_region = { 0 };
	int ret;

	if (!flash)
		return -1;

	// flash_region.size = flash->size;

	// if (!region_is_subregion(&flash_region, region))
	// 	return -1;

	if (!flash->prot_ops) {
		DEBUG((EFI_D_INFO, "%a SPI: Setting write-protection is not "
		       "implemented for this vendor.\n", __FUNCTION__));
		return -1;
	}

	ret = flash->prot_ops->set_write(flash, region, mode);

	if (ret == 0 && mode != SPI_WRITE_PROTECTION_PRESERVE) {
		DEBUG((EFI_D_INFO, "%a SPI: SREG lock-down was set to ", __FUNCTION__));
		switch (mode) {
		case SPI_WRITE_PROTECTION_NONE:
			DEBUG((EFI_D_INFO, "%a NEVER\n", __FUNCTION__));
		break;
		case SPI_WRITE_PROTECTION_PIN:
			DEBUG((EFI_D_INFO, "%a WP\n", __FUNCTION__));
		break;
		case SPI_WRITE_PROTECTION_REBOOT:
			DEBUG((EFI_D_INFO, "%a REBOOT\n", __FUNCTION__));
		break;
		case SPI_WRITE_PROTECTION_PERMANENT:
			DEBUG((EFI_D_INFO, "%a PERMANENT\n", __FUNCTION__));
		break;
		default:
			DEBUG((EFI_D_INFO, "%a UNKNOWN\n", __FUNCTION__));
		break;
		}
	}

	return ret;
}
#ifdef SPI_FLASH_HAS_VOLATILE_GROUP
static UINT32 volatile_group_count;

int spi_flash_volatile_group_begin(const struct spi_flash *flash)
{
	int ret = 0;
	UINT32 count;
	count = volatile_group_count;
	if (count == 0)
		ret = chipset_volatile_group_begin(flash);

	count++;
	volatile_group_count = count;
	return ret;
}

int spi_flash_volatile_group_end(const struct spi_flash *flash)
{
	int ret = 0;
	UINT32 count;
	count = volatile_group_count;
	count--;
	volatile_group_count = count;

	if (count == 0)
		ret = chipset_volatile_group_end(flash);

	return ret;
}
#else

int spi_flash_volatile_group_begin(const struct spi_flash *flash)
{
	int ret = 0;
	return ret;
}

int spi_flash_volatile_group_end(const struct spi_flash *flash)
{
	int ret = 0;
	return ret;
}
#endif /* SPI_FLASH_HAS_VOLATILE_GROUP */

int spi_flash_ctrlr_protect_region(const struct spi_flash *flash,
				   const struct region *region,
				   const enum ctrlr_prot_type type)
{
	const struct spi_ctrlr *ctrlr;
	// struct region flash_region = { 0 };

	if (!flash)
		return -1;

	// flash_region.size = flash->size;

	// if (!region_is_subregion(&flash_region, region))
	// 	return -1;

	ctrlr = flash->spi.ctrlr;

	if (!ctrlr)
		return -1;

	if (ctrlr->flash_protect)
		return ctrlr->flash_protect(flash, region, type);

	return -1;
}

// int region_is_subregion(const struct region *p, const struct region *c)
// {
// 	if (region_offset(c) < region_offset(p))
// 		return 0;

// 	if (region_end(c) > region_end(p))
// 		return 0;

// 	if (region_end(c) < region_offset(c))
// 		return 0;

// 	return 1;
// }

// inline __SIZE_TYPE__ region_offset(const struct region *r)
// {
// 	return r->offset;
// }

// inline __SIZE_TYPE__ region_end(const struct region *r)
// {
// 	return region_offset(r) + region_sz(r);
// }

// inline __SIZE_TYPE__ region_sz(const struct region *r)
// {
// 	return r->size;
// }

int chipset_volatile_group_begin(const struct spi_flash *flash)
{
	return 0;
	// if (!CONFIG(HUDSON_IMC_FWM))
	// 	return 0;

	// ImcSleep(NULL);
	// return 0;
}

int chipset_volatile_group_end(const struct spi_flash *flash)
{
	return 0;
	// if (!CONFIG(HUDSON_IMC_FWM))
	// 	return 0;

	// ImcWakeup(NULL);
	// return 0;
}

int spi_flash_vector_helper(const struct spi_slave *slave,
	struct spi_op vectors[], __SIZE_TYPE__ count,
	int (*func)(const struct spi_slave *slave, const VOID *dout,
		    __SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin))
{
	int ret;
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

const struct spi_flash_ops_descriptor spi_flash_pp_0x20_sector_desc = {
	.erase_cmd = 0x20, /* Sector Erase */
	.status_cmd = 0x05, /* Read Status */
	.pp_cmd = 0x02, /* Page Program */
	.wren_cmd = 0x06, /* Write Enable */
	.ops = {
		.read = spi_flash_cmd_read,
		.write = spi_flash_cmd_write_page_program,
		.erase = spi_flash_cmd_erase,
		.status = spi_flash_cmd_status,
	},
};

const struct spi_flash_ops_descriptor spi_flash_pp_0xd8_sector_desc = {
	.erase_cmd = 0xd8, /* Sector Erase */
	.status_cmd = 0x05, /* Read Status */
	.pp_cmd = 0x02, /* Page Program */
	.wren_cmd = 0x06, /* Write Enable */
	.ops = {
		.read = spi_flash_cmd_read,
		.write = spi_flash_cmd_write_page_program,
		.erase = spi_flash_cmd_erase,
		.status = spi_flash_cmd_status,
	},
};
