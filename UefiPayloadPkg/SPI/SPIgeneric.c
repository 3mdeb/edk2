/* SPDX-License-Identifier: GPL-2.0-or-later */

//#include <assert.h>
//#include <commonlib/helpers.h>
//#include <stddef.h>
//#include <string.h>
#include <stddef.h>
#include <string.h>
#include <Include/PiDxe.h>
#include "SPIgeneric.h"

UINT32 spi_claim_bus(CONST struct spi_slave *slave)
{
	CONST struct spi_ctrlr *ctrlr = slave->ctrlr;
	if (ctrlr && ctrlr->claim_bus)
		return ctrlr->claim_bus(slave);
	return 0;
}

VOID spi_release_bus(CONST struct spi_slave *slave)
{
	CONST struct spi_ctrlr *ctrlr = slave->ctrlr;
	if (ctrlr && ctrlr->release_bus)
		ctrlr->release_bus(slave);
}

static INT32 spi_xfer_single_op(CONST struct spi_slave *slave,
			struct spi_op *op)
{
	CONST struct spi_ctrlr *ctrlr = slave->ctrlr;
	INT32 ret;

	if (!ctrlr || !ctrlr->xfer)
		return -1;

	ret = ctrlr->xfer(slave, op->dout, op->bytesout, op->din, op->bytesin);
	if (ret)
		op->status = SPI_OP_FAILURE;
	else
		op->status = SPI_OP_SUCCESS;

	return ret;
}

static INT32 spi_xfer_vector_default(CONST struct spi_slave *slave,
				struct spi_op vectors[], __SIZE_TYPE__ count)
{
	__SIZE_TYPE__ i;
	INT32 ret;

	for (i = 0; i < count; i++) {
		ret = spi_xfer_single_op(slave, &vectors[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int spi_xfer_vector(const struct spi_slave *slave,
		struct spi_op vectors[], __SIZE_TYPE__ count)
{
	CONST struct spi_ctrlr *ctrlr = slave->ctrlr;

	if (ctrlr && ctrlr->xfer_vector)
		return ctrlr->xfer_vector(slave, vectors, count);

	return spi_xfer_vector_default(slave, vectors, count);
}

UINT32 spi_xfer(CONST struct spi_slave *slave, CONST void *dout, __SIZE_TYPE__ bytesout,
	     VOID *din, __SIZE_TYPE__ bytesin)
{
	CONST struct spi_ctrlr *ctrlr = slave->ctrlr;

	if (ctrlr && ctrlr->xfer)
		return ctrlr->xfer(slave, dout, bytesout, din, bytesin);

	return -1;
}

UINT32 spi_crop_chunk(CONST struct spi_slave *slave, UINT32 cmd_len,
			UINT32 buf_len)
{
	CONST struct spi_ctrlr *ctrlr = slave->ctrlr;
	UINT32 ctrlr_max;
	BOOLEAN deduct_cmd_len;
	BOOLEAN deduct_opcode_len;

	if (!ctrlr)
		return 0;

	deduct_cmd_len = !!(ctrlr->flags & SPI_CNTRLR_DEDUCT_CMD_LEN);
	deduct_opcode_len = !!(ctrlr->flags & SPI_CNTRLR_DEDUCT_OPCODE_LEN);
	ctrlr_max = ctrlr->max_xfer_size;

	//assert (ctrlr_max != 0);

	/* Assume opcode is always one byte and deduct it from the cmd_len
	   as the hardware has a separate register for the opcode. */
	if (deduct_opcode_len)
		cmd_len--;

	if (deduct_cmd_len && (ctrlr_max > cmd_len))
		ctrlr_max -= cmd_len;

	return MIN(ctrlr_max, buf_len);
}

// __attribute__((__weak__))
// VOID spi_init(VOID)
// {
// 	/* Default weak implementation - do nothing. */
// }

UINT32 spi_setup_slave(UINT32 bus, UINT32 cs, struct spi_slave *slave)
{
	__SIZE_TYPE__ i;

	memset(slave, 0, sizeof(*slave));

	for (i = 0; i < spi_ctrlr_bus_map_count; i++) {
		if ((spi_ctrlr_bus_map[i].bus_start <= bus) &&
		    (spi_ctrlr_bus_map[i].bus_end >= bus)) {
			slave->ctrlr = spi_ctrlr_bus_map[i].ctrlr;
			break;
		}
	}

	if (slave->ctrlr == NULL)
		return -1;

	slave->bus = bus;
	slave->cs = cs;

	if (slave->ctrlr->setup)
		return slave->ctrlr->setup(slave);

	return 0;
}
