/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <Include/PiDxe.h>
#include "SPI.h"

/*
 * SPI flash internal definitions
 */

#ifndef SPI_FLASH_INTERNAL_H
#define SPI_FLASH_INTERNAL_H

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

/* Send a single-byte command to the device and read the response */
int spi_flash_cmd(CONST struct spi_slave *spi, UINT8 cmd, VOID *response, __SIZE_TYPE__ len);

/*
 * Send a multi-byte command to the device followed by (optional)
 * data. Used for programming the flash array, etc.
 */
int spi_flash_cmd_write(CONST struct spi_slave *spi, CONST UINT8 *cmd,
			__SIZE_TYPE__ cmd_len, CONST VOID *data, __SIZE_TYPE__ data_len);

/* Send a command to the device and wait for some bit to clear itself. */
int spi_flash_cmd_poll_bit(CONST struct spi_flash *flash, unsigned long timeout,
			   UINT8 cmd, UINT8 poll_bit);

/*
 * Send the read status command to the device and wait for the wip
 * (write-in-progress) bit to clear itself.
 */
int spi_flash_cmd_wait_ready(CONST struct spi_flash *flash, unsigned long timeout);

/* Erase sectors. */
int spi_flash_cmd_erase(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len);

/* Read status register. */
int spi_flash_cmd_status(CONST struct spi_flash *flash, UINT8 *reg);

/* Write to flash utilizing page program semantics. */
int spi_flash_cmd_write_page_program(CONST struct spi_flash *flash, UINT32 offset,
				__SIZE_TYPE__ len, CONST VOID *buf);

/* Read len bytes into buf at offset. */
int spi_flash_cmd_read(CONST struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len, VOID *buf);

/* Release from deep sleep an provide alternative rdid information. */
int stmicro_release_deep_sleep_identify(CONST struct spi_slave *spi, UINT8 *idcode);

struct spi_flash_part_id {
	/* rdid command constructs 2x 16-bit id using the following method
	 * for matching after reading 5 bytes (1st byte is manuf id):
	 *    id[0] = (id[1] << 8) | id[2]
	 *    id[1] = (id[3] << 8) | id[4]
	 */
	UINT16 id[2];
	/* Log based 2 total number of sectors. */
	UINT16 nr_sectors_shift: 4;
	UINT16 fast_read_dual_output_support : 1;
	UINT16 _reserved_for_flags: 3;
	/* Block protection. Currently used by Winbond. */
	UINT16 protection_granularity_shift : 5;
	UINT16 bp_bits : 3;
};

struct spi_flash_ops_descriptor {
	UINT8 erase_cmd; /* Sector Erase */
	UINT8 status_cmd; /* Read Status Register */
	UINT8 pp_cmd; /* Page program command, if supported. */
	UINT8 wren_cmd; /* Write Enable command. */
	struct spi_flash_ops ops;
};

/* Vendor info represents a common set of organization and commands by a given
 * vendor. One can implement multiple sets from a single vendor by having
 * separate objects. */
struct spi_flash_vendor_info {
	UINT8 id;
	UINT8 page_size_shift : 4; /* if page programming oriented. */
	/* Log based 2 sector size */
	UINT8 sector_size_kib_shift : 4;
	UINT16 nr_part_ids;
	CONST struct spi_flash_part_id *ids;
	UINT16 match_id_mask[2]; /* matching bytes of the id for this set*/
	CONST struct spi_flash_ops_descriptor *desc;
	CONST struct spi_flash_protection_ops *prot_ops;
	/* Returns 0 on success. !0 otherwise. */
	int (*after_probe)(CONST struct spi_flash *flash);
};

/* Manufacturer-specific probe information */
extern CONST struct spi_flash_vendor_info spi_flash_adesto_vi;
extern CONST struct spi_flash_vendor_info spi_flash_amic_vi;
extern CONST struct spi_flash_vendor_info spi_flash_atmel_vi;
extern CONST struct spi_flash_vendor_info spi_flash_eon_vi;
extern CONST struct spi_flash_vendor_info spi_flash_gigadevice_vi;
extern CONST struct spi_flash_vendor_info spi_flash_macronix_vi;
/* Probing order matters between the spansion sequence. */
extern CONST struct spi_flash_vendor_info spi_flash_spansion_ext1_vi;
extern CONST struct spi_flash_vendor_info spi_flash_spansion_ext2_vi;
extern CONST struct spi_flash_vendor_info spi_flash_spansion_vi;
extern CONST struct spi_flash_vendor_info spi_flash_sst_ai_vi;
extern CONST struct spi_flash_vendor_info spi_flash_sst_vi;
extern CONST struct spi_flash_vendor_info spi_flash_stmicro1_vi;
extern CONST struct spi_flash_vendor_info spi_flash_stmicro2_vi;
extern CONST struct spi_flash_vendor_info spi_flash_stmicro3_vi;
extern CONST struct spi_flash_vendor_info spi_flash_stmicro4_vi;
extern CONST struct spi_flash_vendor_info spi_flash_winbond_vi;

/* Page Programming Command Set with 0x20 Sector Erase command. */
extern CONST struct spi_flash_ops_descriptor spi_flash_pp_0x20_sector_desc;
/* Page Programming Command Set with 0xd8 Sector Erase command. */
extern CONST struct spi_flash_ops_descriptor spi_flash_pp_0xd8_sector_desc;

#endif /* SPI_FLASH_INTERNAL_H */
