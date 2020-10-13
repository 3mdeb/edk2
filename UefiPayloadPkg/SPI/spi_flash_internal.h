/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * SPI flash internal definitions
 */

#ifndef SPI_FLASH_INTERNAL_H
#define SPI_FLASH_INTERNAL_H

#include <Include/PiDxe.h>
#include "SPI_fvb.h"

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

/* Common register masks */
#define REG_BUSY_MASK 											0x01
#define REG_WRITE_ENABLE_MASK 							0x02
#define REG_BLOCK_PROTECT_0_MASK 						0x04
#define REG_BLOCK_PROTECT_1_MASK 						0x08
#define REG_BLOCK_PROTECT_2_MASK 						0x10
#define REG_TOP_BOTTOM_PROTECTION_MASK 			0x20
#define REG_BLOCK_PROTECTION_MASK 					0x40
#define REG_STATUS_REGISTER_PROTECTION_MASK 0x80


/* Send a single-byte command to the device and read the response */
int spi_flash_cmd(const struct spi_slave *spi, UINT8 cmd, VOID *response, __SIZE_TYPE__ len);

/*
 * Send a multi-byte command to the device followed by (optional)
 * data. Used for programming the flash array, etc.
 */
int spi_flash_cmd_write(const struct spi_slave *spi, const UINT8 *cmd,
			__SIZE_TYPE__ cmd_len, const VOID *data, __SIZE_TYPE__ data_len);

/* Send a command to the device and wait for some bit to clear itself. */
int spi_flash_cmd_poll_bit(const struct spi_flash *flash, unsigned long timeout,
			   UINT8 cmd, UINT8 poll_bit);

/*
 * Send the read status command to the device and wait for the wip
 * (write-in-progress) bit to clear itself.
 */
int spi_flash_cmd_wait_ready(const struct spi_flash *flash, unsigned long timeout);

/* Erase sectors. */
int spi_flash_cmd_erase(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len);

/* Read status register. */
int spi_flash_cmd_status(const struct spi_flash *flash, UINT8 *reg);

/* Write to flash utilizing page program semantics. */
int spi_flash_cmd_write_page_program(const struct spi_flash *flash, UINT32 offset,
				__SIZE_TYPE__ len, const VOID *buf);

/* Read len bytes into buf at offset. */
int spi_flash_cmd_read(const struct spi_flash *flash, UINT32 offset, __SIZE_TYPE__ len, VOID *buf);

/* Release from deep sleep an provide alternative rdid information. */
int stmicro_release_deep_sleep_identify(const struct spi_slave *spi, UINT8 *idcode);

int spi_flash_volatile_group_begin(const struct spi_flash *flash);
int spi_flash_volatile_group_end(const struct spi_flash *flash);
// int region_is_subregion(const struct region *p, const struct region *c);
int chipset_volatile_group_begin(const struct spi_flash *flash);
int chipset_volatile_group_end(const struct spi_flash *flash);
// inline __SIZE_TYPE__ region_offset(const struct region *r);
// inline __SIZE_TYPE__ region_end(const struct region *r);
// inline __SIZE_TYPE__ region_sz(const struct region *r);
int spi_flash_vector_helper(const struct spi_slave *slave,
	struct spi_op vectors[], __SIZE_TYPE__ count,
	int (*func)(const struct spi_slave *slave, const VOID *dout,
		    __SIZE_TYPE__ bytesout, VOID *din, __SIZE_TYPE__ bytesin));

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
	const struct spi_flash_part_id *ids;
	UINT16 match_id_mask[2]; /* matching bytes of the id for this set*/
	const struct spi_flash_ops_descriptor *desc;
	const struct spi_flash_protection_ops *prot_ops;
	/* Returns 0 on success. !0 otherwise. */
	int (*after_probe)(const struct spi_flash *flash);
};

/* Manufacturer-specific probe information */
extern const struct spi_flash_vendor_info spi_flash_adesto_vi;
extern const struct spi_flash_vendor_info spi_flash_amic_vi;
extern const struct spi_flash_vendor_info spi_flash_atmel_vi;
extern const struct spi_flash_vendor_info spi_flash_eon_vi;
extern const struct spi_flash_vendor_info spi_flash_gigadevice_vi;
extern const struct spi_flash_vendor_info spi_flash_macronix_vi;
/* Probing order matters between the spansion sequence. */
extern const struct spi_flash_vendor_info spi_flash_spansion_ext1_vi;
extern const struct spi_flash_vendor_info spi_flash_spansion_ext2_vi;
extern const struct spi_flash_vendor_info spi_flash_spansion_vi;
extern const struct spi_flash_vendor_info spi_flash_sst_ai_vi;
extern const struct spi_flash_vendor_info spi_flash_sst_vi;
extern const struct spi_flash_vendor_info spi_flash_stmicro1_vi;
extern const struct spi_flash_vendor_info spi_flash_stmicro2_vi;
extern const struct spi_flash_vendor_info spi_flash_stmicro3_vi;
extern const struct spi_flash_vendor_info spi_flash_stmicro4_vi;
extern const struct spi_flash_vendor_info spi_flash_winbond_vi;

/* Page Programming Command Set with 0x20 Sector Erase command. */
extern const struct spi_flash_ops_descriptor spi_flash_pp_0x20_sector_desc;
/* Page Programming Command Set with 0xd8 Sector Erase command. */
extern const struct spi_flash_ops_descriptor spi_flash_pp_0xd8_sector_desc;

#endif /* SPI_FLASH_INTERNAL_H */
