
/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef SOUTHBRIDGE_INTEL_SPI_H
#define SOUTHBRIDGE_INTEL_SPI_H

void spi_finalize_ops(void);
void intel_southbridge_override_spi(struct intel_swseq_spi_config *spi_config);

#endif
