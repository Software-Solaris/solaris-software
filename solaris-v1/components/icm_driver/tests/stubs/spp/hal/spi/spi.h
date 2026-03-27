#ifndef STUB_SPI_H
#define STUB_SPI_H

#include "spp/core/types.h"
#include "spp/core/returntypes.h"

/* Declared only - mock implementation in test file */
retval_t SPP_HAL_SPI_Transmit(void *p_handle, spp_uint8_t *p_data, spp_uint8_t len);

#endif
