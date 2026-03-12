#ifndef BMP_SERVICE_H
#define BMP_SERVICE_H

#include "core/returntypes.h"

retval_t BMP_ServiceInit(void* p_spi_bmp);
retval_t BMP_ServiceStart(void);

#endif