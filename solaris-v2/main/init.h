/**
 * @file init.h
 * @brief System initialisation sequence for the Solaris firmware.
 *
 * Exposes a single entry-point that initialises the HAL, registers all
 * producer services with the pub-sub router, and starts the SPP core stack.
 * Call once from app_main() before entering the main loop.
 *
 * Naming conventions used in this file:
 * - Public functions:  SPP_MAIN_*()
 * - Pointer params:    p_*
 */

#ifndef SPP_MAIN_INIT_H
#define SPP_MAIN_INIT_H

#include "spp/core/returnTypes.h"

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */

/**
 * @brief  Initialise the full SPP stack: HAL, producers, and core.
 *
 * Calls SPP_HAL_init(), registers all producer contracts with the pub-sub
 * router, then calls SPP_CORE_init().
 *
 * @return K_SPP_OK on success, or the first error code encountered.
 */
SPP_RetVal_t SPP_MAIN_init(void);

#endif /* SPP_MAIN_INIT_H */
