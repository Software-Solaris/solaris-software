/*
 * stubs/macros.h — Host-test stub for components/general/macros.h.
 *
 * Provides the SPI operation-type constants used by icm20948.c.
 * The ESP-IDF SPI driver types are not needed in host unit tests.
 */
#ifndef STUB_MACROS_H
#define STUB_MACROS_H

/** @brief SPI read operation bit (MSB = 1). */
#define K_READ_OP  0x80U

/** @brief SPI write operation bit (MSB = 0). */
#define K_WRITE_OP 0x00U

#endif /* STUB_MACROS_H */
