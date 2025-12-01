#include "unity.h"
#include "bmp390.h"

TEST_CASE("SPP_HAL_SPI_BusInit strong implementation funciona", "[spi]")
{
    retval_t ret = SPP_HAL_SPI_BusInit();
    TEST_ASSERT_EQUAL(SPP_OK, ret);
}