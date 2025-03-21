#include "unity.h"
#include "driver/spi_master.h"
#include "imu_driver.h" 

#define PIN_NUM_CIPO 19
#define PIN_NUM_COPI 23
#define PIN_NUM_CLK  18

extern spi_bus_config_t buscfg;

TEST_CASE("Prueba de inicialización del IMU SPI", "[imu_driver]")
{
    imu_spi_init();

    TEST_ASSERT_EQUAL(PIN_NUM_COPI, buscfg.COPI_io_num);
    TEST_ASSERT_EQUAL(PIN_NUM_CIPO, buscfg.CIPO_io_num);
    TEST_ASSERT_EQUAL(PIN_NUM_CLK, buscfg.sclk_io_num);
    TEST_ASSERT_EQUAL(-1, buscfg.quadwp_io_num);
    TEST_ASSERT_EQUAL(4096, buscfg.max_transfer_sz);
}

