#include "unity.h"
#include "driver/spi_master.h"
#include "icm20948.h"   // donde está icm20948_init

TEST_CASE("ICM20948 init configura correctamente el bus y el dispositivo SPI", "[icm20948]")
{
    data_t dev; // tu struct con buscfg, devcfg, handle

    esp_err_t ret = icm20948_init(&dev);

    TEST_ASSERT_EQUAL(ESP_OK, ret);

    // Verificamos la configuración del bus
    TEST_ASSERT_EQUAL(PIN_NUM_CIPO, dev.buscfg.miso_io_num);
    TEST_ASSERT_EQUAL(PIN_NUM_COPI, dev.buscfg.mosi_io_num);
    TEST_ASSERT_EQUAL(PIN_NUM_CLK, dev.buscfg.sclk_io_num);
    TEST_ASSERT_EQUAL(-1, dev.buscfg.quadwp_io_num);
    TEST_ASSERT_EQUAL(4096, dev.buscfg.max_transfer_sz);

    // Verificamos la configuración del dispositivo
    TEST_ASSERT_EQUAL(100000, dev.devcfg.clock_speed_hz);
    TEST_ASSERT_EQUAL(3, dev.devcfg.mode);
    TEST_ASSERT_EQUAL(PIN_NUM_CS, dev.devcfg.spics_io_num);
    TEST_ASSERT_EQUAL(20, dev.devcfg.queue_size);

    // Verificamos que el handle se asignó
    TEST_ASSERT_NOT_NULL(dev.handle);
}
