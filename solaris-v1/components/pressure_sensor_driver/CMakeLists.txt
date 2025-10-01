idf_component_register(
    SRCS "bmp390.c"
    INCLUDE_DIRS "include"
    REQUIRES general esp_driver_spi spp_port_wrapper spp_wrapper
    PRIV_REQUIRES esp_driver_gpio
)
