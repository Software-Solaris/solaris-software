idf_component_register(
    SRCS "dataloggerSD.c"
    INCLUDE_DIRS "include"
    REQUIRES general esp_driver_spi spp_port_wrapper spp_wrapper
)
