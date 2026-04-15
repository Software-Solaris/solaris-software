idf_component_register(
    SRCS 
    "general.c"
    "gpio_int.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_spi esp_driver_gpio spp_wrapper
    PRIV_REQUIRES icm_driver pressureSensorDriver    
)
