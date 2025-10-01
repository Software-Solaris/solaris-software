idf_component_register(
    SRCS 
    "general.c"
    "gpio_int.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_spi esp_driver_gpio
    PRIV_REQUIRES icm_driver pressure_sensor_driver    
)
