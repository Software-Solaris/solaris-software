idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES general icm_driver pressure_sensor_driver spp_wrapper spp_port_wrapper
)
