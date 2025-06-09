
idf_component_register(
    SRCS "general.c"
    INCLUDE_DIRS "."
    REQUIRES driver
    PRIV_REQUIRES icm_driver pressure_sensor_driver
)