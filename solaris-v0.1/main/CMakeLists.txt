# Componente “main”
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES pressure_sensor_driver
)
