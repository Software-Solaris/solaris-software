# solaris-v1/components/pressure_sensor_driver/CMakeLists.txt
idf_component_register(
    SRCS "bmp390.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver       # Porque utilizamos driver/spi_master.h
        freertos     # Si usamos includes de FreeRTOS, opcional
)
