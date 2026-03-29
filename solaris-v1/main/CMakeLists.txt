idf_component_register(
    SRCS "main.c" "bmpService.c"
    INCLUDE_DIRS "."
    REQUIRES general icm_driver pressureSensoreDriver datalogger_driver spp_wrapper spp_port_wrapper
)