idf_component_register(
    SRCS "bmp390.c"
    INCLUDE_DIRS "include"
    REQUIRES general spp_port_wrapper spp_wrapper
)
