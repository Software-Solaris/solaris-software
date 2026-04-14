idf_component_register(
    SRCS "kalmanFilter.c"
    INCLUDE_DIRS "include"
    REQUIRES general spp_port_wrapper spp_wrapper
)
