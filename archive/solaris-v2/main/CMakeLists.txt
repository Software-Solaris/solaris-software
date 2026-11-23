idf_component_register(
    SRCS "main.c" "app.c" "custom.c"
    INCLUDE_DIRS "."
    REQUIRES spp spp_ports
)
