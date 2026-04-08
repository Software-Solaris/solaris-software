cmake_minimum_required(VERSION 3.16)
project(icm_driver_bdd_tests C)

find_library(CGREEN_LIB cgreen REQUIRED)
find_path(CGREEN_INCLUDE cgreen/cgreen.h REQUIRED)

set(STUB_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
set(ICM_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ----------------------------------------------------------------
# Helper: registers one BDD test executable.
# Each .c file includes icm20948.c directly, so every binary gets
# its own copy of both the driver and the mock infrastructure.
# ----------------------------------------------------------------
function(add_bdd_suite name)
    add_executable(${name} ${name}.c)
    target_include_directories(${name} PRIVATE
        ${STUB_DIR}             # stubs first — override real ESP-IDF headers
        ${ICM_SRC_DIR}/include  # icm20948.h
        ${ICM_SRC_DIR}          # icm20948_img.dmp3a.h, icm20948.c
        ${CGREEN_INCLUDE}
    )
    target_link_libraries(${name} ${CGREEN_LIB} m)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# ----------------------------------------------------------------
# BDD test suites — one executable per feature area
# ----------------------------------------------------------------
add_bdd_suite(test_spi_register_access)     # writeReg, readReg
add_bdd_suite(test_register_bank_selection) # setBank
add_bdd_suite(test_dmp_memory_write)        # dmpWriteBytes, dmpWrite32, dmpWrite16
add_bdd_suite(test_dmp_output_config)       # dmpWriteOutputConfig
add_bdd_suite(test_power_management)        # lpWakeCycle, resetFifo
add_bdd_suite(test_gyro_calibration_math)   # calcGyroSf
add_bdd_suite(test_firmware_load)           # loadDmp
add_bdd_suite(test_fifo_data_parsing)       # checkFifoData
add_bdd_suite(test_dmp_init_sequence)       # configDmpInit

enable_testing()
