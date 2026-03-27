cmake_minimum_required(VERSION 3.16)
project(icm_driver_tests C)

find_library(CGREEN_LIB cgreen REQUIRED)
find_path(CGREEN_INCLUDE cgreen/cgreen.h REQUIRED)

set(STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
set(ICM_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(test_icm20948 test_icm20948.c)

target_include_directories(test_icm20948 PRIVATE
    ${STUB_DIR}             # Stubs first — override real headers
    ${ICM_SRC_DIR}/include  # icm20948.h
    ${ICM_SRC_DIR}          # icm20948_img.dmp3a.h
    ${CGREEN_INCLUDE}
)

target_link_libraries(test_icm20948 ${CGREEN_LIB} m)

enable_testing()
add_test(NAME icm20948_tests COMMAND test_icm20948)
