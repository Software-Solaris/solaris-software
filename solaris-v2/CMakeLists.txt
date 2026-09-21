cmake_minimum_required(VERSION 3.16)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(EXTRA_COMPONENT_DIRS "compiler")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(solaris-v2)
