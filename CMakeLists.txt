cmake_minimum_required(VERSION 3.0)

set(application_name "owly")
project(${application_name} C CXX ASM)

IF(NOT CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR "Cross compiling only. Please use -DCMAKE_TOOLCHAIN_FILE=/PATH/TO/TOOLCHAIN_FILE .")
ENDIF(NOT CMAKE_CROSSCOMPILING)

# set the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# STM32-specific settings for SPL and CMSIS
set(DEVICE_FAMILY STM32F40_41xxx)
# Speed of external resonator
set(HSE_VALUE 8000000)

# setup libopencm3
add_definitions(-DSTM32F4)
set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/libopencm3)
add_custom_target(libopencm3 make WORKING_DIRECTORY ${LIBOPENCM3_DIR})
link_directories(${LIBOPENCM3_DIR}/lib)
include_directories(${LIBOPENCM3_DIR}/include)
include_directories(include)

# linker file, for STM32F4 discovery board
get_filename_component(LINKER_SCRIPT cmake/stm32f4-discovery.ld ABSOLUTE)

# add libraries
# add_subdirectory(lib)

# add main project
add_subdirectory(src)
