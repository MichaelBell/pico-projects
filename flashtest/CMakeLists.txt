# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pi/pico/pico-sdk")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(flashtest C CXX)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(flashtest flashtest.c )

pico_set_program_name(flashtest "flashtest")
pico_set_program_version(flashtest "0.1")

pico_enable_stdio_uart(flashtest 1)
pico_enable_stdio_usb(flashtest 0)

# Add the standard library to the build
target_link_libraries(flashtest pico_stdlib hardware_dma)

pico_add_extra_outputs(flashtest)

