# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pi/pico/pico-sdk")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(lasertest C CXX)

# Initialise the Pico SDK
pico_sdk_init()

add_subdirectory(VL53L1X)

# Add executable. Default name is the project name, version 0.1

add_executable(lasertest lasertest.c )

pico_set_program_name(lasertest "lasertest")
pico_set_program_version(lasertest "0.1")

pico_enable_stdio_uart(lasertest 0)
pico_enable_stdio_usb(lasertest 1)

# Add the standard library to the build
target_link_libraries(lasertest pico_stdlib)

# Add any user requested libraries
target_link_libraries(lasertest
        hardware_i2c
        vl53l1x
        )

pico_add_extra_outputs(lasertest)

