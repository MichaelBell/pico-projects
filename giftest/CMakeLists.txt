# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pi/pico/pico-sdk")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(giftest C CXX)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(giftest giftest.c st7789_lcd.c gif.c)

pico_set_program_name(giftest "giftest")
pico_set_program_version(giftest "0.1")

pico_enable_stdio_uart(giftest 1)
pico_enable_stdio_usb(giftest 0)

pico_generate_pio_header(giftest ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

# Add the standard library to the build
target_link_libraries(giftest pico_stdlib)

# Add any user requested libraries
target_link_libraries(giftest
        hardware_pio
        hardware_dma
        )

pico_add_extra_outputs(giftest)

