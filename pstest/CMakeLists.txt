# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD pimoroni_picosystem)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(pstest C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pstest pstest.c st7789_lcd.c )

pico_set_program_name(pstest "pstest")
pico_set_program_version(pstest "0.1")

pico_enable_stdio_uart(pstest 1)
pico_enable_stdio_usb(pstest 0)

pico_generate_pio_header(pstest ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

# Add the standard library to the build
target_link_libraries(pstest pico_stdlib)

# Add any user requested libraries
target_link_libraries(pstest
        hardware_dma
        hardware_pio
        )

pico_add_extra_outputs(pstest)

