# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(sdgif C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(sdgif sdgif.c gif.c st7789_lcd.c )

pico_set_program_name(sdgif "sdgif")
pico_set_program_version(sdgif "0.1")

pico_enable_stdio_uart(sdgif 1)
pico_enable_stdio_usb(sdgif 0)

pico_generate_pio_header(sdgif ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

# Add the standard library to the build
target_link_libraries(sdgif pico_stdlib)

# Add any user requested libraries
target_link_libraries(sdgif
        hardware_dma
        hardware_pio
        pico_sd_card
        )

pico_add_extra_outputs(sdgif)

