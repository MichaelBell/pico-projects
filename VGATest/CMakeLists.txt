# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_COPY_TO_RAM 1)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(VGATest C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(VGATest VGATest.c image.c sdring.c )

set_source_files_properties(image.c PROPERTIES COMPILE_FLAGS "-masm-syntax-unified")

pico_set_program_name(VGATest "VGATest")
pico_set_program_version(VGATest "0.1")

pico_enable_stdio_uart(VGATest 0)
pico_enable_stdio_usb(VGATest 0)

pico_generate_pio_header(VGATest ${CMAKE_CURRENT_LIST_DIR}/vga.pio)

# Slow down the flash to allow a faster system clock
target_compile_definitions(VGATest PRIVATE PICO_FLASH_SPI_CLKDIV=4)
target_compile_definitions(bs2_default PRIVATE PICO_FLASH_SPI_CLKDIV=4)

# Add the standard library to the build
target_link_libraries(VGATest pico_stdlib pico_multicore)

# Add any user requested libraries
target_link_libraries(VGATest
        pico_sd_card
        hardware_dma
        hardware_pio
        hardware_interp
        )

pico_add_extra_outputs(VGATest)

