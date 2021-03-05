# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(sdaudio C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(sdaudio sdaudio.c sdring.c)

pico_set_program_name(sdaudio "sdaudio")
pico_set_program_version(sdaudio "0.1")

pico_enable_stdio_uart(sdaudio 0)
pico_enable_stdio_usb(sdaudio 0)

# Add the standard library to the build
target_link_libraries(sdaudio pico_stdlib)

# Add any user requested libraries
target_link_libraries(sdaudio
        hardware_dma
        pico_sd_card
        pico_audio_i2s
        )

pico_add_extra_outputs(sdaudio)

