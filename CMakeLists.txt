cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_msc_sd)

#zephyr_linker_sources(DATA_SECTIONS ${CMAKE_CURRENT_SOURCE_DIR}/sram1.ld)

target_sources(app PRIVATE 
    src/main.c
    src/idma_buffer.c
)

