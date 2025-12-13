cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_msc_sd)

# Enable SRAM1 section for DMA-compatible buffers (required for USB MSC on STM32U5)
# Use RAM_SECTIONS for uninitialized data placed directly in RAM (not copied from Flash)
zephyr_linker_sources(RAM_SECTIONS ${CMAKE_CURRENT_SOURCE_DIR}/sram1.ld)

target_sources(app PRIVATE 
    src/main.c
    src/idma_buffer.c
)

