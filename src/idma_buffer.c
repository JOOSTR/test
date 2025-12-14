#include <zephyr/kernel.h>

__attribute__((section(".sram1_data")))
__aligned(32)
uint8_t idma_bounce_buffer[4096];