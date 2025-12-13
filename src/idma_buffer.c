#include <zephyr/kernel.h>

/* Place in SRAM1 section for STM32U5 IDMA compatibility */
__attribute__((section(".sram1_data")))
__aligned(32)
uint8_t idma_bounce_buffer[4096];