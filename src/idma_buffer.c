#include <zephyr/kernel.h>

__aligned(32)
uint8_t idma_bounce_buffer[4096];