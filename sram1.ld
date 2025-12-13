/* SRAM1 noinit section - for DMA buffers that don't need initialization */
SECTION_DATA_PROLOGUE(.sram1_data,(NOLOAD),)
{
    . = ALIGN(32);
    *(.sram1_data)
    *(.sram1_data.*)
    . = ALIGN(32);
} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)