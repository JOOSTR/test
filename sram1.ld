/* SRAM1 data section - placed in main SRAM with 32-byte alignment */
SECTION_DATA_PROLOGUE(.sram1_data,,)
{
    . = ALIGN(32);
    *(.sram1_data)
    *(.sram1_data.*)
    . = ALIGN(32);
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)