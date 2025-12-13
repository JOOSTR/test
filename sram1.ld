SECTION_DATA_PROLOGUE(.sram1_data,(NOLOAD),)
{
    . = ALIGN(32);
    *(.sram1_data)
    *(.sram1_data.*)
} GROUP_DATA_LINK_IN(SRAM1, SRAM1)