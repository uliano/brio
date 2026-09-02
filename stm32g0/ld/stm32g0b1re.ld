/* ============================================================================
 * stm32g0b1re.ld - hand-written linker script for the STM32G0B1RE
 * (512 KB flash at 0x0800_0000, 144 KB SRAM at 0x2000_0000 - RM0444
 * table 2; the flash size register at 0x1FFF75E0 reads 0x200 KB on the
 * bench chip, checked over SWD before this file was written).
 *
 * Written from scratch, the samc script's twin (section names are the
 * ones every GNU ld bare-metal script shares), because brio needs one
 * thing the vendor templates do not have: a NOLOAD .noinit RAM section
 * excluded from the crt's .bss zero-fill, hosting kernel/platform.hpp's
 * PanicRecord - the breadcrumb must survive a reset exactly as on the
 * other two targets.
 *
 * The vector table sits at the START OF FLASH. Out of reset the boot
 * configuration (nBOOT_SEL/nBOOT0 option bits, the factory default) maps
 * main flash at address 0 as well, and VTOR resets to 0 - so entry 0 is
 * fetched through the alias and the table's own address is its flash
 * address. VTOR is present on this core (RM0444 12.1) and nothing here
 * writes it; the reset/panic pass may.
 *
 * The initial stack pointer is the top of RAM; the stack grows down
 * toward .bss/.noinit with no hardware guard. No heap region: brio
 * allocates nothing, and anything that drags _sbrk in should fail the
 * link loudly.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 144K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        *(.text .text.*)
        *(.rodata .rodata.*)

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
    } > rom

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rom

    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data .data.*)
        . = ALIGN(4);
        __data_end = .;
    } > ram AT > rom
    __data_load_start = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > ram

    /* Survives reset: the crt neither loads nor zeroes it (PanicRecord). */
    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit .noinit.*)
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);
}
