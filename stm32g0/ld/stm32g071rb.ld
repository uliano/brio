/* ============================================================================
 * stm32g071rb.ld - hand-written linker script for the STM32G071RB
 * (128 KB flash at 0x0800_0000 in ONE bank - RM0444 table 5's G071xB
 * row - and 36 KB SRAM at 0x2000_0000 with the parity check disabled,
 * which is the option byte's default - RM0444 table 7).
 *
 * The G0B1RE script's twin (ld/stm32g0b1re.ld carries the reasoning for
 * every section, the NOLOAD .noinit above all), with what differs:
 *
 *  - THERE IS NO BANK 2. The G071 class is single-bank, so the storage
 *    attic brio/stm32g0/nvm_flash.hpp gives the NvHeap and the NvJournal
 *    on the G0B1 does not exist here, and the image may use the whole
 *    128 K. `__brio_rom_end` stops at 0x0802 0000, which is NOT the
 *    G0B1's bank boundary: the storage backends refuse to open (they
 *    answer bad_geometry and write nothing), which is the honest answer
 *    on a part with no read-while-write region - a program that wants
 *    flash storage on a single-bank part is a design question this
 *    script does not answer.
 *  - 36 K of RAM, not 144.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 36K
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

    /* One past the last byte the linker may place: the end of the one
       bank. brio/stm32g0/nvm_flash.hpp reads it as a value. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
