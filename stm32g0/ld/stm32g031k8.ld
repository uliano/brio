/* ============================================================================
 * stm32g031k8.ld - hand-written linker script for the STM32G031K8
 * (64 KB flash at 0x0800_0000 in ONE bank - RM0444 table 5's G031x8
 * row - and 8 KB SRAM at 0x2000_0000, the same 8 K with the parity
 * check on or off - RM0444 table 7).
 *
 * The G0B1RE script's twin (ld/stm32g0b1re.ld carries the reasoning for
 * every section, the NOLOAD .noinit above all), with what differs:
 *
 *  - THERE IS NO BANK 2. The G031 class is single-bank, so the storage
 *    attic brio/stm32g0/nvm_flash.hpp gives the NvHeap and the NvJournal
 *    on the G0B1 does not exist here, and the image may use the whole
 *    64 K. `__brio_rom_end` stops at 0x0801 0000, which is NOT the
 *    G0B1's bank boundary: the storage backends refuse to open (they
 *    answer bad_geometry and write nothing), which is the honest answer
 *    on a part with no read-while-write region - a program that wants
 *    flash storage on a single-bank part is a design question this
 *    script does not answer.
 *  - 8 K of RAM, not 144: a kernel app with a 1 KB console ring fits, a
 *    suite with its instruments may not - the suites say so per board.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
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
