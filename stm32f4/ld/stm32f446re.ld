/* ============================================================================
 * stm32f446re.ld - hand-written linker script for the STM32F446RE
 * (512 Kbyte single-bank flash at 0x0800_0000, RM0390 table 4; 128 Kbyte
 * of contiguous SRAM at 0x2000_0000 - SRAM1 112 K and SRAM2 16 K back to
 * back, RM0390 2.3.1; plus 4 Kbyte of backup SRAM behind the PWR domain,
 * not a region here; no CCM on this part). The flash size register at
 * 0x1FFF7A22 reads 0x200 Kbyte on the bench chip, checked over SWD
 * before this file was written.
 *
 * The stm32f429zi script without the CCM region: the same sections, the
 * same NOLOAD .noinit for kernel/platform.hpp's PanicRecord, the vector
 * table at the start of flash fetched through the boot alias at 0, the
 * stack at the top of RAM, no heap. ROM is the whole array for now - the
 * sector map is uneven (four 16 K, one 64 K, three 128 K) and the
 * storage partition is docs/design/nv-heap.md's decision; __brio_rom_end
 * is how the storage backend will check it.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
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

    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
