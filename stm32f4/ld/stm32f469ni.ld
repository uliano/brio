/* ============================================================================
 * stm32f469ni.ld - hand-written linker script for the STM32F469NI
 * (2 Mbyte dual-bank flash at 0x0800_0000, RM0386 table 4; 320 Kbyte of
 * contiguous SRAM at 0x2000_0000 - SRAM1 160 K, SRAM2 32 K and SRAM3
 * 128 K back to back, RM0386 2.3.1; plus 64 Kbyte of CCM data RAM at
 * 0x1000_0000 reachable by the CPU alone, 2.3.1 - and 4 Kbyte of backup
 * SRAM behind the PWR domain, not a region here). The flash size
 * register at 0x1FFF7A22 reads 0x800 Kbyte on the bench chip, checked
 * over SWD before this file was written.
 *
 * Written from scratch, the stm32g0 script's twin (section names are the
 * ones every GNU ld bare-metal script shares), because brio needs one
 * thing the vendor templates do not have: a NOLOAD .noinit RAM section
 * excluded from the crt's .bss zero-fill, hosting kernel/platform.hpp's
 * PanicRecord - the breadcrumb must survive a reset exactly as on the
 * other targets.
 *
 * The vector table sits at the START OF FLASH. Out of reset with
 * BOOT0 low (the boards' default) main flash is aliased at address 0 as
 * well, and VTOR resets to 0 - so entry 0 is fetched through the alias
 * and the table's own address is its flash address. Nothing here writes
 * VTOR; the reset/panic pass may.
 *
 * The initial stack pointer is the top of the main SRAM; the stack grows
 * down toward .bss/.noinit with no hardware guard. No heap region: brio
 * allocates nothing, and anything that drags _sbrk in should fail the
 * link loudly.
 *
 * THE CCM IS A REGION AND NOTHING IS PLACED IN IT. It is the fastest RAM
 * of the part (zero wait states, on the core's D-bus) and the one no
 * DMA stream can reach (2.3.1) - a program that wants a stack or a
 * hot buffer there names the section itself; putting .data or .bss
 * there by default would hand a DMA engine an address it cannot use.
 *
 * ROM IS THE WHOLE ARRAY for now. The sector map is uneven (four 16 K,
 * one 64 K, seven 128 K per bank) and brio's flash storage on this part
 * is a decision of its own (docs/design/nv-heap.md); when it is taken
 * the storage sectors leave this region and __brio_rom_end below is how
 * the storage backend checks that they did.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 320K
    ccm (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
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

    /* The core-coupled memory: named by a program that wants it, never
       loaded, never zeroed - the crt knows nothing of this region. */
    .ccm (NOLOAD) :
    {
        . = ALIGN(4);
        *(.ccm .ccm.*)
    } > ccm

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    /* One past the last byte the linker may place: the storage backend
       to come reads it as a value. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
