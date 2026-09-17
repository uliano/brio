/* ============================================================================
 * ch32v203k8.ld - hand-written linker script for the CH32V203K8
 * (64 KB code flash and 20 KB SRAM - CH32V203 datasheet V2.8 table 2-1).
 *
 * FLASH AT ZERO, AND WHY THE FIRST WORD IS AN INSTRUCTION. The array
 * answers at two addresses on this family, 0x0800_0000 and the alias at
 * 0x0000_0000 the core boots from; the image is linked at the alias, so
 * the first word fetched after reset is the first word of the image.
 * That word is a JUMP, and the words after it are handler ADDRESSES,
 * because mtvec is set by the crt with both mode bits - MODE1 =
 * absolute addresses, MODE0 = vectored by interrupt number (QingKe V4
 * manual 3.2). WCH's own startup puts the jump in a section of its own
 * and the table after it; this project keeps the CH32V00x's shape, one
 * table whose entry 0 is the instruction, because entry 0 is not a
 * vector on either core.
 *
 * THE LINKER GETS THE WHOLE ARRAY. No storage classes are carved out of
 * flash on this target: whether this family wants an NvHeap and an
 * NvJournal at all is the open NV review (docs/design/nv-heap.md's
 * question, not this script's), and a partition every image pays for is
 * a decision, not a default. `__brio_rom_end` is therefore the end of
 * the array, and it is here so that the day a medium is written it has
 * a floor to compare itself against.
 *
 * .noinit is NOLOAD and the crt neither loads nor zeroes it, which is
 * what lets the kernel's PanicRecord breadcrumb survive a warm reset.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(reset_handler)

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
        . = ALIGN(4);
    } > rom

    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data .data.*)
        . = ALIGN(8);
        /* The global pointer addresses 4 KB around itself, so it is
           placed 2 KB into the small-data area: every .sdata and .sbss
           object is then one gp-relative instruction away. */
        __global_pointer$ = . + 0x800;
        *(.sdata .sdata.*)
        . = ALIGN(4);
        __data_end = .;
    } > ram AT > rom
    __data_load_start = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.sbss .sbss.*)
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
        . = ALIGN(4);
        /* One past the last RAM byte the linker placed: the stack may
           grow down to here and no further. The crt paints the gap up to
           the stack top; brio/ch32v203/platform.hpp measures it. */
        __stack_floor = .;
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    /* One past the last byte the linker may place. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
