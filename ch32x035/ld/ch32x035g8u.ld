/* ============================================================================
 * ch32x035g8u.ld - hand-written linker script for the CH32X035G8U6
 * (62 KB code flash and 20 KB SRAM - CH32X035/X033 datasheet V1.7, the
 * model table; CH32X035 reference manual V1.8, 1.2.1 and table 20-1).
 * Every part of the series has the same two memories, so the seven
 * scripts of ld/ differ in their name alone.
 *
 * FLASH AT ZERO, AND WHY THE FIRST WORD IS AN INSTRUCTION. The code flash
 * answers at 0x0800_0000 and at the alias at 0x0000_0000 the core boots
 * from (RM 1.2: the alias is the flash or the system memory, as the boot
 * mode says); the image is linked at the alias, so the first word the
 * core fetches after reset is the first word of the image. That word is a
 * JUMP, and the words after it are handler ADDRESSES, because mtvec is set
 * by the crt with both mode bits - MODE1 = absolute addresses, MODE0 =
 * vectored by interrupt number (RM 7.5.3.2). WCH's own startup puts the
 * jump in a section of its own and the table after it; this project keeps
 * the sibling projects' shape, one table whose entry 0 is the
 * instruction, because entry 0 is not a vector on this core either.
 *
 * THE WHOLE 62 KB IS THE LINKER'S. No storage medium stands on this
 * series' flash: whether a program wants one is the NV stack review's
 * question (docs/design/nv-heap.md), and the two sibling projects each
 * answer it with a zone this script would have to stop short of. The day
 * a medium is written, `rom` shrinks here and `__brio_rom_end`, the
 * boundary read back as a value, says where it stops.
 *
 * .noinit is NOLOAD and the crt neither loads nor zeroes it, which is
 * what lets the kernel's PanicRecord breadcrumb survive a warm reset.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 62K
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
           the stack top; brio/ch32x035/platform.hpp measures it. */
        __stack_floor = .;
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    /* One past the last byte the linker may place. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
