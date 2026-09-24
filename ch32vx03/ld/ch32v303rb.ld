/* ============================================================================
 * ch32v303rb.ld - hand-written linker script for the CH32V303RB
 * (128 KB code flash and 32 KB SRAM - CH32V307 datasheet V3.5 table
 * 2-1-1; LQFP64M). The QingKe V4F core boots the way the V4B does, so
 * everything below the memories is the CH32V203's script.
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
 * THE LAST 4 KB OF THE WINDOW ARE NOT THE LINKER'S. The flash medium
 * (brio/ch32vx03/nvm_flash.hpp) lives in the top sixteen pages of 256
 * bytes of the zero-wait window this part's program occupies - 4 KB,
 * which is this silicon's write-protection unit and its standard erase
 * grain - so the `rom` region stops that far short of the window and
 * `__brio_rom_end` is where it stops. Nothing the compiler emits can
 * land in the zone, which is what lets the medium's floor be a constant
 * rather than a symbol that moves with every build, and what keeps an
 * image from growing into its own storage; the medium reads
 * `__brio_rom_end` back as the proof, and closes itself instead of
 * writing over code if this script is ever edited past the line. NO HEAP
 * AND NO JOURNAL STAND ON THAT MEDIUM on this family
 * (docs/design/nv-heap.md's question, answered here): the zone is the
 * medium's whole business.
 *
 * .noinit is NOLOAD and the crt neither loads nor zeroes it, which is
 * what lets the kernel's PanicRecord breadcrumb survive a warm reset.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 124K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
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
           the stack top; brio/ch32vx03/platform.hpp measures it. */
        __stack_floor = .;
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    /* One past the last byte the linker may place. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
