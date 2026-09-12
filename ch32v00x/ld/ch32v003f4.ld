/* ============================================================================
 * ch32v003f4.ld - hand-written linker script for the CH32V003F4
 * (16 KB flash at 0x0000_0000 and 2 KB SRAM at 0x2000_0000 - CH32V003
 * datasheet V1.6, 1.4.2; the 1920-byte boot loader lives in its own
 * system area, not in this array).
 *
 * FLASH AT ZERO, AND WHY THE FIRST WORD IS AN INSTRUCTION. This core
 * fetches its first instruction from address 0, so the image starts
 * with the vector table and the table's first word is a JUMP, not a
 * pointer (the words after it ARE pointers - QingKe V2 vectored mode,
 * RM 6.3). That is the opposite of the ARM convention, where word 0 is
 * the initial stack pointer, and it is why .vectors must be KEPT first
 * in .text and why the crt sets the stack pointer itself.
 *
 * THE LINKER GETS 15 K OF THE 16. The array is one, and the storage
 * class (brio/ch32v00x/nvm_flash.hpp: the NvJournal's 1 KB attic of
 * sixteen 64-byte pages - no heap, a 16 KB array has no room for a
 * block allocator) lives in its top kilobyte as a CONSTANT partition -
 * so `rom` stops at 0x3C00 and nothing the compiler emits can land
 * above it. `__brio_rom_end` below is that boundary read back as a
 * value: the media compare it with their floor and refuse to open if
 * this script is ever edited past it.
 *
 * .noinit is NOLOAD and the crt neither loads nor zeroes it, which is
 * what lets the kernel's PanicRecord breadcrumb survive a warm reset.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 15K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 2K
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
           grow down to here and no further. The crt paints the gap up
           to the stack top; brio/ch32v00x/platform.hpp measures it. */
        __stack_floor = .;
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    /* One past the last byte the linker may place: the storage floor.
       brio/ch32v00x/nvm_flash.hpp reads it as a value. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
