/* ============================================================================
 * ch32v006k8.ld - hand-written linker script for the CH32V006K8
 * (62 KB flash at 0x0000_0000 and 8 KB SRAM at 0x2000_0000 - CH32V006
 * datasheet V2.0, table 1-1).
 *
 * FLASH AT ZERO, AND WHY THE FIRST WORD IS AN INSTRUCTION. This core
 * fetches its first instruction from address 0, so the image starts
 * with the vector table and the table's first word is a JUMP, not a
 * pointer (the words after it ARE pointers - QingKe V2 vectored mode,
 * RM 6.3). That is the opposite of the ARM convention, where word 0 is
 * the initial stack pointer, and it is why .vectors must be KEPT first
 * in .text and why the crt sets the stack pointer itself.
 *
 * 62 K, not 64: the top two kilobytes of the flash array are the boot
 * loader's, and the user program's space ends where the datasheet says
 * it does.
 *
 * .noinit is NOLOAD and the crt neither loads nor zeroes it, which is
 * what lets the kernel's PanicRecord breadcrumb survive a warm reset.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 62K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
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
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);
}
