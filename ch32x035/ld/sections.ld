/* ============================================================================
 * sections.ld - the CH32X035 series' hand-written section layout, linked
 * after the part's own script (ld/<part>.ld, its two memories) on every
 * image of this project, which is how one layout serves the seven parts.
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
 * .data IS WRITTEN FIRST AND PLACED SECOND. The image is laid out as
 * .text, .data, .bss and .noinit, in that order, but the .data statement
 * comes first in this file and `INSERT AFTER .text` puts it back after
 * .text. The reason is how ld hands out an input section: to the FIRST
 * statement of the script whose pattern matches it, so a function's
 * `.text.<name>` named in .data reaches .data only when .data's statement
 * is read before .text's `*(.text .text.*)`. The scheduler path's
 * placement (below) names sections that way. The layout ld computes, and
 * every image's bytes, are those of the order the sections stand in.
 * ld accepts an INSERT only in a script that follows another one on the
 * link line - with none before it, ld adds its own default script - and
 * only pointing at a section written after it, which the part's script
 * and this file's order satisfy.
 *
 * .noinit is NOLOAD and the crt neither loads nor zeroes it, which is
 * what lets the kernel's PanicRecord breadcrumb survive a warm reset.
 * ==========================================================================*/

ENTRY(reset_handler)

SECTIONS
{
    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        /* CODE THAT RUNS FROM SRAM, placed first in .data so that the
           crt's copy of .data moves it into SRAM before main() and an
           image that places nothing there gets zero bytes from these
           lines.
           The preemptive kernel's scheduler path comes first, where the
           program's `// build: scheduler` line puts it (ram by default;
           docs/ch32x035/platform.md, "The scheduler's place"): the
           included fragment is ld/scheduler_ram/scheduler_text.ld, which
           names the path's sections, or ld/scheduler_flash/
           scheduler_text.ld, which names none - the build gives the
           linker the directory of the one the image asks for
           (ch32x035/CMakeLists.txt). In an image with no preemptive
           kernel the path's sections do not exist and the fragment
           places nothing.
           `.fast_text` is code that MAY run from SRAM, for speed -
           what a program opts into, util/placement.hpp's Hot<InRam> and
           a program's own placed function; what it buys is the fetch:
           the code flash runs behind two wait states at 48 MHz (RM
           20.3.1), the SRAM answers without them
           (docs/ch32x035/platform.md, "A handler in SRAM").
           `.ram_text` is code that MUST, for correctness - nothing of
           this stratum today, the name kept for what the family's own
           pieces may need. */
        INCLUDE scheduler_text.ld
        *(.fast_text .fast_text.*)
        *(.ram_text .ram_text.*)
        . = ALIGN(4);
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
}
INSERT AFTER .text;

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
