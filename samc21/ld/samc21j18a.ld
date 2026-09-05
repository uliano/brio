/* ============================================================================
 * samc21j18a.ld - hand-written linker script for the ATSAMC21J18A
 * (256 KB flash at 0x0, 32 KB SRAM at 0x20000000 - DS60001479M ch. 10).
 *
 * Written from scratch (the DFP ships a reference script, cribbed for the
 * section names only) because brio needs one thing the vendor script does
 * not provide: a NOLOAD .noinit RAM section excluded from startup's .bss
 * zero-fill, hosting kernel/platform.hpp's PanicRecord - the breadcrumb
 * must survive a reset exactly as it does on AVR (.noinit there).
 *
 * The initial stack pointer is the top of RAM (vector table entry 0,
 * startup_samc21.cpp); the stack grows down toward .bss/.noinit with no
 * hardware guard - the standard bare-metal layout. No heap region is
 * defined on purpose: brio allocates nothing at run time, and anything
 * that drags _sbrk in should fail the link loudly.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        /* The vector table must sit at address 0 (VTOR resets to 0). */
        KEEP(*(.vectors))
        *(.text .text.*)
        *(.rodata .rodata.*)

        /* Static constructor pointers, walked by Reset_Handler itself
         * before main (not by newlib's __libc_init_array, which would
         * drag crti.o's _init back in). brio types are monostate and
         * constexpr so this stays empty in practice, but the walk is
         * kept so a future translation unit with a real constructor is
         * run, not silently skipped. */
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
    } > rom

    /* C++ EH tables would land here; -fno-exceptions keeps them empty.
     * Kept (discarding them breaks the link when libgcc references its
     * own unwind info for division helpers built with EH). */
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rom

    /* Initialized data: load image in flash, run address in RAM. */
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

    /* Survives reset: startup neither loads nor zeroes it (PanicRecord). */
    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit .noinit.*)
    } > ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);
}
