/* ============================================================================
 * rp2040_2m.ld - hand-written linker script for an RP2040 behind a 2 MB
 * quad-SPI flash (a Raspberry Pi Pico's W25Q16, a WeAct board's ZD25Q16):
 * the flash executed in place at 0x1000_0000 through the XIP cache, the
 * 264 KB of SRAM as the datasheet lays it out (2.6.2) - four 64 KB banks
 * striped word by word over 0x2000_0000..0x2003_FFFF, then two 4 KB banks
 * at 0x2004_0000 and 0x2004_1000 that are NOT striped.
 *
 * Written from scratch, the stm32g0 script's twin (the section names are
 * the ones every GNU ld bare-metal script shares), with the two things
 * this chip adds:
 *
 *  - THE FIRST 256 BYTES OF FLASH ARE THE SECOND-STAGE BOOTLOADER (2.8.1):
 *    the bootrom copies them to SRAM, checks their CRC32 and runs them to
 *    set the XIP SSI up for this flash chip; the stage then vectors
 *    through the table it expects at flash offset 0x100 - VTOR written,
 *    SP and PC loaded from it (src/glue/boot2_*.S). So `.boot2` is the
 *    first output section, exactly 256 bytes, and `.text` MUST begin
 *    with the vector table at ORIGIN + 0x100; both are asserted below.
 *    Nothing here writes VTOR again.
 *
 *  - THE STACKS LIVE IN THE TWO UNSTRIPED BANKS, one core each: core 0's
 *    stack is the top of the bank at 0x2004_1000, core 1's the bank at
 *    0x2004_0000 (reserved, untouched until a second kernel runs there)
 *    - the arrangement the chip was drawn for, a core's stack traffic
 *    never contending with the other core's or the DMA's on a striped
 *    bank. The initial stack pointer is the top of scratch_y; the stack
 *    grows down through scratch_y and would then run into scratch_x
 *    with no hardware guard. Four kilobytes per core.
 *
 *  - THE TOP 64 KB OF THE FLASH ARE NOT THE LINKER'S: they are the
 *    storage partition of brio/rp2040/nvm_flash.hpp (the block heap and
 *    the value journal), a constant of that header, so `flash` stops at
 *    1984K and `__brio_flash_end` says where; the header reads the
 *    symbol back and closes the storage if a script ever grew past it.
 *
 *  - CODE THAT RUNS WITH THE FLASH DISCONNECTED lives in `.ram_text`,
 *    an input section placed inside .data so the crt's copy loop moves
 *    it into SRAM with the initialized data: brio/rp2040/flash.hpp's
 *    engine calls the bootrom's flash functions from there, because
 *    while a sector is erased or a page programmed the XIP window is
 *    not there to fetch from.
 *
 * The .data/.bss/.noinit sections take the main 256 KB. .noinit is
 * NOLOAD and excluded from the crt's zero-fill: kernel/platform.hpp's
 * PanicRecord lives there and must survive a reset, as on every target.
 * No heap region: brio allocates nothing, and anything that drags _sbrk
 * in should fail the link loudly.
 * ==========================================================================*/

MEMORY
{
    flash (rx)      : ORIGIN = 0x10000000, LENGTH = 1984K   /* 2048K less the 64K storage partition */
    ram (rwx)       : ORIGIN = 0x20000000, LENGTH = 256K
    scratch_x (rwx) : ORIGIN = 0x20040000, LENGTH = 4K
    scratch_y (rwx) : ORIGIN = 0x20041000, LENGTH = 4K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .boot2 :
    {
        __boot2_start = .;
        KEEP(*(.boot2))
        __boot2_end = .;
    } > flash
    ASSERT(__boot2_end - __boot2_start == 256,
           "the second-stage bootloader must be exactly 256 bytes (src/glue/boot2_*.S)")

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
    } > flash
    ASSERT(ADDR(.text) == ORIGIN(flash) + 0x100,
           "the vector table must sit at flash offset 0x100, where the second stage vectors to")

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > flash

    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.ram_text .ram_text.*)
        . = ALIGN(4);
        *(.data .data.*)
        . = ALIGN(4);
        __data_end = .;
    } > ram AT > flash
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

    /* One past the last flash byte the linker may place: the storage
       partition's floor, read back by brio/rp2040/nvm_flash.hpp. */
    __brio_flash_end = ORIGIN(flash) + LENGTH(flash);

    /* Core 0's stack: the whole of the upper unstriped bank. */
    __stack_top = ORIGIN(scratch_y) + LENGTH(scratch_y);
    /* Core 1's, when a second kernel runs there. */
    __stack_one_top = ORIGIN(scratch_x) + LENGTH(scratch_x);
}
