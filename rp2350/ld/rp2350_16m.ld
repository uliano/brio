/* ============================================================================
 * rp2350_16m.ld - hand-written linker script for an RP2350 behind a 16 MB
 * quad-SPI flash, and THE SAME SCRIPT FOR BOTH ARCHITECTURES: nothing in
 * it is Arm's or RISC-V's, because what the two crts put in `.vectors`
 * differs (a Cortex-M vector table, or one jump instruction) while WHERE
 * it goes does not - the lowest address of the image.
 *
 * NO SECOND-STAGE BOOTLOADER (datasheet 5.9.5). The RP2040's first 256
 * flash bytes were a checksummed stage that set the flash interface up;
 * here the bootrom does a best-effort XIP setup itself while scanning the
 * flash (03h serial reads at CLKDIV 12, 5.1.4) and a flash-resident
 * program runs in that state until it chooses to reprogram the QMI. So
 * flash begins with the image, and what makes the image an image is the
 * IMAGE_DEF block the crt places inside the first 4 kB, right behind the
 * vector table or the entry jump.
 *
 * THE MEMORIES (datasheet 2.1, 4.1): flash executed in place at
 * 0x1000_0000 through the XIP cache; 512 KB of SRAM striped word by word
 * over eight banks at 0x2000_0000, then TWO 4 KB banks at 0x2008_0000
 * and 0x2008_1000 that are not striped - the two cores' stacks, one
 * each, exactly as on the RP2040 and for the same reason: a core's stack
 * traffic never contends with the other core's or the DMA's on a striped
 * bank. The initial stack pointer is the top of scratch_y; core 1's is
 * the top of scratch_x, reserved and untouched until a second kernel
 * runs there.
 *
 * THE TOP 64 KB OF THE FLASH ARE NOT THE LINKER'S: the storage partition
 * rule this project carries on every target with a flash of its own -
 * the region stops 64 KB short and `__brio_flash_end` says where, for
 * the flash chapter to stand on.
 *
 * CODE THAT RUNS WITH THE FLASH DISCONNECTED lives in `.ram_text`, an
 * input section placed inside .data so the crt's copy loop moves it into
 * SRAM with the initialized data - the RP2040's arrangement, kept
 * because the flash chapter of this chip needs the same window.
 *
 * `__global_pointer$` is the RISC-V linker relaxation anchor (the crt
 * loads it into gp with relaxation off); on the Arm build it is one
 * unused symbol and no bytes.
 *
 * .noinit is NOLOAD and excluded from the crt's zero-fill: the
 * PanicRecord lives there and must survive a reset, as on every target.
 * No heap region: brio allocates nothing, and anything that drags _sbrk
 * in should fail the link loudly.
 * ==========================================================================*/

MEMORY
{
    flash (rx)      : ORIGIN = 0x10000000, LENGTH = 16320K  /* 16384K less the 64K storage partition */
    ram (rwx)       : ORIGIN = 0x20000000, LENGTH = 512K
    scratch_x (rwx) : ORIGIN = 0x20080000, LENGTH = 4K
    scratch_y (rwx) : ORIGIN = 0x20081000, LENGTH = 4K
}

ENTRY(_start)

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        KEEP(*(.image_def))
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

        /* THE LOAD ADDRESS OF .data IS WHATEVER FLASH CURSOR IS HERE,
           and the crt copies .data to SRAM with WORD loads: on Hazard3 a
           misaligned load is a trap and not a slow access, so a .text
           that happens to end on a half-word (compressed instructions
           make that the usual case) would fault the image before main().
           ALIGN(4) at the end of every flash-resident section is what
           keeps that address a word. */
        . = ALIGN(4);
    } > flash
    ASSERT(ADDR(.text) == ORIGIN(flash),
           "the image must begin at the first flash byte: the bootrom enters an Arm image through the vector table it finds there, and a RISC-V image at its lowest address")

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
    } > flash

    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.ram_text .ram_text.*)
        . = ALIGN(4);
        *(.data .data.*)
        . = ALIGN(8);
        __global_pointer$ = . + 0x800;
        *(.sdata .sdata.* .sdata2 .sdata2.*)
        . = ALIGN(4);
        __data_end = .;
    } > ram AT > flash
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

    /* One past the last flash byte the linker may place: the storage
       partition's floor, for the flash chapter to read back. */
    __brio_flash_end = ORIGIN(flash) + LENGTH(flash);

    /* Core 0's stack: the whole of the upper unstriped bank. */
    __stack_top = ORIGIN(scratch_y) + LENGTH(scratch_y);
    /* Core 1's, when a second kernel runs there. */
    __stack_one_top = ORIGIN(scratch_x) + LENGTH(scratch_x);
}
