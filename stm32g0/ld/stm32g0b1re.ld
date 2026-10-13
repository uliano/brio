/* ============================================================================
 * stm32g0b1re.ld - hand-written linker script for the STM32G0B1RE
 * (512 KB flash at 0x0800_0000, 144 KB SRAM at 0x2000_0000 - RM0444
 * table 2; the flash size register at 0x1FFF75E0 reads 0x200 KB on the
 * bench chip, checked over SWD before this file was written).
 *
 * Written from scratch, the samc21 script's twin (section names are the
 * ones every GNU ld bare-metal script shares), because brio needs one
 * thing the vendor templates do not have: a NOLOAD .noinit RAM section
 * excluded from the crt's .bss zero-fill, hosting kernel/platform.hpp's
 * PanicRecord - the breadcrumb must survive a reset exactly as on the
 * other two targets.
 *
 * The vector table sits at the START OF FLASH. Out of reset the boot
 * configuration (nBOOT_SEL/nBOOT0 option bits, the factory default) maps
 * main flash at address 0 as well, and VTOR resets to 0 - so entry 0 is
 * fetched through the alias and the table's own address is its flash
 * address. VTOR is present on this core (RM0444 12.1) and nothing here
 * writes it; the reset/panic pass may.
 *
 * The initial stack pointer is the top of RAM; the stack grows down
 * toward .bss/.noinit with no hardware guard. No heap region: brio
 * allocates nothing, and anything that drags _sbrk in should fail the
 * link loudly.
 *
 * ROM IS BANK 1 ONLY - 256 K of the part's 512, and the reason is not
 * that the image is small. The upper 256 K is physical BANK 2 (RM0444
 * table 11), and brio/stm32g0/nvm_flash.hpp gives it to the NvHeap and
 * the NvJournal:
 *
 *   - programming or erasing bank 2 leaves reads of bank 1 running
 *     (RM0444 3.3.9, read-while-write), so a record can be stored from
 *     the main loop without stopping the CPU - inside one bank the read
 *     would stall for the whole 22..40 ms of a page erase;
 *   - erratum ES0548 2.2.10 says the prefetch may fail when the CPU
 *     BRANCHES ACROSS BANKS, with no workaround, and its own note lists
 *     "EEPROM emulation or other data storage in bank 2" as the safe
 *     use. Code must therefore stay in one bank; data in the other is
 *     the vendor's sanctioned arrangement - and the one bank is what
 *     lets stm32g0/clock.hpp turn the prefetch on in every image;
 *   - and it makes the storage bounds CONSTANTS rather than linker
 *     symbols that move with every build - the property the samc21's
 *     RWWEE array has for free and the AVR backend has to work for.
 *
 * NO SECTION MAY BE PLACED IN BANK 2. `__brio_rom_end` below is how
 * nvm_flash.hpp checks that at run time: it must stop at 0x0804 0000,
 * and a script edited back to 512 K closes the storage (both mounts
 * answer bad_geometry and write nothing) instead of letting an image
 * grow over stored data. Nothing closes the prefetch that way: a
 * program linked across both banks turns it off itself
 * (FlashAccel::prefetch(false), brio/stm32g0/flash.hpp).
 * ==========================================================================*/

MEMORY
{
    /* 256K = BANK 1 ALONE, and load-bearing: stm32g0/clock.hpp turns the
     * prefetch on, which ES0548 2.2.10 makes safe for code in one bank. */
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 144K
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

    /* Initialized data: load image in flash, run address in RAM.
     *
     * CODE THAT RUNS FROM SRAM, placed first inside .data: the crt's copy
     * of .data moves it into SRAM with the initialized data before main,
     * so it needs no loop of its own, and an image that places nothing
     * there gets zero bytes from these lines. `.fast_text` is code that
     * MAY run from SRAM, for speed - what a program opts into,
     * util/placement.hpp's Hot<InRam> and a program's own placed
     * function; what it buys is the fetch: at 64 MHz the flash answers at
     * two wait states behind the prefetch and the instruction cache
     * (RM0444 3.3.4, 3.3.5), the SRAM at none (2.3) -
     * docs/stm32g0/platform.md, "Code in SRAM". `.ram_text` is code that
     * MUST, for correctness - nothing of this stratum today, the name kept
     * for what the family's own pieces may need. A call between the two
     * memories is out of a BL's reach and the linker puts a long-branch
     * veneer beside the caller (its input section's name and `.__stub`),
     * in whichever memory the caller runs from. */
    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.fast_text .fast_text.*)
        *(.ram_text .ram_text.*)
        . = ALIGN(4);
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

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    /* One past the last byte the linker may place. brio/stm32g0/
       nvm_flash.hpp reads it as a value and refuses to open the storage
       in bank 2 unless it stops at the bank boundary. */
    __brio_rom_end = ORIGIN(rom) + LENGTH(rom);
}
