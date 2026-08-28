// ============================================================================
//  ivsel_boot.cpp - the interrupt vector table belongs at address 0.
//
//  THIS FILE IS COMPILED INTO EVERY IMAGE (CMakeLists.txt's
//  avr_add_app() lists it alongside every app's own source). It is not
//  an app and it has no main(); it is one build invariant expressed as
//  four instructions.
//
//  WHY IT EXISTS. The AVR DA/DB splits the Flash into a BOOT section and
//  an application section, and by default (CPUINT.CTRLA.IVSEL = 0) the
//  hardware looks for the interrupt vector table at the START OF THE
//  APPLICATION SECTION - not at address 0 (DS40002247B 15.5.1, and
//  11.3.1.1 note 1). Every brio image is linked at 0: reset entry,
//  vector table, code, all of it inside BOOT. So the moment the
//  BOOTSIZE fuse stops being 0 - which is what any image that writes
//  its own Flash needs, since code can never write the section it runs
//  from - the vectors the linker wrote at 0 stop being the vectors the
//  hardware reads, and the first interrupt of any kind jumps into
//  erased Flash. The symptom is not a crash but a RESET LOOP (0xFFFF
//  decodes and the program counter eventually wraps to 0), which is a
//  miserable thing to debug.
//
//  Setting IVSEL says "the vectors are at the start of BOOT", and BOOT
//  starts at 0 under EVERY geometry. It is therefore correct with
//  BOOTSIZE = 0 as well - where the hardware ignores the bit entirely -
//  and needs no coordination with the fuses at all. Making it an
//  invariant of every image, rather than something each app remembers,
//  is what keeps a fuse change from silently breaking apps that were
//  built before it.
//
//  WHY .init3. The vector table must be right before anything can
//  enable an interrupt. .init3 runs from the reset vector, after .init2
//  has set the stack pointer and cleared the zero register and before
//  .init4 copies .data - the earliest point at which ordinary code can
//  run at all. Nothing is live across the .initN fragments except the
//  stack pointer and r1, so r16/r17 are free.
//
//  WHY ASSEMBLY. A .init3 fragment is entered by falling into it and
//  left by falling out of it: `naked` is what removes the prologue and
//  the `ret` that would otherwise return to nowhere. With `naked` the
//  compiler is not allowed to generate a body anyway - the whole
//  function must be asm. `used` keeps it through -ffunction-sections
//  and --gc-sections, which have no reason to think anyone calls it.
//
//  The store is Configuration Change Protected with the IOREG key
//  (DS40002247B table 15-3): the key goes into CPU.CCP and the
//  protected write must follow within four instructions. Both values
//  are loaded FIRST so that the store is the very next instruction
//  after the key - one instruction of the four, with three to spare.
//
//  Errata DS80000915F 2.2.4 (a store to an address >= 64 immediately
//  followed by a write to SLPCTRL.CTRLA loses that write) does not
//  apply: nothing here writes SLPCTRL. Verified in firmware.lst, which
//  is where the same claim is checked for the sleep driver.
//
//  The run-time twin of these four instructions is
//  brio::Nvm::vectors_in_boot() (brio/avrdx/nvm.hpp), with
//  vectors_in_boot_armed() as its readback; the two must stay the same
//  store. This one exists because .init3 has no C++ available yet.
// ============================================================================

#include <avr/io.h>

extern "C" [[gnu::naked, gnu::used, gnu::section(".init3")]]
void brio_vectors_in_boot();

void brio_vectors_in_boot() {
    __asm__ __volatile__(
        "ldi r16, %[key]        \n\t"
        "ldi r17, %[ivsel]      \n\t"
        "out %[ccp], r16        \n\t"
        "sts %[ctrla], r17      \n\t"
        :
        : [key] "M"(CCP_IOREG_gc), [ivsel] "M"(CPUINT_IVSEL_bm),
          [ccp] "I"(_SFR_IO_ADDR(CCP)), [ctrla] "n"(&CPUINT.CTRLA));
}
