// startup_samc21.cpp - vector table + reset path for the SAM C21, compiled
// into EVERY image of this project (sam_add_app() lists it alongside the
// app's own source - the samc21 analog of the AVR build's ivsel_boot.cpp
// glue slot). No vendor ASF startup: this file and ld/samc21j18a.ld are the
// whole crt.
//
// The app-binds-the-vector rule, ARM edition: every handler below is a WEAK
// alias for Default_Handler (a spin), and an app binds a vector by defining
// the strong symbol itself -
//
//   extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
//
// exactly as an AVR app writes ISR(RTC_PIT_vect) {...}. Vector names never
// appear in portable code; the IRQ list is samc21j18a.h's IRQn enum
// (SERCOM0..5 = 9..14, TCC0..2, TC0..4, ADC0/1, ... - 31 peripheral lines,
// several sharing line 0).
//
// Reset_Handler does the minimum an image needs: copy .data from flash,
// zero .bss, run static constructors (walking .init_array itself - brio's
// monostate types keep the list empty in practice, see the linker script),
// call main(). It deliberately does NOT touch the clock: SysClock::init()
// in main() owns the clock tree, same division of labor as on AVR.
// .noinit is neither loaded nor zeroed, which is what lets the
// PanicRecord breadcrumb survive a warm reset. Note what the silicon
// does NOT promise: DS60001479M table 18-1 lists what each reset cause
// resets and has no SRAM row at all, for any source - so the breadcrumb's
// magic word (kernel/panic.hpp) is necessary, not merely prudent.
//
// HardFault gets its own weak spin handler rather than folding into
// Default_Handler: on ARMv6-M the core cannot ask whether a debugger is
// attached (DHCSR is debugger-access-only, unlike v7-M), so a BKPT with no
// debugger escalates here - a distinct symbol makes the wreck legible in a
// backtrace. The full panic-breadcrumb-then-reset story is a later platform
// pass (docs/samc/platform.md gap list).

#include <stdint.h>

extern "C" {

// Linker-script symbols (ld/samc21j18a.ld).
extern uint32_t __data_load_start;
extern uint32_t __data_start;
extern uint32_t __data_end;
extern uint32_t __bss_start;
extern uint32_t __bss_end;
extern void (*__preinit_array_start[])();
extern void (*__preinit_array_end[])();
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();

// Deliberately declared as a FUNCTION: entry 0 of the vector table is the
// initial stack pointer, and lying about the symbol's type lets its address
// sit in the Handler array with no reinterpret_cast (which constexpr
// initialization would reject). Nothing ever calls it.
void __stack_top();

}  // extern "C" (reopened below - main() must be declared with C++ linkage)

int main();

extern "C" {

// Not [[noreturn]]: every weak alias below must carry attributes at least
// as strict as its target (-Werror=missing-attributes), and a handler that
// an app later overrides DOES return - the plain void() signature is the
// honest common type.
void Default_Handler()
{
    for (;;) {}
}

void HardFault_Handler()
{
    for (;;) {}
}

// abort(): the ONE libc symbol a brio image really does reference, and it
// gets its definition here rather than from newlib.
//
// With -fno-exceptions libstdc++ compiles every throw site into a call to
// abort(), std::__throw_bad_variant_access() among them - so every app
// built on the AO kernel reaches it the moment the optimizer stops
// proving that branch dead (-Os does; -Og does not, which is how this
// surfaced). newlib's own abort() goes through raise()/_exit() and would
// drag in the whole syscall stub set, defeating the deliberate "no
// nosys.specs" rule in samc/CMakeLists.txt - a rule worth keeping,
// because it is what makes an accidental _sbrk or _write fail the link
// instead of failing silently at run time.
//
// A spin, not a BKPT: on ARMv6-M a BKPT with no debugger attached
// escalates to HardFault and the frame that got here is gone, while a
// halt-and-spin leaves the whole call stack for the debugger to walk.
// Routing this into the panic breadcrumb belongs with the reset/panic
// pass, the samc analog of avrdx/reset.hpp.
[[noreturn]] void abort()
{
    for (;;) {}
}

[[noreturn]] void Reset_Handler()
{
    const uint32_t* src = &__data_load_start;
    for (uint32_t* dst = &__data_start; dst != &__data_end; ) {
        *dst++ = *src++;
    }
    for (uint32_t* dst = &__bss_start; dst != &__bss_end; ) {
        *dst++ = 0u;
    }
    // Static constructors, walked directly (newlib's __libc_init_array
    // would drag in crti.o's _init, which -nostartfiles excludes - this
    // crt owns the whole job instead).
    for (auto** p = __preinit_array_start; p != __preinit_array_end; ++p) {
        (*p)();
    }
    for (auto** p = __init_array_start; p != __init_array_end; ++p) {
        (*p)();
    }
    main();
    for (;;) {}   // a returning main() parks here
}

// Core exceptions.
void NMI_Handler()        __attribute__((weak, alias("Default_Handler")));
void SVC_Handler()        __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler()     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler()    __attribute__((weak, alias("Default_Handler")));

// Peripheral lines, in IRQn order (samc21j18a.h). Line 0 is SHARED by
// MCLK/OSCCTRL/OSC32KCTRL/PAC/SUPC - one vector, one name here; a handler
// for it reads the pending peripheral itself.
void SYSTEM_Handler()     __attribute__((weak, alias("Default_Handler")));  // 0
void WDT_Handler()        __attribute__((weak, alias("Default_Handler")));  // 1
void RTC_Handler()        __attribute__((weak, alias("Default_Handler")));  // 2
void EIC_Handler()        __attribute__((weak, alias("Default_Handler")));  // 3
void FREQM_Handler()      __attribute__((weak, alias("Default_Handler")));  // 4
void TSENS_Handler()      __attribute__((weak, alias("Default_Handler")));  // 5
void NVMCTRL_Handler()    __attribute__((weak, alias("Default_Handler")));  // 6
void DMAC_Handler()       __attribute__((weak, alias("Default_Handler")));  // 7
void EVSYS_Handler()      __attribute__((weak, alias("Default_Handler")));  // 8
void SERCOM0_Handler()    __attribute__((weak, alias("Default_Handler")));  // 9
void SERCOM1_Handler()    __attribute__((weak, alias("Default_Handler")));  // 10
void SERCOM2_Handler()    __attribute__((weak, alias("Default_Handler")));  // 11
void SERCOM3_Handler()    __attribute__((weak, alias("Default_Handler")));  // 12
void SERCOM4_Handler()    __attribute__((weak, alias("Default_Handler")));  // 13
void SERCOM5_Handler()    __attribute__((weak, alias("Default_Handler")));  // 14
void CAN0_Handler()       __attribute__((weak, alias("Default_Handler")));  // 15
void CAN1_Handler()       __attribute__((weak, alias("Default_Handler")));  // 16
void TCC0_Handler()       __attribute__((weak, alias("Default_Handler")));  // 17
void TCC1_Handler()       __attribute__((weak, alias("Default_Handler")));  // 18
void TCC2_Handler()       __attribute__((weak, alias("Default_Handler")));  // 19
void TC0_Handler()        __attribute__((weak, alias("Default_Handler")));  // 20
void TC1_Handler()        __attribute__((weak, alias("Default_Handler")));  // 21
void TC2_Handler()        __attribute__((weak, alias("Default_Handler")));  // 22
void TC3_Handler()        __attribute__((weak, alias("Default_Handler")));  // 23
void TC4_Handler()        __attribute__((weak, alias("Default_Handler")));  // 24
void ADC0_Handler()       __attribute__((weak, alias("Default_Handler")));  // 25
void ADC1_Handler()       __attribute__((weak, alias("Default_Handler")));  // 26
void AC_Handler()         __attribute__((weak, alias("Default_Handler")));  // 27
void DAC_Handler()        __attribute__((weak, alias("Default_Handler")));  // 28
void SDADC_Handler()      __attribute__((weak, alias("Default_Handler")));  // 29
void PTC_Handler()        __attribute__((weak, alias("Default_Handler")));  // 30

}  // extern "C"

// The table itself. Entry 0 is the initial stack pointer (the __stack_top
// "function" above), the rest are handler addresses; the hardware reads
// raw words.
using Handler = void (*)();

__attribute__((section(".vectors"), used))
constexpr Handler vector_table[] = {
    __stack_top,                               // 0: initial SP
    Reset_Handler,                             // 1: reset
    NMI_Handler,                               // 2
    HardFault_Handler,                         // 3
    nullptr, nullptr, nullptr, nullptr,        // 4..7 reserved on v6-M
    nullptr, nullptr, nullptr,                 // 8..10 reserved
    SVC_Handler,                               // 11
    nullptr, nullptr,                          // 12..13 reserved
    PendSV_Handler,                            // 14
    SysTick_Handler,                           // 15
    // Peripheral lines 0..30:
    SYSTEM_Handler, WDT_Handler, RTC_Handler, EIC_Handler,
    FREQM_Handler, TSENS_Handler, NVMCTRL_Handler, DMAC_Handler,
    EVSYS_Handler, SERCOM0_Handler, SERCOM1_Handler, SERCOM2_Handler,
    SERCOM3_Handler, SERCOM4_Handler, SERCOM5_Handler, CAN0_Handler,
    CAN1_Handler, TCC0_Handler, TCC1_Handler, TCC2_Handler,
    TC0_Handler, TC1_Handler, TC2_Handler, TC3_Handler,
    TC4_Handler, ADC0_Handler, ADC1_Handler, AC_Handler,
    DAC_Handler, SDADC_Handler, PTC_Handler,
};
