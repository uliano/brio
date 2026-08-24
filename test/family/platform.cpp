// Platform family smoke TU: the AVR realization of the kernel's
// Platform concept (AvrPlatform), the short-wait role (delay.hpp) and
// the reset/watchdog pair (reset.hpp). SLPCTRL, RSTCTRL and the WDT are
// identical on every DA/DB package - the point of this TU is that the
// concept check and both delay paths compile everywhere, including on
// the DA where CLKCTRL's CFD block does not exist.
#include "avrdx/delay.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "kernel/panic.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 24'000'000>;
using DynClock = DynamicClock<SysClock>;

static_assert(Platform<AvrPlatform>);
static_assert(AvrPlatform::atomic_width == 1);
static_assert(AvrPlatform::ticks_per_second == 1024);

void platform_verbs() {
    AvrPlatform::CriticalSection cs;
    (void)AvrPlatform::now();
    (void)AvrPlatform::sleep_armed();
    (void)AvrPlatform::interrupts_enabled();
    AvrPlatform::break_here();
    AvrPlatform::idle();
    (void)take_panic_record<AvrPlatform>();
}

void delay_paths(uint32_t runtime_us) {
    constexpr SysClock stat;
    delay_us(stat, 25);              // folded: __builtin_avr_delay_cycles
    delay_us(stat, runtime_us);      // the 4-cycle loop from a constexpr rate
    delay_us(DynClock{}, 25);        // a dynamic clock is always the loop
    delay_us_runtime(cycles_per_us(2'000'000), runtime_us);
    delay_cycles(runtime_us);
    static_assert(cycles_per_us(24'000'000) == 24);
    static_assert(cycles_per_us(1'500'000) == 2);   // rounds UP: "at least"
    static_assert(cycles_per_us(32'768) == 1);
}

void reset_verbs() {
    const ResetFlags f = Reset::take_flags();
    if (f.watchdog || Reset::flags().software) {
        Reset::clear_flags();
    }
    (void)Watchdog::arm(WdtTime::ms8);
    (void)Watchdog::arm(WdtTime::s8, WdtTime::ms16);
    Watchdog::clear();
    (void)Watchdog::busy();
    (void)Watchdog::locked();
    (void)Watchdog::enabled();
    (void)Watchdog::period();
    (void)Watchdog::window();
    (void)Watchdog::off();
    static_assert(wdt_time_us(WdtTime::off) == 0);
    static_assert(wdt_time_us(WdtTime::ms8) == 7812);
    static_assert(wdt_time_us(WdtTime::s8) == 8'000'000);
}
