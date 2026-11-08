// bench_sleep.hpp - the sleeping legs' shared tools of the samc21 suites:
// the RTC on OSCULP32K as a counted BACKSTOP, and the race-free sleep
// until a condition the letter names.
//
// APP-LEVEL BENCH TOOLING, not framework: included by its plain name by
// the suites beside it. A suite that sleeps a bus waiting for a foreign
// edge needs both halves - a wake-up nobody should need, armed ahead of
// every sleep and COUNTED (a leg the backstop ended is a failed wake, not
// a slow one), and a sleep that cannot lose the edge it waits for.
//
// THE BACKSTOP: the RTC in 32-bit counter mode on OSCULP32K, the wake-up
// test_samc_sleep proves in STANDBY. One compare (COMP0) armed `ticks` of
// 32768 Hz ahead; its vector's body is backstop_isr(), bound by the app.
//
// THE SLEEP (sleep_until): PRIMASK set, the condition re-tested, WFI -
// which wakes on a pending interrupt even masked - and the unmask lets
// the handler run; repeated until the condition holds or the backstop
// fired, at most 32 times. The tick's interrupt is off for the whole wait
// (SysTickInterruptGuard), so only the source under test, the console and
// the backstop can end a sleep - and the console is drained first, for at
// most 200 ms on the kernel tick (util/test_wait.hpp): its transmitter's
// DRE is a wake-up too (measured: an unfinished line woke the core once a
// character), so a console that does not drain ends the leg with no sleep.
// The NVIC's pending word at the first wake-up is kept: which lines woke
// it.

#pragma once

#include <stdint.h>

#include "samc21/nvic.hpp"
#include "samc21/osc32kctrl.hpp"
#include "samc21/rtc.hpp"
#include "samc21/sleep.hpp"
#include "samc21/ticker.hpp"
#include "util/test_wait.hpp"

namespace benchsleep {

inline volatile bool rtc_fired = false;
/// NVIC->ISPR[0] at the first wake-up of the last sleep_until().
inline uint32_t first_pending = 0;

/// The kernel tick read as microseconds, a millisecond a step: the ruler
/// of the console drain below. It needs no CPU rate - the ticker's rate is
/// the project's constant, 1000 ticks a second (samc21/ticker.hpp) - so it
/// serves every suite that includes this file whatever its clock, and it
/// needs what each of them has at the call: the tick running and
/// interrupts enabled. The product wraps with the tick count, and the
/// difference of two reads stays exact (util/test_wait.hpp, "Half the
/// wrap").
struct TickRuler {
    static uint32_t now() { return brio::Ticker::millis() * 1000u; }
    static constexpr uint32_t hz() { return 1'000'000u; }
};

/// The RTC up on OSCULP32K, 32-bit counter, COMP0 armed as the interrupt.
inline bool backstop_up() {
    using namespace brio;
    if (!Rtc::init()) return false;
    (void)Rtc::enable(false);
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    if (!Rtc::init()) return false;
    if (!Rtc::configure(RtcConfig{.mode = RtcMode::count32, .prescaler = RtcPrescaler::div1})) {
        return false;
    }
    if (!Rtc::enable(true)) return false;
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::compare0);
    Nvic::enable(Rtc::irq());
    return true;
}

/// COMP0 `ticks` of 32768 Hz ahead; false when it is not in the future
/// once armed (no sleep on it then).
inline bool backstop_arm(uint32_t ticks) {
    using namespace brio;
    const uint32_t c = Rtc::count32();
    rtc_fired = false;
    Rtc::clear_flags(RtcFlag::compare0);
    if (!Rtc::set_comp32(c + ticks)) return false;
    return static_cast<int32_t>(Rtc::count32() - (c + ticks)) < 0;
}

inline void backstop_down() { brio::Rtc::disarm(brio::RtcFlag::all); }

/// The RTC vector's body.
[[gnu::always_inline]] inline void backstop_isr() {
    (void)brio::Rtc::isr();
    rtc_fired = true;
}

/// Sleep in `mode` until `done()` or the backstop; returns the WFIs it
/// took (0: the console did not drain within 200 ms, or the mode could
/// not be set - no sleep either way). The SLEEPCFG goes back to IDLE0.
template <typename Serial, typename Done>
uint8_t sleep_until(brio::SleepMode mode, Done done) {
    using namespace brio;
    uint8_t wakes = 0;
    if (!brio::console_drain<TickRuler, Serial>()) return 0;
    if (!Pm::set_sleep_mode(mode)) return 0;
    SysTickInterruptGuard quiet_tick;
    first_pending = 0;
    while (!done() && !rtc_fired && wakes < 32u) {
        __disable_irq();
        if (!done() && !rtc_fired) {
            Pm::sleep();
            if (wakes == 0) first_pending = NVIC->ISPR[0];
            ++wakes;
        }
        __enable_irq();
    }
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    return wakes;
}

}  // namespace benchsleep
