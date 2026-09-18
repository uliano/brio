/*
 * delay.hpp
 *
 * The microsecond busy-wait of this target: AT LEAST `us`, never early,
 * capped below one kernel tick - the contract every brio target states
 * (docs/design/clock.md), here with ONE implementation for both
 * instruction sets.
 *
 * WHY THE RULER IS THE PLATFORM TIMER AND NOT A CORE COUNTER. The other
 * Cortex-M families ride SysTick's VAL: it is a cycle counter in all but
 * name, and cortexm/delay.hpp turns microseconds into cycles through the
 * clock's rate. That path exists on this chip's Cortex-M33 half and on
 * neither of its Hazard3 harts, which have no SysTick at all - so taking
 * it would mean two implementations, two resolutions and two failure
 * modes under one name, on a target whose whole claim is that one source
 * runs on two architectures. The platform timer of 3.1.8 is the other
 * choice and it is the better one: a 64-bit counter in the SIO, single
 * cycle to read from either architecture, counting the MICROSECOND TICK
 * the generator block of 8.5 divides out of clk_ref (rp2350/mtime.hpp).
 *
 * WHAT THAT BUYS, AND WHAT IT COSTS. It buys a wait that does not depend
 * on clk_sys at all: no rate arithmetic, no division anywhere, no table
 * per rate, and a wait that stays honest across a clk_sys switch that
 * happens while it runs. It costs RESOLUTION: the counter steps once a
 * microsecond, its phase against the call is unknown, so a request of
 * `us` waits for `us + 1` counts. Every wait is therefore up to one
 * microsecond long - always LATE, which is the direction the contract
 * asks for, and it is the price of a ruler that is the same on both
 * halves.
 *
 * THE CAP IS ONE MILLISECOND, one kernel tick at this target's 1000 Hz,
 * and it is the design and not a limitation: in a cooperative kernel a
 * dispatch that busy-waits for milliseconds starves every other active
 * object, so a wait of a tick or more is TimeEvent territory
 * (kernel/time_event.hpp) and this file REFUSES it - false, and no time
 * spent - instead of serving it. What remains is what a busy-wait is
 * for: a chip-select setup, an analog settle, a protocol gap.
 *
 * WHAT IT REFUSES BESIDES. A rate of zero (a clock that claims nothing
 * is not a clock), and a ruler that is not running: `Mtime::start(clock)`
 * is this target's one prerequisite for a wait, called by the program
 * that wants one - and already called for you on the Hazard3 half, where
 * the kernel timebase rides the same counter. Nothing here falls back to
 * a counted loop: a loop could only promise "at least" by overshooting
 * wildly, and a caller who wants one can write one.
 *
 * `DelayRate` and `delay_rate(hz)` exist because the IP strata this
 * chip's SPI and I2C are written in state their busy-wait in those words
 * (`brio/pl022/spi.hpp`'s Pl022ChipDelay, `brio/dw_apb_i2c/i2c.hpp`'s
 * ruler members), and because the other families spell the same pair.
 * Here the type carries no rate: a microsecond is a microsecond whatever
 * clk_sys is doing. What it does carry is the one thing the wait still
 * has to know from its caller, which is whether that caller had a clock
 * at all.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/mtime.hpp"
#include "util/clock.hpp"

namespace brio {

/// The first request this file refuses: one kernel tick at 1000 Hz.
inline constexpr uint32_t delay_us_cap = 1000;

/// The precomputed half of a wait. On the families whose ruler counts
/// CPU cycles this holds cycles per microsecond; here the ruler counts
/// microseconds, so all that is left of a rate is whether there was one.
struct DelayRate {
    bool usable = false;
};

/// A clock's rate as a DelayRate. Zero is not a rate, and a wait made
/// from one is refused rather than served wrongly.
constexpr DelayRate delay_rate(uint32_t hz) { return {hz != 0u}; }

/// Busy-wait AT LEAST `us` microseconds on the platform timer.
///
/// True when the time was served; false - AND NO TIME IS SPENT - when
/// the rate is zero, when the request is one kernel tick (1000 us) or
/// more, or when the ruler is not running (nothing called
/// `Mtime::start(clock)` in this program).
///
/// Callable with interrupts masked and from any context allowed to
/// spend the time. The elapsed time includes the interruptions a
/// busy-wait suffers: an ISR that fires mid-wait lengthens it, which is
/// the only honest reading of "at least" on a machine with interrupts.
[[nodiscard]] inline bool delay_us(DelayRate rate, uint32_t us) {
    if (!rate.usable || us >= delay_us_cap) {
        return false;
    }
    if (!Mtime::running()) {
        return false;   // no ruler: nothing here can count time
    }
    if (us == 0u) {
        return true;
    }
    // One count more than asked: the counter's phase against this call
    // is unknown, so `us` counts could be as little as us - 1
    // microseconds of real time. The low half alone is read - one bus
    // access - and the difference is taken in 32 bits, which folds its
    // wrap (71 minutes) correctly.
    const uint32_t start = Mtime::micros();
    const uint32_t wait = us + 1u;
    while (Mtime::micros() - start < wait) {
    }
    return true;
}

/// The same wait from a clock type: the rate is asked of the clock for
/// the contract's sake and for the refusal of a zero one, and nothing
/// else - the ruler is the same microsecond whatever the clock says.
template <typename C>
[[nodiscard]] bool delay_us(C clock, uint32_t us) {
    return delay_us(delay_rate(clock_hz(clock)), us);
}

} // namespace brio
