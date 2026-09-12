/*
 * sleep.hpp
 *
 * The stopping half of the CH32V00x platform: PWR (RM ch. 2) - the two
 * low-power modes, the regulator, the supply monitor, the auto-wakeup
 * unit - and the two sleep sites that run util/power.hpp's model over
 * them.
 *
 * TWO MODES, ONE BIT EACH. Sleep stops the core clock and nothing
 * else; Standby stops every high-frequency clock (HSE, HSI, PLL and
 * the peripheral clocks with them), keeps the SRAM, the registers and
 * the pins, drops the regulator to its low-power setting, and wakes
 * only through an EXTI line - a pad, the PVD, the AWU - or a reset
 * (RM 2.3.3). The choice is PFIC_SCTLR.SLEEPDEEP together with
 * PWR_CTLR.PDDS, and the platform's idle() then sleeps as it always
 * does: the site ARMS, the kernel loop's idle path sleeps, the model
 * of docs/design/power.md unchanged on the CH32V00x as on the others.
 *
 * WHAT A STANDBY DOES TO THIS PROGRAM, and what the sites answer:
 *  - THE CORE WAKES ON THE HSI. The hardware switches SYSCLK to the
 *    HSI and turns the PLL off, so the first instructions after the
 *    wake - the wake vector's handler included - run at the HSI's
 *    rate through whatever HPRE stands. disarm() restores the clock
 *    (Clock::restore(), DynamicClock::restore()); until then a
 *    peripheral clocked from HCLK runs at the wrong rate, which is
 *    why a program that talks on its USART across a Standby expects
 *    the first bytes after the wake to be garbage.
 *  - KERNEL TIME STANDS STILL. The STK stops with the core clock, so
 *    the plain site's rule is the SAM's: a deep sleep is legal only
 *    with no armed time event (the manager's deadline guard), and the
 *    TIMED site lifts that by placing the AWU as the alarm and handing
 *    the slept span back through Ticker::advance().
 *  - THE AWU IS AN ALARM WITHOUT A COUNTER TO READ. It counts the LSI
 *    through a prescaler up to a six-bit window and raises EXTI line 9
 *    at the match (RM 2.3.4, 2.4.3..2.4.5); its count is not readable.
 *    So the witness of a slept span is the alarm itself: when the wake
 *    WAS the AWU, the span is the interval that was programmed; when
 *    something else woke the core first, the span is unknown and the
 *    site advances by NOTHING - late maturation is legal, early is not,
 *    and a guess would risk early. A program that sleeps deep with
 *    other wake sources should size its deadlines knowing this.
 *  - THE LSI IS "ABOUT 128 kHz". The site MEASURES it at init(), awake,
 *    against the STK: a short AWU window timed on HCLK gives the rate
 *    the alarm will run at, and the counts for a deadline are computed
 *    from that rate rounded UP, so an alarm lands late rather than
 *    early (the kernel's own "at least").
 *
 * The regulator's modes (LDO_MODE), the flash's low-power setting and
 * the PVD are exposed as verbs of `Pwr` and judged by no site: what
 * they are worth is a current on the bench meter.
 *
 * THE TWO PARTS (device::): the modes, the AWU - its window, its
 * prescaler table, its line - and the PVD's flag are the same on the
 * CH32V003 and the CH32V006. What differs is the PVD's threshold
 * field (three bits from 2.85 to 4.4 V on the CH32V003, whose supply
 * runs to 5.5 V; two bits from 1.87 to 2.66 V on the CH32V006), so
 * `PvdLevel` names each part's own levels and `pvd_rising_mv()` says
 * what a level is; and the CH32V003 has neither LDO_MODE nor FLASH_LP,
 * whose verbs write nothing and read the reset state there.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/exti.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/ticker.hpp"
#include "kernel/platform.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"

namespace brio {

struct PwrRegs {
    volatile uint32_t CTLR;     ///< 0x00 PDDS, LDO_MODE, PVDE, PLS, FLASH_LP
    volatile uint32_t CSR;      ///< 0x04 PVD0
    volatile uint32_t AWUCSR;   ///< 0x08 AWUEN
    volatile uint32_t AWUWR;    ///< 0x0c the six-bit window
    volatile uint32_t AWUPSC;   ///< 0x10 the prescaler code
};

inline PwrRegs* pwr() { return reinterpret_cast<PwrRegs*>(pb1_base + 0x7000); }

inline constexpr uint32_t pwr_pdds         = 1UL << 1;
inline constexpr uint32_t pwr_ldo_mask     = 0x3UL << 2;
inline constexpr uint32_t pwr_ldo_low      = 0x1UL << 2;   ///< 1.0 V
inline constexpr uint32_t pwr_ldo_normal   = 0x2UL << 2;   ///< 1.2 V, the reset value
inline constexpr uint32_t pwr_ldo_saving   = 0x3UL << 2;   ///< 1.0 V, "energy saving"
inline constexpr uint32_t pwr_pvde         = 1UL << 4;
inline constexpr uint32_t pwr_pls_mask     = ((1UL << device::pvd_level_bits) - 1u) << 5;
inline constexpr uint32_t pwr_flash_lp_reg = 1UL << 9;
inline constexpr uint32_t pwr_pvd0         = 1UL << 2;     ///< in CSR: below the threshold
inline constexpr uint32_t pwr_awuen        = 1UL << 1;

/// PWR_CTLR.PLS: the PVD's threshold, each part's own table (RM 2.4.1
/// of each manual; the rising edge names the level, the falling one
/// sits 20 mV below on the CH32V006 and 150..200 mV below on the
/// CH32V003).
enum class PvdLevelCh32v006 : uint8_t { v1_87 = 0, v2_23 = 1, v2_43 = 2, v2_66 = 3 };
enum class PvdLevelCh32v003 : uint8_t { v2_85 = 0, v3_05 = 1, v3_3 = 2, v3_5 = 3, v3_7 = 4, v3_9 = 5, v4_1 = 6, v4_4 = 7 };
using PvdLevel = std::conditional_t<device::part == Ch32Part::v003, PvdLevelCh32v003, PvdLevelCh32v006>;

/// The rising threshold of a level, in millivolts.
constexpr uint32_t pvd_rising_mv(PvdLevel level) {
    if constexpr (device::part == Ch32Part::v003) {
        constexpr uint32_t table[8] = {2850, 3050, 3300, 3500, 3700, 3900, 4100, 4400};
        return table[static_cast<uint8_t>(level) & 7u];
    } else {
        constexpr uint32_t table[4] = {1870, 2230, 2430, 2660};
        return table[static_cast<uint8_t>(level) & 3u];
    }
}
/// The falling threshold of a level, in millivolts: where PVD0 clears
/// again once VDD climbs back (the hysteresis of each manual's table).
constexpr uint32_t pvd_falling_mv(PvdLevel level) {
    if constexpr (device::part == Ch32Part::v003) {
        constexpr uint32_t table[8] = {2700, 2900, 3150, 3300, 3500, 3700, 3900, 4200};
        return table[static_cast<uint8_t>(level) & 7u];
    } else {
        constexpr uint32_t table[4] = {1850, 2210, 2410, 2600};
        return table[static_cast<uint8_t>(level) & 3u];
    }
}
/// The lowest and the highest level of the part, for a program that
/// wants a threshold without naming a voltage.
inline constexpr PvdLevel pvd_level_lowest = static_cast<PvdLevel>(0);
inline constexpr PvdLevel pvd_level_highest = static_cast<PvdLevel>((1u << device::pvd_level_bits) - 1u);
static_assert(pvd_rising_mv(pvd_level_lowest) < pvd_rising_mv(pvd_level_highest));

/// The power controller, monostate. Its gate on the PB1 bus is opened
/// by every verb: a PWR register read through a closed gate answers
/// rubbish (0x3F on the bench), and a site that trusted it would arm
/// nothing.
struct Pwr {
    Pwr() = delete;

    static void open() { rcc()->PB1PCENR |= rcc_pb1_pwr; }

    /// Standby (true) or Sleep (false) for the next deep sleep instruction.
    static void standby(bool on) {
        open();
        if (on) { pwr()->CTLR |= pwr_pdds; } else { pwr()->CTLR &= ~pwr_pdds; }
    }
    static bool standby() { open(); return (pwr()->CTLR & pwr_pdds) != 0u; }

    /// The regulator's mode (one of the pwr_ldo_* codes): the CH32V006's
    /// alone (device::pwr_has_ldo_modes) - on the CH32V003 the verb
    /// writes nothing and reads the normal mode, which is all that
    /// part has.
    static void ldo(uint32_t mode) {
        if constexpr (device::pwr_has_ldo_modes) {
            open();
            pwr()->CTLR = (pwr()->CTLR & ~pwr_ldo_mask) | (mode & pwr_ldo_mask);
        } else {
            (void)mode;
        }
    }
    static uint32_t ldo() {
        if constexpr (device::pwr_has_ldo_modes) {
            open();
            return pwr()->CTLR & pwr_ldo_mask;
        } else {
            return pwr_ldo_normal;
        }
    }

    static void pvd(bool on, PvdLevel level = pvd_level_lowest) {
        open();
        uint32_t c = pwr()->CTLR & ~(pwr_pvde | pwr_pls_mask);
        c |= static_cast<uint32_t>(level) << 5;
        if (on) { c |= pwr_pvde; }
        pwr()->CTLR = c;
    }
    static bool pvd() { open(); return (pwr()->CTLR & pwr_pvde) != 0u; }
    /// True while VDD sits below the PVD's threshold.
    static bool supply_low() { open(); return (pwr()->CSR & pwr_pvd0) != 0u; }

    /// The flash's low-power setting: the CH32V006's alone
    /// (device::pwr_has_flash_low_power); nothing written on the CH32V003.
    static void flash_low_power(bool on) {
        if constexpr (device::pwr_has_flash_low_power) {
            open();
            if (on) { pwr()->CTLR |= pwr_flash_lp_reg; } else { pwr()->CTLR &= ~pwr_flash_lp_reg; }
        } else {
            (void)on;
        }
    }
};

/// PWR_AWUPSC: the LSI divider ahead of the window counter (RM 2.4.5).
/// Codes 0 and 1 both mean "off" (the LSI undivided).
constexpr uint32_t awu_prescaler_divider(uint8_t code) {
    return code < 2u ? 1u : code == 14u ? 10240u : code == 15u ? 61440u : (1UL << (code - 1u));
}

/// The auto-wakeup unit: a window counter on LSI/prescaler that raises
/// EXTI line 9. Monostate.
struct Awu {
    Awu() = delete;

    static constexpr uint8_t window_max = 63;   ///< AWUWR is six bits; the period is window + 1 counts

    /// Turn the LSI on and arm line 9 on its rising edge as an
    /// INTERRUPT line of the EXTI - which is what sets a flag a program
    /// can poll and a pending bit a WFE with SEVONPEND wakes on (RM
    /// 6.4.2's first shape) - without enabling it in the PFIC unless a
    /// handler is wanted (`with_interrupt`). The unit stays disabled.
    /// Returns false if the LSI never came ready.
    static bool init(bool with_interrupt = false) {
        Pwr::open();
        Rcc::lsi(true);
        for (uint32_t i = 0; i < 100'000u; ++i) {
            if (Rcc::lsi_ready()) {
                Exti::rising(exti_line_awu, true);
                Exti::interrupt(exti_line_awu, true);
                Exti::event(exti_line_awu, true);
                clear();
                if (with_interrupt) {
                    Pfic::enable(Irq::awu);
                } else {
                    Pfic::disable(Irq::awu);
                }
                return true;
            }
        }
        return false;
    }

    /// Program one period: `prescaler` a code of RM 2.4.5, `window` the
    /// six-bit value (period = window + 1 counts), then enable. The unit
    /// counts from the enable.
    static void arm(uint8_t prescaler, uint8_t window) {
        Pwr::open();
        pwr()->AWUCSR = 0;
        pwr()->AWUPSC = prescaler & 0xFu;
        pwr()->AWUWR = window & 0x3Fu;
        clear();
        pwr()->AWUCSR = pwr_awuen;
    }

    static void disarm() {
        Pwr::open();
        pwr()->AWUCSR = 0;
        clear();
    }

    static bool enabled() { Pwr::open(); return (pwr()->AWUCSR & pwr_awuen) != 0u; }
    static bool fired() { return Exti::flag(exti_line_awu); }
    /// The EXTI flag and the PFIC's pending bit both: a wake through
    /// SEVONPEND leaves the pending bit standing (RM 6.4.2), and a
    /// standing pending bit would end the next WFE at once.
    static void clear() {
        Exti::clear(exti_line_awu);
        Pfic::clear_pending(Irq::awu);
    }

    /// The period a code and window give at a measured LSI rate, in
    /// microseconds - for a program that wants to know.
    static constexpr uint32_t period_us(uint8_t prescaler, uint8_t window, uint32_t lsi_hz_measured) {
        return static_cast<uint32_t>(
            (static_cast<uint64_t>(awu_prescaler_divider(prescaler)) * (window + 1u) * 1'000'000u) /
            lsi_hz_measured);
    }
};

/**
 * The plain sleep site: the depth ladder onto this family's two modes.
 * `light` is Sleep; `standby` and `deep` both map to Standby, the
 * deepest this silicon has, so the never-deeper rule is the identity
 * for those two; `none` disarms. A Standby restores nothing about the
 * clock by itself - see the timed site for the whole story, and
 * `disarm()` here, which puts the tree back if it finds it fallen.
 */
template <typename Clock>
struct Ch32SleepSite {
    Ch32SleepSite() = delete;

    static bool arm(SleepDepth d) {
        switch (d) {
            case SleepDepth::none:
                disarm();
                return true;
            case SleepDepth::light:
                pfic_sctlr() &= ~sctlr_sleepdeep;
                Pwr::standby(false);
                armed_ = SleepDepth::light;
                return true;
            case SleepDepth::standby:
            case SleepDepth::deep:
                Pwr::standby(true);
                pfic_sctlr() |= sctlr_sleepdeep;
                armed_ = SleepDepth::standby;
                return true;
        }
        return false;
    }

    static void disarm() {
        pfic_sctlr() &= ~sctlr_sleepdeep;
        Pwr::standby(false);
        const bool was_deep = armed_ == SleepDepth::standby;
        armed_ = SleepDepth::none;
        // A Standby that ran leaves SYSCLK on the HSI with the PLL off:
        // the tree goes back to what the program's clock type says.
        // Idempotent and cheap when the sleep never went deep or never
        // happened, so it is not made conditional on either.
        if (was_deep) {
            (void)Clock::restore();
        }
    }

    static SleepDepth armed() { return armed_; }

private:
    static inline SleepDepth armed_ = SleepDepth::none;
};

static_assert(SleepSite<Ch32SleepSite<Clock<ClockSource::pll, 48'000'000>>>);

/**
 * The timed site: the plain site plus the AWU as the alarm that lifts
 * the no-deadline rule from a Standby. arm() places the nearest armed
 * time event's distance on the AWU, rounded UP in the LSI's measured
 * rate; disarm() advances kernel time by the programmed span when the
 * AWU is what fired, and by nothing otherwise (see the file header).
 *
 * The app binds the AWU vector when it wants a handler; the site
 * itself needs none - line 9 is armed as an EVENT and ends the WFE.
 */
template <Platform P, typename Clock>
struct Ch32TimedSleepSite {
    Ch32TimedSleepSite() = delete;

    using Plain = Ch32SleepSite<Clock>;

    /// The LSI on and MEASURED against the STK: one AWU period of 64
    /// undivided counts timed in HCLK cycles. About half a millisecond
    /// at 128 kHz; the result is what the alarm arithmetic uses.
    static bool init() {
        if (!Awu::init()) {
            return false;
        }
        Awu::arm(0, Awu::window_max);
        const uint32_t start = stk()->CNT;
        uint32_t polls = 0;
        while (!Awu::fired() && polls < 10'000'000u) {
            ++polls;
        }
        const uint32_t end = stk()->CNT;
        Awu::disarm();
        if (!(polls < 10'000'000u)) {
            return false;
        }
        const uint32_t period = stk()->CMP + 1u;
        uint32_t cycles = end >= start ? end - start : end + period - start;
        // Sixty-four LSI counts in `cycles` HCLK cycles: the rate, in Hz.
        // A window that crossed a tick reload once is folded in; a
        // measurement is under one tick at any HCLK this family runs.
        lsi_hz_ = static_cast<uint32_t>(
            (static_cast<uint64_t>(clock_hz(Clock{})) * (Awu::window_max + 1u)) / cycles);
        ready_ = lsi_hz_ > 0u;
        return ready_;
    }

    static uint32_t lsi_hz() { return lsi_hz_; }

    static bool arm(SleepDepth d) {
        if (!Plain::arm(d)) {
            return false;
        }
        if (!ready_ || !is_deep_mode(Plain::armed())) {
            return true;
        }
        const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
        if (!next.has_value()) {
            return true;   // nothing waiting: the sleep may last
        }
        // ticks -> LSI counts at the measured rate, rounded UP, then the
        // coarsest prescaler that keeps the window in six bits. The
        // rounding is the "at least": the alarm lands late, never early.
        const uint64_t counts =
            (static_cast<uint64_t>(*next) * lsi_hz_ + P::ticks_per_second - 1u) /
            P::ticks_per_second;
        uint8_t code = 0;
        while (code < 15u && counts / awu_prescaler_divider(code) > Awu::window_max + 1u) {
            ++code;
        }
        const uint32_t window_counts = static_cast<uint32_t>(
            (counts + awu_prescaler_divider(code) - 1u) / awu_prescaler_divider(code));
        const uint8_t window = window_counts == 0u ? 0u
                             : static_cast<uint8_t>(window_counts > Awu::window_max + 1u
                                                        ? Awu::window_max : window_counts - 1u);
        span_ticks_ = static_cast<uint32_t>(
            (static_cast<uint64_t>(awu_prescaler_divider(code)) * (window + 1u) *
             P::ticks_per_second) / lsi_hz_);
        prescaler_ = code;
        window_ = window;
        tick_at_arm_ = Ticker::ticks();
        Awu::arm(code, window);
        alarm_armed_ = true;
        return true;
    }

    /// Catch kernel time up by the FROZEN span: the alarm's programmed
    /// span minus what the STK itself counted since arm() - the AWU
    /// counts from arm(), the core may stay awake a while before it
    /// sleeps and after it wakes, and those ticks were delivered.
    /// Advancing them again would mature events EARLY, which the time
    /// contract forbids. Nothing is advanced when the alarm did not
    /// fire: the span is then unknown (see the file header).
    static void disarm() {
        if (alarm_armed_) {
            const bool by_alarm = Awu::fired();
            Awu::disarm();
            alarm_armed_ = false;
            const uint32_t awake = Ticker::ticks() - tick_at_arm_;   // wrap-safe
            if (by_alarm && span_ticks_ > awake) {
                last_advance_ = span_ticks_ - awake;
                Ticker::advance(last_advance_);
            } else {
                last_advance_ = 0;
            }
        }
        Plain::disarm();
    }

    /// What the last arm() placed: for a program that wants to say so.
    static uint32_t span_ticks() { return span_ticks_; }
    static uint8_t prescaler_code() { return prescaler_; }
    static uint8_t window() { return window_; }

    static SleepDepth armed() { return Plain::armed(); }

    /// What the last disarm() handed the ticker: the alarm's span, or 0
    /// when something else woke the core first.
    static uint32_t last_advance() { return last_advance_; }

private:
    static inline bool ready_ = false;
    static inline bool alarm_armed_ = false;
    static inline uint32_t lsi_hz_ = 0;
    static inline uint32_t span_ticks_ = 0;
    static inline uint32_t last_advance_ = 0;
    static inline uint32_t tick_at_arm_ = 0;
    static inline uint8_t prescaler_ = 0;
    static inline uint8_t window_ = 0;
};

} // namespace brio
