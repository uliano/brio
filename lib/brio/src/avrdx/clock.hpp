/*
 * clock.hpp
 *
 * Clock<source, source_hz, div>: the main clock of an AVR DA/DB as a
 * TYPE, and the ONE truth about the CPU/peripheral frequency that every
 * other driver of this target derives its divisors from (Uart baud,
 * Twi baud, delay_us cycles). Nothing here is a list of allowed
 * frequencies: the parameters are the configuration of the clock tree
 * (which oscillator, at what rate, through which main prescaler) and
 * `hz` is constexpr arithmetic on them, with the datasheet's limits
 * as static_asserts. That is the shape a richer tree (PLL, per-bus
 * dividers) takes on other targets too: more parameters, more
 * arithmetic, more asserts, same idea, all inside that target's file.
 *
 * Sources (ClockSource):
 *  - internal: OSCHF at one of the frequencies its FRQSEL can produce
 *    (1, 2, 3, 4, 8, 12, 16, 20, 24 MHz), no external parts;
 *  - crystal:  a high-frequency crystal on PA0/PA1 (XOSCHF, DB family
 *    only; PA0/PA1 stop being GPIO). If it fails to start, init() falls
 *    back to OSCHF at the SAME frequency - which is why a crystal rate
 *    must be one OSCHF can produce: Clock::hz stays true either way
 *    and init() just reports which source is running;
 *  - external: a clock signal on PA0 (EXTCLK), DA and DB. No fallback
 *    other than the same OSCHF rule.
 * The main prescaler (ClockDiv) divides the source for CLK_PER = CLK_CPU:
 * hz = source_hz / div. On AVR Dx CPU and peripherals share one clock
 * domain, so there is one `hz`; a target with several domains exposes
 * several (hclk, pclk1, ...), one truth each.
 *
 * `is_static` = true: hz is a compile-time constant. The runtime regime
 * is DynamicClock<Boot, Users...> below: same source as the static Boot
 * configuration, main prescaler changed at run time, `hz()` a value,
 * and every change fanned out SYNCHRONOUSLY to the listed Users (types
 * with a static rebase(hz)) before set() returns - publish semantics,
 * fold mechanics: a queued event would reach a low-priority driver
 * after it had already run at the wrong rate. Drivers take either kind
 * through the same init(clock) and read the rate with clock_hz(clock),
 * constexpr for the static kind. The RTC/PIT timebase (kernel ticks,
 * time events) never moves.
 *
 * F_CPU. This project does not define it at all (platformio.ini unflags
 * the -DF_CPU PlatformIO would pass from the board manifest): the rate
 * has one truth, Clock::hz, and avr-libc's util/delay.h / setbaud.h -
 * which need F_CPU - therefore do not compile, on purpose: nothing can
 * assume a rate Clock does not state. brio code never reads F_CPU: it
 * takes Clock (as a tag object) where it needs the rate. Should brio be
 * built in a project that still defines F_CPU, the static_assert below
 * refuses a value different from hz - two truths never diverge silently.
 *
 * Usage:
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;            // an empty tag, like `serial`
 *   ...
 *   const bool from_crystal = SysClock::init();   // first line of main()
 *   Serial::init(clock, 460800);
 *   brio::delay_us(clock, 10);
 *
 * CLKCTRL registers are CCP-protected, hence _PROTECTED_WRITE. The RTC
 * (kernel timebase) is NOT touched here: it runs from OSC32K/XOSC32K,
 * independent of the main clock (see ticker.hpp).
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <concepts>

namespace brio {

enum class ClockSource : uint8_t { internal, crystal, external };

/// Main prescaler (CLKCTRL.MCLKCTRLB). div1 = prescaler disabled.
enum class ClockDiv : uint8_t {
    div1 = 0,
    div2 = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm,
    div4 = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm,
    div6 = CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm,
    div8 = CLKCTRL_PDIV_8X_gc | CLKCTRL_PEN_bm,
    div10 = CLKCTRL_PDIV_10X_gc | CLKCTRL_PEN_bm,
    div12 = CLKCTRL_PDIV_12X_gc | CLKCTRL_PEN_bm,
    div16 = CLKCTRL_PDIV_16X_gc | CLKCTRL_PEN_bm,
    div24 = CLKCTRL_PDIV_24X_gc | CLKCTRL_PEN_bm,
    div32 = CLKCTRL_PDIV_32X_gc | CLKCTRL_PEN_bm,
    div48 = CLKCTRL_PDIV_48X_gc | CLKCTRL_PEN_bm,
    div64 = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm,
};

constexpr uint32_t clock_divisor(ClockDiv d) {
    switch (d) {
    case ClockDiv::div1: return 1;
    case ClockDiv::div2: return 2;
    case ClockDiv::div4: return 4;
    case ClockDiv::div6: return 6;
    case ClockDiv::div8: return 8;
    case ClockDiv::div10: return 10;
    case ClockDiv::div12: return 12;
    case ClockDiv::div16: return 16;
    case ClockDiv::div24: return 24;
    case ClockDiv::div32: return 32;
    case ClockDiv::div48: return 48;
    case ClockDiv::div64: return 64;
    }
    return 0;
}

/// OSCHF FRQSEL code for a rate it can produce, 0xFF otherwise.
constexpr uint8_t oschf_frqsel(uint32_t hz) {
    switch (hz) {
    case 1'000'000: return CLKCTRL_FRQSEL_1M_gc;
    case 2'000'000: return CLKCTRL_FRQSEL_2M_gc;
    case 3'000'000: return CLKCTRL_FRQSEL_3M_gc;
    case 4'000'000: return CLKCTRL_FRQSEL_4M_gc;
    case 8'000'000: return CLKCTRL_FRQSEL_8M_gc;
    case 12'000'000: return CLKCTRL_FRQSEL_12M_gc;
    case 16'000'000: return CLKCTRL_FRQSEL_16M_gc;
    case 20'000'000: return CLKCTRL_FRQSEL_20M_gc;
    case 24'000'000: return CLKCTRL_FRQSEL_24M_gc;
    default: return 0xFF;
    }
}

template <ClockSource src, uint32_t src_hz, ClockDiv div = ClockDiv::div1>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t source_hz = src_hz;
    static constexpr uint32_t divisor = clock_divisor(div);
    static constexpr uint32_t hz = src_hz / divisor;   ///< CLK_PER = CLK_CPU
    static constexpr bool is_static = true;

    static_assert(divisor > 0, "invalid main prescaler");
    static_assert(oschf_frqsel(src_hz) != 0xFF,
                  "brio Clock: the source rate must be one OSCHF can produce "
                  "(1/2/3/4/8/12/16/20/24 MHz) - it is the fallback that keeps "
                  "Clock::hz true when an external source fails to start");
    static_assert(src_hz % divisor == 0, "source rate not divisible by the prescaler");
#if defined(F_CPU)
    static_assert(hz == F_CPU,
                  "brio Clock: this build defines F_CPU with a value different "
                  "from Clock::hz. Either unflag -DF_CPU (this project does) or "
                  "make it equal: never two truths");
#endif
#if !defined(CLKCTRL_XOSCHFCTRLA)
    static_assert(src != ClockSource::crystal,
                  "brio Clock: XOSCHF (HF crystal) exists only on the DB family");
#endif

    /// Bring CLK_PER to `hz`. Returns true when running from the
    /// requested source, false when an external source failed to start
    /// and OSCHF (same rate) is running instead. Call first in main().
    static bool init() {
        // Baseline and fallback: OSCHF at the target rate, prescaler set.
        _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA,
                         static_cast<uint8_t>(oschf_frqsel(src_hz) | CLKCTRL_RUNSTDBY_bm));
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, static_cast<uint8_t>(div));
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
        (void)wait_status(CLKCTRL_OSCHFS_bm, 0xFFFFu);

        if constexpr (src == ClockSource::internal) {
            return true;
        } else {
            if (!start_external()) {
#if defined(CLKCTRL_XOSCHFCTRLA)
                _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, 0);   // release PA0/PA1
#endif
                return false;                               // OSCHF fallback, same hz
            }
            _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
            if (!wait_status(CLKCTRL_EXTS_bm, 0xFFFFu)) {
                _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
                return false;                               // OSCHF fallback, same hz
            }
            return true;
        }
    }

private:
    static bool wait_status(uint8_t mask, uint32_t timeout) {
        while (timeout--) {
            if (CLKCTRL.MCLKSTATUS & mask) {
                return true;
            }
        }
        return false;
    }

#if defined(CLKCTRL_XOSCHFCTRLA)
    static constexpr uint8_t frqrange() {
        return src_hz <= 8'000'000 ? CLKCTRL_FRQRANGE_8M_gc
             : src_hz <= 16'000'000 ? CLKCTRL_FRQRANGE_16M_gc
             : src_hz <= 24'000'000 ? CLKCTRL_FRQRANGE_24M_gc
                                    : CLKCTRL_FRQRANGE_32M_gc;
    }
#endif

    // Start the external source and wait for it: crystal (4k-cycle
    // start-up) or external clock (256 cycles).
    static bool start_external() {
#if defined(CLKCTRL_XOSCHFCTRLA)
        constexpr uint8_t sel = (src == ClockSource::crystal)
            ? static_cast<uint8_t>(CLKCTRL_SELHF_XTAL_gc) | static_cast<uint8_t>(CLKCTRL_CSUTHF_4K_gc)
            : static_cast<uint8_t>(CLKCTRL_SELHF_EXTCLOCK_gc) | static_cast<uint8_t>(CLKCTRL_CSUTHF_256_gc);
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA,
                         static_cast<uint8_t>(CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm |
                                              sel | frqrange()));
        return wait_status(CLKCTRL_EXTS_bm, 0xFFFFu);
#else
        // DA: EXTCLK on PA0 is selected directly by MCLKCTRLA; there is
        // no start-up to wait for, only the status flag once selected.
        return true;
#endif
    }
};

/// The rate of any clock type: a constant for a static Clock, a value
/// for a DynamicClock. Drivers write `clock_hz(clock)` and get folding
/// for free when it can fold.
template <typename C>
constexpr uint32_t clock_hz(C) {
    if constexpr (C::is_static) {
        return C::hz;
    } else {
        return C::hz();
    }
}

/// For a clocked driver's init(clock): true when the clock is static
/// (nothing to follow) or when the dynamic clock lists Driver among the
/// users it rebases. static_assert(clock_follows<Clock, Driver>()).
template <typename C, typename Driver>
constexpr bool clock_follows() {
    if constexpr (C::is_static) {
        return true;
    } else {
        return C::template rebases<Driver>;
    }
}

/**
 * The runtime regime. Boot is a static Clock<...> naming the source and
 * its rate; set(div) fans the new rate out to Users (each a type with
 * `static void rebase(uint32_t hz)`) in list order, synchronously, and
 * THEN reprograms the main prescaler - so a user can drain what it has
 * in flight at the old rate before adopting the new one. Call set() only when nothing that depends
 * on the rate is mid-transfer: a driver's rebase() may wait for its own
 * hardware to go idle (Uart drains TX first), but a bus transaction in
 * flight is the caller's problem - in an AO system, ask the bus AOs
 * (idle state) before switching. F_CPU must not be defined for this
 * type: no rate is fixed any more.
 *
 *   using Boot = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   using SysClock = brio::DynamicClock<Boot, Serial, Twi0>;
 *   constexpr SysClock clock;
 *   ...
 *   SysClock::init();                 // Boot's init: 24 MHz
 *   SysClock::set(brio::ClockDiv::div6);   // 4 MHz, Serial and Twi0 rebased
 */
template <typename Boot, typename... Users>
struct DynamicClock {
    static constexpr ClockSource source = Boot::source;
    static constexpr uint32_t source_hz = Boot::source_hz;
    static constexpr bool is_static = false;
    static_assert(Boot::divisor == 1, "DynamicClock: give Boot the source rate with div1; "
                                      "the prescaler is what set() changes");
#if defined(F_CPU)
    static_assert(false, "brio DynamicClock: F_CPU must not be defined with a "
                         "runtime clock (build_unflags = -DF_CPU)");
#endif

    static uint32_t hz() { return hz_; }
    static ClockDiv div() { return div_; }

    /// Is U one of the users that set() rebases? Drivers assert this in
    /// init(clock): a clocked driver forgotten in the list would keep
    /// running at the old rate in silence - a compile error instead.
    template <typename U>
    static constexpr bool rebases = (std::same_as<U, Users> || ...);

    /// Boot configuration, then Boot::hz. See Clock::init for the return.
    static bool init() {
        const bool ok = Boot::init();
        hz_ = Boot::hz;
        div_ = ClockDiv::div1;
        return ok;
    }

    /// Rebase every user for the new rate (each may first drain what
    /// it has in flight at the OLD rate), then switch the prescaler.
    /// Between a user's rebase and the switch nothing may transmit -
    /// true in main context, where this must be called.
    static void set(ClockDiv d) {
        const uint32_t next = source_hz / clock_divisor(d);
        (Users::rebase(next), ...);
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, static_cast<uint8_t>(d));
        div_ = d;
        hz_ = next;
    }

private:
    static inline uint32_t hz_ = Boot::hz;
    static inline ClockDiv div_ = ClockDiv::div1;
};

} // namespace brio
