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
 * `is_static` = true: hz is a compile-time constant. A future dynamic
 * clock (runtime prescaler changes for power) is a different type with
 * is_static = false and a synchronous rebase fan-out to every clocked
 * driver - see docs/design/overview.md, "Target strata". Nothing above
 * this file assumes either.
 *
 * F_CPU. The build still defines F_CPU (PlatformIO passes the board's
 * f_cpu; avr-libc's util/delay.h wants it) and legacy code may still
 * read it. To make sure the two truths cannot diverge, Clock
 * static_asserts F_CPU == hz whenever F_CPU is defined: configure a
 * clock the board file does not expect and the build stops with a
 * message, instead of an UART at the wrong baud. brio code itself never
 * reads F_CPU: it takes Clock (as a tag object) where it needs the rate.
 * (A future dynamic clock inverts the guard: F_CPU must NOT be defined
 * - build_unflags = -DF_CPU - so nothing can assume a fixed rate.)
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
                  "brio Clock: F_CPU (board_build.f_cpu / build flag) disagrees "
                  "with Clock::hz. The build's F_CPU must equal the configured "
                  "clock: fix the board file or the Clock parameters, never "
                  "leave two truths");
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

} // namespace brio
