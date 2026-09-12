/*
 * clock.hpp
 *
 * The RP2040 clock tree (datasheet 2.15 to 2.18) in the two strata every
 * brio target uses (docs/design/clock.md):
 *
 *  RESOURCES - monostates, thin typed views with the discipline built in:
 *    Xosc     the crystal oscillator (2.16): the 1..15 MHz range code,
 *             the startup delay in units of 256 crystal periods, the
 *             STABLE flag a bounded wait watches
 *    PllSys   the system PLL (2.18): REFDIV, FBDIV, the two post
 *             dividers, the power bits, the LOCK flag - and PllUsb, the
 *             same block at the USB PLL's address (48 MHz for the USB
 *             controller and for the converter's clk_adc)
 *    Rosc     the ring oscillator (2.17): running/stable, start and stop
 *             - never a rate, so never a Clock
 *    Clocks   the clock generators (2.15): clk_ref and clk_sys with their
 *             GLITCHLESS mux and their aux mux, clk_peri, clk_adc and
 *             clk_rtc with their aux mux alone
 *             and enable - and the switching sequences of 2.15.3.2,
 *             which are the whole point of the block
 *    FreqCounter  the frequency counter (2.15.4): any root or generator
 *             counted against clk_ref - the chip's own ratio check,
 *             and what the bench suite measures every rate with
 *    ClockOut<n> / ClockIn<n>   the four GPIO clock outputs (GP21, GP23,
 *             GP24, GP25) and the two inputs (GP20, GP22): a source
 *             through a divider onto a pin, a pin as a source the
 *             generators and the counter can name
 *
 *  TASK - what an application names:
 *    Clock<source, hz, crystal_hz, peri>   the static main clock: ONE
 *             constexpr truth `hz` every driver derives from (there is
 *             no F_CPU in this build); init() composes the resources and
 *             reports whether the tree reached the rate it claims;
 *             `peri` says whether clk_peri follows clk_sys (the default,
 *             pclk_hz == hz) or sits on the crystal (pclk_hz ==
 *             crystal_hz: the UARTs keep their rate through every
 *             change of clk_sys, the datasheet's reason for clk_peri).
 *
 * THE CLOCK MODEL, and what crosses the util contract. This chip has NO
 * shared bus prescaler and NO enable bit per peripheral: there is one
 * clk_sys for the cores, the bus fabric, the memories and the
 * peripherals' bus interfaces, a separate clk_peri for the UARTs and
 * SPIs (so that they keep their bit rates when clk_sys is changed, the
 * datasheet's own reason for it - 2.15.3.1), and dedicated generators
 * for USB, the ADC and the RTC. Each generator is an aux multiplexer
 * over the chip's sources - the crystal, the ring oscillator, the two
 * PLLs, two GPIO inputs - and a divider; clk_ref and clk_sys have the
 * glitchless mux in front, because they must never stop. What crosses
 * the contract is unchanged: `clock_hz(clock)` is clk_sys, and by
 * default this file pins clk_peri TO clk_sys undivided so that
 * `Clock::pclk_hz`, the rate the UART divides, IS hz and one number
 * serves every driver. `PeriSource::crystal` is the other choice, for
 * a program that moves clk_sys and wants its UARTs untouched: pclk_hz
 * is then the crystal's rate, and a UART on it keeps its divisor
 * through every switch (measured, test_rp2040_clock).
 *
 * THE BOOT STATE. The bootrom leaves the chip on the RING OSCILLATOR:
 * clk_ref and clk_sys both from ROSC at roughly 6.5 MHz, a rate that is
 * neither exact nor stable (2.17), the crystal off, both PLLs in reset,
 * clk_peri disabled. That is what main() starts from, and it is why
 * `ClockSource::internal` is declared and REFUSED: a rate that is not a
 * truth cannot be `hz`. The two sources built:
 *   crystal - XOSC straight into clk_sys: hz == crystal_hz (12 MHz on
 *             every board here);
 *   pll     - XOSC into the system PLL: hz is any rate an exact ratio
 *             reaches within 2.18.2's constraints (the reference after
 *             REFDIV at least 5 MHz, the VCO in 750..1600 MHz, FBDIV in
 *             16..320, each post divider in 1..7, the output at most
 *             133 MHz); searched at compile time, the highest VCO
 *             preferred (the datasheet's jitter advice, 2.18.2.1), the
 *             larger divider in POSTDIV1 (its power advice). 125 MHz
 *             from 12 MHz is FBDIV 125, VCO 1500, 6 x 2.
 *
 * THE SEQUENCE init() RUNS, from the datasheet's own rules (2.15.3.2,
 * 2.16.2, 2.18.3): clk_sys parked on clk_ref (the glitchless mux away
 * from aux, so the aux select - and the PLL behind it - can change
 * under it); the crystal on and STABLE, or KEPT if it already is (a
 * stable crystal is not restarted: the second init() of a program
 * that changes rate costs no millisecond and never lets clk_ref, which
 * may be on the crystal by then, drop); clk_ref onto the crystal; the
 * PLL out of reset, configured while powered down, powered up, LOCK
 * waited for, its post dividers powered; clk_sys's aux onto the PLL
 * and the glitchless mux back onto aux, SELECTED polled; clk_peri
 * stopped, its aux set, restarted. Every wait is bounded and every
 * failure answers false, with the tree left on clk_ref - the caller
 * then knows the rate is NOT the one `hz` claims. So init() is the
 * RATE SWITCH too: a program on `Clock<pll, 125 MHz>` that calls
 * `Clock<pll, 48 MHz>::init()` is at 48 MHz when it returns, with the
 * drivers on clk_sys (the SysTick ticker, a UART on PeriSource::sys)
 * owed a rebase to the new `hz` - measured on the bench in a few
 * hundred microseconds. The ring oscillator is left running: the resus
 * circuit (2.15.5) and the bootrom's next life are on it; `Rosc::stop()`
 * is the program's verb when it wants the microamps.
 *
 * A DYNAMIC CLOCK is not built: this chip has no voltage side to a rate
 * (one regulator setting serves the whole range) and clk_peri's
 * independence makes a rate change cheap on the drivers - the shape of
 * one is the STM32G0's Rates<> pack, the day a program wants it.
 *
 * ERRATUM RP2040-E7: the XOSC and ROSC COUNT registers are unreliable;
 * nothing here uses them - the frequency counter is the measure.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <optional>

#include "rp2040/device.hpp"

#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"
#include "util/clock.hpp"

namespace brio {

/// Where clk_sys comes from. `crystal` and `pll` are implemented; the
/// rest name the tree's other roots so the vocabulary does not change
/// under an application when they are built, and so asking for one
/// today is a compile error with an explanation instead of a wrong clock.
enum class ClockSource : uint8_t {
    crystal,    ///< XOSC straight into clk_sys
    pll,        ///< XOSC x the system PLL (the road to 125 MHz)
    internal,   ///< the ring oscillator: no exact rate, refused
    external,   ///< a clock driven into XIN with the oscillator off (up to 50 MHz)
    gpin,       ///< one of the two GPIO clock inputs
};

inline constexpr uint32_t xosc_min_hz = 1'000'000UL;
inline constexpr uint32_t xosc_max_hz = 15'000'000UL;
inline constexpr uint32_t clk_sys_max_hz = 133'000'000UL;
inline constexpr uint32_t pll_ref_min_hz = 5'000'000UL;
inline constexpr uint32_t pll_vco_min_hz = 750'000'000UL;
inline constexpr uint32_t pll_vco_max_hz = 1'600'000'000UL;
inline constexpr uint16_t pll_fbdiv_min = 16;
inline constexpr uint16_t pll_fbdiv_max = 320;

/// 2.16.3: STARTUP.DELAY counts crystal cycles in multiples of 256, and
/// the delay for a settling time is ceil(crystal_hz * t / 256) - 47 for
/// 12 MHz and the 1 ms the reference design needs.
constexpr uint32_t xosc_startup_delay(uint32_t crystal_hz, uint32_t settle_us = 1000) {
    const uint64_t cycles = static_cast<uint64_t>(crystal_hz) * settle_us / 1'000'000u;
    return static_cast<uint32_t>((cycles + 255u) / 256u);
}

/// A PLL setting: FOUTPOSTDIV = (FREF / refdiv) * fbdiv / (postdiv1 * postdiv2).
struct PllConfig {
    uint8_t refdiv;      ///< 1..63; 1 on every crystal this chip accepts
    uint16_t fbdiv;      ///< 16..320
    uint8_t postdiv1;    ///< 1..7
    uint8_t postdiv2;    ///< 1..7
    constexpr bool valid() const { return fbdiv != 0u; }
};

/// The exact ratio for `out_hz` from `ref_hz` under 2.18.2's constraints,
/// REFDIV 1; the highest VCO wins (less jitter), the larger post divider
/// goes first (less power). fbdiv 0 when none exists.
constexpr PllConfig pll_config_for(uint32_t ref_hz, uint32_t out_hz) {
    PllConfig best{.refdiv = 1, .fbdiv = 0, .postdiv1 = 0, .postdiv2 = 0};
    uint64_t best_vco = 0;
    if (out_hz == 0u || out_hz > clk_sys_max_hz || ref_hz < pll_ref_min_hz) {
        return best;
    }
    for (uint16_t fbdiv = pll_fbdiv_min; fbdiv <= pll_fbdiv_max; ++fbdiv) {
        const uint64_t vco = static_cast<uint64_t>(ref_hz) * fbdiv;
        if (vco < pll_vco_min_hz || vco > pll_vco_max_hz || vco <= best_vco) {
            continue;
        }
        // The first hit at this VCO has the largest POSTDIV1 the ratio
        // allows (the loops descend), which is the datasheet's tip.
        bool hit = false;
        for (uint8_t pd1 = 7; pd1 >= 1 && !hit; --pd1) {
            for (uint8_t pd2 = pd1; pd2 >= 1 && !hit; --pd2) {
                const uint32_t divisor = static_cast<uint32_t>(pd1) * pd2;
                if (vco % divisor == 0u && vco / divisor == out_hz) {
                    best = {.refdiv = 1, .fbdiv = fbdiv, .postdiv1 = pd1, .postdiv2 = pd2};
                    best_vco = vco;
                    hit = true;
                }
            }
        }
    }
    return best;
}

// ---- the crystal oscillator ---------------------------------------------------

struct Xosc {
    Xosc() = delete;

    /// Start the oscillator for a crystal in the 1..15 MHz range with
    /// STARTUP.DELAY = `startup_delay` (xosc_startup_delay()), and wait
    /// for STABLE. False when it did not rise within the bounded wait:
    /// the oscillator is then stopped again and the tree is untouched.
    ///
    /// A crystal already STABLE is kept, its delay register refreshed:
    /// restarting it would stop clk_ref when clk_ref is on it, and a
    /// stable crystal has nothing to prove.
    static bool init(uint32_t startup_delay) {
        if (stable()) {
            XOSC->STARTUP = startup_delay & XOSC_STARTUP_DELAY_BITS;
            return true;
        }
        XOSC->CTRL = XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ;
        XOSC->STARTUP = startup_delay & XOSC_STARTUP_DELAY_BITS;
        hw_set(XOSC->CTRL, XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB);
        // A few milliseconds at most for any crystal (2.16.2); the
        // budget is a multiple of that at the ring oscillator's rate.
        for (uint32_t spins = 2'000'000u; spins != 0u; --spins) {
            if (stable()) {
                return true;
            }
        }
        stop();
        return false;
    }

    static bool stable() { return (XOSC->STATUS & XOSC_STATUS_STABLE_BITS) != 0u; }
    static bool enabled() { return (XOSC->STATUS & XOSC_STATUS_ENABLED_BITS) != 0u; }
    /// STARTUP.DELAY as written.
    static uint32_t startup_delay() { return XOSC->STARTUP & XOSC_STARTUP_DELAY_BITS; }

    /// Stop the oscillator. Only with nothing clocked from it: clk_ref
    /// and clk_sys must have been moved away first.
    static void stop() {
        hw_write_masked(XOSC->CTRL, XOSC_CTRL_ENABLE_VALUE_DISABLE << XOSC_CTRL_ENABLE_LSB,
                        XOSC_CTRL_ENABLE_BITS);
    }
};

// ---- the two PLLs ---------------------------------------------------------------

/// One PLL block (2.18): the system PLL and the USB PLL are the same
/// register file at two addresses behind two reset bits.
template <uint32_t base, uint32_t reset_bit>
struct PllBlock {
    PllBlock() = delete;

    static PLL_SYS_Type& regs() { return *reinterpret_cast<PLL_SYS_Type*>(base); }

    /// Bring the PLL up on `cfg` (2.18.3's sequence): out of reset,
    /// dividers written while powered down, VCO powered, LOCK waited for,
    /// post dividers set and powered. False when LOCK did not rise; the
    /// PLL is then powered down again.
    static bool init(const PllConfig& cfg) {
        if (!Resets::cycle(reset_bit)) {
            return false;
        }
        regs().CS = cfg.refdiv & PLL_CS_REFDIV_BITS;
        regs().FBDIV_INT = cfg.fbdiv & PLL_FBDIV_INT_BITS;
        // VCO and the reference on, the post dividers still off.
        hw_clear(regs().PWR, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);
        bool locked_now = false;
        for (uint32_t spins = 1'000'000u; spins != 0u; --spins) {
            if (locked()) {
                locked_now = true;
                break;
            }
        }
        if (!locked_now) {
            stop();
            return false;
        }
        regs().PRIM = (static_cast<uint32_t>(cfg.postdiv1) << PLL_PRIM_POSTDIV1_LSB) |
                        (static_cast<uint32_t>(cfg.postdiv2) << PLL_PRIM_POSTDIV2_LSB);
        hw_clear(regs().PWR, PLL_PWR_POSTDIVPD_BITS);
        return true;
    }

    static bool locked() { return (regs().CS & PLL_CS_LOCK_BITS) != 0u; }

    /// The dividers as the registers hold them.
    static PllConfig config() {
        return PllConfig{
            .refdiv = static_cast<uint8_t>(regs().CS & PLL_CS_REFDIV_BITS),
            .fbdiv = static_cast<uint16_t>(regs().FBDIV_INT & PLL_FBDIV_INT_BITS),
            .postdiv1 = static_cast<uint8_t>((regs().PRIM & PLL_PRIM_POSTDIV1_BITS) >>
                                             PLL_PRIM_POSTDIV1_LSB),
            .postdiv2 = static_cast<uint8_t>((regs().PRIM & PLL_PRIM_POSTDIV2_BITS) >>
                                             PLL_PRIM_POSTDIV2_LSB)};
    }

    /// Power the PLL down. Only with nothing clocked from it.
    static void stop() {
        hw_set(regs().PWR, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);
    }
};

/// The system PLL: clk_sys's source under Clock<ClockSource::pll>.
using PllSys = PllBlock<PLL_SYS_BASE, ResetBlock::pll_sys>;
/// The USB PLL: 48 MHz for the USB controller and for clk_adc.
using PllUsb = PllBlock<PLL_USB_BASE, ResetBlock::pll_usb>;

constexpr bool operator==(const PllConfig& a, const PllConfig& b) {
    return a.refdiv == b.refdiv && a.fbdiv == b.fbdiv && a.postdiv1 == b.postdiv1 &&
           a.postdiv2 == b.postdiv2;
}

// ---- the ring oscillator -------------------------------------------------------

/**
 * The ring oscillator (2.17): the chip's boot clock, about 6.5 MHz and
 * exact to nothing - it drifts with voltage and temperature and varies
 * chip to chip - so it is never a `Clock`. What a program does with it:
 * leave it running (the default, and what the resus circuit and the
 * bootrom expect), or stop it once clk_ref and clk_sys are on the
 * crystal, for the microamps. The frequency range and the drive
 * strength stages stay at their reset values: tuning a clock that is
 * not a truth buys nothing brio can state.
 */
struct Rosc {
    Rosc() = delete;

    static bool running() { return (ROSC->STATUS & ROSC_STATUS_ENABLED_BITS) != 0u; }
    static bool stable() { return (ROSC->STATUS & ROSC_STATUS_STABLE_BITS) != 0u; }

    /// Start (or keep) the oscillator and wait, bounded, for STABLE.
    static bool start() {
        hw_write_masked(ROSC->CTRL, ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB,
                        ROSC_CTRL_ENABLE_BITS);
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (stable()) {
                return true;
            }
        }
        return false;
    }

    /// Stop the oscillator. Only with clk_ref and clk_sys elsewhere:
    /// stopping the clock a core runs on stops the core.
    static void stop() {
        hw_write_masked(ROSC->CTRL, ROSC_CTRL_ENABLE_VALUE_DISABLE << ROSC_CTRL_ENABLE_LSB,
                        ROSC_CTRL_ENABLE_BITS);
    }
};

// ---- the clock generators ---------------------------------------------------------

/// clk_ref's glitchless sources (CLK_REF_CTRL.SRC).
enum class RefSource : uint8_t {
    rosc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH,
    aux = CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX,
    xosc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
};

/// clk_adc's aux sources (CLK_ADC_CTRL.AUXSRC).
enum class AdcAux : uint8_t {
    pll_usb = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    pll_sys = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    rosc = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH,
    xosc = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

/// clk_rtc's aux sources (CLK_RTC_CTRL.AUXSRC).
enum class RtcAux : uint8_t {
    pll_usb = CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    pll_sys = CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    rosc = CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH,
    xosc = CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

/// clk_sys's aux sources (CLK_SYS_CTRL.AUXSRC).
enum class SysAux : uint8_t {
    pll_sys = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    pll_usb = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    rosc = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC,
    xosc = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

/// clk_peri's aux sources (CLK_PERI_CTRL.AUXSRC).
enum class PeriAux : uint8_t {
    clk_sys = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
    pll_sys = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    pll_usb = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    rosc = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH,
    xosc = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

struct Clocks {
    Clocks() = delete;

    /// clk_ref's glitchless mux onto `src`, SELECTED polled (2.15.3.2).
    static bool ref_select(RefSource src) {
        hw_write_masked(CLOCKS->CLK_REF_CTRL,
                        static_cast<uint32_t>(src) << CLOCKS_CLK_REF_CTRL_SRC_LSB,
                        CLOCKS_CLK_REF_CTRL_SRC_BITS);
        return wait_selected(CLOCKS->CLK_REF_SELECTED, static_cast<uint8_t>(src));
    }

    /// clk_sys's glitchless mux back onto clk_ref: the parking position
    /// from which the aux select may change.
    static bool sys_from_ref() {
        hw_clear(CLOCKS->CLK_SYS_CTRL, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
        return wait_selected(CLOCKS->CLK_SYS_SELECTED,
                             CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF);
    }

    /// clk_sys onto an aux source: the aux select written while the
    /// glitchless mux sits on clk_ref (sys_from_ref() first), then the
    /// glitchless mux onto aux, SELECTED polled.
    static bool sys_from_aux(SysAux aux) {
        if (sys_source() != CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF && !sys_from_ref()) {
            return false;
        }
        hw_write_masked(CLOCKS->CLK_SYS_CTRL,
                        static_cast<uint32_t>(aux) << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS);
        hw_set(CLOCKS->CLK_SYS_CTRL, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX);
        return wait_selected(CLOCKS->CLK_SYS_SELECTED,
                             CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX);
    }

    /// The glitchless mux's current position for clk_sys (0 = clk_ref,
    /// 1 = aux).
    static uint8_t sys_source() {
        return static_cast<uint8_t>(CLOCKS->CLK_SYS_CTRL & CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    }

    /// The integer dividers of the two always-on generators, 1 = undivided.
    static void ref_divider(uint8_t div) {
        CLOCKS->CLK_REF_DIV = static_cast<uint32_t>(div) << CLOCKS_CLK_REF_DIV_INT_LSB;
    }
    static void sys_divider(uint32_t div) {
        CLOCKS->CLK_SYS_DIV = div << CLOCKS_CLK_SYS_DIV_INT_LSB;
    }

    /// clk_peri onto `aux`, undivided (it has no divider): a generator
    /// without a glitchless mux is STOPPED while its aux select changes
    /// (2.15.3.2), then restarted. The two source cycles the stop needs
    /// are spent as a short spin.
    static void peri_select(PeriAux aux) {
        hw_clear(CLOCKS->CLK_PERI_CTRL, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
        for (uint32_t spins = 64u; spins != 0u; --spins) {
            __NOP();
        }
        hw_write_masked(CLOCKS->CLK_PERI_CTRL,
                        static_cast<uint32_t>(aux) << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS);
        hw_set(CLOCKS->CLK_PERI_CTRL, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
    }

    static bool peri_enabled() {
        return (CLOCKS->CLK_PERI_CTRL & CLOCKS_CLK_PERI_CTRL_ENABLE_BITS) != 0u;
    }

    /// clk_adc onto `aux` through its two-bit integer divider (1..3),
    /// the same stop-select-start as clk_peri: the generator has no
    /// glitchless mux. The converter wants 48 MHz (4.9.2): the USB PLL.
    static void adc_select(AdcAux aux, uint8_t div = 1) {
        hw_clear(CLOCKS->CLK_ADC_CTRL, CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);
        for (uint32_t spins = 64u; spins != 0u; --spins) {
            __NOP();
        }
        CLOCKS->CLK_ADC_DIV = (static_cast<uint32_t>(div) << CLOCKS_CLK_ADC_DIV_INT_LSB) & CLOCKS_CLK_ADC_DIV_INT_BITS;
        hw_write_masked(CLOCKS->CLK_ADC_CTRL,
                        static_cast<uint32_t>(aux) << CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS);
        hw_set(CLOCKS->CLK_ADC_CTRL, CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);
    }
    static void adc_stop() { hw_clear(CLOCKS->CLK_ADC_CTRL, CLOCKS_CLK_ADC_CTRL_ENABLE_BITS); }

    /// clk_rtc onto `aux` through its 24.8 divider (`div_int` 1..2^24 - 1
    /// and `div_frac` in 256ths; the crystal over 256 is 46875 Hz, the
    /// chapter's example), the same stop-select-start. The RTC wants 1
    /// to 65536 Hz of it (4.8.4).
    static void rtc_select(RtcAux aux, uint32_t div_int, uint8_t div_frac = 0) {
        hw_clear(CLOCKS->CLK_RTC_CTRL, CLOCKS_CLK_RTC_CTRL_ENABLE_BITS);
        for (uint32_t spins = 64u; spins != 0u; --spins) {
            __NOP();
        }
        CLOCKS->CLK_RTC_DIV = ((div_int << CLOCKS_CLK_RTC_DIV_INT_LSB) & CLOCKS_CLK_RTC_DIV_INT_BITS) | div_frac;
        hw_write_masked(CLOCKS->CLK_RTC_CTRL,
                        static_cast<uint32_t>(aux) << CLOCKS_CLK_RTC_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_RTC_CTRL_AUXSRC_BITS);
        hw_set(CLOCKS->CLK_RTC_CTRL, CLOCKS_CLK_RTC_CTRL_ENABLE_BITS);
    }
    static void rtc_stop() { hw_clear(CLOCKS->CLK_RTC_CTRL, CLOCKS_CLK_RTC_CTRL_ENABLE_BITS); }
    static bool rtc_enabled() {
        return (CLOCKS->CLK_RTC_CTRL & CLOCKS_CLK_RTC_CTRL_ENABLE_BITS) != 0u;
    }
    static RtcAux rtc_source() {
        return static_cast<RtcAux>((CLOCKS->CLK_RTC_CTRL & CLOCKS_CLK_RTC_CTRL_AUXSRC_BITS) >>
                                   CLOCKS_CLK_RTC_CTRL_AUXSRC_LSB);
    }
    /// The divider as it stands, in 256ths.
    static uint32_t rtc_divider256() { return CLOCKS->CLK_RTC_DIV; }
    static bool adc_enabled() {
        return (CLOCKS->CLK_ADC_CTRL & CLOCKS_CLK_ADC_CTRL_ENABLE_BITS) != 0u;
    }
    static AdcAux adc_source() {
        return static_cast<AdcAux>((CLOCKS->CLK_ADC_CTRL & CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS) >>
                                   CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB);
    }
    static PeriAux peri_source() {
        return static_cast<PeriAux>((CLOCKS->CLK_PERI_CTRL & CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS) >>
                                    CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB);
    }
    static uint32_t sys_divider() {
        return (CLOCKS->CLK_SYS_DIV & CLOCKS_CLK_SYS_DIV_INT_BITS) >> CLOCKS_CLK_SYS_DIV_INT_LSB;
    }

private:
    /// SELECTED is one-hot in the source index; a switch completes
    /// within a few cycles of each clock involved.
    static bool wait_selected(volatile uint32_t& selected, uint8_t src) {
        const uint32_t bit = 1u << src;
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if ((selected & bit) != 0u) {
                return true;
            }
        }
        return false;
    }
};

// ---- the frequency counter (2.15.4) ------------------------------------------------

/// What the counter can count: the roots and the generators of the tree
/// (FC0_SRC's values).
enum class CountSource : uint8_t {
    pll_sys = CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY,
    pll_usb = CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY,
    rosc = CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC,
    rosc_ph = CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC_PH,
    xosc = CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_FC0_SRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_FC0_SRC_VALUE_CLKSRC_GPIN1,
    clk_ref = CLOCKS_FC0_SRC_VALUE_CLK_REF,
    clk_sys = CLOCKS_FC0_SRC_VALUE_CLK_SYS,
    clk_peri = CLOCKS_FC0_SRC_VALUE_CLK_PERI,
    clk_usb = CLOCKS_FC0_SRC_VALUE_CLK_USB,
    clk_adc = CLOCKS_FC0_SRC_VALUE_CLK_ADC,
    clk_rtc = CLOCKS_FC0_SRC_VALUE_CLK_RTC,
};

/**
 * The frequency counter: a source counted against clk_ref over
 * 2^interval reference microseconds, the result in kHz with five
 * fraction bits. It is the chip's own ratio check - with clk_ref on the
 * crystal, every count is "how many of the crystal's periods" - and the
 * measure behind every rate the bench suite states. The MIN/MAX pass
 * thresholds are left wide open: brio judges the number, not a flag.
 */
struct FreqCounter {
    FreqCounter() = delete;

    /// `ref_hz` is clk_ref's rate (the crystal's, after Clock::init());
    /// `interval` 0..15 selects the counting window, 2^interval
    /// microseconds of it, the longest the finest (about 30 Hz at 15,
    /// 32 ms of counting). Nullopt when the source is dead (STATUS.DIED:
    /// a stopped oscillator, an unwired input) or the count never
    /// finished.
    static std::optional<uint32_t> count_hz(CountSource src, uint32_t ref_hz,
                                            uint8_t interval = 15) {
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if ((CLOCKS->FC0_STATUS & CLOCKS_FC0_STATUS_RUNNING_BITS) == 0u) {
                break;
            }
        }
        CLOCKS->FC0_REF_KHZ = (ref_hz / 1000u) & CLOCKS_FC0_REF_KHZ_BITS;
        CLOCKS->FC0_INTERVAL = interval & CLOCKS_FC0_INTERVAL_BITS;
        CLOCKS->FC0_MIN_KHZ = 0;
        CLOCKS->FC0_MAX_KHZ = CLOCKS_FC0_MAX_KHZ_BITS;
        CLOCKS->FC0_SRC = static_cast<uint32_t>(src);
        bool done = false;
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if ((CLOCKS->FC0_STATUS & CLOCKS_FC0_STATUS_DONE_BITS) != 0u) {
                done = true;
                break;
            }
        }
        if (!done || (CLOCKS->FC0_STATUS & CLOCKS_FC0_STATUS_DIED_BITS) != 0u) {
            return std::nullopt;
        }
        const uint32_t result = CLOCKS->FC0_RESULT;
        const uint32_t khz = (result & CLOCKS_FC0_RESULT_KHZ_BITS) >> CLOCKS_FC0_RESULT_KHZ_LSB;
        const uint32_t frac = result & CLOCKS_FC0_RESULT_FRAC_BITS;
        return khz * 1000u + (frac * 1000u) / 32u;
    }
};

// ---- the GPIO clock outputs and inputs (2.15.3.1, table 281) ----------------------

/// What a GPIO clock output can carry (CLK_GPOUTn_CTRL.AUXSRC).
enum class GpoutSource : uint8_t {
    pll_sys = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    gpin0 = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
    pll_usb = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    rosc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_ROSC_CLKSRC,
    xosc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    clk_sys = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS,
    clk_usb = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_USB,
    clk_adc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_ADC,
    clk_rtc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_RTC,
    clk_ref = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_REF,
};

/// The pin each output and input is bonded to (table 281): GPOUT0..3 on
/// GP21, GP23, GP24, GP25; GPIN0/1 on GP20, GP22.
constexpr uint8_t gpout_pin(uint8_t n) {
    return n == 0u ? 21u : n == 1u ? 23u : n == 2u ? 24u : 25u;
}
constexpr uint8_t gpin_pin(uint8_t n) { return n == 0u ? 20u : 22u; }

/// The four generators: CTRL, DIV and SELECTED, three words apiece.
static_assert(offsetof(CLOCKS_Type, CLK_GPOUT1_CTRL) ==
                  offsetof(CLOCKS_Type, CLK_GPOUT0_CTRL) + 12u,
              "the GPOUT generators are three words apart");
static_assert(offsetof(CLOCKS_Type, CLK_GPOUT0_DIV) == offsetof(CLOCKS_Type, CLK_GPOUT0_CTRL) + 4u);

/**
 * A GPIO clock output: `source` through an integer-plus-8-bit-fraction
 * divider onto its pin, the pin handed to the clock function. The
 * generator has no glitchless mux, so it is stopped while the source
 * changes (2.15.3.2), and a divider change on the fly is fine. The one
 * clock a counter on another board can measure - the crystal, or
 * clk_sys itself, on a wire.
 */
template <uint8_t n>
struct ClockOut {
    static_assert(n < 4u, "brio ClockOut: the RP2040 has four GPIO clock outputs, GPOUT0..3");
    ClockOut() = delete;

    static constexpr uint8_t pin = gpout_pin(n);

    /// The output on: `source` / (div_int + div_frac / 256). div_int in
    /// 1..2^24 - 1 (0 is the datasheet's 2^16, refused as a surprise).
    static bool init(GpoutSource source, uint32_t div_int, uint8_t div_frac = 0,
                     const PinConfig& cfg = {}) {
        if (div_int == 0u || div_int > (CLOCKS_CLK_GPOUT0_DIV_INT_BITS >> CLOCKS_CLK_GPOUT0_DIV_INT_LSB)) {
            return false;
        }
        hw_clear(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        for (uint32_t spins = 64u; spins != 0u; --spins) {
            __NOP();
        }
        hw_write_masked(ctrl(), static_cast<uint32_t>(source) << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_BITS);
        div() = (div_int << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) | div_frac;
        hw_set(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        Pin<pin>::function(PinFunction::clock, cfg);
        return true;
    }

    /// The output off and the pin released.
    static void stop() {
        hw_clear(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        Pin<pin>::release();
    }

    static bool enabled() { return (ctrl() & CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS) != 0u; }

private:
    static volatile uint32_t& ctrl() { return *(&CLOCKS->CLK_GPOUT0_CTRL + 3u * n); }
    static volatile uint32_t& div() { return *(&CLOCKS->CLK_GPOUT0_DIV + 3u * n); }
};

/**
 * A GPIO clock input: the pin handed to the clock function, after which
 * the generators (SysAux::gpin0, PeriAux::gpin0) and the counter
 * (CountSource::gpin0) can name it. No pull by default: a clock is
 * driven.
 */
template <uint8_t n>
struct ClockIn {
    static_assert(n < 2u, "brio ClockIn: the RP2040 has two GPIO clock inputs, GPIN0 and GPIN1");
    ClockIn() = delete;

    static constexpr uint8_t pin = gpin_pin(n);
    static constexpr CountSource count_source = n == 0u ? CountSource::gpin0 : CountSource::gpin1;

    static void init(const PinConfig& cfg = {}) { Pin<pin>::function(PinFunction::clock, cfg); }
    static void release() { Pin<pin>::release(); }
};

// ---- the task ------------------------------------------------------------------

/// Where clk_peri comes from under the task: clk_sys undivided (the
/// default: pclk_hz == hz) or the crystal (pclk_hz == crystal_hz, and
/// the UARTs keep their rate through every change of clk_sys).
enum class PeriSource : uint8_t { sys, crystal };

/**
 * The static main clock: `hz` is the ONE compile-time truth about clk_sys
 * that every driver of this target derives from.
 *
 *   using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
 *   constexpr SysClock clock;
 *   SysClock::init();                // first thing in main()
 *   Serial::init(clock, 115200);     // drivers ask the tag
 *
 * `hz` is clk_sys; `pclk_hz` is clk_peri, equal to `hz` because this
 * task feeds clk_peri from clk_sys undivided (the file header).
 * `crystal_hz` is the board's crystal, 12 MHz on the reference design
 * and every board brio has met.
 */
template <ClockSource src, uint32_t out_hz, uint32_t crystal_hz = 12'000'000UL,
          PeriSource peri = PeriSource::sys>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr PeriSource peri_source = peri;
    static constexpr uint32_t hz = out_hz;        ///< clk_sys
    /// clk_peri: clk_sys undivided, or the crystal (PeriSource::crystal).
    static constexpr uint32_t pclk_hz = peri == PeriSource::sys ? out_hz : crystal_hz;
    static constexpr uint32_t xtal_hz = crystal_hz;
    static constexpr bool is_static = true;

    static_assert(src == ClockSource::crystal || src == ClockSource::pll,
                  "brio Clock: only ClockSource::crystal (XOSC into clk_sys) and "
                  "ClockSource::pll (XOSC x the system PLL) are implemented on the "
                  "RP2040 - the ring oscillator has no exact rate to be a truth, and an "
                  "external clock or a GPIO input arrives with its first consumer");
    static_assert(crystal_hz >= xosc_min_hz && crystal_hz <= xosc_max_hz,
                  "brio Clock: the RP2040's crystal oscillator takes a 1..15 MHz crystal "
                  "(datasheet 2.16.1)");
    static_assert(src != ClockSource::crystal || out_hz == crystal_hz,
                  "brio Clock: on the crystal, clk_sys IS the crystal's rate");
    static_assert(src != ClockSource::pll || pll_config_for(crystal_hz, out_hz).valid(),
                  "brio Clock: no exact PLL ratio reaches this rate from the crystal - the "
                  "VCO must sit in 750..1600 MHz, FBDIV in 16..320, each post divider in "
                  "1..7, and clk_sys may not exceed 133 MHz (datasheet 2.18.2)");
    static_assert(out_hz <= clk_sys_max_hz, "clk_sys must not exceed 133 MHz");

    /// The PLL setting `hz` needs (meaningful for the pll source).
    static constexpr PllConfig pll = pll_config_for(crystal_hz, out_hz);
    /// The crystal's startup delay register value (1 ms of settling).
    static constexpr uint32_t startup_delay = xosc_startup_delay(crystal_hz);

    /// Bring clk_sys to `hz`. Returns false when the crystal did not
    /// start, the PLL did not lock or a mux switch did not take - the
    /// tree is then left on the ring oscillator and the caller knows the
    /// rate is NOT the one `hz` claims. Call first in main(), before any
    /// driver init.
    ///
    /// The whole sequence is RE-STATED rather than assumed: the bootrom
    /// leaves the tree on the ring oscillator, but a debugger's reset
    /// or a previous life may have left anything behind, and `hz` is a
    /// promise. Called on a running tree it is the RATE SWITCH (the file
    /// header): the crystal is kept, the PLL re-locked at the new ratio
    /// with clk_sys parked on clk_ref meanwhile.
    static bool init() {
        // Park clk_sys on clk_ref while clk_ref is still whatever it was:
        // both glitchless, nothing stops.
        if (!Clocks::sys_from_ref()) {
            return false;
        }
        Clocks::sys_divider(1);
        Clocks::ref_divider(1);
        if (!Xosc::init(startup_delay)) {
            return false;
        }
        if (!Clocks::ref_select(RefSource::xosc)) {
            return false;
        }
        if constexpr (src == ClockSource::pll) {
            if (!PllSys::init(pll)) {
                return false;
            }
            if (!Clocks::sys_from_aux(SysAux::pll_sys)) {
                return false;
            }
        }
        // clk_sys is now the crystal (through clk_ref) or the PLL.
        Clocks::peri_select(peri == PeriSource::sys ? PeriAux::clk_sys : PeriAux::xosc);
        return true;
    }

    /// A source counted against the crystal, which is clk_ref after
    /// init(): FreqCounter::count_hz with this clock's reference.
    static std::optional<uint32_t> count_hz(CountSource what, uint8_t interval = 15) {
        return FreqCounter::count_hz(what, crystal_hz, interval);
    }
};

} // namespace brio
