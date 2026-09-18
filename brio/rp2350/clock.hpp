/*
 * clock.hpp
 *
 * The RP2350 clock tree (datasheet chapter 8) in the two strata every
 * brio target uses (docs/design/clock.md):
 *
 *  RESOURCES - monostates, thin typed views with the discipline built in:
 *    Xosc     the crystal oscillator (8.2): the frequency range code, the
 *             startup delay in units of 256 crystal periods, the STABLE
 *             flag a bounded wait watches
 *    Rosc     the ring oscillator (8.3): running/stable, start and stop -
 *             never a rate, so never a Clock
 *    PllSys   the system PLL (8.6): REFDIV, FBDIV, the two post dividers,
 *             the power bits, the LOCK flag - and PllUsb, the same block
 *             at the USB PLL's address (48 MHz for the USB controller and
 *             for the converter's clk_adc)
 *    Clocks   the clock generators (8.1): clk_ref and clk_sys with their
 *             GLITCHLESS mux and their aux mux, clk_peri, clk_usb,
 *             clk_adc and clk_hstx with their aux mux alone, and the
 *             switching sequences of 8.1.4 which are the whole point of
 *             the block
 *    TickGenerator<which>  the tick generators of 8.5: clk_ref divided to
 *             a 1 us tick for each of the six consumers that count time
 *             rather than cycles - the two system timers, the watchdog,
 *             the two SysTicks and THE RISC-V PLATFORM TIMER, which is
 *             this target's timebase on its RISC-V half
 *    FreqCounter  the frequency counter (8.1.7): any root or generator
 *             counted against clk_ref - the chip's own ratio check, and
 *             what a bench suite measures every rate with
 *    ClockOut<n> / ClockIn<n>   the four GPIO clock outputs (GP21, GP23,
 *             GP24, GP25) and the two inputs (GP20, GP22)
 *
 *  TASK - what an application names:
 *    Clock<source, hz, crystal_hz, peri>   the static main clock: ONE
 *             constexpr truth `hz` every driver derives from (there is no
 *             F_CPU in this build); init() composes the resources and
 *             reports whether the tree reached the rate it claims.
 *
 * THE CLOCK MODEL is the RP2040's, and so is most of this file: no bus
 * prescaler and no enable bit per peripheral, one clk_sys for the cores,
 * the fabric, the memories and the peripherals' bus interfaces, a
 * separate clk_peri for the UARTs and SPIs so they keep their bit rates
 * when clk_sys changes, dedicated generators for USB, the ADC and (new
 * here) the HSTX. What is NOT the RP2040's: there is no clk_rtc, because
 * there is no RTC - the always-on timer in POWMAN took its place, with a
 * clock of its own; clk_ref gains the LOW-POWER OSCILLATOR as a fourth
 * source; the dividers are 16.16 where the RP2040's were 24.8, and
 * clk_peri has one at all; and the tick generators of 8.5 are a block of
 * their own, where the RP2040 derived its microsecond from the watchdog.
 * Every register constant below is the device header's, so none of this
 * is retyped from the other chip.
 *
 * THE BOOT STATE, AND WHY init() PARKS BEFORE IT TOUCHES ANYTHING. The
 * bootrom leaves the chip on the ring oscillator - and that is only the
 * COLD case. A debugger's `reset run` is a request to reset the
 * PROCESSORS, not the chip: the clock tree, the pads and the peripherals
 * stay exactly as the previous image left them, so an image may start on
 * a PLL it did not lock, with clk_sys running out of the very block
 * init() is about to hold in reset. So init() walks the tree back to a
 * source IT started before it touches a PLL: clk_sys onto clk_ref (both
 * glitchless, so nothing stops), clk_ref onto the ring oscillator once
 * that has been started and reported stable, and only then the crystal
 * and the PLL. It is also the rate switch, for the same reason it is on
 * the RP2040: calling init() on a running tree re-locks the PLL at the
 * new ratio with clk_sys parked meanwhile.
 *
 * `ClockSource::internal` is declared and REFUSED: the ring oscillator's
 * rate is neither exact nor stable (8.3), and a rate that is not a truth
 * cannot be `hz`. The two sources built are the crystal straight into
 * clk_sys and the crystal through the system PLL, whose exact ratio is
 * searched at compile time under 8.6.1's constraints - the reference
 * after REFDIV at least 5 MHz, the VCO in 750..1600 MHz, FBDIV in
 * 16..320, each post divider in 1..7, the output at most 150 MHz. From a
 * 12 MHz crystal, 150 MHz is FBDIV 125, VCO 1500 MHz, 5 x 2.
 *
 * WHAT THIS FILE DOES NOT COVER YET is the clock chapter's own document:
 * the low-power oscillator, the resus circuit, the PLL's lock-loss
 * interrupt, the top-level clock gates and the dynamic rate switch.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <optional>

#include "rp2350/device.hpp"

#include "rp2350/pin.hpp"
#include "rp2350/resets.hpp"
#include "util/clock.hpp"

namespace brio {

/// Where clk_sys comes from. `crystal` and `pll` are implemented; the
/// rest name the tree's other roots so the vocabulary does not change
/// under an application when they are built, and so asking for one today
/// is a compile error with an explanation instead of a wrong clock.
enum class ClockSource : uint8_t {
    crystal,    ///< XOSC straight into clk_sys
    pll,        ///< XOSC x the system PLL (the road to 150 MHz)
    internal,   ///< the ring oscillator: no exact rate, refused
    external,   ///< a clock driven into XIN with the oscillator off
    gpin,       ///< one of the two GPIO clock inputs
};

inline constexpr uint32_t xosc_min_hz = 1'000'000UL;
inline constexpr uint32_t xosc_max_hz = 50'000'000UL;
inline constexpr uint32_t clk_sys_max_hz = 150'000'000UL;
inline constexpr uint32_t pll_ref_min_hz = 5'000'000UL;
inline constexpr uint32_t pll_vco_min_hz = 750'000'000UL;
inline constexpr uint32_t pll_vco_max_hz = 1'600'000'000UL;
inline constexpr uint16_t pll_fbdiv_min = 16;
inline constexpr uint16_t pll_fbdiv_max = 320;

/// 8.2.3: STARTUP.DELAY counts crystal cycles in multiples of 256, and
/// the delay for a settling time is ceil(crystal_hz * t / 256) - 47 for
/// 12 MHz and the 1 ms the reference design needs.
constexpr uint32_t xosc_startup_delay(uint32_t crystal_hz, uint32_t settle_us = 1000) {
    const uint64_t cycles = static_cast<uint64_t>(crystal_hz) * settle_us / 1'000'000u;
    return static_cast<uint32_t>((cycles + 255u) / 256u);
}

/// The FREQ_RANGE code for a crystal (8.2.1): four ranges here, where
/// the RP2040 had one.
constexpr uint32_t xosc_range_code(uint32_t crystal_hz) {
    if (crystal_hz <= 15'000'000UL) { return XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ; }
    if (crystal_hz <= 30'000'000UL) { return XOSC_CTRL_FREQ_RANGE_VALUE_10_30MHZ; }
    if (crystal_hz <= 60'000'000UL) { return XOSC_CTRL_FREQ_RANGE_VALUE_25_60MHZ; }
    return XOSC_CTRL_FREQ_RANGE_VALUE_40_100MHZ;
}

/// A PLL setting: FOUTPOSTDIV = (FREF / refdiv) * fbdiv / (postdiv1 * postdiv2).
struct PllConfig {
    uint8_t refdiv;      ///< 1..63; 1 on every crystal this chip accepts
    uint16_t fbdiv;      ///< 16..320
    uint8_t postdiv1;    ///< 1..7
    uint8_t postdiv2;    ///< 1..7
    constexpr bool valid() const { return fbdiv != 0u; }
};

/// The exact ratio for `out_hz` from `ref_hz` under 8.6.1's constraints,
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

    /// Start the oscillator for a crystal of `crystal_hz` with
    /// STARTUP.DELAY = `startup_delay` (xosc_startup_delay()), and wait
    /// for STABLE. False when it did not rise within the bounded wait:
    /// the oscillator is then stopped again and the tree is untouched.
    ///
    /// A crystal already STABLE is kept, its delay register refreshed:
    /// restarting it would stop clk_ref when clk_ref is on it, and a
    /// stable crystal has nothing to prove.
    static bool init(uint32_t startup_delay, uint32_t crystal_hz) {
        if (stable()) {
            XOSC->STARTUP = startup_delay & XOSC_STARTUP_DELAY_BITS;
            return true;
        }
        XOSC->CTRL = xosc_range_code(crystal_hz);
        XOSC->STARTUP = startup_delay & XOSC_STARTUP_DELAY_BITS;
        hw_set(XOSC->CTRL, XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB);
        // A few milliseconds at most for any crystal (8.2.2); the budget
        // is a multiple of that at the ring oscillator's rate.
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

// ---- the ring oscillator -------------------------------------------------------

/**
 * The ring oscillator (8.3): the chip's boot clock, exact to nothing -
 * it drifts with voltage and temperature and varies chip to chip - so it
 * is never a `Clock`. What it is here is THE SAFE PLACE TO PARK: a
 * source this file can start itself, so that clk_ref (and clk_sys behind
 * it) has somewhere to stand while the crystal and the PLL are taken
 * apart. The frequency range and the drive strength stages stay at their
 * reset values: tuning a clock that is not a truth buys nothing brio can
 * state.
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

// ---- the two PLLs ---------------------------------------------------------------

/// One PLL block (8.6): the system PLL and the USB PLL are the same
/// register file at two addresses behind two reset bits.
template <uint32_t base, uint32_t reset_bit>
struct PllBlock {
    PllBlock() = delete;

    static PLL_SYS_Type& regs() { return *reinterpret_cast<PLL_SYS_Type*>(base); }

    /// Bring the PLL up on `cfg` (8.6.2's sequence): out of reset,
    /// dividers written while powered down, VCO powered, LOCK waited for,
    /// post dividers set and powered. False when LOCK did not rise; the
    /// PLL is then powered down again. The reset CYCLE matters here and
    /// not only at boot: a debugger's reset leaves a locked PLL locked,
    /// and a ratio is written into a stopped block.
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

// ---- the clock generators ---------------------------------------------------------

/// clk_ref's glitchless sources (CLK_REF_CTRL.SRC). The low-power
/// oscillator is this chip's addition.
enum class RefSource : uint8_t {
    rosc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH,
    aux = CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX,
    xosc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
    lposc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_LPOSC_CLKSRC,
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

    /// clk_ref's glitchless mux onto `src`, SELECTED polled (8.1.4).
    static bool ref_select(RefSource src) {
        hw_write_masked(CLOCKS->CLK_REF_CTRL,
                        static_cast<uint32_t>(src) << CLOCKS_CLK_REF_CTRL_SRC_LSB,
                        CLOCKS_CLK_REF_CTRL_SRC_BITS);
        return wait_selected(CLOCKS->CLK_REF_SELECTED, static_cast<uint8_t>(src));
    }

    /// The glitchless mux's current position for clk_ref.
    static uint8_t ref_source() {
        return static_cast<uint8_t>(CLOCKS->CLK_REF_CTRL & CLOCKS_CLK_REF_CTRL_SRC_BITS);
    }

    /// clk_sys's glitchless mux back onto clk_ref: the parking position
    /// from which the aux select - and the PLL behind it - may change.
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

    /// The integer dividers of the two always-on generators, 1 =
    /// undivided. The field is the INTEGER half of a 16.16 divider here,
    /// where the RP2040's was 24.8.
    static void ref_divider(uint8_t div) {
        CLOCKS->CLK_REF_DIV = (static_cast<uint32_t>(div) << CLOCKS_CLK_REF_DIV_INT_LSB) &
                              CLOCKS_CLK_REF_DIV_INT_BITS;
    }
    static void sys_divider(uint32_t div) {
        CLOCKS->CLK_SYS_DIV = (div << CLOCKS_CLK_SYS_DIV_INT_LSB) & CLOCKS_CLK_SYS_DIV_INT_BITS;
    }
    static uint32_t sys_divider() {
        return (CLOCKS->CLK_SYS_DIV & CLOCKS_CLK_SYS_DIV_INT_BITS) >> CLOCKS_CLK_SYS_DIV_INT_LSB;
    }

    /// clk_peri onto `aux` through its two-bit integer divider (1..3): a
    /// generator without a glitchless mux is STOPPED while its aux select
    /// changes (8.1.4), then restarted. The two source cycles the stop
    /// needs are spent as a short spin.
    static void peri_select(PeriAux aux, uint8_t div = 1) {
        hw_clear(CLOCKS->CLK_PERI_CTRL, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
        for (uint32_t spins = 64u; spins != 0u; --spins) {
            __asm__ volatile("");
        }
        CLOCKS->CLK_PERI_DIV = (static_cast<uint32_t>(div) << CLOCKS_CLK_PERI_DIV_INT_LSB) &
                               CLOCKS_CLK_PERI_DIV_INT_BITS;
        hw_write_masked(CLOCKS->CLK_PERI_CTRL,
                        static_cast<uint32_t>(aux) << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS);
        hw_set(CLOCKS->CLK_PERI_CTRL, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
    }

    static bool peri_enabled() {
        return (CLOCKS->CLK_PERI_CTRL & CLOCKS_CLK_PERI_CTRL_ENABLE_BITS) != 0u;
    }
    static PeriAux peri_source() {
        return static_cast<PeriAux>((CLOCKS->CLK_PERI_CTRL & CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS) >>
                                    CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB);
    }

private:
    /// SELECTED is one-hot in the source index; a switch completes within
    /// a few cycles of each clock involved.
    static bool wait_selected(volatile uint32_t& selected, uint8_t src) {
        const uint32_t bit = 1UL << src;
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if ((selected & bit) != 0u) {
                return true;
            }
        }
        return false;
    }
};

// ---- the tick generators (8.5) ------------------------------------------------

/// The six consumers of a tick, in the order their register triples sit
/// in the TICKS block.
enum class TickConsumer : uint8_t {
    proc0 = 0,      ///< core 0's SysTick, in its reference-clock mode
    proc1 = 1,      ///< core 1's
    timer0 = 2,     ///< the first system timer
    timer1 = 3,     ///< the second
    watchdog = 4,   ///< the watchdog's countdown
    riscv = 5,      ///< THE RISC-V PLATFORM TIMER (the SIO's MTIME)
};

/**
 * One tick generator: clk_ref divided by CYCLES, delivered to one
 * consumer. A tick is not a clock - it drives no register's clock input
 * - it is a periodic pulse that lets a counter measure TIME while
 * clk_sys moves under it, which is why the system timers and the RISC-V
 * platform timer keep their microsecond through every rate change.
 *
 * The generator must be STOPPED before CYCLES is written (8.5.1), which
 * start() does; the RUNNING flag is the readback.
 */
template <TickConsumer which>
struct TickGenerator {
    TickGenerator() = delete;

    /// Start the generator with `cycles` of clk_ref per tick - 12 for a
    /// microsecond off a 12 MHz crystal. False when the count does not
    /// fit the nine-bit field or the generator did not start.
    static bool start(uint32_t cycles) {
        if (cycles == 0u || cycles > TICKS_PROC0_CYCLES_BITS) {
            return false;
        }
        stop();
        cycles_reg() = cycles;
        ctrl() = TICKS_PROC0_CTRL_ENABLE_BITS;
        for (uint32_t spins = 10'000u; spins != 0u; --spins) {
            if (running()) {
                return true;
            }
        }
        return false;
    }

    static void stop() {
        ctrl() = 0;
        while ((ctrl() & TICKS_PROC0_CTRL_RUNNING_BITS) != 0u) {
        }
    }

    static bool running() { return (ctrl() & TICKS_PROC0_CTRL_RUNNING_BITS) != 0u; }
    /// The cycle count as written.
    static uint32_t cycles() { return cycles_reg() & TICKS_PROC0_CYCLES_BITS; }
    /// The generator's own countdown, for the record.
    static uint32_t count() { return count_reg(); }

private:
    static constexpr uint32_t stride = TICKS_PROC1_CTRL_OFFSET - TICKS_PROC0_CTRL_OFFSET;
    static constexpr uint32_t base = TICKS_PROC0_CTRL_OFFSET +
                                     stride * static_cast<uint32_t>(which);
    static volatile uint32_t& ctrl() { return reg_at(TICKS_BASE, base); }
    static volatile uint32_t& cycles_reg() { return reg_at(TICKS_BASE, base + 4u); }
    static volatile uint32_t& count_reg() { return reg_at(TICKS_BASE, base + 8u); }
};

// ---- the frequency counter (8.1.7) ------------------------------------------------

/// What the counter can count: the roots and the generators of the tree
/// (FC0_SRC's values - NOT the RP2040's numbering).
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
    clk_hstx = CLOCKS_FC0_SRC_VALUE_CLK_HSTX,
    lposc = CLOCKS_FC0_SRC_VALUE_LPOSC_CLKSRC,
};

/**
 * The frequency counter: a source counted against clk_ref over
 * 2^interval reference microseconds, the result in kHz with five
 * fraction bits. It is the chip's own ratio check - with clk_ref on the
 * crystal, every count is "how many of the crystal's periods" - and the
 * measure behind every rate this stratum states. The MIN/MAX pass
 * thresholds are left wide open: brio judges the number, not a flag.
 */
struct FreqCounter {
    FreqCounter() = delete;

    /// `ref_hz` is clk_ref's rate (the crystal's, after Clock::init());
    /// `interval` 0..15 selects the counting window, 2^interval
    /// microseconds of it, the longest the finest. Nullopt when the
    /// source is dead (STATUS.DIED: a stopped oscillator, an unwired
    /// input) or the count never finished.
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

// ---- the GPIO clock outputs and inputs ------------------------------------------

/// What a GPIO clock output can carry (CLK_GPOUTn_CTRL.AUXSRC).
enum class GpoutSource : uint8_t {
    pll_sys = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    gpin0 = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
    pll_usb = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    rosc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_ROSC_CLKSRC,
    xosc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    lposc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_LPOSC_CLKSRC,
    clk_sys = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS,
    clk_usb = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_USB,
    clk_adc = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_ADC,
    clk_ref = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_REF,
    clk_peri = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_PERI,
    clk_hstx = CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_HSTX,
};

/// The pin each output and input is bonded to: GPOUT0..3 on GP21, GP23,
/// GP24, GP25; GPIN0/1 on GP20, GP22 - the RP2040's own pins, and the
/// function number is 9 here where it was 8 there.
constexpr uint8_t gpout_pin(uint8_t n) {
    return n == 0u ? 21u : n == 1u ? 23u : n == 2u ? 24u : 25u;
}
constexpr uint8_t gpin_pin(uint8_t n) { return n == 0u ? 20u : 22u; }

/// The four generators: CTRL, DIV and SELECTED, three words apiece.
static_assert(CLOCKS_CLK_GPOUT1_CTRL_OFFSET == CLOCKS_CLK_GPOUT0_CTRL_OFFSET + 12u,
              "the GPOUT generators are three words apart");

/**
 * A GPIO clock output: `source` through a 16.16 divider onto its pin,
 * the pin handed to the clock function. The generator has no glitchless
 * mux, so it is stopped while the source changes (8.1.4). The one clock
 * a counter on another board can measure - the crystal, or clk_sys
 * itself, on a wire.
 */
template <uint8_t n>
struct ClockOut {
    static_assert(n < 4u, "brio ClockOut: the RP2350 has four GPIO clock outputs, GPOUT0..3");
    ClockOut() = delete;

    static constexpr uint8_t pin = gpout_pin(n);

    /// The output on: `source` / (div_int + div_frac / 65536). div_int in
    /// 1..65535 (0 would be the divider's own special case, refused as a
    /// surprise).
    static bool init(GpoutSource source, uint32_t div_int, uint16_t div_frac = 0,
                     const PinConfig& cfg = {}) {
        if (div_int == 0u ||
            div_int > (CLOCKS_CLK_GPOUT0_DIV_INT_BITS >> CLOCKS_CLK_GPOUT0_DIV_INT_LSB)) {
            return false;
        }
        hw_clear(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        for (uint32_t spins = 64u; spins != 0u; --spins) {
            __asm__ volatile("");
        }
        hw_write_masked(ctrl(),
                        static_cast<uint32_t>(source) << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_BITS);
        div() = (div_int << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) | div_frac;
        hw_set(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        return Pin<pin>::function(PinFunction::gpck, cfg);
    }

    /// The output off and the pin released.
    static void stop() {
        hw_clear(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        (void)Pin<pin>::release();
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
    static_assert(n < 2u, "brio ClockIn: the RP2350 has two GPIO clock inputs, GPIN0 and GPIN1");
    ClockIn() = delete;

    static constexpr uint8_t pin = gpin_pin(n);
    static constexpr CountSource count_source = n == 0u ? CountSource::gpin0 : CountSource::gpin1;

    static bool init(const PinConfig& cfg = {}) { return Pin<pin>::function(PinFunction::gpck, cfg); }
    static bool release() { return Pin<pin>::release(); }
};

// ---- the task ------------------------------------------------------------------

/// Where clk_peri comes from under the task: clk_sys undivided (the
/// default: pclk_hz == hz) or the crystal (pclk_hz == crystal_hz, and
/// the serial ports keep their rate through every change of clk_sys).
enum class PeriSource : uint8_t { sys, crystal };

/**
 * The static main clock: `hz` is the ONE compile-time truth about
 * clk_sys that every driver of this target derives from.
 *
 *   using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000>;
 *   constexpr SysClock clock;
 *   SysClock::init();                // first thing in main()
 *
 * `hz` is clk_sys; `pclk_hz` is clk_peri; `ref_hz` is clk_ref, which
 * after init() is the crystal and which is what the tick generators of
 * 8.5 divide. `crystal_hz` is the board's crystal, 12 MHz on the
 * reference design and on every board brio has met.
 */
template <ClockSource src, uint32_t out_hz, uint32_t crystal_hz = 12'000'000UL,
          PeriSource peri = PeriSource::sys>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr PeriSource peri_source = peri;
    static constexpr uint32_t hz = out_hz;        ///< clk_sys
    /// clk_peri: clk_sys undivided, or the crystal (PeriSource::crystal).
    static constexpr uint32_t pclk_hz = peri == PeriSource::sys ? out_hz : crystal_hz;
    /// clk_ref after init(): the crystal, which is also what every tick
    /// generator of 8.5 divides.
    static constexpr uint32_t ref_hz = crystal_hz;
    static constexpr uint32_t xtal_hz = crystal_hz;
    static constexpr bool is_static = true;

    static_assert(src == ClockSource::crystal || src == ClockSource::pll,
                  "brio Clock: only ClockSource::crystal (XOSC into clk_sys) and "
                  "ClockSource::pll (XOSC x the system PLL) are implemented on the "
                  "RP2350 - the ring oscillator has no exact rate to be a truth, and an "
                  "external clock or a GPIO input arrives with its first consumer");
    static_assert(crystal_hz >= xosc_min_hz && crystal_hz <= xosc_max_hz,
                  "brio Clock: the RP2350's crystal oscillator takes a 1..50 MHz crystal "
                  "(datasheet 8.2)");
    static_assert(src != ClockSource::crystal || out_hz == crystal_hz,
                  "brio Clock: on the crystal, clk_sys IS the crystal's rate");
    static_assert(src != ClockSource::pll || pll_config_for(crystal_hz, out_hz).valid(),
                  "brio Clock: no exact PLL ratio reaches this rate from the crystal - the "
                  "VCO must sit in 750..1600 MHz, FBDIV in 16..320, each post divider in "
                  "1..7, and clk_sys may not exceed 150 MHz (datasheet 8.6.1)");
    static_assert(out_hz <= clk_sys_max_hz, "clk_sys must not exceed 150 MHz");

    /// The PLL setting `hz` needs (meaningful for the pll source).
    static constexpr PllConfig pll = pll_config_for(crystal_hz, out_hz);
    /// The crystal's startup delay register value (1 ms of settling).
    static constexpr uint32_t startup_delay = xosc_startup_delay(crystal_hz);

    /// Bring clk_sys to `hz`. Returns false when the crystal did not
    /// start, the PLL did not lock or a mux switch did not take - the
    /// tree is then left wherever the parking put it, and the caller
    /// knows the rate is NOT the one `hz` claims. Call first in main(),
    /// before any driver init.
    ///
    /// THE WHOLE SEQUENCE IS RE-STATED rather than assumed, and the
    /// parking is the first act (the file header says why): a debugger's
    /// reset resets the cores and nothing else, so this may be running
    /// out of the very PLL it is about to hold in reset.
    static bool init() {
        // 1. clk_sys onto clk_ref: both glitchless, nothing stops.
        if (!Clocks::sys_from_ref()) {
            return false;
        }
        Clocks::sys_divider(1);
        // 2. clk_ref onto a source THIS function started. If the ring
        //    oscillator will not start, clk_ref is left where it is: it
        //    is then either already the crystal (which the next step
        //    keeps running) or something the caller chose, and moving it
        //    onto a dead oscillator would stop the core outright.
        if (Rosc::start()) {
            (void)Clocks::ref_select(RefSource::rosc);
        }
        Clocks::ref_divider(1);
        // 3. the crystal, kept if it is already stable.
        if (!Xosc::init(startup_delay, crystal_hz)) {
            return false;
        }
        if (!Clocks::ref_select(RefSource::xosc)) {
            return false;
        }
        // 4. the PLL, configured in a block held in reset first.
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
