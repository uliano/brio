/*
 * clock.hpp
 *
 * The RP2350 clock tree (datasheet chapter 8 whole) in the two strata
 * every brio target uses (docs/design/clock.md):
 *
 *  RESOURCES - monostates, thin typed views with the discipline built in:
 *    Xosc     the crystal oscillator (8.2): the four frequency ranges,
 *             the startup delay in units of 256 crystal periods, the
 *             STABLE flag a bounded wait watches, the countdown COUNT
 *             register that times a short wait in crystal periods, and
 *             the BADWRITE flag that catches a password the silicon
 *             refused
 *    Rosc     the ring oscillator (8.3): the boot clock and THE SAFE
 *             PLACE TO PARK - running/stable, start and stop, the four
 *             stage ranges, the eight drive strengths behind their
 *             password, THE FREQUENCY RANDOMISER this chip adds, the
 *             output divider, the one-bit random source and the same
 *             countdown COUNT. Never a rate, so never a Clock
 *    Lposc    the low-power oscillator (8.4): a nominal 32.768 kHz RC
 *             oscillator in the ALWAYS-ON domain that needs no starting
 *             and cannot be stopped, trimmable to about 1.5 per cent -
 *             a fourth root of clk_ref, and the only clock that outlives
 *             the switched core
 *    PllSys   the system PLL (8.6): REFDIV, FBDIV, the two post
 *             dividers, the power bits, LOCK and the STICKY LOCK-LOSS
 *             this chip adds with its own interrupt - and PllUsb, the
 *             same block at the USB PLL's address (48 MHz for the USB
 *             controller and for the converter's clk_adc)
 *    Clocks   the clock generators (8.1): clk_ref and clk_sys with their
 *             GLITCHLESS mux and their aux mux, clk_peri, clk_usb,
 *             clk_adc and clk_hstx with their aux mux alone, the
 *             switching sequences of 8.1.3.2 which are the whole point
 *             of the block, and the top-level gates of 8.1.3.5
 *    Resus    the resuscitation circuit (8.1.5): the watchdog of clk_sys
 *             itself, which puts the glitchless mux back on clk_ref when
 *             clk_sys stops - a debugging aid, and the one way a chip
 *             whose clock a program killed still answers
 *    TickGenerator<which>  the tick generators of 8.5: clk_ref divided to
 *             a 1 us tick for each of the six consumers that count time
 *             rather than cycles - the two system timers, the watchdog,
 *             the two SysTicks and THE RISC-V PLATFORM TIMER, which is
 *             this target's timebase on its RISC-V half and its ruler on
 *             both
 *    FreqCounter  the frequency counter (8.1.4): any root or generator
 *             counted against clk_ref - the chip's own ratio check, and
 *             what a bench suite measures every rate with; measure()
 *             adds the hardware's own verdict between two bounds
 *    ClockOut<n> / ClockIn<n>   the four GPIO clock outputs (GP21, GP23,
 *             GP24, GP25) and the two inputs (GP20, GP22)
 *
 *  TASK - what an application names:
 *    Clock<source, hz, crystal_hz, peri>   the static main clock: ONE
 *             constexpr truth `hz` every driver derives from (there is no
 *             F_CPU in this build); init() composes the resources and
 *             reports whether the tree reached the rate it claims.
 *
 * THE CLOCK MODEL is the RP2040's, and so is much of this file: no bus
 * prescaler and no enable bit per peripheral, one clk_sys for the cores,
 * the fabric, the memories and the peripherals' bus interfaces, a
 * separate clk_peri for the UARTs and SPIs so they keep their bit rates
 * when clk_sys changes, dedicated generators for USB, the ADC and (new
 * here) the HSTX. What is NOT the RP2040's: there is no clk_rtc, because
 * there is no RTC - the always-on timer in POWMAN took its place, with a
 * clock of its own; clk_ref gains the LOW-POWER OSCILLATOR as a fourth
 * source and is capped at 25 MHz; the dividers are 16.16 where the
 * RP2040's were 24.8, and clk_peri has one at all; the tick generators of
 * 8.5 are a block of their own, where the RP2040 derived its microsecond
 * from the watchdog; the ring oscillator can randomise its own frequency;
 * the PLLs report a LOST lock and not only a present one; and the
 * frequency counter's source numbering is another one. Every register
 * constant below is the device header's, so none of this is retyped from
 * the other chip.
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
/// 8.2.2's note: whatever the crystal, clk_ref may not exceed this - a
/// faster crystal is divided down by the clk_ref divider.
inline constexpr uint32_t clk_ref_max_hz = 25'000'000UL;
inline constexpr uint32_t clk_sys_max_hz = 150'000'000UL;
inline constexpr uint32_t pll_ref_min_hz = 5'000'000UL;
inline constexpr uint32_t pll_vco_min_hz = 750'000'000UL;
inline constexpr uint32_t pll_vco_max_hz = 1'600'000'000UL;
inline constexpr uint16_t pll_fbdiv_min = 16;
inline constexpr uint16_t pll_fbdiv_max = 320;
/// The low-power oscillator's nominal rate (8.4): plus or minus 20 per
/// cent untrimmed, about 1.5 trimmed, and drifting with temperature and
/// supply either way - a number to divide by, never a truth to state.
inline constexpr uint32_t lposc_nominal_hz = 32'768UL;
/// What the USB controller and the converter ask of their generators.
inline constexpr uint32_t clk_usb_hz = 48'000'000UL;
inline constexpr uint32_t clk_adc_hz = 48'000'000UL;

/// 8.2.4: STARTUP.DELAY counts crystal cycles in multiples of 256, and
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

// ---- the crystal oscillator (8.2) ----------------------------------------------

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
        XOSC->STARTUP = startup_delay & XOSC_STARTUP_DELAY_BITS;
        // ONE WHOLE-WORD WRITE, with both passwords spelled. The header
        // says why: a masked write leaves zeros in the field it does not
        // mean to touch, and a zero is not one of this field's codes.
        XOSC->CTRL = (XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB) |
                     xosc_range_code(crystal_hz);
        // A few milliseconds at most for any crystal (8.2.3); the budget
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
    /// Whether the startup timer counts in units of 1024 rather than 256
    /// crystal periods (STARTUP.X4), for a crystal that wants four times
    /// the longest plain delay.
    static bool startup_x4() { return (XOSC->STARTUP & XOSC_STARTUP_X4_BITS) != 0u; }
    static void startup_x4(bool on) {
        hw_write_masked(XOSC->STARTUP, on ? XOSC_STARTUP_X4_BITS : 0u, XOSC_STARTUP_X4_BITS);
    }
    /// The range the block reports it is in (STATUS.FREQ_RANGE, two bits
    /// where CTRL's field is twelve).
    static uint8_t range() {
        return static_cast<uint8_t>(XOSC->STATUS & XOSC_STATUS_FREQ_RANGE_BITS);
    }

    /// A short wait in CRYSTAL periods and not in instructions (8.2.5):
    /// the count is written, the hardware counts it down at the
    /// crystal's rate, and the caller polls it to zero. Independent of
    /// clk_sys, the compiler and the code's placement - which is what
    /// makes it better than a loop of NOPs. Returns false at once when
    /// the crystal is not running, there being nothing to count with.
    static bool wait_periods(uint16_t periods) {
        if (!stable()) {
            return false;
        }
        XOSC->COUNT = periods;
        while ((XOSC->COUNT & XOSC_COUNT_BITS) != 0u) {
        }
        return true;
    }
    static uint16_t count() { return static_cast<uint16_t>(XOSC->COUNT & XOSC_COUNT_BITS); }

    /// A write the block refused because its password was wrong (the
    /// twelve-bit ENABLE and FREQ_RANGE fields are passwords, not flags).
    /// Write-to-clear.
    static bool badwrite() { return (XOSC->STATUS & XOSC_STATUS_BADWRITE_BITS) != 0u; }
    static void clear_badwrite() { XOSC->STATUS = XOSC_STATUS_BADWRITE_BITS; }

    /// Stop the oscillator. Only with nothing clocked from it: clk_ref
    /// and clk_sys must have been moved away first. The range the block
    /// is in is carried over, both passwords in one word.
    static void stop() {
        XOSC->CTRL = (XOSC_CTRL_ENABLE_VALUE_DISABLE << XOSC_CTRL_ENABLE_LSB) | range_code();
    }

    /// The range field as CTRL holds it (twelve bits, one of the four
    /// codes) - as opposed to range(), which is STATUS's two-bit report.
    static uint32_t range_code() { return XOSC->CTRL & XOSC_CTRL_FREQ_RANGE_BITS; }

    /// DORMANT (8.2.6, 6.5.3): the keyword that stops this oscillator
    /// and with it every clock derived from it - the core's included
    /// when clk_sys runs on it, so this call returns only once a
    /// configured wake (a GPIO event the IO bank's dormant-wake logic
    /// detects, or the always-on timer's alarm) has restarted it and it
    /// is STABLE again. WITH NO WAKE CONFIGURED IT NEVER RETURNS;
    /// rp2350/sleep.hpp's site refuses to arm that. The PLLs are not
    /// stopped by the silicon: stop them first.
    static void dormant() {
        XOSC->DORMANT = XOSC_DORMANT_VALUE_DORMANT;
        while (!stable()) {
        }
    }
};

// ---- the ring oscillator (8.3) -------------------------------------------------

/// The four stage ranges (8.3.5): how many of the eight inverter stages
/// are in the loop. MEDIUM is about 1.33 times LOW, HIGH about 2, and
/// TOOHIGH about 4 - which the datasheet names as it does because the
/// block's own logic does not run there.
enum class RoscRange : uint16_t {
    low = ROSC_CTRL_FREQ_RANGE_VALUE_LOW,
    medium = ROSC_CTRL_FREQ_RANGE_VALUE_MEDIUM,
    high = ROSC_CTRL_FREQ_RANGE_VALUE_HIGH,
    toohigh = ROSC_CTRL_FREQ_RANGE_VALUE_TOOHIGH,
};

/**
 * The ring oscillator (8.3): the chip's boot clock, exact to nothing -
 * it drifts with voltage and temperature and varies chip to chip, and
 * the datasheet guarantees only 4.6 to 19.6 MHz around a nominal 11 - so
 * it is never a `Clock`. What it is here is THE SAFE PLACE TO PARK: a
 * source this file can start itself, so that clk_ref (and clk_sys behind
 * it) has somewhere to stand while the crystal and the PLL are taken
 * apart.
 *
 * ITS CONFIGURATION IS OFFERED AND NOT USED BY THE TASK. The frequency
 * range, the eight stages' drive strengths and the output divider are
 * what a program that chooses to run on this oscillator plays with; the
 * two disciplines the chapter states are enforced here - a range is
 * stepped one at a time (increasing does not glitch, decreasing does),
 * and every write carries the password its register wants, with BADWRITE
 * as the readback when it did not.
 *
 * THE RANDOMISER IS THIS CHIP'S ADDITION (8.3.2, 8.3.6): the first two
 * stages' drive strengths can be driven by an LFSR, which spreads the
 * oscillator's spectrum instead of leaving it a tone - and raises the
 * frequency by up to 22 per cent of the default in the LOW range. The
 * seed is writable at any time and restarts the sequence.
 *
 * AND ITS CTRL IS WRITTEN AS A WHOLE WORD, ALWAYS - WITH TWO TRAPS IN
 * IT. ENABLE and FREQ_RANGE are twelve-bit PASSWORDS and not fields: the
 * only values they take are 0xfab / 0xd1e and the four range codes, and
 * anything else sets STATUS.BADWRITE. The first trap is that a
 * read-modify-write through the atomic SET / CLEAR / XOR aliases writes
 * zeros into the field it does not mean to touch, which is exactly such
 * an invalid value - measured on this silicon, where the masked write
 * moved the oscillator AND raised BADWRITE. The second is that
 * FREQ_RANGE RESETS TO 0xaa0, which is none of its own four codes, so
 * carrying the field over into a write is refused as well: every verb
 * below writes the whole word with both passwords made legal, LOW
 * standing in for the reset value (the eight-stage loop it already
 * behaves as), and start() does not write CTRL at all when the
 * oscillator is already running. The crystal oscillator's CTRL is the
 * same register design and gets the same whole-word treatment.
 */
struct Rosc {
    Rosc() = delete;

    static bool running() { return (ROSC->STATUS & ROSC_STATUS_ENABLED_BITS) != 0u; }
    static bool stable() { return (ROSC->STATUS & ROSC_STATUS_STABLE_BITS) != 0u; }
    /// Whether the output divider is running - the divider is what every
    /// consumer of this oscillator actually sees.
    static bool div_running() { return (ROSC->STATUS & ROSC_STATUS_DIV_RUNNING_BITS) != 0u; }

    /// Start (or keep) the oscillator and wait, bounded, for STABLE. An
    /// oscillator already enabled is NOT written to: at boot this one is
    /// running with a FREQ_RANGE that is none of its own four codes (the
    /// reset value 0xaa0), and writing that back would be the invalid
    /// value BADWRITE is for.
    static bool start() {
        if (!running()) {
            write_ctrl(ROSC_CTRL_ENABLE_VALUE_ENABLE, legal_range_code());
        }
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (stable()) {
                return true;
            }
        }
        return false;
    }

    /// Stop the oscillator. Only with clk_ref and clk_sys elsewhere:
    /// stopping the clock a core runs on stops the core.
    static void stop() { write_ctrl(ROSC_CTRL_ENABLE_VALUE_DISABLE, legal_range_code()); }

    /// The stage range as the register holds it. At reset the field
    /// holds neither of the four codes, and the loop behaves as LOW.
    static uint16_t range_code() {
        return static_cast<uint16_t>(ROSC->CTRL & ROSC_CTRL_FREQ_RANGE_BITS);
    }
    /// One step of range. 8.3.5: increasing does not glitch the output,
    /// DECREASING DOES - so a caller lowering the range must have moved
    /// clk_ref and clk_sys away first, or hold in reset whatever this
    /// oscillator clocks.
    static void range(RoscRange r) {
        write_ctrl(legal_enable_code(), static_cast<uint16_t>(r));
    }

    /// The drive strength of one of the eight stages, 0..3 (0 the
    /// default, each step another drive transistor and a shorter delay,
    /// with a diminishing effect). Stages 0..3 live in FREQA and 4..7 in
    /// FREQB, both behind the same password; a stage the range has taken
    /// out of the loop still propagates the signal, so 8.3.5 asks that
    /// its drive be at least the lowest of the stages still in.
    static void drive(uint8_t stage, uint8_t strength) {
        if (stage > 7u || strength > 3u) {
            return;
        }
        const uint32_t shift = 4u * (stage & 3u);
        volatile uint32_t& reg = stage < 4u ? ROSC->FREQA : ROSC->FREQB;
        const uint32_t kept = reg & ~(0x7u << shift) & 0xFFFFu;
        reg = freq_password | kept | (static_cast<uint32_t>(strength) << shift);
    }
    static uint8_t drive(uint8_t stage) {
        if (stage > 7u) {
            return 0;
        }
        const uint32_t shift = 4u * (stage & 3u);
        const uint32_t reg = stage < 4u ? ROSC->FREQA : ROSC->FREQB;
        return static_cast<uint8_t>((reg >> shift) & 0x7u);
    }

    /// The randomiser of 8.3.6: the first two stages' drive strengths
    /// taken from an LFSR instead of from FREQA. Both stages together is
    /// what the datasheet recommends; one alone gives about half the
    /// frequency rise.
    static void randomise(bool stage0, bool stage1) {
        const uint32_t kept = ROSC->FREQA & 0xFFFFu &
                              ~(ROSC_FREQA_DS0_RANDOM_BITS | ROSC_FREQA_DS1_RANDOM_BITS);
        ROSC->FREQA = freq_password | kept |
                      (stage0 ? ROSC_FREQA_DS0_RANDOM_BITS : 0u) |
                      (stage1 ? ROSC_FREQA_DS1_RANDOM_BITS : 0u);
    }
    static bool randomised() {
        return (ROSC->FREQA & (ROSC_FREQA_DS0_RANDOM_BITS | ROSC_FREQA_DS1_RANDOM_BITS)) != 0u;
    }
    /// Seed the randomiser's LFSR. Legal at any time; it restarts the
    /// sequence.
    static void seed(uint32_t value) { ROSC->RANDOM = value; }

    /// The output divider (8.3.7), written behind its own password as
    /// 0xaa00 + div: 1..127 divide by that, and EVERY OTHER VALUE - zero
    /// among them - divides by 128, which is why this verb takes the
    /// divisor and not the word. Changeable while running, and the
    /// output does not glitch.
    static void divider(uint8_t div) {
        ROSC->DIV = ROSC_DIV_VALUE_PASS | static_cast<uint32_t>(div);
    }
    /// The divisor in force: the register's low byte, with the block's
    /// own rule applied - what is not 1..127 is 128.
    static uint8_t divider() {
        const uint8_t div = static_cast<uint8_t>(ROSC->DIV & 0xFFu);
        return (div == 0u || div > 127u) ? 128u : div;
    }

    /// One bit off the oscillator (8.3.8). Random only while the cores
    /// run on ANOTHER clock: read from a core this oscillator clocks, the
    /// read is correlated with the phase it samples. Not a security
    /// source by the datasheet's own words.
    static bool random_bit() { return (ROSC->RANDOMBIT & ROSC_RANDOMBIT_BITS) != 0u; }

    /// The same countdown the crystal has (8.3.9), in this oscillator's
    /// periods.
    static bool wait_periods(uint16_t periods) {
        if (!running()) {
            return false;
        }
        ROSC->COUNT = periods;
        while ((ROSC->COUNT & ROSC_COUNT_BITS) != 0u) {
        }
        return true;
    }
    static uint16_t count() { return static_cast<uint16_t>(ROSC->COUNT & ROSC_COUNT_BITS); }

    /// A write the block refused for a wrong password. Write-to-clear.
    static bool badwrite() { return (ROSC->STATUS & ROSC_STATUS_BADWRITE_BITS) != 0u; }
    static void clear_badwrite() { ROSC->STATUS = ROSC_STATUS_BADWRITE_BITS; }

    /// DORMANT (8.3.10, 6.5.3): the same keyword as the crystal's, the
    /// same contract - it returns once a configured wake has restarted
    /// the oscillator, this one in about a microsecond against the
    /// crystal's milliseconds.
    static void dormant() {
        ROSC->DORMANT = ROSC_DORMANT_VALUE_DORMANT;
        while (!stable()) {
        }
    }

private:
    static constexpr uint32_t freq_password = ROSC_FREQA_PASSWD_VALUE_PASS
                                              << ROSC_FREQA_PASSWD_LSB;
    /// The ENABLE password as CTRL holds it, made legal: anything that
    /// is neither code means enabled, which is the block's own rule for
    /// an invalid setting.
    static uint16_t legal_enable_code() {
        const uint16_t code = static_cast<uint16_t>((ROSC->CTRL & ROSC_CTRL_ENABLE_BITS) >>
                                                    ROSC_CTRL_ENABLE_LSB);
        return code == ROSC_CTRL_ENABLE_VALUE_DISABLE
                   ? static_cast<uint16_t>(ROSC_CTRL_ENABLE_VALUE_DISABLE)
                   : static_cast<uint16_t>(ROSC_CTRL_ENABLE_VALUE_ENABLE);
    }
    /// The range as CTRL holds it, made legal: the field RESETS TO 0xaa0,
    /// which is none of its four codes, and writing that back would be
    /// refused - so it is carried over as LOW, the eight-stage loop it
    /// already behaves as.
    static uint16_t legal_range_code() {
        switch (range_code()) {
            case ROSC_CTRL_FREQ_RANGE_VALUE_MEDIUM:
                return static_cast<uint16_t>(ROSC_CTRL_FREQ_RANGE_VALUE_MEDIUM);
            case ROSC_CTRL_FREQ_RANGE_VALUE_HIGH:
                return static_cast<uint16_t>(ROSC_CTRL_FREQ_RANGE_VALUE_HIGH);
            case ROSC_CTRL_FREQ_RANGE_VALUE_TOOHIGH:
                return static_cast<uint16_t>(ROSC_CTRL_FREQ_RANGE_VALUE_TOOHIGH);
            default:
                return static_cast<uint16_t>(ROSC_CTRL_FREQ_RANGE_VALUE_LOW);
        }
    }
    /// Both passwords, one store.
    static void write_ctrl(uint32_t enable, uint32_t range) {
        ROSC->CTRL = (enable << ROSC_CTRL_ENABLE_LSB) | (range & ROSC_CTRL_FREQ_RANGE_BITS);
    }
};

// ---- the low-power oscillator (8.4) --------------------------------------------

/**
 * The 32.768 kHz RC oscillator of the ALWAYS-ON domain.
 *
 * It is not started and cannot be stopped from here: it runs as soon as
 * the core supply is up and the power-on reset is released, it sequences
 * the chip's own start-up, and it is what the always-on timer counts
 * while the switched core is powered down. What software has is a TRIM -
 * 63 steps of one to three per cent each, which bring an initial
 * accuracy of plus or minus 20 per cent to about 1.5 - and the two
 * registers that TELL the always-on timer what rate to assume.
 *
 * ITS REGISTERS LIVE IN POWMAN, the always-on block, and they are
 * PASSWORD PROTECTED: a write whose top sixteen bits are not 0x5AFE is
 * ignored and sets BADPASSWD. So every write here is a whole word with
 * the password in it - never one of the atomic aliases, which would
 * carry bits and not a password. The block itself is the power chapter's;
 * this oscillator is 8.4's, which is why its verbs are here.
 */
struct Lposc {
    Lposc() = delete;

    /// The trim code, 0..63: 32 is the factory default, below it slower,
    /// above it faster, each step one to three per cent of the initial
    /// frequency.
    static uint8_t trim() {
        return static_cast<uint8_t>((POWMAN->LPOSC & POWMAN_LPOSC_TRIM_BITS) >>
                                    POWMAN_LPOSC_TRIM_LSB);
    }
    static void trim(uint8_t code) {
        if (code > 63u) {
            return;
        }
        const uint32_t kept = POWMAN->LPOSC & POWMAN_LPOSC_BITS & ~POWMAN_LPOSC_TRIM_BITS;
        POWMAN->LPOSC = password | kept | (static_cast<uint32_t>(code) << POWMAN_LPOSC_TRIM_LSB);
    }
    /// The MODE field as the block holds it (its reset value is 3, the
    /// low-power oscillator's own mode; the other codes are the always-on
    /// domain's business and no verb here writes them).
    static uint8_t mode() {
        return static_cast<uint8_t>(POWMAN->LPOSC & POWMAN_LPOSC_MODE_BITS);
    }

    /// What the always-on timer is TOLD this oscillator runs at: an
    /// integer number of kilohertz and a 16-bit fraction of one. The
    /// reset pair is 32 and 0xc49c, which is 32.768 kHz. These two
    /// registers change no oscillator - they change the arithmetic a
    /// counter does with its ticks.
    static uint8_t freq_khz_int() {
        return static_cast<uint8_t>(POWMAN->LPOSC_FREQ_KHZ_INT & POWMAN_LPOSC_FREQ_KHZ_INT_BITS);
    }
    static uint16_t freq_khz_frac() {
        return static_cast<uint16_t>(POWMAN->LPOSC_FREQ_KHZ_FRAC &
                                     POWMAN_LPOSC_FREQ_KHZ_FRAC_BITS);
    }
    static void freq_khz(uint8_t whole, uint16_t frac) {
        POWMAN->LPOSC_FREQ_KHZ_INT = password | (whole & POWMAN_LPOSC_FREQ_KHZ_INT_BITS);
        POWMAN->LPOSC_FREQ_KHZ_FRAC = password | frac;
    }
    /// The pair above as hertz, which is what a program comparing it
    /// against a measurement wants.
    static uint32_t declared_hz() {
        return static_cast<uint32_t>(freq_khz_int()) * 1000u +
               (static_cast<uint32_t>(freq_khz_frac()) * 1000u) / 65536u;
    }

    /// A write to a protected register that carried the wrong password:
    /// write-to-clear, and the proof that a password is a password.
    static bool bad_password() { return (POWMAN->BADPASSWD & POWMAN_BADPASSWD_BITS) != 0u; }
    static void clear_bad_password() { POWMAN->BADPASSWD = POWMAN_BADPASSWD_BITS; }

private:
    static constexpr uint32_t password = 0x5AFEu << 16;
};

// ---- the two PLLs (8.6) ---------------------------------------------------------

/// One PLL block (8.6): the system PLL and the USB PLL are the same
/// register file at two addresses behind two reset bits.
template <uint32_t base, uint32_t reset_bit, IRQn_Type irq_line>
struct PllBlock {
    PllBlock() = delete;

    static PLL_SYS_Type& regs() { return *reinterpret_cast<PLL_SYS_Type*>(base); }

    /// Bring the PLL up on `cfg` (8.6.4's sequence): out of reset,
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
        // A ratio just written is not a lock that was lost: the sticky
        // flag is cleared so that what stands afterwards is news.
        clear_lock_lost();
        return true;
    }

    static bool locked() { return (regs().CS & PLL_CS_LOCK_BITS) != 0u; }
    /// The live complement of LOCK, which the chapter offers beside it.
    static bool unlocked() { return (regs().CS & PLL_CS_LOCK_N_BITS) != 0u; }

    /// THE LOCK THAT WAS LOST, which the RP2040 had no way to see: a
    /// sticky bit that stands from the moment the PLL falls out of lock
    /// until it is cleared, whether or not anybody was watching, and
    /// which can raise this block's own interrupt.
    static bool lock_lost() { return (regs().INTR & PLL_INTR_LOCK_N_STICKY_BITS) != 0u; }
    static void clear_lock_lost() { regs().INTR = PLL_INTR_LOCK_N_STICKY_BITS; }
    static void lock_lost_interrupt(bool on) {
        hw_write_masked(regs().INTE, on ? PLL_INTE_LOCK_N_STICKY_BITS : 0u,
                        PLL_INTE_LOCK_N_STICKY_BITS);
    }
    static bool lock_lost_interrupt() { return (regs().INTE & PLL_INTE_LOCK_N_STICKY_BITS) != 0u; }
    /// The interrupt as the core would see it (INTS = INTR masked by
    /// INTE, or forced through INTF).
    static bool interrupt_pending() { return (regs().INTS & PLL_INTS_LOCK_N_STICKY_BITS) != 0u; }
    static void force_interrupt(bool on) {
        hw_write_masked(regs().INTF, on ? PLL_INTF_LOCK_N_STICKY_BITS : 0u,
                        PLL_INTF_LOCK_N_STICKY_BITS);
    }
    /// This PLL's interrupt line - the app binds `isr_pll_sys` or
    /// `isr_pll_usb`, whichever block it armed.
    static constexpr IRQn_Type irq() { return irq_line; }

    /// The ISR body: true when this block's lock-loss interrupt was
    /// standing, in which case it has been cleared. A lost lock is not
    /// something a driver can mend - the consumer of the clock decides.
    [[gnu::always_inline]] static bool isr() {
        if (!interrupt_pending()) {
            return false;
        }
        clear_lock_lost();
        return true;
    }

    /// BYPASS (8.6.5): the reference passed straight through to the
    /// output, the VCO out of the picture. The output cannot be used
    /// while this bit changes, which is why nothing in the task touches
    /// it - it is here because the chapter has it.
    static void bypass(bool on) {
        hw_write_masked(regs().CS, on ? PLL_CS_BYPASS_BITS : 0u, PLL_CS_BYPASS_BITS);
    }
    static bool bypassed() { return (regs().CS & PLL_CS_BYPASS_BITS) != 0u; }

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
    /// Whether any of the three power-down bits stands.
    static bool stopped() {
        return (regs().PWR & (PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS)) != 0u;
    }
};

/// The system PLL: clk_sys's source under Clock<ClockSource::pll>.
using PllSys = PllBlock<PLL_SYS_BASE, ResetBlock::pll_sys, PLL_SYS_IRQ_IRQn>;
/// The USB PLL: 48 MHz for the USB controller and for clk_adc.
using PllUsb = PllBlock<PLL_USB_BASE, ResetBlock::pll_usb, PLL_USB_IRQ_IRQn>;

constexpr bool operator==(const PllConfig& a, const PllConfig& b) {
    return a.refdiv == b.refdiv && a.fbdiv == b.fbdiv && a.postdiv1 == b.postdiv1 &&
           a.postdiv2 == b.postdiv2;
}

// ---- the clock generators (8.1.3) --------------------------------------------------

/// clk_ref's glitchless sources (CLK_REF_CTRL.SRC). The low-power
/// oscillator is this chip's addition.
enum class RefSource : uint8_t {
    rosc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH,
    aux = CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX,
    xosc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
    lposc = CLOCKS_CLK_REF_CTRL_SRC_VALUE_LPOSC_CLKSRC,
};

/// clk_ref's aux sources (CLK_REF_CTRL.AUXSRC), reached by selecting
/// RefSource::aux. The USB PLL and the two GPIO inputs - the crystal and
/// the two on-chip oscillators are on the glitchless mux instead.
enum class RefAux : uint8_t {
    pll_usb = CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    gpin0 = CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
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

/// clk_usb's aux sources (CLK_USB_CTRL.AUXSRC): the controller wants
/// 48 MHz, the USB PLL's own rate.
enum class UsbAux : uint8_t {
    pll_usb = CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    pll_sys = CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    rosc = CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH,
    xosc = CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

/// clk_adc's aux sources (CLK_ADC_CTRL.AUXSRC): the converter wants
/// 48 MHz too (12.4).
enum class AdcAux : uint8_t {
    pll_usb = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    pll_sys = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    rosc = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH,
    xosc = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
    gpin0 = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

/// clk_hstx's aux sources (CLK_HSTX_CTRL.AUXSRC): the high-speed
/// transmit interface, which the RP2040 had not - and which is the one
/// generator with NO ring oscillator among its sources.
enum class HstxAux : uint8_t {
    clk_sys = CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS,
    pll_sys = CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
    pll_usb = CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    gpin0 = CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0,
    gpin1 = CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_GPIN1,
};

/// One bit per clock endpoint in the two gate registers (8.1.3.5: the
/// same layout for SLEEP_ENx and WAKE_ENx, and ENABLEDx is what stands
/// this instant): `en0` the CLOCKS_WAKE_EN0 bits, `en1` the thirty-one
/// of WAKE_EN1. Every gate is open at reset, which is why a program that
/// composes a set composes it from the block bits its own chapters
/// publish.
struct SleepClocks {
    uint32_t en0 = 0xFFFF'FFFFu;
    uint32_t en1 = CLOCKS_SLEEP_EN1_BITS;

    constexpr SleepClocks operator|(SleepClocks o) const { return {en0 | o.en0, en1 | o.en1}; }
    constexpr SleepClocks operator&(SleepClocks o) const { return {en0 & o.en0, en1 & o.en1}; }
    constexpr SleepClocks operator~() const { return {~en0, ~en1 & CLOCKS_SLEEP_EN1_BITS}; }
    constexpr bool operator==(const SleepClocks&) const = default;
};

/// Every gate open: the reset value, and a sleep that prunes nothing.
inline constexpr SleepClocks sleep_clocks_all{};
/// Every gate shut - which is not a state a running program survives,
/// and is here as the empty set a composition starts from.
inline constexpr SleepClocks sleep_clocks_none{0u, 0u};

struct Clocks {
    Clocks() = delete;

    // ---- clk_ref and clk_sys, the two with a glitchless mux ------------

    /// clk_ref's glitchless mux onto `src`, SELECTED polled (8.1.3.2).
    static bool ref_select(RefSource src) {
        hw_write_masked(CLOCKS->CLK_REF_CTRL,
                        static_cast<uint32_t>(src) << CLOCKS_CLK_REF_CTRL_SRC_LSB,
                        CLOCKS_CLK_REF_CTRL_SRC_BITS);
        return wait_selected(CLOCKS->CLK_REF_SELECTED, static_cast<uint8_t>(src));
    }

    /// clk_ref onto one of its AUX sources: the glitchless mux is moved
    /// off aux first (onto the ring oscillator, the source this chip
    /// boots on and the one always startable), the aux select changed,
    /// then the mux back onto aux.
    static bool ref_from_aux(RefAux aux) {
        if (ref_source() == static_cast<uint8_t>(RefSource::aux) &&
            !ref_select(RefSource::rosc)) {
            return false;
        }
        hw_write_masked(CLOCKS->CLK_REF_CTRL,
                        static_cast<uint32_t>(aux) << CLOCKS_CLK_REF_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_REF_CTRL_AUXSRC_BITS);
        return ref_select(RefSource::aux);
    }

    /// The glitchless mux's current position for clk_ref.
    static uint8_t ref_source() {
        return static_cast<uint8_t>(CLOCKS->CLK_REF_CTRL & CLOCKS_CLK_REF_CTRL_SRC_BITS);
    }
    static RefAux ref_aux_source() {
        return static_cast<RefAux>((CLOCKS->CLK_REF_CTRL & CLOCKS_CLK_REF_CTRL_AUXSRC_BITS) >>
                                   CLOCKS_CLK_REF_CTRL_AUXSRC_LSB);
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
    static SysAux sys_aux_source() {
        return static_cast<SysAux>((CLOCKS->CLK_SYS_CTRL & CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS) >>
                                   CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB);
    }

    /// clk_ref's divider: an INTEGER of eight bits and no fraction,
    /// 1 = undivided. It is what a crystal above the 25 MHz clk_ref
    /// ceiling is brought down with (8.2.2).
    static void ref_divider(uint8_t div) {
        CLOCKS->CLK_REF_DIV = (static_cast<uint32_t>(div) << CLOCKS_CLK_REF_DIV_INT_LSB) &
                              CLOCKS_CLK_REF_DIV_INT_BITS;
    }
    static uint8_t ref_divider() {
        return static_cast<uint8_t>((CLOCKS->CLK_REF_DIV & CLOCKS_CLK_REF_DIV_INT_BITS) >>
                                    CLOCKS_CLK_REF_DIV_INT_LSB);
    }

    /// clk_sys's divider, 16.16 where the RP2040's was 24.8: the integer
    /// part in 1..65535 and a fraction in 65536ths, which the divider
    /// realizes by alternating between two integers - a jittery clock
    /// that is honest about being one. Changeable on the fly: the
    /// divider synchronizes the change to the end of a cycle.
    static void sys_divider(uint32_t div_int, uint16_t div_frac = 0) {
        CLOCKS->CLK_SYS_DIV = ((div_int << CLOCKS_CLK_SYS_DIV_INT_LSB) &
                               CLOCKS_CLK_SYS_DIV_INT_BITS) |
                              div_frac;
    }
    static uint32_t sys_divider() {
        return (CLOCKS->CLK_SYS_DIV & CLOCKS_CLK_SYS_DIV_INT_BITS) >> CLOCKS_CLK_SYS_DIV_INT_LSB;
    }
    static uint16_t sys_divider_frac() {
        return static_cast<uint16_t>(CLOCKS->CLK_SYS_DIV & CLOCKS_CLK_SYS_DIV_FRAC_BITS);
    }

    // ---- the four generators with an aux mux alone ----------------------
    //
    // Each is stopped, its ENABLED polled DOWN, its aux select and
    // divider written, then started and its ENABLED polled up - 8.1.3.2's
    // third sequence, with the datasheet's own advice for the wait: poll
    // the status, do not count instructions, because the generated clock
    // may be far slower than the one counting.

    /// clk_peri: the UARTs' and SPIs' clock, with a two-bit integer
    /// divider (1..3) and no fraction.
    static bool peri_select(PeriAux aux, uint8_t div = 1) {
        return aux_select(CLOCKS->CLK_PERI_CTRL, CLOCKS->CLK_PERI_DIV, static_cast<uint32_t>(aux),
                          div_word(div, CLOCKS_CLK_PERI_DIV_INT_LSB, CLOCKS_CLK_PERI_DIV_INT_BITS));
    }
    static void peri_stop() { generator_stop(CLOCKS->CLK_PERI_CTRL); }
    static bool peri_enabled() { return generator_enabled(CLOCKS->CLK_PERI_CTRL); }
    static PeriAux peri_source() {
        return static_cast<PeriAux>(aux_of(CLOCKS->CLK_PERI_CTRL));
    }
    static uint8_t peri_divider() {
        return static_cast<uint8_t>((CLOCKS->CLK_PERI_DIV & CLOCKS_CLK_PERI_DIV_INT_BITS) >>
                                    CLOCKS_CLK_PERI_DIV_INT_LSB);
    }

    /// clk_usb: 48 MHz for the controller, with a four-bit divider.
    static bool usb_select(UsbAux aux, uint8_t div = 1) {
        return aux_select(CLOCKS->CLK_USB_CTRL, CLOCKS->CLK_USB_DIV, static_cast<uint32_t>(aux),
                          div_word(div, CLOCKS_CLK_USB_DIV_INT_LSB, CLOCKS_CLK_USB_DIV_INT_BITS));
    }
    static void usb_stop() { generator_stop(CLOCKS->CLK_USB_CTRL); }
    static bool usb_enabled() { return generator_enabled(CLOCKS->CLK_USB_CTRL); }
    static UsbAux usb_source() { return static_cast<UsbAux>(aux_of(CLOCKS->CLK_USB_CTRL)); }
    static uint8_t usb_divider() {
        return static_cast<uint8_t>((CLOCKS->CLK_USB_DIV & CLOCKS_CLK_USB_DIV_INT_BITS) >>
                                    CLOCKS_CLK_USB_DIV_INT_LSB);
    }

    /// clk_adc: 48 MHz for the converter, with a four-bit divider.
    static bool adc_select(AdcAux aux, uint8_t div = 1) {
        return aux_select(CLOCKS->CLK_ADC_CTRL, CLOCKS->CLK_ADC_DIV, static_cast<uint32_t>(aux),
                          div_word(div, CLOCKS_CLK_ADC_DIV_INT_LSB, CLOCKS_CLK_ADC_DIV_INT_BITS));
    }
    static void adc_stop() { generator_stop(CLOCKS->CLK_ADC_CTRL); }
    static bool adc_enabled() { return generator_enabled(CLOCKS->CLK_ADC_CTRL); }
    static AdcAux adc_source() { return static_cast<AdcAux>(aux_of(CLOCKS->CLK_ADC_CTRL)); }
    static uint8_t adc_divider() {
        return static_cast<uint8_t>((CLOCKS->CLK_ADC_DIV & CLOCKS_CLK_ADC_DIV_INT_BITS) >>
                                    CLOCKS_CLK_ADC_DIV_INT_LSB);
    }

    /// clk_hstx: the high-speed transmit interface's clock, with a
    /// two-bit divider. This chip's own generator - the RP2040 had none.
    static bool hstx_select(HstxAux aux, uint8_t div = 1) {
        return aux_select(CLOCKS->CLK_HSTX_CTRL, CLOCKS->CLK_HSTX_DIV, static_cast<uint32_t>(aux),
                          div_word(div, CLOCKS_CLK_HSTX_DIV_INT_LSB, CLOCKS_CLK_HSTX_DIV_INT_BITS));
    }
    static void hstx_stop() { generator_stop(CLOCKS->CLK_HSTX_CTRL); }
    static bool hstx_enabled() { return generator_enabled(CLOCKS->CLK_HSTX_CTRL); }
    static HstxAux hstx_source() { return static_cast<HstxAux>(aux_of(CLOCKS->CLK_HSTX_CTRL)); }
    static uint8_t hstx_divider() {
        return static_cast<uint8_t>((CLOCKS->CLK_HSTX_DIV & CLOCKS_CLK_HSTX_DIV_INT_BITS) >>
                                    CLOCKS_CLK_HSTX_DIV_INT_LSB);
    }

    // ---- the top-level clock gates (8.1.3.5) ----------------------------

    /// The gates that apply while the chip is in its SLEEP state (both
    /// cores asleep, the DMA idle). All open at reset, which means sleep
    /// does nothing until a program says what it may stop.
    static void sleep_enables(SleepClocks c) {
        CLOCKS->SLEEP_EN0 = c.en0;
        CLOCKS->SLEEP_EN1 = c.en1 & CLOCKS_SLEEP_EN1_BITS;
    }
    static SleepClocks sleep_enables() {
        return {.en0 = CLOCKS->SLEEP_EN0, .en1 = CLOCKS->SLEEP_EN1 & CLOCKS_SLEEP_EN1_BITS};
    }
    /// The gates that apply AWAKE: a destination gated here is off until
    /// the gate opens again, its configuration kept. All open at reset.
    static void wake_enables(SleepClocks c) {
        CLOCKS->WAKE_EN0 = c.en0;
        CLOCKS->WAKE_EN1 = c.en1 & CLOCKS_WAKE_EN1_BITS;
    }
    static SleepClocks wake_enables() {
        return {.en0 = CLOCKS->WAKE_EN0, .en1 = CLOCKS->WAKE_EN1 & CLOCKS_WAKE_EN1_BITS};
    }
    /// The gates as they stand this instant (ENABLED0/1): the wake set,
    /// read from an awake core.
    static SleepClocks enabled() {
        return {.en0 = CLOCKS->ENABLED0, .en1 = CLOCKS->ENABLED1 & CLOCKS_ENABLED1_BITS};
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

    /// ENABLE, ENABLED, KILL and AUXSRC sit at the same bit positions in
    /// every generator that has them, which is why one helper serves the
    /// four.
    static bool wait_enabled(volatile uint32_t& ctrl, bool want) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            const bool now = (ctrl & CLOCKS_CLK_PERI_CTRL_ENABLED_BITS) != 0u;
            if (now == want) {
                return true;
            }
        }
        return false;
    }
    static void generator_stop(volatile uint32_t& ctrl) {
        hw_clear(ctrl, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
        (void)wait_enabled(ctrl, false);
    }
    static bool generator_enabled(volatile uint32_t& ctrl) {
        return (ctrl & CLOCKS_CLK_PERI_CTRL_ENABLE_BITS) != 0u;
    }
    static uint8_t aux_of(volatile uint32_t& ctrl) {
        return static_cast<uint8_t>((ctrl & CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS) >>
                                    CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB);
    }
    static constexpr uint32_t div_word(uint8_t div, uint32_t lsb, uint32_t bits) {
        return (static_cast<uint32_t>(div) << lsb) & bits;
    }
    /// The whole sequence for a generator with no glitchless mux. False
    /// when it would not stop or would not restart - either of which
    /// means its source is not running.
    static bool aux_select(volatile uint32_t& ctrl, volatile uint32_t& div, uint32_t aux,
                           uint32_t div_value) {
        hw_clear(ctrl, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
        if (!wait_enabled(ctrl, false)) {
            return false;
        }
        div = div_value;
        hw_write_masked(ctrl, aux << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS);
        hw_set(ctrl, CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
        return wait_enabled(ctrl, true);
    }
};

// ---- the resus circuit (8.1.5) ---------------------------------------------------

/**
 * The watchdog of clk_sys itself: when no edge of clk_sys arrives within
 * TIMEOUT cycles of clk_ref, the circuit forces the glitchless mux back
 * onto clk_ref and raises the CLOCKS interrupt. It exists so that a
 * program which stopped its own system clock still answers a debugger
 * instead of locking up beyond diagnosis.
 *
 * IT IS A DEBUGGING AID AND THE DATASHEET SAYS SO. A clk_sys that is
 * merely SLOWER than the timeout assumes is indistinguishable from one
 * that stopped, so an armed resus on a program that divides clk_sys down
 * is a trap of its own making; and there is no way back at all if
 * clk_ref is what stopped. Nothing in this stratum arms it - a program
 * does, knowingly.
 *
 * After a resus the way back is: reconfigure clk_sys (Clock::init()),
 * then clear() - which is what releases the forced selection.
 */
struct Resus {
    Resus() = delete;

    /// Arm the circuit with a timeout in clk_ref cycles, 1..255.
    static void enable(uint8_t timeout) {
        CLOCKS->CLK_SYS_RESUS_CTRL =
            CLOCKS_CLK_SYS_RESUS_CTRL_ENABLE_BITS |
            (static_cast<uint32_t>(timeout) & CLOCKS_CLK_SYS_RESUS_CTRL_TIMEOUT_BITS);
    }
    static void disable() { hw_clear(CLOCKS->CLK_SYS_RESUS_CTRL, CLOCKS_CLK_SYS_RESUS_CTRL_ENABLE_BITS); }
    static bool enabled() {
        return (CLOCKS->CLK_SYS_RESUS_CTRL & CLOCKS_CLK_SYS_RESUS_CTRL_ENABLE_BITS) != 0u;
    }
    static uint8_t timeout() {
        return static_cast<uint8_t>(CLOCKS->CLK_SYS_RESUS_CTRL &
                                    CLOCKS_CLK_SYS_RESUS_CTRL_TIMEOUT_BITS);
    }

    /// Whether a resus has happened and still stands (STATUS.RESUSSED).
    static bool resussed() {
        return (CLOCKS->CLK_SYS_RESUS_STATUS & CLOCKS_CLK_SYS_RESUS_STATUS_RESUSSED_BITS) != 0u;
    }
    /// Release the forced selection: clk_sys follows CLK_SYS_CTRL again.
    /// Do this only once clk_sys has a source that runs.
    static void clear() { hw_set(CLOCKS->CLK_SYS_RESUS_CTRL, CLOCKS_CLK_SYS_RESUS_CTRL_CLEAR_BITS); }
    /// Force a resus from software: the same act the circuit performs,
    /// on demand, which is the only way to exercise the path without
    /// stopping clk_sys for real.
    static void force() { hw_set(CLOCKS->CLK_SYS_RESUS_CTRL, CLOCKS_CLK_SYS_RESUS_CTRL_FRCE_BITS); }
    static void unforce() {
        hw_clear(CLOCKS->CLK_SYS_RESUS_CTRL, CLOCKS_CLK_SYS_RESUS_CTRL_FRCE_BITS);
    }

    /// The block's one interrupt (INTE/INTS/INTF/INTR over the single
    /// CLK_SYS_RESUS bit), on the CLOCKS line - an app binds
    /// `isr_clocks`.
    static void interrupt(bool on) {
        hw_write_masked(CLOCKS->INTE, on ? CLOCKS_INTE_CLK_SYS_RESUS_BITS : 0u,
                        CLOCKS_INTE_CLK_SYS_RESUS_BITS);
    }
    static bool interrupt() { return (CLOCKS->INTE & CLOCKS_INTE_CLK_SYS_RESUS_BITS) != 0u; }
    static bool raw() { return (CLOCKS->INTR & CLOCKS_INTR_CLK_SYS_RESUS_BITS) != 0u; }
    static bool pending() { return (CLOCKS->INTS & CLOCKS_INTS_CLK_SYS_RESUS_BITS) != 0u; }
    static void force_interrupt(bool on) {
        hw_write_masked(CLOCKS->INTF, on ? CLOCKS_INTF_CLK_SYS_RESUS_BITS : 0u,
                        CLOCKS_INTF_CLK_SYS_RESUS_BITS);
    }
    static constexpr IRQn_Type irq() { return CLOCKS_IRQ_IRQn; }

    /// The ISR body: true when a resus was standing. It does NOT clear
    /// the resus - clearing it hands clk_sys back to a mux setting that
    /// may still be dead, so the handler reports and the program decides.
    [[gnu::always_inline]] static bool isr() { return pending(); }
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

/// How many clk_ref cycles make one microsecond - 12 on a 12 MHz
/// crystal, which is what every consumer of a tick is set to. Zero when
/// clk_ref is not a whole number of megahertz, there being no honest
/// count then.
constexpr uint32_t tick_cycles_per_us(uint32_t ref_hz) {
    return (ref_hz == 0u || (ref_hz % 1'000'000UL) != 0u) ? 0u : ref_hz / 1'000'000UL;
}

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
    static_assert(static_cast<uint8_t>(which) <= static_cast<uint8_t>(TickConsumer::riscv),
                  "brio TickGenerator: the TICKS block of the RP2350 has six generators - "
                  "the two SysTicks, the two system timers, the watchdog and the RISC-V "
                  "platform timer (datasheet 8.5.2)");
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

// ---- the frequency counter (8.1.4) ------------------------------------------------

/// What the counter can count: the roots and the generators of the tree
/// (FC0_SRC's values - NOT the RP2040's numbering).
enum class CountSource : uint8_t {
    none = CLOCKS_FC0_SRC_VALUE_NULL,
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
    otp = CLOCKS_FC0_SRC_VALUE_OTP_CLK2FC,
};

/// What the counter says besides the number: its own verdict between the
/// two bounds a caller may set, and whether the source died mid-count.
struct FcMeasurement {
    uint32_t hz = 0;
    bool done = false;   ///< the count finished at all
    bool pass = false;   ///< within [min_khz, max_khz]
    bool slow = false;
    bool fast = false;
    bool died = false;   ///< the source stopped while it was being counted
};

/// The counting window of an FC0_INTERVAL code, in microseconds of
/// clk_ref (8.1.4's table: 2^interval up to 6, then 125 us doubling).
constexpr uint32_t fc_interval_us(uint8_t interval) {
    if (interval <= 6u) {
        return 1UL << interval;
    }
    return 125UL << (interval - 7u);
}

/**
 * The frequency counter: a source counted against clk_ref over the
 * window FC0_INTERVAL names, the result in kHz with five fraction bits.
 * It is the chip's own ratio check - with clk_ref on the crystal, every
 * count is "how many of the crystal's periods" - and the measure behind
 * every rate this stratum states.
 *
 * count_hz() leaves the MIN/MAX thresholds wide open, because brio judges
 * the number and not a flag; measure() is the other face, where the
 * hardware is given two bounds and asked for its own verdict.
 */
struct FreqCounter {
    FreqCounter() = delete;

    /// `ref_hz` is clk_ref's rate (the crystal's, after Clock::init());
    /// `interval` 0..15 selects the counting window, the longest the
    /// finest. Nullopt when the source is dead (STATUS.DIED: a stopped
    /// oscillator, an unwired input) or the count never finished.
    static std::optional<uint32_t> count_hz(CountSource src, uint32_t ref_hz,
                                            uint8_t interval = 15) {
        const FcMeasurement m = measure(src, ref_hz, interval, 0u, CLOCKS_FC0_MAX_KHZ_BITS);
        if (!m.done || m.died) {
            return std::nullopt;
        }
        return m.hz;
    }

    /// The same count with the hardware's own verdict: `min_khz` and
    /// `max_khz` are the window it judges against, and PASS / SLOW /
    /// FAST / DIED come back beside the number.
    static FcMeasurement measure(CountSource src, uint32_t ref_hz, uint8_t interval,
                                 uint32_t min_khz, uint32_t max_khz) {
        FcMeasurement out{};
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if ((CLOCKS->FC0_STATUS & CLOCKS_FC0_STATUS_RUNNING_BITS) == 0u) {
                break;
            }
        }
        CLOCKS->FC0_REF_KHZ = (ref_hz / 1000u) & CLOCKS_FC0_REF_KHZ_BITS;
        CLOCKS->FC0_INTERVAL = interval & CLOCKS_FC0_INTERVAL_BITS;
        CLOCKS->FC0_MIN_KHZ = min_khz & CLOCKS_FC0_MIN_KHZ_BITS;
        CLOCKS->FC0_MAX_KHZ = max_khz & CLOCKS_FC0_MAX_KHZ_BITS;
        // The source write is what starts the count.
        CLOCKS->FC0_SRC = static_cast<uint32_t>(src);
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if ((CLOCKS->FC0_STATUS & CLOCKS_FC0_STATUS_DONE_BITS) != 0u) {
                out.done = true;
                break;
            }
        }
        const uint32_t status = CLOCKS->FC0_STATUS;
        out.pass = (status & CLOCKS_FC0_STATUS_PASS_BITS) != 0u;
        out.slow = (status & CLOCKS_FC0_STATUS_SLOW_BITS) != 0u;
        out.fast = (status & CLOCKS_FC0_STATUS_FAST_BITS) != 0u;
        out.died = (status & CLOCKS_FC0_STATUS_DIED_BITS) != 0u;
        if (!out.done) {
            return out;
        }
        const uint32_t result = CLOCKS->FC0_RESULT;
        const uint32_t khz = (result & CLOCKS_FC0_RESULT_KHZ_BITS) >> CLOCKS_FC0_RESULT_KHZ_LSB;
        const uint32_t frac = result & CLOCKS_FC0_RESULT_FRAC_BITS;
        out.hz = khz * 1000u + (frac * 1000u) / 32u;
        return out;
    }

    /// The delay in clk_ref cycles the counter waits before it starts
    /// counting a newly selected source (FC0_DELAY, 0..7 as a power of
    /// two): what a source with a slow start needs.
    static void start_delay(uint8_t code) {
        CLOCKS->FC0_DELAY = code & CLOCKS_FC0_DELAY_BITS;
    }
    static uint8_t start_delay() {
        return static_cast<uint8_t>(CLOCKS->FC0_DELAY & CLOCKS_FC0_DELAY_BITS);
    }
    /// Stop counting: the NULL source, which is also how a program hands
    /// the counter back. True when RUNNING had dropped by the time it
    /// returned - the counter finishes the window it is in, and a window
    /// is up to 32 ms of clk_ref long.
    static bool stop() {
        CLOCKS->FC0_SRC = static_cast<uint32_t>(CountSource::none);
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if (!running()) {
                return true;
            }
        }
        return false;
    }
    static bool running() {
        return (CLOCKS->FC0_STATUS & CLOCKS_FC0_STATUS_RUNNING_BITS) != 0u;
    }
};

// ---- the GPIO clock outputs and inputs (8.1.6.3, 8.1.6.4) ------------------------

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
 * mux, so it is stopped while the source changes (8.1.3.2). The one
 * clock a counter on another board can measure - the crystal, or clk_sys
 * itself, on a wire.
 *
 * Beside the divider it has the three controls the always-on generators
 * have not: DUTY CYCLE CORRECTION, which puts an odd divisor's falling
 * edge back where an even one's would be; a PHASE of up to three source
 * cycles; and NUDGE, one source cycle of delay per write. They are here
 * and not on clk_usb / clk_adc / clk_hstx (which have the same three)
 * because this is the generator whose output leaves the chip, where an
 * instrument can see what they did.
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
        if (!wait_enabled(false)) {
            return false;
        }
        hw_write_masked(ctrl(),
                        static_cast<uint32_t>(source) << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB,
                        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_BITS);
        div() = (div_int << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) | div_frac;
        hw_set(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        if (!wait_enabled(true)) {
            return false;
        }
        return Pin<pin>::function(PinFunction::gpck, cfg);
    }

    /// The output off and the pin released.
    static void stop() {
        hw_clear(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
        (void)wait_enabled(false);
        (void)Pin<pin>::release();
    }

    /// Whether the generator was asked to run, and whether it is.
    static bool enabled() { return (ctrl() & CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS) != 0u; }
    static bool running() { return (ctrl() & CLOCKS_CLK_GPOUT0_CTRL_ENABLED_BITS) != 0u; }
    static GpoutSource source() {
        return static_cast<GpoutSource>((ctrl() & CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_BITS) >>
                                        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB);
    }
    static uint32_t divider() {
        return (div() & CLOCKS_CLK_GPOUT0_DIV_INT_BITS) >> CLOCKS_CLK_GPOUT0_DIV_INT_LSB;
    }
    static uint16_t divider_frac() {
        return static_cast<uint16_t>(div() & CLOCKS_CLK_GPOUT0_DIV_FRAC_BITS);
    }

    /// Duty cycle correction (8.1.3.4): an odd divisor's output is not
    /// even - divide by three and the high time is a third - and this
    /// moves the falling edge back to the source's. It does nothing at
    /// all for an even divisor. Changeable while the clock runs.
    static void duty_correction(bool on) {
        hw_write_masked(ctrl(), on ? CLOCKS_CLK_GPOUT0_CTRL_DC50_BITS : 0u,
                        CLOCKS_CLK_GPOUT0_CTRL_DC50_BITS);
    }
    static bool duty_correction() { return (ctrl() & CLOCKS_CLK_GPOUT0_CTRL_DC50_BITS) != 0u; }
    /// The enable's phase, 0..3 source cycles.
    static void phase(uint8_t cycles) {
        hw_write_masked(ctrl(),
                        (static_cast<uint32_t>(cycles) << CLOCKS_CLK_GPOUT0_CTRL_PHASE_LSB) &
                            CLOCKS_CLK_GPOUT0_CTRL_PHASE_BITS,
                        CLOCKS_CLK_GPOUT0_CTRL_PHASE_BITS);
    }
    static uint8_t phase() {
        return static_cast<uint8_t>((ctrl() & CLOCKS_CLK_GPOUT0_CTRL_PHASE_BITS) >>
                                    CLOCKS_CLK_GPOUT0_CTRL_PHASE_LSB);
    }
    /// One source cycle of delay, once per write: how two generators are
    /// brought into step without stopping either.
    static void nudge() { hw_xor(ctrl(), CLOCKS_CLK_GPOUT0_CTRL_NUDGE_BITS); }

private:
    static volatile uint32_t& ctrl() { return *(&CLOCKS->CLK_GPOUT0_CTRL + 3u * n); }
    static volatile uint32_t& div() { return *(&CLOCKS->CLK_GPOUT0_DIV + 3u * n); }
    static bool wait_enabled(bool want) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (((ctrl() & CLOCKS_CLK_GPOUT0_CTRL_ENABLED_BITS) != 0u) == want) {
                return true;
            }
        }
        return false;
    }
};

/**
 * A GPIO clock input: the pin handed to the clock function, after which
 * the generators (SysAux::gpin0, PeriAux::gpin0, RefAux::gpin0) and the
 * counter (CountSource::gpin0) can name it. No pull by default: a clock
 * is driven.
 *
 * The chapter caps these inputs at 50 MHz, and warns that a source whose
 * accuracy is worse than 1000 ppm must not drive a generator at its
 * maximum rate - the design margins assume that much.
 */
template <uint8_t n>
struct ClockIn {
    static_assert(n < 2u, "brio ClockIn: the RP2350 has two GPIO clock inputs, GPIN0 and GPIN1");
    ClockIn() = delete;

    static constexpr uint8_t pin = gpin_pin(n);
    static constexpr CountSource count_source = n == 0u ? CountSource::gpin0 : CountSource::gpin1;
    static constexpr uint32_t max_hz = 50'000'000UL;

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
    static_assert(crystal_hz <= clk_ref_max_hz,
                  "brio Clock: clk_ref may not exceed 25 MHz (datasheet 8.2.2), and this "
                  "task puts the crystal on it undivided - a faster crystal wants the "
                  "clk_ref divider and a task that knows it");
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
        return Clocks::peri_select(peri == PeriSource::sys ? PeriAux::clk_sys : PeriAux::xosc);
    }

    /// A source counted against the crystal, which is clk_ref after
    /// init(): FreqCounter::count_hz with this clock's reference.
    static std::optional<uint32_t> count_hz(CountSource what, uint8_t interval = 15) {
        return FreqCounter::count_hz(what, crystal_hz, interval);
    }
};

} // namespace brio
