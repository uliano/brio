/*
 * tcd.hpp
 *
 * The AVR DA/DB 12-bit timer/counter type D (TCD, DS40002247B ch. 25)
 * in two strata, as docs/avrdx/tcd.md describes:
 *
 *  RESOURCE - Tcd<0>: the typed view of the one instance. A config
 *  struct owns the whole configuration (route, clock and the two
 *  prescalers, waveform mode, the four compare values, the output
 *  override, the two event inputs with their modes, the fault outputs,
 *  the delay block, the dither); init<cfg>() folds it and refuses at
 *  compile time what this package or the errata cannot do, init(cfg)
 *  computes it at run time and returns false instead. Then the verbs:
 *  the THREE SYNCHRONIZATION DISCIPLINES of this peripheral (below),
 *  the double-buffered compares, the two 12-bit captures with their
 *  read-L-then-H rule, the strobes, the flags, the two ISR bodies and
 *  release() - every claimed pin handed back to PORT.
 *
 *  TASK - TcdPwm<route>: the complementary output pair with dead time,
 *  the reason this peripheral exists. One ramp mode, WOA and WOB (and
 *  optionally WOC/WOD mirroring either), a symmetric dead time on both
 *  edges, duty updates through the double buffers. The other usage
 *  types this timer can serve (the four-ramp bridge drive, the dual
 *  slope resonant converter, the capture instruments) are deliberately
 *  NOT built: they belong with their first real user.
 *
 * The three synchronization disciplines (25.3.3.1) - the whole shape of
 * this driver follows from them, because the TCD core runs on a clock
 * ASYNCHRONOUS to CLK_PER:
 *  1. ENABLE (CTRLA bit 0) may only be written while STATUS.ENRDY is
 *     '1'. Every verb that touches it waits, bounded, and returns false
 *     rather than write into a closed window.
 *  2. The CTRLE strobes (SYNC, SYNCEOC, RESTART, SCAPTUREA/B, DISEOC)
 *     and the AUPDATE path may only be issued while STATUS.CMDRDY is
 *     '1'; the same bounded wait guards them, and the double-buffered
 *     registers (CMPxSET/CLR, DLYCTRL/DLYVAL, DITCTRL/DITVAL, DBGCTRL)
 *     are written under it too.
 *  3. The STATIC registers (CTRLB, CTRLC, CTRLD, EVCTRLA/B, INPUTCTRLA/
 *     B, FAULTCTRL and every CTRLA bit except ENABLE) cannot be written
 *     while the TCD is enabled. The setters ENFORCE it: they return
 *     false and write nothing when ENABLE reads '1'.
 * FAULTCTRL is also under Configuration Change Protection (IOREG key):
 *  it goes out through _PROTECTED_WRITE.
 *
 * Facts that shape the code (25.3, 25.5, errata DS80000915F 2.14.x and
 * DS80000882C 2.13.x, plus the CLKCTRL PLL items 2.5.3/2.5.4 and 2.4.1):
 *  - the counter is 12 bits: every compare and capture is masked to
 *    0x0FFF. CAPTUREA/B are read LOW BYTE FIRST, and the read of the
 *    HIGH byte is what releases the buffer for the next capture - the
 *    resource performs the pair, so no caller can leak the discipline;
 *  - CLK_TCD comes from OSCHF, the PLL, EXTCLK or CLK_PER, then through
 *    SYNCPRES (1/2/4/8) and CNTPRES (1/4/32): the counter runs at
 *    src / (SYNCPRES * CNTPRES), the synchronizer at src / SYNCPRES;
 *  - the PLL is the only consumer-visible reason the PLL exists on this
 *    silicon: 16-24 MHz in, 32-48 MHz out, and it runs ONLY while a
 *    peripheral requests it - the TCD is that peripheral;
 *  - INPUT MODE VALIDITY (table 25-5) is a real constraint, not advice:
 *    modes 2/3/5/6 need two- or four-ramp; dual slope takes only 0 and
 *    4 once errata 2.14.3 has taken mode 7 away. tcd_input_mode_valid()
 *    is the table, checked at compile time by init<cfg>() and at run
 *    time by init(cfg);
 *  - dithering is NOT supported in dual slope (25.3.3.5): refused;
 *  - input blanking and the programmable output event share DLYCTRL, so
 *    they are one field here (TcdDelaySelect) - asking for both is not
 *    expressible;
 *  - ERRATA 2.14.1 / 2.13.1 (DB A4/A5, DA every revision): asynchronous
 *    input events (CFG = ASYNC) MISS EVENTS when CNTPRES is not DIV1.
 *    It is fixed in DB rev. B0, so it is NOT refused here - it is
 *    stated, and the workaround is to divide with SYNCPRES instead;
 *  - ERRATA 2.14.2 / 2.13.2 (DB A4/A5, DA every revision): on ANY
 *    alternate route, CMPAEN in FAULTCTRL gates ALL FOUR outputs - a
 *    WOB-only configuration on ALT1/2/3 drives nothing. No workaround
 *    but "also set CMPAEN". Rev-dependent again, so the route table
 *    below states it and does not refuse it;
 *  - ERRATA 2.14.3 / 2.13.3 (EVERY revision of both families): input
 *    mode 7 (WAITSW, halt and wait for a software RESTART) does not
 *    work with CMPASET = 0 or in dual slope. Both are REFUSED here, at
 *    compile time and at run time - no silicon revision escapes it;
 *  - routes (PORTMUX.TCDROUTEA), exactly the device headers' own enums,
 *    with the pin-level bonding facts they carry in their comments:
 *      DEFAULT  PA4 PA5 PA6 PA7   every package
 *      ALT1     PB4 PB5 -   -     48-pin (WOC/WOD PINLESS)
 *               PB4 PB5 PB6 PB7   64-pin
 *      ALT2     PF0 PF1 -   -     28-pin (WOC/WOD PINLESS)
 *               PF0 PF1 PF2 PF3   32/48/64-pin
 *      ALT3     PG4 PG5 PG6 PG7   64-pin only
 *    A pinless position keeps the route usable: the missing branch is
 *    compiled out (port_exists + if constexpr), a config that enables
 *    an output with no pin is refused;
 *  - the TCD has TWO interrupt vectors, TCD0_OVF and TCD0_TRIG, and
 *    four event generators (CMPBCLR, CMPASET, CMPBSET, PROGEV) plus two
 *    event users (input A and input B) - all in evsys.hpp;
 *  - STATUS.PWMACTA/B are toggle DETECTORS: hardware sets them on every
 *    edge of WOA/WOB and software clears them by writing a one. They
 *    are the only way to ask "is this output still moving?" without a
 *    pin.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <type_traits>

#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "util/clock.hpp"

namespace brio {

// ---- routes and pins (17.5.x, 25.2.2) ---------------------------------------

/// TCD pin routing (PORTMUX.TCDROUTEA). The values ARE the device
/// header's own group values. There is no pinless code here: unlike the
/// serial peripherals, the TCD's route field has no NONE - an instance
/// with no output enabled in FAULTCTRL is the pinless configuration.
enum class TcdRoute : uint8_t {
    def = 0,
    alt1 = 1,
    alt2 = 2,
    alt3 = 3,
};
static_assert(static_cast<uint8_t>(TcdRoute::def) == PORTMUX_TCD0_DEFAULT_gv &&
              static_cast<uint8_t>(TcdRoute::alt2) == PORTMUX_TCD0_ALT2_gv,
              "the route codes must be the device header's group values");

/// The pin count of this package, read off the device header's own
/// tiers (PORTG is 64-pin, PORTB/PORTE appear together at 48, TWI1
/// marks the 32-pin step). Every step matters to a TCD route table.
#if defined(PORTG)
inline constexpr uint8_t tcd_package_pins = 64;
#elif defined(PORTE)
inline constexpr uint8_t tcd_package_pins = 48;
#elif defined(TWI1)
inline constexpr uint8_t tcd_package_pins = 32;
#else
inline constexpr uint8_t tcd_package_pins = 28;
#endif

/// The four waveform outputs (25.2.2), in pin order. WOA and WOB are
/// generated by the two compare units; WOC and WOD are copies of one of
/// them, selected by CTRLC.CMPCSEL/CMPDSEL.
enum class TcdOutput : uint8_t { woa = 0, wob = 1, woc = 2, wod = 3 };

/// Where one output of one route sits, and whether THIS package bonds
/// it out. `bonded == false` means the silicon has the function but no
/// pin for it here (48-pin ALT1 WOC/WOD, 28-pin ALT2 WOC/WOD).
struct TcdPin {
    char port = '?';
    uint8_t pin = 0;
    bool bonded = false;
    constexpr explicit operator bool() const { return bonded; }
};

/// The port of a route, straight from the device headers.
constexpr char tcd_port_letter(TcdRoute r) {
    switch (r) {
        case TcdRoute::def: return 'A';
        case TcdRoute::alt1: return 'B';
        case TcdRoute::alt2: return 'F';
        case TcdRoute::alt3: return 'G';
    }
    return '?';
}

/// The pin number WOA sits on; the other three follow it.
constexpr uint8_t tcd_first_pin(TcdRoute r) {
    return r == TcdRoute::alt2 ? 0 : 4;
}

/// Whether this package offers a route at all (the device header's
/// route enum, package by package).
constexpr bool tcd_route_exists(TcdRoute r) {
    switch (r) {
        case TcdRoute::def: return true;                       // PA4-PA7 everywhere
        case TcdRoute::alt1: return tcd_package_pins >= 48;     // PORTB
        case TcdRoute::alt2: return true;                       // PORTF, every package
        case TcdRoute::alt3: return tcd_package_pins >= 64;     // PORTG
    }
    return false;
}

/// The position of one output of one route on THIS package, with the
/// pin-level bonding facts the headers' comments carry: 48-pin ALT1
/// stops at PB5 and 28-pin ALT2 stops at PF1, in both families.
constexpr TcdPin tcd_pin(TcdRoute r, TcdOutput o) {
    if (!tcd_route_exists(r)) return {};
    const char port = tcd_port_letter(r);
    if (port == '?' || !port_exists(port)) return {};
    const uint8_t pin = static_cast<uint8_t>(tcd_first_pin(r) + static_cast<uint8_t>(o));
    const bool high_half = static_cast<uint8_t>(o) >= 2;        // WOC / WOD
    if (high_half && r == TcdRoute::alt1 && tcd_package_pins < 64) return {port, pin, false};
    if (high_half && r == TcdRoute::alt2 && tcd_package_pins < 32) return {port, pin, false};
    return {port, pin, true};
}

// ---- the knobs (25.5.1 - 25.5.18) -------------------------------------------

/// CTRLA.CLKSEL: what CLK_TCD is. The values ARE the header's codes.
enum class TcdClock : uint8_t {
    oschf = TCD_CLKSEL_OSCHF_gc,     ///< the internal high-frequency oscillator
    pll = TCD_CLKSEL_PLL_gc,         ///< the PLL (its only consumer on this silicon)
    extclk = TCD_CLKSEL_EXTCLK_gc,   ///< the external clock / crystal oscillator
    clkper = TCD_CLKSEL_CLKPER_gc,   ///< the main clock after its prescaler
};

/// CTRLA.SYNCPRES: divides CLK_TCD into the SYNCHRONIZER clock, which
/// the delay block and the input processing logic run on.
enum class TcdSyncPrescaler : uint8_t {
    div1 = TCD_SYNCPRES_DIV1_gc,
    div2 = TCD_SYNCPRES_DIV2_gc,
    div4 = TCD_SYNCPRES_DIV4_gc,
    div8 = TCD_SYNCPRES_DIV8_gc,
};

/// CTRLA.CNTPRES: divides the synchronizer clock into the COUNTER
/// clock. Errata 2.14.1: anything but DIV1 loses asynchronous input
/// events on the affected revisions - divide with SYNCPRES instead.
enum class TcdCountPrescaler : uint8_t {
    div1 = TCD_CNTPRES_DIV1_gc,
    div4 = TCD_CNTPRES_DIV4_gc,
    div32 = TCD_CNTPRES_DIV32_gc,
};

constexpr uint8_t tcd_sync_divisor(TcdSyncPrescaler p) {
    switch (p) {
        case TcdSyncPrescaler::div1: return 1;
        case TcdSyncPrescaler::div2: return 2;
        case TcdSyncPrescaler::div4: return 4;
        case TcdSyncPrescaler::div8: return 8;
    }
    return 1;
}
constexpr uint8_t tcd_count_divisor(TcdCountPrescaler p) {
    switch (p) {
        case TcdCountPrescaler::div1: return 1;
        case TcdCountPrescaler::div4: return 4;
        case TcdCountPrescaler::div32: return 32;
    }
    return 1;
}

/// CLK_TCD_SYNC and CLK_TCD_CNT for a source rate (25.2.1).
constexpr uint32_t tcd_sync_hz(uint32_t src_hz, TcdSyncPrescaler s) {
    return src_hz / tcd_sync_divisor(s);
}
constexpr uint32_t tcd_counter_hz(uint32_t src_hz, TcdSyncPrescaler s, TcdCountPrescaler c) {
    return src_hz / (static_cast<uint32_t>(tcd_sync_divisor(s)) * tcd_count_divisor(c));
}

/// CTRLB.WGMODE: how the counter walks a TCD cycle (25.3.3.2).
enum class TcdWaveform : uint8_t {
    one_ramp = TCD_WGMODE_ONERAMP_gc,     ///< 0..CMPBCLR, one reset per cycle
    two_ramp = TCD_WGMODE_TWORAMP_gc,     ///< 0..CMPACLR, 0..CMPBCLR
    four_ramp = TCD_WGMODE_FOURRAMP_gc,   ///< 0..each of the four compares in turn
    dual_slope = TCD_WGMODE_DS_gc,        ///< CMPBCLR down to 0 and up again
};

/// INPUTCTRLA/B.INPUTMODE: what an input event does (25.3.3.4.5). The
/// values ARE the header's codes.
enum class TcdInputMode : uint8_t {
    none = TCD_INPUTMODE_NONE_gc,                  ///< 0: no action (captures still work)
    jump_wait = TCD_INPUTMODE_JMPWAIT_gc,          ///< 1: stop, jump to the opposite cycle, wait
    exec_wait = TCD_INPUTMODE_EXECWAIT_gc,         ///< 2: stop, execute the opposite cycle, wait
    exec_fault = TCD_INPUTMODE_EXECFAULT_gc,       ///< 3: execute the opposite cycle while faulted
    freq = TCD_INPUTMODE_FREQ_gc,                  ///< 4: stop all outputs, keep the frequency
    exec_dead_time = TCD_INPUTMODE_EXECDT_gc,      ///< 5: dead-times only while faulted
    wait = TCD_INPUTMODE_WAIT_gc,                  ///< 6: stop all, jump to the next cycle, wait
    wait_sw = TCD_INPUTMODE_WAITSW_gc,             ///< 7: stop all, wait for a RESTART command
    edge_trig = TCD_INPUTMODE_EDGETRIG_gc,         ///< 8: on the edge, end the on-time and jump
    edge_trig_freq = TCD_INPUTMODE_EDGETRIGFREQ_gc,///< 9: on the edge, block the rest of the on-time
    level_trig_freq = TCD_INPUTMODE_LVLTRIGFREQ_gc,///< 10: blocked while the level lasts
};

/// EVCTRLx.CFG: how the input event is qualified. FILTER and ASYNC are
/// mutually exclusive by construction - the register field says so.
enum class TcdEventConfig : uint8_t {
    neither = TCD_CFG_NEITHER_gc,   ///< plain synchronous detection
    filter = TCD_CFG_FILTER_gc,     ///< four equal samples of CLK_TCD_CNT required
    async = TCD_CFG_ASYNC_gc,       ///< the event overrides the output instantly
};

/// EVCTRLx.ACTION: a fault only, or a fault AND a counter capture.
enum class TcdEventAction : uint8_t {
    fault = TCD_ACTION_FAULT_gc,
    capture = TCD_ACTION_CAPTURE_gc,
};

/// DLYCTRL.DLYSEL: the ONE function the delay block serves. Input
/// blanking and the programmable output event share the trigger, the
/// prescaler and the value (25.3.4.1), so asking for both is not
/// expressible - which is the refusal.
enum class TcdDelaySelect : uint8_t {
    off = TCD_DLYSEL_OFF_gc,
    input_blanking = TCD_DLYSEL_INBLANK_gc,
    output_event = TCD_DLYSEL_EVENT_gc,
};

/// DLYCTRL.DLYTRIG: which compare match starts the delay counter.
enum class TcdDelayTrigger : uint8_t {
    cmpaset = TCD_DLYTRIG_CMPASET_gc,
    cmpaclr = TCD_DLYTRIG_CMPACLR_gc,
    cmpbset = TCD_DLYTRIG_CMPBSET_gc,
    cmpbclr = TCD_DLYTRIG_CMPBCLR_gc,   ///< end of cycle
};

/// DLYCTRL.DLYPRESC: the delay clock is CLK_TCD_SYNC divided by this.
enum class TcdDelayPrescaler : uint8_t {
    div1 = TCD_DLYPRESC_DIV1_gc,
    div2 = TCD_DLYPRESC_DIV2_gc,
    div4 = TCD_DLYPRESC_DIV4_gc,
    div8 = TCD_DLYPRESC_DIV8_gc,
};

constexpr uint8_t tcd_delay_divisor(TcdDelayPrescaler p) {
    switch (p) {
        case TcdDelayPrescaler::div1: return 1;
        case TcdDelayPrescaler::div2: return 2;
        case TcdDelayPrescaler::div4: return 4;
        case TcdDelayPrescaler::div8: return 8;
    }
    return 1;
}

/// DITCTRL.DITHERSEL: which part of the cycle absorbs the extra tick
/// when the dither accumulator overflows (25.3.3.5, table 25-7).
enum class TcdDitherSelect : uint8_t {
    on_time_b = TCD_DITHERSEL_ONTIMEB_gc,
    on_time_ab = TCD_DITHERSEL_ONTIMEAB_gc,
    dead_time_b = TCD_DITHERSEL_DEADTIMEB_gc,
    dead_time_ab = TCD_DITHERSEL_DEADTIMEAB_gc,
};

/// CTRLC.CMPCSEL / CMPDSEL: which generated waveform WOC and WOD copy.
enum class TcdWaveformSelect : uint8_t { pwm_a = 0, pwm_b = 1 };

/// The counter, the compares and the captures are 12 bits wide.
inline constexpr uint16_t tcd_compare_max = 0x0FFF;

/// Extra TCD clock cycles added to the CYCLE when the dither
/// accumulator overflows (table 25-7). Where this is 0 the compensation
/// is taken out of the FOLLOWING output state instead, so the cycle
/// length does not move - only the duty does.
constexpr uint8_t tcd_dither_cycle_cost(TcdWaveform w, TcdDitherSelect d) {
    const bool on_time = d == TcdDitherSelect::on_time_b || d == TcdDitherSelect::on_time_ab;
    const bool both = d == TcdDitherSelect::on_time_ab || d == TcdDitherSelect::dead_time_ab;
    switch (w) {
        case TcdWaveform::one_ramp: return on_time ? 1 : 0;
        case TcdWaveform::two_ramp: return on_time ? (both ? 2 : 1) : 0;
        case TcdWaveform::four_ramp: return both ? 2 : 1;
        case TcdWaveform::dual_slope: return 0;                 // not supported at all
    }
    return 0;
}

/// The length of one TCD cycle in counter ticks, per the chapter's own
/// formulas (25.3.3.2): one ramp counts to CMPBCLR, two ramp adds the
/// CMPACLR ramp, four ramp walks all four, and dual slope goes down and
/// up again.
///
/// The dual-slope line is the one place where the BENCH corrects the
/// chapter. 25.3.3.2.4 prints T = (2 x CMPBCLR + 1) / f; the silicon
/// measures 2 x (CMPBCLR + 1) counter ticks exactly, at more than one
/// geometry (test_avr_tcd d) - the ramp down and the ramp up are each
/// CMPBCLR + 1 ticks long, like every other mode's ramp. The measured
/// number is what this function returns.
constexpr uint32_t tcd_cycle_ticks(TcdWaveform w, uint16_t a_set, uint16_t a_clr,
                                   uint16_t b_set, uint16_t b_clr) {
    switch (w) {
        case TcdWaveform::one_ramp: return static_cast<uint32_t>(b_clr) + 1u;
        case TcdWaveform::two_ramp:
            return static_cast<uint32_t>(a_clr) + 1u + static_cast<uint32_t>(b_clr) + 1u;
        case TcdWaveform::four_ramp:
            return static_cast<uint32_t>(a_set) + 1u + static_cast<uint32_t>(a_clr) + 1u +
                   static_cast<uint32_t>(b_set) + 1u + static_cast<uint32_t>(b_clr) + 1u;
        case TcdWaveform::dual_slope: return 2u * (static_cast<uint32_t>(b_clr) + 1u);
    }
    return 0;
}

/// Table 25-5, with errata 2.14.3 folded in: the input modes that need
/// a counter which resets more than once per cycle are refused in one
/// ramp, dual slope takes almost nothing, and mode 7 is refused in dual
/// slope on EVERY silicon revision of both families.
constexpr bool tcd_input_mode_valid(TcdWaveform w, TcdInputMode m) {
    const uint8_t code = static_cast<uint8_t>(m);
    if (code > 10) return false;
    switch (w) {
        case TcdWaveform::one_ramp:
            // 2, 3, 5 and 6 execute "the opposite compare cycle": there
            // is only one ramp to execute.
            return code != 2 && code != 3 && code != 5 && code != 6;
        case TcdWaveform::two_ramp:
        case TcdWaveform::four_ramp:
            return true;
        case TcdWaveform::dual_slope:
            // The table leaves 0, 4 and 7; errata 2.14.3 / 2.13.3 takes
            // 7 away on every revision.
            return code == 0 || code == 4;
    }
    return false;
}

/// One event input's whole configuration (EVCTRLx + INPUTCTRLx).
struct TcdEventInput {
    bool enable = false;                                  ///< TRIGEI
    TcdEventAction action = TcdEventAction::fault;        ///< also capture the counter
    bool rising = true;                                   ///< EDGE: rising edge / high level
    TcdEventConfig config = TcdEventConfig::neither;
    TcdInputMode mode = TcdInputMode::none;
};

/// Everything the instance is configured with. The compares are the
/// values the registers take (12 bits); the chapter's formulas above
/// turn them into a cycle length.
struct TcdConfig {
    TcdRoute route = TcdRoute::def;

    // CTRLA
    TcdClock clock = TcdClock::oschf;
    TcdSyncPrescaler sync_prescaler = TcdSyncPrescaler::div1;
    TcdCountPrescaler count_prescaler = TcdCountPrescaler::div1;

    // CTRLB
    TcdWaveform waveform = TcdWaveform::one_ramp;

    // the four double-buffered compares
    uint16_t compare_a_set = 0;
    uint16_t compare_a_clear = 0;    ///< ignored in dual slope (25.3.3.2.4)
    uint16_t compare_b_set = 0;
    uint16_t compare_b_clear = 0;

    // CTRLC
    bool compare_override = false;   ///< CMPOVR: CTRLD drives the outputs per state
    bool auto_update = false;        ///< AUPDATE: a write to CMPBCLRH requests an end-of-cycle sync
    bool fifty_percent = false;      ///< FIFTY: a write to one SET (or CLR) lands in both
    TcdWaveformSelect wo_c = TcdWaveformSelect::pwm_a;
    TcdWaveformSelect wo_d = TcdWaveformSelect::pwm_a;

    // CTRLD (only in force with compare_override)
    uint8_t compare_a_value = 0;     ///< CMPAVAL[3:0], per table 25-12 / 25-13
    uint8_t compare_b_value = 0;     ///< CMPBVAL[3:0]

    // the two inputs
    TcdEventInput input_a{};
    TcdEventInput input_b{};

    // FAULTCTRL (CCP): which outputs exist, and the level each takes
    // while a fault is active
    bool enable_woa = false;
    bool enable_wob = false;
    bool enable_woc = false;
    bool enable_wod = false;
    bool fault_woa = false;
    bool fault_wob = false;
    bool fault_woc = false;
    bool fault_wod = false;

    // the delay block: input blanking OR the programmable output event
    TcdDelaySelect delay = TcdDelaySelect::off;
    TcdDelayTrigger delay_trigger = TcdDelayTrigger::cmpaset;
    TcdDelayPrescaler delay_prescaler = TcdDelayPrescaler::div1;
    uint8_t delay_value = 0;

    // the dither accumulator
    TcdDitherSelect dither_select = TcdDitherSelect::on_time_b;
    uint8_t dither = 0;              ///< DITHER[3:0]: 0 disables it

    bool debug_run = false;          ///< keep running while the CPU is halted
    bool fault_on_debug = false;     ///< FAULTDET: a break raises both triggers
    bool enable = true;              ///< leave the instance enabled when init() returns
};

/// Is this configuration legal on this package and this silicon?
///  - the route must exist here;
///  - every ENABLED output must have a bonded pin on that route;
///  - both input modes must be valid for the waveform mode (table 25-5
///    with errata 2.14.3 folded in);
///  - input mode 7 needs CMPASET != 0 (errata 2.14.3, every revision);
///  - dithering is not supported in dual slope (25.3.3.5);
///  - no compare may exceed 12 bits.
/// Deliberately NOT refused: errata 2.14.1 (async events with a counter
/// prescaler) and 2.14.2 (CMPAEN gating every output on an alternate
/// route) are fixed in DB rev. B0, so refusing them would make the
/// driver useless on the silicon that works. They are documented, and
/// the suite measures them.
constexpr bool tcd_config_valid(const TcdConfig& c) {
    if (!tcd_route_exists(c.route)) return false;
    if (c.compare_a_set > tcd_compare_max || c.compare_a_clear > tcd_compare_max ||
        c.compare_b_set > tcd_compare_max || c.compare_b_clear > tcd_compare_max) return false;
    if (c.enable_woa && !tcd_pin(c.route, TcdOutput::woa).bonded) return false;
    if (c.enable_wob && !tcd_pin(c.route, TcdOutput::wob).bonded) return false;
    if (c.enable_woc && !tcd_pin(c.route, TcdOutput::woc).bonded) return false;
    if (c.enable_wod && !tcd_pin(c.route, TcdOutput::wod).bonded) return false;
    if (!tcd_input_mode_valid(c.waveform, c.input_a.mode)) return false;
    if (!tcd_input_mode_valid(c.waveform, c.input_b.mode)) return false;
    if ((c.input_a.mode == TcdInputMode::wait_sw || c.input_b.mode == TcdInputMode::wait_sw) &&
        c.compare_a_set == 0) return false;
    if (c.dither != 0 && c.waveform == TcdWaveform::dual_slope) return false;
    if (c.dither > 0x0F) return false;
    if (c.compare_a_value > 0x0F || c.compare_b_value > 0x0F) return false;
    return true;
}

/// The two trigger flags, as take_triggers() returns them.
struct TcdTriggers {
    bool a;
    bool b;
};

/// The two PWM activity detectors, as take_pwm_activity() returns them.
struct TcdActivity {
    bool a;
    bool b;
};

// ---- the resource -----------------------------------------------------------

/// How many spins the bounded ENRDY/CMDRDY waits take before giving up.
/// A closed window costs a handful of CLK_TCD cycles; a source that is
/// not running never opens it, and the verb returns false instead of
/// hanging the program.
inline constexpr uint16_t tcd_sync_spins = 0xFFFFu;

template <uint8_t n>
class Tcd {
    static_assert(n == 0, "this family provides one TCD instance, TCD0");

public:
    Tcd() = delete;

    static constexpr uint8_t index = n;

    /// The event vocabulary of this instance (evsys.hpp).
    using CmpBClrEvent = EvTcdCmpBClr;   ///< generator: counter == CMPBCLR (end of cycle)
    using CmpASetEvent = EvTcdCmpASet;   ///< generator: counter == CMPASET
    using CmpBSetEvent = EvTcdCmpBSet;   ///< generator: counter == CMPBSET
    using ProgEvent = EvTcdProgEv;       ///< generator: the programmable output event
    using InputA = EvTcdInputA;          ///< user: event input A
    using InputB = EvTcdInputB;          ///< user: event input B

    static constexpr TCD_t& regs() { return TCD0; }

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: every register value folded, and everything
    /// tcd_config_valid() refuses turned into a diagnostic here.
    template <TcdConfig cfg>
    static bool init() {
        static_assert(tcd_route_exists(cfg.route),
                      "this package does not offer this TCD route");
        static_assert(!cfg.enable_woa || tcd_pin(cfg.route, TcdOutput::woa).bonded,
                      "this package does not bond WOA on this route");
        static_assert(!cfg.enable_wob || tcd_pin(cfg.route, TcdOutput::wob).bonded,
                      "this package does not bond WOB on this route");
        static_assert(!cfg.enable_woc || tcd_pin(cfg.route, TcdOutput::woc).bonded,
                      "this package does not bond WOC on this route (48-pin ALT1 and "
                      "28-pin ALT2 stop at the second output)");
        static_assert(!cfg.enable_wod || tcd_pin(cfg.route, TcdOutput::wod).bonded,
                      "this package does not bond WOD on this route (48-pin ALT1 and "
                      "28-pin ALT2 stop at the second output)");
        static_assert(tcd_input_mode_valid(cfg.waveform, cfg.input_a.mode) &&
                      tcd_input_mode_valid(cfg.waveform, cfg.input_b.mode),
                      "this input mode is not valid in this waveform generation mode "
                      "(table 25-5: modes 2/3/5/6 need two- or four-ramp; dual slope "
                      "takes only 0 and 4, mode 7 being refused there by errata 2.14.3)");
        static_assert((cfg.input_a.mode != TcdInputMode::wait_sw &&
                       cfg.input_b.mode != TcdInputMode::wait_sw) || cfg.compare_a_set != 0,
                      "input mode 7 (WAITSW) does not work with CMPASET = 0 - errata "
                      "2.14.3 / 2.13.3, every silicon revision of both families");
        static_assert(cfg.dither == 0 || cfg.waveform != TcdWaveform::dual_slope,
                      "dithering is not supported in Dual Slope mode (25.3.3.5)");
        static_assert(tcd_config_valid(cfg), "invalid TCD configuration");
        return init(cfg);
    }

    /// Run-time form. Refuses (writing nothing) what tcd_config_valid()
    /// rejects; then disables under ENRDY, writes every static register
    /// with the peripheral down, loads the double buffers, claims the
    /// enabled output pins and enables under ENRDY again. Interrupts
    /// stay off: the task or the app enables them.
    static bool init(const TcdConfig& cfg) {
        if (!tcd_config_valid(cfg)) return false;
        if (!disable()) return false;

        route(cfg.route);
        regs().CTRLA = static_cast<uint8_t>(               // ENABLE = 0: the protected bits land
            static_cast<uint8_t>(cfg.clock) |
            static_cast<uint8_t>(cfg.sync_prescaler) |
            static_cast<uint8_t>(cfg.count_prescaler));
        regs().CTRLB = static_cast<uint8_t>(cfg.waveform);
        regs().CTRLC = static_cast<uint8_t>(
            (cfg.compare_override ? TCD_CMPOVR_bm : 0) |
            (cfg.auto_update ? TCD_AUPDATE_bm : 0) |
            (cfg.fifty_percent ? TCD_FIFTY_bm : 0) |
            (cfg.wo_c == TcdWaveformSelect::pwm_b ? TCD_CMPCSEL_bm : 0) |
            (cfg.wo_d == TcdWaveformSelect::pwm_b ? TCD_CMPDSEL_bm : 0));
        regs().CTRLD = static_cast<uint8_t>(
            (cfg.compare_a_value & 0x0Fu) |
            static_cast<uint8_t>((cfg.compare_b_value & 0x0Fu) << 4));
        regs().EVCTRLA = event_bits(cfg.input_a);
        regs().EVCTRLB = event_bits(cfg.input_b);
        regs().INPUTCTRLA = static_cast<uint8_t>(cfg.input_a.mode);
        regs().INPUTCTRLB = static_cast<uint8_t>(cfg.input_b.mode);
        regs().INTCTRL = 0;
        regs().DBGCTRL = static_cast<uint8_t>(
            (cfg.debug_run ? TCD_DBGRUN_bm : 0) | (cfg.fault_on_debug ? TCD_FAULTDET_bm : 0));
        regs().DLYCTRL = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.delay) |
            static_cast<uint8_t>(cfg.delay_trigger) |
            static_cast<uint8_t>(cfg.delay_prescaler));
        regs().DLYVAL = cfg.delay_value;
        regs().DITCTRL = static_cast<uint8_t>(cfg.dither_select);
        regs().DITVAL = static_cast<uint8_t>(cfg.dither & 0x0Fu);

        // The compares, low byte first (the high byte of CMPBCLR is the
        // AUPDATE trigger, so it goes last of all).
        write12(regs().CMPASETL, regs().CMPASETH, cfg.compare_a_set);
        write12(regs().CMPACLRL, regs().CMPACLRH, cfg.compare_a_clear);
        write12(regs().CMPBSETL, regs().CMPBSETH, cfg.compare_b_set);
        write12(regs().CMPBCLRL, regs().CMPBCLRH, cfg.compare_b_clear);

        // FAULTCTRL is CCP-protected (IOREG key) and claims the pins.
        fault_control(cfg.enable_woa, cfg.enable_wob, cfg.enable_woc, cfg.enable_wod,
                      cfg.fault_woa, cfg.fault_wob, cfg.fault_woc, cfg.fault_wod);
        claim_pins(cfg);

        regs().INTFLAGS = static_cast<uint8_t>(TCD_OVF_bm | TCD_TRIGA_bm | TCD_TRIGB_bm);
        regs().STATUS = static_cast<uint8_t>(TCD_PWMACTA_bm | TCD_PWMACTB_bm);
        route_ = cfg.route;
        outputs_ = static_cast<uint8_t>((cfg.enable_woa ? 1u : 0u) | (cfg.enable_wob ? 2u : 0u) |
                                        (cfg.enable_woc ? 4u : 0u) | (cfg.enable_wod ? 8u : 0u));
        return cfg.enable ? enable() : true;
    }

    /// Hand everything back: the peripheral disabled, every output
    /// released in FAULTCTRL, the claimed pins back to PORT inputs, the
    /// route back to DEFAULT. The "Entry routes, Exit disconnects"
    /// idiom leaves no trace.
    static bool release() {
        const bool down = disable();
        fault_control(false, false, false, false, false, false, false, false);
        release_pins();
        regs().INTCTRL = 0;
        regs().INTFLAGS = static_cast<uint8_t>(TCD_OVF_bm | TCD_TRIGA_bm | TCD_TRIGB_bm);
        PORTMUX.TCDROUTEA = static_cast<uint8_t>(PORTMUX.TCDROUTEA & ~PORTMUX_TCD0_gm);
        outputs_ = 0;
        route_ = TcdRoute::def;
        return down;
    }

    // ---- discipline 1: ENABLE under ENRDY ---------------------------------

    /// STATUS.ENRDY: the ENABLE bit is synchronized and writable again.
    static bool enable_ready() { return (regs().STATUS & TCD_ENRDY_bm) != 0; }
    static bool wait_enable_ready(uint16_t spins = tcd_sync_spins) {
        while (spins--) {
            if (enable_ready()) return true;
        }
        return false;
    }
    /// Enable under the discipline; false when the window never opened
    /// (a CLK_TCD source that is not running is the usual cause).
    static bool enable(uint16_t spins = tcd_sync_spins) {
        if (!wait_enable_ready(spins)) return false;
        regs().CTRLA |= TCD_ENABLE_bm;
        return true;
    }
    /// Disable immediately (the counter stops as soon as the write is
    /// synchronized). For "at the end of the cycle" use disable_at_end().
    static bool disable(uint16_t spins = tcd_sync_spins) {
        if (!wait_enable_ready(spins)) return false;
        regs().CTRLA &= static_cast<uint8_t>(~TCD_ENABLE_bm);
        return wait_enable_ready(spins);
    }
    static bool enabled() { return (regs().CTRLA & TCD_ENABLE_bm) != 0; }

    // ---- discipline 2: the CTRLE strobes under CMDRDY ----------------------

    /// STATUS.CMDRDY: the previous command reached the TCD domain.
    static bool command_ready() { return (regs().STATUS & TCD_CMDRDY_bm) != 0; }
    static bool wait_command_ready(uint16_t spins = tcd_sync_spins) {
        while (spins--) {
            if (command_ready()) return true;
        }
        return false;
    }

    /// Load the double-buffered registers now (CTRLE.SYNC).
    static bool sync(uint16_t spins = tcd_sync_spins) { return strobe(TCD_SYNC_bm, spins); }
    /// Load them at the end of the next TCD cycle (CTRLE.SYNCEOC).
    static bool sync_at_end(uint16_t spins = tcd_sync_spins) { return strobe(TCD_SYNCEOC_bm, spins); }
    /// Restart the counter (CTRLE.RESTART) - the escape from input mode 7.
    static bool restart(uint16_t spins = tcd_sync_spins) { return strobe(TCD_RESTART_bm, spins); }
    /// Capture the counter into CAPTUREA / CAPTUREB by software.
    static bool software_capture_a(uint16_t spins = tcd_sync_spins) {
        return strobe(TCD_SCAPTUREA_bm, spins) && wait_command_ready(spins);
    }
    static bool software_capture_b(uint16_t spins = tcd_sync_spins) {
        return strobe(TCD_SCAPTUREB_bm, spins) && wait_command_ready(spins);
    }
    /// Disable at the end of the TCD cycle (CTRLE.DISEOC). ENRDY stays
    /// low until the TCD has actually gone down (25.5.5).
    static bool disable_at_end(uint16_t spins = tcd_sync_spins) { return strobe(TCD_DISEOC_bm, spins); }

    // ---- the double-buffered compares -------------------------------------
    //
    // Written low byte first. With CTRLC.AUPDATE set, the write of
    // CMPBCLR's HIGH byte is itself the end-of-cycle synchronization
    // request (25.5.3), which is why compare_b_clear() is the verb an
    // auto-updating application calls LAST.

    static bool compare_a_set(uint16_t v, uint16_t spins = tcd_sync_spins) {
        if (!wait_command_ready(spins)) return false;
        write12(regs().CMPASETL, regs().CMPASETH, v);
        return true;
    }
    static bool compare_a_clear(uint16_t v, uint16_t spins = tcd_sync_spins) {
        if (!wait_command_ready(spins)) return false;
        write12(regs().CMPACLRL, regs().CMPACLRH, v);
        return true;
    }
    static bool compare_b_set(uint16_t v, uint16_t spins = tcd_sync_spins) {
        if (!wait_command_ready(spins)) return false;
        write12(regs().CMPBSETL, regs().CMPBSETH, v);
        return true;
    }
    static bool compare_b_clear(uint16_t v, uint16_t spins = tcd_sync_spins) {
        if (!wait_command_ready(spins)) return false;
        write12(regs().CMPBCLRL, regs().CMPBCLRH, v);
        return true;
    }
    static uint16_t compare_a_set() { return read12(regs().CMPASETL, regs().CMPASETH); }
    static uint16_t compare_a_clear() { return read12(regs().CMPACLRL, regs().CMPACLRH); }
    static uint16_t compare_b_set() { return read12(regs().CMPBSETL, regs().CMPBSETH); }
    static uint16_t compare_b_clear() { return read12(regs().CMPBCLRL, regs().CMPBCLRH); }

    /// DITVAL is double-buffered too (25.5.17): the new fraction is
    /// adopted at the next update condition.
    static bool dither(uint8_t v, uint16_t spins = tcd_sync_spins) {
        if (v > 0x0F) return false;
        if (!wait_command_ready(spins)) return false;
        regs().DITVAL = v;
        return true;
    }
    static uint8_t dither() { return static_cast<uint8_t>(regs().DITVAL & TCD_DITHER_gm); }

    // ---- the captures (25.5.19, 25.5.20) ----------------------------------
    //
    // Read LOW byte then HIGH byte: the read of the high byte is what
    // releases the capture buffer for the next capture. The pair is
    // performed here, so no caller can half-read one.

    static uint16_t capture_a() { return read12(regs().CAPTUREAL, regs().CAPTUREAH); }
    static uint16_t capture_b() { return read12(regs().CAPTUREBL, regs().CAPTUREBH); }

    // ---- discipline 3: the static registers, only while disabled -----------
    //
    // Each of these returns false and writes NOTHING while the TCD is
    // enabled (25.3.3.1: "Static registers cannot be updated while the
    // TCD is enabled").

    static bool waveform(TcdWaveform w) {
        if (enabled()) return false;
        regs().CTRLB = static_cast<uint8_t>(w);
        return true;
    }
    static TcdWaveform waveform() {
        return static_cast<TcdWaveform>(regs().CTRLB & TCD_WGMODE_gm);
    }
    static bool clock(TcdClock c, TcdSyncPrescaler s, TcdCountPrescaler p) {
        if (enabled()) return false;
        regs().CTRLA = static_cast<uint8_t>(static_cast<uint8_t>(c) | static_cast<uint8_t>(s) |
                                            static_cast<uint8_t>(p));
        return true;
    }
    static TcdClock clock() { return static_cast<TcdClock>(regs().CTRLA & TCD_CLKSEL_gm); }
    static bool input_mode_a(TcdInputMode m) {
        if (enabled()) return false;
        regs().INPUTCTRLA = static_cast<uint8_t>(m);
        return true;
    }
    static bool input_mode_b(TcdInputMode m) {
        if (enabled()) return false;
        regs().INPUTCTRLB = static_cast<uint8_t>(m);
        return true;
    }
    static bool event_input_a(const TcdEventInput& e) {
        if (enabled()) return false;
        regs().EVCTRLA = event_bits(e);
        regs().INPUTCTRLA = static_cast<uint8_t>(e.mode);
        return true;
    }
    static bool event_input_b(const TcdEventInput& e) {
        if (enabled()) return false;
        regs().EVCTRLB = event_bits(e);
        regs().INPUTCTRLB = static_cast<uint8_t>(e.mode);
        return true;
    }
    /// CTRLC's output plumbing: the override switch, the auto-update
    /// path, the fifty-percent mirror, and which waveform WOC/WOD copy.
    static bool output_control(bool override_values, bool auto_update, bool fifty,
                               TcdWaveformSelect c, TcdWaveformSelect d) {
        if (enabled()) return false;
        regs().CTRLC = static_cast<uint8_t>(
            (override_values ? TCD_CMPOVR_bm : 0) | (auto_update ? TCD_AUPDATE_bm : 0) |
            (fifty ? TCD_FIFTY_bm : 0) |
            (c == TcdWaveformSelect::pwm_b ? TCD_CMPCSEL_bm : 0) |
            (d == TcdWaveformSelect::pwm_b ? TCD_CMPDSEL_bm : 0));
        return true;
    }
    /// CTRLD: the level each waveform takes in each of the four states
    /// (tables 25-12 and 25-13), in force only with the override on.
    static bool output_values(uint8_t a_values, uint8_t b_values) {
        if (enabled()) return false;
        if (a_values > 0x0F || b_values > 0x0F) return false;
        regs().CTRLD = static_cast<uint8_t>(a_values | static_cast<uint8_t>(b_values << 4));
        return true;
    }
    /// FAULTCTRL, under CCP: which outputs are enabled on their pins and
    /// what level each takes while a fault is active. Enabled outputs
    /// are ALSO driven as PORT outputs by claim_pins()/init().
    static void fault_control(bool en_a, bool en_b, bool en_c, bool en_d,
                              bool val_a, bool val_b, bool val_c, bool val_d) {
        const uint8_t v = static_cast<uint8_t>(
            (en_a ? TCD_CMPAEN_bm : 0) | (en_b ? TCD_CMPBEN_bm : 0) |
            (en_c ? TCD_CMPCEN_bm : 0) | (en_d ? TCD_CMPDEN_bm : 0) |
            (val_a ? TCD_CMPA_bm : 0) | (val_b ? TCD_CMPB_bm : 0) |
            (val_c ? TCD_CMPC_bm : 0) | (val_d ? TCD_CMPD_bm : 0));
        _PROTECTED_WRITE(TCD0.FAULTCTRL, v);
    }
    static uint8_t fault_control() { return regs().FAULTCTRL; }

    /// The delay block (25.3.3.4.1, 25.3.4.1): input blanking OR the
    /// programmable output event - never both, the register field says
    /// so. Static: the TCD must be down.
    static bool delay(TcdDelaySelect sel, TcdDelayTrigger trig, TcdDelayPrescaler pre,
                      uint8_t value) {
        if (enabled()) return false;
        regs().DLYCTRL = static_cast<uint8_t>(static_cast<uint8_t>(sel) |
                                              static_cast<uint8_t>(trig) |
                                              static_cast<uint8_t>(pre));
        regs().DLYVAL = value;
        return true;
    }
    static bool input_blanking_enabled() {
        return (regs().DLYCTRL & TCD_DLYSEL_gm) == static_cast<uint8_t>(TCD_DLYSEL_INBLANK_gc);
    }
    static bool output_event_enabled() {
        return (regs().DLYCTRL & TCD_DLYSEL_gm) == static_cast<uint8_t>(TCD_DLYSEL_EVENT_gc);
    }
    /// The blanking window / event delay in CLK_TCD_SYNC cycles.
    static uint16_t delay_cycles() {
        const TcdDelayPrescaler p =
            static_cast<TcdDelayPrescaler>(regs().DLYCTRL & TCD_DLYPRESC_gm);
        return static_cast<uint16_t>(tcd_delay_divisor(p) * regs().DLYVAL);
    }

    // ---- status, flags, interrupts ----------------------------------------

    /// STATUS.PWMACTA/B: toggle detectors, cleared by writing a one.
    /// Read AND clear in one verb, so "did this waveform move since I
    /// last asked?" is one call. A plain store of the two bits: an RMW
    /// would write back CMDRDY/ENRDY too.
    ///
    /// BENCH: the chapter says "each time the WOx output toggles", and
    /// the pad is NOT what it watches. With an input mode holding the
    /// outputs off (mode 4, mode 10) the pins provably stand still and
    /// PWMACT keeps setting - it follows the WAVEFORM GENERATOR behind
    /// the fault override. The W1C clear itself is sound (with the TCD
    /// disabled the bits stay down). To ask about the PAD, read the pin.
    static TcdActivity take_pwm_activity() {
        const uint8_t s = regs().STATUS;
        regs().STATUS = static_cast<uint8_t>(TCD_PWMACTA_bm | TCD_PWMACTB_bm);
        return {(s & TCD_PWMACTA_bm) != 0, (s & TCD_PWMACTB_bm) != 0};
    }
    static void clear_pwm_activity() {
        regs().STATUS = static_cast<uint8_t>(TCD_PWMACTA_bm | TCD_PWMACTB_bm);
    }

    static bool ovf_flag() { return (regs().INTFLAGS & TCD_OVF_bm) != 0; }
    static bool trig_a_flag() { return (regs().INTFLAGS & TCD_TRIGA_bm) != 0; }
    static bool trig_b_flag() { return (regs().INTFLAGS & TCD_TRIGB_bm) != 0; }
    static void clear_ovf() { regs().INTFLAGS = TCD_OVF_bm; }
    static void clear_trig_a() { regs().INTFLAGS = TCD_TRIGA_bm; }
    static void clear_trig_b() { regs().INTFLAGS = TCD_TRIGB_bm; }
    static void clear_flags() {
        regs().INTFLAGS = static_cast<uint8_t>(TCD_OVF_bm | TCD_TRIGA_bm | TCD_TRIGB_bm);
    }

    static void enable_ovf_interrupt(bool on) { irq(TCD_OVF_bm, on); }
    static void enable_trig_a_interrupt(bool on) { irq(TCD_TRIGA_bm, on); }
    static void enable_trig_b_interrupt(bool on) { irq(TCD_TRIGB_bm, on); }

    /// ISR body for TCD0_OVF_vect: one TCD cycle finished, flag cleared.
    [[gnu::always_inline]] static void ovf() { regs().INTFLAGS = TCD_OVF_bm; }
    /// ISR body for TCD0_TRIG_vect: which input triggered, both cleared.
    /// With EVCTRLx.ACTION = capture, the matching capture_a()/b() is
    /// then ready to be read.
    [[gnu::always_inline]] static TcdTriggers take_triggers() {
        const uint8_t f = regs().INTFLAGS;
        regs().INTFLAGS = static_cast<uint8_t>(f & (TCD_TRIGA_bm | TCD_TRIGB_bm));
        return {(f & TCD_TRIGA_bm) != 0, (f & TCD_TRIGB_bm) != 0};
    }

    // ---- events -------------------------------------------------------------

    /// Input A takes its event from channel ch (TRIGEI stays as the
    /// configuration left it: listening alone arms nothing).
    template <uint8_t ch>
    static void input_a_on(EventChannel<ch> c) { InputA::listen(c); }
    template <uint8_t ch>
    static void input_b_on(EventChannel<ch> c) { InputB::listen(c); }
    static void input_a_off() { InputA::unlisten(); }
    static void input_b_off() { InputB::unlisten(); }

    /// The route in force and the outputs init() claimed (bit 0 = WOA).
    static TcdRoute route() { return route_; }
    static uint8_t claimed_outputs() { return outputs_; }

    /// Drive the pins of the enabled outputs. The branch for a position
    /// this package lacks is compiled OUT (a runtime `if` would
    /// instantiate a Pin on a missing port and kill the whole instance);
    /// tcd_config_valid() has already refused such a configuration.
    static void claim_pins(const TcdConfig& cfg) {
        pin_dir<TcdOutput::woa>(cfg.route, cfg.enable_woa, true);
        pin_dir<TcdOutput::wob>(cfg.route, cfg.enable_wob, true);
        pin_dir<TcdOutput::woc>(cfg.route, cfg.enable_woc, true);
        pin_dir<TcdOutput::wod>(cfg.route, cfg.enable_wod, true);
    }

private:
    static void irq(uint8_t bit, bool on) {
        if (on) regs().INTCTRL |= bit;
        else regs().INTCTRL &= static_cast<uint8_t>(~bit);
    }

    static bool strobe(uint8_t bit, uint16_t spins) {
        if (!wait_command_ready(spins)) return false;
        regs().CTRLE = bit;
        return true;
    }

    static uint8_t event_bits(const TcdEventInput& e) {
        return static_cast<uint8_t>(
            (e.enable ? TCD_TRIGEI_bm : 0) |
            (e.action == TcdEventAction::capture ? TCD_ACTION_bm : 0) |
            (e.rising ? TCD_EDGE_bm : 0) |
            static_cast<uint8_t>(e.config));
    }

    static void route(TcdRoute r) {
        PORTMUX.TCDROUTEA = static_cast<uint8_t>((PORTMUX.TCDROUTEA & ~PORTMUX_TCD0_gm) |
                                                 static_cast<uint8_t>(r));
    }

    static void release_pins() {
        const uint8_t claimed = outputs_;
        pin_dir<TcdOutput::woa>(route_, (claimed & 1u) != 0, false);
        pin_dir<TcdOutput::wob>(route_, (claimed & 2u) != 0, false);
        pin_dir<TcdOutput::woc>(route_, (claimed & 4u) != 0, false);
        pin_dir<TcdOutput::wod>(route_, (claimed & 8u) != 0, false);
    }

    /// Drive (or release) the pin one output takes on `r`, if this
    /// package has it. Every route/output pair whose port is absent here
    /// compiles to nothing.
    template <TcdOutput o>
    static void pin_dir(TcdRoute r, bool active, bool as_output) {
        if (!active) return;
        apply_dir<TcdRoute::def, o>(r, as_output);
        apply_dir<TcdRoute::alt1, o>(r, as_output);
        apply_dir<TcdRoute::alt2, o>(r, as_output);
        apply_dir<TcdRoute::alt3, o>(r, as_output);
    }
    template <TcdRoute r, TcdOutput o>
    static void apply_dir(TcdRoute want, bool as_output) {
        if constexpr (tcd_pin(r, o).bonded) {
            using P = Pin<tcd_pin(r, o).port, tcd_pin(r, o).pin>;
            if (want == r) {
                if (as_output) P::output(); else P::input();
            }
        } else {
            (void)want;
            (void)as_output;
        }
    }

    static void write12(volatile uint8_t& lo, volatile uint8_t& hi, uint16_t v) {
        const uint16_t m = static_cast<uint16_t>(v & tcd_compare_max);
        lo = static_cast<uint8_t>(m);
        hi = static_cast<uint8_t>(m >> 8);
    }
    static uint16_t read12(volatile uint8_t& lo, volatile uint8_t& hi) {
        const uint8_t l = lo;                 // LOW first
        const uint8_t h = hi;                 // the HIGH read releases the buffer
        return static_cast<uint16_t>((static_cast<uint16_t>(h) << 8) | l);
    }

    static inline TcdRoute route_ = TcdRoute::def;
    static inline uint8_t outputs_ = 0;
};

// ---- the task ---------------------------------------------------------------

/// What a TcdPwm is asked for.
struct TcdPwmConfig {
    TcdClock clock = TcdClock::clkper;
    TcdSyncPrescaler sync_prescaler = TcdSyncPrescaler::div1;
    TcdCountPrescaler count_prescaler = TcdCountPrescaler::div1;

    /// The rate of the SELECTED clock source, before the prescalers.
    /// Ignored (and taken from the clock object) when clock == clkper.
    uint32_t source_hz = 0;

    /// The cycle rate wanted, in hertz. 0 means "use period_ticks".
    uint32_t hz = 0;
    /// The cycle length in counter ticks, when hz is 0.
    uint16_t period_ticks = 0;

    /// The dead time inserted on BOTH edges, in counter ticks.
    uint16_t dead_time_ticks = 0;

    /// The initial on-time of WOA, in counter ticks (0 = half of what
    /// is left after the two dead times).
    uint16_t duty_ticks = 0;

    bool woc = false;                                   ///< also drive WOC
    bool wod = false;                                   ///< also drive WOD
    TcdWaveformSelect wo_c = TcdWaveformSelect::pwm_a;
    TcdWaveformSelect wo_d = TcdWaveformSelect::pwm_b;

    /// Which level each output takes while an input event faults it.
    bool fault_high_a = false;
    bool fault_high_b = false;

    /// Update the double buffers at the end of the cycle automatically,
    /// the moment CMPBCLR's high byte is written (CTRLC.AUPDATE). With
    /// this off, duty() is followed by sync_at_end() when the caller
    /// wants the change on a cycle boundary.
    bool auto_update = false;
};

/**
 * TcdPwm<route>: the complementary pair with dead time - what this
 * timer is for. One ramp mode, the TCD cycle laid out as
 *
 *     0 .. CMPASET        dead time A   (both outputs low)
 *     CMPASET .. CMPACLR  on-time A     (WOA high)
 *     CMPACLR .. CMPBSET  dead time B   (both outputs low)
 *     CMPBSET .. CMPBCLR  on-time B     (WOB high)
 *
 * so CMPASET = dead - 1, CMPACLR = CMPASET + duty, CMPBSET = CMPACLR +
 * dead and CMPBCLR = period - 1. duty(v) moves the pair CMPACLR /
 * CMPBSET through the double buffers, which is why the two dead times
 * stay exactly what they were asked to be at every duty.
 *
 * NOT a PwmChannel: `max` here is a run-time value (period minus the two
 * dead times), and the concept wants a compile-time one. A half-bridge
 * driver is not a dimmable LED.
 *
 * A ClockUser: rebase() re-derives the period from the new CLK_PER when
 * the TCD is clocked from CLK_PER and the caller asked for a frequency;
 * a TCD on OSCHF, the PLL or EXTCLK does not move with the main clock at
 * all, and rebase() then leaves it alone.
 *
 *   using Pwm = brio::TcdPwm<brio::TcdRoute::def>;
 *   Pwm::init(clock, {.clock = brio::TcdClock::pll, .source_hz = 48'000'000,
 *                     .hz = 100'000, .dead_time_ticks = 12});
 *   Pwm::duty(Pwm::max() / 4);
 */
template <TcdRoute route>
struct TcdPwm {
    TcdPwm() = delete;
    static_assert(tcd_route_exists(route), "this package does not offer this TCD route");
    static_assert(tcd_pin(route, TcdOutput::woa).bonded &&
                  tcd_pin(route, TcdOutput::wob).bonded,
                  "a complementary pair needs both WOA and WOB bonded on this route");

    using T = Tcd<0>;

    /// Configure and start. False when the geometry does not fit 12
    /// bits, when the requested rate is not reachable, or when the
    /// resource refuses the configuration.
    template <typename Clock>
    static bool init(Clock clock_obj, const TcdPwmConfig& cfg) {
        static_assert(clock_follows<Clock, TcdPwm>(),
                      "TcdPwm on a DynamicClock that does not list it: a TCD clocked "
                      "from CLK_PER would silently change frequency");
        cfg_ = cfg;
        return apply(source_rate(clock_hz(clock_obj), cfg));
    }

    /// ClockUser: only a TCD clocked from CLK_PER and asked for a rate
    /// in hertz has anything to follow.
    static void rebase(uint32_t clk_per_hz) {
        if (cfg_.clock != TcdClock::clkper || cfg_.hz == 0) return;
        (void)apply(clk_per_hz);
    }

    /// The largest on-time WOA can be asked for: the cycle minus the two
    /// dead times. 0 before a successful init().
    static uint16_t max() { return max_; }
    /// The cycle length in counter ticks, and the counter's rate.
    static uint16_t period_ticks() { return period_; }
    static uint16_t dead_ticks() { return dead_; }
    static uint32_t counter_hz() { return counter_hz_; }
    /// The TCD cycle rate this geometry produces.
    static uint32_t cycle_hz() { return period_ ? counter_hz_ / period_ : 0; }

    /// WOA on-time in counter ticks; WOB takes the rest of the cycle
    /// minus the two dead times. Goes through the double buffers, so it
    /// lands at the next synchronization: at the end of the cycle by
    /// itself with auto_update, otherwise at the caller's sync() /
    /// sync_at_end(). False when v exceeds max().
    static bool duty(uint16_t v) {
        if (period_ == 0 || v > max_) return false;
        const uint16_t a_clr = static_cast<uint16_t>(a_set_ + v);
        const uint16_t b_set = static_cast<uint16_t>(a_clr + dead_);
        if (b_set > period_ - 1u) return false;
        if (!T::compare_a_clear(a_clr)) return false;
        if (!T::compare_b_set(b_set)) return false;
        duty_ = v;
        // With AUPDATE the write of CMPBCLR's high byte IS the request,
        // so rewriting the (unchanged) period is what schedules the
        // update - and it costs nothing else.
        if (cfg_.auto_update) return T::compare_b_clear(static_cast<uint16_t>(period_ - 1u));
        return true;
    }
    static uint16_t duty() { return duty_; }

    /// Adopt the pending duty now, or at the end of the cycle.
    static bool sync() { return T::sync(); }
    static bool sync_at_end() { return T::sync_at_end(); }

    /// Stop at the end of the cycle (both outputs low first), or now.
    static bool stop_at_end() { return T::disable_at_end(); }
    static bool stop() { return T::disable(); }
    static bool start() { return T::enable(); }
    static bool release() { period_ = 0; max_ = 0; return T::release(); }

private:
    static uint32_t source_rate(uint32_t clk_per_hz, const TcdPwmConfig& cfg) {
        return cfg.clock == TcdClock::clkper ? clk_per_hz : cfg.source_hz;
    }

    static bool apply(uint32_t src_hz) {
        const uint32_t cnt_hz = tcd_counter_hz(src_hz, cfg_.sync_prescaler, cfg_.count_prescaler);
        uint32_t period = cfg_.period_ticks;
        if (cfg_.hz != 0) {
            if (cnt_hz == 0) return false;
            period = (cnt_hz + cfg_.hz / 2u) / cfg_.hz;
        }
        if (period < 4 || period > static_cast<uint32_t>(tcd_compare_max) + 1u) return false;
        const uint16_t dead = cfg_.dead_time_ticks;
        if (static_cast<uint32_t>(dead) * 2u + 2u > period) return false;

        counter_hz_ = cnt_hz;
        period_ = static_cast<uint16_t>(period);
        dead_ = dead;
        a_set_ = dead == 0 ? 0u : static_cast<uint16_t>(dead - 1u);
        max_ = static_cast<uint16_t>(period_ - 1u - a_set_ - dead_);
        uint16_t duty = cfg_.duty_ticks;
        if (duty == 0 || duty > max_) duty = static_cast<uint16_t>(max_ / 2u);
        duty_ = duty;

        const uint16_t a_clr = static_cast<uint16_t>(a_set_ + duty_);
        const uint16_t b_set = static_cast<uint16_t>(a_clr + dead_);
        TcdConfig c{};
        c.route = route;
        c.clock = cfg_.clock;
        c.sync_prescaler = cfg_.sync_prescaler;
        c.count_prescaler = cfg_.count_prescaler;
        c.waveform = TcdWaveform::one_ramp;
        c.compare_a_set = a_set_;
        c.compare_a_clear = a_clr;
        c.compare_b_set = b_set;
        c.compare_b_clear = static_cast<uint16_t>(period_ - 1u);
        c.auto_update = cfg_.auto_update;
        c.wo_c = cfg_.wo_c;
        c.wo_d = cfg_.wo_d;
        c.enable_woa = true;
        c.enable_wob = true;
        c.enable_woc = cfg_.woc;
        c.enable_wod = cfg_.wod;
        c.fault_woa = cfg_.fault_high_a;
        c.fault_wob = cfg_.fault_high_b;
        c.fault_woc = cfg_.wo_c == TcdWaveformSelect::pwm_a ? cfg_.fault_high_a : cfg_.fault_high_b;
        c.fault_wod = cfg_.wo_d == TcdWaveformSelect::pwm_a ? cfg_.fault_high_a : cfg_.fault_high_b;
        return T::init(c);
    }

    static inline TcdPwmConfig cfg_{};
    static inline uint32_t counter_hz_ = 0;
    static inline uint16_t period_ = 0;
    static inline uint16_t dead_ = 0;
    static inline uint16_t a_set_ = 0;
    static inline uint16_t max_ = 0;
    static inline uint16_t duty_ = 0;
};

static_assert(ClockUser<TcdPwm<TcdRoute::def>>);

} // namespace brio
