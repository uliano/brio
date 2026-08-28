/*
 * tca.hpp
 *
 * The AVR DA/DB 16-bit timer/counter type A (TCA, DS40002247B ch. 23)
 * in two strata, as docs/avrdx/tca.md describes:
 *
 *  RESOURCE - Tca<n>: the typed view of one instance in NORMAL mode
 *  (16-bit counter, PER, three buffered compare channels, the waveform
 *  modes, the two event inputs, the commands) - a config struct owns
 *  the configuration, init<cfg>() folds it, init(cfg) computes it; the
 *  verbs: count/period/compare (direct and buffered), the flags, the
 *  interrupt enables, the ISR bodies (one vector PER flag: ovf(),
 *  cmp<ch>()), restart/update/reset, direction, the event hooks
 *  event_a_on / event_b_on over evsys.hpp. The split view (two 8-bit
 *  timers) is reached through split(): the TcaPwm task lives there.
 *
 *  TASKS - what an application names:
 *    TcaPwm<n, port>              split mode: six 8-bit PWM channels
 *    TcaPwm16<n, port, steps>     single-slope: three 16-bit PWM channels
 *                                 (buffered, clean endpoints), one period
 *    FrequencyGenerator<n, port>  FRQ: a square wave of a frequency on WO0
 *                                 (ClockUser)
 *    Heartbeat<n, port>           a period at a rate: OVF interrupt/event
 *                                 plus up to three pulses of chosen widths
 *                                 at the start of each period on WO0..2
 *                                 (ClockUser)
 *    EventCounter<n>              count events on input A (16 bits), with
 *                                 direction from input B if wanted
 *  A task owns its instance.
 *
 * Facts that shape the code (23.3, 23.5, errata DS80000915F 2.12.1):
 *  - CLK_PER / N, N in 1/2/4/8/16/64/256/1024 (the same CLKSEL codes in
 *    both modes); the prescaled clock is exported to the TCBs;
 *  - PER and CMPn are double-buffered: write PERBUF/CMPnBUF and the
 *    value lands at the UPDATE condition (TOP in NORMAL/FRQ, BOTTOM in
 *    the PWM modes) - the glitch-free way to change a running waveform;
 *    unbuffered writes are immediate (a PER below CNT makes the counter
 *    wrap to MAX first). ALUPD locks the update until every used buffer
 *    is written - a multi-register change lands in one period;
 *  - single-slope: WOn set at BOTTOM, cleared at the match; CMP = 0 is a
 *    static low, CMP > PER a static high (clean endpoints in NORMAL mode
 *    - in split mode CMP > TOP is LOW, hence TcaPwm's PORT endpoints);
 *    dual-slope: up then down, OVF at TOP / both / BOTTOM by mode;
 *    FRQ: WOn toggles at each CMPn match, period in CMP0: f = CLK_PER
 *    / (2 N (CMP0 + 1)), at most CLK_PER / 2;
 *  - RESTART (command or event) zeroes CNT and all WO; on A4/A5 it also
 *    resets DIR to up in NORMAL/FRQ (2.12.1); RESET only when disabled;
 *    switching normal <-> split wants disable + RESET;
 *  - event input A: count on positive/any edge, count the clock while
 *    high, or direction from the level (up when low); input B: restart
 *    on positive/any edge or while high, or direction (ORed with A);
 *    level actions need the event slower than the timer clock;
 *  - routes (PORTMUX.TCAROUTEA, the whole group moves): TCA0 -> PORTA/
 *    B/C/D/E/F (and PORTG on 64-pin) pins 0..5 (PORTE only PE0..3 on
 *    48 pins); TCA1 -> PORTB or PORTG (64-pin) pins 0..5, or the
 *    three-channel PORTC/PORTE (64-pin) with WO0..2 on pins 4..6; the
 *    device header defines exactly this package's codes and
 *    tca_route_code() mirrors it. WOn drives the pin only if CMPnEN
 *    and the pin is an output (PORT INVEN inverts it).
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// ---- the knobs (23.5) -------------------------------------------------------------

/// CLKSEL: CLK_PER / N (the codes are shared by normal and split mode).
enum class TcaClock : uint8_t {
    div1 = TCA_SINGLE_CLKSEL_DIV1_gc,
    div2 = TCA_SINGLE_CLKSEL_DIV2_gc,
    div4 = TCA_SINGLE_CLKSEL_DIV4_gc,
    div8 = TCA_SINGLE_CLKSEL_DIV8_gc,
    div16 = TCA_SINGLE_CLKSEL_DIV16_gc,
    div64 = TCA_SINGLE_CLKSEL_DIV64_gc,
    div256 = TCA_SINGLE_CLKSEL_DIV256_gc,
    div1024 = TCA_SINGLE_CLKSEL_DIV1024_gc,
};

constexpr uint16_t tca_divisor(TcaClock c) {
    switch (c) {
        case TcaClock::div1: return 1;
        case TcaClock::div2: return 2;
        case TcaClock::div4: return 4;
        case TcaClock::div8: return 8;
        case TcaClock::div16: return 16;
        case TcaClock::div64: return 64;
        case TcaClock::div256: return 256;
        case TcaClock::div1024: return 1024;
    }
    return 1;
}

/// WGMODE (normal mode).
enum class TcaMode : uint8_t {
    normal = TCA_SINGLE_WGMODE_NORMAL_gc,             ///< count to PER, no waveform
    frequency = TCA_SINGLE_WGMODE_FRQ_gc,             ///< toggle WOn at CMPn match, period CMP0
    single_slope = TCA_SINGLE_WGMODE_SINGLESLOPE_gc,  ///< PWM: set at BOTTOM, clear at match
    dual_slope_top = TCA_SINGLE_WGMODE_DSTOP_gc,      ///< PWM up/down, OVF at TOP
    dual_slope_both = TCA_SINGLE_WGMODE_DSBOTH_gc,    ///< OVF at TOP and BOTTOM
    dual_slope_bottom = TCA_SINGLE_WGMODE_DSBOTTOM_gc,///< OVF at BOTTOM
};

/// EVACTA: what event input A does.
enum class TcaEventA : uint8_t {
    none = 0xFF,
    count_posedge = TCA_SINGLE_EVACTA_CNT_POSEDGE_gc,
    count_anyedge = TCA_SINGLE_EVACTA_CNT_ANYEDGE_gc,
    count_while_high = TCA_SINGLE_EVACTA_CNT_HIGHLVL_gc,
    direction = TCA_SINGLE_EVACTA_UPDOWN_gc,          ///< up while low, down while high
};

/// EVACTB: what event input B does.
enum class TcaEventB : uint8_t {
    none = 0xFF,
    direction = TCA_SINGLE_EVACTB_UPDOWN_gc,
    restart_posedge = TCA_SINGLE_EVACTB_RESTART_POSEDGE_gc,
    restart_anyedge = TCA_SINGLE_EVACTB_RESTART_ANYEDGE_gc,
    restart_while_high = TCA_SINGLE_EVACTB_RESTART_HIGHLVL_gc,
};

struct TcaConfig {
    TcaMode mode = TcaMode::normal;
    TcaClock clock = TcaClock::div1;
    uint16_t period = 0xFFFF;        ///< PER (TOP in every mode but FRQ, where CMP0 is)
    uint16_t compare0 = 0;
    uint16_t compare1 = 0;
    uint16_t compare2 = 0;
    uint8_t outputs = 0;             ///< CMPnEN bits 0..2: WOn on the pin (driven as output by init)
    char route = 0;                  ///< 'A'..'G': PORTMUX group; 0 = leave the route alone
    TcaEventA event_a = TcaEventA::none;
    TcaEventB event_b = TcaEventB::none;
    bool auto_lock_update = false;   ///< ALUPD: buffers land together once all are written
    bool count_down = false;         ///< DIR at start
    bool run_standby = false;
    bool debug_run = false;
};

/// PORTMUX code for "TCA n to port p", or 0xFF when that route does
/// not exist on THIS package - the device header is the authority
/// (it defines exactly the codes each package routes: e.g. TCA0 ->
/// PORTG and TCA1 -> PORTE/PORTG on 64-pin parts only, no PORTB/
/// PORTE codes at all on 28/32-pin). TCA1 on PORTC and PORTE are
/// three-channel routes (WO0..2 on pins 4..6).
constexpr uint8_t tca_route_code(uint8_t n, char p) {
    // The gates are the PORT instance macros (the route codes are
    // enumerators, not macros): a package routes a TCA to exactly the
    // ports it bonds, and PORTG doubles as the 64-pin marker (the
    // three-channel TCA1 -> PORTE exists on 64-pin parts only).
    if (n == 0) {
        switch (p) {
            case 'A': return PORTMUX_TCA0_PORTA_gc;
#ifdef PORTB
            case 'B': return PORTMUX_TCA0_PORTB_gc;
#endif
            case 'C': return PORTMUX_TCA0_PORTC_gc;
            case 'D': return PORTMUX_TCA0_PORTD_gc;
#ifdef PORTE
            case 'E': return PORTMUX_TCA0_PORTE_gc;
#endif
            case 'F': return PORTMUX_TCA0_PORTF_gc;
#ifdef PORTG
            case 'G': return PORTMUX_TCA0_PORTG_gc;
#endif
            default: return 0xFF;
        }
    }
#ifdef TCA1
    switch (p) {
        case 'B': return PORTMUX_TCA1_PORTB_gc;
        case 'C': return PORTMUX_TCA1_PORTC_gc;
#ifdef PORTG
        case 'E': return PORTMUX_TCA1_PORTE_gc;
        case 'G': return PORTMUX_TCA1_PORTG_gc;
#endif
        default: return 0xFF;
    }
#else
    (void)p;
    return 0xFF;
#endif
}

/// The pin number of WO wo on that route: pin wo, except TCA1's
/// three-channel routes (PORTC, PORTE) where WO0..2 sit on pins 4..6.
constexpr uint8_t tca_wo_pin(uint8_t n, char p, uint8_t wo) {
    return (n == 1 && (p == 'C' || p == 'E')) ? static_cast<uint8_t>(4 + wo) : wo;
}

/// The port pin mask of a WO mask (bits 0..5) on that route.
constexpr uint8_t tca_pin_mask(uint8_t n, char p, uint8_t wo_mask) {
    uint8_t pins = 0;
    for (uint8_t wo = 0; wo < 6; ++wo) {
        if (wo_mask & (1u << wo)) pins |= static_cast<uint8_t>(1u << tca_wo_pin(n, p, wo));
    }
    return pins;
}

constexpr bool tca_config_valid(uint8_t n, const TcaConfig& c) {
    if (c.outputs > 7) return false;
    if (c.route != 0 && tca_route_code(n, c.route) == 0xFF) return false;
    return true;
}

/// The waveform-generator flags of one instance, as take_flags() returns them.
struct TcaFlags {
    bool ovf;
    bool cmp0;
    bool cmp1;
    bool cmp2;
};

// ---- the resource ---------------------------------------------------------------

template <uint8_t n>
class Tca {
#ifdef TCA1
    static_assert(n <= 1, "AVR DA/DB: TCA0 and (48/64-pin parts) TCA1");
#else
    static_assert(n == 0, "AVR DA/DB 28/32-pin: TCA0 only");
#endif

public:
    Tca() = delete;

    static constexpr uint8_t index = n;

    /// The event vocabulary of this instance (evsys.hpp).
    using OvfEvent = EvTcaOvf<n>;                    ///< generator: overflow (normal) / LUNF (split)
    using HunfEvent = EvTcaHunf<n>;                  ///< generator: high-half underflow (split)
    template <uint8_t ch> using CmpEvent = EvTcaCmp<n, ch>;
    using EventA = EvTcaCntA<n>;                     ///< user: count / direction
    using EventB = EvTcaCntB<n>;                     ///< user: restart / direction

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: the configuration checked and folded.
    template <TcaConfig cfg>
    static void init() {
        static_assert(tca_config_valid(n, cfg),
                      "TcaConfig: outputs is a 3-bit mask; the route must be one this "
                      "package's PORTMUX has (the device header lists its TCAROUTEA codes)");
        init(cfg);
    }

    /// Run-time form: disables, hard-resets (known state, normal mode),
    /// routes, drives the enabled outputs' pins, writes every knob,
    /// clears the flags, enables. Interrupts stay off. False, touching
    /// nothing, for an invalid configuration.
    static bool init(const TcaConfig& cfg) {
        if (!tca_config_valid(n, cfg)) return false;
        auto& t = single();
        reset();                                        // works from split mode too (CMDEN)
        if (cfg.route) route(cfg.route);
        if (cfg.outputs && cfg.route) {
            drive_outputs(cfg.route, cfg.outputs);
        }
        t.CTRLB = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.mode) |
            (cfg.auto_lock_update ? TCA_SINGLE_ALUPD_bm : 0) |
            static_cast<uint8_t>(cfg.outputs << 4));    // CMP0EN..CMP2EN = bits 4..6
        t.EVCTRL = static_cast<uint8_t>(
            (cfg.event_a != TcaEventA::none ? (static_cast<uint8_t>(cfg.event_a) | TCA_SINGLE_CNTAEI_bm) : 0) |
            (cfg.event_b != TcaEventB::none ? (static_cast<uint8_t>(cfg.event_b) | TCA_SINGLE_CNTBEI_bm) : 0));
        t.PER = cfg.period;
        t.CMP0 = cfg.compare0;
        t.CMP1 = cfg.compare1;
        t.CMP2 = cfg.compare2;
        t.CNT = 0;
        if (cfg.count_down) t.CTRLESET = TCA_SINGLE_DIR_bm;
        t.DBGCTRL = cfg.debug_run ? TCA_SINGLE_DBGRUN_bm : 0;
        t.INTFLAGS = TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm;
        t.CTRLA = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.clock) |
            (cfg.run_standby ? TCA_SINGLE_RUNSTDBY_bm : 0) |
            TCA_SINGLE_ENABLE_bm);
        return true;
    }

    static void enable() { single().CTRLA |= TCA_SINGLE_ENABLE_bm; }
    static void disable() { single().CTRLA &= static_cast<uint8_t>(~TCA_SINGLE_ENABLE_bm); }

    /// Change the prescaler under a running timer (the count goes on).
    static void clock(TcaClock c) {
        auto& t = single();
        t.CTRLA = static_cast<uint8_t>((t.CTRLA & ~TCA_SINGLE_CLKSEL_gm) | static_cast<uint8_t>(c));
    }

    /// PORTMUX: move the WO group to port p (no check: init() checks).
    static void split_irq(uint8_t bit, bool on) {
        if (on) split().INTCTRL |= bit; else split().INTCTRL &= static_cast<uint8_t>(~bit);
    }

    static void route(char p) {
#ifdef TCA1
        constexpr uint8_t mask = n == 0 ? PORTMUX_TCA0_gm : PORTMUX_TCA1_gm;
#else
        constexpr uint8_t mask = PORTMUX_TCA0_gm;
#endif
        PORTMUX.TCAROUTEA = static_cast<uint8_t>(
            (PORTMUX.TCAROUTEA & ~mask) | tca_route_code(n, p));
    }

    /// Disable + hard reset: the way into split mode (TcaPwm) or back.
    /// The command goes through the SPLIT view with CMDEN = both: in
    /// split mode a RESET without CMDEN is silently ignored (23.7.6),
    /// and the CMDEN bits are don't-care in normal mode - one write
    /// works from either side.
    static void reset() {
        split().CTRLA = 0;
        split().CTRLESET = static_cast<uint8_t>(static_cast<uint8_t>(TCA_SPLIT_CMD_RESET_gc) |
                                                static_cast<uint8_t>(TCA_SPLIT_CMDEN_BOTH_gc));
    }

    // ---- the counter ------------------------------------------------------

    static uint16_t count() { return single().CNT; }
    static void count(uint16_t v) { single().CNT = v; }
    static uint16_t period() { return single().PER; }
    /// Immediate (unbuffered) writes.
    static void period(uint16_t v) { single().PER = v; }
    template <uint8_t ch>
    static void compare(uint16_t v) { cmp_reg<ch>() = v; }
    template <uint8_t ch>
    static uint16_t compare() { return cmp_reg<ch>(); }
    /// Buffered writes: land at the UPDATE condition (glitch-free).
    static void period_buffered(uint16_t v) { single().PERBUF = v; }
    template <uint8_t ch>
    static void compare_buffered(uint16_t v) { cmpbuf<ch>() = v; }
    /// CTRLF buffer-valid flags: a buffered write still pending.
    static bool update_pending() { return (single().CTRLFCLR & (TCA_SINGLE_PERBV_bm | TCA_SINGLE_CMP0BV_bm | TCA_SINGLE_CMP1BV_bm | TCA_SINGLE_CMP2BV_bm)) != 0; }

    /// Enable/disable WOn on its pin at run time (CMPnEN).
    template <uint8_t ch>
    static void output(bool on) {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        constexpr uint8_t bit = static_cast<uint8_t>(TCA_SINGLE_CMP0EN_bm << ch);
        if (on) single().CTRLB |= bit; else single().CTRLB &= static_cast<uint8_t>(~bit);
    }

    /// CTRLC: WOn's output VALUE - readable any time, writable only
    /// while the timer is disabled (23.5.3): the one handle to park a
    /// waveform pin high or low between runs. Toward the CCL the
    /// output bypasses CMPnEN: a parked value feeds a LUT too. The
    /// split-mode twins (LCMPnOV/HCMPnOV) are reachable via split().
    template <uint8_t ch>
    static void output_value(bool high) {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        constexpr uint8_t bit = static_cast<uint8_t>(TCA_SINGLE_CMP0OV_bm << ch);
        if (high) single().CTRLC |= bit; else single().CTRLC &= static_cast<uint8_t>(~bit);
    }
    template <uint8_t ch>
    static bool output_value() {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        return (single().CTRLC & static_cast<uint8_t>(TCA_SINGLE_CMP0OV_bm << ch)) != 0;
    }

    // ---- commands, direction ----------------------------------------------

    /// CNT = 0, all WO low, the period restarts (2.12.1: DIR back to up on A4/A5).
    static void restart() { single().CTRLESET = TCA_SINGLE_CMD_RESTART_gc; }
    /// Force the buffer copy now (ignores LUPD).
    static void update() { single().CTRLESET = TCA_SINGLE_CMD_UPDATE_gc; }
    static void direction_down(bool down) {
        if (down) single().CTRLESET = TCA_SINGLE_DIR_bm; else single().CTRLECLR = TCA_SINGLE_DIR_bm;
    }
    static bool counting_down() { return (single().CTRLECLR & TCA_SINGLE_DIR_bm) != 0; }
    /// LUPD: hold the buffered values back until released.
    static void lock_update(bool lock) {
        if (lock) single().CTRLESET = TCA_SINGLE_LUPD_bm; else single().CTRLECLR = TCA_SINGLE_LUPD_bm;
    }

    // ---- flags and interrupts ---------------------------------------------

    static bool ovf_flag() { return (single().INTFLAGS & TCA_SINGLE_OVF_bm) != 0; }
    template <uint8_t ch>
    static bool cmp_flag() { return (single().INTFLAGS & cmp_bit<ch>()) != 0; }
    static void clear_ovf() { single().INTFLAGS = TCA_SINGLE_OVF_bm; }
    template <uint8_t ch>
    static void clear_cmp() { single().INTFLAGS = cmp_bit<ch>(); }

    static void enable_ovf_interrupt(bool on) { irq(TCA_SINGLE_OVF_bm, on); }
    template <uint8_t ch>
    static void enable_cmp_interrupt(bool on) { irq(cmp_bit<ch>(), on); }

    /// ISR bodies: one vector per flag (TCAn_OVF_vect, TCAn_CMPm_vect).
    [[gnu::always_inline]] static void ovf() { clear_ovf(); }
    template <uint8_t ch>
    [[gnu::always_inline]] static void cmp() { clear_cmp<ch>(); }
    /// All four at once, cleared (for a handler that polls).
    static TcaFlags take_flags() {
        const uint8_t f = single().INTFLAGS;
        single().INTFLAGS = f;
        return {(f & TCA_SINGLE_OVF_bm) != 0, (f & TCA_SINGLE_CMP0_bm) != 0,
                (f & TCA_SINGLE_CMP1_bm) != 0, (f & TCA_SINGLE_CMP2_bm) != 0};
    }

    // ---- split-mode flags and interrupts (23.7.7 / 23.7.8) ----------------
    // The low half underflow shares the OVF vector (TCAn_OVF_vect =
    // LUNF), the high half has its own (TCAn_HUNF_vect); the LCMPn
    // flags share the CMPn vectors. Only the low half has compare
    // interrupts in split mode.

    static bool lunf_flag() { return (split().INTFLAGS & TCA_SPLIT_LUNF_bm) != 0; }
    static bool hunf_flag() { return (split().INTFLAGS & TCA_SPLIT_HUNF_bm) != 0; }
    template <uint8_t ch>
    static bool lcmp_flag() {
        static_assert(ch <= 2, "split low-half compare channels: 0..2");
        return (split().INTFLAGS & static_cast<uint8_t>(TCA_SPLIT_LCMP0_bm << ch)) != 0;
    }
    static void clear_lunf() { split().INTFLAGS = TCA_SPLIT_LUNF_bm; }
    static void clear_hunf() { split().INTFLAGS = TCA_SPLIT_HUNF_bm; }
    template <uint8_t ch>
    static void clear_lcmp() { split().INTFLAGS = static_cast<uint8_t>(TCA_SPLIT_LCMP0_bm << ch); }

    static void enable_lunf_interrupt(bool on) { split_irq(TCA_SPLIT_LUNF_bm, on); }
    static void enable_hunf_interrupt(bool on) { split_irq(TCA_SPLIT_HUNF_bm, on); }
    template <uint8_t ch>
    static void enable_lcmp_interrupt(bool on) {
        static_assert(ch <= 2, "split low-half compare channels: 0..2");
        split_irq(static_cast<uint8_t>(TCA_SPLIT_LCMP0_bm << ch), on);
    }

    /// Split-mode ISR bodies (vectors as above).
    [[gnu::always_inline]] static void lunf() { clear_lunf(); }
    [[gnu::always_inline]] static void hunf() { clear_hunf(); }
    template <uint8_t ch>
    [[gnu::always_inline]] static void lcmp() { clear_lcmp<ch>(); }

    // ---- events -----------------------------------------------------------

    /// Event input A / B from channel ch; the ACTION is in the config.
    template <uint8_t ch>
    static void event_a_on(EventChannel<ch> c) { EventA::listen(c); }
    template <uint8_t ch>
    static void event_b_on(EventChannel<ch> c) { EventB::listen(c); }

    // ---- the register views -----------------------------------------------

    static constexpr TCA_SINGLE_t& single() {
        if constexpr (n == 0) return TCA0.SINGLE;
#ifdef TCA1
        else return TCA1.SINGLE;
#endif
    }
    static constexpr TCA_SPLIT_t& split() {
        if constexpr (n == 0) return TCA0.SPLIT;
#ifdef TCA1
        else return TCA1.SPLIT;
#endif
    }

    /// Drive the pins of the WO channels in `mask` (bits 0..5) on port p
    /// as outputs, low (the route's pin numbering applied).
    static void drive_outputs(char p, uint8_t mask) {
        const uint8_t pins = tca_pin_mask(n, p, mask);
        volatile PORT_t& port = port_of(p);
        port.OUTCLR = pins;
        port.DIRSET = pins;
    }

    static constexpr volatile PORT_t& port_of(char p) { return port_by_letter(p); }

private:
    template <uint8_t ch>
    static constexpr uint8_t cmp_bit() {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        return static_cast<uint8_t>(TCA_SINGLE_CMP0_bm << ch);
    }
    template <uint8_t ch>
    static volatile uint16_t& cmp_reg() {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        return (&single().CMP0)[ch];
    }
    template <uint8_t ch>
    static volatile uint16_t& cmpbuf() {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        return (&single().CMP0BUF)[ch];
    }
    static void irq(uint8_t bit, bool on) {
        if (on) single().INTCTRL |= bit; else single().INTCTRL &= static_cast<uint8_t>(~bit);
    }
};

// ---- tasks ----------------------------------------------------------------------

/// The smallest prescaler at which `ticks_at_div1` fits in 16 bits (as
/// a TOP, i.e. ticks <= 65536), and the count at that prescaler.
struct TcaTiming {
    TcaClock clock;
    uint32_t ticks;        ///< 0 when it fits at no prescaler
};
constexpr TcaTiming tca_timing(uint64_t ticks_at_div1) {
    constexpr TcaClock divs[] = {TcaClock::div1, TcaClock::div2, TcaClock::div4, TcaClock::div8,
                                 TcaClock::div16, TcaClock::div64, TcaClock::div256, TcaClock::div1024};
    for (TcaClock d : divs) {
        const uint64_t t = (ticks_at_div1 + tca_divisor(d) - 1) / tca_divisor(d);
        if (t >= 1 && t <= 0x10000u) return {d, static_cast<uint32_t>(t)};
    }
    return {TcaClock::div1, 0};
}
/// Period of `hz` in ticks at CLK_PER, rounded to nearest.
constexpr uint64_t tca_period_ticks(uint32_t clk_per_hz, uint32_t hz) {
    return hz ? (static_cast<uint64_t>(clk_per_hz) + hz / 2) / hz : 0;
}

/// TcaPwm<n, port>: TCA n in SPLIT mode as six independent 8-bit PWM
/// channels, WO0..WO5 on pins 0..5 of one port. Both halves in single-
/// slope PWM with PER = 255, one shared prescaler: f = CLK_PER / N / 256
/// (24 MHz, div16 -> ~5.9 kHz: no flicker, no whine). The compare
/// hardware has no clean 0 % / 100 % in split mode (CMP = 0 still gives
/// one clock of output, CMP = PER leaves one clock off), so duty 0 and
/// 255 disable the channel's waveform and drive the pin from PORT.OUT -
/// DxCore's policy. Active-high (common-cathode LED); Pin::invert(true)
/// for active-low loads. Six channels or nothing: the partial routes
/// (TCA1 on PORTC) are not offered. Not a ClockUser: the frequency
/// follows CLK_PER and a LED does not care - an app that does can call
/// clock() itself. Channel<ch> is the PwmChannel type (max 255) that
/// generic actuators (RgbLamp) take.
///
///   using Bar = brio::TcaPwm<0, 'C'>;   // TCA0 -> PC0..PC5
///   Bar::init();                       // split mode, div16, all six on
///   Bar::duty<2>(64);                  // PC2 at 25 %
template <uint8_t n, char PortLetter>
class TcaPwm {
    using R = Tca<n>;
    static_assert(tca_route_code(n, PortLetter) != 0xFF
                      && !(n == 1 && (PortLetter == 'C' || PortLetter == 'E')),
                  "TcaPwm needs a six-channel route: any route of this package except "
                  "TCA1 -> PORTC/PORTE (three channels on pins 4..6). Caveat: TCA0 -> "
                  "PORTE on 48-pin parts has only PE0..PE3 bonded (WO4/5 land nowhere)");

public:
    TcaPwm() = delete;

    static constexpr uint8_t channels = 6;

    /// Route the timer to the port, enter split mode with PER = 255 on
    /// both halves, all six compare outputs enabled at duty 0, pins as
    /// outputs driven low, counter running from the chosen prescaler.
    static void init(TcaClock clock = TcaClock::div16) {
        R::reset();                                     // known state (also clears CTRLD)
        R::route(PortLetter);
        auto& t = R::split();
        t.CTRLD = TCA_SPLIT_SPLITM_bm;
        t.LPER = 255;
        t.HPER = 255;
        t.LCMP0 = 0; t.LCMP1 = 0; t.LCMP2 = 0;
        t.HCMP0 = 0; t.HCMP1 = 0; t.HCMP2 = 0;
        R::drive_outputs(PortLetter, 0x3F);            // duty 0 = pins low, outputs
        t.CTRLB = 0;                                   // endpoints policy: all off = PORT.OUT
        t.CTRLA = static_cast<uint8_t>(static_cast<uint8_t>(clock) | TCA_SPLIT_ENABLE_bm);
    }

    /// Change the shared prescaler (all six frequencies move).
    static void clock(TcaClock c) {
        auto& t = R::split();
        t.CTRLA = static_cast<uint8_t>((t.CTRLA & ~TCA_SPLIT_CLKSEL_gm) | static_cast<uint8_t>(c));
    }

    /// Set channel ch (WO0..WO5 = pin 0..5) to value/256 duty.
    /// 0 and 255 leave the waveform and drive the pin from PORT.OUT.
    template <uint8_t ch>
    static void duty(uint8_t value) {
        static_assert(ch < channels, "TCA split mode has six channels, WO0..WO5");
        auto& t = R::split();
        volatile PORT_t& port = R::port_of(PortLetter);
        if (value == 0) {
            t.CTRLB &= static_cast<uint8_t>(~cmp_enable_bit<ch>());
            port.OUTCLR = static_cast<uint8_t>(1u << ch);
        } else if (value == 255) {
            t.CTRLB &= static_cast<uint8_t>(~cmp_enable_bit<ch>());
            port.OUTSET = static_cast<uint8_t>(1u << ch);
        } else {
            cmp<ch>() = value;
            t.CTRLB |= cmp_enable_bit<ch>();
        }
    }

    /// Channel ch as a PwmChannel type (max 255).
    template <uint8_t ch>
    struct Channel {
        static_assert(ch < channels, "TCA split mode has six channels, WO0..WO5");
        static constexpr uint16_t max = 255;
        static void duty(uint16_t v) { TcaPwm::duty<ch>(static_cast<uint8_t>(v)); }
    };
    static_assert(PwmChannel<Channel<0>>);

private:
    // Split-mode compare registers interleave: LCMP0, HCMP0, LCMP1, HCMP1,
    // LCMP2, HCMP2. WO0-2 -> LCMPn, WO3-5 -> HCMPn.
    template <uint8_t ch>
    static volatile uint8_t& cmp() {
        if constexpr (ch < 3) return (&R::split().LCMP0)[2 * ch];
        else return (&R::split().HCMP0)[2 * (ch - 3)];
    }
    // CTRLB: LCMP0EN..LCMP2EN = bits 0-2, HCMP0EN..HCMP2EN = bits 4-6.
    template <uint8_t ch>
    static constexpr uint8_t cmp_enable_bit() {
        return ch < 3 ? static_cast<uint8_t>(1u << ch) : static_cast<uint8_t>(1u << (ch + 1));
    }
};

/// TcaPwm16<n, port, steps>: three 16-bit PWM channels (WO0..2) of a
/// shared period of `steps` ticks (PER = steps - 1, f = CLK_PER / N /
/// steps, resolution log2(steps) bits) in single-slope mode, duty
/// written BUFFERED (lands at BOTTOM: glitch-free) with the hardware's
/// clean endpoints: 0 = static low, steps = static high (CMP > TOP).
/// Channel<ch> is a PwmChannel with max = steps. Not a ClockUser (as
/// TcaPwm). A servo at 50 Hz: steps 60000 at div8 from 24 MHz.
template <uint8_t n, char PortLetter, uint16_t steps = 0xFFFF>
class TcaPwm16 {
    using R = Tca<n>;
    static_assert(steps >= 4, "TcaPwm16: at least two bits of resolution (steps >= 4)");
    static_assert(tca_route_code(n, PortLetter) != 0xFF, "TcaPwm16: no such TCA route");

public:
    TcaPwm16() = delete;

    static constexpr uint16_t max = steps;

    /// outputs: WO0..2 mask; all start at duty 0 (static low).
    static void init(TcaClock clock = TcaClock::div1, uint8_t outputs = 0x07) {
        R::init({.mode = TcaMode::single_slope, .clock = clock,
                 .period = static_cast<uint16_t>(steps - 1),
                 .outputs = static_cast<uint8_t>(outputs & 0x07), .route = PortLetter});
    }
    static void clock(TcaClock c) { R::clock(c); }

    template <uint8_t ch>
    static void duty(uint16_t v) {
        static_assert(ch <= 2, "TcaPwm16: WO0..WO2");
        R::template compare_buffered<ch>(v >= steps ? static_cast<uint16_t>(steps) : v);
    }

    template <uint8_t ch>
    struct Channel {
        static constexpr uint16_t max = steps;
        static void duty(uint16_t v) { TcaPwm16::duty<ch>(v); }
    };
    static_assert(PwmChannel<Channel<0>>);
};

/// TcaPwmCentered<n, port, steps>: three 16-bit CENTER-ALIGNED PWM
/// channels (WO0..2) in dual-slope mode. The counter runs 0 -> steps
/// -> 0; WOn is set at the match while counting down and cleared at
/// the match while counting up, so the pulse is symmetric around
/// BOTTOM: changing the duty moves both edges and the pulse centre
/// stands still (f = CLK_PER / N / (2 steps), duty = v / steps, width
/// = 2 v ticks of the timer clock). The OVF event is this task's
/// point: `EventAt` places it at the centre of the pulse (bottom),
/// the centre of the low time (top), or both - the electrically
/// quiet instants, made to pace an ADC with no CPU (EvTcaOvf ->
/// EvAdc0Start: the AnalogSampler's hardware pace measuring a
/// PWM-driven load away from its switching edges). Duty writes are
/// BUFFERED (land at BOTTOM, glitch-free); the endpoints leave the
/// waveform and drive the pin from PORT, like TcaPwm. Only channels
/// in init's `outputs` mask have their pins driven. Ratiometric -
/// NOT a ClockUser: the duty survives a clock change, the frequency
/// follows CLK_PER. Works on the three-channel TCA1 routes too
/// (WO0..2 on pins 4..6).
template <uint8_t n, char PortLetter, uint16_t steps = 0xFFFF>
class TcaPwmCentered {
    using R = Tca<n>;
    static_assert(steps >= 4, "TcaPwmCentered: at least two bits of resolution (steps >= 4)");
    static_assert(tca_route_code(n, PortLetter) != 0xFF,
                  "TcaPwmCentered: no such TCA route on this package");

public:
    TcaPwmCentered() = delete;

    static constexpr uint16_t max = steps;

    /// Where the OVF interrupt/event falls in the period.
    enum class EventAt : uint8_t { top, both, bottom };

    static bool init(TcaClock clock = TcaClock::div1, uint8_t outputs = 0x01,
                     EventAt event_at = EventAt::bottom) {
        const TcaMode m = event_at == EventAt::top    ? TcaMode::dual_slope_top
                        : event_at == EventAt::both   ? TcaMode::dual_slope_both
                                                      : TcaMode::dual_slope_bottom;
        return R::init({.mode = m, .clock = clock, .period = steps,
                        .outputs = static_cast<uint8_t>(outputs & 0x7),
                        .route = PortLetter});
    }

    /// Change the shared prescaler (all frequencies move, duties hold).
    static void clock(TcaClock c) { R::clock(c); }

    /// Duty v/steps on WOn, buffered; 0 and max leave the waveform and
    /// drive the pin from PORT (clean rails).
    template <uint8_t ch>
    static void duty(uint16_t v) {
        static_assert(ch <= 2, "TCA compare channels: 0..2");
        volatile PORT_t& port = R::port_of(PortLetter);
        constexpr uint8_t pin = static_cast<uint8_t>(1u << tca_wo_pin(n, PortLetter, ch));
        if (v == 0) {
            R::template output<ch>(false);
            port.OUTCLR = pin;
        } else if (v >= max) {
            R::template output<ch>(false);
            port.OUTSET = pin;
        } else {
            R::template compare_buffered<ch>(v);
            R::template output<ch>(true);
        }
    }

    /// Channel ch as a PwmChannel type (max = steps).
    template <uint8_t ch>
    struct Channel {
        static constexpr uint16_t max = steps;
        static void duty(uint16_t v) { TcaPwmCentered::duty<ch>(v); }
    };
    static_assert(PwmChannel<Channel<0>>);
};

/// FrequencyGenerator<n, port>: a square wave of a frequency on WO0
/// (pin 0 of the route), FRQ mode: f = CLK_PER / (2 N (CMP0 + 1)) - the
/// prescaler is the smallest that fits, the frequency is rounded to the
/// nearest achievable (actual_hz() tells). A ClockUser: rebase() keeps
/// the frequency on a clock change. set_hz() under a running output is
/// buffered (lands at the next toggle).
template <uint8_t n, char PortLetter>
class FrequencyGenerator {
    using R = Tca<n>;
    static_assert(tca_route_code(n, PortLetter) != 0xFF, "FrequencyGenerator: no such TCA route");

public:
    FrequencyGenerator() = delete;

    template <typename Clock>
    static bool init(Clock clock, uint32_t hz) {
        static_assert(clock_follows<Clock, FrequencyGenerator>(),
                      "FrequencyGenerator on a DynamicClock that does not list it: the frequency would drift");
        clk_per_ = clock_hz(clock);
        const TcaTiming w = timing(clk_per_, hz);
        if (w.ticks == 0) return false;
        hz_ = hz;
        div_ = w.clock;
        ticks_ = w.ticks;
        R::init({.mode = TcaMode::frequency, .clock = w.clock,
                 .compare0 = static_cast<uint16_t>(w.ticks - 1), .outputs = 0x01, .route = PortLetter});
        return true;
    }
    /// A new frequency under a running output: the prescaler may change
    /// (immediate) and CMP0 is buffered.
    static bool set_hz(uint32_t hz) {
        const TcaTiming w = timing(clk_per_, hz);
        if (w.ticks == 0) return false;
        hz_ = hz;
        div_ = w.clock;
        ticks_ = w.ticks;
        R::clock(w.clock);
        R::template compare_buffered<0>(static_cast<uint16_t>(w.ticks - 1));
        return true;
    }
    static void rebase(uint32_t clk_per_hz) {
        clk_per_ = clk_per_hz;
        set_hz(hz_);
    }
    /// The frequency really produced (prescaler and rounding applied) -
    /// from the values set, not read back: in FRQ mode CMP0 keeps
    /// reading the old value after a buffered change the output already
    /// follows (bench).
    static uint32_t actual_hz() {
        return clk_per_ / (2u * tca_divisor(div_) * (ticks_));
    }
    static void stop() { R::disable(); }
    static void start() { R::enable(); }

private:
    /// Half periods in CLK_PER ticks: CLK_PER / (2 hz).
    static constexpr TcaTiming timing(uint32_t clk_per_hz, uint32_t hz) {
        if (hz == 0) return {TcaClock::div1, 0};
        return tca_timing((static_cast<uint64_t>(clk_per_hz) + hz) / (2ull * hz));
    }
    static inline uint32_t clk_per_ = 0;
    static inline uint32_t hz_ = 0;
    static inline TcaClock div_ = TcaClock::div1;
    static inline uint32_t ticks_ = 1;
};

/// Heartbeat<n, port>: a period at a rate with an OVF interrupt/event at
/// each start, plus up to three pulses (WO0..2, single-slope: high
/// from BOTTOM for pulse<ch>(ticks) ticks) that mark moments inside
/// the period - the shape of a synchronized sequence (fire a one-shot
/// here, sample there, the OVF event clocks the rest). A ClockUser:
/// rebase() keeps the rate; the pulses are re-derived from their
/// microseconds. ISR body: beat() clears OVF.
template <uint8_t n, char PortLetter>
class Heartbeat {
    using R = Tca<n>;
    static_assert(tca_route_code(n, PortLetter) != 0xFF, "Heartbeat: no such TCA route");

public:
    Heartbeat() = delete;

    /// `hz` beats per second; outputs: WO mask for the pulses (their
    /// widths start at 0 = static low). False if the period fits no
    /// prescaler (below 0.36 Hz at 24 MHz).
    template <typename Clock>
    static bool init(Clock clock, uint32_t hz, uint8_t outputs = 0, bool interrupt = true) {
        static_assert(clock_follows<Clock, Heartbeat>(),
                      "Heartbeat on a DynamicClock that does not list it: the rate would drift");
        clk_per_ = clock_hz(clock);
        hz_ = hz;
        outputs_ = outputs & 0x07;
        if (!set()) return false;
        R::enable_ovf_interrupt(interrupt);
        return true;
    }
    /// Pulse on WOch for `us` microseconds from the start of each period
    /// (buffered: from the next period; clipped to the period).
    template <uint8_t ch>
    static void pulse_us(uint32_t us) {
        pulse_us_[ch] = us;
        R::template compare_buffered<ch>(ticks_for(us, R::period()));
    }
    static void rebase(uint32_t clk_per_hz) {
        clk_per_ = clk_per_hz;
        const bool irq = (R::single().INTCTRL & TCA_SINGLE_OVF_bm) != 0;
        set();
        R::enable_ovf_interrupt(irq);
    }
    [[gnu::always_inline]] static void beat() { R::clear_ovf(); }
    static uint32_t tick_hz() { return clk_per_ / tca_divisor(div_); }

private:
    static bool set() {
        const TcaTiming w = tca_timing(tca_period_ticks(clk_per_, hz_));
        if (w.ticks == 0) return false;
        div_ = w.clock;
        const uint16_t top = static_cast<uint16_t>(w.ticks - 1);
        R::init({.mode = TcaMode::single_slope, .clock = w.clock, .period = top,
                 .compare0 = ticks_for(pulse_us_[0], top), .compare1 = ticks_for(pulse_us_[1], top),
                 .compare2 = ticks_for(pulse_us_[2], top),
                 .outputs = outputs_, .route = PortLetter});
        return true;
    }
    /// A pulse width in ticks of the current prescaler, clipped to TOP + 1
    /// (> TOP = static high for the whole period).
    static uint16_t ticks_for(uint32_t us, uint16_t top) {
        const uint64_t t = (static_cast<uint64_t>(tick_hz()) * us + 999'999u) / 1'000'000u;
        return t > top ? static_cast<uint16_t>(top + 1) : static_cast<uint16_t>(t);
    }
    static inline uint32_t clk_per_ = 0;
    static inline uint32_t hz_ = 0;
    static inline uint8_t outputs_ = 0;
    static inline TcaClock div_ = TcaClock::div1;
    static inline uint32_t pulse_us_[3] = {0, 0, 0};
};

/// EventCounter<n>: the TCA counting events on input A - positive
/// edges by default (any edge, or the clock while high, as options) -
/// 16 bits, 0..65535 then OVF (overflowed() reads and clears it). With
/// direction_on(channel) input B's LEVEL sets the direction (down while
/// high): an up/down counter, the shape of a quadrature decode with a
/// CCL deriving direction (AN2434). ISR body: ovf() on wrap.
template <uint8_t n>
class EventCounter {
    using R = Tca<n>;

public:
    EventCounter() = delete;

    template <uint8_t ch>
    static void init(EventChannel<ch> source, TcaEventA action = TcaEventA::count_posedge) {
        R::init({.mode = TcaMode::normal, .clock = TcaClock::div1, .period = 0xFFFF,
                 .event_a = action});
        R::event_a_on(source);
    }
    /// Input B level = direction (down while high).
    template <uint8_t ch>
    static void direction_on(EventChannel<ch> c) {
        R::single().EVCTRL |= static_cast<uint8_t>(TCA_SINGLE_EVACTB_UPDOWN_gc | TCA_SINGLE_CNTBEI_bm);
        R::event_b_on(c);
    }
    static uint16_t count() { return R::count(); }
    static void reset() { R::count(0); R::clear_ovf(); }
    static bool overflowed() {
        const bool o = R::ovf_flag();
        if (o) R::clear_ovf();
        return o;
    }
    [[gnu::always_inline]] static void ovf() { R::clear_ovf(); }
};

static_assert(ClockUser<FrequencyGenerator<0, 'A'>>);
static_assert(ClockUser<Heartbeat<0, 'A'>>);

} // namespace brio
