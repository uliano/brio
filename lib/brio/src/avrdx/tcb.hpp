/*
 * tcb.hpp
 *
 * The AVR DA/DB 16-bit timer/counter type B (TCB, DS40002247B ch. 24)
 * in two strata, as docs/avrdx/tcb.md describes:
 *
 *  RESOURCE - Tcb<n>: the typed view of one instance. A config struct
 *  owns the whole configuration (mode, clock, compare, event input and
 *  its edge, filter, async, cascade, sync-update, output pin and its
 *  route, run-standby); init<cfg>() folds it, init(cfg) computes it at
 *  run time. Then the verbs the modes share: count/compare/capture,
 *  running, the two flags, the two interrupt enables, the ISR body
 *  (take_flags: ONE vector per instance, CAPT and OVF share it), the
 *  event hooks capture_on / count_on over evsys.hpp.
 *
 *  TASKS - what an application names, each a mode with its knobs
 *  expressed in the application's units (a clock and microseconds or
 *  hertz; the resource speaks ticks):
 *    PeriodicTick<Tcb>     INT: a CAPT interrupt at a rate (ClockUser)
 *    Timeout<Tcb>          TIMEOUT: "the next edge came too late"
 *    OneShotPulse<Tcb>     SINGLE: a pulse of a width on WO at an event
 *    PulseCounter<Tcb>     CAPT mode, clock = event: count edges
 *    CascadedCounter<L, M> two TCBs, 32-bit count with a coherent capture
 *    FrequencyMeter<Tcb>   FRQ: period between equal edges
 *    PulseWidthMeter<Tcb>  PW: time between opposite edges
 *    DutyMeter<Tcb>        FRQPW: period AND width in one sequence
 *    Pwm8<Tcb, period>     PWM8: an 8-bit PwmChannel on WO
 *  A task owns its instance; two tasks on one Tcb<n> is the app's bug.
 *
 * Facts that shape the code (24.3, 24.5, errata DS80000915F 2.13.x):
 *  - the counter clock is CLK_PER, CLK_PER/2, the prescaled clock of a
 *    TCA (CLK_TCA: in step with that TCA, SYNCUPD restarts with it) or
 *    the positive edges of an event channel (the COUNT user); the
 *    capture/start/stop edges come from the CAPT user, enabled by
 *    CAPTEI, and EDGE says which edge does what - per mode (24.5.3);
 *  - reading CCMP clears CAPT (on the low byte): capture() in the ISR
 *    IS the acknowledge; in FRQPW read CNT (period) BEFORE CCMP
 *    (width), the read of CCMP re-arms the sequence. The 16-bit
 *    accesses go through the instance's one TEMP register: reads of
 *    CNT/CCMP from the main context while the ISR reads them too want
 *    a critical section (the task bodies below run in the ISR);
 *  - modes must not change while enabled: init() starts from CTRLA =
 *    0 and clears the flags after configuring (a flag may set during
 *    configuration);
 *  - single-shot: WO low when idle, high while counting; enabling
 *    starts a spurious pulse unless CNT holds TOP - init() writes it;
 *    a new event during the pulse is ignored; after TOP, WO stays low
 *    at least one CLK_TCB and a new event is honoured two CLK_PER
 *    later; ASYNC drives WO high on the event itself (the counter
 *    starts 2-3 CLK_TCB later) and accepts events shorter than CLK_PER;
 *  - 32-bit cascade: the LSB counts the source (CASCADE = 0), the MSB
 *    counts the LSB's OVF event (CLKSEL = EVENT, CASCADE = 1: its CAPT
 *    input is delayed one CLK_PER to absorb the carry), both in the
 *    same capture mode, one CAPT event to both;
 *  - errata 2.13.1 (A4/A5): in PWM8 CCMP and CNT are 16-bit only -
 *    Pwm8 writes CCMP as one word (period and duty together);
 *  - routes (PORTMUX.TCBROUTEA, one bit per instance): TCB0 PA2 /
 *    PF4, TCB1 PA3 / PF5, TCB2 PC0 / PB4, TCB3 PB5 / PC1. On the bench
 *    board PF4/PF5 are the console: TCB0/TCB1 output on PA2/PA3.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <type_traits>

#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// ---- the knobs (24.5.1 - 24.5.3) ---------------------------------------------------

/// CNTMODE, in the datasheet's order (the values ARE the codes).
enum class TcbMode : uint8_t {
    periodic = TCB_CNTMODE_INT_gc,               ///< count 0..TOP, CAPT at TOP, restart
    timeout = TCB_CNTMODE_TIMEOUT_gc,            ///< start on one edge, stop on the next, CAPT if TOP first
    capture = TCB_CNTMODE_CAPT_gc,               ///< free-running 0..MAX, CNT -> CCMP on the edge
    frequency = TCB_CNTMODE_FRQ_gc,              ///< capture and restart on the edge: period
    pulse_width = TCB_CNTMODE_PW_gc,             ///< restart on one edge, capture on the other: width
    frequency_pulse_width = TCB_CNTMODE_FRQPW_gc,///< start, capture (width), stop (period in CNT)
    single_shot = TCB_CNTMODE_SINGLE_gc,         ///< a pulse of CCMP ticks on WO at the edge
    pwm8 = TCB_CNTMODE_PWM8_gc,                  ///< period CCMPL + 1, high CCMPH
};

/// CLKSEL: the counter clock.
enum class TcbClock : uint8_t {
    div1 = TCB_CLKSEL_DIV1_gc,     ///< CLK_PER
    div2 = TCB_CLKSEL_DIV2_gc,     ///< CLK_PER / 2
    tca0 = TCB_CLKSEL_TCA0_gc,     ///< CLK_TCA of TCA0 (its prescaled clock)
    tca1 = TCB_CLKSEL_TCA1_gc,     ///< CLK_TCA of TCA1
    event = TCB_CLKSEL_EVENT_gc,   ///< positive edges of the COUNT event input
};

struct TcbConfig {
    TcbMode mode = TcbMode::periodic;
    TcbClock clock = TcbClock::div1;
    uint16_t compare = 0xFFFF;     ///< CCMP: TOP / pulse width / (PWM8) duty << 8 | period - 1
    bool event_input = false;      ///< CAPTEI: the CAPT event acts (mode-dependent)
    bool edge = false;             ///< EDGE: which edge does what, per mode (24.5.3)
    bool filter = false;           ///< noise canceler on the event input (4 CLK_PER samples)
    bool async = false;            ///< single-shot: WO high on the event itself
    bool cascade = false;          ///< the MSB half of a 32-bit pair (CAPT input delayed 1 CLK_PER)
    bool sync_update = false;      ///< restart when the clocking TCA (or TCA0) restarts
    bool output = false;           ///< CCMPEN: WO overrides the pin (driven as output by init)
    bool output_init_high = false; ///< CCMPINIT: WO idle level in the modes without one of their own
    bool alt_pin = false;          ///< PORTMUX: WO on the ALT1 position
    bool run_standby = false;
    bool debug_run = false;        ///< keep counting while the CPU is halted by the debugger
};

/// The two flags of one instance, as take_flags() returns them.
struct TcbFlags {
    bool capt;
    bool ovf;
};

/// Ticks per second of the counter for a peripheral clock, for the two
/// CLK_PER choices; 0 when the rate is not the TCB's to know (a TCA's
/// prescaled clock, an event).
constexpr uint32_t tcb_tick_hz(uint32_t clk_per_hz, TcbClock c) {
    return c == TcbClock::div1 ? clk_per_hz : c == TcbClock::div2 ? clk_per_hz / 2 : 0;
}

/// Ticks for a duration in microseconds at a tick rate ("at least":
/// rounds up); 65536 ticks is the most a TCB counts (TOP = 0xFFFF),
/// anything longer saturates at 65537 so callers can refuse.
constexpr uint32_t tcb_ticks_for_us(uint32_t tick_hz, uint32_t us) {
    const uint64_t t = (static_cast<uint64_t>(tick_hz) * us + 999'999u) / 1'000'000u;
    return t > 0x10000u ? 0x10001u : static_cast<uint32_t>(t);
}

// ---- the resource ---------------------------------------------------------------

template <uint8_t n>
class Tcb {
#ifdef TCB3
    static_assert(n <= 3, "AVR DA/DB 48-pin: TCB0..TCB3 (TCB4 on 64 pins only)");
#else
    static_assert(n <= 2, "AVR DA/DB 28/32-pin: TCB0..TCB2");
#endif

public:
    Tcb() = delete;

    static constexpr uint8_t index = n;

    /// The waveform output pin in the two PORTMUX positions.
    using OutDefault =
        std::conditional_t<n == 0, Pin<'A', 2>,
        std::conditional_t<n == 1, Pin<'A', 3>,
        std::conditional_t<n == 2, Pin<'C', 0>, Pin<'B', 5>>>>;
    using OutAlt =
        std::conditional_t<n == 0, Pin<'F', 4>,
        std::conditional_t<n == 1, Pin<'F', 5>,
        std::conditional_t<n == 2, Pin<'B', 4>, Pin<'C', 1>>>>;

    /// The event vocabulary of this instance (evsys.hpp).
    using CaptEvent = EvTcbCapt<n>;      ///< generator: CAPT flag set
    using OvfEvent = EvTcbOvf<n>;        ///< generator: overflow (the carry)
    using CaptIn = EvTcbCaptIn<n>;       ///< user: the capture/start/stop edge
    using CountIn = EvTcbCountIn<n>;     ///< user: the counter clock (CLKSEL = event)

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: every register value folded.
    template <TcbConfig cfg>
    static void init() { init(cfg); }

    /// Run-time form. Disables, routes and drives the output pin if asked,
    /// writes mode/event/compare, parks CNT (at TOP for single-shot: no
    /// spurious pulse on enable), clears the flags, enables with the
    /// clock. Interrupts stay off: the task or the app enables them.
    static void init(const TcbConfig& cfg) {
        auto& t = regs();
        t.CTRLA = 0;                                   // never change mode while enabled
        t.INTCTRL = 0;
        route(cfg.alt_pin);
        if (cfg.output) {
            if (cfg.alt_pin) OutAlt::output(); else OutDefault::output();
        }
        t.CTRLB = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.mode) |
            (cfg.output ? TCB_CCMPEN_bm : 0) |
            (cfg.output_init_high ? TCB_CCMPINIT_bm : 0) |
            (cfg.async ? TCB_ASYNC_bm : 0));
        t.EVCTRL = static_cast<uint8_t>(
            (cfg.event_input ? TCB_CAPTEI_bm : 0) |
            (cfg.edge ? TCB_EDGE_bm : 0) |
            (cfg.filter ? TCB_FILTER_bm : 0));
        t.CCMP = cfg.compare;
        t.CNT = cfg.mode == TcbMode::single_shot ? cfg.compare : 0;
        t.DBGCTRL = cfg.debug_run ? TCB_DBGRUN_bm : 0;
        t.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
        t.CTRLA = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.clock) |
            (cfg.cascade ? TCB_CASCADE_bm : 0) |
            (cfg.sync_update ? TCB_SYNCUPD_bm : 0) |
            (cfg.run_standby ? TCB_RUNSTDBY_bm : 0) |
            TCB_ENABLE_bm);
    }

    static void enable() { regs().CTRLA |= TCB_ENABLE_bm; }
    static void disable() { regs().CTRLA &= static_cast<uint8_t>(~TCB_ENABLE_bm); }
    static bool enabled() { return (regs().CTRLA & TCB_ENABLE_bm) != 0; }

    // ---- the counter ------------------------------------------------------

    static uint16_t count() { return regs().CNT; }
    static void count(uint16_t v) { regs().CNT = v; }
    /// CCMP as a compare value (TOP, pulse width, PWM word).
    static uint16_t compare() { return regs().CCMP; }
    static void compare(uint16_t v) { regs().CCMP = v; }
    /// CCMP as a capture: the read clears CAPT (24.5.5).
    [[gnu::always_inline]] static uint16_t capture() { return regs().CCMP; }
    /// STATUS.RUN: counting (single-shot: the pulse is in progress).
    static bool running() { return (regs().STATUS & TCB_RUN_bm) != 0; }

    // ---- flags and interrupts ---------------------------------------------

    static bool capt_flag() { return (regs().INTFLAGS & TCB_CAPT_bm) != 0; }
    static bool ovf_flag() { return (regs().INTFLAGS & TCB_OVF_bm) != 0; }
    static void clear_capt() { regs().INTFLAGS = TCB_CAPT_bm; }
    static void clear_ovf() { regs().INTFLAGS = TCB_OVF_bm; }

    static void enable_capt_interrupt(bool on) { irq(TCB_CAPT_bm, on); }
    static void enable_ovf_interrupt(bool on) { irq(TCB_OVF_bm, on); }

    /// ISR body for TCBn_INT_vect: which flags were up, both cleared.
    /// (In the capture modes a capture() before this clears CAPT by
    /// itself; clearing again is harmless.)
    [[gnu::always_inline]] static TcbFlags take_flags() {
        const uint8_t f = regs().INTFLAGS;
        regs().INTFLAGS = f;
        return {(f & TCB_CAPT_bm) != 0, (f & TCB_OVF_bm) != 0};
    }

    // ---- events -----------------------------------------------------------

    /// The CAPT event (capture / start / stop / trigger per the mode)
    /// comes from channel ch; enables CAPTEI.
    template <uint8_t ch>
    static void capture_on(EventChannel<ch> c) {
        CaptIn::listen(c);
        regs().EVCTRL |= TCB_CAPTEI_bm;
    }
    static void capture_on_events(bool on) {
        if (on) regs().EVCTRL |= TCB_CAPTEI_bm;
        else regs().EVCTRL &= static_cast<uint8_t>(~TCB_CAPTEI_bm);
    }
    /// The counter clock is the positive edge of channel ch's event:
    /// only meaningful with `clock = TcbClock::event`.
    template <uint8_t ch>
    static void count_on(EventChannel<ch> c) { CountIn::listen(c); }

    static constexpr TCB_t& regs() {
        if constexpr (n == 0) return TCB0;
        else if constexpr (n == 1) return TCB1;
        else if constexpr (n == 2) return TCB2;
#ifdef TCB3
        else return TCB3;
#endif
    }

private:
    static void irq(uint8_t bit, bool on) {
        if (on) regs().INTCTRL |= bit; else regs().INTCTRL &= static_cast<uint8_t>(~bit);
    }
    static void route(bool alt) {
        constexpr uint8_t bit = static_cast<uint8_t>(1u << n);
        if (alt) PORTMUX.TCBROUTEA |= bit; else PORTMUX.TCBROUTEA &= static_cast<uint8_t>(~bit);
    }
};

// ---- tasks ----------------------------------------------------------------------

/// Picks div1 when the duration fits 16 bits at CLK_PER, div2 when it
/// fits at CLK_PER/2; returns {clock, ticks} with ticks == 0 when it
/// fits neither (the caller refuses).
struct TcbTiming {
    TcbClock clock;
    uint32_t ticks;
};
constexpr TcbTiming tcb_timing_for_us(uint32_t clk_per_hz, uint32_t us) {
    const uint32_t t1 = tcb_ticks_for_us(clk_per_hz, us);
    if (t1 >= 1 && t1 <= 0x10000u) return {TcbClock::div1, t1};
    const uint32_t t2 = tcb_ticks_for_us(clk_per_hz / 2, us);
    if (t2 >= 1 && t2 <= 0x10000u) return {TcbClock::div2, t2};
    return {TcbClock::div1, 0};
}
constexpr TcbTiming tcb_timing_for_hz(uint32_t clk_per_hz, uint32_t hz) {
    if (hz == 0) return {TcbClock::div1, 0};
    const uint32_t t1 = (clk_per_hz + hz / 2) / hz;
    if (t1 >= 1 && t1 <= 0x10000u) return {TcbClock::div1, t1};
    const uint32_t t2 = (clk_per_hz / 2 + hz / 2) / hz;
    if (t2 >= 1 && t2 <= 0x10000u) return {TcbClock::div2, t2};
    return {TcbClock::div1, 0};
}

/// PeriodicTick<Tcb>: a CAPT interrupt (and event) at a rate - the TCB
/// as the timebase of something that is not the kernel's Ticker (a
/// sampling pace, a heartbeat). Periodic mode, TOP = ticks - 1. A
/// ClockUser: rebase() keeps the rate on a clock change (the phase
/// restarts). ISR body: tick() clears the flag.
template <typename T>
struct PeriodicTick {
    PeriodicTick() = delete;

    /// At `hz` interrupts per second (rounded to the nearest tick count);
    /// false if it does not fit 16 bits at CLK_PER/2 (then use the RTC).
    template <typename Clock>
    static bool init(Clock clock, uint32_t hz, bool interrupt = true) {
        static_assert(clock_follows<Clock, PeriodicTick>(),
                      "PeriodicTick on a DynamicClock that does not list it: the rate would drift");
        hz_ = hz;
        return set(clock_hz(clock), interrupt);
    }
    /// Every `us` microseconds (rounded up: "at least").
    template <typename Clock>
    static bool init_us(Clock clock, uint32_t us, bool interrupt = true) {
        static_assert(clock_follows<Clock, PeriodicTick>(),
                      "PeriodicTick on a DynamicClock that does not list it: the rate would drift");
        us_ = us;
        hz_ = 0;
        return set(clock_hz(clock), interrupt);
    }
    static void rebase(uint32_t clk_per_hz) { set(clk_per_hz, (T::regs().INTCTRL & TCB_CAPT_bm) != 0); }

    /// ISR body for TCBn_INT_vect.
    [[gnu::always_inline]] static void tick() { T::clear_capt(); }

    static void stop() { T::disable(); }
    static void start() { T::count(0); T::enable(); }

private:
    static bool set(uint32_t clk_per_hz, bool interrupt) {
        const TcbTiming w = hz_ ? tcb_timing_for_hz(clk_per_hz, hz_) : tcb_timing_for_us(clk_per_hz, us_);
        if (w.ticks == 0) return false;
        T::init({.mode = TcbMode::periodic, .clock = w.clock,
                 .compare = static_cast<uint16_t>(w.ticks - 1)});
        T::enable_capt_interrupt(interrupt);
        return true;
    }
    static inline uint32_t hz_ = 0;
    static inline uint32_t us_ = 0;
};

/// Timeout<Tcb>: watches an event - the counter starts on one edge and
/// stops on the next; if `us` elapse first, CAPT fires (interrupt and
/// event). A watchdog on a signal's activity, a "button held longer
/// than" detector (edge = false: start on the rising edge, stop on the
/// falling - CAPT = held longer than us; edge = true: the reverse).
/// After the stop edge the counter freezes until the next start edge.
/// ISR body: expired() clears CAPT.
template <typename T>
struct Timeout {
    Timeout() = delete;

    template <typename Clock, uint8_t ch>
    static bool init(Clock clock, uint32_t us, EventChannel<ch> c,
                     bool edge = false, bool filter = false) {
        const TcbTiming w = tcb_timing_for_us(clock_hz(clock), us);
        if (w.ticks == 0) return false;
        T::init({.mode = TcbMode::timeout, .clock = w.clock,
                 .compare = static_cast<uint16_t>(w.ticks - 1),
                 .event_input = true, .edge = edge, .filter = filter});
        T::capture_on(c);
        T::enable_capt_interrupt(true);
        return true;
    }
    [[gnu::always_inline]] static void expired() { T::clear_capt(); }
    /// Still between the start edge and the stop edge.
    static bool pending() { return T::running(); }
};

/// OneShotPulse<Tcb>: a pulse of a width on WO, started by an event
/// (any source: a pin, a comparator, another timer, a software pulse
/// on the channel - fire()). Single-shot mode. `any_edge` = both
/// edges trigger (else the positive); `async` = WO rises on the event
/// itself, not on the next CLK_TCB. The retrigger rule (a new event
/// during the pulse is ignored) is the hardware's. CAPT fires at the
/// end of the pulse: pulse_done() is the ISR body if the app wants it.
template <typename T>
struct OneShotPulse {
    OneShotPulse() = delete;

    struct Options {
        bool any_edge = false;
        bool async = false;
        bool alt_pin = false;
        bool filter = false;
    };

    /// Width in microseconds ("at least"); false if it does not fit.
    template <typename Clock, uint8_t ch>
    static bool init(Clock clock, uint32_t width_us, EventChannel<ch> trigger, Options o = {}) {
        const TcbTiming w = tcb_timing_for_us(clock_hz(clock), width_us);
        if (w.ticks == 0) return false;
        init_ticks(w.clock, static_cast<uint16_t>(w.ticks), trigger, o);
        return true;
    }
    /// Width in ticks of the chosen clock (div1/div2/a TCA's clock).
    template <uint8_t ch>
    static void init_ticks(TcbClock clock, uint16_t width_ticks, EventChannel<ch> trigger, Options o = {}) {
        channel_ = ch;
        T::init({.mode = TcbMode::single_shot, .clock = clock, .compare = width_ticks,
                 .event_input = true, .edge = o.any_edge, .filter = o.filter,
                 .async = o.async, .output = true, .alt_pin = o.alt_pin});
        T::capture_on(trigger);
    }
    /// Change the width for the NEXT pulse (takes effect at once: do it
    /// while idle).
    static void width_ticks(uint16_t t) { T::compare(t); }

    /// Software trigger: a pulse on the trigger channel (ORed with the
    /// channel's generator, so it works on a pin-sourced channel too).
    static void fire() {
        if (channel_ < 8) EVSYS.SWEVENTA = static_cast<uint8_t>(1u << channel_);
        else EVSYS.SWEVENTB = static_cast<uint8_t>(1u << (channel_ - 8));
    }
    static bool busy() { return T::running(); }
    [[gnu::always_inline]] static void pulse_done() { T::clear_capt(); }

private:
    static inline uint8_t channel_ = 0;
};

/// PulseCounter<Tcb>: counts the positive edges of an event (a pin, a
/// comparator, a timer output, the PIT) - the TCB clocked by the COUNT
/// event, free-running 0..MAX in capture mode. count() reads the live
/// value; OVF tells it wrapped (overflowed() reads and clears it);
/// snapshot_on(ch) makes another event latch CNT into CCMP (a gated
/// count: the latched value is captured(), CAPT fires - ISR body
/// captured() too). A 32-bit count is CascadedCounter.
template <typename T>
struct PulseCounter {
    PulseCounter() = delete;

    template <uint8_t ch>
    static void init(EventChannel<ch> source) {
        T::init({.mode = TcbMode::capture, .clock = TcbClock::event, .compare = 0});
        T::count_on(source);
    }
    template <uint8_t ch>
    static void snapshot_on(EventChannel<ch> c) { T::capture_on(c); }

    static uint16_t count() { return T::count(); }
    static void reset() { T::count(0); T::clear_ovf(); }
    static bool overflowed() {
        const bool o = T::ovf_flag();
        if (o) T::clear_ovf();
        return o;
    }
    /// The value latched by the last snapshot event (clears CAPT).
    [[gnu::always_inline]] static uint16_t captured() { return T::capture(); }
};

/// CascadedCounter<Lsb, Msb>: two TCBs as one 32-bit counter with a
/// coherent capture. Lsb counts the source (CLK_PER, CLK_PER/2, a TCA's
/// clock, or an event via count_on); Msb counts Lsb's OVF event, routed
/// on `carry`; both capture on `snapshot` (Msb with CASCADE: its input
/// is delayed one CLK_PER to absorb the carry). read() pulses the
/// snapshot channel and returns the latched 32 bits; captured() returns
/// them after an external snapshot event (ISR body: CAPT fires on both,
/// bind Lsb's vector). Costs two event channels.
template <typename Lsb, typename Msb>
struct CascadedCounter {
    CascadedCounter() = delete;
    static_assert(Lsb::index != Msb::index, "CascadedCounter: two different TCBs");

    template <uint8_t carry, uint8_t snapshot>
    static void init(TcbClock source, EventChannel<carry> c, EventChannel<snapshot> s) {
        snapshot_ = snapshot;
        Msb::init({.mode = TcbMode::capture, .clock = TcbClock::event, .compare = 0,
                   .event_input = true, .cascade = true});
        Lsb::init({.mode = TcbMode::capture, .clock = source, .compare = 0, .event_input = true});
        c.source(typename Lsb::OvfEvent{});
        Msb::count_on(c);
        Msb::capture_on(s);
        Lsb::capture_on(s);
    }
    /// Lsb counts the positive edges of channel ch's event (source = event).
    template <uint8_t ch>
    static void count_on(EventChannel<ch> c) { Lsb::count_on(c); }

    static void reset() {
        Lsb::disable();
        Lsb::count(0);
        Msb::count(0);
        Lsb::enable();
    }
    /// Latch and read: a software event on the snapshot channel, wait
    /// for both CAPT flags (the MSB's input is one CLK_PER behind), then
    /// the two captures. Bounded wait: a miswired pair returns stale
    /// values instead of hanging.
    static uint32_t read() {
        if (snapshot_ < 8) EVSYS.SWEVENTA = static_cast<uint8_t>(1u << snapshot_);
        else EVSYS.SWEVENTB = static_cast<uint8_t>(1u << (snapshot_ - 8));
        for (uint8_t i = 0; i < 16 && !(Lsb::capt_flag() && Msb::capt_flag()); ++i) {
        }
        return captured();
    }
    /// The latched 32 bits (clears CAPT on both).
    [[gnu::always_inline]] static uint32_t captured() {
        const uint16_t lo = Lsb::capture();
        const uint16_t hi = Msb::capture();
        return (static_cast<uint32_t>(hi) << 16) | lo;
    }

private:
    static inline uint8_t snapshot_ = 0;
};

/// The common part of the three capture meters: the clock and the
/// tick rate (for the conversions), the event wiring, the overflow.
template <typename T>
struct CaptureMeterBase {
    CaptureMeterBase() = delete;

    /// Ticks per second of the measurement (0 when clocked by a TCA).
    static uint32_t tick_hz() { return tick_hz_; }
    /// A period in ticks -> Hz (0 for 0 ticks).
    static uint32_t hz(uint16_t period_ticks) {
        return period_ticks ? tick_hz_ / period_ticks : 0;
    }
    /// A duration in ticks -> microseconds (rounded to nearest).
    static uint32_t us(uint16_t ticks) {
        return tick_hz_ ? static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000u + tick_hz_ / 2) / tick_hz_) : 0;
    }
    /// The counter wrapped since the last call: the signal is slower
    /// than 65536 ticks (reads and clears OVF).
    static bool overflowed() {
        const bool o = T::ovf_flag();
        if (o) T::clear_ovf();
        return o;
    }

protected:
    template <typename Clock, uint8_t ch>
    static void setup(Clock clock, TcbMode mode, TcbClock tclk, EventChannel<ch> c, bool edge, bool filter) {
        tick_hz_ = tcb_tick_hz(clock_hz(clock), tclk);
        T::init({.mode = mode, .clock = tclk, .compare = 0,
                 .event_input = true, .edge = edge, .filter = filter});
        T::capture_on(c);
        T::enable_capt_interrupt(true);
    }
    static inline uint32_t tick_hz_ = 0;
};

/// FrequencyMeter<Tcb>: the period of a signal, captured between two
/// equal edges (rising by default, falling with `falling`), the
/// counter restarting at each: CAPT fires per period with the period
/// in ticks. ISR body: period_ticks() (the read clears CAPT); hz()
/// converts. Range: 65535 ticks of the chosen clock (div2 doubles it;
/// OVF says "too slow"). The TCB-on-TCA clock (TcbClock::tca0/1)
/// extends the range to the TCA's prescaler - then hz()/us() do not
/// know the rate: convert with the TCA's.
template <typename T>
struct FrequencyMeter : CaptureMeterBase<T> {
    template <typename Clock, uint8_t ch>
    static void init(Clock clock, EventChannel<ch> source, TcbClock tclk = TcbClock::div1,
                     bool falling = false, bool filter = false) {
        CaptureMeterBase<T>::setup(clock, TcbMode::frequency, tclk, source, falling, filter);
    }
    [[gnu::always_inline]] static uint16_t period_ticks() { return T::capture(); }
};

/// PulseWidthMeter<Tcb>: the time a signal stays high (restart on the
/// rising edge, capture on the falling; `low` measures the low time).
/// ISR body: width_ticks().
template <typename T>
struct PulseWidthMeter : CaptureMeterBase<T> {
    template <typename Clock, uint8_t ch>
    static void init(Clock clock, EventChannel<ch> source, TcbClock tclk = TcbClock::div1,
                     bool low = false, bool filter = false) {
        CaptureMeterBase<T>::setup(clock, TcbMode::pulse_width, tclk, source, low, filter);
    }
    [[gnu::always_inline]] static uint16_t width_ticks() { return T::capture(); }
};

/// DutyMeter<Tcb>: period and high time in one sequence (start on a
/// rising edge, capture the width at the falling, stop at the next
/// rising: CAPT with CNT = period, CCMP = width, frozen until CCMP is
/// read). `inverted` = the sequence starts on a falling edge (then
/// width is the low time). ISR body: reading() - CNT first, then CCMP,
/// the read of CCMP re-arms. duty_permille() is pure arithmetic.
template <typename T>
struct DutyMeter : CaptureMeterBase<T> {
    struct Reading {
        uint16_t period_ticks;
        uint16_t width_ticks;
    };
    template <typename Clock, uint8_t ch>
    static void init(Clock clock, EventChannel<ch> source, TcbClock tclk = TcbClock::div1,
                     bool inverted = false, bool filter = false) {
        CaptureMeterBase<T>::setup(clock, TcbMode::frequency_pulse_width, tclk, source, inverted, filter);
    }
    [[gnu::always_inline]] static Reading reading() {
        const uint16_t period = T::count();      // BEFORE CCMP: its read re-arms the sequence
        const uint16_t width = T::capture();
        return {period, width};
    }
    static uint16_t duty_permille(Reading r) {
        return r.period_ticks ? static_cast<uint16_t>((static_cast<uint32_t>(r.width_ticks) * 1000u + r.period_ticks / 2) / r.period_ticks) : 0;
    }
};

/// Pwm8<Tcb, period>: the TCB as one 8-bit PWM channel on WO - a
/// PwmChannel with max = period (CCMPL = period, frequency = CLK_TCB /
/// (period + 1)). duty(v): high for v ticks of period + 1; v = 0 is a
/// clean low, v = max overrides the pin high from PORT (the hardware's
/// best is period / (period + 1)), the same endpoint policy as TcaPwm.
/// Errata 2.13.1: CCMP written as one 16-bit word.
template <typename T, uint8_t period = 255>
struct Pwm8 {
    Pwm8() = delete;
    static_assert(period >= 3, "Pwm8: period (CCMPL) >= 3");

    static constexpr uint16_t max = period;

    static void init(TcbClock clock = TcbClock::div1, bool alt_pin = false) {
        alt_ = alt_pin;
        T::init({.mode = TcbMode::pwm8, .clock = clock,
                 .compare = static_cast<uint16_t>(period), .output = true, .alt_pin = alt_pin});
    }
    static void duty(uint16_t v) {
        if (v >= max) {
            T::regs().CTRLB &= static_cast<uint8_t>(~TCB_CCMPEN_bm);
            if (alt_) T::OutAlt::set(); else T::OutDefault::set();
            return;
        }
        T::compare(static_cast<uint16_t>((v << 8) | period));   // one word: errata 2.13.1
        T::regs().CTRLB |= TCB_CCMPEN_bm;
    }

private:
    static inline bool alt_ = false;
};

static_assert(PwmChannel<Pwm8<Tcb<0>>>);
static_assert(ClockUser<PeriodicTick<Tcb<0>>>);

} // namespace brio
