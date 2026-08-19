/*
 * ccl.hpp
 *
 * The AVR DA/DB configurable custom logic (CCL, DS40002247B ch. 31) in
 * two strata, as docs/avrdx/ccl.md describes:
 *
 *  RESOURCES - Ccl: the block (enable/disable, RUNSTDBY, the three
 *  sequencers, the one interrupt vector with its per-LUT flags); Lut<n>:
 *  one look-up table - a config struct owns its three inputs, the truth
 *  table, filter, edge detector, clock, output pin, interrupt sense;
 *  init<cfg>() / init(cfg) write it, the EVENTA/EVENTB users and the
 *  LUTn output generator are its event vocabulary (evsys.hpp).
 *
 *  TASK - ToggleFlipFlop<pair>: a JK flip-flop on a LUT pair toggled
 *  by one event (a divide-by-two / a latch a timer sets and resets
 *  with its own events, no CPU); more tasks come with their users.
 *
 * Facts that shape the code (31.3, errata DS80000915F 2.4.1):
 *  - every LUT register but ENABLE is enable-protected: written while
 *    the LUT is disabled, or together with ENABLE = 1 in the same write
 *    (init does the latter: CTRLB, CTRLC, TRUTH first, then CTRLA with
 *    ENABLE). On silicon A4/A5 (2.4.1) reconfiguring ANY LUT wants the
 *    whole CCL disabled, which drops every other LUT meanwhile - so the
 *    protocol is: Ccl::disable(); Ccl::sequencer<pair>(...) (BEFORE the
 *    even LUT's init: SEQSEL is protected by that LUT's ENABLE - bench:
 *    written after, silently ignored); Lut<..>::init(...) for each;
 *    Ccl::enable(). A LUT reconfigured
 *    under a running block is a glitch on all of them, by design of
 *    the silicon, not of this driver;
 *  - inputs: the same menu for the three inputs, but the peripheral
 *    entries select the INSTANCE by input index - input 0/1/2 of a LUT
 *    sees AC0/1/2, TCB0/1/2 WO, TCA WO0/1/2, USART0/1/2 TXD, ZCD0/1/2,
 *    SPI0 MOSI/MOSI/SCK, TCD0 WOA/WOB/WOC; `in`/`pin` is the LUT's own
 *    pin IN0/1/2 (pins 0/1/2 of its port), `link` is LUT[n+1]'s output
 *    (LUT0 into the last), `feedback` the pair's sequencer output,
 *    `event_a`/`event_b` the two EVSYS users (raw, no detection);
 *  - TRUTHn[k] is the output for input pattern k (IN2 the MSB); an
 *    unused input is `mask` (reads 0): build the table with lut_truth()
 *    from a lambda of three bools;
 *  - the filter/edge/sequencer clock is CLK_PER by default, or the
 *    LUT's own input 2 (then input 2 reads 0 in the truth table), OSCHF,
 *    OSC32K, OSC1K; SYNCH delays two clocks, FILTER needs the input
 *    stable over two clocks and delays four; the edge detector REQUIRES
 *    a filter option and yields a one-clock pulse on a rising edge;
 *  - the sequencer of a pair is clocked by the EVEN LUT's clock, takes
 *    the even LUT as D/J/D/S and the odd as G/K/G/R, and its output IS
 *    the even LUT's output (and FEEDBACK for both); the odd LUT's own
 *    output stays available; disabling the even LUT clears it;
 *  - pins (PORTMUX.CCLROUTEA moves only OUT): IN0/1/2 = pins 0/1/2 and
 *    OUT = pin 3 (ALT1 pin 6) of PORTA/C/D/F/B/G for LUT0/1/2/3/4/5
 *    (LUT3 has no ALT1); the driver sets OUT as an output when asked;
 *  - one vector (CCL_CCL_vect) for all LUTs, per-LUT sense (rising /
 *    falling / both) on the LUT output; the LUTn output is an EVSYS
 *    level generator (0x10 + n).
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"

namespace brio {

// ---- the knobs (31.5) -------------------------------------------------------------

/// INSELx: one of the three inputs of a LUT. The instance the
/// peripheral entries mean depends on WHICH input (0/1/2) they sit on.
enum class LutInput : uint8_t {
    mask = 0x0,        ///< tied low
    feedback = 0x1,    ///< the pair's sequencer output
    link = 0x2,        ///< LUT[n+1] output (LUT0 takes the last LUT)
    event_a = 0x3,     ///< EVSYS user LUTnA
    event_b = 0x4,     ///< EVSYS user LUTnB
    pin = 0x5,         ///< the LUT's own pin INx
    ac = 0x6,          ///< ACx OUT (x = input index)
    zcd = 0x7,         ///< ZCDx OUT
    usart_txd = 0x8,   ///< USARTx TXD (host modes)
    spi0 = 0x9,        ///< SPI0 MOSI (inputs 0/1), SCK (input 2); host mode
    tca0 = 0xA,        ///< TCA0 WOx
    tca1 = 0xB,        ///< TCA1 WOx
    tcb = 0xC,         ///< TCBx WO
    tcd0 = 0xD,        ///< TCD0 WOA/WOB/WOC
};

enum class LutFilter : uint8_t {
    none = CCL_FILTSEL_DISABLE_gc,
    sync = CCL_FILTSEL_SYNCH_gc,       ///< two-clock synchronizer
    filter = CCL_FILTSEL_FILTER_gc,    ///< stable for > 2 clocks, four-clock delay
};

enum class LutClock : uint8_t {
    clk_per = CCL_CLKSRC_CLKPER_gc,
    input2 = CCL_CLKSRC_IN2_gc,        ///< the LUT's input 2 (then 0 in the truth table)
    oschf = CCL_CLKSRC_OSCHF_gc,       ///< before the prescaler
    osc32k = CCL_CLKSRC_OSC32K_gc,
    osc1k = CCL_CLKSRC_OSC1K_gc,
};

/// Interrupt sense on the LUT output (INTMODEn).
enum class LutSense : uint8_t { none = 0, rising = 1, falling = 2, both = 3 };

enum class Sequencer : uint8_t {
    none = CCL_SEQSEL_DISABLE_gc,
    d_flip_flop = CCL_SEQSEL_DFF_gc,    ///< D = even LUT, G = odd (transparent while G)
    jk_flip_flop = CCL_SEQSEL_JK_gc,    ///< J = even, K = odd (1/1 toggles)
    d_latch = CCL_SEQSEL_LATCH_gc,      ///< D = even, G = odd
    rs_latch = CCL_SEQSEL_RS_gc,        ///< S = even, R = odd (1/1 forbidden)
};

struct LutConfig {
    LutInput in0 = LutInput::mask;
    LutInput in1 = LutInput::mask;
    LutInput in2 = LutInput::mask;
    uint8_t truth = 0;                 ///< TRUTHn: bit k = output for input pattern k (in2 MSB)
    LutFilter filter = LutFilter::none;
    bool edge_detect = false;          ///< needs a filter option
    LutClock clock = LutClock::clk_per;
    bool output_pin = false;           ///< OUTEN: LUTn-OUT on the pin (driven as output)
    bool alt_pin = false;              ///< PORTMUX: OUT on pin 6 instead of 3
    LutSense interrupt = LutSense::none;
};

/// The truth table from a predicate of the three inputs:
///   lut_truth([](bool a, bool b, bool c) { return a && !b; })
template <typename F>
constexpr uint8_t lut_truth(F f) {
    uint8_t t = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        if (f((k & 1) != 0, (k & 2) != 0, (k & 4) != 0)) t |= static_cast<uint8_t>(1u << k);
    }
    return t;
}

constexpr bool lut_config_valid(const LutConfig& c) {
    if (c.edge_detect && c.filter == LutFilter::none) return false;   // 31.3.1.6
    return true;
}

/// The port letter of LUT n's pins: A, C, D, F, B, G.
constexpr char lut_port(uint8_t n) {
    constexpr char p[] = {'A', 'C', 'D', 'F', 'B', 'G'};
    return p[n];
}

// ---- the block ------------------------------------------------------------------

struct Ccl {
    Ccl() = delete;

    /// Enable the block (after every LUT and sequencer is configured).
    static void enable(bool run_standby = false) {
        CCL.CTRLA = static_cast<uint8_t>(CCL_ENABLE_bm | (run_standby ? CCL_RUNSTDBY_bm : 0));
    }
    /// Disable the whole block: the state every reconfiguration starts
    /// from (errata 2.4.1). All LUT outputs drop.
    static void disable() { CCL.CTRLA = 0; }
    static bool enabled() { return (CCL.CTRLA & CCL_ENABLE_bm) != 0; }

    /// The sequencer of pair p (LUT 2p / 2p+1). Enable-protected by the
    /// even LUT: written while LUT 2p is disabled, BEFORE its init()
    /// (bench: written after, it is silently ignored). Disables the
    /// even LUT first, to be safe.
    template <uint8_t pair>
    static void sequencer(Sequencer s) {
        static_assert(pair <= 2, "LUT pairs: 0 (LUT0/1), 1 (LUT2/3), 2 (LUT4/5)");
        (&CCL.LUT0CTRLA)[8 * pair] &= static_cast<uint8_t>(~CCL_ENABLE_bm);   // the even LUT off
        (&CCL.SEQCTRL0)[pair] = static_cast<uint8_t>(s);
    }

    /// ISR body for CCL_CCL_vect: the LUTs whose sense fired (bit n),
    /// cleared.
    [[gnu::always_inline]] static uint8_t take_flags() {
        const uint8_t f = CCL.INTFLAGS;
        CCL.INTFLAGS = f;
        return f;
    }
};

// ---- one look-up table ------------------------------------------------------------

template <uint8_t n>
class Lut {
    static_assert(n <= 5, "CCL: LUT0..LUT5 (LUT4/5 on 48/64-pin parts)");

public:
    Lut() = delete;

    static constexpr uint8_t index = n;
    static constexpr char port = lut_port(n);

    using In0 = Pin<port, 0>;
    using In1 = Pin<port, 1>;
    using In2 = Pin<port, 2>;
    using OutDefault = Pin<port, 3>;
    using OutAlt = Pin<port, 6>;

    using OutEvent = EvLut<n>;            ///< generator: the LUT output level
    using EventA = EvLutIn<n, 'A'>;       ///< users: the two event inputs
    using EventB = EvLutIn<n, 'B'>;

    /// Compile-time form.
    template <LutConfig cfg>
    static void init() {
        static_assert(lut_config_valid(cfg), "LutConfig: the edge detector needs a filter option (sync or filter)");
        init(cfg);
    }

    /// Run-time form. Call with the block DISABLED (Ccl::disable()):
    /// writes inputs, truth, route, output pin, interrupt sense, then
    /// CTRLA with ENABLE = 1 (the enable-protected fields land together
    /// with the enable). False, touching nothing, for an invalid config.
    static bool init(const LutConfig& cfg) {
        if (!lut_config_valid(cfg)) return false;
        ctrla() = 0;                                          // disabled: the protected registers open
        ctrlb() = static_cast<uint8_t>(static_cast<uint8_t>(cfg.in0) | (static_cast<uint8_t>(cfg.in1) << 4));
        ctrlc() = static_cast<uint8_t>(cfg.in2);
        truth() = cfg.truth;
        route(cfg.alt_pin);
        if (cfg.output_pin) {
            if (cfg.alt_pin) OutAlt::output(); else OutDefault::output();
        }
        sense(cfg.interrupt);
        ctrla() = static_cast<uint8_t>(
            CCL_ENABLE_bm |
            (cfg.edge_detect ? CCL_EDGEDET_bm : 0) |
            (cfg.output_pin ? CCL_OUTEN_bm : 0) |
            static_cast<uint8_t>(cfg.filter) |
            static_cast<uint8_t>(cfg.clock));
        return true;
    }

    static void enable() { ctrla() |= CCL_ENABLE_bm; }
    /// Disable this LUT alone (its filter/edge logic clears one clock
    /// later; the pair's sequencer clears if this is the even one).
    static void disable() { ctrla() &= static_cast<uint8_t>(~CCL_ENABLE_bm); }

    /// The interrupt sense on this LUT's output (INTCTRL0/1).
    static void sense(LutSense s) {
        volatile uint8_t& r = n < 4 ? CCL.INTCTRL0 : CCL.INTCTRL1;
        const uint8_t shift = static_cast<uint8_t>(2 * (n % 4));
        r = static_cast<uint8_t>((r & ~(0x03u << shift)) | (static_cast<uint8_t>(s) << shift));
    }
    static bool flag() { return (CCL.INTFLAGS & (1u << n)) != 0; }
    static void clear_flag() { CCL.INTFLAGS = static_cast<uint8_t>(1u << n); }

    /// Event input A / B from channel ch (the LUT must select event_a /
    /// event_b on one of its inputs).
    template <uint8_t ch>
    static void event_a_on(EventChannel<ch> c) { EventA::listen(c); }
    template <uint8_t ch>
    static void event_b_on(EventChannel<ch> c) { EventB::listen(c); }

    static volatile uint8_t& ctrla() { return (&CCL.LUT0CTRLA)[4 * n]; }
    static volatile uint8_t& ctrlb() { return (&CCL.LUT0CTRLB)[4 * n]; }
    static volatile uint8_t& ctrlc() { return (&CCL.LUT0CTRLC)[4 * n]; }
    static volatile uint8_t& truth() { return (&CCL.TRUTH0)[4 * n]; }

private:
    static void route(bool alt) {
        constexpr uint8_t bit = static_cast<uint8_t>(1u << n);
        if (alt) PORTMUX.CCLROUTEA |= bit; else PORTMUX.CCLROUTEA &= static_cast<uint8_t>(~bit);
    }
};

// ---- tasks ----------------------------------------------------------------------

/// ToggleFlipFlop<pair>: a JK flip-flop on LUT pair p (LUT 2p / 2p+1)
/// whose J and K are both the event on the even LUT's event input A -
/// it toggles once per event pulse (a one-CLK_PER pulse seen by the
/// CLK_PER-clocked flip-flop: exactly one toggle). A divide-by-two of
/// any pulse generator with no CPU: a timer's CAPT/OVF event becomes a
/// square wave at half its rate on the even LUT's pin and event.
/// Configures both LUTs and the sequencer; the caller brackets it with
/// Ccl::disable() / Ccl::enable() together with its other LUTs.
template <uint8_t pair>
struct ToggleFlipFlop {
    ToggleFlipFlop() = delete;
    static_assert(pair <= 2, "LUT pairs: 0, 1, 2");

    using Even = Lut<2 * pair>;
    using Odd = Lut<2 * pair + 1>;

    /// The toggle event from channel ch; output on the even LUT's pin if
    /// asked (its OutEvent is always available).
    template <uint8_t ch>
    static void init(EventChannel<ch> toggle, bool output_pin = false, bool alt_pin = false) {
        Even::disable();                                   // SEQSEL is enable-protected by the even LUT
        Ccl::sequencer<pair>(Sequencer::jk_flip_flop);
        Even::init({.in0 = LutInput::event_a, .truth = lut_truth([](bool a, bool, bool) { return a; }),
                    .output_pin = output_pin, .alt_pin = alt_pin});
        Odd::init({.in0 = LutInput::event_a, .truth = lut_truth([](bool a, bool, bool) { return a; })});
        Even::event_a_on(toggle);
        Odd::event_a_on(toggle);
    }
};

} // namespace brio
