/*
 * ccl.hpp
 *
 * The configurable custom logic (CCL) of the AVR DA, DB and EA: small
 * look-up tables (LUTs) that compute a logic function of three inputs -
 * pins, event channels, other peripherals' outputs - and drive a pin, an
 * event channel or an interrupt with it, with no CPU, and in sleep as
 * far as the LUT's clock runs there (docs/avrxt/ccl.md): glue logic on
 * pins, a divide-by-two of a timer's event, a one-clock pulse
 * on an edge, a filtered input that wakes the device. The chapter is
 * DS40002247B ch. 31 on the AVR DA/DB (six LUTs and three sequencers on
 * 48/64 pins, four and two on 28/32) and DS40002443A ch. 29 on the AVR
 * EA (four LUTs and two sequencers on every package, the same IP but for
 * the input menu). The header has two strata, as docs/avrxt/ccl.md
 * describes:
 *
 *  the resources - Ccl: the block (enable/disable, RUNSTDBY, the three
 *  sequencers, the one interrupt vector with its per-LUT flags); Lut<n>:
 *  one look-up table - a config struct owns its three inputs, the truth
 *  table, filter, edge detector, clock, output pin, interrupt sense;
 *  init<cfg>() / init(cfg) write it, the EVENTA/EVENTB users and the
 *  LUTn output generator are its event vocabulary (evsys.hpp);
 *
 *  the task - ToggleFlipFlop<pair>: a JK flip-flop on a LUT pair toggled
 *  by one event (a divide-by-two / a latch a timer sets and resets
 *  with its own events, no CPU); more tasks come with their users.
 *
 * The smallest use, PB3 = PB0 AND PB1 on a 48/64-pin AVR DA/DB (LUT4's
 * port is PORTB):
 *
 *   using And = brio::Lut<4>;
 *   brio::Ccl::disable();
 *   And::init<brio::LutConfig{.in0 = brio::LutInput::pin, .in1 = brio::LutInput::pin,
 *                             .truth = brio::lut_truth([](bool a, bool b, bool) { return a && b; }),
 *                             .output_pin = true}>();
 *   brio::Ccl::enable();
 *
 * Ccl::disable() stops the whole block, the state a configuration starts
 * from; init<cfg>() checks the configuration at compile time, takes the
 * LUT's own pins 0 and 1 as inputs 0 and 1, writes the truth table
 * lut_truth() computes from the lambda (input 2 is `mask`, low) and,
 * with output_pin, makes pin 3 an output carrying the result;
 * Ccl::enable() starts the block. LUT4 exists on the 48/64-pin AVR
 * DA/DB alone; elsewhere a lower LUT does the same on its own port
 * (lut_port()). What decides correctness even there: on the AVR DB's
 * silicon A4/A5 a LUT is reconfigured only with the whole block
 * disabled (errata 2.4.1, below), which the disable/enable bracket does;
 * the driver configures the output pin and nothing else of the port, so
 * the input pins are read as PORT leaves them; and an interrupt sense
 * other than none needs the program's ISR(CCL_CCL_vect) calling
 * Ccl::take_flags().
 *
 * Who calls what, and from where. Nothing here waits or masks
 * interrupts. Ccl::enable() and disable() are one store each; flag()
 * and clear_flag() one load or one store, legal from any context.
 * Ccl::sequencer(), Lut<n>::init(), sense(), enable() and disable() of
 * one LUT, and ToggleFlipFlop::init() read, modify and write registers
 * shared between LUTs (INTCTRL, PORTMUX.CCLROUTEA) or the pins' port,
 * and a Lut keeps a note of the pin it owns: they are setup, in main()
 * or in the handler of the one active object that owns the wiring (an
 * active object is a component of a brio program with its own event
 * queue, served one event at a time by the kernel,
 * docs/learn/introduction.md). Ccl::take_flags() is the interrupt body,
 * called from the program's ISR(CCL_CCL_vect) and from nowhere else.
 *
 * Facts that shape the code (31.3, errata DS80000915F 2.4.1):
 *  - every LUT register but ENABLE is enable-protected: written while
 *    the LUT is disabled, or together with ENABLE = 1 in the same write
 *    (init does both: CTRLA = 0 first, then CTRLB, CTRLC, TRUTH while
 *    the LUT is disabled, then CTRLA's own fields together with
 *    ENABLE). On silicon A4/A5 (2.4.1) reconfiguring any LUT wants the
 *    whole CCL disabled, which drops every other LUT meanwhile - so the
 *    protocol is: Ccl::disable(); Ccl::sequencer<pair>(...) (before the
 *    even LUT's init: SEQSEL is protected by that LUT's ENABLE and a
 *    write after it is silently ignored, measured); Lut<..>::init(...)
 *    for each; Ccl::enable(). A LUT reconfigured under a running block
 *    is a glitch on all of them, by design of the silicon, not of this
 *    driver. The AVR EA has no such erratum: its protection is the LUT's
 *    own, the even LUT's for a sequencer (DS40002443A 29.3.1.1), and on
 *    its rev B1 one LUT re-initialised under a running block
 *    (Lut<n>::init() alone, which disables and re-enables that LUT only)
 *    takes its new table at once with the other LUTs undisturbed
 *    (measured, docs/avrxt/ccl.md) - there the block's disable is a
 *    choice, not the protocol;
 *  - inputs: the same menu for the three inputs, but the peripheral
 *    entries select the instance by input index - input 0/1/2 of a LUT
 *    sees AC0/1/2 (AC0/1/1 on the AVR EA, which has no AC2), TCB0/1/2
 *    WO, TCA WO0/1/2, USART0/1/2 TXD, ZCD0/1/2, SPI0 MOSI/MOSI/SCK,
 *    TCD0 WOA/WOB/WOC (no ZCD and no TCD on the AVR EA: those inputs are
 *    not named there); `in`/`pin` is the LUT's own
 *    pin IN0/1/2 (pins 0/1/2 of its port), `link` is LUT[n+1]'s output
 *    (LUT0 into the last), `feedback` the pair's sequencer output,
 *    `event_a`/`event_b` the two EVSYS users (raw, no detection);
 *  - TRUTHn[k] is the output for input pattern k (IN2 the MSB); an
 *    unused input is `mask` (reads 0): build the table with lut_truth()
 *    from a lambda of three bools;
 *  - the filter/edge/sequencer clock is CLK_PER by default, or the
 *    LUT's own input 2 (then input 2 reads 0 in the truth table), OSCHF,
 *    OSC32K, OSC1K; SYNCH delays two clocks, FILTER needs the input
 *    stable over two clocks and delays four; the edge detector requires
 *    a filter option and yields a one-clock pulse on a rising edge;
 *  - the sequencer of a pair is clocked by the even LUT's clock, takes
 *    the even LUT as D/J/D/S and the odd as G/K/G/R, and its output is
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

#include "avrxt/device_tables.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/pin.hpp"

namespace brio {

// ---- the knobs (31.5) -------------------------------------------------------------

/// INSELx: one of the three inputs of a LUT. The instance the
/// peripheral entries mean depends on which input (0/1/2) they sit on.
/// The codes are the device header's INSEL0 enumerators, and the static
/// assertions below check that the same code means the same kind on
/// INSEL1 and INSEL2 (on the AVR EA, whose AC code means AC1 on input 2,
/// lut_input_ac() says which comparator). An input whose peripheral this
/// part has not got is not named at all: `tca1` without TCA1 (the
/// 28/32-pin AVR DA/DB), `zcd` without a ZCD, `tcd0` without the TCD.
enum class LutInput : uint8_t {
    mask = CCL_INSEL0_MASK_gc,             ///< tied low
    feedback = CCL_INSEL0_FEEDBACK_gc,     ///< the pair's sequencer output
    link = CCL_INSEL0_LINK_gc,             ///< LUT[n+1] output (the last LUT takes LUT0)
    event_a = CCL_INSEL0_EVENTA_gc,        ///< EVSYS user LUTnA
    event_b = CCL_INSEL0_EVENTB_gc,        ///< EVSYS user LUTnB
    pin = CCL_INSEL0_IN0_gc,               ///< the LUT's own pin INx
    ac = CCL_INSEL0_AC0_gc,                ///< ACx OUT (x = input index; lut_input_ac())
#ifdef ZCD0
    zcd = CCL_INSEL0_ZCD0_gc,              ///< ZCDx OUT
#endif
    usart_txd = CCL_INSEL0_USART0_gc,      ///< USARTx TXD (host modes)
    spi0 = CCL_INSEL0_SPI0_gc,             ///< SPI0 MOSI (inputs 0/1), SCK (input 2); host mode (ccl.md)
    tca0 = CCL_INSEL0_TCA0_gc,             ///< TCA0 WOx
#ifdef TCA1
    tca1 = CCL_INSEL0_TCA1_gc,             ///< TCA1 WOx
#endif
    tcb = CCL_INSEL0_TCB0_gc,              ///< TCBx WO
#ifdef TCD0
    tcd0 = CCL_INSEL0_TCD0_gc,             ///< TCD0 WOA/WOB/WOC
#endif
};

/// One code per kind serves the three inputs (INSEL1 four bits up in
/// CTRLB, INSEL2 in CTRLC): checked kind by kind against the header,
/// for every instance-indexed kind whose instance on that input exists.
constexpr bool ccl_insel_same(uint8_t in0, uint8_t in1, uint8_t in2) {
    return in1 == static_cast<uint8_t>(in0 << 4) && in2 == in0;
}
static_assert(ccl_insel_same(CCL_INSEL0_MASK_gc, CCL_INSEL1_MASK_gc, CCL_INSEL2_MASK_gc) &&
              ccl_insel_same(CCL_INSEL0_FEEDBACK_gc, CCL_INSEL1_FEEDBACK_gc, CCL_INSEL2_FEEDBACK_gc) &&
              ccl_insel_same(CCL_INSEL0_LINK_gc, CCL_INSEL1_LINK_gc, CCL_INSEL2_LINK_gc) &&
              ccl_insel_same(CCL_INSEL0_EVENTA_gc, CCL_INSEL1_EVENTA_gc, CCL_INSEL2_EVENTA_gc) &&
              ccl_insel_same(CCL_INSEL0_EVENTB_gc, CCL_INSEL1_EVENTB_gc, CCL_INSEL2_EVENTB_gc) &&
              ccl_insel_same(CCL_INSEL0_IN0_gc, CCL_INSEL1_IN1_gc, CCL_INSEL2_IN2_gc) &&
              ccl_insel_same(CCL_INSEL0_USART0_gc, CCL_INSEL1_USART1_gc, CCL_INSEL2_USART2_gc) &&
              ccl_insel_same(CCL_INSEL0_SPI0_gc, CCL_INSEL1_SPI0_gc, CCL_INSEL2_SPI0_gc) &&
              ccl_insel_same(CCL_INSEL0_TCA0_gc, CCL_INSEL1_TCA0_gc, CCL_INSEL2_TCA0_gc) &&
              ccl_insel_same(CCL_INSEL0_TCB0_gc, CCL_INSEL1_TCB1_gc, CCL_INSEL2_TCB2_gc),
              "LutInput: a kind's INSEL code differs between the three inputs on this part");
static_assert(static_cast<uint8_t>(CCL_INSEL1_AC1_gc) == static_cast<uint8_t>(CCL_INSEL0_AC0_gc << 4),
              "LutInput::ac: the AC code differs between inputs 0 and 1 on this part");
static_assert(ccl_insel2_ac_code == static_cast<uint8_t>(CCL_INSEL0_AC0_gc),
              "LutInput::ac: the AC code differs between inputs 0 and 2 on this part");

/// The comparator `LutInput::ac` selects on LUT input `input` (0..2):
/// ACx on input x, except where the part has no AC2 - the AVR EA, whose
/// input 2 sees AC1 under the same code (the device tables' probe of the
/// header's INSEL2 enumerators, DS40002443A 29.2.2.1).
constexpr uint8_t lut_input_ac(uint8_t input) {
    return input < 2 ? input : ccl_insel2_ac_instance;
}
#ifdef ZCD1
static_assert(static_cast<uint8_t>(CCL_INSEL1_ZCD1_gc) == static_cast<uint8_t>(CCL_INSEL0_ZCD0_gc << 4),
              "LutInput::zcd: the ZCD code differs between inputs 0 and 1 on this part");
#endif
#ifdef ZCD2
static_assert(static_cast<uint8_t>(CCL_INSEL2_ZCD2_gc) == static_cast<uint8_t>(CCL_INSEL0_ZCD0_gc),
              "LutInput::zcd: the ZCD code differs between inputs 0 and 2 on this part");
#endif
#ifdef TCA1
static_assert(ccl_insel_same(CCL_INSEL0_TCA1_gc, CCL_INSEL1_TCA1_gc, CCL_INSEL2_TCA1_gc),
              "LutInput::tca1: the TCA1 code differs between the three inputs on this part");
#endif
#ifdef TCD0
static_assert(ccl_insel_same(CCL_INSEL0_TCD0_gc, CCL_INSEL1_TCD0_gc, CCL_INSEL2_TCD0_gc),
              "LutInput::tcd0: the TCD0 code differs between the three inputs on this part");
#endif

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
    d_flip_flop = CCL_SEQSEL_DFF_gc,    ///< D = even LUT, EN = odd, clocked by the even LUT's clock
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

/// The port letter of LUT n's pins: A, C, D, F, B, G.
constexpr char lut_port(uint8_t n) {
    constexpr char p[] = {'A', 'C', 'D', 'F', 'B', 'G'};
    return p[n];
}

// ---- the block ------------------------------------------------------------------

struct Ccl {
    Ccl() = delete;

    /// Enable the block (after every LUT and sequencer is configured):
    /// one store to CTRLA, RUNSTDBY with it when asked.
    static void enable(bool run_standby = false) {
        CCL.CTRLA = static_cast<uint8_t>(CCL_ENABLE_bm | (run_standby ? CCL_RUNSTDBY_bm : 0));
    }
    /// Disable the whole block: the state every reconfiguration starts
    /// from (errata 2.4.1). All LUT outputs drop.
    static void disable() { CCL.CTRLA = 0; }
    static bool enabled() { return (CCL.CTRLA & CCL_ENABLE_bm) != 0; }

    /// The sequencer of pair p (LUT 2p / 2p+1). Enable-protected by the
    /// even LUT: written while LUT 2p is disabled, before its init()
    /// (written after, it is silently ignored - measured). Disables the
    /// even LUT first, to be safe.
    template <uint8_t pair>
    static void sequencer(Sequencer s) {
        static_assert(2 * pair + 1 < lut_count,
                      "no such LUT pair on this package (pair 2 = LUT4/5, the 48/64-pin AVR DA/DB only)");
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

constexpr bool lut_config_valid(uint8_t n, const LutConfig& c) {
    if (c.edge_detect && c.filter == LutFilter::none) return false;   // 31.3.1.6
    if (c.output_pin && !port_exists(lut_port(n))) return false;      // LUT5's pins are PORTG (64-pin)
    if (c.alt_pin && n == 3) return false;      // LUT3 has no ALT1: the CCLROUTEA bit is reserved, PF6 is RESET
    return true;
}

// ---- one look-up table ------------------------------------------------------------

template <uint8_t n>
class Lut {
    static_assert(n < lut_count, "no such CCL LUT on this package (LUT4/5 on the 48/64-pin AVR DA/DB only)");

public:
    Lut() = delete;

    static constexpr uint8_t index = n;
    static constexpr char port = lut_port(n);
    /// This package bonds the LUT's port (LUT5's is PORTG, 64-pin
    /// only): without it the LUT still works on events - only the pin
    /// faces are gone. LUT3 additionally has no ALT1 on any package.
    static constexpr bool has_pins = port_exists(port);
    static constexpr bool has_alt_pin = has_pins && n != 3;

    using In0 = Pin<port, 0>;
    using In1 = Pin<port, 1>;
    using In2 = Pin<port, 2>;
    using OutDefault = Pin<port, 3>;
    using OutAlt = Pin<port, 6>;

    using OutEvent = EvLut<n>;            ///< generator: the LUT output level
    using EventA = EvLutIn<n, 'A'>;       ///< users: the two event inputs
    using EventB = EvLutIn<n, 'B'>;

    /// Compile-time form: lut_config_valid()'s refusals become compile
    /// errors, then the run-time form below does the writes. Setup.
    template <LutConfig cfg>
    static void init() {
        static_assert(lut_config_valid(n, cfg),
                      "LutConfig: the edge detector needs a filter option (sync or "
                      "filter); the output pin needs a port this package bonds; "
                      "LUT3 has no ALT1 (reserved route, PF6 is RESET)");
        init(cfg);
    }

    /// Run-time form. Call with the block disabled (Ccl::disable(); on the
    /// AVR EA a running block takes it too, the file header):
    /// writes inputs, truth, route, output pin, interrupt sense, then
    /// CTRLA with ENABLE = 1 (the enable-protected fields land together
    /// with the enable). False, touching nothing, for an invalid config.
    static bool init(const LutConfig& cfg) {
        if (!lut_config_valid(n, cfg)) return false;
        ctrla() = 0;                                          // disabled: the protected registers open
        ctrlb() = static_cast<uint8_t>(static_cast<uint8_t>(cfg.in0) | (static_cast<uint8_t>(cfg.in1) << 4));
        ctrlc() = static_cast<uint8_t>(cfg.in2);
        truth() = cfg.truth;
        route(cfg.alt_pin);
        drive_pin(cfg);
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
#ifdef CCL_LUT4CTRLA
        volatile uint8_t& r = n < 4 ? CCL.INTCTRL0 : CCL.INTCTRL1;
#else
        volatile uint8_t& r = CCL.INTCTRL0;      // 4 LUTs: no INTCTRL1
#endif
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

    /// Drive (and release) the output pin per the config. The branches
    /// for a port this package lacks are compiled out (a runtime `if`
    /// would instantiate the missing Pin and kill the whole LUT -
    /// LUT5's port is PORTG); lut_config_valid has already refused a
    /// config asking for them. A position taken by a previous init and
    /// abandoned by this one goes back to input.
    static void drive_pin(const LutConfig& cfg) {
        if constexpr (has_pins) {
            const uint8_t want = cfg.output_pin ? (cfg.alt_pin ? 2u : 1u) : 0u;
            if (owned_ == 1 && want != 1) OutDefault::input();
            if constexpr (has_alt_pin) {
                if (owned_ == 2 && want != 2) OutAlt::input();
                if (want == 2) OutAlt::output();
            }
            if (want == 1) OutDefault::output();
            owned_ = static_cast<uint8_t>(want);
        } else {
            (void)cfg;
        }
    }
    static inline uint8_t owned_ = 0;

public:
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
    static_assert(2 * pair + 1 < lut_count,
                  "no such LUT pair on this package (pair 2 = LUT4/5, the 48/64-pin AVR DA/DB only)");

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
