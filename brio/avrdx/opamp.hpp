/*
 * opamp.hpp
 *
 * The AVR DB analog signal conditioning block (OPAMP, DS40002247B
 * ch. 35) as brio sees it - see docs/avrdx/opamp.md. DB ONLY: the DA
 * family has no such peripheral, so this whole header is gated on the
 * device header's `OPAMP` symbol and `opamp_count` (evsys.hpp) is 0
 * there. Include it anywhere; use it inside `#ifdef OPAMP`.
 *
 *  RESOURCES - OpampSystem: the block every op amp shares (the one
 *  ENABLE, the TIMEBASE that turns CLK_PER into microseconds for the
 *  settle timers, PWRCTRL.IRSEL, DBGCTRL). It is the ClockUser of the
 *  chapter: rebase() rewrites TIMEBASE so a settle time keeps meaning
 *  the same microseconds after a CLK_PER change. Opamp<n>, n < 2 or 3
 *  by package: one op amp - a config struct owns the two input
 *  multiplexers, the three resistor-ladder multiplexers, the output
 *  driver mode, the enable/disable regime (the chapter's three), the
 *  settle time and RUNSTBY; init<cfg>() / init(cfg) write it. Verbs:
 *  settled(), the four event users (enable / disable / dump / drive),
 *  the READY event generator, the offset calibration byte.
 *
 *  TASKS - what an application names (each takes the Opamp<n> it runs
 *  on: OpampPga<Opamp<0>>):
 *    OpampFollower<Op>      unity-gain buffer of any positive source
 *    OpampPga<Op>           non-inverting PGA, gain 1 + R2/R1
 *    OpampInvertingPga<Op>  inverting PGA about VDD/2, gain -R2/R1
 *    InstrumentationAmp<>   the chapter's three-op-amp recipe (35.3.7),
 *                           only on packages that have OP2
 *  The eight gains the ladder makes are exact rationals (OpampGain) and
 *  the tasks keep them so: for_gain() picks the wiper for a gain the
 *  ladder really makes and returns nothing for one it does not.
 *  The INTEGRATOR (figure 35-6) is deliberately NOT a task: it needs an
 *  external R and C and a DUMP policy, and it is born with its first
 *  user. Its mechanism is all here - MUXNEG = inn with the ladder off,
 *  and dump_on(channel).
 *
 * Facts that shape the code (35.3, 35.5, 39.20):
 *  - three op amps OP0/OP1/OP2 (OP2 on 48/64-pin parts only - the
 *    device header is the authority and `opamp_count` reads it), each
 *    with three dedicated pads: OP0 INP PD1 / OUT PD2 / INN PD3,
 *    OP1 INP PD4 / OUT PD5 / INN PD7 (PD6 is the DAC's), OP2 INP PE1 /
 *    OUT PE2 / INN PE3. init() disables the digital input buffer of
 *    every pad it claims and release() puts it back;
 *  - MUXPOS: INP / WIP / DAC / GND / VDD/2, plus LINKOUT (OP[n-1]
 *    output) on OP1 and OP2 and LINKWIP (OP0's wiper) on OP2 alone -
 *    the driver refuses the other combinations at compile and run time;
 *    MUXNEG: INN / WIP / OUT / DAC, the same four everywhere;
 *  - the ladder is 16R: MUXTOP OFF/OUT/VDD, MUXBOT OFF/INP/INN/DAC/
 *    LINKOUT/GND, MUXWIP splitting it into R1 (bottom) and R2 (top) as
 *    15+1, 14+2, 12+4, 8+8, 6+10, 4+12, 2+14, 1+15. MUXBOT's LINKOUT
 *    is OP[n-1]'s output - and for OP0 that is OP2's (35.5.7 note 1),
 *    so it is refused on a package without OP2;
 *  - the DAC entries mean the BUFFERED DAC output: the DAC's OUTEN must
 *    be on (34.3.2.3) and PD6 is then the DAC's;
 *  - the internal timer: TIMEBASE is one less than the number of
 *    CLK_PER cycles that reach 1 us (35.5.3), OPnSETTLE is that many
 *    microseconds, and SETTLED rises when warm-up + settle are done.
 *    The settle timer RESTARTS on every write to OPnCTRLA, OPnINMUX or
 *    OPnRESMUX, which is why init() writes CTRLA last;
 *  - three enable regimes (35.3.2.7): ALWAYSON alone (software, deaf to
 *    events), EVENTEN alone (the ENABLE/DISABLE events own it), both
 *    (software enable, DUMP and DRIVE still listened to). READY is
 *    generated whenever EVENTEN is set;
 *  - the four event users are OPAMP's own (evsys.hpp): ENABLEn and
 *    DISABLEn are EDGE-detected, DUMPn and DRIVEn are LEVELS - a DUMP
 *    or DRIVE channel must HOLD its level, a pulse does nothing;
 *  - OUTMODE OFF still lets the DRIVEn event raise the driver; when a
 *    driver is on, the OUT pad belongs to the op amp and no other
 *    peripheral can drive it;
 *  - OPnCAL comes out of reset with the production value from fuses;
 *    0x00 is the most negative offset trim, 0x80 none, 0xFF the most
 *    positive, one step = 0.5 mV (39-27);
 *  - PWRCTRL.IRSEL trades the rail-to-rail input range (VICM -0.3 V ..
 *    VDD + 0.3 V) for VDD - 0.7 V and less current. Errata DS80000915F
 *    2.8.2 makes that bit READ-ONLY on silicon A4 - the verb therefore
 *    returns what the silicon actually took, and promises nothing else.
 *    2.8.1 (A4 too) is a current figure with no work-around and no
 *    register face: it is recorded in the doc, not in code;
 *  - no interrupts at all (35.3.4): the only outward signal is the
 *    READYn event.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <avr/io.h>

#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "util/clock.hpp"

#ifdef OPAMP

namespace brio {

// ---- the knobs (35.5) -------------------------------------------------------------

/// MUXPOS: the non-inverting (+) input. The codes are the device
/// header's, taken from the instance that names each of them; `link_wip`
/// is 35.5.8's 0x6 (the header spells it OPAMP_OP2INMUX_MUXPOS_LINKWIP_gv
/// and only the headers of packages WITH OP2 define it).
enum class OpampPos : uint8_t {
    inp      = OPAMP_OP0INMUX_MUXPOS_INP_gv,       ///< OPnINP pad
    wiper    = OPAMP_OP0INMUX_MUXPOS_WIP_gv,       ///< OPn's own ladder wiper
    dac      = OPAMP_OP0INMUX_MUXPOS_DAC_gv,       ///< the BUFFERED DAC output
    gnd      = OPAMP_OP0INMUX_MUXPOS_GND_gv,
    vdd_div2 = OPAMP_OP0INMUX_MUXPOS_VDDDIV2_gv,   ///< VDD/2, +-3 %
    link_out = OPAMP_OP1INMUX_MUXPOS_LINKOUT_gv,   ///< OP[n-1] output: OP1 and OP2 only
    link_wip = 0x06,                               ///< OP0's ladder wiper: OP2 only
};

/// MUXNEG: the inverting (-) input. The same four on every instance.
enum class OpampNeg : uint8_t {
    inn   = OPAMP_OP0INMUX_MUXNEG_INN_gv,    ///< OPnINN pad
    wiper = OPAMP_OP0INMUX_MUXNEG_WIP_gv,    ///< OPn's own ladder wiper
    out   = OPAMP_OP0INMUX_MUXNEG_OUT_gv,    ///< OPn output: unity gain
    dac   = OPAMP_OP0INMUX_MUXNEG_DAC_gv,    ///< the BUFFERED DAC output
};

/// MUXTOP: what the top of the ladder (above R2) hangs from.
enum class OpampTop : uint8_t {
    off = OPAMP_OP0RESMUX_MUXTOP_OFF_gv,
    out = OPAMP_OP0RESMUX_MUXTOP_OUT_gv,     ///< OPn output: the PGA feedback
    vdd = OPAMP_OP0RESMUX_MUXTOP_VDD_gv,
};

/// MUXBOT: what the bottom of the ladder (below R1) hangs from.
enum class OpampBot : uint8_t {
    off      = OPAMP_OP0RESMUX_MUXBOT_OFF_gv,
    inp      = OPAMP_OP0RESMUX_MUXBOT_INP_gv,
    inn      = OPAMP_OP0RESMUX_MUXBOT_INN_gv,
    dac      = OPAMP_OP0RESMUX_MUXBOT_DAC_gv,      ///< the BUFFERED DAC output
    link_out = OPAMP_OP0RESMUX_MUXBOT_LINKOUT_gv,  ///< OP[n-1] output; for OP0 that is OP2's
    gnd      = OPAMP_OP0RESMUX_MUXBOT_GND_gv,
};

/// MUXWIP: where the wiper taps the 16R ladder. The names are the
/// silicon's; the RATIO each one makes depends on the topology, so ask
/// opamp_noninverting_gain() / opamp_inverting_gain() for that.
enum class OpampWiper : uint8_t {
    wip0 = 0,   ///< R1 = 15R, R2 = 1R
    wip1 = 1,   ///< R1 = 14R, R2 = 2R
    wip2 = 2,   ///< R1 = 12R, R2 = 4R
    wip3 = 3,   ///< R1 = 8R,  R2 = 8R
    wip4 = 4,   ///< R1 = 6R,  R2 = 10R
    wip5 = 5,   ///< R1 = 4R,  R2 = 12R
    wip6 = 6,   ///< R1 = 2R,  R2 = 14R
    wip7 = 7,   ///< R1 = 1R,  R2 = 15R
};

/// OUTMODE. Codes 0x2/0x3 are reserved (35.5.5) and have no name here.
enum class OpampOutput : uint8_t {
    off    = OPAMP_OP0CTRLA_OUTMODE_OFF_gv,     ///< driver off - a DRIVEn event still raises it
    normal = OPAMP_OP0CTRLA_OUTMODE_NORMAL_gv,  ///< driver on, the pad is the op amp's
};

/// The three enable/disable regimes of 35.3.2.7, plus the state an op
/// amp that is configured but not running is in.
enum class OpampMode : uint8_t {
    off,                    ///< ALWAYSON = 0, EVENTEN = 0: dead, deaf, silent
    software,               ///< ALWAYSON = 1, EVENTEN = 0: on, no events either way
    event,                  ///< ALWAYSON = 0, EVENTEN = 1: ENABLEn/DISABLEn own it
    software_with_events,   ///< ALWAYSON = 1, EVENTEN = 1: on, DUMPn/DRIVEn heard, READYn issued
};

struct OpampConfig {
    OpampPos positive = OpampPos::inp;
    OpampNeg negative = OpampNeg::out;          ///< a voltage follower, out of the box
    OpampTop top = OpampTop::off;
    OpampBot bottom = OpampBot::off;
    OpampWiper wiper = OpampWiper::wip0;
    OpampOutput output = OpampOutput::normal;
    OpampMode mode = OpampMode::software;
    uint8_t settle_us = 127;    ///< SETTLE[6:0]; 35.3.2.6: use the max when it is unknown
    bool run_standby = false;   ///< RUNSTBY (the data sheet spells it RUNSTDBY)
};

// ---- the ladder's arithmetic -------------------------------------------------------

/// An exact gain as a rational (negative numerator = an inverting one):
/// the ladder's gains are 16/15, 8/7, 4/3, 2, ... - naming them by a
/// rounded integer would be a lie.
struct OpampGain {
    int16_t num;
    int16_t den;
    constexpr bool operator==(const OpampGain&) const = default;
};

/// R1 (bottom, below the wiper) and R2 (top) in units of R; R1 + R2 = 16R.
constexpr uint8_t opamp_ladder_r1(OpampWiper w) {
    constexpr uint8_t r[] = {15, 14, 12, 8, 6, 4, 2, 1};
    return r[static_cast<uint8_t>(w)];
}
constexpr uint8_t opamp_ladder_r2(OpampWiper w) {
    return static_cast<uint8_t>(16u - opamp_ladder_r1(w));
}

namespace detail {
constexpr int16_t opamp_gcd(int16_t a, int16_t b) {
    if (a < 0) a = static_cast<int16_t>(-a);
    while (b != 0) { const int16_t t = static_cast<int16_t>(a % b); a = b; b = t; }
    return a == 0 ? 1 : a;
}
constexpr OpampGain opamp_reduce(int16_t num, int16_t den) {
    const int16_t g = opamp_gcd(num, den);
    return {static_cast<int16_t>(num / g), static_cast<int16_t>(den / g)};
}
}  // namespace detail

/// Non-inverting PGA (figure 35-4): VOUT = VIN x (1 + R2/R1) = VIN x 16/R1.
/// The eight the ladder makes: 16/15, 8/7, 4/3, 2, 8/3, 4, 8, 16.
constexpr OpampGain opamp_noninverting_gain(OpampWiper w) {
    return detail::opamp_reduce(16, opamp_ladder_r1(w));
}

/// Inverting PGA (figure 35-5): VOUT - VDD/2 = -(R2/R1) x (VIN - VDD/2).
/// The eight: -1/15, -1/7, -1/3, -1, -5/3, -3, -7, -15.
constexpr OpampGain opamp_inverting_gain(OpampWiper w) {
    return detail::opamp_reduce(static_cast<int16_t>(-opamp_ladder_r2(w)), opamp_ladder_r1(w));
}

/// A gain as parts per thousand, for printing and for comparing a
/// measurement against the exact value (rounded to nearest, sign kept).
constexpr int32_t opamp_gain_x1000(OpampGain g) {
    const int32_t n = static_cast<int32_t>(g.num) * 1000;
    const int32_t d = g.den;
    return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d);
}

/// The wiper whose non-inverting / inverting gain is exactly num/den,
/// if the ladder makes it (nothing otherwise - the ladder is discrete
/// and an app must not silently get a neighbour).
constexpr std::optional<OpampWiper> opamp_noninverting_wiper(int16_t num, int16_t den) {
    for (uint8_t i = 0; i < 8; ++i) {
        const OpampWiper w = static_cast<OpampWiper>(i);
        if (opamp_noninverting_gain(w) == detail::opamp_reduce(num, den)) return w;
    }
    return {};
}
constexpr std::optional<OpampWiper> opamp_inverting_wiper(int16_t num, int16_t den) {
    for (uint8_t i = 0; i < 8; ++i) {
        const OpampWiper w = static_cast<OpampWiper>(i);
        if (opamp_inverting_gain(w) == detail::opamp_reduce(num, den)) return w;
    }
    return {};
}

// ---- the pads (DS40002247B ch. 3) ---------------------------------------------------

/// The pin (port letter, pin number) of one of an op amp's three pads.
/// {0, 0} = this package does not bond it. OP2's pads are PORTE, which
/// is exactly the port the packages without OP2 also lack.
struct OpampPin { char port; uint8_t pin; };

constexpr OpampPin opamp_inp_pin(uint8_t n) {
    constexpr OpampPin p[] = {{'D', 1}, {'D', 4}, {'E', 1}};
    return n < 3 && port_exists(p[n].port) ? p[n] : OpampPin{0, 0};
}
constexpr OpampPin opamp_out_pin(uint8_t n) {
    constexpr OpampPin p[] = {{'D', 2}, {'D', 5}, {'E', 2}};
    return n < 3 && port_exists(p[n].port) ? p[n] : OpampPin{0, 0};
}
constexpr OpampPin opamp_inn_pin(uint8_t n) {
    constexpr OpampPin p[] = {{'D', 3}, {'D', 7}, {'E', 3}};
    return n < 3 && port_exists(p[n].port) ? p[n] : OpampPin{0, 0};
}

// ---- what a configuration may ask of instance n --------------------------------------

/// Does this configuration name an input pad (MUXPOS INP, MUXNEG INN,
/// MUXBOT INP/INN)? Used both to validate and to know which pads init()
/// claims.
constexpr bool opamp_uses_inp(const OpampConfig& c) {
    return c.positive == OpampPos::inp || c.bottom == OpampBot::inp;
}
constexpr bool opamp_uses_inn(const OpampConfig& c) {
    return c.negative == OpampNeg::inn || c.bottom == OpampBot::inn;
}

/// What init<cfg>() static_asserts and init(cfg) returns false for.
constexpr bool opamp_config_valid(uint8_t n, const OpampConfig& c) {
    if (n >= opamp_count) return false;
    // MUXPOS link codes exist per instance (35.5.8).
    if (c.positive == OpampPos::link_out && n == 0) return false;
    if (c.positive == OpampPos::link_wip && n != 2) return false;
    // MUXBOT LINKOUT is OP[n-1]'s output, and OP0's neighbour is OP2
    // (35.5.7 note 1): without OP2 there is nothing on the other end.
    if (c.bottom == OpampBot::link_out && n == 0 && opamp_count < 3) return false;
    if (c.settle_us > OPAMP_SETTLE_gm) return false;
    // The pads this package must bond for the configuration to mean anything.
    if (opamp_uses_inp(c) && opamp_inp_pin(n).port == 0) return false;
    if (opamp_uses_inn(c) && opamp_inn_pin(n).port == 0) return false;
    if (c.output == OpampOutput::normal && opamp_out_pin(n).port == 0) return false;
    return true;
}

/// TIMEBASE for a peripheral clock: one less than the number of CLK_PER
/// cycles that reach 1 us (35.5.3 - the same number 35.3.2.6 arrives at
/// by rounding a non-integer down). 24 MHz -> 23, 12 MHz -> 11, and
/// anything at or below 1 MHz -> 0, where one CLK_PER cycle is already
/// a microsecond or more and a settle time is longer than it says.
constexpr uint8_t opamp_timebase(uint32_t clk_per_hz) {
    const uint32_t cycles = (clk_per_hz + 999'999u) / 1'000'000u;   // ceil to 1 us
    const uint32_t v = cycles == 0 ? 0 : cycles - 1;
    return static_cast<uint8_t>(v > OPAMP_TIMEBASE_gm ? OPAMP_TIMEBASE_gm : v);
}

/// How long one TIMEBASE tick REALLY lasts, in nanoseconds: a settle
/// time of S microseconds costs S of these. It is 1000 ns only when
/// CLK_PER is a whole number of MHz; anywhere else the tick is longer
/// (TIMEBASE rounds the cycle count up) and a settle time is longer
/// than it says. Exact for any clock that is a whole number of kHz.
constexpr uint32_t opamp_timebase_ns(uint32_t clk_per_hz) {
    if (clk_per_hz < 1000u) return 0;
    const uint32_t cycles = static_cast<uint32_t>(opamp_timebase(clk_per_hz)) + 1u;
    return (cycles * 1'000'000u) / (clk_per_hz / 1000u);
}

// ---- the block ------------------------------------------------------------------------

/// OpampSystem: everything the three op amps share - the one ENABLE,
/// the TIMEBASE their settle timers count in, PWRCTRL and DBGCTRL.
/// It is the chapter's ClockUser: a DynamicClock that carries op amps
/// must list OpampSystem (once, not per instance - there is one
/// TIMEBASE register) or the settle timers would keep counting the old
/// clock's microseconds.
struct OpampSystem {
    OpampSystem() = delete;

    /// Write TIMEBASE for this clock, the debug and power bits, and
    /// enable the block. Configure the op amps first (35.3.1) or right
    /// after - a per-op CTRLA write restarts its settle timer either way.
    template <typename Clock>
    static void init(Clock clock, bool reduced_range = false, bool debug_run = false) {
        static_assert(clock_follows<Clock, OpampSystem>(),
                      "this OpampSystem is initialized with a DynamicClock that does not "
                      "list it among its Users: TIMEBASE would keep the old rate and every "
                      "settle time would mean the wrong number of microseconds");
        OPAMP.CTRLA = 0;
        OPAMP.TIMEBASE = opamp_timebase(clock_hz(clock));
        OPAMP.DBGCTRL = debug_run ? OPAMP_DBGRUN_bm : 0;
        (void)OpampSystem::reduced_input_range(reduced_range);
        timebase_hz_ = clock_hz(clock);
        OPAMP.CTRLA = OPAMP_ENABLE_bm;
    }

    static void enable() { OPAMP.CTRLA = OPAMP_ENABLE_bm; }
    /// The whole block off. Every op amp stops and releases its OUT pad;
    /// the per-op configuration stays in the registers.
    static void disable() { OPAMP.CTRLA = 0; }
    static bool enabled() { return (OPAMP.CTRLA & OPAMP_ENABLE_bm) != 0; }

    static uint8_t timebase() { return OPAMP.TIMEBASE; }
    static void timebase(uint8_t v) { OPAMP.TIMEBASE = static_cast<uint8_t>(v & OPAMP_TIMEBASE_gm); }

    /// The peripheral clock changed (DynamicClock fan-out): TIMEBASE
    /// again, so a settle time keeps meaning the microseconds it says.
    /// Nothing else in the chapter depends on CLK_PER.
    static void rebase(uint32_t hz) {
        OPAMP.TIMEBASE = opamp_timebase(hz);
        timebase_hz_ = hz;
    }
    /// The clock TIMEBASE was last written for.
    static uint32_t clock_hz_timebase() { return timebase_hz_; }

    /// PWRCTRL.IRSEL: true asks for the REDUCED input range (VICM up to
    /// VDD - 0.7 V) and less current, false for rail-to-rail. Returns
    /// what the silicon actually took: errata DS80000915F 2.8.2 makes
    /// this bit read-only on rev. A4, where the range is always
    /// rail-to-rail whatever is written. There is no work-around, so
    /// the driver reports rather than promises.
    static bool reduced_input_range(bool on) {
        OPAMP.PWRCTRL = on ? OPAMP_IRSEL_bm : 0;
        return reduced_input_range();
    }
    static bool reduced_input_range() { return (OPAMP.PWRCTRL & OPAMP_IRSEL_bm) != 0; }

    /// DBGCTRL: keep the DIGITAL interface running while the CPU is
    /// halted. The analog half runs regardless (35.3.6).
    static void debug_run(bool on) { OPAMP.DBGCTRL = on ? OPAMP_DBGRUN_bm : 0; }

private:
    static inline uint32_t timebase_hz_ = 0;
};

static_assert(ClockUser<OpampSystem>);

// ---- one op amp -----------------------------------------------------------------------

template <uint8_t n>
class Opamp {
    static_assert(n < opamp_count,
                  "no such op amp on this package (OP2 needs a 48- or 64-pin part)");

public:
    Opamp() = delete;

    static constexpr uint8_t index = n;

    /// The three pads, as compile-time descriptors ({0,0} when this
    /// package does not bond them).
    static constexpr OpampPin inp_pin = opamp_inp_pin(n);
    static constexpr OpampPin out_pin = opamp_out_pin(n);
    static constexpr OpampPin inn_pin = opamp_inn_pin(n);

    /// The event vocabulary of this instance (evsys.hpp).
    using ReadyEvent = EvOpampReady<n>;                          ///< generator: settling done
    using EnableIn  = EvOpampCtl<n, OpampAction::enable>;        ///< user, EDGE, async
    using DisableIn = EvOpampCtl<n, OpampAction::disable>;       ///< user, EDGE, sync
    using DumpIn    = EvOpampCtl<n, OpampAction::dump>;          ///< user, LEVEL, sync
    using DriveIn   = EvOpampCtl<n, OpampAction::drive>;         ///< user, LEVEL, sync

    // ---- configuration -------------------------------------------------------

    /// Compile-time form: every register value folded, the combinations
    /// this instance and this package cannot make refused here.
    template <OpampConfig cfg>
    static void init() {
        static_assert(opamp_config_valid(n, cfg),
                      "OpampConfig: MUXPOS LINKOUT exists on OP1/OP2 only and LINKWIP on OP2 "
                      "only; MUXBOT LINKOUT of OP0 is OP2's output, which needs a package with "
                      "OP2; SETTLE is 7 bits; a pad the package does not bond cannot be used");
        (void)init(cfg);
    }

    /// Run-time form. Turns the pads it claims into analog inputs,
    /// writes INMUX / RESMUX / SETTLE, then CTRLA - last on purpose: a
    /// write to any of the three restarts the settle timer, so the one
    /// that also starts the op amp goes in when everything else is
    /// already in place (35.3.2.6). The block's own ENABLE is
    /// OpampSystem's. False, touching nothing, for a configuration this
    /// instance or this package cannot make.
    static bool init(const OpampConfig& cfg) {
        if (!opamp_config_valid(n, cfg)) return false;
        claimed_ = 0;
        if (opamp_uses_inp(cfg)) { analog_input(inp_pin); claimed_ |= 1u; }
        if (opamp_uses_inn(cfg)) { analog_input(inn_pin); claimed_ |= 2u; }
        if (cfg.output == OpampOutput::normal) { analog_input(out_pin); claimed_ |= 4u; }
        inmux() = static_cast<uint8_t>(
            (static_cast<uint8_t>(cfg.positive) << OPAMP_MUXPOS_gp) |
            (static_cast<uint8_t>(cfg.negative) << OPAMP_MUXNEG_gp));
        resmux() = static_cast<uint8_t>(
            (static_cast<uint8_t>(cfg.top) << OPAMP_MUXTOP_gp) |
            (static_cast<uint8_t>(cfg.bottom) << OPAMP_MUXBOT_gp) |
            (static_cast<uint8_t>(cfg.wiper) << OPAMP_MUXWIP_gp));
        settle_reg() = static_cast<uint8_t>(cfg.settle_us & OPAMP_SETTLE_gm);
        ctrla() = ctrla_byte(cfg);
        return true;
    }

    /// Stop this op amp and give its pads back to PORT: OUTMODE OFF and
    /// both enable bits down, then the digital input buffer re-enabled
    /// on exactly the pads the last init() claimed. The block and the
    /// other op amps are untouched.
    static void release() {
        ctrla() = 0;
        if (claimed_ & 1u) digital_input(inp_pin);
        if (claimed_ & 2u) digital_input(inn_pin);
        if (claimed_ & 4u) digital_input(out_pin);
        claimed_ = 0;
    }

    // ---- running -------------------------------------------------------------

    /// ALWAYSON alone. Restarts the settle timer (every CTRLA write does).
    static void enable() { ctrla() |= OPAMP_ALWAYSON_bm; }
    static void disable() { ctrla() &= static_cast<uint8_t>(~OPAMP_ALWAYSON_bm); }
    static bool always_on() { return (ctrla() & OPAMP_ALWAYSON_bm) != 0; }

    /// The whole regime in one write (35.3.2.7).
    static void mode(OpampMode m) {
        ctrla() = static_cast<uint8_t>(
            (ctrla() & static_cast<uint8_t>(~(OPAMP_ALWAYSON_bm | OPAMP_EVENTEN_bm))) |
            mode_bits(m));
    }
    static OpampMode mode() {
        const uint8_t c = ctrla();
        const bool always = (c & OPAMP_ALWAYSON_bm) != 0;
        const bool events = (c & OPAMP_EVENTEN_bm) != 0;
        return always ? (events ? OpampMode::software_with_events : OpampMode::software)
                      : (events ? OpampMode::event : OpampMode::off);
    }

    /// Rewrite CTRLA with what it already holds. Nothing changes except
    /// the thing the write itself does: the settle timer RESTARTS
    /// (35.3.2.6). The verb an app uses to re-arm SETTLED and the READYn
    /// event without touching the configuration.
    static void restart() { const uint8_t c = ctrla(); ctrla() = c; }

    /// EVENTEN alone, keeping ALWAYSON. With it clear the op amp neither
    /// hears an event nor issues READYn.
    static void events(bool on) {
        if (on) ctrla() |= OPAMP_EVENTEN_bm;
        else ctrla() &= static_cast<uint8_t>(~OPAMP_EVENTEN_bm);
    }

    static void output(OpampOutput m) {
        ctrla() = static_cast<uint8_t>((ctrla() & static_cast<uint8_t>(~OPAMP_OUTMODE_gm)) |
                                       (static_cast<uint8_t>(m) << OPAMP_OUTMODE_gp));
    }
    static OpampOutput output() {
        return static_cast<OpampOutput>((ctrla() & OPAMP_OUTMODE_gm) >> OPAMP_OUTMODE_gp);
    }

    static void run_standby(bool on) {
        if (on) ctrla() |= OPAMP_RUNSTBY_bm;
        else ctrla() &= static_cast<uint8_t>(~OPAMP_RUNSTBY_bm);
    }

    // ---- the multiplexers, live ------------------------------------------------

    /// Move an input or the ladder under a running op amp. Each of these
    /// writes RESTARTS the settle timer and glitches the output while
    /// the analog switches change over (35.3.2.1): wait_settled() after.
    static bool positive(OpampPos p) {
        if (p == OpampPos::link_out && n == 0) return false;
        if (p == OpampPos::link_wip && n != 2) return false;
        inmux() = static_cast<uint8_t>((inmux() & static_cast<uint8_t>(~OPAMP_MUXPOS_gm)) |
                                       (static_cast<uint8_t>(p) << OPAMP_MUXPOS_gp));
        return true;
    }
    static void negative(OpampNeg p) {
        inmux() = static_cast<uint8_t>((inmux() & static_cast<uint8_t>(~OPAMP_MUXNEG_gm)) |
                                       (static_cast<uint8_t>(p) << OPAMP_MUXNEG_gp));
    }
    static void wiper(OpampWiper w) {
        resmux() = static_cast<uint8_t>((resmux() & static_cast<uint8_t>(~OPAMP_MUXWIP_gm)) |
                                        (static_cast<uint8_t>(w) << OPAMP_MUXWIP_gp));
    }
    static OpampWiper wiper() {
        return static_cast<OpampWiper>((resmux() & OPAMP_MUXWIP_gm) >> OPAMP_MUXWIP_gp);
    }
    static void ladder(OpampTop t, OpampBot b, OpampWiper w) {
        resmux() = static_cast<uint8_t>((static_cast<uint8_t>(t) << OPAMP_MUXTOP_gp) |
                                        (static_cast<uint8_t>(b) << OPAMP_MUXBOT_gp) |
                                        (static_cast<uint8_t>(w) << OPAMP_MUXWIP_gp));
    }

    // ---- the internal timer -----------------------------------------------------

    /// SETTLE[6:0]: microseconds of TIMEBASE ticks allowed for the
    /// output to settle after warm-up. Writing it does NOT restart the
    /// timer (only CTRLA / INMUX / RESMUX do).
    static void settle_us(uint8_t us) { settle_reg() = static_cast<uint8_t>(us & OPAMP_SETTLE_gm); }
    static uint8_t settle_us() { return static_cast<uint8_t>(settle_reg() & OPAMP_SETTLE_gm); }

    /// STATUS.SETTLED: warm-up and settle time are over. Cleared while
    /// the op amp waits, including after any configuration write.
    static bool settled() { return (status() & OPAMP_SETTLED_bm) != 0; }
    /// Blocking wait. Never returns while the op amp is disabled - the
    /// timer only runs when it is on, which is the caller's business.
    static void wait_settled() { while (!settled()) {} }

    // ---- offset calibration (35.3.2.8) --------------------------------------------

    /// OPnCAL. Out of reset it holds the production value from fuses;
    /// 0x00 is the most negative trim, 0x80 none, 0xFF the most
    /// positive, one step 0.5 mV typical (39-27). Writing it does not
    /// restart the settle timer.
    static uint8_t cal() { return cal_reg(); }
    static void cal(uint8_t v) { cal_reg() = v; }

    // ---- events ---------------------------------------------------------------------

    /// Enable / disable this op amp from a channel. EDGE-detected: a
    /// software pulse on the channel is enough. Both need EVENTEN, and
    /// only the `event` regime lets them decide (in
    /// software_with_events they are ignored, 35.3.2.7).
    template <uint8_t ch>
    static void enable_on(EventChannel<ch> c) { EnableIn::listen(c); events(true); }
    template <uint8_t ch>
    static void disable_on(EventChannel<ch> c) { DisableIn::listen(c); events(true); }

    /// DUMP: close the switch from VOUT to VINN for as long as the
    /// channel's LEVEL is high - the integrator's reset (figure 35-6).
    /// A pulse does nothing: the user is a level, not an edge.
    template <uint8_t ch>
    static void dump_on(EventChannel<ch> c) { DumpIn::listen(c); events(true); }
    /// DRIVE: raise the output driver for as long as the channel's LEVEL
    /// is high, whatever OUTMODE says. Also a level.
    template <uint8_t ch>
    static void drive_on(EventChannel<ch> c) { DriveIn::listen(c); events(true); }

    /// Stop listening (each user separately - `events(false)` silences
    /// all four at once but also stops READYn).
    static void enable_off() { EnableIn::unlisten(); }
    static void disable_off() { DisableIn::unlisten(); }
    static void dump_off() { DumpIn::unlisten(); }
    static void drive_off() { DriveIn::unlisten(); }

    // ---- the registers ---------------------------------------------------------------

    /// OPnCTRLA .. OPnCAL: six registers per instance on a stride of
    /// eight from OP0CTRLA (35.4). Pointer arithmetic, because the
    /// device headers of packages without OP2 do not name its members.
    static volatile uint8_t& ctrla()      { return (&OPAMP.OP0CTRLA)[8 * n + 0]; }
    static volatile uint8_t& status()     { return (&OPAMP.OP0CTRLA)[8 * n + 1]; }
    static volatile uint8_t& resmux()     { return (&OPAMP.OP0CTRLA)[8 * n + 2]; }
    static volatile uint8_t& inmux()      { return (&OPAMP.OP0CTRLA)[8 * n + 3]; }
    static volatile uint8_t& settle_reg() { return (&OPAMP.OP0CTRLA)[8 * n + 4]; }
    static volatile uint8_t& cal_reg()    { return (&OPAMP.OP0CTRLA)[8 * n + 5]; }

private:
    static constexpr uint8_t mode_bits(OpampMode m) {
        switch (m) {
            case OpampMode::off: return 0;
            case OpampMode::software: return OPAMP_ALWAYSON_bm;
            case OpampMode::event: return OPAMP_EVENTEN_bm;
            case OpampMode::software_with_events: return OPAMP_ALWAYSON_bm | OPAMP_EVENTEN_bm;
        }
        return 0;
    }
    static constexpr uint8_t ctrla_byte(const OpampConfig& c) {
        return static_cast<uint8_t>(
            mode_bits(c.mode) |
            (static_cast<uint8_t>(c.output) << OPAMP_OUTMODE_gp) |
            (c.run_standby ? OPAMP_RUNSTBY_bm : 0));
    }
    /// Digital input buffer off on a pad the op amp uses (nothing for
    /// {0, 0}: a package that does not bond it cannot be configured to
    /// use it either - opamp_config_valid refused first).
    static void analog_input(OpampPin p) {
        if (p.port == 0) return;
        volatile uint8_t& ctrl = pinctrl_of(p.port, p.pin);
        ctrl = static_cast<uint8_t>((ctrl & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc);
    }
    static void digital_input(OpampPin p) {
        if (p.port == 0) return;
        volatile uint8_t& ctrl = pinctrl_of(p.port, p.pin);
        ctrl = static_cast<uint8_t>((ctrl & ~PORT_ISC_gm) | PORT_ISC_INTDISABLE_gc);
    }

    static inline uint8_t claimed_ = 0;   ///< which pads the last init() took
};

// ---- tasks ---------------------------------------------------------------------------

/// OpampFollower<Op>: a unity-gain buffer (figure 35-3). MUXNEG = OUT,
/// the ladder off; the source is any MUXPOS the instance offers - the
/// INP pad by default, but VDD/2, the DAC or the previous op amp's
/// output just as well.
///
///   using Buf = brio::OpampFollower<brio::Opamp<0>>;
///   Buf::init();                 // PD1 in, PD2 out
///   Buf::wait_settled();
template <typename Op>
struct OpampFollower {
    OpampFollower() = delete;

    static bool init(OpampPos source = OpampPos::inp, uint8_t settle_us = 127,
                     OpampOutput out = OpampOutput::normal,
                     OpampMode mode = OpampMode::software) {
        return Op::init({.positive = source, .negative = OpampNeg::out,
                         .top = OpampTop::off, .bottom = OpampBot::off,
                         .wiper = OpampWiper::wip0, .output = out, .mode = mode,
                         .settle_us = settle_us});
    }
    static constexpr OpampGain gain() { return {1, 1}; }
    static bool settled() { return Op::settled(); }
    static void wait_settled() { Op::wait_settled(); }
    static void release() { Op::release(); }
};

/// OpampPga<Op>: the non-inverting PGA of figure 35-4 - VOUT = VIN x
/// (1 + R2/R1), the ladder from the output down to ground with the
/// wiper on the inverting input. gain() is the exact rational the
/// chosen wiper makes; for_gain() picks the wiper for a gain the
/// ladder can make, and refuses one it cannot.
///
///   using Amp = brio::OpampPga<brio::Opamp<0>>;
///   Amp::init(brio::OpampWiper::wip3);         // x2 from PD1 to PD2
template <typename Op>
struct OpampPga {
    OpampPga() = delete;

    static bool init(OpampWiper w, OpampPos source = OpampPos::inp,
                     uint8_t settle_us = 127, OpampOutput out = OpampOutput::normal,
                     OpampMode mode = OpampMode::software) {
        wiper_ = w;
        return Op::init({.positive = source, .negative = OpampNeg::wiper,
                         .top = OpampTop::out, .bottom = OpampBot::gnd,
                         .wiper = w, .output = out, .mode = mode, .settle_us = settle_us});
    }
    /// The gain the ladder makes, exactly: 16/15, 8/7, 4/3, 2, 8/3, 4, 8, 16.
    static constexpr OpampGain gain_of(OpampWiper w) { return opamp_noninverting_gain(w); }
    static OpampGain gain() { return opamp_noninverting_gain(wiper_); }
    /// The wiper for an exact gain, or nothing (the ladder is discrete).
    static constexpr std::optional<OpampWiper> for_gain(int16_t num, int16_t den = 1) {
        return opamp_noninverting_wiper(num, den);
    }
    /// Change gain under a running op amp: a glitch and a new settling.
    static void set(OpampWiper w) { wiper_ = w; Op::wiper(w); }
    static bool settled() { return Op::settled(); }
    static void wait_settled() { Op::wait_settled(); }
    static void release() { Op::release(); }

private:
    static inline OpampWiper wiper_ = OpampWiper::wip0;
};

/// OpampInvertingPga<Op>: figure 35-5 - VOUT = -(R2/R1)(VIN - VDD/2) +
/// VDD/2, the non-inverting input tied to VDD/2 and the ladder's bottom
/// carrying the signal. The default `source` is the INN pad, as the
/// chapter draws it; MUXBOT also takes the DAC, which needs no wire.
template <typename Op>
struct OpampInvertingPga {
    OpampInvertingPga() = delete;

    static bool init(OpampWiper w, OpampBot source = OpampBot::inn,
                     uint8_t settle_us = 127, OpampOutput out = OpampOutput::normal,
                     OpampMode mode = OpampMode::software) {
        wiper_ = w;
        return Op::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::wiper,
                         .top = OpampTop::out, .bottom = source,
                         .wiper = w, .output = out, .mode = mode, .settle_us = settle_us});
    }
    /// The gain, exactly and negative: -1/15, -1/7, -1/3, -1, -5/3, -3, -7, -15.
    static constexpr OpampGain gain_of(OpampWiper w) { return opamp_inverting_gain(w); }
    static OpampGain gain() { return opamp_inverting_gain(wiper_); }
    static constexpr std::optional<OpampWiper> for_gain(int16_t num, int16_t den = 1) {
        return opamp_inverting_wiper(num, den);
    }
    static void set(OpampWiper w) { wiper_ = w; Op::wiper(w); }
    static bool settled() { return Op::settled(); }
    static void wait_settled() { Op::wait_settled(); }
    static void release() { Op::release(); }

private:
    static inline OpampWiper wiper_ = OpampWiper::wip0;
};

// ---- the instrumentation amplifier (35.3.7, figure 35-12) -----------------------------

/// The seven gains the chapter's instrumentation amplifier can make
/// (table 35-14). They are not a selection: the recipe needs
/// R1(OP2) = 16R/(1 + G) and R1(OP0) = 16R x G/(1 + G) to BOTH be
/// wiper positions, and exactly these seven satisfy it.
enum class InstrumentationGain : uint8_t {
    div15,   ///< 1/15
    div7,    ///< 1/7
    div3,    ///< 1/3
    unity,   ///< 1
    x3,
    x7,
    x15,
};

constexpr OpampGain instrumentation_gain(InstrumentationGain g) {
    switch (g) {
        case InstrumentationGain::div15: return {1, 15};
        case InstrumentationGain::div7:  return {1, 7};
        case InstrumentationGain::div3:  return {1, 3};
        case InstrumentationGain::unity: return {1, 1};
        case InstrumentationGain::x3:    return {3, 1};
        case InstrumentationGain::x7:    return {7, 1};
        case InstrumentationGain::x15:   return {15, 1};
    }
    return {1, 1};
}
/// Table 35-14's two columns.
constexpr OpampWiper instrumentation_wiper_op0(InstrumentationGain g) {
    constexpr OpampWiper w[] = {OpampWiper::wip7, OpampWiper::wip6, OpampWiper::wip5,
                                OpampWiper::wip3, OpampWiper::wip2, OpampWiper::wip1,
                                OpampWiper::wip0};
    return w[static_cast<uint8_t>(g)];
}
constexpr OpampWiper instrumentation_wiper_op2(InstrumentationGain g) {
    constexpr OpampWiper w[] = {OpampWiper::wip0, OpampWiper::wip1, OpampWiper::wip2,
                                OpampWiper::wip3, OpampWiper::wip5, OpampWiper::wip6,
                                OpampWiper::wip7};
    return w[static_cast<uint8_t>(g)];
}

/// InstrumentationAmp: the three-op-amp recipe of figure 35-12 and
/// tables 35-13/35-14. OP0 buffers V2 and divides its output to the
/// wiper OP2's positive input takes (LINKWIP); OP1 buffers V1 and
/// becomes the bottom of OP2's ladder (LINKOUT); OP2 closes the loop.
/// The output is OP2OUT = gain x (V2 - V1), referenced to ground -
/// which is why the useful window is the op amp's output swing,
/// 0.15 V .. VDD - 0.15 V.
///
/// The two inputs default to the INP pads the chapter draws, but any
/// MUXPOS source works for either (VDD/2 and the DAC are the wireless
/// ones): `init(gain, v2_source, v1_source)`.
///
/// Only on packages with OP2 - the type does not exist otherwise, so a
/// 28/32-pin build fails to compile rather than half-working.
template <typename Op0 = Opamp<0>, typename Op1 = Opamp<1>, typename Op2 = Opamp<2>>
struct InstrumentationAmp {
    InstrumentationAmp() = delete;
    static_assert(opamp_count >= 3,
                  "the instrumentation amplifier needs OP2: a 48- or 64-pin part");

    /// `observe_stages` raises the output drivers of OP0 and OP1 as
    /// well, so V2 and V1 can be read on their own pads. The recipe
    /// does not need them - the two links are internal - and table
    /// 35-13 says nothing about OUTMODE; it is a measurement aid, and
    /// it costs those two pads.
    static bool init(InstrumentationGain g,
                     OpampPos v2_source = OpampPos::inp,
                     OpampPos v1_source = OpampPos::inp,
                     uint8_t settle_us = 127,
                     bool observe_stages = false) {
        gain_ = g;
        const OpampOutput stage = observe_stages ? OpampOutput::normal : OpampOutput::off;
        // OP0: follower of V2, its ladder a divider from OP0OUT to GND.
        if (!Op0::init({.positive = v2_source, .negative = OpampNeg::out,
                        .top = OpampTop::out, .bottom = OpampBot::gnd,
                        .wiper = instrumentation_wiper_op0(g),
                        .output = stage, .mode = OpampMode::software,
                        .settle_us = settle_us})) return false;
        // OP1: plain follower of V1, ladder off.
        if (!Op1::init({.positive = v1_source, .negative = OpampNeg::out,
                        .top = OpampTop::off, .bottom = OpampBot::off,
                        .wiper = OpampWiper::wip0,
                        .output = stage, .mode = OpampMode::software,
                        .settle_us = settle_us})) return false;
        // OP2: positive input = OP0's wiper, ladder from OP2OUT down to
        // OP1OUT with the wiper on the inverting input.
        return Op2::init({.positive = OpampPos::link_wip, .negative = OpampNeg::wiper,
                          .top = OpampTop::out, .bottom = OpampBot::link_out,
                          .wiper = instrumentation_wiper_op2(g),
                          .output = OpampOutput::normal, .mode = OpampMode::software,
                          .settle_us = settle_us});
    }

    /// The exact gain of the configuration in force.
    static OpampGain gain() { return instrumentation_gain(gain_); }
    static constexpr OpampGain gain_of(InstrumentationGain g) { return instrumentation_gain(g); }

    /// Change gain: both wipers move together, and both op amps glitch
    /// and settle again.
    static void set(InstrumentationGain g) {
        gain_ = g;
        Op0::wiper(instrumentation_wiper_op0(g));
        Op2::wiper(instrumentation_wiper_op2(g));
    }

    static bool settled() { return Op0::settled() && Op1::settled() && Op2::settled(); }
    static void wait_settled() { Op0::wait_settled(); Op1::wait_settled(); Op2::wait_settled(); }
    static void release() { Op2::release(); Op1::release(); Op0::release(); }

private:
    static inline InstrumentationGain gain_ = InstrumentationGain::unity;
};

}  // namespace brio

#endif  // OPAMP
