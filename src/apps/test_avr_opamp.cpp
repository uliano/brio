// test_avr_opamp - the OPAMP test SUITE for the AVR DB target:
// avrdx/opamp.hpp (the block, the three op amps, both input
// multiplexers, the resistor ladder, the output driver, the internal
// timer with its READY event, the four event users and the offset
// calibration) and the tasks built on it.
//
// Reference test of that header (docs/avrdx/opamp.md): keep it passing.
//
// NOTHING TO WIRE. The whole instrument is inside the chip: the DAC
// (buffered output, which is the one the op amps tap) is the signal
// source, the ADC measures every op amp output ON ITS OWN PAD, the
// event system carries READY into a TCB that counts CLK_PER ticks, and
// a PORT pin drives the level the DUMP and DRIVE event users need.
// Everything is ratiometric: the DAC and the ADC both run on VDD, so a
// DAC code c aims at ADC count 4c whatever the rail is.
//
// PINS IT CLAIMS, all of PORTD plus three of PORTE:
//   PD0  the DUMP/DRIVE level source (driven from PORT, read as an event)
//   PD1  OP0INP    PD2  OP0OUT (AIN2)    PD3  OP0INN
//   PD4  OP1INP    PD5  OP1OUT (AIN5)    PD7  OP1INN
//   PD6  DAC0 OUT (AIN6) - the source, buffer ON as the op amps require
//   PE1  OP2INP    PE2  OP2OUT (AIN10)   PE3  OP2INN
// PE0 is NEVER touched: on this desk it is the wire to the other board.
//
// Event channels 0 (READY), 1 (the ENABLE strobe), 2 (the DISABLE
// strobe) and 4 (the PD0 level, PORTD pins are channels 2-3... see the
// EVSYS legality rule - PORTD pins go on channels 2 and 3, so the level
// rides channel 3).
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800.
//
// Commands: ? | a the block, the registers and the errata | b the
// voltage follower | c the non-inverting gain table | d the inverting
// gain table | e the internal sources and the op-to-op links | f the
// instrumentation amplifier | g the internal timer, the READY event and
// a clock rebase | h offset calibration | i the four event users
// | z = a..i

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/adc.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/dac.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/opamp.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "avrdx/vref.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = AvrPlatform;
using SysClock = Clock<ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using Meter = Adc<0>;
using Source = Dac<0>;

/// The stopwatch that latches the READY event in hardware.
using Ready = Tcb<0>;

using ChReady = EventChannel<0>;    ///< OPAMP READY -> the TCB capture
using ChEnable = EventChannel<1>;   ///< software strobe -> ENABLE (edge)
using ChDisable = EventChannel<2>;  ///< software strobe -> DISABLE (edge)
using ChLevel = EventChannel<3>;    ///< PD0 -> DUMP / DRIVE (level); PORTD pins are 2-3

/// The clock the rebase leg needs. Everything else runs on the static
/// one; the users are exactly the drivers whose arithmetic must follow.
using DynClock = DynamicClock<SysClock, Serial, Meter, OpampSystem>;

// ---- the pads ------------------------------------------------------------------
using Op0Inp = Pin<'D', 1>;
using Op0Out = Pin<'D', 2>;
using Op0Inn = Pin<'D', 3>;
using Op1Inp = Pin<'D', 4>;
using Op1Out = Pin<'D', 5>;
using Op1Inn = Pin<'D', 7>;
using Op2Inp = Pin<'E', 1>;
using Op2Out = Pin<'E', 2>;
using Op2Inn = Pin<'E', 3>;
using DacPad = Pin<'D', 6>;
using LevelPin = Pin<'D', 0>;

using Op0 = Opamp<0>;
using Op1 = Opamp<1>;
using Op2 = Opamp<2>;

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

// ---- the test harness -------------------------------------------------------------
// The letter registry, the verdict lines and the two tallies bench.py
// reads all live in util/testbench.hpp; these two are the shorthand the
// tests below call.
TestBench<Serial> bench;

uint16_t vdd_mv_ = 5000;          ///< measured once at boot, for the mV columns
uint8_t cal_at_boot[3] = {0, 0, 0};
uint8_t timebase_at_boot = 0;

void verdict(const char* name, bool ok) { bench.verdict(name, ok); }
void verdict(const char* a, const char* b, bool ok) { bench.verdict(a, b, ok); }
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
/// |a - b| <= permille of |b|, the way an analog tolerance is stated.
bool near_permille(int32_t a, int32_t b, int32_t permille) {
    const int32_t ref = b >= 0 ? b : -b;
    const int32_t tol = (ref * permille + 500) / 1000;
    return near(a, b, tol < 1 ? 1 : tol);
}

// ---- measuring -------------------------------------------------------------------

/// A count threshold in the x256 units everything below is measured in.
/// A function, not `n * 256`: `int` is SIXTEEN bits here and 2048 * 256
/// would quietly be zero.
constexpr int32_t counts_x(int32_t counts) { return counts * 256; }

/// Counts x 256: `reps` accumulations of 16 samples each, averaged.
/// The extra eight bits are real - the residual an offset measurement
/// looks for is a fraction of one 12-bit count.
template <typename Pad>
uint32_t measure_x256(uint8_t reps = 8) {
    Meter::select(AnalogIn<Pad>{});
    (void)Meter::read();                       // the first after a mux change
    uint32_t total = 0;
    for (uint8_t i = 0; i < reps; ++i) total += Meter::read();
    return (total << 8) / (16u * reps);
}
/// Counts (12-bit full scale = VDD) as millivolts. The x256 value is
/// dropped to x16 first: 4095 x 256 x 5000 overflows 32 bits, and a
/// sixteenth of a count is far below what any of this resolves.
uint16_t counts_mv(int32_t counts_x256) {
    const int32_t c = counts_x256 >= 0 ? counts_x256 : -counts_x256;
    const uint32_t mv = ((static_cast<uint32_t>(c) >> 4) * vdd_mv_) / (4096UL * 16UL);
    return static_cast<uint16_t>(mv);
}
/// The same in MICROvolts: the offset residual this suite chases is a
/// fraction of one 12-bit count, and one count is 1.2 mV at a 5 V rail.
int32_t counts_uv_signed(int32_t counts_x256) {
    const int32_t c = counts_x256 >= 0 ? counts_x256 : -counts_x256;
    const int32_t uv = static_cast<int32_t>(
        (static_cast<uint32_t>(c) * vdd_mv_ / 256u) * 1000u / 4096u);
    return counts_x256 >= 0 ? uv : -uv;
}
int16_t counts_mv_signed(int32_t counts_x256) {
    const int16_t m = static_cast<int16_t>(counts_mv(counts_x256));
    return counts_x256 >= 0 ? m : static_cast<int16_t>(-m);
}

void adc_normal() {
    (void)Meter::reconfigure(clock, {.reference = Ref::vdd,
                                     .reference_always_on = true,
                                     .prescaler = AdcPresc::div16,
                                     .sample_length = 16,
                                     .accumulate = 16});
    delay_us(clock, 500);
}

uint16_t read_vdd_mv() {
    (void)Meter::reconfigure(clock, {.reference = Ref::v2048,
                                     .reference_always_on = true,
                                     .prescaler = AdcPresc::div16,
                                     .sample_length = 32,
                                     .accumulate = 16});
    delay_us(clock, 1000);
    Meter::select(AdcInput::vdd_div10);
    (void)Meter::read();
    uint32_t total = 0;
    for (uint8_t i = 0; i < 8; ++i) total += Meter::read();
    const uint16_t mv = adc_mv(total / 8u, 4096UL * 16UL, 2048);
    adc_normal();
    return static_cast<uint16_t>(mv * 10u);
}

/// The source: a DAC code, given time to settle. The buffered output
/// RISES in tens of microseconds but FALLS at the sink limit - about
/// 1 uA into the pad's capacitance (dac.md) - so a downward step is
/// given several times longer.
uint16_t source_code_ = 0;
void source(uint16_t code) {
    const bool down = code < source_code_;
    Source::set(code);
    source_code_ = code;
    delay_us(clock, down ? 6000 : 1500);
}

// ---- housekeeping ----------------------------------------------------------------

void quiesce() {
    Op0::release();
    Op1::release();
    Op2::release();
    OpampSystem::disable();
    ChReady::off();
    ChEnable::off();
    ChDisable::off();
    ChLevel::off();
    Op0::enable_off();
    Op0::disable_off();
    Op0::dump_off();
    Op0::drive_off();
    Ready::disable();
    LevelPin::clear();
    LevelPin::input();
    Op0Out::configure({});                  // pull-up off, buffer back on
    adc_normal();
}

/// The block, at 24 MHz, ready for an op amp.
void system_up() {
    OpampSystem::init(clock);
}

const char* wiper_name(OpampWiper w) {
    constexpr const char* n[] = {"WIP0", "WIP1", "WIP2", "WIP3",
                                 "WIP4", "WIP5", "WIP6", "WIP7"};
    return n[static_cast<uint8_t>(w)];
}

void print_gain(OpampGain g) {
    print(serial, g.num);
    if (g.den != 1) print(serial, "/", g.den);
}

// ---- a: the block, the registers and the errata ------------------------------------
void ta_block() {
    print(serial, "a the block, the register faces, and what this silicon does with "
                  "IRSEL", crlf);
    quiesce();

    print(serial, "  opamp_count = ", opamp_count, " (the device header's OP2 registers "
                  "decide), VDD ", vdd_mv_, " mV", crlf);
    verdict("this 48-pin part has three op amps", opamp_count == 3);

    // TIMEBASE: reset value and the chapter's arithmetic.
    print(serial, "  TIMEBASE at boot ", timebase_at_boot, " (reset value 0x01 per 35.5.3)",
          crlf);
    verdict("TIMEBASE comes out of reset at 1", timebase_at_boot == 1);
    system_up();
    print(serial, "  TIMEBASE for 24 MHz = ", OpampSystem::timebase(),
          " (one tick = ", opamp_timebase_ns(SysClock::hz), " ns)", crlf);
    verdict("TIMEBASE = ceil(CLK_PER / 1 MHz) - 1 = 23", OpampSystem::timebase() == 23);
    OpampSystem::rebase(12'000'000u);
    verdict("rebase(12 MHz) writes 11", OpampSystem::timebase() == 11);
    OpampSystem::rebase(SysClock::hz);
    verdict("and back to 23", OpampSystem::timebase() == 23);
    OpampSystem::timebase(0x7F);
    verdict("TIMEBASE is 7 bits (0x7F reads back whole)", OpampSystem::timebase() == 0x7F);
    OpampSystem::timebase(opamp_timebase(SysClock::hz));

    verdict("the block reads back enabled", OpampSystem::enabled());
    OpampSystem::disable();
    verdict("and disabled", !OpampSystem::enabled());
    OpampSystem::enable();

    // The per-instance register file: stride 8, six registers, and the
    // reserved bits of each.
    Op0::inmux() = 0xFF;
    Op1::inmux() = 0x00;
    print(serial, "  OP0INMUX written 0xFF reads ", hex(Op0::inmux()),
          " (bit 7 is reserved), OP1INMUX ", hex(Op1::inmux()), crlf);
    verdict("the three instances are 8 bytes apart and do not alias",
            Op0::inmux() != 0x00 && Op1::inmux() == 0x00);
    Op2::inmux() = 0x55;
    verdict("OP2's register file is reachable too", Op2::inmux() == 0x55);
    Op0::inmux() = 0;
    Op1::inmux() = 0;
    Op2::inmux() = 0;

    Op0::resmux() = 0xFF;
    print(serial, "  OP0RESMUX written 0xFF reads ", hex(Op0::resmux()),
          " (MUXTOP 0x3 and MUXBOT 0x6/0x7 are reserved codes)", crlf);
    Op0::resmux() = 0;

    Op0::settle_us(0x7F);
    verdict("SETTLE is 7 bits", Op0::settle_us() == 0x7F);
    Op0::settle_us(0xFF);
    verdict("and the eighth bit does not stick", Op0::settle_us() == 0x7F);

    // OUTMODE's reserved codes: the enum has no name for them, the
    // register does. What does the silicon keep?
    Op0::ctrla() = static_cast<uint8_t>(0x3u << OPAMP_OUTMODE_gp);
    print(serial, "  OPnCTRLA with the reserved OUTMODE 0x3 reads ", hex(Op0::ctrla()), crlf);
    Op0::ctrla() = 0;

    // CAL: the production value from the fuses (35.5.10).
    print(serial, "  OPnCAL at boot: OP0 ", hex(cal_at_boot[0]), ", OP1 ",
          hex(cal_at_boot[1]), ", OP2 ", hex(cal_at_boot[2]),
          " (0x80 = no trim, one step 0.5 mV)", crlf);
    verdict("the calibration bytes are programmed, not blank",
            cal_at_boot[0] != 0xFF && cal_at_boot[1] != 0xFF && cal_at_boot[2] != 0xFF);
    const uint8_t keep = Op0::cal();
    Op0::cal(0x5A);
    verdict("OPnCAL is writable", Op0::cal() == 0x5A);
    Op0::cal(keep);

    // DBGCTRL.
    OpampSystem::debug_run(true);
    verdict("DBGCTRL.DBGRUN sets", (OPAMP.DBGCTRL & OPAMP_DBGRUN_bm) != 0);
    OpampSystem::debug_run(false);

    // PWRCTRL.IRSEL - errata DS80000915F 2.8.2 says READ-ONLY on rev A4.
    const bool took_reduced = OpampSystem::reduced_input_range(true);
    const bool took_full = [] {
        const bool r = OpampSystem::reduced_input_range(false);
        return !r;
    }();
    print(serial, "  IRSEL: writing 1 reads back ", took_reduced ? "1" : "0",
          ", writing 0 reads back ", took_full ? "0" : "1",
          " on silicon rev ", hex(SYSCFG.REVID),
          took_reduced ? "  -> WRITABLE: erratum 2.8.2 does not apply to this die"
                       : "  -> READ-ONLY: erratum 2.8.2 observed", crlf);
    verdict("IRSEL reads back 0 after writing 0 (rail-to-rail either way)", took_full);
    verdict("IRSEL is writable on this silicon (errata 2.8.2 is rev. A4 only)",
            took_reduced);
    OpampSystem::reduced_input_range(false);

    // The refusals the driver owes the chapter.
    verdict("MUXPOS LINKOUT is refused on OP0",
            !Op0::positive(OpampPos::link_out));
    verdict("and accepted on OP1", Op1::positive(OpampPos::link_out));
    verdict("MUXPOS LINKWIP is refused on OP1", !Op1::positive(OpampPos::link_wip));
    verdict("and accepted on OP2", Op2::positive(OpampPos::link_wip));
    verdict("a settle time above 127 us is refused",
            !Op0::init({.settle_us = 200}));

    quiesce();
}

// ---- b: the voltage follower ---------------------------------------------------------
void tb_follower() {
    print(serial, "b the voltage follower: OP0 tracks the DAC, measured on its own pad",
          crlf);
    quiesce();
    system_up();

    verdict("OpampFollower init (source = the buffered DAC)",
            OpampFollower<Op0>::init(OpampPos::dac, 40));
    OpampFollower<Op0>::wait_settled();
    verdict("SETTLED is up after the wait", Op0::settled());

    print(serial, "    DAC code   PD6 (source)   PD2 (OP0OUT)   error", crlf);
    int32_t worst = 0;
    bool all_ok = true;
    for (uint16_t code = 100; code <= 900; code += 100) {
        source(code);
        const int32_t in = static_cast<int32_t>(measure_x256<DacPad>());
        const int32_t out = static_cast<int32_t>(measure_x256<Op0Out>());
        const int32_t err = out - in;
        if ((err < 0 ? -err : err) > worst) worst = err < 0 ? -err : err;
        if (!near(out, in, counts_x(40))) all_ok = false;
        print(serial, "    ", code, "          ", counts_mv(in), " mV        ",
              counts_mv(out), " mV        ", counts_mv_signed(err), " mV", crlf);
    }
    print(serial, "  worst follower error over the sweep: ", counts_mv(worst),
          " mV (the op amp's own offset plus the ADC's; spec +-16 mV over "
          "temperature, 39-27)", crlf);
    verdict("the follower tracks the source within 40 counts everywhere", all_ok);

    // A follower is a follower whatever the positive source is.
    verdict("re-source the follower onto VDD/2", Op0::positive(OpampPos::vdd_div2));
    Op0::restart();
    Op0::wait_settled();
    const int32_t half = static_cast<int32_t>(measure_x256<Op0Out>());
    print(serial, "  following VDD/2: ", counts_mv(half), " mV, half of VDD is ",
          vdd_mv_ / 2, " mV", crlf);
    verdict("VDD/2 comes out within its +-3 % (39-27)",
            near_permille(half, counts_x(2048), 40));

    quiesce();
}

// ---- c: the non-inverting gain table --------------------------------------------------

/// The slope of OP0's output against the measured source, over two DAC
/// points chosen so the output stays inside 0.15 V .. VDD - 0.15 V.
/// Returns the gain x 1000; 0 when the two points could not be placed.
int32_t measure_slope(uint16_t code_lo, uint16_t code_hi) {
    source(code_lo);
    const int32_t in_lo = static_cast<int32_t>(measure_x256<DacPad>());
    const int32_t out_lo = static_cast<int32_t>(measure_x256<Op0Out>());
    source(code_hi);
    const int32_t in_hi = static_cast<int32_t>(measure_x256<DacPad>());
    const int32_t out_hi = static_cast<int32_t>(measure_x256<Op0Out>());
    const int32_t din = in_hi - in_lo;
    if (din == 0) return 0;
    return ((out_hi - out_lo) * 1000) / din;
}

void tc_noninverting() {
    print(serial, "c the non-inverting PGA: every wiper of the ladder, measured", crlf);
    quiesce();
    system_up();

    print(serial, "    wiper  R1/R2   gain 1+R2/R1   measured   error", crlf);
    bool all_ok = true;
    for (uint8_t i = 0; i < 8; ++i) {
        const OpampWiper w = static_cast<OpampWiper>(i);
        const OpampGain g = opamp_noninverting_gain(w);
        const int32_t want = opamp_gain_x1000(g);
        // Place the two output points at ~500 and ~3600 counts:
        // out = 4 x code x gain, so code = out x den / (4 x num).
        const int32_t den = g.den, num = g.num;   // int is 16 bits: widen first
        const uint16_t lo = static_cast<uint16_t>((500L * den) / (4 * num));
        const uint16_t hi = static_cast<uint16_t>((3600L * den) / (4 * num));
        if (!OpampPga<Op0>::init(w, OpampPos::dac, 40)) {
            verdict("PGA init refused for ", wiper_name(w), false);
            all_ok = false;
            continue;
        }
        OpampPga<Op0>::wait_settled();
        const int32_t got = measure_slope(lo, hi);
        const bool ok = near_permille(got, want, 100);
        if (!ok) all_ok = false;
        print(serial, "    ", wiper_name(w), "   ", opamp_ladder_r1(w), "R/",
              opamp_ladder_r2(w), "R    ");
        print_gain(g);
        print(serial, " = ", want, "/1000   ", got, "/1000   ",
              ((got - want) * 1000) / want, " permille", crlf);
    }
    verdict("every ladder gain is within the +-10 % the data sheet allows "
            "(system gain accuracy, 39-27)", all_ok);

    // The task's own arithmetic, against the silicon it just drove.
    verdict("OpampPga::for_gain(2) picks WIP3",
            OpampPga<Op0>::for_gain(2) == OpampWiper::wip3);
    verdict("OpampPga::for_gain(5) refuses: the ladder has no x5",
            !OpampPga<Op0>::for_gain(5).has_value());

    quiesce();
}

// ---- d: the inverting gain table --------------------------------------------------
void td_inverting() {
    print(serial, "d the inverting PGA about VDD/2, its input the DAC on MUXBOT", crlf);
    quiesce();
    system_up();

    // The sweep stays ABOVE VDD/2 on purpose. The ladder's bottom is the
    // DAC's buffered output, and that buffer SOURCES a milliampere but
    // SINKS about a microamp (dac.md): with the input above VDD/2 the
    // output is below it and the ~75 uA of ladder current flows OUT of
    // the DAC, which is the direction it can serve. Below VDD/2 the
    // current would reverse and the DAC pin would simply be dragged up.
    print(serial, "    wiper  gain -R2/R1   measured   error", crlf);
    bool all_ok = true;
    for (uint8_t i = 0; i < 8; ++i) {
        const OpampWiper w = static_cast<OpampWiper>(i);
        const OpampGain g = opamp_inverting_gain(w);
        const int32_t want = opamp_gain_x1000(g);
        // out - 2048 = gain x (in - 2048): keep the output above 150
        // counts, so in - 2048 stays under 1898 x R1/R2.
        const int32_t r1 = opamp_ladder_r1(w);
        const int32_t r2 = opamp_ladder_r2(w);
        int32_t span = (1898 * r1) / r2;
        if (span > 1950) span = 1950;
        const uint16_t lo = static_cast<uint16_t>((2048 + 30) / 4);
        const uint16_t hi = static_cast<uint16_t>((2048 + span) / 4);
        if (!OpampInvertingPga<Op0>::init(w, OpampBot::dac, 40)) {
            verdict("inverting PGA init refused for ", wiper_name(w), false);
            all_ok = false;
            continue;
        }
        OpampInvertingPga<Op0>::wait_settled();
        const int32_t got = measure_slope(lo, hi);
        const bool ok = near_permille(got, want, 100);
        if (!ok) all_ok = false;
        print(serial, "    ", wiper_name(w), "   ");
        print_gain(g);
        print(serial, " = ", want, "/1000   ", got, "/1000   ",
              ((got - want) * 1000) / (want < 0 ? -want : want), " permille", crlf);
    }
    verdict("every inverting gain is within +-10 %", all_ok);

    // The pivot: with the input AT VDD/2 the output should be VDD/2 too,
    // whatever the gain (that is what makes VDD/2 the reference point).
    (void)OpampInvertingPga<Op0>::init(OpampWiper::wip3, OpampBot::dac, 40);
    OpampInvertingPga<Op0>::wait_settled();
    source(512);
    const int32_t pivot = static_cast<int32_t>(measure_x256<Op0Out>());
    print(serial, "  input at DAC code 512 (VDD/2): output ", counts_mv(pivot),
          " mV against VDD/2 = ", vdd_mv_ / 2, " mV", crlf);
    verdict("the inverting stage pivots about VDD/2",
            near_permille(pivot, counts_x(2048), 50));

    verdict("OpampInvertingPga::for_gain(-3) picks WIP5",
            OpampInvertingPga<Op0>::for_gain(-3) == OpampWiper::wip5);

    quiesce();
}

// ---- e: the internal sources and the op-to-op links ---------------------------------
void te_links() {
    print(serial, "e the sources that need no pin, and one op amp feeding the next", crlf);
    quiesce();
    system_up();

    // GND as a positive source: a follower of ground runs into the
    // output's own floor (VO >= 0.15 V under load, 39-27).
    verdict("follower of GND", OpampFollower<Op1>::init(OpampPos::gnd, 40));
    OpampFollower<Op1>::wait_settled();
    const int32_t at_gnd = static_cast<int32_t>(measure_x256<Op1Out>());
    print(serial, "  MUXPOS = GND: OP1OUT sits at ", counts_mv(at_gnd),
          " mV (the output swing's floor, not a true zero)", crlf);
    verdict("a follower of GND lands near the bottom rail", at_gnd < counts_x(300));

    // VDD/2 on OP1 as well - the same reference the inverting PGA uses.
    verdict("follower of VDD/2", OpampFollower<Op1>::init(OpampPos::vdd_div2, 40));
    OpampFollower<Op1>::wait_settled();
    const int32_t at_half = static_cast<int32_t>(measure_x256<Op1Out>());
    print(serial, "  MUXPOS = VDD/2: OP1OUT ", counts_mv(at_half), " mV", crlf);
    verdict("OP1's VDD/2 agrees with the rail's half",
            near_permille(at_half, counts_x(2048), 40));

    // The unity link: OP0 follows the DAC, OP1 follows OP0.
    verdict("OP0 = follower of the DAC", OpampFollower<Op0>::init(OpampPos::dac, 40));
    verdict("OP1 = follower of OP0's output (MUXPOS LINKOUT)",
            OpampFollower<Op1>::init(OpampPos::link_out, 40));
    OpampFollower<Op0>::wait_settled();
    OpampFollower<Op1>::wait_settled();
    source(500);
    const int32_t a0 = static_cast<int32_t>(measure_x256<Op0Out>());
    const int32_t a1 = static_cast<int32_t>(measure_x256<Op1Out>());
    print(serial, "  OP0OUT ", counts_mv(a0), " mV -> OP1OUT ", counts_mv(a1),
          " mV (two offsets in series)", crlf);
    verdict("the link carries the value from one op amp to the next",
            near(a1, a0, counts_x(40)));

    // The cascade of figure 35-8: x2 then x2.
    verdict("OP0 = non-inverting PGA x2 from the DAC",
            OpampPga<Op0>::init(OpampWiper::wip3, OpampPos::dac, 40));
    verdict("OP1 = non-inverting PGA x2 from OP0's output",
            OpampPga<Op1>::init(OpampWiper::wip3, OpampPos::link_out, 40));
    OpampPga<Op0>::wait_settled();
    OpampPga<Op1>::wait_settled();
    print(serial, "    DAC code   OP0OUT     OP1OUT     OP1/DAC", crlf);
    bool cascade_ok = true;
    for (uint16_t code = 100; code <= 220; code += 60) {
        source(code);
        const int32_t in = static_cast<int32_t>(measure_x256<DacPad>());
        const int32_t s0 = static_cast<int32_t>(measure_x256<Op0Out>());
        const int32_t s1 = static_cast<int32_t>(measure_x256<Op1Out>());
        const int32_t total = (s1 * 1000) / in;
        if (!near_permille(total, 4000, 100)) cascade_ok = false;
        print(serial, "    ", code, "        ", counts_mv(in), " mV   ",
              counts_mv(s0), " mV   ", counts_mv(s1), " mV   ", total, "/1000", crlf);
    }
    verdict("two x2 stages in series make x4 within +-10 %", cascade_ok);

    // The other link: OP0's WIPER into OP2's positive input.
    verdict("OP0 = follower of the DAC with a ladder to ground",
            Op0::init({.positive = OpampPos::dac, .negative = OpampNeg::out,
                       .top = OpampTop::out, .bottom = OpampBot::gnd,
                       .wiper = OpampWiper::wip3, .settle_us = 40}));
    verdict("OP2 = follower of OP0's WIPER (MUXPOS LINKWIP)",
            OpampFollower<Op2>::init(OpampPos::link_wip, 40));
    Op0::wait_settled();
    Op2::wait_settled();
    source(600);
    const int32_t whole = static_cast<int32_t>(measure_x256<Op0Out>());
    const int32_t tapped = static_cast<int32_t>(measure_x256<Op2Out>());
    print(serial, "  OP0OUT ", counts_mv(whole), " mV, its WIP3 wiper seen by OP2: ",
          counts_mv(tapped), " mV (R1/(R1+R2) = 8/16)", crlf);
    verdict("LINKWIP carries OP0's ladder tap to OP2",
            near_permille(tapped, whole / 2, 100));

    quiesce();
}

// ---- f: the instrumentation amplifier -----------------------------------------------
void tf_instrumentation() {
    print(serial, "f the instrumentation amplifier (35.3.7): V2 = the DAC, V1 = ground, "
                  "output on PE2", crlf);
    quiesce();
    system_up();

    constexpr InstrumentationGain gains[] = {
        InstrumentationGain::div15, InstrumentationGain::div7, InstrumentationGain::div3,
        InstrumentationGain::unity, InstrumentationGain::x3, InstrumentationGain::x7,
        InstrumentationGain::x15,
    };
    // V1 is OP1 following GROUND, which its output swing puts a little
    // above zero: that buys the WHOLE DAC range for V2 - with V1 at
    // VDD/2 the gain of 1/15 could not lift the output off its floor at
    // all. The slope is taken against the DIFFERENCE the op amps really
    // see, V2 - V1 measured on PD2 and PD5, so wherever V1 actually
    // sits, and whether it moves under the ladder's load, is measured
    // rather than assumed.
    print(serial, "    gain     OP0 wip  OP2 wip   V1        measured slope   error",
          crlf);
    bool all_ok = true;
    for (const InstrumentationGain g : gains) {
        const OpampGain want_g = instrumentation_gain(g);
        const int32_t want = opamp_gain_x1000(want_g);
        // observe_stages so V1 and V2 can be read on PD5 and PD2.
        if (!InstrumentationAmp<>::init(g, OpampPos::dac, OpampPos::gnd, 60, true)) {
            verdict("instrumentation init refused", false);
            all_ok = false;
            continue;
        }
        InstrumentationAmp<>::wait_settled();
        // OUT = gain x (V2 - V1): place the two V2 points so OUT covers
        // as much of 250 .. 3700 counts as the gain allows. V1 is OP1
        // at its output floor, which measures about 13 mV = 11 counts -
        // and at gain 15 that difference decides whether the top point
        // lands inside the output swing or against the rail.
        const int32_t den = want_g.den, num = want_g.num;   // int is 16 bits
        const int32_t lo_counts = 15 + (250L * den) / num;
        int32_t hi_counts = 15 + (3700L * den) / num;
        if (hi_counts > 4000) hi_counts = 4000;
        const uint16_t lo = static_cast<uint16_t>(lo_counts / 4);
        const uint16_t hi = static_cast<uint16_t>(hi_counts / 4);
        source(lo);
        const int32_t v2_lo = static_cast<int32_t>(measure_x256<Op0Out>(16));
        const int32_t v1_lo = static_cast<int32_t>(measure_x256<Op1Out>(16));
        const int32_t out_lo = static_cast<int32_t>(measure_x256<Op2Out>(16));
        source(hi);
        const int32_t v2_hi = static_cast<int32_t>(measure_x256<Op0Out>(16));
        const int32_t v1_hi = static_cast<int32_t>(measure_x256<Op1Out>(16));
        const int32_t out_hi = static_cast<int32_t>(measure_x256<Op2Out>(16));
        const int32_t d = (v2_hi - v1_hi) - (v2_lo - v1_lo);
        const int32_t got = d == 0 ? 0 : ((out_hi - out_lo) * 1000) / d;
        const bool ok = near_permille(got, want, 120);
        if (!ok) all_ok = false;
        print(serial, "    ");
        print_gain(want_g);
        print(serial, "        ", wiper_name(instrumentation_wiper_op0(g)), "     ",
              wiper_name(instrumentation_wiper_op2(g)), "      ", counts_mv(v1_lo),
              "/", counts_mv(v1_hi), " mV   ", got, "/1000        ",
              ((got - want) * 1000) / want, " permille", crlf);
    }
    verdict("the three-op-amp recipe makes all seven of table 35-14's gains "
            "within +-12 %", all_ok);

    // One end-to-end value, not a slope: OUT = gain x (V2 - V1), with
    // V1 on the well-regulated VDD/2 this time.
    (void)InstrumentationAmp<>::init(InstrumentationGain::unity, OpampPos::dac,
                                     OpampPos::vdd_div2, 60, true);
    InstrumentationAmp<>::wait_settled();
    source(800);
    const int32_t v2 = static_cast<int32_t>(measure_x256<Op0Out>());
    const int32_t v1 = static_cast<int32_t>(measure_x256<Op1Out>());
    const int32_t out = static_cast<int32_t>(measure_x256<Op2Out>());
    print(serial, "  gain 1: V2 ", counts_mv(v2), " mV, V1 ", counts_mv(v1),
          " mV, V2 - V1 = ", counts_mv(v2 - v1), " mV, OP2OUT ", counts_mv(out),
          " mV", crlf);
    verdict("at unity gain the output IS the difference of the two inputs",
            near(out, v2 - v1, counts_x(120)));

    verdict("only seven gains exist: the recipe needs BOTH R1 values to be "
            "wiper positions",
            instrumentation_wiper_op0(InstrumentationGain::unity) == OpampWiper::wip3 &&
            instrumentation_wiper_op2(InstrumentationGain::unity) == OpampWiper::wip3);

    quiesce();
}

// ---- g: the internal timer, the READY event, and a clock rebase -----------------------

/// Start OP0's settle timer with the stopwatch zeroed on the same
/// instruction stream and report the ticks the CPU counted to SETTLED.
/// Interrupts are off: at 460800 baud the console's own DRE interrupt
/// fires every 22 us, and a hundred-microsecond measurement made
/// through it wanders by whole microseconds.
uint16_t poll_settling() {
    P::CriticalSection cs;
    Ready::clear_capt();
    Ready::count(0);
    Op0::restart();                        // the settle timer starts here
    uint32_t guard = 0;
    while (!Op0::settled() && ++guard < 400000u) {
    }
    return Ready::count();
}

/// The same measurement with NO software in the path: in EVENT_ENABLED
/// mode an ENABLE event starts the op amp and the READY event latches
/// the counter. Returns 0 when READY never came.
uint16_t event_settling() {
    ChDisable::pulse();
    delay_us(clock, 300);
    Ready::clear_capt();
    Ready::count(0);
    ChEnable::pulse();
    uint32_t guard = 0;
    while (!Ready::capt_flag() && ++guard < 400000u) {
    }
    if (guard >= 400000u) return 0;
    return Ready::capture();
}

void tg_timer() {
    print(serial, "g the internal timer: SETTLE in microseconds, SETTLED, the READY "
                  "event and a clock rebase", crlf);
    quiesce();
    system_up();

    // The stopwatch: a free-running TCB at CLK_PER that latches CNT into
    // CCMP when OP0's READY event arrives. No software in the path.
    Ready::init({.mode = TcbMode::capture, .clock = TcbClock::div1,
                 .event_input = true});
    Ready::capture_on(ChReady{});
    ChReady::source(Op0::ReadyEvent{});
    Op0::enable_on(ChEnable{});
    Op0::disable_on(ChDisable{});

    // EVENT_ENABLED (ALWAYSON = 0, EVENTEN = 1): the events own the op
    // amp, and this is the mode the READY event lives in.
    verdict("OP0 as a follower of VDD/2, enabled and disabled by events",
            Op0::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::out,
                       .mode = OpampMode::event, .settle_us = 10}));

    print(serial, "    SETTLE   ENABLE event -> READY event", crlf);
    constexpr uint8_t settles[] = {10, 40, 80, 127};
    uint16_t ticks[4] = {0, 0, 0, 0};
    bool ready_always = true;
    for (uint8_t i = 0; i < 4; ++i) {
        Op0::settle_us(settles[i]);
        ticks[i] = event_settling();
        if (ticks[i] == 0) ready_always = false;
        print(serial, "    ", settles[i], " us     ", ticks[i], " ticks (",
              ticks[i] / cycles_per_us, " us)", crlf);
    }
    verdict("EVENT_ENABLED issues the READY event at every settle time",
            ready_always);

    // The sharp measurement: the DIFFERENCES are pure SETTLE, so the
    // warm-up and the strobe's own few cycles cancel.
    const int32_t d1 = static_cast<int32_t>(ticks[1]) - ticks[0];   // 30 us
    const int32_t d2 = static_cast<int32_t>(ticks[2]) - ticks[1];   // 40 us
    const int32_t d3 = static_cast<int32_t>(ticks[3]) - ticks[2];   // 47 us
    print(serial, "  SETTLE deltas: 10->40 us = ", d1, " ticks (want ",
          30 * cycles_per_us, "), 40->80 = ", d2, " (want ", 40 * cycles_per_us,
          "), 80->127 = ", d3, " (want ", 47 * cycles_per_us, ")", crlf);
    verdict("one SETTLE unit is one TIMEBASE microsecond (30 us step)",
            near(d1, 30 * static_cast<int32_t>(cycles_per_us), 40));
    verdict("...and again over 40 us",
            near(d2, 40 * static_cast<int32_t>(cycles_per_us), 40));
    verdict("...and again over 47 us",
            near(d3, 47 * static_cast<int32_t>(cycles_per_us), 40));

    // Whatever the four readings have in common on top of SETTLE is the
    // WARM-UP: the time the internal timer waits for the op amp's own
    // circuitry before it starts counting the settle time out.
    const int32_t warmup = static_cast<int32_t>(ticks[0]) -
                           10 * static_cast<int32_t>(cycles_per_us);
    print(serial, "  ENABLE event to READY beyond the programmed settle time: ",
          warmup, " ticks = ", warmup / static_cast<int32_t>(cycles_per_us),
          " us (39-27's TON, the ELECTRICAL turn-on, is 1 us typ.)", crlf);
    verdict("the warm-up the internal timer waits is a good ten microseconds, "
            "an order of magnitude more than the data sheet's TON",
            warmup > 8 * static_cast<int32_t>(cycles_per_us) &&
            warmup < 30 * static_cast<int32_t>(cycles_per_us));

    // SW_ENABLED_WITH_EVENTS: on by software, DUMP and DRIVE heard - but
    // is READY issued? 35.3.3 says any mode with EVENTEN generates
    // events; 35.3.2.6 names EVENT_ENABLED alone.
    Op0::enable_off();
    Op0::disable_off();
    verdict("OP0 in SW_ENABLED_WITH_EVENTS, same follower",
            Op0::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::out,
                       .mode = OpampMode::software_with_events, .settle_us = 80}));
    Ready::clear_capt();
    const uint16_t polled80 = poll_settling();
    const bool fired_sw = Ready::capt_flag();
    Ready::clear_capt();
    print(serial, "  SW_ENABLED_WITH_EVENTS, SETTLE 80 us: SETTLED polled at ",
          polled80, " ticks (", polled80 / cycles_per_us, " us), READY ",
          fired_sw ? "SEEN" : "NOT ISSUED", crlf);
    print(serial, "  ...and a RESTART of an op amp that is already running costs ",
          static_cast<int32_t>(polled80) -
              80 * static_cast<int32_t>(cycles_per_us),
          " ticks over its SETTLE, against the ", warmup,
          " a cold ENABLE pays: the warm-up is charged to STARTING the op amp, "
          "not to re-arming its timer", crlf);
    verdict("SETTLED itself works in this mode, and takes the 80 us it was told "
            "with NO warm-up on top",
            near(polled80, 80 * static_cast<int32_t>(cycles_per_us),
                 4 * static_cast<int32_t>(cycles_per_us)));
    verdict("READY is issued ONLY in EVENT_ENABLED mode: with ALWAYSON set the "
            "op amp hears DUMP and DRIVE but generates nothing (35.3.2.6 is "
            "exact where 35.3.3 reads wider)", !fired_sw);

    // SETTLED is cleared by a configuration write, not only by an enable.
    Op0::settle_us(100);
    Op0::restart();
    const bool cleared = !Op0::settled();
    Op0::wait_settled();
    verdict("a write to OPnCTRLA clears SETTLED and restarts the timer", cleared);
    Op0::negative(OpampNeg::out);          // an INMUX write, same effect
    const bool cleared_mux = !Op0::settled();
    Op0::wait_settled();
    verdict("a write to OPnINMUX does the same (35.3.2.6)", cleared_mux);
    Op0::ladder(OpampTop::off, OpampBot::off, OpampWiper::wip0);
    const bool cleared_res = !Op0::settled();
    Op0::wait_settled();
    verdict("and so does a write to OPnRESMUX", cleared_res);

    // With EVENTEN down there is no READY at all.
    Op0::events(false);
    Ready::clear_capt();
    Op0::restart();
    Op0::wait_settled();
    verdict("with EVENTEN clear the op amp issues no READY event",
            !Ready::capt_flag());

    // The rebase: TIMEBASE follows CLK_PER, so a settle time keeps
    // meaning the microseconds it says. Measured event to event.
    verdict("back to EVENT_ENABLED for the rebase",
            Op0::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::out,
                       .mode = OpampMode::event, .settle_us = 80}));
    Op0::enable_on(ChEnable{});
    Op0::disable_on(ChDisable{});
    const uint16_t at24 = event_settling();
    const uint32_t us24 = at24 / cycles_per_us;
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    delay_us(DynClock{}, 2000);
    print(serial, "  after the switch TIMEBASE reads ", OpampSystem::timebase(), crlf);
    verdict("TIMEBASE followed the clock down to 11", OpampSystem::timebase() == 11);
    const uint16_t at12 = event_settling();
    const uint32_t us12 = at12 / (cycles_per_us / 2);   // the TCB halved too
    verdict("back to 24 MHz", DynClock::set(SysClock::hz));
    delay_us(DynClock{}, 2000);
    const uint16_t back = event_settling();
    const uint32_t usb = back / cycles_per_us;
    print(serial, "  SETTLE 80 us: 24 MHz -> ", at24, " ticks (", us24,
          " us); 12 MHz -> ", at12, " ticks (", us12, " us); back -> ",
          back, " ticks (", usb, " us)", crlf);
    verdict("the settle time holds in TIME across the rebase",
            near(static_cast<int32_t>(us12), static_cast<int32_t>(us24), 4));
    verdict("and the tick count HALVES, because the ruler halved with it",
            near(at12, at24 / 2, 40));
    verdict("back at 24 MHz the reading returns", near(back, at24, 40));
    verdict("TIMEBASE is 23 again", OpampSystem::timebase() == 23);

    ChReady::off();
    Ready::disable();
    quiesce();
}

// ---- h: offset calibration -------------------------------------------------------------

/// The follower's residual (OP0OUT - the source at its own pad), in
/// counts x 256, averaged over a spread of source levels so the ADC's
/// quantization decorrelates and a fraction of a count becomes visible.
int32_t follower_residual() {
    int32_t sum = 0;
    constexpr uint16_t codes[] = {200, 350, 500, 650, 800};
    for (const uint16_t c : codes) {
        source(c);
        // out, in, out: any residual drift of the source between the two
        // pads averages out instead of counting as offset.
        const int32_t o1 = static_cast<int32_t>(measure_x256<Op0Out>(8));
        const int32_t in = static_cast<int32_t>(measure_x256<DacPad>(12));
        const int32_t o2 = static_cast<int32_t>(measure_x256<Op0Out>(8));
        sum += (o1 + o2) / 2 - in;
    }
    return sum / 5;
}

void th_calibration() {
    print(serial, "h offset calibration (35.3.2.8): measure it, trim it, measure again",
          crlf);
    quiesce();
    system_up();

    const uint8_t production = cal_at_boot[0];
    verdict("OP0 as a voltage follower of the DAC, the chapter's calibration shape",
            OpampFollower<Op0>::init(OpampPos::dac, 60));
    OpampFollower<Op0>::wait_settled();

    Op0::cal(production);
    const int32_t before = follower_residual();
    print(serial, "  at the production CAL ", hex(production), ": residual ", before,
          " counts x256 = ", counts_uv_signed(before), " uV", crlf);

    // The step size, AMPLIFIED: a x16 PGA turns 0.5 mV of input offset
    // into 8 mV of output, which the ADC can see.
    verdict("OP0 as a x16 non-inverting PGA, to amplify the trim's own step",
            OpampPga<Op0>::init(OpampWiper::wip7, OpampPos::dac, 60));
    OpampPga<Op0>::wait_settled();
    source(40);
    Op0::cal(0x40);
    delay_us(clock, 2000);
    const int32_t at40 = static_cast<int32_t>(measure_x256<Op0Out>(12));
    Op0::cal(0xC0);
    delay_us(clock, 2000);
    const int32_t atC0 = static_cast<int32_t>(measure_x256<Op0Out>(12));
    const int32_t swing = atC0 - at40;                       // 128 CAL steps, x16
    const int32_t step_uv = (static_cast<int32_t>(counts_mv(swing)) * 1000) / (128 * 16);
    print(serial, "  CAL 0x40 -> ", counts_mv(at40), " mV, CAL 0xC0 -> ",
          counts_mv(atC0), " mV at gain 16: the output moved ", counts_mv(swing),
          " mV, so 128 steps move the INPUT by ", counts_mv(swing) / 16,
          " mV and one step is ", step_uv, " uV (39-27 says 500 uV), direction ",
          swing > 0 ? "UP with CAL" : "DOWN with CAL", crlf);
    // 35.5.10 calls 0x00 "the most negative value of offset adjustment"
    // and 0xFF "the most positive". At the OUTPUT of a non-inverting
    // stage this die does the opposite: a rising CAL lowers the output,
    // i.e. the trim is applied with the sign of the INVERTING input.
    verdict("a rising CAL moves the output DOWN - the trim's sign at the pad is "
            "the opposite of the plain reading of 35.5.10", swing < 0);
    verdict("one calibration step is around half a millivolt",
            step_uv > 250 && step_uv < 900);

    // Trim: scan CAL at unity gain and keep the value with the smallest
    // residual. Coarse then fine, because a full 256-step scan would
    // cost minutes.
    verdict("back to the follower for the trim", OpampFollower<Op0>::init(OpampPos::dac, 60));
    OpampFollower<Op0>::wait_settled();
    uint8_t best = production;
    int32_t best_abs = before < 0 ? -before : before;
    for (int16_t c = 0; c <= 255; c += 16) {
        Op0::cal(static_cast<uint8_t>(c));
        delay_us(clock, 2000);
        const int32_t r = follower_residual();
        const int32_t a = r < 0 ? -r : r;
        if (a < best_abs) { best_abs = a; best = static_cast<uint8_t>(c); }
    }
    const uint8_t coarse = best;
    for (int16_t c = coarse - 15; c <= coarse + 15; c += 3) {
        if (c < 0 || c > 255) continue;
        Op0::cal(static_cast<uint8_t>(c));
        delay_us(clock, 2000);
        const int32_t r = follower_residual();
        const int32_t a = r < 0 ? -r : r;
        if (a < best_abs) { best_abs = a; best = static_cast<uint8_t>(c); }
    }
    Op0::cal(best);
    delay_us(clock, 2000);
    const int32_t after = follower_residual();
    print(serial, "  best CAL found ", hex(best), ": residual ", after,
          " counts x256 = ", counts_uv_signed(after), " uV (was ",
          counts_uv_signed(before), " uV at ", hex(production), ")", crlf);
    const int32_t before_abs = before < 0 ? -before : before;
    const int32_t after_abs = after < 0 ? -after : after;
    verdict("the trim does not make the offset worse", after_abs <= before_abs);
    verdict("the trimmed residual is inside one ADC count", after_abs <= 256);

    Op0::cal(production);
    print(serial, "  production CAL ", hex(production), " restored (it is a RAM "
                  "register: a reset reloads it from the fuses anyway)", crlf);
    verdict("the production value is back", Op0::cal() == production);

    quiesce();
}

// ---- i: the four event users -------------------------------------------------------
void ti_events() {
    print(serial, "i the event users: ENABLE and DISABLE are edges, DUMP and DRIVE "
                  "are levels", crlf);
    quiesce();
    system_up();

    // The two strobes: software events on their own channels.
    Op0::enable_on(ChEnable{});
    Op0::disable_on(ChDisable{});

    // EVENT_ENABLED: ALWAYSON down, the events own the op amp.
    verdict("OP0 in EVENT_ENABLED mode, a follower of VDD/2",
            Op0::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::out,
                       .mode = OpampMode::event, .settle_us = 20}));
    delay_us(clock, 500);
    verdict("with no event yet, SETTLED is down", !Op0::settled());
    const int32_t idle = static_cast<int32_t>(measure_x256<Op0Out>());

    ChEnable::pulse();
    delay_us(clock, 500);
    const bool up = Op0::settled();
    const int32_t running = static_cast<int32_t>(measure_x256<Op0Out>());
    print(serial, "  before the ENABLE event PD2 reads ", counts_mv(idle),
          " mV, after it ", counts_mv(running), " mV (VDD/2 is ", vdd_mv_ / 2,
          " mV)", crlf);
    verdict("the ENABLE event starts the op amp and SETTLED rises", up);
    verdict("and its output is the VDD/2 it was told to follow",
            near_permille(running, counts_x(2048), 60));

    ChDisable::pulse();
    delay_us(clock, 500);
    verdict("the DISABLE event stops it: SETTLED goes down", !Op0::settled());

    // A second ENABLE while already settled: 35.3.2.7 says the op amp
    // stays enabled and READY is issued again immediately.
    ChEnable::pulse();
    delay_us(clock, 500);
    Ready::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .event_input = true});
    Ready::capture_on(ChReady{});
    ChReady::source(Op0::ReadyEvent{});
    Ready::clear_capt();
    ChEnable::pulse();
    delay_us(clock, 200);
    verdict("an ENABLE event on an already-settled op amp re-issues READY at once",
            Ready::capt_flag());
    ChReady::off();
    Ready::disable();

    ChDisable::pulse();
    delay_us(clock, 500);
    Op0::enable_off();
    Op0::disable_off();

    // DRIVE, a LEVEL. The pad carries a pull-up throughout, so what the
    // ADC reads says who owns it - but only after the pull-up itself is
    // proven, with every op amp off.
    LevelPin::clear();
    LevelPin::output();
    ChLevel::source(EvPin<LevelPin>{});
    Op0::release();
    Op0Out::configure({.pullup = true, .sense = PinSense::input_disable});
    delay_us(clock, 500);
    const int32_t pulled_up = static_cast<int32_t>(measure_x256<Op0Out>());
    verdict("with the op amp OFF, the pull-up alone takes PD2 to the rail",
            pulled_up > counts_x(3500));

    verdict("OP0 with OUTMODE OFF, following VDD/2, events on",
            Op0::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::out,
                       .output = OpampOutput::off,
                       .mode = OpampMode::software_with_events, .settle_us = 20}));
    Op0Out::configure({.pullup = true, .sense = PinSense::input_disable});
    Op0::drive_on(ChLevel{});
    Op0::wait_settled();
    delay_us(clock, 500);
    const int32_t undriven = static_cast<int32_t>(measure_x256<Op0Out>());
    LevelPin::set();
    delay_us(clock, 500);
    const int32_t driven = static_cast<int32_t>(measure_x256<Op0Out>());
    LevelPin::clear();
    delay_us(clock, 500);
    const int32_t released = static_cast<int32_t>(measure_x256<Op0Out>());
    ChLevel::off();
    ChLevel::pulse();
    delay_us(clock, 300);
    const int32_t pulsed = static_cast<int32_t>(measure_x256<Op0Out>());
    ChLevel::source(EvPin<LevelPin>{});
    print(serial, "  OUTMODE OFF, op amp RUNNING, pull-up on the pad: ",
          counts_mv(undriven), " mV; DRIVE level high: ", counts_mv(driven),
          " mV; low again: ", counts_mv(released), " mV; after a software PULSE: ",
          counts_mv(pulsed), " mV", crlf);
    verdict("a RUNNING op amp holds its OUT pad even with OUTMODE OFF - it beats "
            "the pull-up, so 'output driver disabled' does not mean the pad is "
            "released",
            undriven < counts_x(500) && pulled_up > counts_x(3500));
    verdict("the DRIVE event raises the output driver while its level is high",
            near_permille(driven, counts_x(2048), 60));
    verdict("and the pad returns to its un-driven state when the level drops",
            near(released, undriven, counts_x(200)));
    verdict("a PULSE on the same channel does nothing: DRIVE is a level, not an edge",
            near(pulsed, undriven, counts_x(200)));
    Op0::drive_off();
    Op0::release();
    Op0Out::configure({});

    // DUMP, also a LEVEL. Open loop (MUXNEG = the INN pad, left
    // floating, ladder off) the output runs to a rail; DUMP shorts VOUT
    // to VINN and the op amp becomes a follower of VDD/2.
    verdict("OP0 open loop: + on VDD/2, - on the floating INN pad",
            Op0::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::inn,
                       .mode = OpampMode::software_with_events, .settle_us = 20}));
    Op0::wait_settled();
    delay_us(clock, 5000);
    const int32_t open_loop = static_cast<int32_t>(measure_x256<Op0Out>());
    Op0::dump_on(ChLevel{});
    LevelPin::set();
    delay_us(clock, 5000);
    const int32_t dumped = static_cast<int32_t>(measure_x256<Op0Out>());
    LevelPin::clear();
    delay_us(clock, 5000);
    const int32_t after_5ms = static_cast<int32_t>(measure_x256<Op0Out>());
    for (uint8_t i = 0; i < 20; ++i) delay_us(clock, 20000);   // 400 ms
    const int32_t after_400ms = static_cast<int32_t>(measure_x256<Op0Out>());
    print(serial, "  open loop PD2 ", counts_mv(open_loop),
          " mV; DUMP level high: ", counts_mv(dumped),
          " mV; 5 ms after the level drops: ", counts_mv(after_5ms),
          " mV; 400 ms after: ", counts_mv(after_400ms), " mV", crlf);
    verdict("open loop the output sits at a rail",
            open_loop > counts_x(3500) || open_loop < counts_x(400));
    verdict("the DUMP event closes VOUT onto VINN: the op amp follows VDD/2",
            near_permille(dumped, counts_x(2048), 80));
    // This is the integrator, with the INN pad's own stray capacitance as
    // C: the dump leaves the node near VDD/2, and when the switch opens
    // the op amp integrates whatever tiny current that node sees. So the
    // output neither stays put nor snaps back to the rail - it WANDERS,
    // which is exactly the behaviour figure 35-6 is drawn for.
    const int32_t d5 = after_5ms > open_loop ? after_5ms - open_loop
                                             : open_loop - after_5ms;
    const int32_t d400 = after_400ms > open_loop ? after_400ms - open_loop
                                                 : open_loop - after_400ms;
    (void)d400;
    verdict("milliseconds after the level drops the output is still nowhere near "
            "the open-loop rail: the floating node keeps the dump",
            d5 > counts_x(800));
    verdict("and over hundreds of milliseconds it walks away again - an "
            "integrator whose only capacitor is the pad's own stray",
            (after_400ms > after_5ms ? after_400ms - after_5ms
                                     : after_5ms - after_400ms) > counts_x(100));
    Op0::dump_off();

    quiesce();
    LevelPin::input();
}

// ---- the menu ----------------------------------------------------------------------

/// Registered once at startup. A false return would be a programming
/// error (a duplicate key or a full table), not a runtime condition.
void register_tests() {
    bench.letter('a', "the block, the registers and IRSEL", ta_block);
    bench.letter('b', "the voltage follower", tb_follower);
    bench.letter('c', "the non-inverting gain table", tc_noninverting);
    bench.letter('d', "the inverting gain table", td_inverting);
    bench.letter('e', "internal sources and op-to-op links", te_links);
    bench.letter('f', "the instrumentation amplifier", tf_instrumentation);
    bench.letter('g', "the internal timer, READY and a rebase", tg_timer);
    bench.letter('h', "offset calibration", th_calibration);
    bench.letter('i', "the four event users", ti_events);
}

void help() {
    print(serial, "test_avr_opamp:", crlf);
    bench.menu();
}

}  // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
// The stopwatch's own vector: an unbound vector on this part is a jump
// to 0, which is a reset loop and not a crash.
ISR(TCB0_INT_vect) { (void)Ready::take_flags(); }

int main() {
    // Before anything writes them: the OPAMP registers as the reset
    // left them (TIMEBASE 0x01, the CAL bytes loaded from the fuses).
    timebase_at_boot = OPAMP.TIMEBASE;
    cal_at_boot[0] = Op0::cal();
    cal_at_boot[1] = Op1::cal();
    cal_at_boot[2] = Op2::cal();

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    sei();

    (void)Meter::init(clock, {.reference = Ref::vdd,
                              .reference_always_on = true,
                              .prescaler = AdcPresc::div16,
                              .sample_length = 16,
                              .accumulate = 16});
    // The op amps tap the BUFFERED DAC output (34.3.2.3), so OUTEN is
    // on and PD6 is the DAC's for the whole run.
    Source::init({.reference = Ref::vdd, .output_pin = true});
    Source::set(0);
    vdd_mv_ = read_vdd_mv();

    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_opamp - OPAMP test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ",
          hex(SYSCFG.REVID), ", VDD ", vdd_mv_, " mV, ", opamp_count,
          " op amps)", crlf);
    register_tests();
    help();
    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            help();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "? for help", crlf);
        }
        bench.prompt();
    }
}
