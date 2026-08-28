// test_samc_supc - the reference bench suite for samc/supc.hpp, the
// SAM C21 Supply Controller (DS60001479M ch. 22).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and NOTHING IS FORCED. The supply is not this
// program's to dip, so no brown-out is ever provoked: the BODVDD's
// RESET and INT actions are configured and read back but never fired,
// and every threshold this suite sets carries ACTION = none, under which
// STATUS.BODVDDDET still tracks the comparison. That is what makes a
// THRESHOLD SWEEP - raising the level until the detector says the supply
// is below it - a safe measurement rather than a reboot.
//
// THE INSTRUMENT IS THE BANDGAP, and it closes a loop three drivers
// wide. SUPC's VREF produces INTREF; samc/ac.hpp's comparator takes it
// as its negative input and its own 64-step VDD scaler as the positive
// one; sweeping the scaler until the comparison flips locates VDD
// against a known voltage. That is the gap docs/samc/ac.md has been
// carrying - the bandgap input needs SUPC.VREF.VREFOE, and until this
// header there was nothing to turn it on with.
//
// What is exercised, letter by letter:
//   a  the block, the fuses, and the half of this chapter that is
//      deliberately read-only
//   b  THE BANDGAP: VREFOE, the AC loop, VDD measured against INTREF at
//      two reference levels, and erratum 1.5.6 looked for
//   c  BODVDD: the enable-protection discipline observed, the threshold
//      swept to find VDD a second way, the step size derived, the
//      sampling mode's own trap, and the fuse configuration restored
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/clock.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/supc.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
using Led = Pin<'B', 23>;

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

using Comp = AcComparator<0>;
constexpr uint8_t ac_gen = 0;   ///< GCLK_AC from generator 0 (48 MHz)

/// The BODVDD configuration this board boots with, captured once at
/// start-up so letter c can put it back exactly as the fuses left it.
uint32_t boot_bodvdd = 0;

void wait_ms(uint32_t ms) {
    const uint32_t deadline = Ticker::millis() + ms;
    while (static_cast<int32_t>(Ticker::millis() - deadline) < 0) {
    }
}

/// A few microseconds for an analog block to settle. Volatile sink so
/// the loop survives -Os.
void settle() {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < 20'000UL; ++i) {
        sink = sink + 1u;
    }
}

// ---------------------------------------------------------------------------
// The bandgap loop: VDD located against INTREF
// ---------------------------------------------------------------------------
//
// The comparator's positive input is its own VDD scaler, which produces
// VDD x (value + 1) / 64 (40.8.12); its negative input is INTREF. So
// STATE is high exactly while VDD x (value + 1) / 64 exceeds the
// reference, and the SMALLEST value at which that happens brackets VDD
// between reference x 64 / (value + 1) and reference x 64 / value.
//
// Returns 0xFF when no step flips, which is the honest answer for a
// reference above VDD (the 4.096 V level on a 3.3 V board, say).
uint8_t scaler_crossing(uint32_t* spurious_flags = nullptr) {
    uint8_t found = 0xFF;
    for (uint16_t v = 0; v < 64; ++v) {
        Comp::scaler(static_cast<uint8_t>(v));
        settle();
        if (Comp::ready() && Comp::state()) {
            found = static_cast<uint8_t>(v);
            break;
        }
        if (spurious_flags != nullptr && Comp::flag_set()) {
            *spurious_flags = *spurious_flags + 1u;
            Comp::clear_flag();
        }
    }
    return found;
}

/// VDD in millivolts from a crossing step, taking the midpoint of the
/// bracket the step defines. Zero when nothing crossed.
uint32_t vdd_mv_from(uint8_t crossing, uint16_t reference_mv) {
    if (crossing == 0xFFu || crossing == 0u) {
        return 0;
    }
    // Below the crossing: VDD x crossing / 64 <= ref. At it:
    // VDD x (crossing+1) / 64 > ref. Midpoint of the two bounds.
    const uint32_t lo = (static_cast<uint32_t>(reference_mv) * 64u) / (crossing + 1u);
    const uint32_t hi = (static_cast<uint32_t>(reference_mv) * 64u) / crossing;
    return (lo + hi) / 2u;
}

bool near(uint32_t got, uint32_t want, uint32_t band) {
    return got + band >= want && want + band >= got;
}

/**
 * Bring the bandgap and the comparator up together and return the
 * crossing step for a given reference level, or 0xFF.
 *
 * ERRATUM 1.5.6, EVERY REVISION: enabling a comparator whose negative
 * input is the bandgap can raise a SPURIOUS COMPn flag. The obligation
 * ac.hpp states is to clear it after the enable and before arming any
 * interrupt, which is what happens here - and `spurious` reports
 * whether there was one to clear, because an erratum nobody looks for
 * is an erratum nobody can confirm.
 */
uint8_t crossing_at(VrefLevel level, bool* spurious) {
    if (!Vref::configure(VrefConfig{.level = level, .output_enable = true})) {
        return 0xFF;
    }
    settle();
    if (!Comp::configure(AcConfig{.positive = AcPositive::vscale,
                                  .negative = AcNegative::bandgap,
                                  .speed = AcSpeed::high})) {
        return 0xFF;
    }
    Comp::scaler(0);
    Comp::clear_flag();
    if (!Comp::enable(true)) {
        return 0xFF;
    }
    settle();
    *spurious = Comp::flag_set();
    Comp::clear_flag();   // the erratum's obligation, kept whether or not

    uint32_t ignored = 0;
    const uint8_t c = scaler_crossing(&ignored);
    (void)Comp::enable(false);
    return c;
}

// =============================================================================
// a - the block, the fuses, and the read-only half
// =============================================================================
void ta_block() {
    print(serial, "  SUPC status=", hex(Supc::status()),
          " BODVDD=", hex(BodVdd::reg()), " BODCORE=", hex(BodCore::reg()),
          " VREG=", hex(Vreg::reg()), " VREF=", hex(Vref::reg()), crlf);

    bench.verdict("SUPC is IRQ 0, shared with four other blocks",
                  Supc::irq() == SUPC_IRQn);

    // THE REGULATOR CANNOT BE OFF. 22.8.6 forbids clearing the bit and
    // this driver offers no way to; the verdict is that the silicon
    // agrees it is set.
    bench.verdict("the main voltage regulator is enabled, as it must "
                  "always be", Vreg::enabled());

    // THE FUSES. BODVDD's reset value comes from the NVM user row at
    // every power-on or user reset (22.6.3.2) - the same coupling
    // reset.hpp records for the watchdog, seen here from the SUPC end.
    const NvmUserRow row = NvmUserRow::read();
    print(serial, "  user row says: level=", row.bodvdd_level(),
          " enabled=", row.bodvdd_disabled() ? 0 : 1,
          " action=", row.bodvdd_action(),
          " hysteresis=", row.bodvdd_hysteresis() ? 1 : 0, crlf);
    print(serial, "  the register says: level=", BodVdd::level(),
          " enabled=", BodVdd::enabled() ? 1 : 0,
          " action=", static_cast<uint8_t>(BodVdd::action()),
          " hysteresis=", BodVdd::hysteresis() ? 1 : 0, crlf);
    bench.verdict("THE FUSE ROW AND THE BODVDD REGISTER AGREE FIELD BY "
                  "FIELD - two drivers describing one thing from opposite "
                  "ends",
                  BodVdd::matches_fuses());
    bench.verdict("and the level is the production setting, 8 (about 2.8 V "
                  "by table 45-18)",
                  row.bodvdd_level() == BodVdd::level_2v8);

    // A continuously running detector reports ready; STATUS.BODVDDRDY
    // is never set in sampling mode, which letter c measures.
    bench.verdict("the detector the fuses started reports ready",
                  !BodVdd::enabled() || BodVdd::ready());
    bench.verdict("and does NOT report a detection at this supply",
                  !BodVdd::detected());
    bench.verdict("its synchronization is idle", BodVdd::sync_ready());

    // THE HALF THAT IS DELIBERATELY READ-ONLY, and the half the chapter
    // does not draw at all: SUPC_BODCORE sits at offset 0x14, which
    // 22.7's register summary marks Reserved, and STATUS bits 3..5 are
    // in the device header and not in the chapter. Reading is how one
    // finds out whether anything is behind them.
    print(serial, "  BODCORE (the register ch. 22 does not draw): enabled=",
          BodCore::enabled() ? 1 : 0, " action=",
          static_cast<uint8_t>(BodCore::action()), " hysteresis=",
          BodCore::hysteresis() ? 1 : 0, crlf);
    print(serial, "  and its three undrawn status bits: ready=",
          BodCore::ready() ? 1 : 0, " detect=", BodCore::detected() ? 1 : 0,
          " sync=", BodCore::sync_ready() ? 1 : 0, crlf);
    bench.verdict("THE CORE DETECTOR IS RUNNING - the register the chapter "
                  "calls Reserved is real, and the device header was right "
                  "to declare it",
                  BodCore::enabled());
    bench.verdict("it is not detecting, which is the only acceptable answer "
                  "for a board that is running",
                  !BodCore::detected());
    // It is calibrated in production and its calibration must not be
    // changed (22.6.3.4), which is why this driver has no setter at
    // all. The verdict is on the DRIVER's shape, not the silicon's.
    bench.verdict("and the driver offers no way to write it - the shape is "
                  "the safety",
                  true);

    // The interrupt surface.
    Supc::arm(SupcFlag::bodvdd_detect);
    bench.verdict("an interrupt source arms",
                  (Supc::armed() & SupcFlag::bodvdd_detect) != 0u);
    Supc::disarm(SupcFlag::all);
    bench.verdict("and disarms", Supc::armed() == 0u);

    // The refusals.
    bench.verdict("a BODVDD level past its six-bit field is refused",
                  !BodVdd::config_valid(BodVddConfig{.level = 64}));
    bench.verdict("a Reserved VREF level code is refused",
                  !Vref::config_valid(VrefConfig{.level = static_cast<VrefLevel>(1)}));
    bench.verdict("and the three implemented ones are not",
                  Vref::config_valid(VrefConfig{.level = VrefLevel::v1_024}) &&
                      Vref::config_valid(VrefConfig{.level = VrefLevel::v2_048}) &&
                      Vref::config_valid(VrefConfig{.level = VrefLevel::v4_096}));
}

// =============================================================================
// b - the bandgap, and VDD located against it
// =============================================================================
void tb_bandgap() {
    bench.verdict("the AC block comes up", Ac::init(ac_gen));

    Vref::output_enable(false);
    bench.verdict("the bandgap output can be off, as a reset leaves it "
                  "(22.6.2.1)",
                  !Vref::output_enabled());

    bool spurious_1v = false;
    const uint8_t c1 = crossing_at(VrefLevel::v1_024, &spurious_1v);
    bench.verdict("VREFOE is on and the reference is 1.024 V",
                  Vref::output_enabled() && Vref::level() == VrefLevel::v1_024);
    bench.verdict("THE COMPARATOR CAN SEE THE BANDGAP - the scaler crosses "
                  "it - which is what SUPC.VREF.VREFOE buys and what "
                  "ac.md's gap list was waiting for",
                  c1 != 0xFFu && c1 != 0u);

    bool spurious_2v = false;
    const uint8_t c2 = crossing_at(VrefLevel::v2_048, &spurious_2v);
    bench.verdict("the same comparator crosses the 2.048 V reference too",
                  c2 != 0xFFu && c2 != 0u);

    const uint32_t vdd1 = vdd_mv_from(c1, vref_mv(VrefLevel::v1_024));
    const uint32_t vdd2 = vdd_mv_from(c2, vref_mv(VrefLevel::v2_048));
    print(serial, "  INTREF 1.024 V crosses at scaler step ", c1, " -> VDD ",
          vdd1, " mV", crlf);
    print(serial, "  INTREF 2.048 V crosses at scaler step ", c2, " -> VDD ",
          vdd2, " mV", crlf);

    if (vdd1 != 0u && vdd2 != 0u) {
        // THE VERDICT THAT PROVES THE REFERENCE IS REAL: doubling the
        // reference must double the step at which VDD's fraction
        // overtakes it. A floating input would not track.
        bench.verdict("DOUBLING THE REFERENCE DOUBLES THE CROSSING STEP - "
                      "the bandgap is a voltage and not a floating pin",
                      near(static_cast<uint32_t>(c2) + 1u,
                           2u * (static_cast<uint32_t>(c1) + 1u), 2u));
        bench.verdict("and the two levels agree on VDD to within 5 %",
                      near(vdd1, vdd2, vdd2 / 20u));
        bench.verdict("which lands inside the 2.7..5.5 V this family "
                      "accepts (table 45-20)",
                      vdd2 >= 2700u && vdd2 <= 5500u);
    }

    // ERRATUM 1.5.6, looked for rather than assumed. ac.hpp states the
    // obligation - clear the flag after enabling with MUXNEG = bandgap -
    // and this is the only place in the stratum that can say whether
    // there was ever anything to clear.
    print(serial, "  erratum 1.5.6 (spurious COMP flag on a bandgap enable): ",
          spurious_1v ? "SEEN at 1.024 V" : "not seen at 1.024 V", ", ",
          spurious_2v ? "SEEN at 2.048 V" : "not seen at 2.048 V", crlf);
    bench.verdict("ERRATUM 1.5.6 IS REAL ON THIS DIE - at least one of the "
                  "two enables raised a COMP flag with nothing to flag",
                  spurious_1v || spurious_2v);
    // The flag the sweep itself sets is not the erratum's: a comparator
    // that really flips is entitled to raise it. What is asserted here
    // is only that it clears.
    Comp::clear_flag();
    bench.verdict("and the flag clears on demand, which is all the "
                  "obligation asks", !Comp::flag_set());

    // THE THIRD REFERENCE, and the one that makes the answer hard to
    // argue with: 4.096 V is four times 1.024 V, and if the three
    // crossings land on one VDD the bandgap is a real voltage measured
    // three ways.
    bool spurious_4v = false;
    const uint8_t c4 = crossing_at(VrefLevel::v4_096, &spurious_4v);
    const uint32_t vdd4 = vdd_mv_from(c4, vref_mv(VrefLevel::v4_096));
    if (c4 == 0xFFu) {
        print(serial, "  INTREF 4.096 V never crosses - this board's VDD is "
              "below it", crlf);
    } else {
        print(serial, "  INTREF 4.096 V crosses at scaler step ", c4,
              " -> VDD ", vdd4, " mV", crlf);
        bench.verdict("ALL THREE REFERENCE LEVELS AGREE ON VDD to within 5 %",
                      vdd4 != 0u && near(vdd4, vdd2, vdd2 / 20u));
    }

    Vref::output_enable(false);
    bench.verdict("the bandgap output is handed back", !Vref::output_enabled());
    Ac::release();
}

// =============================================================================
// c - BODVDD: the discipline, the threshold, and the way back
// =============================================================================
void tc_bodvdd() {
    // The board's own configuration, captured at boot before anything
    // could touch it. Restored at the end of this letter whatever
    // happens in between.
    print(serial, "  the boot BODVDD register was ", hex(boot_bodvdd), crlf);

    // THE ENABLE-PROTECTION, OBSERVED. 22.6.3.1: while ENABLE is 1 a
    // write to a protected field is DISCARDED. So a level written to a
    // running detector must NOT take, and the same level written to a
    // stopped one must.
    bench.verdict("the detector is running to start with", BodVdd::enabled());
    const uint8_t level_before = BodVdd::level();
    const uint8_t other_level = static_cast<uint8_t>(level_before + 1u);
    (void)BodVdd::wait_sync();
    SUPC_REGS->SUPC_BODVDD =
        (BodVdd::reg() & ~static_cast<uint32_t>(SUPC_BODVDD_LEVEL_Msk)) |
        SUPC_BODVDD_LEVEL(other_level);
    (void)BodVdd::wait_sync();
    bench.verdict("A LEVEL WRITTEN TO A RUNNING DETECTOR IS DISCARDED - "
                  "22.6.3.1's enable-protection, observed",
                  BodVdd::level() == level_before);

    // Now through the driver, which spends the whole dance.
    bench.verdict("the same level goes in through configure(), which "
                  "disables first",
                  BodVdd::configure(BodVddConfig{.level = other_level,
                                                 .action = BodAction::none}));
    bench.verdict("and it took", BodVdd::level() == other_level);
    bench.verdict("with the action this suite insists on everywhere - "
                  "nothing is ever forced here",
                  BodVdd::action() == BodAction::none);

    // A CONTINUOUS detector reports ready; a SAMPLED one never does
    // (22.8.4), which is a trap worth catching before someone waits on
    // it forever.
    bench.verdict("a continuous detector reports ready", BodVdd::ready());
    bench.verdict("a SAMPLED detector is configured",
                  BodVdd::configure(BodVddConfig{.level = other_level,
                                                 .action = BodAction::none,
                                                 .sampled = true,
                                                 .prescaler = BodPrescaler::div2}));
    wait_ms(20);
    const bool sampled_ready = BodVdd::ready();
    print(serial, "  sampling mode: STATUS.BODVDDRDY reads ",
          sampled_ready ? 1 : 0, " (22.8.4 says it is never set there); the "
          "sampling clock is ", bod_sample_mhz(BodPrescaler::div2) / 1000u,
          " Hz nominal", crlf);
    bench.verdict("and it does NOT report ready, exactly as 22.8.4 says",
                  !sampled_ready);

    // THE THRESHOLD SWEEP. Continuous mode, ACTION = none, level raised
    // until the detector says the supply is below it. Nothing is
    // forced: the supply never moves, only the threshold.
    uint8_t crossing = 0xFF;
    for (uint16_t lv = 0; lv < 64; ++lv) {
        if (!BodVdd::configure(BodVddConfig{.level = static_cast<uint8_t>(lv),
                                            .action = BodAction::none})) {
            continue;
        }
        // Give the analog comparison time to settle before believing
        // STATUS: table 45-18 puts the start-up at 3.1 us.
        settle();
        if (BodVdd::detected()) {
            crossing = static_cast<uint8_t>(lv);
            break;
        }
    }
    bench.verdict("THE THRESHOLD SWEEP FINDS A CROSSING - the supply is "
                  "located without ever being disturbed",
                  crossing != 0xFFu && crossing > BodVdd::level_2v8);
    print(serial, "  BODVDD first detects at level ", crossing,
          " (level 8 is 2.80 V by table 45-18)", crlf);

    // THE STEP SIZE, which table 45-18 states as 60 mV typical while
    // its own three anchor points imply about 47.5 mV. With VDD known
    // from letter b's bandgap loop, one crossing settles it:
    //   VDD = 2800 mV + (crossing - 8) x step
    // The AC is brought up again here so the letter stands alone.
    bench.verdict("the AC block comes up for the cross-check", Ac::init(ac_gen));
    bool spurious = false;
    const uint8_t c2 = crossing_at(VrefLevel::v2_048, &spurious);
    const uint32_t vdd = vdd_mv_from(c2, vref_mv(VrefLevel::v2_048));
    Vref::output_enable(false);
    Ac::release();

    if (vdd != 0u && crossing != 0xFFu && crossing > BodVdd::level_2v8) {
        const uint32_t step_uv =
            ((vdd - 2800u) * 1000u) / (crossing - BodVdd::level_2v8);
        print(serial, "  VDD measured against the bandgap: ", vdd,
              " mV, so the BODVDD step is ", step_uv / 1000u, ".",
              (step_uv % 1000u) / 100u,
              " mV - against 60 mV stated and about 47.5 mV implied by the "
              "same table's own anchors", crlf);
        // A BAND, not a coin toss: the crossing is granular to one
        // level, so the step is only known to about its own size
        // divided by (crossing - 8). What is assertable is that it is a
        // few tens of millivolts and not a few hundred.
        bench.verdict("the step is tens of millivolts, as both of the "
                      "table's numbers agree it should be",
                      step_uv > 30'000u && step_uv < 80'000u);
    }

    // HYSTERESIS, configured and read back. Table 45-18 puts the
    // separation at 40..75 mV, which is LESS than one level step, so a
    // sweep cannot resolve it and this suite does not pretend to.
    bench.verdict("hysteresis configures",
                  BodVdd::configure(BodVddConfig{.level = BodVdd::level_2v8,
                                                 .action = BodAction::none,
                                                 .hysteresis = true}));
    bench.verdict("and reads back", BodVdd::hysteresis());
    print(serial, "  the hysteresis is 40..75 mV (table 45-18), narrower "
          "than one level step - this suite configures it and does not "
          "claim to have measured it", crlf);

    // THE WAY BACK: the exact register the fuses produced, through the
    // same enable-protection dance.
    // Three steps, not two: stop, write the configuration while it is
    // stopped, then set ENABLE on its own. Writing the configuration
    // AND ENABLE in one store does not work here - the enable-protected
    // fields do not take when the same store raises ENABLE (measured:
    // the register came back with the old level and the new enable).
    (void)BodVdd::wait_sync();
    SUPC_REGS->SUPC_BODVDD =
        BodVdd::reg() & ~static_cast<uint32_t>(SUPC_BODVDD_ENABLE_Msk);
    (void)BodVdd::wait_sync();
    SUPC_REGS->SUPC_BODVDD =
        boot_bodvdd & ~static_cast<uint32_t>(SUPC_BODVDD_ENABLE_Msk);
    (void)BodVdd::wait_sync();
    const bool restored_config =
        BodVdd::reg() == (boot_bodvdd & ~static_cast<uint32_t>(SUPC_BODVDD_ENABLE_Msk));
    const bool restored = BodVdd::enable((boot_bodvdd & SUPC_BODVDD_ENABLE_Msk) != 0u);
    print(serial, "  restored BODVDD reads ", hex(BodVdd::reg()),
          ", the boot value was ", hex(boot_bodvdd), crlf);
    bench.verdict("the boot configuration goes back in while the detector "
                  "is stopped", restored_config);
    bench.verdict("the boot configuration is restored bit for bit",
                  restored && BodVdd::reg() == boot_bodvdd);
    bench.verdict("so the fuses and the register agree again",
                  BodVdd::matches_fuses());
    bench.verdict("and the board is protected as it was found",
                  BodVdd::enabled() && BodVdd::action() == BodAction::reset);
}

void banner() {
    print(serial, crlf,
          "test_samc_supc - SAMC21J18A SUPC (ch. 22): the brown-out "
          "detectors, the regulator and the bandgap, clk=", SysClock::hz,
          " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    // Captured before anything in this program can have written it, so
    // letter c has the fuses' own word to put back.
    boot_bodvdd = brio::BodVdd::reg();

    brio::enable_interrupts();

    bench.letter('a', "the block, the fuses and the read-only half", ta_block);
    bench.letter('b', "the bandgap, and VDD located against it", tb_bandgap);
    bench.letter('c', "BODVDD: discipline, threshold and the way back",
                 tc_bodvdd);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " BODVDD=", hex(boot_bodvdd), crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
