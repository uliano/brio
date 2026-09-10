// test_ch32_opa - the reference bench suite for the CH32V00x's OPA:
// ch32v00x/opa.hpp over RM ch. 17, measured with NO WIRE - the two
// inputs at the levels their own pulls give them, the output read on
// the ADC's channel 9, the bias reference as the known voltage.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. PA2 (the positive input, OPA_CHP0) and PA4 (the
// differential PGA's negative input) are read through their internal
// pulls; PC0 is the LED.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the lock as found, the key pair, the
//      refusals, a configuration read back, the lock's one-way
//   b  THE PGA INTO THE ADC: the positive input pulled down and up
//      through a gain of 4, the output on channel 9 at the rails
//   c  THE DIFFERENTIAL PGA: its sign from the inputs at opposite
//      rails - and the finding that its input network is not one a
//      pad's pull can drive, so the bias reference stays unmeasured
//   d  CMP2: the key lifts the lock, and CMP_EN2 does not take on this
//      part - the comparators are the CH32V007's
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v00x/adc.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/opa.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;
using PosPad = Pin<'A', 2>;
using NegPad = Pin<'A', 4>;

TestBench<Serial> bench;

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

/// The ADC on channel 9 at a slow sample, settled.
uint16_t read_opa() {
    Adc::select(AdcInput::opa);
    (void)delay_us(clock, 200);
    return Adc::read_settled(4);
}

void adc_ready() {
    (void)Adc::init(clock, {.prescaler_code = 0x18});
    Adc::sample_time_all(AdcSampleTime::cycles239_5);
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    print(serial, "  as found: CTLR1=", hex(Opa::ctlr1()), " CTLR2=", hex(Opa::regs().CTLR2), " locked=",
          Opa::locked(), crlf);
    // The lock stands at reset: a configuration is refused, and takes
    // after the key pair.
    const bool refused_locked = Opa::locked() && !Opa::configure({});
    Opa::unlock();
    const bool unlocked = !Opa::locked();
    const bool took = Opa::configure({.positive = OpaPositive::pd3, .negative = OpaNegative::gain16,
                                      .output = OpaOutput::pa5, .bias = OpaBias::vdd_over_4, .high_speed = true});
    const uint32_t expected = (1UL << 1) | (2UL << 4) | (5UL << 8) | opa_fb_en1 | opa_vben | opa_vbsel |
                              (3UL << 18) | opa_hs1;
    print(serial, "  after the keys: locked=", Opa::locked(), " CTLR1=", hex(Opa::ctlr1()), " (", hex(expected),
          " spelled)", crlf);
    bench.verdict("OPA_LOCK stands at reset and refuses a configuration; the key pair lifts it",
                  refused_locked && unlocked);
    bench.verdict("a configuration lands as spelled (PSEL, NSEL, MODE, FB, VBEN/VBSEL, HS)",
                  took && (Opa::ctlr1() & ~opa_lock & ~opa_en1) == expected);
    bench.verdict("the refusals: a PGA gain without the feedback, gain 32 differential, a bias without a gain",
                  !opa_config_valid({.negative = OpaNegative::gain8, .feedback = false}) &&
                      !opa_config_valid({.negative = OpaNegative::gain32, .differential = true}) &&
                      !opa_config_valid({.negative = OpaNegative::pa1, .bias = OpaBias::vdd_over_2}));
    (void)Opa::configure({});
}

// ===========================================================================
// b - the PGA into the ADC
// ===========================================================================

void tb_pga() {
    adc_ready();
    Opa::unlock();
    (void)Opa::configure({.positive = OpaPositive::pa2, .negative = OpaNegative::gain4, .output = OpaOutput::internal});
    Opa::enable(true);
    PosPad::input(PinPull::down);
    console_drain();
    const uint16_t low = read_opa();
    PosPad::input(PinPull::up);
    console_drain();
    const uint16_t high = read_opa();
    Opa::enable(false);
    const uint16_t off = read_opa();
    PosPad::release();
    print(serial, "  PGA x4, PA2 pulled down: channel 9 reads ", low, "; pulled up: ", high, "; the OPA off: ", off,
          crlf);
    bench.verdict("the PGA's output reaches the ADC's channel 9: near zero with the input low, saturated with "
                  "it high",
                  low <= 100u && high >= 3900u);
    Adc::release();
}

// ===========================================================================
// c - the bias reference
// ===========================================================================

void tc_bias() {
    adc_ready();
    // VDD from VREFINT, for the expected counts.
    Adc::select(AdcInput::vrefint);
    console_drain();
    const uint16_t vref = Adc::read_settled(4);
    Opa::unlock();
    struct Case {
        const char* name;
        OpaBias bias;
        PinPull pos, neg;
        uint16_t expect;   // counts, 0xFFFF = a rail
    };
    const Case cases[] = {
        {"VDD/2, both inputs up", OpaBias::vdd_over_2, PinPull::up, PinPull::up, 2048},
        {"VDD/4, both inputs up", OpaBias::vdd_over_4, PinPull::up, PinPull::up, 1024},
        {"VDD/2, both inputs down", OpaBias::vdd_over_2, PinPull::down, PinPull::down, 2048},
        {"VDD/2, P up N down", OpaBias::vdd_over_2, PinPull::up, PinPull::down, 4095},
        {"VDD/2, P down N up", OpaBias::vdd_over_2, PinPull::down, PinPull::up, 0},
    };
    // THE DIFFERENTIAL PGA'S INPUTS ARE NOT HIGH-IMPEDANCE: figure 17-1
    // draws 3.2 kOhm resistors into the network, and a pad's pull (tens
    // of kOhm) cannot hold such an input at a rail - so the same-level
    // rows are printed as what they read and not judged (measured: both
    // pulled up reads near zero, both pulled down near 0.77 VDD, neither
    // the bias); the opposite-rail rows still tell the sign.
    uint8_t rails_right = 0;
    for (const Case& c : cases) {
        (void)Opa::configure({.positive = OpaPositive::pa2, .negative = OpaNegative::gain4,
                              .output = OpaOutput::internal, .differential = true, .bias = c.bias});
        Opa::enable(true);
        PosPad::input(c.pos);
        NegPad::input(c.neg);
        console_drain();
        const uint16_t v = read_opa();
        Opa::enable(false);
        const bool rail_row = c.pos != c.neg;
        const bool ok = rail_row && v >= (c.expect > 100u ? c.expect - 100u : 0u) && v <= c.expect + 100u;
        print(serial, "  differential PGA x4, ", c.name, ": ", v,
              rail_row ? (ok ? "  (a rail, as expected)" : "  (a rail expected: OFF)")
                       : "  (the bias if the pulls held the inputs - they do not, see the source)",
              crlf);
        if (ok) {
            ++rails_right;
        }
    }
    PosPad::release();
    NegPad::release();
    print(serial, "  (VREFINT read ", vref, " counts: the supply is ", Adc::supply_mv(vref), " mV)", crlf);
    bench.verdict("the differential PGA's sign: P above N saturates high, N above P saturates low, on channel 9",
                  rails_right == 2u);
    print(serial, "  -> the bias reference needs a SOURCE on the inputs (3.2 kOhm input network): not measured here",
          crlf);
    Adc::release();
}

// ===========================================================================
// d - CMP2
// ===========================================================================

void td_cmp2() {
    Opa::cmp_unlock();
    const bool unlocked = !Opa::cmp_locked();
    const bool on = Opa::cmp2_enable(true) && Opa::cmp2_enabled();
    (void)Opa::cmp2_enable(false);
    print(serial, "  CMP2: CMP_KEY lifts the lock ", unlocked, ", CMP_EN2 takes ", on, "; CTLR2=", hex(Opa::regs().CTLR2),
          crlf);
    print(serial, "  -> ", on ? "CMP2 enables on this part"
                             : "CMP_EN2 DOES NOT TAKE on the CH32V006: the comparators are the CH32V007's (17's "
                               "opening paragraph), the lock alone answers",
          crlf);
    bench.verdict("CMP_KEY lifts CMP_LOCK; whether CMP_EN2 takes is the finding above", unlocked);
}

void banner() {
    print(serial, crlf, "test_ch32_opa - CH32V006K8 OPA (RM ch. 17), nothing to wire", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless: the lock as found, the keys, a configuration, the refusals", ta_block);
    bench.letter('b', "THE PGA into the ADC's channel 9: the input at both rails", tb_pga);
    bench.letter('c', "THE DIFFERENTIAL PGA: its sign at the rails, the bias unmeasurable through pulls", tc_bias);
    bench.letter('d', "CMP2: the key lifts the lock, CMP_EN2 on this part", td_cmp2);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
