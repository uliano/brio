// test_ch32_opa - the reference bench suite for the CH32V00x's OPA:
// ch32v00x/opa.hpp over RM ch. 17 of each part, measured with NO WIRE
// - the inputs at the levels their own pulls give them, the output
// read on the ADC's channel the part routes it to (9 on the CH32V006,
// an internal route; 7 on the CH32V003, the pad PD4 itself).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. PA2 (the positive input, OPA_CHP0) and PA4 (the
// differential PGA's negative input) are read through their internal
// pulls on the CH32V006; on the CH32V003 PA2 and PD0 (OPN1) are the
// two inputs and PD4 the output pad; PC0 is the LED.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the lock as found, the key pair, the
//      refusals, a configuration read back - on the CH32V003 the three
//      bits of EXTEND_CTR, no lock, the other part's selections refused
//   b  THE OPA INTO THE ADC: on the CH32V006 the positive input pulled
//      down and up through a gain of 4, the output on channel 9 at the
//      rails; on the CH32V003 the amplifier open-loop as a comparator,
//      PA2 against PD0 through their pulls, the output PD4 at the rails
//      on channel 7
//   c  (CH32V006) THE DIFFERENTIAL PGA: its sign from the inputs at
//      opposite rails - and the finding that its input network is not
//      one a pad's pull can drive, so the bias reference stays
//      unmeasured
//   d  CMP2: the key lifts the lock, and CMP_EN2 does not take on the
//      CH32V006 - the comparators are the CH32V007's; on the CH32V003
//      there is no comparator and the verbs say so
//
// build: boards = v006k8,v003f4
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
using NegPad = Pin<'A', 4>;        ///< the differential PGA's (CH32V006)
using NegPadV003 = Pin<'D', 0>;    ///< OPN1 (CH32V003)
using OutPad = Pin<'D', 4>;        ///< the CH32V003's output, read as ADC channel 7

/// A count spoken at 12 bits, scaled to the part's full scale.
constexpr uint16_t scaled(uint16_t counts12) {
    return static_cast<uint16_t>((static_cast<uint32_t>(counts12) * adc_steps) / 4096u);
}

TestBench<Serial> bench;

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

/// The ADC on the OPA's channel at a slow sample, settled.
uint16_t read_opa() {
    Adc::select_channel(Opa::adc_channel);
    (void)delay_us(clock, 200);
    return Adc::read_settled(4);
}

void adc_ready() {
    (void)Adc::init(clock, {.prescaler_code = 0x18});
    Adc::sample_time_all(adc_sample_longest);
    if constexpr (!Opa::has_block) {
        AnalogIn<OutPad>::claim();   // the output pad is the ADC's input
    }
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    if constexpr (Opa::has_block) {
        print(serial, "  as found: CTLR1=", hex(Opa::control()), " CTLR2=", hex(Opa::regs().CTLR2), " locked=",
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
        print(serial, "  after the keys: locked=", Opa::locked(), " CTLR1=", hex(Opa::control()), " (", hex(expected),
              " spelled)", crlf);
        bench.verdict("OPA_LOCK stands at reset and refuses a configuration; the key pair lifts it",
                      refused_locked && unlocked);
        bench.verdict("a configuration lands as spelled (PSEL, NSEL, MODE, FB, VBEN/VBSEL, HS)",
                      took && (Opa::control() & ~opa_lock & ~opa_en1) == expected);
        bench.verdict("the refusals: a PGA gain without the feedback, gain 32 differential, a bias without a gain",
                      !opa_config_valid({.negative = OpaNegative::gain8, .feedback = false}) &&
                          !opa_config_valid({.negative = OpaNegative::gain32, .differential = true}) &&
                          !opa_config_valid({.negative = OpaNegative::pa1, .bias = OpaBias::vdd_over_2}));
        (void)Opa::configure({});
    } else {
        // Three bits of EXTEND_CTR, no lock: the register as found (the
        // lock-up monitor's LKUPEN is its reset value), the two
        // selections written and read back, the enable, and the other
        // part's selections refused.
        print(serial, "  as found: EXTEND_CTR=", hex(Opa::control()), " locked=", Opa::locked(), crlf);
        const bool never_locked = !Opa::locked() && !Opa::lock() && !Opa::locked();
        const bool took = Opa::configure({.positive = OpaPositive::pd7, .negative = OpaNegative::pd0});
        const bool spelled = (Opa::control() & opa_exten_mask) == (opa_exten_psel | opa_exten_nsel);
        Opa::enable(true);
        const bool on = Opa::enabled() && (Opa::control() & opa_exten_en) != 0u;
        Opa::enable(false);
        const bool back = Opa::configure({}) && (Opa::control() & opa_exten_mask) == 0u && !Opa::enabled();
        print(serial, "  PSEL+NSEL written: EXTEND_CTR=", hex(Opa::control()), " (after the default again)", crlf);
        bench.verdict("no lock on this part: never locked, lock() answers false", never_locked);
        bench.verdict("OPA_PSEL and OPA_NSEL land as spelled and the enable is its own bit, LKUPEN untouched",
                      took && spelled && on && back && (Opa::control() & exten_lkupen) != 0u);
        bench.verdict("the refusals: PD3 as the positive input, a PGA gain, the internal output, the feedback "
                      "switch, a bias, the high-speed mode - the other part's OPA",
                      !opa_config_valid({.positive = OpaPositive::pd3}) &&
                          !opa_config_valid({.negative = OpaNegative::gain4, .feedback = true}) &&
                          !opa_config_valid({.output = OpaOutput::internal}) && !opa_config_valid({.feedback = true}) &&
                          !opa_config_valid({.bias = OpaBias::vdd_over_2}) && !opa_config_valid({.high_speed = true}));
    }
}

// ===========================================================================
// b - the PGA into the ADC
// ===========================================================================

void tb_pga() {
    adc_ready();
    Opa::unlock();
    if constexpr (Opa::has_block) {
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
                      low <= scaled(100) && high >= scaled(3900));
    } else {
        // No gain and no feedback switch: the amplifier open-loop is a
        // comparator of its two pads. PA2 (OPP0) against PD0 (OPN1),
        // each through its own pull, the output PD4 read on channel 7.
        (void)Opa::configure({.positive = OpaPositive::pa2, .negative = OpaNegative::pd0});
        Opa::enable(true);
        PosPad::input(PinPull::down);
        NegPadV003::input(PinPull::up);
        console_drain();
        const uint16_t low = read_opa();
        PosPad::input(PinPull::up);
        NegPadV003::input(PinPull::down);
        console_drain();
        const uint16_t high = read_opa();
        Opa::enable(false);
        PosPad::release();
        NegPadV003::release();
        OutPad::input(PinPull::down);
        (void)delay_us(clock, 100);
        AnalogIn<OutPad>::claim();
        const uint16_t off = read_opa();
        OutPad::release();
        print(serial, "  open-loop, PA2 down / PD0 up: PD4 on channel 7 reads ", low, "; PA2 up / PD0 down: ", high,
              "; the OPA off, PD4 released: ", off, crlf);
        bench.verdict("the amplifier open-loop drives PD4 to the rail its inputs order: low with P below N, high "
                      "with P above N, on the ADC's channel 7",
                      low <= scaled(100) && high >= scaled(3900));
    }
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
    if constexpr (Opa::has_block) {
        print(serial, "  CMP2: CMP_KEY lifts the lock ", unlocked, ", CMP_EN2 takes ", on, "; CTLR2=",
              hex(Opa::regs().CTLR2), crlf);
        print(serial, "  -> ", on ? "CMP2 enables on this part"
                                 : "CMP_EN2 DOES NOT TAKE on the CH32V006: the comparators are the CH32V007's (17's "
                                   "opening paragraph), the lock alone answers",
              crlf);
        bench.verdict("CMP_KEY lifts CMP_LOCK; whether CMP_EN2 takes is the finding above", unlocked);
    } else {
        print(serial, "  CMP2: no comparator on this part - the lock never lifts (", !unlocked, "), the enable never "
              "takes (", !on, ")", crlf);
        bench.verdict("the CMP2 verbs answer as a part with no comparator: locked for good, nothing enabled",
                      !unlocked && !on && !Opa::cmp2_enabled());
    }
}

void banner() {
    print(serial, crlf, "test_ch32_opa - ", device::part_name, " OPA (RM ch. 17), nothing to wire", crlf);
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
    bench.letter('b', "THE OPA into the ADC: the inputs at the rails through their pulls", tb_pga);
    if constexpr (Opa::has_block) {
        bench.letter('c', "THE DIFFERENTIAL PGA: its sign at the rails, the bias unmeasurable through pulls", tc_bias);
    }
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
