// test_v203_opa - the reference bench suite for the CH32V203's two
// operational amplifiers: ch32v203/opa.hpp over RM ch. 30, read by the
// converter of ch32v203/adc.hpp on the pad each output lands on.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT AN AMPLIFIER WITH NO WIRE IS. This block has no internal
// feedback and no gain - four bits an amplifier and nothing else - so
// with nothing strapped from an output pad back to a negative input pad
// it is an open-loop stage: a COMPARATOR whose output saturates to
// whichever rail its two inputs put it at. That is what this suite
// measures, because it is what the board can offer: both inputs are
// pads driven by their own port, and the output is read as an ANALOG
// CHANNEL, because every OPA output pad is also an ADC input pad. A
// follower, a divider and the input offset voltage want one wire and
// are in the document's gap list.
//
// THE CLOCK. The converter that reads the amplifier is rated at 14 MHz
// and ADCCLK is PCLK2 divided by at most eight, so this suite runs the
// PLL on the HSI at 96 MHz exactly as test_v203_adc does: ADCCLK
// 12 MHz, in specification, no crystal needed.
//
// THE PADS, all of them free on this board. OPA1: PB15 and PB0 are its
// two positive inputs, PB11 and PA6 its two negative ones, PA3
// (channel 3) and PB1 (channel 9) its two outputs. OPA2: PB14 and PA7,
// PB10 and PA5, PA2 (channel 2) and PA4 (channel 4). NEVER TOUCHED:
// PA9/PA10 (the console), PA13/PA14 (the debug port), PA11/PA12 (the
// USB pads), PC14/PC15 and PD0/PD1 (the crystals), PA0 (the KEY) - and
// PB2, the LED, toggled per command as every suite of this target does.
//
// What is exercised, letter by letter:
//   a  OPA2 AS A COMPARATOR: its inputs driven to each rail in turn and
//      its output read on the channel its pad carries, both ways round
//   b  ITS OTHER PADS: both positive inputs, both negative ones and
//      both outputs, each selection proven by the level that follows it
//   c  OPA1, the same measurements on its own six pads
//   d  THE REGISTER: four bits an amplifier, the two fields
//      independent, the reset state, and every selection read back
//   e  THE OUTPUT ITSELF: what the saturated rails measure, and the
//      amplifier driving its pad against that pad's own pull
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v203/adc.hpp"
#include "ch32v203/clock.hpp"
#include "ch32v203/opa.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32v203Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

using SysClock = Clock<ClockSource::pll, 96'000'000>;
constexpr SysClock clock;
static_assert(SysClock::adc_in_spec);

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The amplifiers' pads
// ---------------------------------------------------------------------------

using P2a = OpaIn<2, OpaPin::positive0>;   // PB14
using P2b = OpaIn<2, OpaPin::positive1>;   // PA7
using N2a = OpaIn<2, OpaPin::negative0>;   // PB10
using N2b = OpaIn<2, OpaPin::negative1>;   // PA5
using O2a = OpaOut<2, OpaPin::out0>;       // PA2, channel 2
using O2b = OpaOut<2, OpaPin::out1>;       // PA4, channel 4

using P1a = OpaIn<1, OpaPin::positive0>;   // PB15
using P1b = OpaIn<1, OpaPin::positive1>;   // PB0
using N1a = OpaIn<1, OpaPin::negative0>;   // PB11
using N1b = OpaIn<1, OpaPin::negative1>;   // PA6
using O1a = OpaOut<1, OpaPin::out0>;       // PA3, channel 3
using O1b = OpaOut<1, OpaPin::out1>;       // PB1, channel 9

/// The supply this board runs at, measured from VREFINT at the top of
/// every letter so the millivolt figures are not a guess.
uint16_t vdda_mv = 3300;

void wait_us(uint32_t us) {
    const uint32_t per_us = SysClock::hz / 1'000'000u;
    const uint32_t period = stk()->CMPLR + 1u;
    uint32_t last = stk()->CNTL;
    uint32_t acc = 0;
    while (acc / per_us < us) {
        const uint32_t now = stk()->CNTL;
        acc += (now >= last) ? (now - last) : (now + period - last);
        last = now;
    }
}

/// The converter up, the supply measured, every pad this suite owns
/// back to a floating input and both amplifiers off.
void bring_up() {
    (void)Adc<1>::init(clock, AdcConfig{.internal_sources = true});
    Adc<1>::sample_time_all(adc_sample_longest);
    Adc<1>::select(AdcInput::vrefint);
    vdda_mv = Adc<1>::vdda_mv(Adc<1>::read_settled(4));
}

void all_off() {
    Opa<1>::release();
    Opa<2>::release();
    P1a::pin::release();
    P1b::pin::release();
    N1a::pin::release();
    N1b::pin::release();
    O1a::pin::release();
    O1b::pin::release();
    P2a::pin::release();
    P2b::pin::release();
    N2a::pin::release();
    N2b::pin::release();
    O2a::pin::release();
    O2b::pin::release();
    Adc<1>::release();
}

/// One settled conversion of a channel.
uint16_t read_channel(uint8_t ch) {
    Adc<1>::select_channel(ch);
    return Adc<1>::read_settled(3);
}

uint16_t millivolts(uint16_t counts) { return Adc<1>::millivolts(counts, vdda_mv); }

// ===========================================================================
// a - OPA2 as a comparator, both ways round
// ===========================================================================
void ta_comparator() {
    all_off();
    bring_up();

    // The second input pair (PA7 positive, PA5 negative) and the first
    // output (PA2, channel 2). The inputs are driven by the port; the
    // output is handed to the analog multiplexer.
    P2b::pin::output(true);
    N2b::pin::output(false);
    O2a::claim();
    const bool cfg = Opa<2>::configure({.positive = OpaPin::positive1,
                                        .negative = OpaPin::negative1,
                                        .output = OpaPin::out0});

    const uint16_t idle = read_channel(O2a::adc_channel);
    Opa<2>::enable(true);
    wait_us(1000);
    const uint16_t high = read_channel(O2a::adc_channel);

    // The inputs swapped: the same amplifier, the other rail.
    P2b::pin::clear();
    N2b::pin::set();
    wait_us(1000);
    const uint16_t low = read_channel(O2a::adc_channel);

    // And back, so the verdict is about the inputs and not about time.
    P2b::pin::set();
    N2b::pin::clear();
    wait_us(1000);
    const uint16_t high_again = read_channel(O2a::adc_channel);

    Opa<2>::enable(false);
    wait_us(1000);
    const uint16_t off = read_channel(O2a::adc_channel);

    print(serial, "  OPA2, positive PA7 negative PA5, output PA2 (channel ",
          O2a::adc_channel, "): before the enable the pad floats and reads ", idle, " counts",
          crlf);
    print(serial, "  P above N: ", high, " counts = ", millivolts(high),
          " mV; P below N: ", low, " counts = ", millivolts(low), " mV; back again: ",
          high_again, crlf);
    print(serial, "  disabled again it floats again, holding ", off,
          " counts of whatever charge was left on it", crlf);
    bench.verdict("the amplifier takes its three selections", cfg);
    bench.verdict("with its positive input above its negative one the output saturates to "
                  "the upper rail",
                  high >= 3900u);
    bench.verdict("with them swapped it saturates to the lower one", low <= 100u);
    bench.verdict("and it follows the inputs back", high_again >= 3900u);
    bench.verdict("THE OUTPUT REACHES THE CONVERTER THROUGH THE PAD: an OPA output pad is an "
                  "ADC input pad, which is the whole of this block's route to the converter",
                  high != low);

    all_off();
}

// ===========================================================================
// b - OPA2's other pads
// ===========================================================================
/// Drive one pad and read the other against the OPPOSITE pull: a
/// floating neighbour that merely follows is not taken for a strap, and
/// both pads are left as they were found.
template <class Driver, class Reader>
bool pad_strapped() {
    Reader::input(PinPull::down);
    Driver::output(true);
    wait_us(20);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    wait_us(20);
    const bool low = !Reader::read();
    Driver::release();
    Reader::release();
    return high && low;
}

void tb_selections() {
    all_off();
    bring_up();

    // THE OTHER OUTPUT PAD IS ONLY THIS BOARD'S IF NOTHING IS STRAPPED
    // TO IT: PA4 is OPA2's OUT1 and a bus select on another desk, and a
    // push-pull neighbour on the same node would answer for the
    // amplifier. The strap is probed HERE, before anything is driven,
    // and never cached - a wire can leave the desk between two letters.
    const bool out1_strapped = pad_strapped<O2b::pin, N2b::pin>();

    // Both input pairs are driven, one pair at each rail, so that
    // SELECTING the other pair is what changes the output.
    P2a::pin::output(true);    // PB14 high
    N2a::pin::output(false);   // PB10 low
    P2b::pin::output(false);   // PA7 low
    N2b::pin::output(true);    // PA5 high
    O2a::claim();
    O2b::claim();

    (void)Opa<2>::configure({.positive = OpaPin::positive0,
                             .negative = OpaPin::negative0,
                             .output = OpaPin::out0});
    Opa<2>::enable(true);
    wait_us(1000);
    const uint16_t pair0 = read_channel(O2a::adc_channel);

    (void)Opa<2>::positive(OpaPin::positive1);
    (void)Opa<2>::negative(OpaPin::negative1);
    wait_us(1000);
    const uint16_t pair1 = read_channel(O2a::adc_channel);

    if (out1_strapped) {
        print(serial, "  SKIPPED, no verdict claimed for the output move: PA4 (OPA2's OUT1) "
                      "is strapped to PA5, which this letter drives - a wire beats an "
                      "amplifier, and the pad would answer for the strap and not for the "
                      "selection.",
              crlf);
        bench.verdict("the driver says which channel the selected output lands on",
                      Opa<2>::adc_channel() == O2a::adc_channel);
        all_off();
        return;
    }
    const uint16_t other_before = read_channel(O2b::adc_channel);
    (void)Opa<2>::output(OpaPin::out1);
    wait_us(1000);
    const uint16_t other_after = read_channel(O2b::adc_channel);
    const uint16_t first_after = read_channel(O2a::adc_channel);
    const uint8_t routed = Opa<2>::adc_channel();

    print(serial, "  the CH0 pair (PB14 high, PB10 low) puts the output at ", pair0,
          " counts; the CH1 pair (PA7 low, PA5 high) at ", pair1, crlf);
    print(serial, "  moved to OUT1 (PA4, channel ", O2b::adc_channel, "): that pad read ",
          other_before, " counts before and ", other_after, " after, while OUT0 fell to ",
          first_after, crlf);
    bench.verdict("the positive input is a choice of two pads, and the choice is what the "
                  "output follows",
                  pair0 >= 3900u);
    bench.verdict("so is the negative one: the same amplifier on the other pair reads the "
                  "other rail",
                  pair1 <= 100u);
    bench.verdict("the output is a choice of two pads too: the one just selected carries the "
                  "level",
                  other_after <= 100u && other_before != other_after);
    bench.verdict("and the one it left stops being driven", first_after > other_after + 100u);
    bench.verdict("the driver says which channel the selected output lands on",
                  routed == O2b::adc_channel);

    all_off();
}

// ===========================================================================
// c - OPA1 on its own six pads
// ===========================================================================
void tc_first_amplifier() {
    all_off();
    bring_up();

    P1b::pin::output(true);    // PB0 high
    N1b::pin::output(false);   // PA6 low
    O1a::claim();
    (void)Opa<1>::configure({.positive = OpaPin::positive1,
                             .negative = OpaPin::negative1,
                             .output = OpaPin::out0});
    Opa<1>::enable(true);
    wait_us(1000);
    const uint16_t high = read_channel(O1a::adc_channel);

    P1b::pin::clear();
    N1b::pin::set();
    wait_us(1000);
    const uint16_t low = read_channel(O1a::adc_channel);

    // Its other input pair and its other output.
    P1a::pin::output(true);    // PB15 high
    N1a::pin::output(false);   // PB11 low
    O1b::claim();
    (void)Opa<1>::configure({.positive = OpaPin::positive0,
                             .negative = OpaPin::negative0,
                             .output = OpaPin::out1});
    wait_us(1000);
    const uint16_t other = read_channel(O1b::adc_channel);

    print(serial, "  OPA1, positive PB0 negative PA6, output PA3 (channel ",
          O1a::adc_channel, "): ", high, " counts one way, ", low, " the other", crlf);
    print(serial, "  its CH0 pair (PB15 high, PB11 low) on OUT1 (PB1, channel ",
          O1b::adc_channel, "): ", other, " counts", crlf);
    bench.verdict("the first amplifier saturates high with its positive input above its "
                  "negative one",
                  high >= 3900u);
    bench.verdict("and low with them swapped", low <= 100u);
    bench.verdict("its other input pair and its other output work the same way, on four more "
                  "pads",
                  other >= 3900u);

    all_off();
}

// ===========================================================================
// d - the register
// ===========================================================================
void td_register() {
    all_off();
    Opa<1>::release();
    Opa<2>::release();
    const uint8_t reset1 = Opa<1>::field();
    const uint8_t reset2 = Opa<2>::field();

    // Every one of the eight selections of one amplifier, read back.
    bool all_read_back = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        const OpaConfig c{
            .positive = (i & 1u) != 0u ? OpaPin::positive1 : OpaPin::positive0,
            .negative = (i & 2u) != 0u ? OpaPin::negative1 : OpaPin::negative0,
            .output = (i & 4u) != 0u ? OpaPin::out1 : OpaPin::out0,
        };
        if (!Opa<2>::configure(c)) {
            all_read_back = false;
            continue;
        }
        const OpaConfig back = Opa<2>::configuration();
        if (back.positive != c.positive || back.negative != c.negative ||
            back.output != c.output) {
            all_read_back = false;
        }
    }

    // The two amplifiers share one register and must not disturb each
    // other: four bits apart, and the enable is one of the four.
    Opa<1>::release();
    (void)Opa<2>::configure({.positive = OpaPin::positive1,
                             .negative = OpaPin::negative1,
                             .output = OpaPin::out1});
    Opa<2>::enable(true);
    const uint8_t two_before = Opa<2>::field();
    (void)Opa<1>::configure({.positive = OpaPin::positive1});
    Opa<1>::enable(true);
    const uint8_t two_after = Opa<2>::field();
    const uint8_t one_field = Opa<1>::field();
    Opa<2>::release();
    const uint8_t one_kept = Opa<1>::field();
    const bool one_still_on = Opa<1>::enabled();
    const bool two_now_off = !Opa<2>::enabled();

    // A configuration with a pad in the wrong role is refused.
    const bool wrong_role = !Opa<2>::configure(OpaConfig{.positive = OpaPin::out0});

    print(serial, "  the reset field of each amplifier: ", reset1, " and ", reset2,
          "; OPA2 configured reads ", two_before, ", and still ", two_after,
          " after OPA1 was configured beside it (OPA1's own field ", one_field, ")", crlf);
    bench.verdict("both amplifiers are at zero out of reset - no enable, both selections the "
                  "first pad",
                  reset1 == 0u && reset2 == 0u);
    bench.verdict("every one of the eight selections of an amplifier is written and read back",
                  all_read_back);
    bench.verdict("the two amplifiers share one register and four bits apart: configuring "
                  "one leaves the other exactly as it was",
                  two_before == two_after && two_before == 0xFu);
    bench.verdict("and releasing one leaves the other running", one_kept == one_field &&
                                                                    one_still_on && two_now_off);
    bench.verdict("a configuration that puts a pad in the wrong role is refused", wrong_role);

    all_off();
}

// ===========================================================================
// e - the output itself
// ===========================================================================
void te_output() {
    all_off();
    bring_up();

    P2b::pin::output(true);
    N2b::pin::output(false);
    O2a::claim();
    (void)Opa<2>::configure({.positive = OpaPin::positive1,
                             .negative = OpaPin::negative1,
                             .output = OpaPin::out0});
    Opa<2>::enable(true);
    wait_us(1000);
    const uint16_t voh = read_channel(O2a::adc_channel);
    P2b::pin::clear();
    N2b::pin::set();
    wait_us(1000);
    const uint16_t vol = read_channel(O2a::adc_channel);
    const uint16_t voh_mv = millivolts(voh);
    const uint16_t vol_mv = millivolts(vol);
    const uint16_t drop = static_cast<uint16_t>(vdda_mv > voh_mv ? vdda_mv - voh_mv : 0u);

    // THE DRIVE, against the pad's own pull. The output pad is left a
    // PULLED input - which is a 40 kOhm source of its own - and the
    // amplifier is asked for the opposite rail.
    Opa<2>::enable(false);
    O2a::pin::input(PinPull::down);
    wait_us(1000);
    const uint16_t pulled_only = read_channel(O2a::adc_channel);
    P2b::pin::set();
    N2b::pin::clear();
    Opa<2>::enable(true);
    wait_us(1000);
    const uint16_t pulled_and_driven = read_channel(O2a::adc_channel);

    print(serial, "  saturated: ", voh, " counts = ", voh_mv, " mV high (", drop,
          " mV under a supply of ", vdda_mv, ") and ", vol, " counts = ", vol_mv, " mV low",
          crlf);
    print(serial, "  the datasheet rates the high saturation at VDDA-45 mV into 4 kOhm and "
                  "VDDA-10 into 20 kOhm, and the low one at 0.5 mV; there is no load here but "
                  "the converter's own sampling",
          crlf);
    print(serial, "  the output pad held DOWN by its own 40 kOhm pull reads ", pulled_only,
          " counts with the amplifier off and ", pulled_and_driven, " with it on", crlf);
    bench.verdict("the high saturation is within a tenth of a volt of the supply",
                  drop < 100u);
    bench.verdict("the low one is at ground", vol_mv <= 30u);
    bench.verdict("the amplifier drives its pad against that pad's own pull: 600 uA of drive "
                  "against 40 kOhm is not a contest",
                  pulled_only <= 100u && pulled_and_driven >= 3900u);

    all_off();
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf, "test_v203_opa - the two amplifiers of RM ch. 30", crlf,
          "  no wires: this block has no internal feedback, so with none strapped it is an "
          "open-loop stage - a comparator - read by the ADC on its output pad",
          crlf);
    bench.menu();
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "OPA2 as a comparator, its output read by the converter", ta_comparator);
    bench.letter('b', "its other input pair and its other output pad", tb_selections);
    bench.letter('c', "OPA1 on its own six pads", tc_first_amplifier);
    bench.letter('d', "the register: four bits an amplifier, and they do not collide",
                 td_register);
    bench.letter('e', "the output: the saturated rails, and the drive behind them", te_output);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
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
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
