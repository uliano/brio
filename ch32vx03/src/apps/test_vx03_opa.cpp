// test_vx03_opa - the reference bench suite for the operational
// amplifiers of the CH32V203 and the CH32V303 - two on the first series,
// four on the second: ch32vx03/opa.hpp over RM ch. 30, read by the
// converter of ch32vx03/adc.hpp on the pad each output lands on, and
// where that pad is no converter input (the CH32V303's second outputs,
// on port E) read as a digital level against the opposite pull.
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
// PLL on the HSI at 96 MHz exactly as test_vx03_adc does: ADCCLK
// 12 MHz, in specification, no crystal needed.
//
// THE PADS, all of them free on the CH32V203's board. OPA1: PB15 and PB0
// are its two positive inputs, PB11 and PA6 its two negative ones, PA3
// (channel 3) and PB1 (channel 9) its two outputs. OPA2: PB14 and PA7,
// PB10 and PA5, PA2 (channel 2) and PA4 (channel 4). On the CH32V303 the
// inputs and the first outputs are the same pads and the second outputs
// are PE15 and PE14, with OPA3 on PB13 and PC5, PB2 and PC2, PA1
// (channel 1) and PE7, and OPA4 on PB12 and PC4, PB1 and PC3, PA0
// (channel 0) and PE8; on that board's evaluation wiring several of them
// carry a wire to another pad left a floating input, which a pad driven by
// its own port or by an amplifier drives with it, and PB2 is BOOT1 behind
// ten kilohms to ground. NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14
// (the debug port), PA11/PA12 (the USB pads), PC14/PC15 and PD0/PD1 (the
// crystals), PA0 on the CH32V203's board (its KEY) - and PB2, toggled per
// command as every suite of this target does.
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
// and on the CH32V303 alone:
//   f  OPA3: both input pairs onto its first output, read by the
//      converter, and its second output on port E read as a level
//   g  OPA4, the same on its own six pads
//   h  THE HIGH-SPEED BITS: EXTEN_CTR2 asked of the die - a lot's register,
//      whose address MIRRORS EXTEN_CTR where it is absent - its four
//      OPAn_HSMD each set and read back alone where it is there, and the
//      time each amplifier's output takes to follow a rail-to-rail step of
//      its inputs' difference, in each mode the die has
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/adc.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/opa.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
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

/// A second output on port E, which no converter channel reads: its level
/// taken DIGITALLY against the pull that would say the opposite, so a pad
/// the amplifier does not drive cannot pass for one it does. The swing is
/// 0..2 V there (the CH32V303's table 4-46, note 3), and the four pads are
/// five-volt tolerant ones whose VIH is 1.63 V at 3.3 V (its tables 3-1
/// and 4-21) - which this reading is also a measurement of.
template <class Out>
bool digital_level(bool expect_high) {
    Out::pin::input(expect_high ? PinPull::down : PinPull::up);
    wait_us(200);
    const bool level = Out::pin::read();
    Out::claim();
    return level;
}

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
    if constexpr (O2b::reaches_adc) {
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
    } else {
        // OUT1 is PE14 here, which no converter channel reads: the CH1
        // pair says LOW, taken against the pad's pull-up.
        (void)Opa<2>::output(OpaPin::out1);
        wait_us(1000);
        const bool other_low = !digital_level<O2b>(false);
        // OUT0, left behind, is no longer driven: its own pull wins.
        O2a::pin::input(PinPull::up);
        wait_us(200);
        const bool first_released = O2a::pin::read();
        const uint8_t routed = Opa<2>::adc_channel();

        print(serial, "  the CH0 pair (PB14 high, PB10 low) puts the output at ", pair0,
              " counts; the CH1 pair (PA7 low, PA5 high) at ", pair1, crlf);
        print(serial, "  moved to OUT1 (PE14, no converter channel): ",
              other_low ? "LOW" : "high", " against its pull-up; OUT0 left behind reads ",
              first_released ? "its pull-up" : "LOW", crlf);
        bench.verdict("the positive input is a choice of two pads, and the choice is what the "
                      "output follows",
                      pair0 >= 3900u);
        bench.verdict("so is the negative one: the same amplifier on the other pair reads the "
                      "other rail",
                      pair1 <= 100u);
        bench.verdict("the output is a choice of two pads too: the one just selected carries the "
                      "level, read on PE14 as a level because no converter channel reads it",
                      other_low);
        bench.verdict("and the one it left stops being driven", first_released);
        bench.verdict("the driver says the selected output reaches no converter channel",
                      routed == 0xFFu);
    }

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
    uint16_t other = 0;
    if constexpr (O1b::reaches_adc) {
        other = read_channel(O1b::adc_channel);
    } else {
        // PE15 on the CH32V303: a level, against its pull-down.
        other = digital_level<O1b>(true) ? adc_max_count : uint16_t{0};
    }

    print(serial, "  OPA1, positive PB0 negative PA6, output PA3 (channel ",
          O1a::adc_channel, "): ", high, " counts one way, ", low, " the other", crlf);
    if constexpr (O1b::reaches_adc) {
        print(serial, "  its CH0 pair (PB15 high, PB11 low) on OUT1 (PB1, channel ",
              O1b::adc_channel, "): ", other, " counts", crlf);
    } else {
        print(serial, "  its CH0 pair (PB15 high, PB11 low) on OUT1 (PE15, no converter "
                      "channel): ",
              other != 0u ? "HIGH" : "low", " against its pull-down", crlf);
    }
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
// The CH32V303's letters (f..h)
// ===========================================================================
//
// Each is a template whose last parameter is the class fact that makes it
// meaningful, the work inside `if constexpr` on it, and every amplifier
// and pad it names hangs on a template parameter: on a CH32V203 the body
// is a discarded branch the compiler never instantiates.

/// The upper amplifiers' pads back to floating inputs, the amplifiers and
/// their high-speed bits off.
template <uint8_t n>
void release_upper() {
    Opa<n>::release();
    OpaIn<n, OpaPin::positive0>::pin::release();
    OpaIn<n, OpaPin::positive1>::pin::release();
    OpaIn<n, OpaPin::negative0>::pin::release();
    OpaIn<n, OpaPin::negative1>::pin::release();
    OpaOut<n, OpaPin::out0>::pin::release();
    OpaOut<n, OpaPin::out1>::pin::release();
}

// ===========================================================================
// f, g - OPA3 and OPA4: both input pairs, both outputs
// ===========================================================================
template <uint8_t n, bool on = device::has_opa(n)>
void upper_amplifier() {
    if constexpr (on) {
        using A = Opa<n>;
        using Pa = OpaIn<n, OpaPin::positive0>;
        using Pb = OpaIn<n, OpaPin::positive1>;
        using Na = OpaIn<n, OpaPin::negative0>;
        using Nb = OpaIn<n, OpaPin::negative1>;
        using Oa = OpaOut<n, OpaPin::out0>;
        using Ob = OpaOut<n, OpaPin::out1>;
        static_assert(Oa::reaches_adc && !Ob::reaches_adc);
        all_off();
        release_upper<n>();
        bring_up();

        // The CH0 pair onto OUT0, both ways round.
        Pa::pin::output(true);
        Na::pin::output(false);
        Oa::claim();
        const bool cfg = A::configure({.positive = OpaPin::positive0,
                                       .negative = OpaPin::negative0,
                                       .output = OpaPin::out0});
        A::enable(true);
        wait_us(1000);
        const uint16_t high0 = read_channel(Oa::adc_channel);
        Pa::pin::clear();
        Na::pin::set();
        wait_us(1000);
        const uint16_t low0 = read_channel(Oa::adc_channel);

        // The CH1 pair, the other way round again, so the SELECTION is
        // what moves the output: CH0 still says low.
        Pb::pin::output(true);
        Nb::pin::output(false);
        (void)A::positive(OpaPin::positive1);
        (void)A::negative(OpaPin::negative1);
        wait_us(1000);
        const uint16_t high1 = read_channel(Oa::adc_channel);

        // OUT1, on port E: read as a level, both ways.
        (void)A::output(OpaPin::out1);
        Ob::claim();
        const bool out1_high = digital_level<Ob>(true);
        Pb::pin::clear();
        Nb::pin::set();
        wait_us(1000);
        const bool out1_low = !digital_level<Ob>(false);
        // And OUT0, left behind, is no longer driven: its own pull wins.
        Oa::pin::input(PinPull::up);
        wait_us(200);
        const bool out0_released = Oa::pin::read();

        print(serial, "  OPA", n, " on OUT0 (channel ", Oa::adc_channel, "): CH0 pair ", high0,
              " one way and ", low0, " the other, CH1 pair ", high1, crlf);
        print(serial, "  on OUT1 (P", Ob::pad.port, Ob::pad.pin, ", no converter channel): ",
              out1_high ? "HIGH" : "low", " against its pull-down, ",
              out1_low ? "LOW" : "high", " against its pull-up; OUT0 left behind reads ",
              out0_released ? "its pull-up" : "LOW", crlf);
        bench.verdict(n == 3u ? "OPA3 takes its three selections and saturates to either rail "
                                "from its CH0 pair (PB13, PB2) on PA1"
                              : "OPA4 takes its three selections and saturates to either rail "
                                "from its CH0 pair (PB12, PB1) on PA0",
                      cfg && high0 >= 3900u && low0 <= 100u);
        bench.verdict(n == 3u ? "its CH1 pair (PC5, PC2) moves the same output the other way"
                              : "its CH1 pair (PC4, PC3) moves the same output the other way",
                      high1 >= 3900u);
        bench.verdict(n == 3u ? "and its second output, PE7, carries both levels where no "
                                "converter channel can read it - the 2 V swing clearing VIH"
                              : "and its second output, PE8, carries both levels where no "
                                "converter channel can read it - the 2 V swing clearing VIH",
                      out1_high && out1_low && out0_released);
        release_upper<n>();
        all_off();
    }
}

template <bool on = device::has_opa(3)>
void tf_opa3() {
    upper_amplifier<3, on>();
}
template <bool on = device::has_opa(4)>
void tg_opa4() {
    upper_amplifier<4, on>();
}

// ===========================================================================
// h - the high-speed bits, and how fast an output follows
// ===========================================================================
/// Core cycles from a DIFFERENTIAL step on an amplifier's two inputs to its
/// output pad reading the new level. Both inputs of every CH0 pair are port
/// B pads, so ONE store of that port's output register swaps them: the
/// positive and the negative input change places at the same instant, a
/// full rail-to-rail step of their difference either way. (A step to EQUAL
/// inputs would time the input offset, not the amplifier.) The output pad
/// is a floating digital input for the duration.
template <class Pos, class Neg, class Out>
uint32_t step_cycles(bool rising) {
    static_assert(Pos::pad.port == 'B' && Neg::pad.port == 'B',
                  "every CH0 pair of the four amplifiers is on port B");
    using B = Port<'B'>;
    constexpr uint32_t p = 1UL << Pos::pad.pin;
    constexpr uint32_t m = 1UL << Neg::pad.pin;
    Out::pin::input();
    const uint32_t others = B::out() & ~(p | m);
    B::out_write(others | (rising ? m : p));   // the output at the other rail
    wait_us(200);
    const uint32_t period = stk()->CMPLR + 1u;
    const uint32_t start = stk()->CNTL;
    B::out_write(others | (rising ? p : m));
    uint32_t spins = 100'000UL;
    while (Out::pin::read() != rising && spins-- != 0u) {
    }
    const uint32_t end = stk()->CNTL;
    Out::claim();
    return end >= start ? end - start : end + period - start;
}

template <uint8_t n>
void response(uint32_t (&normal)[2], uint32_t (&fast)[2], bool& fast_taken) {
    using A = Opa<n>;
    using Pos = OpaIn<n, OpaPin::positive0>;
    using Neg = OpaIn<n, OpaPin::negative0>;
    using Out = OpaOut<n, OpaPin::out0>;
    Neg::pin::output(true);
    Pos::pin::output(false);
    (void)A::configure({.positive = OpaPin::positive0,
                        .negative = OpaPin::negative0,
                        .output = OpaPin::out0});
    A::enable(true);
    (void)A::high_speed(false);
    wait_us(100);
    normal[0] = step_cycles<Pos, Neg, Out>(true);
    normal[1] = step_cycles<Pos, Neg, Out>(false);
    fast_taken = A::high_speed(true);
    if (fast_taken) {
        wait_us(100);
        fast[0] = step_cycles<Pos, Neg, Out>(true);
        fast[1] = step_cycles<Pos, Neg, Out>(false);
    }
    A::release();
    Pos::pin::release();
    Neg::pin::release();
    Out::pin::release();
}

/// The four OPAn_HSMD bits as a nibble, bit n - 1 for OPAn.
template <uint8_t first = 1>
uint8_t hsmd_nibble() {
    return static_cast<uint8_t>((Opa<first>::high_speed() ? 1u : 0u) |
                                (Opa<first + 1u>::high_speed() ? 2u : 0u) |
                                (Opa<first + 2u>::high_speed() ? 4u : 0u) |
                                (Opa<first + 3u>::high_speed() ? 8u : 0u));
}

/// OPAn's bit set on its own: does the verb say so, and does the nibble show
/// it, and it alone?
template <uint8_t n>
bool hsmd_alone() {
    const bool took = Opa<n>::high_speed(true);
    const bool alone = took && hsmd_nibble() == (1u << (n - 1u));
    (void)Opa<n>::high_speed(false);
    return alone;
}

/// On a die without EXTEN_CTR2: the verb asked to set OPAn's bit, and
/// EXTEN_CTR - which the address mirrors there - read before and after.
template <uint8_t n>
bool hsmd_refused() {
    const uint32_t before = exten()->CTR;
    const bool took = Opa<n>::high_speed(true);
    const bool read = Opa<n>::high_speed();
    Opa<n>::release();
    return !took && !read && exten()->CTR == before;
}

template <bool on = opa_class_has_four>
void th_high_speed() {
    if constexpr (on) {
        all_off();
        release_upper<3>();
        release_upper<4>();

        // THE REGISTER, ASKED OF THE DIE. EXTEN_CTR2 is a lot's (33.2.2's
        // note), and where it is not there its address mirrors EXTEN_CTR:
        // the two are told apart by what they read.
        const bool present = opa_high_speed_present();
        const uint32_t ctr = exten()->CTR;
        const uint32_t ctr2 = exten()->CTR2;
        print(serial, "  EXTEN_CTR reads ", hex(ctr), " and the EXTEN_CTR2 address ", hex(ctr2),
              present ? ": the register is there" : ": the MIRROR - this die's lot has not got it",
              crlf);
        const uint32_t ctlr_before = Opa<1>::regs().CTLR;
        if (present) {
            const uint8_t alone = static_cast<uint8_t>(
                (hsmd_alone<1>() ? 1u : 0u) + (hsmd_alone<2>() ? 1u : 0u) +
                (hsmd_alone<3>() ? 1u : 0u) + (hsmd_alone<4>() ? 1u : 0u));
            print(serial, "  EXTEN_CTR2's four OPAn_HSMD bits: ", alone,
                  " of 4 set and read back alone", crlf);
            bench.verdict("each amplifier's OPAn_HSMD bit of EXTEN_CTR2 is set, read back and "
                          "cleared on its own - the register 33.2.2's note gives to some lots of "
                          "this class only",
                          alone == 4u && hsmd_nibble() == 0u);
        } else {
            const uint8_t refused = static_cast<uint8_t>(
                (hsmd_refused<1>() ? 1u : 0u) + (hsmd_refused<2>() ? 1u : 0u) +
                (hsmd_refused<3>() ? 1u : 0u) + (hsmd_refused<4>() ? 1u : 0u));
            print(serial, "  the four high-speed verbs refused ", refused,
                  " of 4 times, EXTEN_CTR untouched", crlf);
            bench.verdict("a die whose lot has no EXTEN_CTR2 reads EXTEN_CTR's word at its "
                          "address, and the high-speed verbs answer false having written "
                          "nothing - EXTEN_CTR's USB bits and regulator trims untouched",
                          ctr2 == ctr && refused == 4u);
        }
        bench.verdict("and nothing of that touches OPA_CTLR", Opa<1>::regs().CTLR == ctlr_before);

        // THE RESPONSE: a differential rail-to-rail step on the inputs,
        // timed to the output pad's digital input.
        uint32_t normal[4][2] = {};
        uint32_t fast[4][2] = {};
        bool fast_taken[4] = {};
        response<1>(normal[0], fast[0], fast_taken[0]);
        response<2>(normal[1], fast[1], fast_taken[1]);
        response<3>(normal[2], fast[2], fast_taken[2]);
        response<4>(normal[3], fast[3], fast_taken[3]);
        constexpr uint32_t per_us = SysClock::hz / 1'000'000u;
        bool all_followed = true;
        for (uint8_t i = 0; i < 4u; ++i) {
            print(serial, "  OPA", static_cast<uint8_t>(i + 1u), ": rise ", normal[i][0],
                  " and fall ", normal[i][1], " core cycles");
            if (fast_taken[i]) {
                print(serial, "; high-speed ", fast[i][0], " and ", fast[i][1]);
            }
            print(serial, crlf);
            for (uint8_t k = 0; k < 2u; ++k) {
                if (normal[i][k] > 5u * per_us || (fast_taken[i] && fast[i][k] > 5u * per_us)) {
                    all_followed = false;
                }
            }
        }
        print(serial, "  (the datasheet rates the slew at 8 V/us, and 16 in high-speed mode; the "
                      "counts include the pads' own synchronizers and the poll)",
              crlf);
        bench.verdict("every amplifier's output follows a rail-to-rail step of its inputs' "
                      "difference within 5 us, in each mode the die has",
                      all_followed);
        all_off();
    }
}

/// The CH32V303's three letters, registered where the part has four
/// amplifiers.
template <bool on = opa_class_has_four>
void register_upper_letters() {
    if constexpr (on) {
        bench.letter('f', "OPA3: both input pairs, both outputs", tf_opa3<>);
        bench.letter('g', "OPA4: both input pairs, both outputs", tg_opa4<>);
        bench.letter('h', "the high-speed bits, and how fast an output follows", th_high_speed<>);
    }
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf,
          opa_class_has_four ? "test_vx03_opa - the four amplifiers of RM ch. 30"
                             : "test_vx03_opa - the two amplifiers of RM ch. 30",
          crlf,
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
    register_upper_letters();

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
