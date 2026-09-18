// test_stm32f4_misc - the reference bench suite for the STM32F4's three
// SMALL blocks: the CRC calculation unit, the random number generator and
// the bxCAN controller. They share a suite because none of them fills one
// and because all three are measurable with nothing attached.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, AND ONE PAD PER NODE TO CLAIM. The CRC unit is pure
// arithmetic and the generator is analog and internal. The CAN controller
// has LOOPBACK mode (RM0390 30.5.2), in which the node receives its own
// frames through an internal feedback - a whole transmit-filter-receive
// path with no transceiver and no peer - but letter c measures that it
// STILL needs eleven recessive bits on its RECEIVE PAD before it will
// leave initialization mode, so each instance claims its own CAN_RX pad
// as an input with its pull-up and never drives anything: the pad's own
// pull-up is a bus with nobody on it. That also makes the error-counter
// letter possible, which asks what happens when nobody acknowledges.
// Which pads those are belongs to the BOARD, so the CAN letters exist
// only where this file names ones the board leaves free.
//
// What is exercised, letter by letter:
//   a  the CRC unit: the reset value, one word, a run, the running value
//      composed, the silicon against the software model, CRC_IDR across
//      both kinds of reset, and the cost of a word
//   b  the generator (only on a part that has one): the clock domain, the
//      first-value discard, ten thousand words with their bit balance and
//      a byte-bucket spread, the throughput, the flags and the recovery
//   c  the CAN block: the reset state, the three modes and their
//      acknowledges, the master reset, the option bits and the one the
//      errata forbid
//   d  the bit timing: the search's exactness against CAN_BTR read back,
//      and the register's refusal outside initialization mode
//   e  loopback: every data length, both identifier formats and a remote
//      frame, round-tripped byte-exact
//   f  the filters: the four shapes admitting and rejecting, the filter
//      match index each reports, and the bank range each instance owns
//   g  the transmit scheduler: three mailboxes loaded at once, by
//      identifier and by request order
//   h  the receive FIFO: pending, full, overrun, and which message the
//      lock bit saves
//   i  four bit rates measured in loopback against the arithmetic
//   j  the recessive bus (needs the receive pad): silent mode cannot
//      transmit at all, and in normal mode with nobody to acknowledge the
//      error counter climbs to bus-off and comes back
//   k  CAN2: the shared filter block, the start bank that splits it, and
//      the slave's own loopback
//
// build: boards = f429zi,f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32f4/can.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/crc.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/rng.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
using Led = Pin<'A', 5>;
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

/// SysTick's counter as this suite's ruler: the same VAL-delta
/// accumulation the platform suite measures delay_us with, wrapped in a
/// bounded wait. Nothing (0 cycles) when the predicate never came true
/// inside `timeout_ms`.
template <typename Pred>
std::optional<uint32_t> timed_wait(Pred pred, uint32_t timeout_ms) {
    const uint32_t period = SysTick->LOAD + 1u;
    const uint32_t t0 = Ticker::ticks();
    uint32_t last = SysTick->VAL;
    uint32_t acc = 0;
    for (;;) {
        const uint32_t now = SysTick->VAL;
        acc += (last >= now) ? (last - now) : (last + period - now);
        last = now;
        if (pred()) {
            return acc;
        }
        if (Ticker::ticks() - t0 > timeout_ms) {
            return std::nullopt;
        }
    }
}

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

// =============================================================================
// a - the CRC calculation unit
// =============================================================================

// A message with no structure to it: a run of words a software model and
// a hardware unit must agree on to the bit.
constexpr uint32_t message[16] = {
    0x00000000u, 0xFFFFFFFFu, 0x12345678u, 0x9ABCDEF0u, 0x0F0F0F0Fu, 0xF0F0F0F0u,
    0xDEADBEEFu, 0xCAFEBABEu, 0x00000001u, 0x80000000u, 0x55555555u, 0xAAAAAAAAu,
    0x01234567u, 0x89ABCDEFu, 0xFEDCBA98u, 0x76543210u,
};

void ta_crc() {
    Crc::init();
    bench.verdict("the AHB1 gate reads back on", Crc::clock());

    print(serial, "  CRC_DR after a reset ", hex(Crc::value()), crlf);
    bench.verdict("a reset leaves the initial value 0xFFFFFFFF in CRC_DR",
                  Crc::value() == crc32_ethernet_init);

    // HOW LATE THE RESET IS. Put something else in CRC_DR, ask for the
    // reset, and count the reads of CRC_DR it takes before the initial
    // value appears - which is also what the driver's reset() waits for,
    // because a word written before it lands is swallowed with no flag.
    Crc::feed(0xDEADBEEFu);
    const uint8_t spins = Crc::reset();
    print(serial, "  the reset landed after ", spins, " extra read(s) of CRC_DR", crlf);
    bench.verdict("the reset lands, and inside the driver's bound",
                  spins < Crc::reset_spins && Crc::value() == crc32_ethernet_init);

    // One word, against the model - and the model is pinned to the
    // published CRC-32/MPEG-2 check value in test/family_stm32f4/crc.cpp.
    (void)Crc::reset();
    Crc::feed(0x12345678u);
    const uint32_t one = Crc::value();
    print(serial, "  one word 0x12345678 -> ", hex(one), " (model ",
          hex(crc32_ethernet(&message[2], 1)), ")", crlf);
    bench.verdict("one word is the CRC-32/MPEG-2 of its four bytes, most "
                  "significant first",
                  one == crc32_ethernet(&message[2], 1));

    // The whole run.
    const uint32_t whole = Crc::compute(message, 16);
    print(serial, "  sixteen words -> ", hex(whole), " (model ", hex(crc32_ethernet(message, 16)),
          ")", crlf);
    bench.verdict("a run of sixteen words agrees with the model",
                  whole == crc32_ethernet(message, 16));

    // Reading does not consume: the running value composes, which is what
    // makes a checksum in pieces the same as a checksum in one go.
    (void)Crc::reset();
    Crc::feed(message, 8);
    const uint32_t half = Crc::value();
    (void)half;
    Crc::feed(message + 8, 8);
    bench.verdict("a checksum taken in two halves, with a READ between them, "
                  "is the checksum of the whole",
                  Crc::value() == whole);

    // The empty message.
    (void)Crc::reset();
    bench.verdict("an empty message is the initial value", Crc::value() == crc32_ethernet_init);

    // CRC_IDR: eight bits of scratch that the RESET bit does not touch and
    // the peripheral's reset line does. The device header declares the
    // register as a BYTE where RM0390 4.4 asks for word accesses; this is
    // what the silicon does with the header's store.
    Crc::idr(0x5Au);
    const uint8_t idr_written = Crc::idr();
    (void)Crc::reset();
    const uint8_t idr_after_reset = Crc::idr();
    Crc::reset_block();
    const uint8_t idr_after_block = Crc::idr();
    const uint32_t dr_after_block = Crc::value();
    print(serial, "  CRC_IDR written 0x5A, read ", hex(idr_written), "; after CRC_CR.RESET ",
          hex(idr_after_reset), "; after the RCC reset line ", hex(idr_after_block), crlf);
    bench.verdict("a BYTE store into CRC_IDR sticks (the header's access width)",
                  idr_written == 0x5Au);
    bench.verdict("CRC_CR.RESET leaves CRC_IDR alone", idr_after_reset == 0x5Au);
    bench.verdict("the peripheral's reset line clears CRC_IDR and CRC_DR",
                  idr_after_block == 0u && dr_after_block == crc32_ethernet_init);

    // What a word costs. RM0390 4.2 says the computation takes four AHB
    // cycles and 4.3 says the write stalls until it is done, so the loop's
    // own overhead is what the measurement adds to that.
    Crc::clock(true);
    (void)Crc::reset();
    static constexpr uint32_t words = 1024;
    const uint32_t period = SysTick->LOAD + 1u;
    uint32_t last = SysTick->VAL;
    uint32_t acc = 0;
    for (uint32_t i = 0; i < words; ++i) {
        Crc::feed(message[i & 15u]);
    }
    const uint32_t now = SysTick->VAL;
    acc = (last >= now) ? (last - now) : (last + period - now);
    last = now;
    const uint32_t per_word_tenths = (acc * 10u) / words;
    print(serial, "  ", words, " words in ", acc, " core cycles = ", per_word_tenths / 10u, ".",
          per_word_tenths % 10u, " cycles a word at ", SysClock::hz / 1'000'000u, " MHz", crlf);
    bench.verdict("a word costs at least the chapter's four AHB cycles",
                  per_word_tenths >= 40u);
}

// =============================================================================
// b - the random number generator
// =============================================================================
#if defined(RNG_BASE)

void tb_rng() {
    print(serial, "  the 48 MHz domain (PLL Q) is at ", SysClock::usb_hz / 1'000'000u, " MHz, HCLK/16 is ",
          SysClock::hz / 16u / 1'000'000u, " MHz", crlf);
    bench.verdict("the clock the generator runs on clears RM0090 24.4.2's "
                  "f(HCLK)/16 bar",
                  rng_clock_ratio_ok(SysClock::usb_hz, SysClock::hz));

    const bool up = Rng::init(clock);
    print(serial, "  RNG_CR ", hex(Rng::regs().CR), " RNG_SR ", hex(Rng::regs().SR), crlf);
    bench.verdict("the generator comes up and hands out its first word "
                  "(discarded, as FIPS PUB 140-2 asks)",
                  up);
    bench.verdict("neither error status stands", !Rng::seed_error() && !Rng::clock_error());
    bench.verdict("and neither latched flag either",
                  !Rng::seed_error_flag() && !Rng::clock_error_flag());

    // Ten thousand words: the bit balance and a spread over the sixteen
    // buckets the low nibble picks. NOT a test of randomness - a chi
    // square over 10000 samples says almost nothing - but a test that the
    // datapath is not stuck, not repeating and not biased grossly.
    static constexpr uint32_t count = 10'000;
    uint32_t ones = 0;
    uint32_t buckets[16] = {};
    uint32_t taken = 0;
    uint32_t repeats = 0;
    uint32_t last_word = 0;
    const uint32_t t0 = Ticker::ticks();
    for (uint32_t i = 0; i < count; ++i) {
        const auto v = Rng::read_blocking();
        if (!v) {
            if (Rng::last_error() == RngError::repeated) {
                ++repeats;
                continue;
            }
            break;
        }
        ++taken;
        last_word = *v;
        ones += static_cast<uint32_t>(__builtin_popcount(*v));
        ++buckets[*v & 15u];
    }
    const uint32_t ms = Ticker::ticks() - t0;
    const uint32_t bits = taken * 32u;
    const uint32_t per_mille = bits == 0u ? 0u : (ones * 1000u) / bits;
    uint32_t lo = 0xFFFFFFFFu, hi = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        if (buckets[i] < lo) {
            lo = buckets[i];
        }
        if (buckets[i] > hi) {
            hi = buckets[i];
        }
    }
    print(serial, "  ", taken, " words in ", ms, " ms; ones ", per_mille,
          " per mille of ", bits, " bits; low-nibble buckets ", lo, "..", hi, " (", taken / 16u,
          " expected); the last word ", hex(last_word), crlf);
    bench.verdict("ten thousand words came out", taken == count);
    bench.verdict("no two consecutive words were equal (the continuous test)", repeats == 0u);
    bench.verdict("the bit balance is within one per cent of a half",
                  per_mille >= 490u && per_mille <= 510u);
    bench.verdict("no low-nibble bucket is empty and none holds a quarter of "
                  "the sample",
                  lo > 0u && hi < taken / 4u);
    if (ms != 0u) {
        print(serial, "  throughput ", taken / ms, " words a millisecond (", rng_clocks_per_word,
              " RNG_CLK periods a word is ", (SysClock::usb_hz / rng_clocks_per_word) / 1000u,
              " a millisecond at this rate)", crlf);
    }

    // DRDY goes down with the read and comes back on its own.
    (void)Rng::value();
    const bool cleared = !Rng::ready();
    const auto back = timed_wait([] { return Rng::ready(); }, 5u);
    print(serial, "  DRDY after a read: ", cleared ? "clear" : "still set", "; back in ",
          back ? *back / cycles_per_us : 0u, " us", crlf);
    bench.verdict("reading RNG_DR clears DRDY and the next word raises it again",
                  cleared && back.has_value());

    // The latched flags are rc_w0: a store of a ZERO clears them, and the
    // driver's verbs are what get that right.
    Rng::clear_seed_error();
    Rng::clear_clock_error();
    bench.verdict("the two latched flags clear (rc_w0, not rc_w1)",
                  !Rng::seed_error_flag() && !Rng::clock_error_flag());

    // The recovery sequence of 24.3.2 must be harmless when nothing is
    // wrong: clear, disable, enable, discard.
    bench.verdict("the seed-error recovery sequence brings the generator back "
                  "even when it had not failed",
                  Rng::recover());

    // A word already computed stays in RNG_DR with DRDY up when RNGEN goes
    // down (measured: the first read after the disable still answers), so
    // the disable is judged on what follows that word, not on it.
    Rng::enable(false);
    const bool pending_survives = Rng::read().has_value();
    (void)delay_us(clock, 100);
    print(serial, "  after the disable the word already in RNG_DR ", pending_survives ? "is still handed out" : "is gone",
          "; 100 us later a new one ", Rng::ready() ? "STANDS" : "does not come", crlf);
    bench.verdict("disabled, the generator computes no new word",
                  !Rng::enabled() && !Rng::read().has_value());
    Rng::release();
    bench.verdict("released, the clock gate is shut", !Rng::clock());
}

#endif // RNG_BASE

// =============================================================================
// The CAN letters
// =============================================================================
//
// THEY NEED ONE PAD, AND ONLY THE RECEIVER'S. Letter c measures why: even
// in loopback the node will not leave initialization mode until it has
// seen eleven recessive bits ON ITS RECEIVE PAD, so a suite with no wire
// still has to claim CAN1_RX and let the pad's own pull-up hold it
// recessive. The transmit pad is deliberately NOT claimed - the chip
// drives nothing at all, and the loopback carries the frames.
//
// Which pad that is belongs to the board, so these letters exist only
// where this file names one that the board leaves free. On the
// Nucleo-F446RE that is PA11 (DS10693 table 11: CAN1_RX on AF9; UM1724
// leaves the pad on the extension connector).
#if defined(CAN1_BASE) && defined(STM32F446xx)

using Bus = Can<1>;

constexpr uint32_t can_pclk = apb_hz(clock, false);
constexpr PinSel can_rx_pad{'A', 11, PinFunction::af9};

/// Bring CAN1 up at `bitrate`, with an accept-all filter in bank 0 feeding
/// FIFO 0. `loopback` and `silent` are CAN_BTR's two test bits.
bool can_up(uint32_t bitrate, bool loopback, bool silent, const CanOptions& o = CanOptions{}) {
    Bus::clock(true);
    Bus::filter_clock(true);
    Bus::claim_rx_pad<can_rx_pad>(PinPull::up);
    Bus::master_reset();
    if (!Bus::init_mode(true)) {
        return false;
    }
    if (!Bus::options(o)) {
        return false;
    }
    const CanTiming t = can_timing_for(can_pclk, bitrate);
    if (!Bus::timing(t, loopback, silent)) {
        return false;
    }
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter(can_filter_accept_all(0, 0));
    (void)Bus::filter_init(false);
    return Bus::start();
}

/// Send one frame and wait for the mailbox to report; then wait for the
/// loopback copy. Nothing when either wait timed out.
std::optional<CanFrame> can_round_trip(const CanFrame& out, uint32_t timeout_ms = 20u) {
    const auto mb = Bus::transmit(out);
    if (!mb) {
        return std::nullopt;
    }
    if (!timed_wait([&] { return Bus::result(*mb).completed; }, timeout_ms)) {
        return std::nullopt;
    }
    const bool ok = Bus::result(*mb).ok;
    (void)Bus::clear_result(*mb);
    if (!ok) {
        return std::nullopt;
    }
    if (!timed_wait([] { return Bus::pending(0) != 0u; }, timeout_ms)) {
        return std::nullopt;
    }
    return Bus::receive(0);
}

bool frames_equal(const CanFrame& a, const CanFrame& b) {
    if (a.id != b.id || a.extended != b.extended || a.remote != b.remote || a.length != b.length) {
        return false;
    }
    if (a.remote) {
        return true;   // a remote frame carries no data
    }
    for (uint8_t i = 0; i < a.length; ++i) {
        if (a.data[i] != b.data[i]) {
            return false;
        }
    }
    return true;
}

/// Bring the block from a master reset to normal mode in one test mode or
/// the other, and say whether it got there. What this really asks is
/// whether the receiver saw eleven recessive bits with nothing attached.
bool try_start(bool loopback, bool silent) {
    Bus::master_reset();
    if (!Bus::init_mode(true)) {
        return false;
    }
    if (!Bus::timing(can_timing_for(can_pclk, 1'000'000u), loopback, silent)) {
        return false;
    }
    return Bus::start() && Bus::in_normal();
}

// =============================================================================
// c - the block and its three modes
// =============================================================================
void tc_can_modes() {
    Bus::clock(true);
    Bus::filter_clock(true);
    Bus::master_reset();
    print(serial, "  after a master reset: MCR ", hex(Bus::regs().MCR), " MSR ",
          hex(Bus::regs().MSR), crlf);
    bench.verdict("the reset state is SLEEP, not normal (SLAK set, INAK clear)",
                  Bus::in_sleep() && !Bus::in_init());

    bench.verdict("initialization is requested and acknowledged", Bus::init_mode(true));
    bench.verdict("and it clears the sleep the reset left", Bus::in_init() && !Bus::in_sleep());

    // The options, all of them, read back.
    const CanOptions all{.time_triggered = false,
                         .auto_bus_off = true,
                         .auto_wakeup = true,
                         .no_retransmit = true,
                         .fifo_locked = true,
                         .tx_fifo_priority = true,
                         .debug_freeze = false};
    const bool wrote = Bus::options(all);
    const CanOptions back = Bus::options();
    bench.verdict("every option bit of CAN_MCR is written and reads back",
                  wrote && back.auto_bus_off && back.auto_wakeup && back.no_retransmit &&
                      back.fifo_locked && back.tx_fifo_priority && !back.debug_freeze);
    bench.verdict("and the mode request is not disturbed by writing them", Bus::in_init());

    // The one the errata forbid.
    const bool ttcm_refused = !Bus::options(CanOptions{.time_triggered = true});
    print(serial, "  time-triggered mode is ",
          Bus::time_triggered_supported() ? "not filed against this part class"
                                          : "refused (the errata sheet forbids it)",
          crlf);
    bench.verdict("TTCM is refused where the errata sheet read for this part "
                  "class says the mode does not work",
                  Bus::time_triggered_supported() != ttcm_refused);
    bench.verdict("and a refused configuration writes nothing", !Bus::options().time_triggered);

    // A timing is needed before the block can leave initialization, and a
    // test mode is what makes the exit possible with no bus: leaving
    // initialization costs eleven consecutive recessive bits on the
    // receiver's input, and WHICH INPUT THAT IS depends on the mode.
    const CanTiming t = can_timing_for(can_pclk, 1'000'000u);
    bench.verdict("CAN_BTR takes the timing while INAK stands", Bus::timing(t, true, true));

    // FOUR CASES, AND THEY SAY WHAT THE RECEIVER'S INPUT REALLY IS. 30.5.2
    // says loop back mode disregards "the actual value of the CANRX input
    // pin", which would make a pad unnecessary - and the silicon disagrees
    // about the SYNCHRONIZATION that leaves initialization mode. The two
    // judged cases are the two DEFINED ones: a pad held dominant and a pad
    // held recessive. An unclaimed pad is neither (its level is nobody's),
    // so its two results are printed and not judged.
    Pin<can_rx_pad.port, can_rx_pad.pin>::release();
    const bool no_pad = try_start(true, false);
    const bool no_pad_silent = try_start(true, true);
    Bus::claim_rx_pad<can_rx_pad>(PinPull::down);
    const bool pad_dominant = try_start(true, false);
    Bus::claim_rx_pad<can_rx_pad>(PinPull::up);
    const bool pad_recessive = try_start(true, false);
    print(serial, "  leaving initialization in loopback: pad unclaimed ",
          no_pad ? "yes" : "no", ", unclaimed + silent ", no_pad_silent ? "yes" : "no",
          " (both undefined - an unclaimed pad has no level); claimed and pulled DOWN ",
          pad_dominant ? "yes" : "no", ", claimed and pulled UP ",
          pad_recessive ? "yes" : "no", crlf);
    bench.verdict("a node in LOOPBACK still needs eleven recessive bits ON ITS "
                  "RECEIVE PAD to leave initialization mode - the pin is "
                  "disregarded for the DATA and not for the synchronization",
                  !pad_dominant && pad_recessive);
    bench.verdict("normal mode is reached: neither acknowledge stands", Bus::in_normal());
    print(serial, "  MSR ", hex(Bus::regs().MSR), ": RX reads ", Bus::rx_level() ? "1" : "0",
          ", the last sample point ", Bus::last_sample() ? "1" : "0", crlf);

    // Sleep and back, both acknowledged. Leaving sleep synchronizes too,
    // which is why this is done in loopback and not before it.
    bench.verdict("sleep is requested and acknowledged", Bus::sleep(true) && Bus::in_sleep());
    bench.verdict("and left again, which costs another synchronization",
                  Bus::sleep(false) && !Bus::in_sleep());
    Bus::clear_sleep_ack_flag();

    Bus::master_reset();
    bench.verdict("the master reset puts the block back in sleep with CAN_MCR "
                  "at its reset value",
                  Bus::in_sleep() && !Bus::options().auto_bus_off);
}

// =============================================================================
// d - the bit timing
// =============================================================================
void td_can_timing() {
    print(serial, "  PCLK1 is ", can_pclk / 1'000'000u, " MHz", crlf);

    static constexpr uint32_t rates[4] = {1'000'000u, 500'000u, 250'000u, 125'000u};
    bool all_exact = true;
    for (uint8_t i = 0; i < 4u; ++i) {
        const CanTiming t = can_timing_for(can_pclk, rates[i]);
        const uint32_t got = can_bitrate_of(can_pclk, t);
        print(serial, "  ", rates[i] / 1000u, " kbit/s: BRP ", t.brp, " TS1 ", t.ts1, " TS2 ",
              t.ts2, " SJW ", t.sjw, " = ", can_timing_quanta(t), " tq, sample point ",
              can_sample_point_of(t), " per mille, rate ", got, crlf);
        if (!can_timing_valid(t) || got != rates[i] || can_sample_point_of(t) < 750u ||
            can_sample_point_of(t) > 900u) {
            all_exact = false;
        }
    }
    bench.verdict("the search makes all four classic rates EXACTLY, with a "
                  "sample point between 75 and 90 per cent",
                  all_exact);

    // The register holds what the search found, in the chapter's off-by-one
    // encoding.
    const CanTiming t = can_timing_for(can_pclk, 500'000u);
    Bus::clock(true);
    Bus::master_reset();
    (void)Bus::init_mode(true);
    const bool wrote = Bus::timing(t, true, true);
    const CanTiming read_back = Bus::timing();
    print(serial, "  CAN_BTR ", hex(Bus::regs().BTR), crlf);
    bench.verdict("CAN_BTR reads back the timing that was written",
                  wrote && read_back.brp == t.brp && read_back.ts1 == t.ts1 &&
                      read_back.ts2 == t.ts2 && read_back.sjw == t.sjw);
    bench.verdict("and the two test-mode bits with it", Bus::loopback() && Bus::silent());

    // 30.9.1: the register is writable only in initialization mode, and
    // the driver refuses rather than let a silent drop happen.
    (void)Bus::start();
    const CanTiming other = can_timing_for(can_pclk, 125'000u);
    const bool refused = !Bus::timing(other, true, false);
    bench.verdict("a timing written outside initialization mode is REFUSED", refused);
    bench.verdict("and the register still holds the old one", Bus::timing().brp == t.brp);

    // A rate the APB clock cannot divide into exactly has no timing at all.
    bench.verdict("a rate this PCLK1 cannot make exactly has no timing",
                  !can_timing_valid(can_timing_for(can_pclk, 800'000u)) ||
                      can_bitrate_of(can_pclk, can_timing_for(can_pclk, 800'000u)) == 800'000u);
    Bus::master_reset();
}

// =============================================================================
// e - a frame round-trips in loopback
// =============================================================================
void te_can_loopback() {
    bench.verdict("the node comes up in loopback at 1 Mbit/s",
                  can_up(1'000'000u, true, false));

    // Every data length, byte-exact.
    bool all_dlc = true;
    for (uint8_t dlc = 0; dlc <= 8u; ++dlc) {
        CanFrame out{};
        out.id = 0x100u + dlc;
        out.length = dlc;
        for (uint8_t i = 0; i < dlc; ++i) {
            out.data[i] = static_cast<uint8_t>(0xA0u + i + dlc);
        }
        const auto in = can_round_trip(out);
        if (!in || !frames_equal(out, *in)) {
            all_dlc = false;
            print(serial, "  DLC ", dlc, ": ", in ? "wrong" : "no frame", crlf);
        }
    }
    bench.verdict("every data length from zero to eight round-trips byte-exact", all_dlc);

    // The extended format: twenty-nine bits through the same path.
    CanFrame ext{};
    ext.id = 0x1ABCDEF1u;
    ext.extended = true;
    ext.length = 8;
    for (uint8_t i = 0; i < 8u; ++i) {
        ext.data[i] = static_cast<uint8_t>(i * 17u);
    }
    const auto ext_in = can_round_trip(ext);
    print(serial, "  extended ", hex(ext.id), " -> ",
          ext_in ? hex(ext_in->id) : hex(uint32_t{0}), ext_in && ext_in->extended ? " (IDE)" : "",
          crlf);
    bench.verdict("a 29-bit identifier survives the round trip",
                  ext_in && frames_equal(ext, *ext_in));

    // The widest and the narrowest identifiers of each format.
    CanFrame edge{};
    edge.id = can_std_id_max;
    edge.length = 1;
    edge.data[0] = 0x5A;
    const auto edge_in = can_round_trip(edge);
    bench.verdict("the highest standard identifier round-trips",
                  edge_in && frames_equal(edge, *edge_in));
    edge.id = 0;
    const auto zero_in = can_round_trip(edge);
    bench.verdict("and identifier zero", zero_in && frames_equal(edge, *zero_in));

    // A remote frame: the request carries a length and no data.
    CanFrame rtr{};
    rtr.id = 0x321;
    rtr.remote = true;
    rtr.length = 4;
    const auto rtr_in = can_round_trip(rtr);
    print(serial, "  remote frame: ", rtr_in ? "received" : "none", " RTR ",
          rtr_in && rtr_in->remote ? "set" : "clear", " DLC ", rtr_in ? rtr_in->length : 0u, crlf);
    bench.verdict("a remote frame arrives with RTR set and its length intact",
                  rtr_in && rtr_in->remote && rtr_in->length == 4u && rtr_in->id == 0x321u);

    // A frame the vocabulary refuses never reaches a mailbox.
    CanFrame bad{};
    bad.id = 0x800;   // twelve bits, and the standard format has eleven
    bench.verdict("a frame with an identifier too wide for its format is "
                  "refused before the mailbox",
                  !Bus::transmit(bad).has_value());
    bad.id = 0;
    bad.length = 9;
    bench.verdict("and one with more than eight bytes", !Bus::transmit(bad).has_value());
    Bus::master_reset();
}

// =============================================================================
// f - the filters
// =============================================================================

/// Whether a frame with this identifier reaches FIFO 0, and with which
/// filter match index.
struct Admitted {
    bool arrived = false;
    uint8_t fmi = 0;
};

Admitted try_id(uint32_t id, bool extended = false, bool remote = false) {
    CanFrame out{};
    out.id = id;
    out.extended = extended;
    out.remote = remote;
    out.length = 1;
    out.data[0] = 0x11;
    const auto mb = Bus::transmit(out);
    Admitted a{};
    if (!mb) {
        return a;
    }
    if (!timed_wait([&] { return Bus::result(*mb).completed; }, 20u)) {
        return a;
    }
    (void)Bus::clear_result(*mb);
    // A frame the filters reject is transmitted and simply never stored;
    // a millisecond is a hundred bit times at this rate, so the absence is
    // decided and not merely early.
    if (!timed_wait([] { return Bus::pending(0) != 0u; }, 2u)) {
        return a;
    }
    const auto in = Bus::receive(0);
    if (in) {
        a.arrived = true;
        a.fmi = in->filter_index;
    }
    return a;
}

void tf_can_filters() {
    bench.verdict("the node comes up in loopback at 1 Mbit/s",
                  can_up(1'000'000u, true, false));

    print(serial, "  CAN1 owns banks ", Bus::first_bank(), "..", Bus::bank_limit() - 1u,
          " of ", Bus::filter_banks, " (CAN2SB reads ", Bus::start_bank(), ")", crlf);
    bench.verdict("a bank past the block's end is refused",
                  !Bus::filter(can_filter_accept_all(Bus::filter_banks)));

    // One 32-bit MASK filter: the top seven bits of the identifier must
    // match, the rest is don't-care.
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter(CanFilter{0, CanFilterScale::single32, CanFilterMode::mask, 0,
                                can_filter32(0x240u, false, false),
                                can_filter32(0x7F0u, false, false), true});
    (void)Bus::filter_init(false);
    const Admitted in_mask = try_id(0x24Au);
    const Admitted out_mask = try_id(0x250u);
    print(serial, "  32-bit mask 0x240/0x7F0: 0x24A ", in_mask.arrived ? "admitted" : "rejected",
          " (FMI ", in_mask.fmi, "), 0x250 ", out_mask.arrived ? "admitted" : "rejected", crlf);
    bench.verdict("a 32-bit mask filter admits what matches under the mask and "
                  "rejects what does not",
                  in_mask.arrived && !out_mask.arrived);

    // One 32-bit LIST filter bank: two identifiers, exactly.
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter(CanFilter{0, CanFilterScale::single32, CanFilterMode::list, 0,
                                can_filter32(0x111u, false, false),
                                can_filter32(0x222u, false, false), true});
    (void)Bus::filter_init(false);
    const Admitted list_a = try_id(0x111u);
    const Admitted list_b = try_id(0x222u);
    const Admitted list_c = try_id(0x333u);
    print(serial, "  32-bit list {0x111, 0x222}: FMI ", list_a.fmi, " and ", list_b.fmi,
          "; 0x333 ", list_c.arrived ? "admitted" : "rejected", crlf);
    bench.verdict("a 32-bit list bank admits its two identifiers and nothing else",
                  list_a.arrived && list_b.arrived && !list_c.arrived);
    bench.verdict("and the two halves report DIFFERENT filter match indices, "
                  "one apart",
                  list_b.fmi == list_a.fmi + 1u);

    // Four 16-bit LIST filters in one bank.
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter(CanFilter{0, CanFilterScale::dual16, CanFilterMode::list, 0,
                                can_filter16_pair(can_filter16(0x401u, false, false),
                                                  can_filter16(0x402u, false, false)),
                                can_filter16_pair(can_filter16(0x403u, false, false),
                                                  can_filter16(0x404u, false, false)),
                                true});
    (void)Bus::filter_init(false);
    const Admitted q1 = try_id(0x401u);
    const Admitted q4 = try_id(0x404u);
    const Admitted q5 = try_id(0x405u);
    print(serial, "  16-bit list {0x401..0x404}: FMI ", q1.fmi, " and ", q4.fmi, "; 0x405 ",
          q5.arrived ? "admitted" : "rejected", crlf);
    bench.verdict("a 16-bit list bank holds FOUR identifiers, each with its own "
                  "match index",
                  q1.arrived && q4.arrived && !q5.arrived && q4.fmi == q1.fmi + 3u);

    // Two 16-bit MASK filters in one bank: identifier and mask side by
    // side in each register.
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter(CanFilter{0, CanFilterScale::dual16, CanFilterMode::mask, 0,
                                can_filter16_pair(can_filter16(0x500u, false, false),
                                                  can_filter16(0x7F0u, false, false)),
                                can_filter16_pair(can_filter16(0x600u, false, false),
                                                  can_filter16(0x7FFu, false, false)),
                                true});
    (void)Bus::filter_init(false);
    const Admitted m1 = try_id(0x50Au);
    const Admitted m2 = try_id(0x600u);
    const Admitted m3 = try_id(0x601u);
    print(serial, "  16-bit mask {0x500/0x7F0, 0x600/0x7FF}: 0x50A ",
          m1.arrived ? "admitted" : "rejected", ", 0x600 ", m2.arrived ? "admitted" : "rejected",
          ", 0x601 ", m3.arrived ? "admitted" : "rejected", crlf);
    bench.verdict("two 16-bit mask filters in one bank each admit their own "
                  "group",
                  m1.arrived && m2.arrived && !m3.arrived);

    // A deactivated bank admits nothing, and nothing else is left to catch
    // the frame.
    (void)Bus::filter_active(0, false);
    bench.verdict("with every bank deactivated the hardware discards the frame "
                  "and the software never sees it",
                  !try_id(0x50Au).arrived);

    // A filter on FIFO 1.
    (void)Bus::filter_init(true);
    (void)Bus::filter(CanFilter{0, CanFilterScale::single32, CanFilterMode::mask, 1, 0u, 0u, true});
    (void)Bus::filter_init(false);
    CanFrame f1{};
    f1.id = 0x77;
    f1.length = 2;
    const auto mb = Bus::transmit(f1);
    const bool sent = mb && timed_wait([&] { return Bus::result(*mb).completed; }, 20u).has_value();
    if (mb) {
        (void)Bus::clear_result(*mb);
    }
    const bool on_one = timed_wait([] { return Bus::pending(1) != 0u; }, 5u).has_value();
    print(serial, "  a bank assigned to FIFO 1: FIFO 0 has ", Bus::pending(0), ", FIFO 1 has ",
          Bus::pending(1), crlf);
    bench.verdict("CAN_FFA1R sends the frame to the other FIFO",
                  sent && on_one && Bus::pending(0) == 0u);
    (void)Bus::receive(1);

    // The bank's whole configuration reads back.
    const auto held = Bus::filter(0);
    bench.verdict("a bank reads back the scale, mode, FIFO and activation it "
                  "was given",
                  held && held->scale == CanFilterScale::single32 &&
                      held->mode == CanFilterMode::mask && held->fifo == 1u && held->active);

    // A MASTER RESET DOES NOT TOUCH THE FILTER BLOCK (30.9.2 lists CAN_MCR
    // and the FMP bits and nothing else), so a letter that configured banks
    // puts them back itself.
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter_init(false);
    Bus::master_reset();
}

// =============================================================================
// g - the transmit scheduler
// =============================================================================

/// Load three mailboxes WHILE THE BLOCK IS IN INITIALIZATION MODE, where
/// no transfer starts, so that all three are pending the moment normal
/// mode is reached and the scheduler really has to choose.
bool load_three(bool by_request_order) {
    Bus::clock(true);
    Bus::filter_clock(true);
    Bus::master_reset();
    if (!Bus::init_mode(true)) {
        return false;
    }
    if (!Bus::options(CanOptions{.tx_fifo_priority = by_request_order})) {
        return false;
    }
    if (!Bus::timing(can_timing_for(can_pclk, 125'000u), true, false)) {
        return false;
    }
    (void)Bus::filter_init(true);
    (void)Bus::filters_off();
    (void)Bus::filter(can_filter_accept_all(0, 0));
    (void)Bus::filter_init(false);

    static constexpr uint32_t ids[3] = {0x300u, 0x100u, 0x200u};
    for (uint8_t i = 0; i < 3u; ++i) {
        CanFrame f{};
        f.id = ids[i];
        f.length = 1;
        f.data[0] = static_cast<uint8_t>(i);
        if (!Bus::transmit(f)) {
            return false;
        }
    }
    return Bus::free_mailboxes() == 0u && Bus::start();
}

void tg_can_priority() {
    // By identifier: the lowest goes first, whatever order the mailboxes
    // were loaded in.
    bool ok = load_three(false);
    uint32_t order[3] = {};
    for (uint8_t i = 0; i < 3u; ++i) {
        if (!timed_wait([] { return Bus::pending(0) != 0u; }, 50u)) {
            ok = false;
            break;
        }
        const auto f = Bus::receive(0);
        if (!f) {
            ok = false;
            break;
        }
        order[i] = f->id;
    }
    print(serial, "  loaded 0x300, 0x100, 0x200; by identifier they arrived ", hex(order[0]),
          " ", hex(order[1]), " ", hex(order[2]), crlf);
    bench.verdict("with TXFP clear the scheduler transmits by IDENTIFIER, "
                  "lowest first",
                  ok && order[0] == 0x100u && order[1] == 0x200u && order[2] == 0x300u);

    // By request order: the mailboxes go out as they were filled.
    ok = load_three(true);
    for (uint8_t i = 0; i < 3u; ++i) {
        if (!timed_wait([] { return Bus::pending(0) != 0u; }, 50u)) {
            ok = false;
            break;
        }
        const auto f = Bus::receive(0);
        if (!f) {
            ok = false;
            break;
        }
        order[i] = f->id;
    }
    print(serial, "  with TXFP set they arrived ", hex(order[0]), " ", hex(order[1]), " ",
          hex(order[2]), crlf);
    bench.verdict("with TXFP set it transmits in REQUEST order", ok && order[0] == 0x300u &&
                                                                     order[1] == 0x100u &&
                                                                     order[2] == 0x200u);

    // The mailbox bookkeeping: three free again, and TSR.CODE names the
    // next one.
    print(serial, "  free mailboxes ", Bus::free_mailboxes(), ", CODE names mailbox ",
          Bus::next_mailbox(), crlf);
    bench.verdict("all three mailboxes are empty when the queue has drained",
                  Bus::free_mailboxes() == 3u);

    // An abort while nothing is pending has no effect; one on a pending
    // request completes it.
    bench.verdict("an abort on an empty mailbox changes nothing",
                  Bus::abort(0) && Bus::mailbox_empty(0));
    Bus::master_reset();
}

// =============================================================================
// h - the receive FIFO and its overrun policy
// =============================================================================

/// Send `n` frames with identifiers 1..n WITHOUT draining, at a rate slow
/// enough that the software is never the bottleneck.
bool fill_fifo(uint8_t n) {
    for (uint8_t i = 1; i <= n; ++i) {
        CanFrame f{};
        f.id = i;
        f.length = 1;
        f.data[0] = i;
        const auto mb = Bus::transmit(f);
        if (!mb) {
            return false;
        }
        if (!timed_wait([&] { return Bus::result(*mb).completed; }, 50u)) {
            return false;
        }
        (void)Bus::clear_result(*mb);
    }
    return true;
}

void th_can_fifo() {
    // The lock CLEAR: the newest message overwrites the last one stored.
    bench.verdict("the node comes up with the receive FIFO unlocked",
                  can_up(1'000'000u, true, false, CanOptions{.fifo_locked = false}));
    bool ok = fill_fifo(4);
    print(serial, "  four frames into a three-deep FIFO: FMP ", Bus::pending(0), " FULL ",
          Bus::full(0) ? "1" : "0", " FOVR ", Bus::overrun(0) ? "1" : "0", crlf);
    bench.verdict("three are pending, FULL stands and FOVR records the fourth",
                  ok && Bus::pending(0) == 3u && Bus::full(0) && Bus::overrun(0));

    uint32_t got[3] = {};
    for (uint8_t i = 0; i < 3u; ++i) {
        const auto f = Bus::receive(0);
        got[i] = f ? f->id : 0u;
    }
    print(serial, "  unlocked, the FIFO held ", got[0], " ", got[1], " ", got[2], crlf);
    bench.verdict("with the lock clear the LAST STORED message is the one "
                  "overwritten - the newest survives",
                  got[0] == 1u && got[1] == 2u && got[2] == 4u);
    bench.verdict("the FIFO is empty once its three messages are released",
                  Bus::pending(0) == 0u);

    Bus::clear_full(0);
    Bus::clear_overrun(0);
    bench.verdict("FULL and FOVR are rc_w1 and clear", !Bus::full(0) && !Bus::overrun(0));

    // The lock SET: the newest is discarded and the three oldest survive.
    bench.verdict("the node comes up with the receive FIFO locked",
                  can_up(1'000'000u, true, false, CanOptions{.fifo_locked = true}));
    ok = fill_fifo(4);
    for (uint8_t i = 0; i < 3u; ++i) {
        const auto f = Bus::receive(0);
        got[i] = f ? f->id : 0u;
    }
    print(serial, "  locked, the FIFO held ", got[0], " ", got[1], " ", got[2], " (FOVR ",
          Bus::overrun(0) ? "1" : "0", ")", crlf);
    bench.verdict("with the lock set the NEWEST message is discarded and the "
                  "three oldest survive",
                  ok && got[0] == 1u && got[1] == 2u && got[2] == 3u);

    // Releasing an empty FIFO does nothing (30.9.2), and reading one gives
    // nothing.
    bench.verdict("releasing an empty FIFO is harmless and reading it gives "
                  "nothing",
                  Bus::release(0) && !Bus::receive(0).has_value());
    Bus::master_reset();
}

// =============================================================================
// i - four bit rates, measured
// =============================================================================
void ti_can_rates() {
    static constexpr uint32_t rates[4] = {1'000'000u, 500'000u, 250'000u, 125'000u};
    uint32_t measured_us[4] = {};
    bool all_ok = true;
    for (uint8_t i = 0; i < 4u; ++i) {
        if (!can_up(rates[i], true, false)) {
            all_ok = false;
            break;
        }
        CanFrame f{};
        f.id = 0x123;
        f.length = 8;
        for (uint8_t k = 0; k < 8u; ++k) {
            f.data[k] = static_cast<uint8_t>(0x5Au ^ k);   // some stuffing, not the worst case
        }
        const auto mb = Bus::transmit(f);
        if (!mb) {
            all_ok = false;
            break;
        }
        const auto cycles = timed_wait([&] { return Bus::result(*mb).completed; }, 100u);
        if (!cycles || !Bus::result(*mb).ok) {
            all_ok = false;
            break;
        }
        (void)Bus::clear_result(*mb);
        (void)Bus::receive(0);
        measured_us[i] = *cycles / cycles_per_us;
        const uint32_t nominal_us = (can_frame_bits(8, false) * 1'000'000u) / rates[i];
        print(serial, "  ", rates[i] / 1000u, " kbit/s: ", measured_us[i], " us for a frame of ",
              can_frame_bits(8, false), " bits (", nominal_us, " us unstuffed, ",
              nominal_us == 0u ? 0u : (measured_us[i] * 100u) / nominal_us, "%)", crlf);
        if (measured_us[i] < nominal_us || measured_us[i] > nominal_us * 3u / 2u) {
            all_ok = false;
        }
    }
    bench.verdict("a frame takes at least its unstuffed bit count at every "
                  "rate, and no more than half as much again (bit stuffing "
                  "adds at most one bit in five)",
                  all_ok);

    // The ratio between the slowest and the fastest is the ratio of the
    // bit times, whatever the stuffing did - a rate-independent verdict.
    if (measured_us[0] != 0u) {
        const uint32_t ratio_tenths = (measured_us[3] * 10u) / measured_us[0];
        print(serial, "  125 kbit/s takes ", ratio_tenths / 10u, ".", ratio_tenths % 10u,
              " times as long as 1 Mbit/s (8 due)", crlf);
        bench.verdict("the frame time scales with the bit time, eightfold from "
                      "1 Mbit/s to 125 kbit/s",
                      ratio_tenths >= 75u && ratio_tenths <= 85u);
    }
    Bus::master_reset();
}

// =============================================================================
// j - the recessive bus: silent mode, and nobody to acknowledge
// =============================================================================
//
// The receive pad pulled up is a bus with no other node on it, permanently
// recessive. CAN1_TX stays unclaimed, so the chip drives nothing at all:
// the transmitter's output goes nowhere and its own monitor sees the
// recessive pad, which is exactly the condition of a node talking to
// silence.
void tj_can_errors() {
    Bus::clock(true);
    Bus::claim_rx_pad<can_rx_pad>(PinPull::up);

    // SILENT ALONE: the node listens and cannot start a transmission
    // (30.5.1). The request stays in its mailbox until it is aborted.
    Bus::filter_clock(true);
    Bus::master_reset();
    bool up = Bus::init_mode(true);
    up = Bus::options(CanOptions{.no_retransmit = true}) && up;
    up = Bus::timing(can_timing_for(can_pclk, 125'000u), false, true) && up;
    up = Bus::start() && up;
    print(serial, "  silent mode, the pad pulled up: RX reads ", Bus::rx_level() ? "1" : "0",
          ", normal mode ", Bus::in_normal() ? "reached" : "NOT reached", crlf);
    bench.verdict("with the receive pad recessive the node synchronizes and "
                  "reaches normal mode", up && Bus::in_normal());
    bench.verdict("the receive line reads recessive", Bus::rx_level());

    CanFrame f{};
    f.id = 0x123;
    f.length = 1;
    f.data[0] = 0x5A;
    const auto mb = Bus::transmit(f);
    const bool stuck = mb && !timed_wait([&] { return Bus::result(*mb).completed; }, 20u);
    print(serial, "  a transmit request in silent mode after 20 ms: ",
          mb && Bus::mailbox_empty(*mb) ? "gone" : "still pending", crlf);
    bench.verdict("SILENT MODE CANNOT START A TRANSMISSION: the request stays "
                  "in its mailbox",
                  stuck && mb && !Bus::mailbox_empty(*mb));
    if (mb) {
        (void)Bus::abort(*mb);
        const bool emptied = timed_wait([&] { return Bus::mailbox_empty(*mb); }, 20u).has_value();
        bench.verdict("and an abort empties it", emptied);
        (void)Bus::clear_result(*mb);
    }
    bench.verdict("no error was counted: a node that never transmits never "
                  "fails to be acknowledged",
                  Bus::tec() == 0u);

    // NORMAL MODE AGAINST A RECESSIVE BUS. Now the node really transmits,
    // and nothing answers: every attempt fails, and with NART set each
    // request costs exactly one attempt.
    Bus::master_reset();
    up = Bus::init_mode(true);
    up = Bus::options(CanOptions{.auto_bus_off = false, .no_retransmit = true}) && up;
    up = Bus::timing(can_timing_for(can_pclk, 1'000'000u), false, false) && up;
    up = Bus::start() && up;
    bench.verdict("the node reaches normal mode with no loopback and no peer", up);

    uint32_t attempts = 0;
    uint8_t tec_first = 0;
    CanError lec = CanError::none;
    while (attempts < 64u && !Bus::bus_off()) {
        const auto m = Bus::transmit(f);
        if (!m) {
            break;
        }
        if (!timed_wait([&] { return Bus::result(*m).completed; }, 20u)) {
            (void)Bus::abort(*m);
            break;
        }
        const CanTxResult r = Bus::result(*m);
        (void)Bus::clear_result(*m);
        ++attempts;
        if (attempts == 1u) {
            tec_first = Bus::tec();
            lec = Bus::last_error();
        }
        if (r.ok) {
            break;   // somebody answered: this is not the bus we thought
        }
    }
    print(serial, "  after one failed attempt TEC is ", tec_first, ", last error code ",
          static_cast<uint8_t>(lec), " (5 = bit dominant error: the transmitter sent a "
          "dominant bit and monitored the pad's recessive one, so the failure comes "
          "before the acknowledge slot); bus-off after ", attempts,
          " attempts, TEC reads ", Bus::tec(),
          " - the register carries the low byte of a nine-bit counter, and what the "
          "error frames themselves cost is not the chapter's to say", crlf);
    bench.verdict("an unacknowledged attempt costs the transmit error counter a "
                  "MULTIPLE OF EIGHT - one for the failed frame, and one more "
                  "for each error flag the node cannot get out either",
                  tec_first >= 8u && (tec_first % 8u) == 0u);
    bench.verdict("the error warning and error passive flags are passed on the "
                  "way",
                  Bus::error_warning() && Bus::error_passive());
    bench.verdict("BUS-OFF is reached, in fewer than the 32 attempts eight at a "
                  "time would need (an error frame nobody sees costs the counter "
                  "too)",
                  Bus::bus_off() && attempts <= 32u);

    // With ABOM clear the node stays off the bus until software asks.
    (void)timed_wait([] { return false; }, 30u);
    print(serial, "  30 ms later, with ABOM clear, BOFF is ", Bus::bus_off() ? "still set" : "gone",
          crlf);
    bench.verdict("with ABOM clear the node stays bus-off with no software",
                  Bus::bus_off());

    // ABOM: the hardware leaves bus-off by itself once it has counted
    // 128 x 11 recessive bits - 1408 bit times, 1.4 ms at 1 Mbit/s.
    (void)Bus::init_mode(true);
    (void)Bus::options(CanOptions{.auto_bus_off = true, .no_retransmit = true});
    (void)Bus::start();
    const auto recovered = timed_wait([] { return !Bus::bus_off(); }, 200u);
    print(serial, "  with ABOM set the recovery took ",
          recovered ? *recovered / cycles_per_us : 0u, " us (1408 bit times is 1408 us at "
          "1 Mbit/s)", crlf);
    bench.verdict("ABOM brings the node back from bus-off with no software",
                  recovered.has_value());
    print(serial, "  after the recovery TEC ", Bus::tec(), " REC ", Bus::rec(), crlf);
    bench.verdict("and the error counters are back under the warning limit",
                  !Bus::error_passive());

    // The one writable field of CAN_ESR.
    Bus::mark_error();
    bench.verdict("the last-error field takes the software code that says "
                  "'nothing since'",
                  Bus::last_error() == CanError::no_change);

    Bus::master_reset();
}

// =============================================================================
// k - the second instance and the block they share
// =============================================================================
#if defined(CAN2_BASE)

using Slave = Can<2>;

// CAN2's own receive pad, for the same reason CAN1 needs one: a node
// leaves initialization mode only after eleven recessive bits on it. PB5
// is CAN2_RX on AF9 (DS10693 table 11) and free on this board.
constexpr PinSel can2_rx_pad{'B', 5, PinFunction::af9};

void tk_can2() {
    Slave::clock(true);
    Slave::filter_clock(true);   // CAN1's gate: the filters are its registers
    Slave::claim_rx_pad<can2_rx_pad>(PinPull::up);
    Bus::master_reset();
    Slave::master_reset();
    // Whatever an earlier letter left in the block: this letter is about
    // the two RANGES, so both start empty.
    (void)Slave::filter_init(true);
    (void)Bus::filters_off();
    (void)Slave::filters_off();
    (void)Slave::filter_init(false);

    print(serial, "  CAN2SB reads ", Slave::start_bank(), " out of reset; CAN1 owns banks 0..",
          Bus::bank_limit() - 1u, ", CAN2 banks ", Slave::first_bank(), "..",
          Slave::bank_limit() - 1u, crlf);
    bench.verdict("the two instances share one filter block, and CAN1 is its "
                  "master",
                  Slave::filter_master == 1u && Bus::filter_master == 1u);
    bench.verdict("the split reads back where it is written",
                  Slave::start_bank(14) && Slave::start_bank() == 14u);

    // The split is enforced by the driver, because the silicon does not:
    // each instance may only configure the banks on its own side of it.
    bench.verdict("CAN1 is refused a bank above the split",
                  !Bus::filter(can_filter_accept_all(14, 0)));
    bench.verdict("CAN2 is refused a bank below it",
                  !Slave::filter(can_filter_accept_all(13, 0)));
    bench.verdict("and each is allowed its own side",
                  Bus::bank_ok(13) && Slave::bank_ok(14) && !Slave::bank_ok(28));

    // CAN2 in loopback, filtering through a bank of CAN1's block.
    bool up = Slave::init_mode(true);
    up = Slave::timing(can_timing_for(can_pclk, 1'000'000u), true, false) && up;
    (void)Slave::filter_init(true);
    (void)Slave::filters_off();
    (void)Slave::filter(can_filter_accept_all(14, 0));
    (void)Slave::filter_init(false);
    up = Slave::start() && up;
    bench.verdict("CAN2 reaches normal mode in loopback", up);

    CanFrame out{};
    out.id = 0x2C2;
    out.length = 3;
    out.data[0] = 0xC2;
    out.data[1] = 0x55;
    out.data[2] = 0xAA;
    const auto mb = Slave::transmit(out);
    bool ok = mb && timed_wait([&] { return Slave::result(*mb).completed; }, 20u).has_value();
    if (mb) {
        ok = Slave::result(*mb).ok && ok;
        (void)Slave::clear_result(*mb);
    }
    ok = ok && timed_wait([] { return Slave::pending(0) != 0u; }, 20u).has_value();
    const auto in = Slave::receive(0);
    print(serial, "  CAN2 round trip: ", in ? "received id " : "nothing, id ",
          hex(in ? in->id : 0u), " FMI ", in ? in->filter_index : 0u, crlf);
    bench.verdict("a frame round-trips on CAN2 through a bank of the shared "
                  "block",
                  ok && in && in->id == out.id && in->length == out.length &&
                      in->data[2] == out.data[2]);
    print(serial, "  the filter match index CAN2 reports for the FIRST bank of "
                  "its own range is ", in ? in->filter_index : 0u, crlf);

    // CAN1's own filters are untouched by CAN2's: the two ranges do not
    // overlap.
    bench.verdict("CAN2's bank is active and CAN1's range is still empty",
                  Slave::filter_active(14) && !Bus::filter_active(0));

    Slave::master_reset();
    Slave::clock(false);
    Bus::master_reset();
}

#endif // CAN2_BASE
#endif // CAN1_BASE && STM32F446xx

void banner() {
    print(serial, crlf,
          "test_stm32f4_misc - the CRC unit, the random number generator and bxCAN", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the CRC calculation unit against its own polynomial", ta_crc);
#if defined(RNG_BASE)
    bench.letter('b', "the random number generator", tb_rng);
#endif
#if defined(CAN1_BASE) && defined(STM32F446xx)
    bench.letter('c', "the CAN block and its three modes", tc_can_modes);
    bench.letter('d', "the bit timing, searched and read back", td_can_timing);
    bench.letter('e', "a frame round-trips in loopback", te_can_loopback);
    bench.letter('f', "the acceptance filters in their four shapes", tf_can_filters);
    bench.letter('g', "the transmit scheduler's two orders", tg_can_priority);
    bench.letter('h', "the receive FIFO and its overrun policy", th_can_fifo);
    bench.letter('i', "four bit rates measured against the arithmetic", ti_can_rates);
    bench.letter('j', "silent mode and the error counters on a recessive bus", tj_can_errors);
#if defined(CAN2_BASE)
    bench.letter('k', "CAN2 and the filter block the pair shares", tk_can2);
#endif
#endif

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
