// test_v203_dma - the reference bench suite for the CH32V203's DMA
// controller: ch32v203/dma.hpp over RM ch. 11, and the two engine slots
// ch32v203/usart.hpp grew for it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT IT MEASURES WITH. Nothing outside the chip. A memory-to-memory
// channel needs no request at all; a channel that does need one takes
// it from a TIMER, whose update reaches the controller over an internal
// line and no pad; the core's own STK counter is the ruler every number
// is weighed against; and the SOURCE of every copy is a four-kilobyte
// pattern in the image itself, which is a memory 11.1 lists as a legal
// source and which costs no RAM. ONE OPTIONAL JUMPER, PA2 to PA3 -
// USART2's transmitter to its own receiver - is what letter g needs: it
// says whether the strap is there (a plain output on one pad read on
// the other, both levels) and reports "no jumper" and passes when it is
// not - the transmit half of that letter needs no listener and runs
// either way.
//
// THE PADS. Only PA2 and PA3, and only in letter g. NEVER TOUCHED:
// PA9/PA10 (the console), PA13/PA14 (the debug port), PA11/PA12 (the
// USB pads), PC14/PC15 and PD0/PD1 (the crystals), PA0 (the KEY) - and
// PB2, the LED, toggled per command as every suite of this target does.
//
// What is exercised, letter by letter:
//   a  MEMORY TO MEMORY: the block's gate, the three widths with and
//      without the increments, four kilobytes moved at each of them and
//      verified byte by byte, the half and complete flags counted, the
//      cost per item against the core's counter, a store into a RUNNING
//      channel refused and a misaligned address refused
//   b  CIRCULAR MODE: a run of compares poured into a timer by its own
//      update request, the buffer reloaded lap after lap, the half and
//      complete flags alternating and remaining() walking down and back
//   c  THE SLEEP: four kilobytes started, the platform's idle() for one
//      tick and for ten, the count read on return - the number that
//      says the bus matrix served the core alone - and the same block
//      finished awake and verified
//   d  THE TRANSFER ERROR: five addresses no peripheral answers at,
//      read by a channel with its error interrupt armed; what the flag
//      did, and whether the silicon dropped EN by itself
//   e  THE REQUEST PATH WITH NO WIRE: a timer's update copying another
//      timer's counter into a buffer once per period (the values a
//      staircase whose step is the period), and the BURST ENGINE
//      writing four compares from memory through one address
//   f  THE PRIORITIES: two channels contending for the bus, the order
//      measured by their own completion flags - once with the software
//      priority against the channel number, once with the channel
//      number alone
//   g  THE USART ENGINES: a run drained from the ring by the transmit
//      engine, which needs no listener, and a receive block that
//      outlives its sender and is abandoned - both with the board bare;
//      then, WITH THE JUMPER, the same run coming back through the
//      receive engine and published by harvest()
//   h  THE VECTORS: a block on each of the eight channels, each one's
//      flag reaching its own handler body and no other
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/dma.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/tim.hpp"
#include "ch32v203/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32v203Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

TestBench<Serial> bench;

/// The transport letter g drives: USART2 with both engines, each on the
/// channel table 11-5 gives that direction (7 out, 6 in). The console
/// above has none, which is what makes an empty slot cost nothing.
using Loop = Uart<2, P, 128, 128, UartFormat{},
                  DmaTxEngine<DmaRequestOf<DmaRequest::usart2_tx>::channel>,
                  DmaRxEngine<DmaRequestOf<DmaRequest::usart2_rx>::channel>>;
using LoopRx = DmaRxEngine<DmaRequestOf<DmaRequest::usart2_rx>::channel>;

using Pacer = Tim<2>;    ///< the request source: its update drives channel 2
using Ruler = Tim<3>;    ///< free-running, the datum the staircase samples

constexpr uint32_t tim_hz = Pacer::clock_hz(clock);
constexpr uint32_t ticks_per_us = tim_hz / 1'000'000u;
static_assert(tim_hz == SysClock::hz);

// ---------------------------------------------------------------------------
// The buffers
// ---------------------------------------------------------------------------

/// FOUR KILOBYTES OF SOURCE IN THE IMAGE. 11.1 lists flash among the
/// memories a channel may read, so the pattern lives there: the chip
/// carries one copy, the RAM carries none, and a verdict compares what
/// the controller wrote against what the core reads at the same place.
struct Pattern {
    uint32_t word[1024];
};

constexpr Pattern make_pattern() {
    Pattern p{};
    uint32_t x = 0x1234'5678UL;
    for (uint32_t i = 0; i < 1024u; ++i) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        p.word[i] = x;
    }
    return p;
}

constexpr Pattern source = make_pattern();
constexpr uint16_t source_bytes = 4096;

alignas(4) uint8_t destination[source_bytes];
/// The half-word landing ground of letters b and e: its own storage, so
/// no run of this suite reads one type through a pointer to another.
alignas(4) uint16_t work[16];
constexpr uint16_t sample_count = 16;

const volatile void* source_address() { return static_cast<const volatile void*>(source.word); }
const uint8_t* source_first_bytes() { return reinterpret_cast<const uint8_t*>(&source.word[0]); }

// ---------------------------------------------------------------------------
// The ruler
// ---------------------------------------------------------------------------

/**
 * THE RULER OF THIS SUITE: the core's own STK counter, read as a
 * stopwatch. It counts up to its reload - one tick period, a
 * millisecond here - and starts again, so a span is accumulated poll by
 * poll with one period folded in across each wrap.
 */
class Stopwatch {
public:
    Stopwatch() { start(); }

    void start() {
        last_ = stk()->CNTL;
        acc_ = 0;
    }

    uint32_t cycles() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_;
    }

    uint32_t us() { return cycles() / ticks_per_us; }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

void wait_us(uint32_t us) {
    Stopwatch w;
    while (w.us() < us) {
    }
}

// ---------------------------------------------------------------------------
// The handlers' counters, and which face channels 6 and 7 wear
// ---------------------------------------------------------------------------

volatile uint16_t channel_calls[8] = {};
volatile uint32_t channel_mask[8] = {};
/// Channels 6 and 7 are USART2's two, so their vectors serve the
/// transport in letter g and the bare channel everywhere else.
volatile bool uart_mode = false;

void note(uint8_t index, uint32_t flags) {
    if (flags != 0u) {
        channel_calls[index] = static_cast<uint16_t>(channel_calls[index] + 1u);
        channel_mask[index] = channel_mask[index] | flags;
    }
}

// ---------------------------------------------------------------------------
// Transfers, named once
// ---------------------------------------------------------------------------

DmaTransfer copy(const volatile void* from, volatile void* to, uint16_t items, DmaWidth width,
                 bool increment_source = true, DmaPriority priority = DmaPriority::low) {
    return DmaTransfer{
        .peripheral = const_cast<volatile void*>(from),
        .memory = to,
        .count = items,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = true,
                   .peripheral_increment = increment_source,
                   .memory_increment = true,
                   .peripheral_width = width,
                   .memory_width = width,
                   .priority = priority},
    };
}

/// Poll a channel's completion, bounded: a wedge must end the letter,
/// not the session.
bool wait_complete(uint32_t shift, uint32_t spins = 4'000'000UL) {
    while (spins-- != 0u) {
        if (((Dma::flags() >> shift) & DmaFlag::complete) != 0u) {
            return true;
        }
    }
    return false;
}

bool verify(uint16_t bytes) {
    const uint8_t* want = source_first_bytes();
    for (uint16_t i = 0; i < bytes; ++i) {
        if (destination[i] != want[i]) {
            return false;
        }
    }
    return true;
}

void scrub(uint16_t bytes) {
    for (uint16_t i = 0; i < bytes; ++i) {
        destination[i] = 0;
    }
}

/// Every channel, every line and both timers back to reset, at the top
/// of each letter, so no letter inherits another's state.
void all_off() {
    Dma::open();
    Dma::stop_all();
    for (uint8_t ch = 1; ch <= dma_channel_count; ++ch) {
        Pfic::disable(dma_channel_irq(ch));
        Pfic::clear_pending(dma_channel_irq(ch));
    }
    Pacer::release();
    Ruler::release();
    uart_mode = false;
    for (uint8_t i = 0; i < 8u; ++i) {
        channel_calls[i] = 0;
        channel_mask[i] = 0;
    }
}

// ---------------------------------------------------------------------------
// The jumper
// ---------------------------------------------------------------------------

bool jumper = false;
bool jumper_known = false;

/// Is PA2 strapped to PA3? Drive one as a plain output and read the
/// other, both ways round: a strap shows both levels, an open pad holds
/// whatever its pull says.
bool measure_jumper() {
    using Src = Pin<'A', 2>;
    using Dst = Pin<'A', 3>;
    Src::output(false);
    Dst::input(PinPull::up);
    wait_us(20);
    const bool low_seen = !Dst::read();
    Src::set();
    wait_us(20);
    const bool high_seen = Dst::read();
    Dst::input(PinPull::down);
    Src::clear();
    wait_us(20);
    const bool low_again = !Dst::read();
    Src::release();
    Dst::release();
    return low_seen && high_seen && low_again;
}

void need_jumper() {
    if (!jumper_known) {
        jumper = measure_jumper();
        jumper_known = true;
    }
}

// ===========================================================================
// a - memory to memory: the gate, the widths, the flags, the cost
// ===========================================================================
void ta_memory() {
    all_off();
    using Ch = DmaChannel<1>;

    // The gate. It is closed at reset and every channel verb opens it,
    // so a channel programmed with the gate shut would write nothing.
    Rcc::disable(Bus::hb, rcc_hb_dma1);
    const bool closed = !Dma::opened();
    Ch::stop();
    const bool opened = Dma::opened();
    bench.verdict("the block's gate is closed until a channel verb opens it (RCC_HBPCENR's "
                  "reset value opens the SRAM alone)",
                  closed && opened);

    // The three widths, four kilobytes each, verified byte by byte.
    struct Run {
        DmaWidth width;
        uint16_t items;
        const char* name;
    };
    const Run runs[3] = {{DmaWidth::byte, 4096, "byte"},
                         {DmaWidth::half, 2048, "half"},
                         {DmaWidth::word, 1024, "word"}};
    bool all_exact = true;
    for (const Run& r : runs) {
        scrub(source_bytes);
        Ch::stop();
        Stopwatch w;
        const bool started = Ch::load(copy(source_address(), destination, r.items, r.width));
        const bool done = started && wait_complete(Ch::flag_shift);
        const uint32_t cycles = w.cycles();
        const bool exact = done && verify(source_bytes) && Ch::remaining() == 0u;
        all_exact = all_exact && exact;
        print(serial, "  4096 B as ", r.name, "s: ", r.items, " items in ", cycles,
              " cycles, ", (cycles * 100u) / r.items, "/100 cycles an item", crlf);
    }
    bench.verdict("four kilobytes move at all three widths and arrive byte for byte, the "
                  "count down to zero",
                  all_exact);

    // The flags: the half one rises at the midpoint and stands beside
    // the complete one, and the write-one clear takes every bit down.
    scrub(source_bytes);
    Ch::stop();
    (void)Ch::load(copy(source_address(), destination, 4096, DmaWidth::byte));
    uint32_t half_at = 0;
    uint32_t spins = 4'000'000UL;
    while (spins-- != 0u) {
        const uint32_t f = Ch::flags();
        if (half_at == 0u && (f & DmaFlag::half) != 0u) {
            half_at = 4096u - Ch::remaining();
        }
        if ((f & DmaFlag::complete) != 0u) {
            break;
        }
    }
    const uint32_t ending = Ch::flags();
    Ch::clear(DmaFlag::all);
    const uint32_t cleared = Ch::flags();
    print(serial, "  the half flag rose with ", half_at, " of 4096 moved; INTFR ", hex(ending),
          " -> ", hex(cleared), crlf);
    bench.verdict("the half and complete flags both stand at the end with the global bit "
                  "beside them, and writing one to INTFCR takes all four down",
                  (ending & (DmaFlag::half | DmaFlag::complete | DmaFlag::global)) ==
                          (DmaFlag::half | DmaFlag::complete | DmaFlag::global) &&
                      cleared == 0u);
    bench.verdict("and the half flag rises where 11.2.1 says it does - when fewer than half "
                  "the items are left",
                  half_at >= 2048u && half_at < 2500u);

    // A source that does not increment: one word over a whole run.
    Ch::stop();
    scrub(64);
    (void)Ch::load(copy(source_address(), destination, 16, DmaWidth::word, false));
    const bool fixed_done = wait_complete(Ch::flag_shift);
    bool fixed_ok = fixed_done;
    for (uint8_t i = 0; i < 64u && fixed_ok; ++i) {
        fixed_ok = destination[i] == source_first_bytes()[i & 3u];
    }
    bench.verdict("with the source increment clear one word fills a whole run - the fixed "
                  "cell a full-duplex bus clocks a read from",
                  fixed_ok);

    // A store into a RUNNING channel is refused by this driver, because
    // the silicon would drop it in silence (11.2.1's note).
    Ch::stop();
    (void)Ch::load(copy(source_address(), destination, 4096, DmaWidth::byte));
    const uint32_t cfgr_before = Ch::regs().CFGR;
    const bool configure_refused = !Ch::configure(DmaChannelConfig{});
    const bool count_refused = !Ch::set_count(8);
    const bool prepare_refused =
        !Ch::prepare(copy(source_address(), destination, 8, DmaWidth::byte));
    const bool address_refused = !Ch::set_memory(destination);
    const uint32_t cfgr_after = Ch::regs().CFGR;
    (void)wait_complete(Ch::flag_shift);
    Ch::stop();
    bench.verdict("every configuring verb refuses while EN is set - the configuration, the "
                  "count, a whole transfer and an address - and the register is untouched",
                  configure_refused && count_refused && prepare_refused && address_refused &&
                      cfgr_before == cfgr_after);

    // And the address rule the silicon answers with silence: 11.3.5 and
    // 11.3.6 IGNORE the low bits of a wide access, so this driver
    // refuses the transfer instead of moving the wrong bytes.
    Ch::stop();
    const bool odd_refused =
        !Ch::load(copy(source_address(), destination + 1, 4, DmaWidth::half));
    const bool even_taken =
        Ch::load(copy(source_address(), destination + 2, 4, DmaWidth::half));
    (void)wait_complete(Ch::flag_shift);
    Ch::stop();
    bench.verdict("a half-word transfer is refused at an odd address and taken at an even "
                  "one, because the module would align it in silence",
                  odd_refused && even_taken);

    // The one configuration the chapter forbids.
    const bool circular_m2m_refused =
        !Ch::configure(DmaChannelConfig{.circular = true, .memory_to_memory = true});
    bench.verdict("circular mode is refused for a memory-to-memory channel, which is the one "
                  "combination 11.2.1 forbids",
                  circular_m2m_refused);
    all_off();
}

// ===========================================================================
// b - circular mode, paced by a timer's own update request
// ===========================================================================
void tb_circular() {
    all_off();
    using Ch = DmaChannel<DmaRequestOf<DmaRequest::tim2_up>::channel>;
    static_assert(Ch::number == 2, "table 11-5: TIM2's update is channel 2's request");

    // A run of eight compares, poured one per update into the timer's
    // own first channel. Circular, so the eighth update starts the run
    // again from its beginning.
    for (uint16_t i = 0; i < 8u; ++i) {
        work[i] = static_cast<uint16_t>(100u + 10u * i);
    }

    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 199});   // an update every 200 us
    (void)Pacer::output_channel(
        0, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = 1, .preload = false});

    Ch::stop();
    const bool armed = Ch::prepare(DmaTransfer{
        .peripheral = Pacer::chcvr_address(0),
        .memory = work,
        .count = 8,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .circular = true,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    const bool circular_read_back = Ch::configuration().circular;
    Ch::enable(true);

    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);

    // Four laps: the half and complete flags alternate, one of each a
    // lap, and the count reloads without a hand on it.
    uint16_t half_seen = 0;
    uint16_t full_seen = 0;
    uint16_t walked_down = 0;
    uint16_t last_remaining = Ch::remaining();
    Stopwatch w;
    while (full_seen < 4u && w.us() < 20'000UL) {
        const uint32_t f = Ch::flags();
        if ((f & DmaFlag::half) != 0u) {
            ++half_seen;
            Ch::clear(DmaFlag::half | DmaFlag::global);
        }
        if ((f & DmaFlag::complete) != 0u) {
            ++full_seen;
            Ch::clear(DmaFlag::complete | DmaFlag::global);
        }
        const uint16_t now = Ch::remaining();
        if (now < last_remaining) {
            ++walked_down;
        }
        last_remaining = now;
    }
    const uint32_t laps_us = w.us();
    const bool still_enabled = Ch::enabled();
    const uint32_t landed = Pacer::compare(0);

    Pacer::enable(false);
    print(serial, "  four laps of eight in ", laps_us, " us: half x", half_seen, ", complete x",
          full_seen, ", ", walked_down, " steps down, CH1CVR=", landed, crlf);
    bench.verdict("a circular channel reloads its count and its addresses by itself: the half "
                  "and complete flags come one per lap and the channel is still enabled",
                  armed && circular_read_back && full_seen >= 4u && half_seen >= 4u &&
                      still_enabled);
    bench.verdict("and the count WALKS: remaining() was seen stepping down through the run "
                  "and the value that landed in the timer is one the run carries",
                  walked_down >= 8u && landed >= 100u && landed <= 170u);
    bench.verdict("the laps are paced by the REQUEST and not by the bus: thirty-two updates "
                  "of 200 us are 6400 us, which is what the core's counter says to a tenth",
                  laps_us > 5800UL && laps_us < 7600UL);
    all_off();
}

// ===========================================================================
// c - the sleep: in Sleep no bus master but the core gets a cycle
// ===========================================================================
void tc_sleep() {
    all_off();
    using Ch = DmaChannel<1>;

    // The reference: the whole block, awake.
    scrub(source_bytes);
    Ch::stop();
    Stopwatch w;
    (void)Ch::load(copy(source_address(), destination, 4096, DmaWidth::byte));
    const bool awake_done = wait_complete(Ch::flag_shift);
    const uint32_t awake_us = w.us();
    Ch::stop();

    // The same block, with the core asleep. The console must be quiet
    // first: a transmit interrupt would end the sleep before the tick.
    scrub(source_bytes);
    while (!Serial::tx_idle()) {
    }
    (void)Ch::load(copy(source_address(), destination, 4096, DmaWidth::byte));
    disable_interrupts();
    P::idle();
    const uint16_t left_one = Ch::remaining();
    for (uint8_t i = 0; i < 9u; ++i) {
        disable_interrupts();
        P::idle();
    }
    const uint16_t left_ten = Ch::remaining();
    const uint16_t moved_one = static_cast<uint16_t>(4096u - left_one);
    const uint16_t moved_ten = static_cast<uint16_t>(4096u - left_ten);

    // Awake again: the same channel finishes what it started.
    const bool finished = wait_complete(Ch::flag_shift);
    const bool exact = finished && verify(source_bytes);
    Ch::stop();

    print(serial, "  4096 B awake: ", awake_us, " us; asleep: ", moved_one,
          " B across one idle(), ", moved_ten, " B across ten (", moved_ten / 10u,
          " B a wake)", crlf);
    bench.verdict("the same four kilobytes that take well under a millisecond awake do not "
                  "finish in ten milliseconds of sleep: in Sleep no bus master but the core "
                  "gets a cycle",
                  awake_done && awake_us < 1000UL && moved_ten < 4096u);
    bench.verdict("what moves across a sleep is the handful of items already in flight plus "
                  "what the wake itself runs - tens of bytes a wake, not thousands",
                  moved_one < 256u && moved_ten < 1024u);
    bench.verdict("and the block finishes the moment the core is awake again, byte for byte",
                  exact);
    all_off();
}

// ===========================================================================
// d - the transfer error: a read of an address nothing answers at
// ===========================================================================
void td_error() {
    all_off();
    using Ch = DmaChannel<3>;

    // 11.2.1: "Reading and writing a reserved address area results in a
    // DMA transmission error", the hardware clears EN and the channel
    // stops. Five addresses outside every window figure 1-11 draws.
    const uint32_t holes[5] = {0x3000'0000UL, 0x4000'8000UL, 0x4002'4000UL, 0x6000'0000UL,
                               0xA000'0000UL};
    uint8_t errors = 0;
    uint8_t completions = 0;
    uint8_t left_enabled = 0;
    for (const uint32_t hole : holes) {
        Ch::stop();
        Ch::arm(DmaFlag::error | DmaFlag::complete, true);
        scrub(16);
        (void)Ch::load(
            copy(reinterpret_cast<const volatile void*>(hole), destination, 4, DmaWidth::word));
        uint32_t spins = 200'000UL;
        uint32_t f = 0;
        while (spins-- != 0u) {
            f = Ch::flags();
            if ((f & (DmaFlag::error | DmaFlag::complete)) != 0u) {
                break;
            }
        }
        const bool enabled_after = Ch::enabled();
        if ((f & DmaFlag::error) != 0u) {
            ++errors;
            if (enabled_after) {
                ++left_enabled;
            }
        } else if ((f & DmaFlag::complete) != 0u) {
            ++completions;
        }
        print(serial, "  ", hex(hole), ": flags ", hex(f),
              enabled_after ? " EN set" : " EN clear", " first bytes ", hex(destination[0]),
              hex(destination[1]), hex(destination[2]), hex(destination[3]), crlf);
    }
    Ch::stop();
    print(serial, "  of five holes: ", errors, " raised a transfer error, ", completions,
          " completed as an ordinary block", crlf);
    bench.verdict("five addresses no window of the map answers at were each read by a "
                  "channel with its error interrupt armed, and every one of them ended",
                  static_cast<uint8_t>(errors + completions) == 5u);
    bench.verdict("and wherever a transfer error IS raised the hardware clears EN by itself "
                  "(11.3.3), so no channel was left enabled behind one",
                  left_enabled == 0u);
    all_off();
}

// ===========================================================================
// e - the request path with no wire: a staircase, and the burst engine
// ===========================================================================
void te_request() {
    all_off();
    using Ch = DmaChannel<DmaRequestOf<DmaRequest::tim2_up>::channel>;

    // A free-running ruler at one megahertz, and a pacer whose update
    // moves one sample of it into memory every 500 us.
    Ruler::init();
    (void)Ruler::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 0xFFFF});
    Ruler::enable(true);

    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 499});

    for (uint16_t i = 0; i < sample_count; ++i) {
        work[i] = 0;
    }
    Ch::stop();
    const bool armed = Ch::load(DmaTransfer{
        .peripheral = Ruler::cnt_address(),
        .memory = work,
        .count = sample_count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);
    const bool sampled = wait_complete(Ch::flag_shift);
    Pacer::enable(false);
    Ruler::enable(false);

    // The values are a staircase whose step is the pacer's period.
    uint16_t steps_right = 0;
    for (uint16_t i = 1; i < sample_count; ++i) {
        const uint16_t step = static_cast<uint16_t>(work[i] - work[i - 1u]);
        if (step >= 495u && step <= 505u) {
            ++steps_right;
        }
    }
    print(serial, "  sixteen samples of a 1 MHz counter 500 us apart: ", work[0], " ", work[1],
          " ", work[2], " ... ", work[sample_count - 1u], crlf);
    bench.verdict("a timer's update request moves one sample per period with no pad and no "
                  "core: sixteen samples of another timer's counter, each a period after the "
                  "last to within one per cent",
                  armed && sampled && steps_right == sample_count - 1u);

    // THE BURST ENGINE. One request, several registers: each access to
    // the timer's DMAADR lands on CTLR1 + DBA + an index the controller
    // walks, so four compares are written through ONE address.
    all_off();
    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 999});
    for (uint8_t ch = 0; ch < 4u; ++ch) {
        (void)Pacer::output_channel(
            ch, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = 0, .preload = false});
    }
    work[0] = 111;
    work[1] = 222;
    work[2] = 333;
    work[3] = 444;
    const bool window = Pacer::dma_burst(TimBurstBase::ch1cvr, 4);
    const bool window_read_back =
        Pacer::burst_base() == TimBurstBase::ch1cvr && Pacer::burst_length() == 4u;

    Ch::stop();
    const bool burst_armed = Ch::load(DmaTransfer{
        .peripheral = Pacer::dmaadr_address(),
        .memory = work,
        .count = 4,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);
    const bool burst_done = wait_complete(Ch::flag_shift);
    Pacer::enable(false);

    const uint32_t got[4] = {Pacer::compare(0), Pacer::compare(1), Pacer::compare(2),
                             Pacer::compare(3)};
    print(serial, "  the burst wrote CH1CVR..CH4CVR = ", got[0], " ", got[1], " ", got[2], " ",
          got[3], crlf);
    bench.verdict("the burst engine turns one request into a walk of four registers through "
                  "the single address TIMx_DMAADR - four compares written from memory, read "
                  "back where the waveform reads them",
                  window && window_read_back && burst_armed && burst_done && got[0] == 111u &&
                      got[1] == 222u && got[2] == 333u && got[3] == 444u);
    all_off();
}

// ===========================================================================
// f - the priorities: two channels contending for one bus
// ===========================================================================
void tf_priority() {
    all_off();
    using Low = DmaChannel<1>;
    using High = DmaChannel<2>;
    constexpr uint16_t words = 512;   // 2 KB each, the two halves of the buffer

    // The software priority against the channel number: the LOWER
    // channel starts first and asks for the LOWER priority, so only the
    // arbiter can make the other one finish first.
    Low::stop();
    High::stop();
    const bool a_ready = Low::prepare(
        copy(source_address(), destination, words, DmaWidth::word, true, DmaPriority::low));
    const bool b_ready = High::prepare(copy(source_address(), destination + 2048, words,
                                            DmaWidth::word, true, DmaPriority::very_high));
    Low::enable(true);
    High::enable(true);

    uint8_t first = 0;
    uint32_t spins = 4'000'000UL;
    while (first == 0u && spins-- != 0u) {
        const uint32_t f = Dma::flags();
        const bool low_done = ((f >> Low::flag_shift) & DmaFlag::complete) != 0u;
        const bool high_done = ((f >> High::flag_shift) & DmaFlag::complete) != 0u;
        if (low_done && high_done) {
            first = 3;
        } else if (high_done) {
            first = 2;
        } else if (low_done) {
            first = 1;
        }
    }
    (void)wait_complete(Low::flag_shift);
    print(serial, "  channel 1 (low) started first, channel 2 (very high) second: first to "
                  "complete was ",
          first == 3u ? "both in one read" : first == 2u ? "channel 2" : "channel 1", crlf);
    bench.verdict("the software priority outranks the channel number: with equal work, the "
                  "channel that started second and asked for the higher priority completed "
                  "first",
                  a_ready && b_ready && first == 2u);

    // The tie: equal priorities, and the HIGHER channel given the head
    // start. Then only the fixed hardware priority - the lower index -
    // can reverse it (11.2.1).
    all_off();
    Low::stop();
    High::stop();
    const bool c_ready = High::prepare(copy(source_address(), destination + 2048, words,
                                            DmaWidth::word, true, DmaPriority::medium));
    const bool d_ready = Low::prepare(copy(source_address(), destination, words, DmaWidth::word,
                                           true, DmaPriority::medium));
    High::enable(true);
    Low::enable(true);

    Stopwatch w;
    uint32_t low_cycles = 0;
    uint32_t high_cycles = 0;
    spins = 4'000'000UL;
    while ((low_cycles == 0u || high_cycles == 0u) && spins-- != 0u) {
        const uint32_t f = Dma::flags();
        if (low_cycles == 0u && ((f >> Low::flag_shift) & DmaFlag::complete) != 0u) {
            low_cycles = w.cycles() + 1u;
        }
        if (high_cycles == 0u && ((f >> High::flag_shift) & DmaFlag::complete) != 0u) {
            high_cycles = w.cycles() + 1u;
        }
    }
    print(serial, "  equal priorities, channel 2 started first: channel 1 done at ",
          low_cycles, " cycles, channel 2 at ", high_cycles, crlf);
    bench.verdict("with the priorities equal the lower channel number wins, head start and "
                  "all - the fixed hardware priority of 11.2.1",
                  c_ready && d_ready && low_cycles != 0u && high_cycles != 0u &&
                      low_cycles <= high_cycles);
    all_off();
}

// ===========================================================================
// g - the USART's two engines, on the jumper
// ===========================================================================
void tg_engines() {
    all_off();
    need_jumper();
    print(serial, "  the jumper PA2-PA3 is ", jumper ? "in place" : "ABSENT", crlf);

    uart_mode = true;
    const bool opened = Loop::init(clock, 115200);
    // The receive pad is pulled UP after init: an idle serial line is
    // high, so with no strap the receiver sees no frame instead of
    // whatever a floating pad invents. A transmitter driving the pad
    // overrides it.
    Pin<'A', 3>::input(PinPull::up);
    LoopRx::clear_faults();

    // THE TRANSMIT ENGINE NEEDS NO LISTENER. TXE raises its request
    // whether or not anything is on the other end of the pad, so the
    // half of this letter that drains a ring by blocks is measured
    // with the board bare.
    static const uint8_t message[] = "the channel is the request";
    constexpr uint16_t length = sizeof(message) - 1u;
    Loop::write(message, length);
    Stopwatch w;
    while (!Loop::tx_idle() && w.us() < 20'000UL) {
    }
    const uint32_t drained_us = w.us();
    const bool drained = Loop::tx_idle();
    // 26 bytes of ten bits at 115200 baud are some 2260 us, and the
    // run left the ring in ONE block: the engine's own completion is
    // what released it.
    print(serial, "  ", length, " bytes drained by the transmit engine in ", drained_us,
          " us, faults ", Loop::dma_faults(), crlf);
    bench.verdict("the transmit engine drains the ring by whole blocks with no listener at "
                  "all - the ring is empty and the shift register idle, no fault counted",
                  opened && drained && Loop::dma_faults() == 0u);

    // A RECEIVE THAT OUTLIVES ITS SENDER. The run is armed for the
    // ring's whole free span and the line goes quiet: the block never
    // completes, so abandon() is what hands the channel back.
    const uint16_t standing = LoopRx::capacity();
    const uint16_t arrived = LoopRx::harvest();
    const bool still_running = !LoopRx::idle();
    const bool abandoned = LoopRx::abandon();
    const bool idle_after = LoopRx::idle();
    print(serial, "  the standing receive run is ", standing, " long with ", arrived,
          " in it; faults after abandon(): ", LoopRx::faults(), crlf);
    bench.verdict("a receive block that outlives its sender stands open - harvest() reports "
                  "its progress and abandon() is what ends it, the channel handed back and "
                  "the fault counted",
                  still_running && abandoned && idle_after && LoopRx::faults() == 1u);

    if (!jumper) {
        bench.verdict("the round trip is skipped and says so: without the strap between "
                      "USART2's two pads a transmit engine has no receive engine to reach",
                      true);
        uart_mode = false;
        all_off();
        return;
    }

    // With the strap, the same run comes back. harvest() is a VERB:
    // nothing is published until it is asked - and the first call is
    // also what re-arms the run abandon() has just thrown away.
    (void)Loop::harvest();
    Loop::write(message, length);
    uint16_t got = 0;
    uint8_t seen[64] = {};
    Stopwatch r;
    while (got < length && r.us() < 50'000UL) {
        (void)Loop::harvest();
        uint8_t b = 0;
        while (got < length && Loop::read_byte(b)) {
            seen[got++] = b;
        }
    }
    bool same = got == length;
    for (uint16_t i = 0; i < got && same; ++i) {
        same = seen[i] == message[i];
    }
    print(serial, "  ", got, " of ", length, " bytes round the jumper, faults ",
          Loop::dma_faults(), ", overruns ", Loop::rx_overruns(), crlf);
    bench.verdict("a run poured out by the transmit engine comes back through the receive "
                  "one byte for byte, published by harvest() and by nothing else",
                  same && Loop::dma_faults() == 0u);

    uart_mode = false;
    all_off();
}

// ===========================================================================
// h - the vectors: one line per channel, each answering for its own
// ===========================================================================
template <uint8_t ch>
void one_vector(uint8_t& ran, uint8_t& clean) {
    using C = DmaChannel<ch>;
    C::stop();
    C::arm(DmaFlag::complete, true);
    Pfic::enable(C::irq());
    (void)C::load(copy(source_address(), destination, 16, DmaWidth::word));
    Stopwatch w;
    while (channel_calls[ch - 1u] == 0u && w.us() < 2000UL) {
    }
    if (channel_calls[ch - 1u] == 1u) {
        ++ran;
    }
    if (channel_mask[ch - 1u] == DmaFlag::complete) {
        ++clean;
    }
    C::stop();
}

void th_vectors() {
    all_off();

    // A small block on every channel in turn, with the completion armed
    // and the line enabled: each handler must run for its own channel
    // and see its own flag.
    uint8_t ran = 0;
    uint8_t clean = 0;
    one_vector<1>(ran, clean);
    one_vector<2>(ran, clean);
    one_vector<3>(ran, clean);
    one_vector<4>(ran, clean);
    one_vector<5>(ran, clean);
    one_vector<6>(ran, clean);
    one_vector<7>(ran, clean);
    one_vector<8>(ran, clean);

    print(serial, "  bodies that ran: ", ran,
          " of 8; bodies that saw only their own complete flag: ", clean, crlf);
    bench.verdict("all eight channels have a vector of their own - seven consecutive and the "
                  "eighth on this device class's own tail - and each body ran exactly once",
                  ran == 8u);
    bench.verdict("and each body saw the ARMED flag and nothing else: isr() returns what is "
                  "up of what was armed, clears exactly that, and decides nothing",
                  clean == 8u);
    all_off();
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf, "test_v203_dma - the DMA controller of RM ch. 11", crlf,
          "  one optional jumper: PA2 to PA3 (USART2's own two pads) is what letter g needs",
          crlf, "  the source of every copy is a 4 KB pattern in the image itself", crlf);
    bench.menu();
}

}  // namespace

// The eight vectors. Channels 6 and 7 are USART2's, so their bodies
// serve the transport while letter g runs and the bare channel the rest
// of the time.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() { note(0, brio::DmaChannel<1>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { note(1, brio::DmaChannel<2>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { note(2, brio::DmaChannel<3>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() { note(3, brio::DmaChannel<4>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() { note(4, brio::DmaChannel<5>::isr()); }

extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    if (uart_mode) {
        (void)Loop::dma_isr();
    } else {
        note(5, brio::DmaChannel<6>::isr());
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    if (uart_mode) {
        (void)Loop::dma_isr();
    } else {
        note(6, brio::DmaChannel<7>::isr());
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() { note(7, brio::DmaChannel<8>::isr()); }

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Loop::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "memory to memory: the gate, the widths, the flags, the cost", ta_memory);
    bench.letter('b', "circular mode, paced by a timer's update request", tb_circular);
    bench.letter('c', "the sleep: what a channel moves while the core is not awake", tc_sleep);
    bench.letter('d', "the transfer error: five addresses nothing answers at", td_error);
    bench.letter('e', "the request path with no wire: a staircase, and the burst engine",
                 te_request);
    bench.letter('f', "the priorities: two channels contending for one bus", tf_priority);
    bench.letter('g', "the USART's two engines, on the jumper", tg_engines);
    bench.letter('h', "the vectors: one line per channel", th_vectors);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
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
