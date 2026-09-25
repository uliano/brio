// test_vx03_dma - the reference bench suite for the DMA controllers of the
// CH32V203 and the CH32V303: ch32vx03/dma.hpp over RM ch. 11, and the two
// engine slots ch32vx03/usart.hpp grew for it. The CH32V203 has one
// controller of eight channels. The CH32V303 has two - DMA1 with seven, and
// DMA2 with eleven whose last four report in a flag register of their own -
// and letters i..n are that second controller's: registered on the part
// that has it, and compiled out of every other image.
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
// either way. On the CH32V303's evaluation board the same two pads are
// wired to UART4's instead (PA2 to PC11, PC10 to PA3): letter g finds no
// strap there and says so, and letter n takes those two wires, each one
// looked for the same way before it is used.
//
// THE PADS. PA2 and PA3 in letter g, and on the CH32V303 PA2, PA3, PC10
// and PC11 in letter n. NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14
// (the debug port), PA11/PA12 (the USB pads), PC14/PC15 and PD0/PD1 (the
// crystals), PA0 (the CH32V203 board's KEY) - and PB2, the CH32V203
// board's LED, toggled per command as every suite of this target does.
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
//      tick and for ten with the count of working bus masters set aside
//      (it would keep the core awake), the count read on return - the
//      number that says the bus matrix served the core alone - and the
//      same block finished awake and verified; on the CH32V303 the same
//      again on DMA2's first channel
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
//   h  THE VECTORS: a block on each of DMA1's channels, each one's flag
//      reaching its own handler body and no other
// and on the CH32V303 alone, the second controller:
//   i  MEMORY TO MEMORY ON DMA2: its gate, a kilobyte copied on each of
//      the eleven channels and verified, and every completion flag found
//      where 11.3.11 puts it - INTFR for channels 1..7, the extended
//      register for 8..11, with INTFR's reserved top nibble watched
//   j  DMA2'S ELEVEN VECTORS: five at 72..76 and six at 98..103 of this
//      class's table, each body running for its own channel alone
//   k  REQUESTS ON DMA2: TIM5's update copying another timer's counter
//      into a buffer once a period (table 11-3's row), and TIM6's and
//      TIM7's wherever this build's timer driver reaches those timers
//   l  THE BLOCK ENGINES ON DMA2: the loop engine playing a table into
//      TIM5's compare lap after lap and the ping-pong engine filling two
//      blocks in turn, each serviced from its channel's own handler
//   m  THE 64 KB RULE: thirty-two bytes of flash straddling 0x0801_0000,
//      refused by DMA1 (11.2.3's note) and moved by DMA2 - and then handed
//      to DMA1 through the piecewise verbs, which never see a transfer
//      whole, to show whether this die's lot is one the note restricts
//   n  TWO CONTROLLERS ON ONE PAIR OF WIRES: USART2's engines on DMA1 and
//      UART4's on DMA2, a run each way across the board's two wires -
//      when the wires are there - with what each receiver framed while
//      its pad floated drained and counted first, and both ports released
//      at the end, since a receiver left running keeps its request on
//      its channel
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
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
                  DmaTxEngine<1, DmaRequestOf<DmaRequest::usart2_tx>::channel>,
                  DmaRxEngine<1, DmaRequestOf<DmaRequest::usart2_rx>::channel>>;
using LoopRx = DmaRxEngine<1, DmaRequestOf<DmaRequest::usart2_rx>::channel>;

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
        if (((Dma<1>::flags() >> shift) & DmaFlag::complete) != 0u) {
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
    Dma<1>::open();
    Dma<1>::stop_all();
    for (uint8_t ch = 1; ch <= Dma<1>::channel_count; ++ch) {
        Pfic::disable(dma_channel_irq(1, ch));
        Pfic::clear_pending(dma_channel_irq(1, ch));
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
    using Ch = DmaChannel<1, 1>;

    // The gate. It is closed at reset and every channel verb opens it,
    // so a channel programmed with the gate shut would write nothing.
    Rcc::disable(Bus::hb, rcc_hb_dma1);
    const bool closed = !Dma<1>::opened();
    Ch::stop();
    const bool opened = Dma<1>::opened();
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
    using Ch = DmaChannel<1, DmaRequestOf<DmaRequest::tim2_up>::channel>;
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

/// Letter c's measurement on the second controller, where the part has one:
/// the same four kilobytes on DMA2's first channel, awake and across one
/// idle() and ten. A template, so a part with one controller carries none
/// of it.
template <uint8_t C = 2>
void sleep_on_dma2() {
    if constexpr (C <= device::dma_controller_count) {
        using Ch = DmaChannel<C, 1>;
        scrub(source_bytes);
        Ch::stop();
        Stopwatch w;
        (void)Ch::load(copy(source_address(), destination, 4096, DmaWidth::byte));
        uint32_t spins = 4'000'000UL;
        while (!Ch::flag(DmaFlag::complete) && spins-- != 0u) {
        }
        const bool awake_done = Ch::flag(DmaFlag::complete);
        const uint32_t awake_us = w.us();
        Ch::stop();

        scrub(source_bytes);
        while (!Serial::tx_idle()) {
        }
        (void)Ch::load(copy(source_address(), destination, 4096, DmaWidth::byte));
        BusActivity::left();
        disable_interrupts();
        P::idle();
        const uint16_t left_one = Ch::remaining();
        for (uint8_t i = 0; i < 9u; ++i) {
            disable_interrupts();
            P::idle();
        }
        const uint16_t left_ten = Ch::remaining();
        BusActivity::entered();
        const uint16_t moved_one = static_cast<uint16_t>(4096u - left_one);
        const uint16_t moved_ten = static_cast<uint16_t>(4096u - left_ten);
        spins = 4'000'000UL;
        while (!Ch::flag(DmaFlag::complete) && spins-- != 0u) {
        }
        const bool exact = Ch::flag(DmaFlag::complete) && verify(source_bytes);
        Ch::stop();
        print(serial, "  DMA2: 4096 B awake: ", awake_us, " us; asleep: ", moved_one,
              " B across one idle(), ", moved_ten, " B across ten (", moved_ten / 10u,
              " B a wake)", crlf);
        bench.verdict("the second controller starves in Sleep the same way: four kilobytes that "
                      "take well under a millisecond awake do not finish in ten milliseconds of "
                      "sleep, tens of bytes a wake",
                      awake_done && awake_us < 1000UL && moved_one < 256u && moved_ten < 1024u);
        bench.verdict("and its block too finishes the moment the core is awake again, byte for "
                      "byte",
                      exact);
        Dma<C>::stop_all();
    }
}

// ===========================================================================
// c - the sleep: in Sleep no bus master but the core gets a cycle
// ===========================================================================
void tc_sleep() {
    all_off();
    using Ch = DmaChannel<1, 1>;

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
    // The count of working bus masters set aside for the ten sleeps, as
    // test_vx03_platform does: over a working channel the platform's
    // idle() returns at once (ch32vx03/bus_activity.hpp), and this letter
    // is about what the bus does while the core really sleeps.
    BusActivity::left();
    disable_interrupts();
    P::idle();
    const uint16_t left_one = Ch::remaining();
    for (uint8_t i = 0; i < 9u; ++i) {
        disable_interrupts();
        P::idle();
    }
    const uint16_t left_ten = Ch::remaining();
    BusActivity::entered();
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
    sleep_on_dma2();
}

// ===========================================================================
// d - the transfer error: a read of an address nothing answers at
// ===========================================================================
void td_error() {
    all_off();
    using Ch = DmaChannel<1, 3>;

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
    using Ch = DmaChannel<1, DmaRequestOf<DmaRequest::tim2_up>::channel>;

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
    using Low = DmaChannel<1, 1>;
    using High = DmaChannel<1, 2>;
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
        const uint32_t f = Dma<1>::flags();
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
        const uint32_t f = Dma<1>::flags();
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
    // DMA1's eighth channel is the CH32V203's alone.
    if constexpr (dma_channel_exists(1, ch)) {
        using C = DmaChannel<1, ch>;
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

    constexpr uint8_t count = Dma<1>::channel_count;
    print(serial, "  bodies that ran: ", ran,
          count == 8u ? " of 8; bodies that saw only their own complete flag: "
                      : " of 7; bodies that saw only their own complete flag: ",
          clean, crlf);
    bench.verdict(count == 8u
                      ? "all eight channels have a vector of their own - seven consecutive and "
                        "the eighth on this device class's own tail - and each body ran "
                        "exactly once"
                      : "all seven channels of DMA1 have a vector of their own, consecutive, "
                        "and each body ran exactly once",
                  ran == count);
    bench.verdict("and each body saw the ARMED flag and nothing else: isr() returns what is "
                  "up of what was armed, clears exactly that, and decides nothing",
                  clean == count);
    all_off();
}

// ===========================================================================
// The second controller - the CH32V303's alone (letters i..n)
// ===========================================================================
//
// Every letter below is a template on the controller's number, 2 by
// default, and does its work inside `if constexpr` on the part's count of
// controllers: on a part with one the body is a discarded branch the
// compiler never instantiates, and the image carries none of it - its
// state included, which lives in a template for the same reason.

/// What DMA2's handlers serve besides the bare channel: the two block
/// engines of letter l, and UART4's transport in letter n.
enum class Dma2Role : uint8_t { bare, player, source, uart4 };

template <uint8_t C = 2>
struct Dma2State {
    static inline volatile uint16_t calls[11] = {};
    static inline volatile uint32_t mask[11] = {};
    static inline volatile Dma2Role role = Dma2Role::bare;
    /// The ping-pong engine's two blocks.
    static inline volatile uint16_t block_a[8] = {};
    static inline volatile uint16_t block_b[8] = {};

    static void note(uint8_t index, uint32_t flags) {
        if (flags != 0u) {
            calls[index] = static_cast<uint16_t>(calls[index] + 1u);
            mask[index] = mask[index] | flags;
        }
    }

    static void reset() {
        role = Dma2Role::bare;
        for (uint8_t i = 0; i < 11u; ++i) {
            calls[i] = 0;
            mask[i] = 0;
        }
    }
};

/// The engines letter l drives: TIM5's update request, which table 11-3
/// puts on DMA2's channel 2, pacing a player and then a source. Every name
/// here hangs on a template parameter, so nothing of it is formed on a
/// part without the timer or the controller.
template <uint8_t C = 2, uint8_t timer = 5, DmaRequest request = DmaRequest::tim5_up>
struct Dma2Engines {
    using Row = DmaRequestOf<request>;
    static_assert(Row::controller == C, "table 11-3: TIM5's update is a DMA2 request");
    using Paced = Tim<timer>;
    using Player = DmaLoopEngine<C, Row::channel, uint16_t>;
    using Source = DmaPingPongEngine<C, Row::channel, uint16_t>;
};

/// UART4 with both its engines, where table 11-3 puts them: DMA2's channel
/// 5 out and channel 3 in. With an engine on each side the port arms no
/// interrupt of its own, so no UART4 vector is bound anywhere.
template <uint8_t C = 2, DmaRequest tx = DmaRequest::uart4_tx, DmaRequest rx = DmaRequest::uart4_rx>
struct Dma2Cross {
    using TxRow = DmaRequestOf<tx>;
    using RxRow = DmaRequestOf<rx>;
    static_assert(TxRow::controller == C && RxRow::controller == C,
                  "table 11-3: UART4's two requests are DMA2's on this class");
    using Tx = DmaTxEngine<C, TxRow::channel>;
    using Rx = DmaRxEngine<C, RxRow::channel>;
    using Port = Uart<4, P, 128, 128, UartFormat{}, Tx, Rx>;
    /// The two wires between the ports: USART2's pads and UART4's
    /// default column, whose port C only the larger packages bond.
    using Out2 = Pin<'A', 2>;
    using In2 = Pin<'A', 3>;
    using Out4 = Pin<Port::pads.tx.port, Port::pads.tx.pin>;
    using In4 = Pin<Port::pads.rx.port, Port::pads.rx.pin>;
};

template <uint8_t C>
void all_off2() {
    if constexpr (C <= device::dma_controller_count) {
        Dma<C>::open();
        Dma<C>::stop_all();
        for (uint8_t ch = 1; ch <= Dma<C>::channel_count; ++ch) {
            Pfic::disable(dma_channel_irq(C, ch));
            Pfic::clear_pending(dma_channel_irq(C, ch));
        }
        Dma2State<C>::reset();
    }
}

/// Is `Src` wired to `Dst`? The jumper test of letter g for any pair.
template <typename Src, typename Dst>
bool strapped() {
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

/// Everything a transport's receive engine has collected, published and
/// read away: how many bytes that was.
template <typename Port>
uint16_t drain() {
    uint16_t n = 0;
    (void)Port::harvest();
    uint8_t b = 0;
    while (Port::read_byte(b)) {
        ++n;
    }
    return n;
}

// ===========================================================================
// i - DMA2: memory to memory on every one of its eleven channels
// ===========================================================================
template <uint8_t C, uint8_t ch>
bool dma2_copy(uint32_t& cycles, bool& in_place, uint32_t& intfr) {
    using Ch = DmaChannel<C, ch>;
    Ch::stop();
    scrub(1024);
    Stopwatch w;
    const bool started = Ch::load(copy(source_address(), destination, 256, DmaWidth::word));
    uint32_t spins = 4'000'000UL;
    while (!Ch::flag(DmaFlag::complete) && spins-- != 0u) {
    }
    cycles = w.cycles();
    const bool done = Ch::flag(DmaFlag::complete);
    // Where the flag stands: INTFR for channels 1..7, the extended
    // register for 8..11 - and INTFR as a whole, whose top nibble is
    // reserved on this controller.
    intfr = Dma<C>::flags();
    const uint32_t extended = Dma<C>::extended_flags();
    in_place = Ch::extended ? ((extended >> Ch::flag_shift) & DmaFlag::complete) != 0u
                            : ((intfr >> Ch::flag_shift) & DmaFlag::complete) != 0u;
    const bool exact = done && verify(1024) && Ch::remaining() == 0u;
    Ch::clear(DmaFlag::all);
    const bool cleared = Ch::flags() == 0u;
    Ch::stop();
    return started && exact && cleared;
}

template <uint8_t C = 2>
void ti_dma2_memory() {
    if constexpr (C <= device::dma_controller_count) {
        all_off();
        all_off2<C>();

        // The gate: DMA2EN, beside DMA1EN in RCC_HBPCENR, closed at reset.
        Rcc::disable(Bus::hb, rcc_hb_dma2);
        const bool closed = !Dma<C>::opened();
        DmaChannel<C, 1>::stop();
        const bool opened = Dma<C>::opened();
        bench.verdict("DMA2's gate is closed until a channel verb opens it", closed && opened);

        uint32_t cycles[11] = {};
        bool placed[11] = {};
        uint32_t intfr[11] = {};
        const bool exact[11] = {
            dma2_copy<C, 1>(cycles[0], placed[0], intfr[0]),
            dma2_copy<C, 2>(cycles[1], placed[1], intfr[1]),
            dma2_copy<C, 3>(cycles[2], placed[2], intfr[2]),
            dma2_copy<C, 4>(cycles[3], placed[3], intfr[3]),
            dma2_copy<C, 5>(cycles[4], placed[4], intfr[4]),
            dma2_copy<C, 6>(cycles[5], placed[5], intfr[5]),
            dma2_copy<C, 7>(cycles[6], placed[6], intfr[6]),
            dma2_copy<C, 8>(cycles[7], placed[7], intfr[7]),
            dma2_copy<C, 9>(cycles[8], placed[8], intfr[8]),
            dma2_copy<C, 10>(cycles[9], placed[9], intfr[9]),
            dma2_copy<C, 11>(cycles[10], placed[10], intfr[10]),
        };
        uint8_t copies = 0;
        uint8_t in_place = 0;
        uint8_t quiet_top = 0;
        for (uint8_t i = 0; i < 11u; ++i) {
            if (exact[i]) {
                ++copies;
            }
            if (placed[i]) {
                ++in_place;
            }
            // Channels 8..11 report elsewhere: INTFR's top nibble, where
            // DMA1's eighth channel reports on the other series, stays
            // clear while they run.
            if (i >= 7u && (intfr[i] & 0xF000'0000UL) == 0u) {
                ++quiet_top;
            }
            print(serial, "  DMA2 channel ", static_cast<uint16_t>(i + 1u), ": 256 words in ",
                  cycles[i], " cycles, INTFR ", hex(intfr[i]), exact[i] ? "" : " - FAILED",
                  placed[i] ? "" : " - the flag was not where 11.3.11 puts it", crlf);
        }
        bench.verdict("all eleven channels of the second controller copy a kilobyte memory to "
                      "memory, byte for byte, the count down to zero and the flags cleared",
                      copies == 11u);
        bench.verdict("channels 1..7 report in DMA2_INTFR and channels 8..11 in the extended "
                      "register DMA2_EXTEM_INTFR, four bits a channel",
                      in_place == 11u);
        bench.verdict("and INTFR's top nibble stays clear while channels 8..11 run",
                      quiet_top == 4u);
        all_off2<C>();
    }
}

// ===========================================================================
// j - DMA2's eleven vectors
// ===========================================================================
template <uint8_t C, uint8_t ch>
void dma2_one_vector(uint8_t& ran, uint8_t& clean) {
    using Ch = DmaChannel<C, ch>;
    using S = Dma2State<C>;
    Ch::stop();
    Ch::arm(DmaFlag::complete, true);
    Pfic::enable(Ch::irq());
    (void)Ch::load(copy(source_address(), destination, 16, DmaWidth::word));
    Stopwatch w;
    while (S::calls[ch - 1u] == 0u && w.us() < 2000UL) {
    }
    if (S::calls[ch - 1u] == 1u) {
        ++ran;
    }
    if (S::mask[ch - 1u] == DmaFlag::complete) {
        ++clean;
    }
    Pfic::disable(Ch::irq());
    Ch::stop();
}

template <uint8_t C = 2>
void tj_dma2_vectors() {
    if constexpr (C <= device::dma_controller_count) {
        all_off();
        all_off2<C>();
        uint8_t ran = 0;
        uint8_t clean = 0;
        dma2_one_vector<C, 1>(ran, clean);
        dma2_one_vector<C, 2>(ran, clean);
        dma2_one_vector<C, 3>(ran, clean);
        dma2_one_vector<C, 4>(ran, clean);
        dma2_one_vector<C, 5>(ran, clean);
        dma2_one_vector<C, 6>(ran, clean);
        dma2_one_vector<C, 7>(ran, clean);
        dma2_one_vector<C, 8>(ran, clean);
        dma2_one_vector<C, 9>(ran, clean);
        dma2_one_vector<C, 10>(ran, clean);
        dma2_one_vector<C, 11>(ran, clean);
        print(serial, "  DMA2 bodies that ran: ", ran,
              " of 11; bodies that saw only their own complete flag: ", clean, crlf);
        bench.verdict("all eleven channels of DMA2 have a vector of their own - five at 72..76 "
                      "and six at 98..103 of this class's table - and each body ran exactly "
                      "once",
                      ran == 11u);
        bench.verdict("and each body saw its armed flag and nothing else, the four channels of "
                      "the extended register included",
                      clean == 11u);
        all_off2<C>();
    }
}

// ===========================================================================
// k - requests on DMA2: a timer's update, no pad
// ===========================================================================
/// The staircase of letter e on the second controller: timer `n`'s update
/// request moving one sample of the free-running ruler a period. A
/// template on the timer, so that a timer this build's timer driver does
/// not reach is a discarded branch and a line that says so.
template <uint8_t n, DmaRequest request>
void dma2_staircase(const char* verdict) {
    if constexpr (tim_present(n)) {
        using Clocker = Tim<n>;
        using Row = DmaRequestOf<request>;
        using Ch = DmaChannel<Row::controller, Row::channel>;
        Ruler::init();
        (void)Ruler::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                         .period = 0xFFFF});
        Ruler::enable(true);
        Clocker::init();
        (void)Clocker::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(Clocker::clock_hz(clock) / 1'000'000u - 1u),
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
        Clocker::interrupts(tim_ude, true);
        Clocker::enable(true);
        uint32_t spins = 4'000'000UL;
        while (!Ch::flag(DmaFlag::complete) && spins-- != 0u) {
        }
        const bool sampled = Ch::flag(DmaFlag::complete);
        Clocker::enable(false);
        Ruler::enable(false);
        Ch::stop();
        uint16_t steps_right = 0;
        for (uint16_t i = 1; i < sample_count; ++i) {
            const uint16_t step = static_cast<uint16_t>(work[i] - work[i - 1u]);
            if (step >= 495u && step <= 505u) {
                ++steps_right;
            }
        }
        print(serial, "  TIM", n, "'s update on DMA", Row::controller, " channel ", Row::channel,
              ": ", work[0], " ", work[1], " ... ", work[sample_count - 1u], crlf);
        bench.verdict(verdict, armed && sampled && steps_right == sample_count - 1u);
        Clocker::release();
        Ruler::release();
    } else {
        print(serial, "  TIM", n, ": this build's timer driver does not reach it, so its "
                      "request is not driven here",
              crlf);
    }
}

template <uint8_t C = 2>
void tk_dma2_requests() {
    if constexpr (C <= device::dma_controller_count) {
        all_off();
        all_off2<C>();
        if constexpr (device::has_tim5) {
            dma2_staircase<5, DmaRequest::tim5_up>(
                "TIM5's update request moves a staircase through DMA2's channel 2 (table 11-3): "
                "sixteen samples of another timer's counter, each a period after the last");
        }
        dma2_staircase<6, DmaRequest::tim6_up>(
            "TIM6's update request moves the same staircase through DMA2's channel 3");
        dma2_staircase<7, DmaRequest::tim7_up>(
            "TIM7's update request moves the same staircase through DMA2's channel 4");
        all_off2<C>();
    }
}

// ===========================================================================
// l - the block engines on DMA2, each serviced from its channel's handler
// ===========================================================================
template <uint8_t C = 2>
void tl_dma2_engines() {
    if constexpr (C <= device::dma_controller_count && device::has_tim5) {
        all_off();
        all_off2<C>();
        using E = Dma2Engines<C>;
        using Player = typename E::Player;
        using Source = typename E::Source;
        using S = Dma2State<C>;
        using Paced = typename E::Paced;

        // THE PLAYER: a table of eight compares poured into TIM5's first
        // channel by its own update, for ever - the controller's circular
        // mode, the handler doing nothing but count the laps.
        for (uint16_t i = 0; i < 8u; ++i) {
            work[i] = static_cast<uint16_t>(100u + 10u * i);
        }
        Paced::init();
        (void)Paced::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(Paced::clock_hz(clock) / 1'000'000u - 1u),
            .period = 199});   // an update every 200 us
        (void)Paced::output_channel(
            0, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = 1, .preload = false});
        S::role = Dma2Role::player;
        Player::arm(Paced::chcvr_address(0), DmaPriority::high);
        const bool playing = Player::start(work, 8);
        Paced::interrupts(tim_ude, true);
        Paced::enable(true);
        Stopwatch w;
        while (Player::laps() < 8u && w.us() < 50'000UL) {
        }
        const uint32_t laps_us = w.us();
        const uint32_t laps = Player::laps();
        const uint32_t landed = Paced::compare(0);
        const bool still_running = Player::running();
        Paced::enable(false);
        Player::stop();
        // The timer back to reset between the two engines: an update the
        // stopped player never served would otherwise stand as a request
        // and hand the source a first sample off the timer's pace.
        Paced::release();
        print(serial, "  the loop engine on DMA2 channel ", Player::channel, ": ", laps,
              " laps of eight 200 us updates in ", laps_us, " us, CH1CVR=", landed,
              ", faults ", Player::faults(), crlf);
        bench.verdict("DmaLoopEngine plays a table for ever on DMA2: eight laps of eight "
                      "updates counted by the channel's own handler at the timer's pace, the "
                      "value that landed one the table carries",
                      playing && still_running && laps >= 8u && laps_us > 11'000UL &&
                          laps_us < 15'000UL && landed >= 100u && landed <= 170u &&
                          Player::faults() == 0u);

        // THE SOURCE: the same request filling two caller-owned blocks
        // with samples of a free-running counter - the handler hands a
        // full block over and starts the other, the loop below reads one
        // and releases it.
        Ruler::init();
        (void)Ruler::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                         .period = 0xFFFF});
        Ruler::enable(true);
        Paced::init();
        (void)Paced::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(Paced::clock_hz(clock) / 1'000'000u - 1u),
            .period = 199});
        S::role = Dma2Role::source;
        Source::arm(Ruler::cnt_address(), DmaPriority::high);
        const bool streaming = Source::start(S::block_a, S::block_b, 8);
        Paced::interrupts(tim_ude, true);
        Paced::enable(true);
        uint16_t blocks = 0;
        uint16_t first[8] = {};
        Stopwatch s;
        while (blocks < 4u && s.us() < 50'000UL) {
            volatile uint16_t* full = Source::ready();
            if (full != nullptr) {
                if (blocks == 0u) {
                    for (uint8_t i = 0; i < 8u; ++i) {
                        first[i] = full[i];
                    }
                }
                (void)Source::release();
                ++blocks;
            }
        }
        Paced::enable(false);
        Ruler::enable(false);
        uint8_t steps = 0;
        for (uint8_t i = 1; i < 8u; ++i) {
            const uint16_t step = static_cast<uint16_t>(first[i] - first[i - 1u]);
            if (step >= 195u && step <= 205u) {
                ++steps;
            }
        }
        print(serial, "  the ping-pong engine on DMA2 channel ", Source::channel, ": ", blocks,
              " blocks of eight, overruns ", Source::overruns(), ", first block ", first[0], " ",
              first[1], " ... ", first[7], crlf);
        bench.verdict("DmaPingPongEngine fills two blocks in turn on DMA2, each a staircase of "
                      "the timer's period, handed over by the handler and released with no "
                      "overrun",
                      streaming && blocks >= 4u && steps == 7u && Source::overruns() == 0u);
        Source::stop();
        Paced::release();
        Ruler::release();
        all_off2<C>();
    }
}

// ===========================================================================
// m - the 64 KB rule of the CH32V303's DMA1
// ===========================================================================
template <uint8_t C = 2>
void tm_dma_64k() {
    if constexpr (C <= device::dma_controller_count) {
        all_off();
        all_off2<C>();
        // Thirty-two bytes of flash straddling 0x0801_0000: sixteen below
        // the boundary and sixteen above it. 11.2.3's note bars DMA1 of
        // this class from such a span on some lots, and a program cannot
        // read its lot - so DMA1 refuses it on every one; DMA2 carries no
        // such note.
        constexpr uint32_t straddle = 0x0800'FFF0UL;
        const DmaTransfer t = copy(reinterpret_cast<const volatile void*>(straddle), destination,
                                   32, DmaWidth::byte);
        using One = DmaChannel<1, 1>;
        using Two = DmaChannel<C, 1>;
        static_assert(One::bounded_to_64k && !Two::bounded_to_64k);
        One::stop();
        const bool refused = !One::accepts(t) && !One::load(t) && !One::enabled();
        const bool below_taken = One::accepts(copy(reinterpret_cast<const volatile void*>(straddle),
                                                  destination, 16, DmaWidth::byte));
        Two::stop();
        scrub(64);
        const bool taken = Two::load(t);
        uint32_t spins = 1'000'000UL;
        while (!Two::flag(DmaFlag::complete) && spins-- != 0u) {
        }
        bool same = Two::flag(DmaFlag::complete);
        const volatile uint8_t* flash = reinterpret_cast<const volatile uint8_t*>(straddle);
        for (uint8_t i = 0; i < 32u && same; ++i) {
            same = destination[i] == flash[i];
        }
        Two::stop();
        print(serial, "  0x0800FFF0..0x0801000F: DMA1 ", refused ? "refused it" : "TOOK it",
              ", DMA2 ", taken ? "moved it" : "refused it", same ? ", byte for byte" : ", WRONG",
              crlf);

        // THE SAME SPAN ON DMA1 WITH THE REFUSAL BYPASSED. The piecewise
        // verbs never see a transfer whole, so they do not ask the rule:
        // configuring the channel, its two addresses and its count one by
        // one hands DMA1 the span prepare() refused. What arrives says
        // whether THIS die's lot is one 11.2.3's note restricts - byte for
        // byte, or the upper half read from the bottom of the same 64 KB
        // page, the wrap such a lot would make.
        One::stop();
        scrub(64);
        const bool pieces =
            One::configure(DmaChannelConfig{.memory_to_memory = true,
                                            .peripheral_increment = true,
                                            .memory_increment = true}) &&
            One::set_peripheral(reinterpret_cast<volatile void*>(straddle)) &&
            One::set_memory(destination) && One::set_count(32);
        One::enable(true);
        spins = 1'000'000UL;
        while (!One::flag(DmaFlag::complete) && spins-- != 0u) {
        }
        const bool one_done = One::flag(DmaFlag::complete);
        One::stop();
        bool one_exact = one_done;
        for (uint8_t i = 0; i < 32u && one_exact; ++i) {
            one_exact = destination[i] == flash[i];
        }
        // The wrap: the sixteen bytes above the boundary taken from the
        // bottom of the page the transfer started in.
        const volatile uint8_t* page = reinterpret_cast<const volatile uint8_t*>(0x0800'0000UL);
        bool wrapped = one_done;
        for (uint8_t i = 0; i < 16u && wrapped; ++i) {
            wrapped = destination[i] == flash[i] && destination[16u + i] == page[i];
        }
        print(serial, "  the same span on DMA1 through the piecewise verbs: ",
              !pieces ? "NOT PROGRAMMED" : !one_done ? "NO completion"
              : one_exact ? "byte for byte - this die's lot is not one the note restricts"
              : wrapped   ? "the upper half read from 0x08000000 - the wrap the note warns of"
                          : "WRONG, and not the wrap either",
              crlf);
        bench.verdict("with the refusal bypassed DMA1 completes the crossing span, and what it "
                      "delivered is either the flash itself or the wrap the note describes - a "
                      "measurement of this die's lot, which the driver does not rely on",
                      pieces && one_done && (one_exact || wrapped));
        bench.verdict("DMA1 of this class refuses a span that crosses a 64 KB boundary and takes "
                      "the half of it that does not (11.2.3's note, kept on every lot because "
                      "none can be read)",
                      refused && below_taken);
        bench.verdict("and DMA2 moves the same thirty-two bytes exactly as the core reads them",
                      taken && same);
        all_off2<C>();
    }
}

// ===========================================================================
// n - two controllers on one pair of wires: USART2 on DMA1, UART4 on DMA2
// ===========================================================================
template <uint8_t C = 2>
void tn_dma2_uart4() {
    if constexpr (C <= device::dma_controller_count && device::has_usart(4)) {
        all_off();
        all_off2<C>();
        using X = Dma2Cross<C>;
        using Port4 = typename X::Port;
        using S = Dma2State<C>;
        // UART4's default column is PC10/PC11; USART2's is PA2/PA3.
        static_assert(Port4::pads.tx == Pad{'C', 10} && Port4::pads.rx == Pad{'C', 11});
        const bool forth = strapped<typename X::Out2, typename X::In4>();
        const bool back = strapped<typename X::Out4, typename X::In2>();
        print(serial, "  the wires: PA2 to PC11 ", forth ? "in place" : "ABSENT",
              ", PC10 to PA3 ", back ? "in place" : "ABSENT", crlf);
        if (!forth || !back) {
            bench.verdict("the round trip across the two controllers is skipped and says so: it "
                          "needs USART2's pads wired to UART4's",
                          true);
            all_off2<C>();
            return;
        }

        uart_mode = true;
        S::role = Dma2Role::uart4;
        const bool opened = Loop::init(clock, 115200) && Port4::init(clock, 115200);
        X::In2::input(PinPull::up);
        X::In4::input(PinPull::up);

        // WHAT A RECEIVER TAKES WHILE ITS PAD FLOATS. Each port's init
        // leaves its receive pad a floating input for the peer to drive,
        // and between the first port's init and the second's the peer's
        // transmit pad is not driven yet: whatever the receiver framed
        // then is in its ring. It is drained - and counted - before either
        // direction is measured.
        wait_us(2000);
        const uint16_t stale2 = drain<Loop>();
        const uint16_t stale4 = drain<Port4>();

        // USART2 out on DMA1's channel 7, UART4 in on DMA2's channel 3.
        static const uint8_t forth_msg[] = "one controller out, the other one in";
        constexpr uint16_t forth_len = sizeof(forth_msg) - 1u;
        (void)Port4::harvest();
        Loop::write(forth_msg, forth_len);
        uint8_t seen[64] = {};
        uint16_t got = 0;
        Stopwatch w;
        while (got < forth_len && w.us() < 50'000UL) {
            (void)Port4::harvest();
            uint8_t b = 0;
            while (got < forth_len && Port4::read_byte(b)) {
                seen[got++] = b;
            }
        }
        bool same_forth = got == forth_len;
        for (uint16_t i = 0; i < got && same_forth; ++i) {
            same_forth = seen[i] == forth_msg[i];
        }
        print(serial, "  USART2 (DMA1) to UART4 (DMA2): ", got, " of ", forth_len, " bytes", crlf);

        // UART4 out on DMA2's channel 5, USART2 in on DMA1's channel 6.
        static const uint8_t back_msg[] = "and back the other way";
        constexpr uint16_t back_len = sizeof(back_msg) - 1u;
        (void)Loop::harvest();
        Port4::write(back_msg, back_len);
        got = 0;
        Stopwatch r;
        while (got < back_len && r.us() < 50'000UL) {
            (void)Loop::harvest();
            uint8_t b = 0;
            while (got < back_len && Loop::read_byte(b)) {
                seen[got++] = b;
            }
        }
        bool same_back = got == back_len;
        for (uint16_t i = 0; i < got && same_back; ++i) {
            same_back = seen[i] == back_msg[i];
        }
        print(serial, "  UART4 (DMA2) to USART2 (DMA1): ", got, " of ", back_len,
              " bytes; faults ", Loop::dma_faults(), " and ", Port4::dma_faults(), crlf);
        print(serial, "  drained before the runs, framed while a pad floated: ", stale2,
              " bytes in USART2's ring and ", stale4, " in UART4's", crlf);
        if (!same_back) {
            print(serial, "  USART2 received:");
            for (uint16_t i = 0; i < got; ++i) {
                print(serial, " ", hex(seen[i]));
            }
            print(serial, crlf);
        }
        bench.verdict("a run poured out by USART2's engine on DMA1 comes in through UART4's on "
                      "DMA2, byte for byte - the slots of table 11-3 answering a real request",
                      opened && same_forth);
        bench.verdict("and the way back, UART4's transmit engine on DMA2 into USART2's receive "
                      "engine on DMA1, with no fault counted on either port",
                      same_back && Loop::dma_faults() == 0u && Port4::dma_faults() == 0u);
        // Both ports stopped, not only their channels: a receiver left
        // running with DMAR set raises its request again on the first noise
        // frame, and a request that stands on DMA2's channel 3 is what a
        // later letter's timer request on the same channel would be served
        // by instead - sixteen items in a few microseconds.
        uart_mode = false;
        Port4::release();
        Loop::release();
        all_off2<C>();
        all_off();
    }
}

/// The letters of the second controller, registered where there is one:
/// a template, so a part without DMA2 carries neither their code nor
/// their prose.
template <bool on = (device::dma_controller_count >= 2u)>
void register_dma2_letters() {
    if constexpr (on) {
        bench.letter('i', "DMA2: memory to memory on all eleven channels", ti_dma2_memory<>);
        bench.letter('j', "DMA2: the eleven vectors", tj_dma2_vectors<>);
        bench.letter('k', "DMA2: request-gated transfers from the timers' updates",
                     tk_dma2_requests<>);
        bench.letter('l', "DMA2: the loop and ping-pong engines", tl_dma2_engines<>);
        bench.letter('m', "the 64 KB rule: DMA1 refuses the span, DMA2 moves it", tm_dma_64k<>);
        bench.letter('n', "two controllers on one pair of wires: USART2 and UART4",
                     tn_dma2_uart4<>);
    }
}

/// A DMA2 channel's handler body, where the part has the controller: the
/// bare channel's counters, or the engine or port the running letter
/// handed the channel to.
template <uint8_t ch, uint8_t C = 2>
void dma2_vector() {
    if constexpr (C <= device::dma_controller_count) {
        using S = Dma2State<C>;
        const Dma2Role role = S::role;
        if constexpr (device::has_tim5) {
            if constexpr (ch == Dma2Engines<C>::Row::channel) {
                using Player = typename Dma2Engines<C>::Player;
                using Source = typename Dma2Engines<C>::Source;
                if (role == Dma2Role::player) {
                    const uint8_t f = Player::service();
                    if ((f & Player::flag_error) != 0u) {
                        Player::fail();
                    } else if ((f & Player::flag_complete) != 0u) {
                        Player::lap();
                    }
                    return;
                }
                if (role == Dma2Role::source) {
                    const uint8_t f = Source::service();
                    if ((f & Source::flag_error) != 0u) {
                        Source::fail();
                    } else if ((f & Source::flag_complete) != 0u) {
                        (void)Source::complete();
                    }
                    return;
                }
            }
        }
        if constexpr (device::has_usart(4)) {
            if constexpr (ch == Dma2Cross<C>::TxRow::channel ||
                          ch == Dma2Cross<C>::RxRow::channel) {
                if (role == Dma2Role::uart4) {
                    (void)Dma2Cross<C>::Port::dma_isr();
                    return;
                }
            }
        }
        S::note(static_cast<uint8_t>(ch - 1u), DmaChannel<C, ch>::isr());
    }
}

/// DMA1's eighth channel, the CH32V203's alone: the CH32V303's DMA1 has
/// seven, and its vector table has no entry for an eighth.
template <uint8_t ch = 8>
void dma1_eighth_vector() {
    if constexpr (dma_channel_exists(1, ch)) {
        note(static_cast<uint8_t>(ch - 1u), DmaChannel<1, ch>::isr());
    }
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf,
          device::dma_controller_count == 2u ? "test_vx03_dma - the two DMA controllers of RM ch. 11"
                                             : "test_vx03_dma - the DMA controller of RM ch. 11",
          crlf,
          "  one optional jumper: PA2 to PA3 (USART2's own two pads) is what letter g needs",
          crlf, "  the source of every copy is a 4 KB pattern in the image itself", crlf);
    bench.menu();
}

}  // namespace

// DMA1's vectors, eight on the CH32V203 and seven on the CH32V303.
// Channels 6 and 7 are USART2's, so their bodies serve the transport
// while letters g and n run and the bare channel the rest of the time.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() { note(0, brio::DmaChannel<1, 1>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { note(1, brio::DmaChannel<1, 2>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { note(2, brio::DmaChannel<1, 3>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() { note(3, brio::DmaChannel<1, 4>::isr()); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() { note(4, brio::DmaChannel<1, 5>::isr()); }

extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    if (uart_mode) {
        (void)Loop::dma_isr();
    } else {
        note(5, brio::DmaChannel<1, 6>::isr());
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    if (uart_mode) {
        (void)Loop::dma_isr();
    } else {
        note(6, brio::DmaChannel<1, 7>::isr());
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() { dma1_eighth_vector(); }

// DMA2's eleven, the CH32V303's alone: on the other series these bodies
// are empty and nothing in the vector table names them.
extern "C" BRIO_CH32_INTERRUPT void dma2_channel1_handler() { dma2_vector<1>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel2_handler() { dma2_vector<2>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() { dma2_vector<3>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel4_handler() { dma2_vector<4>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel5_handler() { dma2_vector<5>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel6_handler() { dma2_vector<6>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel7_handler() { dma2_vector<7>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel8_handler() { dma2_vector<8>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel9_handler() { dma2_vector<9>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel10_handler() { dma2_vector<10>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel11_handler() { dma2_vector<11>(); }

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
    register_dma2_letters();

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
