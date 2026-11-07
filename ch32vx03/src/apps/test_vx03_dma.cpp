// test_vx03_dma - the reference bench suite for the DMA controllers of the
// CH32V203 and the CH32V303 (ch32vx03/dma.hpp over RM ch. 11) with no
// peripheral request in the path: the channels memory to memory, the sleep,
// the transfer error, the priorities and the vectors. The CH32V203 has one
// controller of eight channels. The CH32V303 has two - DMA1 with seven, and
// DMA2 with eleven whose last four report in a flag register of their own -
// and letters i, j and m are that second controller's: registered on the
// part that has it, and compiled out of every other image.
//
// THREE SUITES, ONE CHAPTER. A channel has one owner for the whole image
// (docs/design/dma.md, "The ownership rule"), and this suite walks every
// channel of every controller memory to memory - each channel under its
// copy tag, DmaCopyOwner, the owner no request paces. The chapter's
// requests are its two siblings': test_vx03_dma_paced the timers' (a
// staircase, the burst engine, the ring at speed, the block engines), and
// test_vx03_dma_serial the USARTs' (the engines a transport takes, the
// priority rule, the overrun at a message's tail).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT IT MEASURES WITH. Nothing outside the chip. A memory-to-memory
// channel needs no request at all; the core's own STK counter is the ruler
// every number is weighed against; and the SOURCE of every copy is a
// four-kilobyte pattern in the image itself, which is a memory 11.1 lists
// as a legal source and which costs no RAM.
//
// THE PADS. None. NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14 (the
// debug port), PA11/PA12 (the USB pads), PC14/PC15 and PD0/PD1 (the
// crystals), PA0 (the CH32V203 board's KEY) - and PB2, the CH32V203
// board's LED, left undriven, as in every suite of this target.
//
// What is exercised, letter by letter:
//   a  MEMORY TO MEMORY: the block's gate, the three widths with and
//      without the increments, four kilobytes moved at each of them and
//      verified byte by byte, the half and complete flags counted, the
//      cost per item against the core's counter, the nine pairings of two
//      UNEQUAL widths against table 11-1, a store into a RUNNING channel
//      refused and a misaligned address refused
//   c  THE SLEEP: four kilobytes started, the platform's idle() for one
//      tick and for ten with the count of working bus masters set aside
//      (it would keep the core awake), the count read on return - the
//      number that says whether the bus matrix served the core alone,
//      which the dies measured answer differently (a FINDING printed,
//      not judged) - and the same block finished awake and verified; on
//      the CH32V303 the same again on DMA2's first channel
//   d  THE TRANSFER ERROR: five addresses no peripheral answers at,
//      read by a channel with its error interrupt armed, then WRITTEN at
//      each width, with the image's own pattern in the locked flash at
//      both of its addresses, and the top of the SRAM region, which does
//      refuse a channel, both ways; what the flag did, whether the silicon
//      dropped EN by itself, and the ledger's count across it - and the
//      copy engine's block on the address whose write faults
//   f  THE PRIORITIES: two channels contending for the bus, the order
//      measured by their own completion flags - once with the software
//      priority against the channel number, once with the channel
//      number alone
//   h  THE VECTORS: a block on each of DMA1's channels, each one's flag
//      reaching its own handler body and no other
// and on the CH32V303 alone, the second controller:
//   i  MEMORY TO MEMORY ON DMA2: its gate, a kilobyte copied on each of
//      the eleven channels and verified, and every completion flag found
//      where 11.3.11 puts it - INTFR for channels 1..7, the extended
//      register for 8..11, with INTFR's reserved top nibble watched
//   j  DMA2'S ELEVEN VECTORS: five at 72..76 and six at 98..103 of this
//      class's table, each body running for its own channel alone
//   m  THE 64 KB RULE: thirty-two bytes of flash straddling 0x0801_0000,
//      refused by DMA1 (11.2.3's note) and moved by DMA2 - and then handed
//      to DMA1 through the piecewise verbs, which never see a transfer
//      whole, to show whether this die's lot is one the note restricts
//
// build: boards = v203c6,v203c8,v303cb,v303vc
// build: monitor_speed = 115200

#include <stdint.h>
#include <utility>

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

using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

TestBench<Serial> bench;

constexpr uint32_t tim_hz = Tim<2>::clock_hz(clock);
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

/// Aligned to its own size where DMA1 is held to the 64 KB rule (the
/// CH32V303's): a 4 KB block on a 4 KB boundary cannot straddle a 64 KB
/// page, wherever the image grows to put it.
alignas(DmaChannel<1, 1>::bounded_to_64k ? 4096 : 4) constexpr Pattern source = make_pattern();
constexpr uint16_t source_bytes = 4096;

alignas(4) uint8_t destination[source_bytes];

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
// The handlers' counters
// ---------------------------------------------------------------------------

volatile uint16_t channel_calls[8] = {};
volatile uint32_t channel_mask[8] = {};

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

/// Every channel and every line back to reset, at the top of each letter,
/// so no letter inherits another's state.
void all_off() {
    Dma<1>::open();
    Dma<1>::stop_all();
    for (uint8_t ch = 1; ch <= Dma<1>::channel_count; ++ch) {
        Pfic::disable(dma_channel_irq(1, ch));
        Pfic::clear_pending(dma_channel_irq(1, ch));
    }
    for (uint8_t i = 0; i < 8u; ++i) {
        channel_calls[i] = 0;
        channel_mask[i] = 0;
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

    // UNEQUAL WIDTHS (table 11-1, PINC = MINC = 1): each end steps by its
    // own width, a value narrower than its target is zero-extended and a
    // wider one keeps its low bytes, both ends little-endian. Four items of
    // each of the nine pairings, the source the image's pattern and the
    // target scrubbed to 0xEE, every target byte compared - the bytes past
    // the four items included, which must stay 0xEE.
    {
        constexpr DmaWidth widths[3] = {DmaWidth::byte, DmaWidth::half, DmaWidth::word};
        const uint8_t* src = source_first_bytes();
        uint8_t pairs_exact = 0;
        for (const DmaWidth ps : widths) {
            for (const DmaWidth ms : widths) {
                const uint8_t pb = static_cast<uint8_t>(1u << static_cast<uint8_t>(ps));
                const uint8_t mb = static_cast<uint8_t>(1u << static_cast<uint8_t>(ms));
                for (uint8_t i = 0; i < 32u; ++i) {
                    destination[i] = 0xEE;
                }
                Ch::stop();
                DmaTransfer t = copy(source_address(), destination, 4, ps);
                t.config.memory_width = ms;
                const bool started = Ch::load(t);
                const bool done = started && wait_complete(Ch::flag_shift);
                Ch::stop();
                uint8_t wrong = 0;
                for (uint8_t k = 0; k < 4u; ++k) {
                    for (uint8_t b = 0; b < mb; ++b) {
                        const uint8_t want = b < pb ? src[k * pb + b] : 0u;
                        if (destination[k * mb + b] != want) {
                            ++wrong;
                        }
                    }
                }
                for (uint8_t i = static_cast<uint8_t>(4u * mb); i < 32u; ++i) {
                    if (destination[i] != 0xEEu) {
                        ++wrong;
                    }
                }
                if (done && wrong == 0u) {
                    ++pairs_exact;
                } else {
                    print(serial, "  source ", 8u * pb, " bits into target ", 8u * mb, " bits: ",
                          done ? "done" : "NOT DONE", ", ", wrong, " byte(s) off table 11-1, target ",
                          hex(destination[0]), " ", hex(destination[1]), " ", hex(destination[2]), " ",
                          hex(destination[3]), crlf);
                }
            }
        }
        print(serial, "  unequal widths: ", pairs_exact, " of 9 pairings exact against table 11-1", crlf);
        bench.verdict("THE NINE WIDTH PAIRINGS move as table 11-1 draws them: each end stepping by "
                      "its own width, a narrower value zero-extended, a wider one cut to its low "
                      "bytes, nothing written past the last item",
                      pairs_exact == 9u);
    }

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
        print(serial, "  FINDING: in Sleep DMA2's channel ", moved_ten == 4096u ? "KEEPS MOVING" : "STALLS",
              " - ", moved_ten, " B across ten sleeps", crlf);
        bench.verdict("the second controller's block takes well under a millisecond awake, and "
                      "what it moves across ten sleeps is the finding above, not a verdict: the "
                      "dies measured differ (the comment on letter c)",
                      awake_done && awake_us < 1000UL);
        bench.verdict("and its block too finishes the moment the core is awake again, byte for "
                      "byte",
                      exact);
        Dma<C>::stop_all();
    }
}

// ===========================================================================
// c - the sleep: what a channel moves while the core is not awake
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
    // Whether the channel moves while the core sleeps is PRINTED, not
    // judged: RM 2.3.2 says every peripheral runs in Sleep, the
    // CH32V203C8T6 and the CH32V303VCT6 starve the channel (the handful
    // of items in flight and what each wake runs, tens of bytes a wake)
    // and the CH32V303CBT6 serves it (the block done inside the first
    // sleep a tick ends). Either way the idle path refuses a sleep over a
    // working channel (ch32vx03/bus_activity.hpp), so nothing in brio
    // depends on the answer; on a die that serves it the refusal costs a
    // sleep and breaks nothing. The first idle() returns at once on the
    // latch the last interrupt left (ch32vx03/platform.hpp), on every die.
    print(serial, "  FINDING: in Sleep the DMA ", moved_ten == 4096u ? "KEEPS MOVING" : "STALLS", " - ",
          moved_ten, " B across ten sleeps against 4096 awake", crlf);
    bench.verdict("the same four kilobytes take well under a millisecond awake; what they do "
                  "across ten sleeps is the finding above, not a verdict: the dies measured "
                  "differ, and the idle path refuses a sleep over a working channel on all of them",
                  awake_done && awake_us < 1000UL);
    bench.verdict("and the block finishes the moment the core is awake again, byte for byte",
                  exact);
    all_off();
    sleep_on_dma2();
}

// ===========================================================================
// d - the transfer error: reads and writes of addresses nothing answers at
// ===========================================================================

/// The first address letter d found whose word write raises a transfer
/// error, for the copy engine's block after it; zero when none did.
uint32_t te_target = 0;

/// The holes of figure 1-11 letter d reads and writes.
constexpr uint32_t holes[5] = {0x3000'0000UL, 0x4000'8000UL, 0x4002'4000UL, 0x6000'0000UL, 0xA000'0000UL};
/// An address that does refuse a channel, read or written at every width: the
/// SRAM region's top, far past either part's SRAM.
constexpr uint32_t refused_address = 0x2FFF'F000UL;

/// Every word of the SRAM below the stack in use, folded: what a refused store
/// would land in. Masked at its call, so nothing else moves a word under it.
[[gnu::noinline]] uint32_t sram_fold() {
    const uint8_t here = 0;
    const uint32_t end = (static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&here)) - 128u) & ~3UL;
    uint32_t fold = 0x811C'9DC5UL;
    for (uint32_t a = 0x2000'0000UL; a < end; a += 4u) {
        fold = (fold ^ *reinterpret_cast<const volatile uint32_t*>(a)) * 16'777'619UL;
    }
    return fold;
}

void td_error() {
    all_off();
    using Ch = DmaChannel<1, 3>;

    // 11.2.1: "Reading and writing a reserved address area results in a
    // DMA transmission error", the hardware clears EN and the channel
    // stops. Five addresses outside every window figure 1-11 draws.
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
    Ch::arm(DmaFlag::error | DmaFlag::complete, true);
    scrub(16);
    (void)Ch::load(copy(reinterpret_cast<const volatile void*>(refused_address), destination, 4, DmaWidth::word));
    uint32_t refused_read = 0;
    for (uint32_t spins = 200'000UL; spins != 0u && (refused_read & (DmaFlag::error | DmaFlag::complete)) == 0u; --spins) {
        refused_read = Ch::flags();
    }
    const bool refused_read_en = Ch::enabled();
    Ch::stop();
    print(serial, "  of five holes: ", errors, " raised a transfer error, ", completions,
          " completed as an ordinary block; a read of ", hex(refused_address), ": flags ", hex(refused_read),
          refused_read_en ? " EN set" : " EN clear", crlf);
    bench.verdict("A READ OF THE SRAM REGION'S TOP raises the transfer error, and the hardware clears EN",
                  (refused_read & DmaFlag::error) != 0u && !refused_read_en);
    bench.verdict("five addresses no window of the map answers at were each read by a "
                  "channel with its error interrupt armed, and every one of them ended",
                  static_cast<uint8_t>(errors + completions) == 5u);
    bench.verdict("and wherever a transfer error IS raised the hardware clears EN by itself "
                  "(11.3.3), so no channel was left enabled behind one",
                  left_enabled == 0u);

    // WRITES. 11.1 lists the flash among a channel's destinations, and with
    // the FPEC locked - as at every boot; no letter of this suite unlocks
    // it - a store into the array cannot program; 11.2.1's reserved area
    // is the five holes above, written this time. Each target takes four
    // items at each width from a RAM source, the error interrupt armed and
    // the channel's line enabled; the image's pattern - the flash target -
    // read back afterwards must still be itself.
    const uint32_t alias = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&source.word[64]));
    const uint32_t targets[8] = {alias,    flash_array_base + (alias - flash_alias_base),
                                 holes[0], holes[1], holes[2], holes[3], holes[4], refused_address};
    const uint32_t before_word = source.word[64];
    for (uint8_t i = 0; i < 16u; ++i) {
        destination[i] = static_cast<uint8_t>(0xA0u + i);
    }
    uint8_t written = 0;
    uint8_t write_errors = 0;
    uint8_t errors_clean = 0;
    te_target = 0;
    Pfic::clear_pending(dma_channel_irq(1, Ch::number));
    Pfic::enable(dma_channel_irq(1, Ch::number));
    for (const uint32_t target : targets) {
        for (const DmaWidth w : {DmaWidth::byte, DmaWidth::half, DmaWidth::word}) {
            Ch::stop();
            Ch::arm(DmaFlag::error | DmaFlag::complete, true);
            channel_calls[2] = 0;
            channel_mask[2] = 0;
            (void)Ch::load(copy(destination, reinterpret_cast<volatile void*>(target), 4, w));
            uint32_t spins = 200'000UL;
            uint32_t f = 0;
            while (spins-- != 0u) {
                f = Ch::flags() | channel_mask[2];
                if ((f & (DmaFlag::error | DmaFlag::complete)) != 0u) {
                    break;
                }
            }
            wait_us(50);
            f = f | channel_mask[2];
            const bool en = Ch::enabled();
            const uint16_t left = Ch::remaining();
            const bool counted = Dma<1>::counted(Ch::number);
            const uint8_t masters = BusActivity::active();
            Ch::stop();
            const bool given_back = !Dma<1>::counted(Ch::number) && BusActivity::active() == 0u;
            if ((f & (DmaFlag::error | DmaFlag::complete)) != 0u) {
                ++written;
            }
            if ((f & DmaFlag::error) != 0u) {
                ++write_errors;
                if (!en && channel_calls[2] >= 1u && counted && masters == 1u && given_back) {
                    ++errors_clean;
                }
                if (te_target == 0u && w == DmaWidth::word) {
                    te_target = target;
                }
            }
            if (w == DmaWidth::word || (f & DmaFlag::error) != 0u) {
                print(serial, "  a ", 8u << static_cast<uint8_t>(w), "-bit write of 4 to ", hex(target), ": flags ",
                      hex(f), en ? ", EN set" : ", EN clear", ", count left ", left, ", the vector entered ",
                      channel_calls[2], " time(s); the ledger ", counted ? "counting it" : "not counting it", " (",
                      masters, " bus master(s)), ", given_back ? "given back by stop()" : "NOT GIVEN BACK", crlf);
            }
        }
    }
    Ch::stop();
    Pfic::disable(dma_channel_irq(1, Ch::number));
    const bool pattern_kept = source.word[64] == before_word;
    print(serial, "  of 24 writes (8 targets, 3 widths): ", write_errors, " raised a transfer error; the pattern ",
          pattern_kept ? "intact" : "CHANGED", crlf);
    bench.verdict("A WRITE INTO THE LOCKED FLASH, or into a hole of the map, ends at every width - a transfer "
                  "error or an ordinary completion - and the array is untouched",
                  written == 24u && pattern_kept);
    bench.verdict("A WRITE TO THE SRAM REGION'S TOP raises the transfer error at each of the three widths, and "
                  "nothing else written does",
                  write_errors == 3u && te_target == refused_address);
    // WHERE A REFUSED STORE GOES. The SRAM below the stack folded before and
    // after four items at each width, into that address and into the first byte
    // past the part's SRAM, masked, the channel's line silent. A store that
    // landed anywhere in it, SRAM+0 first of all, changes the fold.
    uint8_t refused = 0;
    uint8_t moved = 0;
    Ch::stop();
    for (const uint32_t target : {refused_address, 0x2000'0000UL + device::sram_bytes}) {
        for (const DmaWidth w : {DmaWidth::byte, DmaWidth::half, DmaWidth::word}) {
            P::CriticalSection cs;
            const uint32_t before = sram_fold();
            Ch::arm(DmaFlag::error | DmaFlag::complete, false);
            (void)Ch::load(copy(destination, reinterpret_cast<volatile void*>(target), 4, w));
            uint32_t f = 0;
            for (uint32_t spins = 200'000UL; spins != 0u && (f & (DmaFlag::error | DmaFlag::complete)) == 0u; --spins) {
                f = Ch::flags();
            }
            Ch::stop();
            refused = static_cast<uint8_t>(refused + ((f & DmaFlag::error) != 0u ? 1u : 0u));
            moved = static_cast<uint8_t>(moved + (sram_fold() != before ? 1u : 0u));
        }
    }
    print(serial, "  six refused stores: ", refused, " raised the error, ", moved, " moved a word of the SRAM below the stack",
          crlf);
    bench.verdict("A REFUSED STORE LANDS NOWHERE: at the SRAM region's top and just past the part's SRAM, the error "
                  "raised at each width and no word of the SRAM below the stack moved",
                  refused == 6u && moved == 0u);
    bench.verdict("... and wherever one raises a transfer error the hardware drops EN, the vector is entered, "
                  "and the controller's LEDGER keeps the channel's bus-master count until software stops the "
                  "channel, which gives it back",
                  errors_clean == write_errors);
    if (te_target != 0u) {
        // The same refusal under an ENGINE: the copy engine's block ends on
        // the error - the fault counted, the engine free, the count given
        // back - and its next block, into RAM, runs whole.
        using FaultCopier = DmaCopyEngine<1, 4>;
        FaultCopier::arm(DmaPriority::low, false);
        FaultCopier::clear_faults();
        const bool launched = FaultCopier::copy(reinterpret_cast<uint32_t*>(te_target),
                                                reinterpret_cast<const uint32_t*>(destination), 4);
        uint32_t spins = 200'000UL;
        while (FaultCopier::busy() && spins-- != 0u) {
        }
        const bool freed = !FaultCopier::busy();
        const uint32_t faults = FaultCopier::faults();
        const uint8_t masters_after = BusActivity::active();
        scrub(64);
        const bool relaunched = FaultCopier::copy(reinterpret_cast<uint32_t*>(destination),
                                                  reinterpret_cast<const uint32_t*>(source.word), 16);
        spins = 200'000UL;
        while (FaultCopier::busy() && spins-- != 0u) {
        }
        const bool next_whole = relaunched && !FaultCopier::busy() && verify(64) && FaultCopier::faults() == 1u;
        FaultCopier::stop();
        print(serial, "  the copy engine into ", hex(te_target), ": ", launched ? "launched" : "REFUSED", ", ",
              freed ? "ended" : "STILL BUSY", ", faults ", faults, ", bus masters after ", masters_after,
              "; the next block into RAM ", next_whole ? "whole" : "NOT WHOLE", crlf);
        bench.verdict("the copy engine's block into that address ends on the error with the fault counted, "
                      "the engine free and no bus master left counted - and its next block runs whole",
                      launched && freed && faults == 1u && masters_after == 0u && next_whole);
    } else {
        print(serial, "  no write raised one either: the engines' error path stays unprovoked", crlf);
    }
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
// The second controller - the CH32V303's alone (letters i, j and m)
// ===========================================================================
//
// Every letter below is a template on the controller's number, 2 by
// default, and does its work inside `if constexpr` on the part's count of
// controllers: on a part with one the body is a discarded branch the
// compiler never instantiates, and the image carries none of it - its
// state included, which lives in a template for the same reason.

/// What DMA2's handlers count: the bare channels' completions.
template <uint8_t C = 2>
struct Dma2State {
    static inline volatile uint16_t calls[11] = {};
    static inline volatile uint32_t mask[11] = {};

    static void note(uint8_t index, uint32_t flags) {
        if (flags != 0u) {
            calls[index] = static_cast<uint16_t>(calls[index] + 1u);
            mask[index] = mask[index] | flags;
        }
    }

    static void reset() {
        for (uint8_t i = 0; i < 11u; ++i) {
            calls[i] = 0;
            mask[i] = 0;
        }
    }
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

/// The letters of the second controller, registered where there is one:
/// a template, so a part without DMA2 carries neither their code nor
/// their prose.
template <bool on = (device::dma_controller_count >= 2u)>
void register_dma2_letters() {
    if constexpr (on) {
        bench.letter('i', "DMA2: memory to memory on all eleven channels", ti_dma2_memory<>);
        bench.letter('j', "DMA2: the eleven vectors", tj_dma2_vectors<>);
        bench.letter('m', "the 64 KB rule: DMA1 refuses the span, DMA2 moves it", tm_dma_64k<>);
    }
}


/// A DMA2 channel's handler body, where the part has the controller: the
/// bare channel's counters.
template <uint8_t ch, uint8_t C = 2>
void dma2_vector() {
    if constexpr (C <= device::dma_controller_count) {
        Dma2State<C>::note(static_cast<uint8_t>(ch - 1u), DmaChannel<C, ch>::isr());
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
          device::dma_controller_count == 2u
              ? "test_vx03_dma - the two DMA controllers of RM ch. 11, memory to memory"
              : "test_vx03_dma - the DMA controller of RM ch. 11, memory to memory",
          crlf, "  the source of every copy is a 4 KB pattern in the image itself", crlf);
    bench.menu();
}

/// THE IMAGE'S DMA SLOTS (docs/design/dma.md, "The ownership rule"): every
/// channel of every controller the part has is a copy's, under that
/// channel's copy tag - the copy engine of letter d making the same claim
/// on its channel - and no request has one.
template <uint8_t c, uint8_t... index>
constexpr bool copies_own_every_channel(std::integer_sequence<uint8_t, index...>) {
    return (dma_claim<DmaSlotId<c, index + 1u>, DmaCopyOwner<c, index + 1u>> && ...);
}
static_assert(copies_own_every_channel<1>(std::make_integer_sequence<uint8_t, dma_channels_of(1)>{}));
static_assert(copies_own_every_channel<2>(std::make_integer_sequence<uint8_t, dma_channels_of(2)>{}));

}  // namespace

// DMA1's vectors, eight on the CH32V203 and seven on the CH32V303, each
// counting its own channel's flags.
BRIO_CH32_VECTOR(dma1_channel1_handler) { note(0, brio::DmaChannel<1, 1>::isr()); }
BRIO_CH32_VECTOR(dma1_channel2_handler) { note(1, brio::DmaChannel<1, 2>::isr()); }
BRIO_CH32_VECTOR(dma1_channel3_handler) { note(2, brio::DmaChannel<1, 3>::isr()); }
BRIO_CH32_VECTOR(dma1_channel4_handler) { note(3, brio::DmaChannel<1, 4>::isr()); }
BRIO_CH32_VECTOR(dma1_channel5_handler) { note(4, brio::DmaChannel<1, 5>::isr()); }
BRIO_CH32_VECTOR(dma1_channel6_handler) { note(5, brio::DmaChannel<1, 6>::isr()); }
BRIO_CH32_VECTOR(dma1_channel7_handler) { note(6, brio::DmaChannel<1, 7>::isr()); }
BRIO_CH32_VECTOR(dma1_channel8_handler) { dma1_eighth_vector(); }

// DMA2's eleven, the CH32V303's alone: on the other series these bodies
// are empty and nothing in the vector table names them.
BRIO_CH32_VECTOR(dma2_channel1_handler) { dma2_vector<1>(); }
BRIO_CH32_VECTOR(dma2_channel2_handler) { dma2_vector<2>(); }
BRIO_CH32_VECTOR(dma2_channel3_handler) { dma2_vector<3>(); }
BRIO_CH32_VECTOR(dma2_channel4_handler) { dma2_vector<4>(); }
BRIO_CH32_VECTOR(dma2_channel5_handler) { dma2_vector<5>(); }
BRIO_CH32_VECTOR(dma2_channel6_handler) { dma2_vector<6>(); }
BRIO_CH32_VECTOR(dma2_channel7_handler) { dma2_vector<7>(); }
BRIO_CH32_VECTOR(dma2_channel8_handler) { dma2_vector<8>(); }
BRIO_CH32_VECTOR(dma2_channel9_handler) { dma2_vector<9>(); }
BRIO_CH32_VECTOR(dma2_channel10_handler) { dma2_vector<10>(); }
BRIO_CH32_VECTOR(dma2_channel11_handler) { dma2_vector<11>(); }

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "memory to memory: the gate, the widths, the flags, the cost", ta_memory);
    bench.letter('c', "the sleep: what a channel moves while the core is not awake", tc_sleep);
    bench.letter('d', "the transfer error: five holes read and written, the locked flash written, the SRAM region's top",
                 td_error);
    bench.letter('f', "the priorities: two channels contending for one bus", tf_priority);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
