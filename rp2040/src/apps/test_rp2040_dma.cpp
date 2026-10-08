// test_rp2040_dma - the reference bench suite for the RP2040's DMA
// controller (rp2040/dma.hpp, datasheet 2.5): twelve channels with
// their trigger aliases, the two interrupt lines, chaining, address
// wrapping, the pacing timers, the checksum sniffer, the abort with
// erratum E13's workaround, a bus error - and the two engines in a
// UART's slots, this console's own transmit side among them.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// THE CONSOLE PRINTS THROUGH THE DMA: UART0's transport names a
// transmit engine on channel 0, so every character of every verdict
// below travelled by a block the engine started off the ring's run
// and released on the line-0 interrupt. Its receive side stays on the
// interrupt (a keystroke at a time wants no block). The INSTRUMENT is
// UART1 on GP4/GP5 under its loop-back (the serial suite's), with an
// engine in EACH slot, channels 2 and 3. NOTHING TO WIRE.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp).
//
// What is exercised, letter by letter:
//   a  the block as found and the refusals: twelve channels by the
//      silicon's own count, a channel idle at its reset word, a zero
//      count, a misaligned word, a chain to itself or to a thirteenth
//      channel refused; a channel STALLED on a pacing timer that never
//      requests (BUSY standing, every configuring verb refused), then
//      aborted E13's way - BUSY down, nothing raised, the routes back
//   b  memory to memory at byte, half-word and word width: the copy
//      exact, TRANS_COUNT at zero, the raw status raised and cleared by
//      writing one, BUSY down; four kilobytes of words timed - one
//      transfer a cycle
//   c  the two lines: a completion on channel 4 routed to line 0 counted
//      in its handler, the same routed to line 1 counted in the other,
//      IRQ_QUIET silencing a completion, and a null trigger under
//      IRQ_QUIET raising exactly one
//   d  chaining: channel 4 completes and triggers channel 5, prepared
//      and waiting - both copies exact, both raised
//   e  address wrapping: a sixteen-byte source read through a ring of
//      sixteen fills sixty-four bytes of destination four times over
//   f  a pacing timer: one transfer a microsecond at 125 MHz (X 1, Y
//      125), a thousand bytes timed against the ruler
//   g  the sniffer: the sum, the CRC-16-CCITT (util/crc.hpp's, the
//      same seed) and the CRC-32 of a buffer against their software
//      twins; the reversed and inverted result options
//   h  a bus error: a block reading a hole in the map - what the silicon
//      does is REPORTED (the error bits, the halt, the interrupt), the
//      verdict only that the channel ended and came back clean
//   i  the instrument's engines: 4096 bytes through UART1's loop-back at
//      3 Mbaud with an engine in each slot, harvested by the loop,
//      byte-exact, the line's interrupts counted against the bytes
//   j  this console's transmit engine under a burst longer than its
//      ring: every byte out in order (the host reads it), the blocks
//      counted, no fault
//   m  THE COPY ENGINE AND THE BEAT PER BLOCK: DmaCopyEngine<8> polled -
//      four kilobytes as words, an odd count of half-words, a byte run at
//      odd addresses, each exact with the bytes either side untouched; a
//      fill of 256 words from one cell; a misaligned half-word run, an
//      empty one and a second block over a running one refused - then a
//      transmit engine whose widest beat is a word poured into ONE word
//      cell, unpaced, a byte run, a half-word run and a word run in turn:
//      the cell's low byte, low half and whole word written, CTRL's
//      DATA_SIZE following the run, and a misaligned half-word run
//      refused with the claim given back
//   t  tx_idle() ON THE PAD under the transmit engine: eight frames on
//      GP4 with the loop-back off, the pad read through SIO, tx_idle()
//      turning true at the last stop bit's end
//   v  K BREAKS IN N SLOTS on UART1's loop-back: 64 slots, four of them
//      a break, through the interrupt receiver (60 delivered, each break
//      an entry counted and dropped) and through the receive engine
//      (every byte sent delivered in order, the breaks' zero bytes among
//      them, each break counted once by the error interrupt)
//   (k and l are diagnostics, off the all-key: one engine at a time on
//      the instrument's loop-back)
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/crc.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P = Rp2040Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins, 128, 512, DmaTxEngine<0>>;
constexpr Serial serial;

constexpr UartPins instrument_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using Instrument = Uart<1, instrument_pins, 1024, 1024, DmaTxEngine<2>, DmaRxEngine<3>>;


TestBench<Serial, 16> bench;

volatile uint32_t dma0_irqs = 0;
volatile uint32_t dma1_irqs = 0;
volatile uint32_t ch4_on_line0 = 0;
volatile uint32_t ch4_on_line1 = 0;
volatile uint32_t serial_blocks = 0;

uint32_t us_now() { return Timer::now_low(); }

void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

void console_drain() {
    const uint32_t t0 = us_now();
    while (!Serial::tx_idle() && us_now() - t0 < 500'000u) {
    }
}

// The buffers every letter copies between: aligned for the word width
// and for the sixteen-byte ring of letter e.
alignas(16) uint8_t src[4096];
alignas(16) uint8_t dst[4096];

void fill(uint8_t* p, uint32_t n, uint32_t seed) {
    uint32_t s = seed;
    for (uint32_t i = 0; i < n; ++i) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        p[i] = static_cast<uint8_t>(s);
    }
}
bool same(const uint8_t* a, const uint8_t* b, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

using C4 = DmaChannel<4>;
using C5 = DmaChannel<5>;

/// A block on channel 4 and its wait, bounded.
bool run_c4(const DmaTransfer& t, uint32_t budget_us = 100'000u) {
    if (!C4::load(t)) {
        return false;
    }
    const uint32_t t0 = us_now();
    while (C4::busy() && us_now() - t0 < budget_us) {
    }
    return !C4::busy();
}

// -----------------------------------------------------------------------------
void ta_as_found() {
    print(serial, "  N_CHANNELS=", Dma::channels(), ", block ", Dma::released() ? "released" : "IN RESET",
          ", channel 4: enabled=", C4::enabled(), " busy=", C4::busy(), " errors=", hex(C4::errors()),
          crlf);
    bench.verdict("the silicon counts twelve channels", Dma::channels() == 12u);
    bench.verdict("channel 4 idles clean after the block's reset",
                  !C4::busy() && C4::errors() == 0u && !C4::raised());

    bench.verdict("a zero count is refused", !C4::load({.read = src, .write = dst, .count = 0}));
    bench.verdict("a word transfer from an odd address is refused",
                  !C4::load({.read = src + 1, .write = dst, .count = 4, .config = {.size = DmaSize::word}}));
    bench.verdict("a chain to the channel itself and to a thirteenth channel are refused",
                  !C4::configure({.chain_to = 4}) && !C4::configure({.chain_to = 12}));

    // STALLED: timer 0 requests nothing, so the block never moves.
    DmaTimer<0>::set(0, 1);
    C4::route(0, true);
    const bool started = C4::load({.read = src, .write = dst, .count = 16, .config = {.treq = DmaTimer<0>::dreq()}});
    spin_us(100);
    const bool stalled = C4::busy() && C4::count() == 16u;
    const bool refused = !C4::configure({}) && !C4::set_count(8) && !C4::load({.read = src, .write = dst, .count = 8});
    print(serial, "  stalled on a silent pacing timer: busy=", C4::busy(), " count=", C4::count(),
          ", credits ", C4::debug_credits(), crlf);
    bench.verdict("a channel waiting on a request that never comes stays BUSY at its full count, "
                  "and every configuring verb refuses it",
                  started && stalled && refused);
    const uint32_t irqs_before = ch4_on_line0;
    const bool aborted = C4::abort();
    spin_us(100);
    print(serial, "  aborted: busy=", C4::busy(), " raised=", C4::raised(), " pending0=", C4::pending(0),
          " routed0=", C4::routed(0), " line-0 handler runs +", ch4_on_line0 - irqs_before, crlf);
    bench.verdict("abort() (E13's way) brings BUSY down, leaves nothing raised or pending, "
                  "runs no handler and puts the route back",
                  aborted && !C4::busy() && !C4::raised() && !C4::pending(0) && C4::routed(0) &&
                      ch4_on_line0 == irqs_before);
    C4::stop();
}

// -----------------------------------------------------------------------------
void tb_memory() {
    struct Leg { DmaSize size; uint32_t bytes; const char* name; };
    constexpr Leg legs[] = {{DmaSize::byte, 256, "256 bytes"}, {DmaSize::half, 256, "128 half-words"},
                            {DmaSize::word, 256, "64 words"}};
    for (const Leg& l : legs) {
        fill(src, l.bytes, 0x1234u + l.bytes + static_cast<uint32_t>(l.size));
        fill(dst, l.bytes, 0x9999u);
        const bool done = run_c4({.read = src, .write = dst, .count = l.bytes / dma_size_bytes(l.size),
                                  .config = {.size = l.size}});
        const bool raised = C4::raised();
        C4::clear_raised();
        print(serial, "  ", l.name, ": ", done ? "done" : "NOT DONE", ", count ", C4::count(), ", raw ",
              raised ? "raised" : "not raised", " then ", C4::raised() ? "STILL raised" : "cleared", crlf);
        bench.verdict(l.name, ": the copy is exact, TRANS_COUNT at zero, the raw status raised and "
                      "cleared by writing one",
                      done && same(src, dst, l.bytes) && C4::count() == 0u && raised && !C4::raised());
    }
    fill(src, 4096, 0x5A5Au);
    fill(dst, 4096, 0);
    const uint32_t t0 = us_now();
    const bool done = run_c4({.read = src, .write = dst, .count = 1024, .config = {.size = DmaSize::word}});
    const uint32_t took = us_now() - t0;
    C4::clear_raised();
    print(serial, "  4096 bytes as 1024 words in ", took, " us (8.2 us would be one transfer a cycle at "
          "125 MHz; the read and the write masters share the SRAM with this core's polling)", crlf);
    bench.verdict("four kilobytes of words move exact in under 40 us", done && same(src, dst, 4096) && took < 40u);
}

// -----------------------------------------------------------------------------
void tc_lines() {
    fill(src, 64, 0x77u);
    const uint32_t l0 = ch4_on_line0;
    const uint32_t l1 = ch4_on_line1;
    C4::route(0, true);
    (void)run_c4({.read = src, .write = dst, .count = 64});
    spin_us(50);
    print(serial, "  routed to line 0: line-0 handler +", ch4_on_line0 - l0, ", line-1 handler +",
          ch4_on_line1 - l1, crlf);
    bench.verdict("a completion routed to line 0 is served on line 0 alone, once",
                  ch4_on_line0 == l0 + 1u && ch4_on_line1 == l1 && !C4::pending(0));
    C4::route(0, false);
    C4::route(1, true);
    (void)run_c4({.read = src, .write = dst, .count = 64});
    spin_us(50);
    print(serial, "  routed to line 1: line-0 handler +", ch4_on_line0 - l0 - 1u, ", line-1 handler +",
          ch4_on_line1 - l1, crlf);
    bench.verdict("the same channel routed to line 1 is served on line 1 alone, once",
                  ch4_on_line0 == l0 + 1u && ch4_on_line1 == l1 + 1u && !C4::pending(1));
    C4::route(1, false);
    C4::route(0, true);
    const uint32_t q0 = ch4_on_line0;
    (void)run_c4({.read = src, .write = dst, .count = 64, .config = {.irq_quiet = true}});
    spin_us(50);
    const bool quiet_silent = ch4_on_line0 == q0 && !C4::raised();
    C4::null_trigger();
    spin_us(50);
    print(serial, "  IRQ_QUIET: the completion raised ", ch4_on_line0 - q0, " interrupt(s), the null trigger after it ",
          ch4_on_line0 - q0, " in all", crlf);
    bench.verdict("under IRQ_QUIET a completion is silent and a null trigger raises exactly one",
                  quiet_silent && ch4_on_line0 == q0 + 1u);
    C4::stop();
}

// -----------------------------------------------------------------------------
void td_chain() {
    fill(src, 512, 0xC4C5u);
    fill(dst, 512, 0);
    C5::stop();
    const bool prepared = C5::prepare({.read = src + 256, .write = dst + 256, .count = 256});
    const bool started = C4::load({.read = src, .write = dst, .count = 256, .config = {.chain_to = 5}});
    const uint32_t t0 = us_now();
    while ((C4::busy() || C5::busy() || C5::count() != 0u) && us_now() - t0 < 10'000u) {
    }
    print(serial, "  channel 4 chained to 5: 4 raised=", C4::raised(), " 5 raised=", C5::raised(),
          ", 5's count ", C5::count(), ", both halves ", same(src, dst, 512) ? "exact" : "WRONG", crlf);
    bench.verdict("the chained channel, prepared and waiting, ran at the first one's completion: both "
                  "halves exact, both raised",
                  prepared && started && C4::raised() && C5::raised() && same(src, dst, 512));
    C4::stop();
    C5::stop();
}

// -----------------------------------------------------------------------------
void te_ring() {
    fill(src, 16, 0xABu);
    fill(dst, 64, 0);
    const bool done = run_c4({.read = src, .write = dst, .count = 64, .config = {.ring_bits = 4}});
    bool four_times = true;
    for (uint32_t i = 0; i < 64u; ++i) {
        four_times = four_times && dst[i] == src[i & 15u];
    }
    print(serial, "  a sixteen-byte ring read into sixty-four bytes: ", four_times ? "the pattern four times" : "WRONG",
          crlf);
    bench.verdict("RING_SIZE wraps the read address every sixteen bytes", done && four_times);
    C4::stop();
}

// -----------------------------------------------------------------------------
void tf_pacing() {
    fill(src, 1000, 0x11u);
    DmaTimer<0>::set(1, 125);   // clk_sys / 125 = 1 MHz
    const uint32_t t0 = us_now();
    const bool done = run_c4({.read = src, .write = dst, .count = 1000, .config = {.treq = DmaTimer<0>::dreq()}});
    const uint32_t took = us_now() - t0;
    DmaTimer<0>::stop();
    print(serial, "  a thousand bytes at one request a microsecond: ", took, " us", crlf);
    bench.verdict("the pacing timer at X/Y = 1/125 moves one byte a microsecond, within 10 %",
                  done && same(src, dst, 1000) && took >= 990u && took <= 1100u);
    C4::stop();
}

// -----------------------------------------------------------------------------
uint32_t crc32_msb(const uint8_t* p, uint32_t n, uint32_t crc) {
    for (uint32_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint32_t>(p[i]) << 24;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80000000u) != 0u ? (crc << 1) ^ 0x04C11DB7u : crc << 1;
        }
    }
    return crc;
}
uint32_t reverse32(uint32_t v) {
    uint32_t r = 0;
    for (int i = 0; i < 32; ++i) {
        r = (r << 1) | (v & 1u);
        v >>= 1;
    }
    return r;
}

void tg_sniffer() {
    fill(src, 256, 0xCAFEu);
    uint32_t sum = 0;
    for (uint32_t i = 0; i < 256u; ++i) {
        sum += src[i];
    }
    DmaSniffer::start(4, {.calc = DmaSniffCalc::sum}, 0);
    (void)run_c4({.read = src, .write = dst, .count = 256, .config = {.sniff = true}});
    const uint32_t got_sum = DmaSniffer::result();
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc16}, 0xFFFFu);
    (void)run_c4({.read = src, .write = dst, .count = 256, .config = {.sniff = true}});
    const uint32_t got_crc16 = DmaSniffer::result() & 0xFFFFu;
    const uint16_t want_crc16 = crc16(src, 256);
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc32}, 0xFFFFFFFFu);
    (void)run_c4({.read = src, .write = dst, .count = 256, .config = {.sniff = true}});
    const uint32_t got_crc32 = DmaSniffer::result();
    const uint32_t want_crc32 = crc32_msb(src, 256, 0xFFFFFFFFu);
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc32, .out_reverse = true, .out_invert = true}, 0xFFFFFFFFu);
    (void)run_c4({.read = src, .write = dst, .count = 256, .config = {.sniff = true}});
    const uint32_t got_rev_inv = DmaSniffer::result();
    DmaSniffer::stop();
    print(serial, "  sum ", got_sum, " (", sum, "), crc16 ", hex(got_crc16), " (", hex(want_crc16), "), crc32 ",
          hex(got_crc32), " (", hex(want_crc32), "), reversed+inverted ", hex(got_rev_inv), " (",
          hex(~reverse32(want_crc32)), ")", crlf);
    bench.verdict("the sniffer's sum is the bytes' sum", got_sum == sum);
    bench.verdict("its CRC-16 is util/crc.hpp's CRC-16-CCITT at the same seed", got_crc16 == want_crc16);
    bench.verdict("its CRC-32 is the MSB-first 0x04C11DB7 polynomial from the seed given",
                  got_crc32 == want_crc32);
    bench.verdict("OUT_REV and OUT_INV present the same result bit-reversed and inverted",
                  got_rev_inv == ~reverse32(want_crc32));
    C4::stop();
}

// -----------------------------------------------------------------------------
void th_bus_error() {
    fill(dst, 64, 0);
    C4::route(0, true);
    const uint32_t l0 = ch4_on_line0;
    const auto* hole = reinterpret_cast<const volatile void*>(0x40100000u);   // no peripheral answers here
    const bool started = C4::load({.read = hole, .write = dst, .count = 16, .config = {.size = DmaSize::word}});
    const uint32_t t0 = us_now();
    while (C4::busy() && us_now() - t0 < 10'000u) {
    }
    spin_us(50);
    const uint32_t errors = C4::errors();
    print(serial, "  a word block reading 0x40100000: busy=", C4::busy(), " count=", C4::count(),
          " errors=", hex(errors), " (AHB ", (errors & DmaError::ahb) != 0u, ", read ",
          (errors & DmaError::read) != 0u, ", write ", (errors & DmaError::write) != 0u,
          "), line-0 handler +", ch4_on_line0 - l0, ", dst[0]=", hex(dst[0]), crlf);
    bench.verdict("the block ended one way or the other and the channel came back", started && !C4::busy());
    C4::clear_errors();
    bench.verdict("the error bits clear by writing one", C4::errors() == 0u);
    fill(src, 64, 0x42u);
    const bool again = run_c4({.read = src, .write = dst, .count = 64});
    bench.verdict("and the channel copies again afterwards", again && same(src, dst, 64));
    C4::stop();
}

// -----------------------------------------------------------------------------
void ti_instrument() {
    const bool up = Instrument::init(clock, 3'000'000) && Instrument::loopback(true);
    bench.verdict("the instrument comes up with an engine in each slot, on its loop-back", up);
    uint32_t tx_state = 0x12345678u;
    uint32_t rx_state = 0x12345678u;
    auto next = [](uint32_t& s) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return static_cast<uint8_t>(s);
    };
    const uint32_t irqs0 = dma0_irqs;
    uint32_t sent = 0;
    uint32_t good = 0;
    bool wrong = false;
    const uint32_t t0 = us_now();
    while (good < 4096u && !wrong && us_now() - t0 < 500'000u) {
        while (sent < 4096u) {
            uint8_t chunk[128];
            uint32_t n = 0;
            uint32_t peek = tx_state;
            while (n < sizeof chunk && sent + n < 4096u) {
                chunk[n++] = next(peek);
            }
            const uint32_t queued = Instrument::write_bulk(std::span<const uint8_t>(chunk, n));
            for (uint32_t i = 0; i < queued; ++i) {
                (void)next(tx_state);
            }
            sent += queued;
            if (queued < n) {
                break;
            }
        }
        (void)Instrument::harvest();
        uint8_t got[128];
        const uint32_t n = Instrument::read_bulk(got);
        for (uint32_t i = 0; i < n && !wrong; ++i) {
            if (got[i] != next(rx_state)) {
                wrong = true;
            } else {
                ++good;
            }
        }
    }
    const uint32_t took = us_now() - t0;
    const uint32_t irqs = dma0_irqs - irqs0;
    if (wrong) {
        uint8_t got[8] = {};
        (void)Instrument::read_bulk(got);
        print(serial, "  MISMATCH after ", good, " good bytes: sent ", sent, ", the next received ", hex(got[0]),
              " ", hex(got[1]), " ", hex(got[2]), " ", hex(got[3]), crlf);
    }
    print(serial, "  4096 bytes through UART1's loop-back on two engines at 3 Mbaud: ", good, " exact in ", took,
          " us, ", irqs, " line-0 interrupts (the console's blocks among them), faults ",
          Instrument::dma_faults(), ", errors FE ", Instrument::frame_errors(), " OE ",
          Instrument::hw_overruns(), " ring ", Instrument::rx_overruns(), crlf);
    bench.verdict("every byte the transmit engine poured, the receive engine harvested, in order",
                  good == 4096u && !wrong);
    bench.verdict("no fault, no error, no overrun", Instrument::dma_faults() == 0u &&
                                                        Instrument::frame_errors() == 0u &&
                                                        Instrument::hw_overruns() == 0u &&
                                                        Instrument::rx_overruns() == 0u);
    bench.verdict("the wire's time is what it took (13653 us of frames), within 10 %",
                  took >= 13'000u && took <= 15'100u);
    Instrument::release();
}

// -----------------------------------------------------------------------------
void tj_console_engine() {
    const uint32_t blocks = serial_blocks;
    console_drain();
    const uint32_t b0 = serial_blocks;
    // A burst four times the ring: print() spins on the ring while the
    // engine drains it block by block.
    for (uint32_t line = 0; line < 8u; ++line) {
        print(serial, "  ");
        for (uint32_t i = 0; i < 25u; ++i) {
            print(serial, "0123456789");
        }
        print(serial, crlf);
    }
    console_drain();
    const bool idle = Serial::tx_idle();
    const uint32_t used = serial_blocks - b0;
    print(serial, "  2064 bytes of burst went out in ", used, " block(s) of the console's engine (", blocks,
          " before this letter), faults ", Serial::dma_faults(), ", idle after the drain: ", idle, crlf);
    bench.verdict("the burst left in more than one block and the transport was idle again after it",
                  used > 1u && idle);
    bench.verdict("no fault on the console's engine", Serial::dma_faults() == 0u);
}

// Diagnostic halves: one engine at a time on the same loop-back.
using TxOnly = Uart<1, instrument_pins, 1024, 1024, DmaTxEngine<6>, NoDmaEngine>;
using RxOnly = Uart<1, instrument_pins, 1024, 1024, NoDmaEngine, DmaRxEngine<7>>;

// UART1's vector goes to whichever transport owns the instance now.
using IsrFn = bool (*)();
volatile IsrFn uart1_isr = nullptr;

template <typename T>
void half_loop(const char* name) {
    uart1_isr = &T::isr;
    const bool up = T::init(clock, 3'000'000) && T::loopback(true);
    uint8_t out[512];
    fill(out, 512, 0x3131u);
    uint8_t in[512] = {};
    uint32_t got = 0;
    const uint32_t t0 = us_now();
    (void)T::write_bulk(out);
    while (got < 512u && us_now() - t0 < 100'000u) {
        (void)T::harvest();
        got += T::read_bulk(std::span<uint8_t>(in + got, 512u - got));
    }
    print(serial, "  ", name, ": up=", up, " got ", got, " in ", us_now() - t0, " us, exact=", same(out, in, got),
          "; first ", hex(in[0]), " ", hex(in[1]), " ", hex(in[2]), " (", hex(out[0]), " ", hex(out[1]), " ",
          hex(out[2]), "); rx ch7 count ", DmaChannel<7>::count(), " busy ", DmaChannel<7>::busy(),
          " credits ", DmaChannel<7>::debug_credits(), " raw ", DmaChannel<7>::raised(), "; tx ch6 count ",
          DmaChannel<6>::count(), " busy ", DmaChannel<6>::busy(), "; UART FR ", hex(T::Resource::flags()),
          " DMACR ", hex(T::Resource::regs().UARTDMACR), crlf);
    bench.verdict(name, ": 512 bytes back exact", got == 512u && same(out, in, 512));
    T::release();
    uart1_isr = nullptr;
}
void tk_tx_only() { half_loop<TxOnly>("the transmit engine alone, the receiver on its interrupt"); }
void tl_rx_only() { half_loop<RxOnly>("the receive engine alone, the transmitter on its interrupt"); }

/// tx_idle() under the transmit engine, on the pad: eight frames of 0xFF
/// at 9600 baud on GP4 with the loop-back off, the pad read through SIO's
/// GPIO_IN - one falling edge a frame, its start bit - and tx_idle() (the
/// ring empty, no block in flight, UARTFR.BUSY clear) turning true at the
/// last stop bit's end, ten bit times after the last edge.
void tt_tx_idle_engine() {
    constexpr uint32_t bit_us = 1'000'000u / 9600u;
    uart1_isr = &TxOnly::isr;
    const bool up = TxOnly::init(clock, 9600);
    spin_us(5000);
    static const uint8_t ones[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    bool level = Pin<4>::read();
    uint32_t falls = 0;
    uint32_t last_fall = 0;
    uint32_t idle_at = 0;
    bool idle_seen = false;
    (void)TxOnly::write_bulk(ones);
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 12u * 10u * bit_us) {
        const uint32_t t = us_now();
        const bool now_level = Pin<4>::read();
        if (level && !now_level) {
            last_fall = t;
            ++falls;
        }
        level = now_level;
        if (!idle_seen && TxOnly::tx_idle()) {
            idle_at = t;
            idle_seen = true;
        }
    }
    const int32_t after = static_cast<int32_t>(idle_at - (last_fall + 10u * 1'000'000u / 9600u));
    print(serial, "  8 frames of 0xFF at 9600 on GP4 through the transmit engine: ", falls,
          " start bits; tx_idle() ", idle_seen ? "" : "NEVER ", "true ", after,
          " us from the last stop bit's end (a bit is ", bit_us, " us)", crlf);
    bench.verdict("tx_idle() IS THE WIRE'S under the transmit engine: never before the last "
                  "stop bit's end, and within a bit time after it",
                  up && idle_seen && falls == 8u && after >= -2 &&
                      after <= static_cast<int32_t>(bit_us));
    TxOnly::release();
    uart1_isr = nullptr;
}

// -----------------------------------------------------------------------------
using Copier = DmaCopyEngine<8>;
using Beat = DmaTxEngine<9, uint32_t>;
alignas(4) volatile uint32_t beat_cell = 0;
volatile uint32_t beat_blocks = 0;

/// The copy engine's block waited out, bounded.
bool copier_done() {
    const uint32_t t0 = us_now();
    while (Copier::busy() && us_now() - t0 < 10'000u) {
    }
    return !Copier::busy() && Copier::errors() == 0u;
}
/// One block of the beat engine waited out on its completion count.
bool beat_done(uint32_t before) {
    const uint32_t t0 = us_now();
    while (beat_blocks == before && us_now() - t0 < 10'000u) {
    }
    return beat_blocks != before;
}
/// The beat engine's channel's DATA_SIZE field, read back.
uint32_t beat_size() {
    return (DmaChannel<Beat::channel>::ctrl() & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
}

void tm_engines() {
    Copier::arm(DmaReport::errors);   // polled: the channel on no line
    auto* const s32 = reinterpret_cast<uint32_t*>(src);
    auto* const d32 = reinterpret_cast<uint32_t*>(dst);
    fill(src, 4096, 0xC0DEu);
    fill(dst, 4096, 0u);
    const bool w = Copier::copy(d32, s32, 1024u) && copier_done();
    bench.verdict("copy: 4096 bytes as 1024 words, exact", w && same(src, dst, 4096));

    fill(dst, 4096, 0u);
    const bool h = Copier::copy(reinterpret_cast<uint16_t*>(dst + 2), reinterpret_cast<const uint16_t*>(src + 2), 255u) &&
                   copier_done();
    bench.verdict("copy: 255 half-words, exact, the bytes either side untouched",
                  h && same(src + 2, dst + 2, 510) && dst[0] == 0u && dst[1] == 0u && dst[512] == 0u);

    fill(dst, 4096, 0u);
    const bool b = Copier::copy(dst + 1, src + 3, 1001u) && copier_done();
    bench.verdict("copy: 1001 bytes between odd addresses, exact, the bytes either side untouched",
                  b && same(src + 3, dst + 1, 1001) && dst[0] == 0u && dst[1002] == 0u);

    const uint32_t cell = 0xA5C30F96u;
    fill(dst, 4096, 0u);
    const bool f = Copier::fill(d32, &cell, 256u) && copier_done();
    bool filled = true;
    for (uint32_t i = 0; i < 256u; ++i) {
        filled = filled && d32[i] == cell;
    }
    bench.verdict("fill: 256 words from one cell, exact, the word after untouched", f && filled && d32[256] == 0u);

    const bool misaligned = !Copier::copy(reinterpret_cast<uint16_t*>(dst + 1), reinterpret_cast<const uint16_t*>(src), 4u);
    const bool empty = !Copier::copy(d32, s32, 0u);
    const bool first = Copier::copy(d32, s32, 1024u);
    const bool second = Copier::copy(d32, s32, 4u);
    (void)copier_done();
    bench.verdict("refused: a half-word run at an odd address, an empty run, a block over a running one",
                  misaligned && empty && first && !second);
    Copier::stop();

    // the beat per block, one engine
    Beat::arm(&beat_cell, Dreq::permanent);
    alignas(4) static const uint8_t bytes[4] = {0x11u, 0x22u, 0x33u, 0x44u};
    alignas(4) static const uint16_t halves[2] = {0x5566u, 0x7788u};
    alignas(4) static const uint32_t words[1] = {0x99AABBCCu};
    beat_cell = 0xEEEEEEEEu;
    uint32_t n0 = beat_blocks;
    const bool b8 = Beat::start(std::span<const uint8_t>(bytes)) && beat_done(n0);
    const uint32_t after8 = beat_cell;
    const uint32_t size8 = beat_size();
    n0 = beat_blocks;
    const bool b16 = Beat::start(std::span<const uint16_t>(halves)) && beat_done(n0);
    const uint32_t after16 = beat_cell;
    const uint32_t size16 = beat_size();
    n0 = beat_blocks;
    const bool b32 = Beat::start(std::span<const uint32_t>(words)) && beat_done(n0);
    const uint32_t after32 = beat_cell;
    const uint32_t size32 = beat_size();
    print(serial, "  one word cell, three runs: ", hex(after8), " (DATA_SIZE ", size8, "), ", hex(after16),
          " (", size16, "), ", hex(after32), " (", size32, ")", crlf);
    bench.verdict("the beat follows the run: a byte run writes the cell's low byte, a half-word run its "
                  "low half, a word run the whole word, DATA_SIZE 0, 1, 2",
                  b8 && b16 && b32 && after8 == 0xEEEEEE44u && after16 == 0xEEEE7788u && after32 == 0x99AABBCCu &&
                      size8 == 0u && size16 == 1u && size32 == 2u);
    const bool odd = !Beat::start(reinterpret_cast<const uint16_t*>(bytes + 1), 1u);
    bench.verdict("a half-word run at an odd address is refused and the claim given back", odd && !Beat::busy());
    Beat::stop();
}

/// K BREAKS IN N SLOTS on UART1's loop-back at 115200: 64 slots, four of
/// them a break (UARTLCR_H.BRK held two frames) instead of their byte.
/// Sent through the transmit engine; received by the interrupt receiver
/// (TxOnly) and by the receive engine (Instrument), the stream judged
/// against the 60 bytes that were sent.
template <typename T>
uint32_t breaks_through(uint8_t* got, uint32_t room, uint32_t& kept) {
    constexpr uint32_t frame_us = 10u * 1'000'000u / 115200u + 1u;
    const bool up = T::init(clock, 115200) && T::loopback(true);
    spin_us(20 * frame_us);
    uint8_t junk[32];
    (void)T::harvest();
    while (T::read_bulk(junk) != 0u) {
    }
    T::clear_errors();
    kept = 0;
    for (uint32_t i = 0; i < 64u; ++i) {
        if (i == 9u || i == 24u || i == 41u || i == 58u) {
            const uint32_t t0 = us_now();
            while (!T::tx_idle() && us_now() - t0 < 100u * frame_us) {
            }
            T::Resource::break_send(true);
            spin_us(2u * frame_us);
            T::Resource::break_send(false);
            spin_us(frame_us);
            continue;
        }
        const uint8_t b = static_cast<uint8_t>(0x40u + i);
        while (T::write_bulk({&b, 1}) == 0u) {
        }
        ++kept;
    }
    spin_us(80u * frame_us);
    (void)T::harvest();
    uint32_t n = 0;
    while (n < room) {
        const uint32_t k = T::read_bulk({got + n, room - n});
        if (k == 0u) {
            break;
        }
        n += k;
    }
    return up ? n : 0u;
}

void tv_breaks() {
    static uint8_t got[128];
    uint32_t kept = 0;
    // The interrupt receiver: each break an entry flagged BE and FE, dropped.
    uart1_isr = &TxOnly::isr;
    const uint32_t n_irq = breaks_through<TxOnly>(got, sizeof got, kept);
    uint32_t wrong = 0;
    for (uint32_t i = 0, j = 0; i < 64u; ++i) {
        if (i == 9u || i == 24u || i == 41u || i == 58u) {
            continue;
        }
        if (j >= n_irq || got[j] != static_cast<uint8_t>(0x40u + i)) {
            ++wrong;
        }
        ++j;
    }
    const uint32_t be = TxOnly::break_errors();
    print(serial, "  64 slots, 4 breaks, the interrupt receiver: ", n_irq, " of ", kept,
          " bytes back, ", wrong, " wrong, BE ", be, " FE ", TxOnly::frame_errors(), crlf);
    bench.verdict("K BREAKS IN N SLOTS, the interrupt receiver: N - K bytes intact and in "
                  "order, each break its own entry, counted and dropped",
                  n_irq == kept && wrong == 0u && be == 4u);
    TxOnly::release();
    uart1_isr = nullptr;

    // The receive engine: the channel moves the BYTE of every entry - a
    // break's zero included, its flags living in bits the byte beat drops
    // - the error interrupt counts each, and no clear reads UARTDR, so
    // every byte sent is in the stream, in order.
    uart1_isr = &Instrument::isr;
    const uint32_t n_dma = breaks_through<Instrument>(got, sizeof got, kept);
    uint32_t matched = 0;
    uint32_t zeros = 0;
    for (uint32_t j = 0, i = 0; j < n_dma; ++j) {
        while (i < 64u && (i == 9u || i == 24u || i == 41u || i == 58u)) {
            ++i;
        }
        if (i < 64u && got[j] == static_cast<uint8_t>(0x40u + i)) {
            ++matched;
            ++i;
        } else if (got[j] == 0u) {
            ++zeros;
        }
    }
    const uint32_t be_dma = Instrument::break_errors();
    print(serial, "  the same under the receive engine: ", n_dma, " bytes back, the ", matched,
          " of ", kept, " sent among them in order, ", zeros, " break zeros, BE ", be_dma,
          " FE ", Instrument::frame_errors(), crlf);
    bench.verdict("AND UNDER THE ENGINE NO CLEAR TAKES A BYTE: every byte sent is in the "
                  "stream and in order, each break counted once by the error interrupt; "
                  "each break is also the zero byte of its entry, which a byte beat cannot "
                  "tell from data (docs/pl011/README.md)",
                  matched == kept && zeros == 4u && n_dma == kept + 4u && be_dma == 4u);
    Instrument::release();
    uart1_isr = nullptr;
}

void banner() {
    print(serial, crlf, "test_rp2040_dma - the RP2040 DMA (datasheet 2.5), this console on a transmit engine, "
          "clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_uart1() {
    if (uart1_isr != nullptr) {
        (void)uart1_isr();
    }
}
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_dma_0() {
    dma0_irqs = dma0_irqs + 1u;
    if (C4::pending(0)) {
        C4::clear_pending(0);
        ch4_on_line0 = ch4_on_line0 + 1u;
    }
    if (brio::DmaChannel<0>::pending(0)) {
        serial_blocks = serial_blocks + 1u;   // counted before the transport clears it
    }
    (void)Serial::dma_isr();
    (void)Instrument::dma_isr();
    (void)TxOnly::dma_isr();
    (void)RxOnly::dma_isr();
    if ((Beat::service() & Beat::flag_complete) != 0u) {
        (void)Beat::complete();
        beat_blocks = beat_blocks + 1u;
    }
}
extern "C" void isr_dma_1() {
    dma1_irqs = dma1_irqs + 1u;
    if (C4::pending(1)) {
        C4::clear_pending(1);
        ch4_on_line1 = ch4_on_line1 + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::DmaLine<0>::enable();
    brio::DmaLine<1>::enable();
    brio::enable_interrupts();

    bench.letter('a', "the block as found, the refusals, a stalled channel aborted", ta_as_found);
    bench.letter('b', "memory to memory at three widths, and timed", tb_memory);
    bench.letter('c', "the two lines, IRQ_QUIET, the null trigger", tc_lines);
    bench.letter('d', "chaining", td_chain);
    bench.letter('e', "address wrapping", te_ring);
    bench.letter('f', "a pacing timer", tf_pacing);
    bench.letter('g', "the sniffer: sum, CRC-16, CRC-32", tg_sniffer);
    bench.letter('h', "a bus error", th_bus_error);
    bench.letter('i', "the instrument's two engines on the loop-back", ti_instrument);
    bench.letter('j', "this console's transmit engine under a burst", tj_console_engine);
    bench.letter('k', "diagnostic: the transmit engine alone", tk_tx_only, false);
    bench.letter('l', "diagnostic: the receive engine alone", tl_rx_only, false);
    bench.letter('m', "the copy engine, and the beat per block", tm_engines);
    bench.letter('t', "tx_idle() on the pad under the transmit engine", tt_tx_idle_engine);
    bench.letter('v', "K breaks in N slots: the interrupt receiver and the engine", tv_breaks);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " dma=", dma_ok ? "released" : "FAILED",
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
