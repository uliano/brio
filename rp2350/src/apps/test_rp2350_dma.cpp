// test_rp2350_dma - the reference bench suite for the RP2350's DMA
// controller (rp2350/dma.hpp, datasheet 12.6): sixteen channels with
// their trigger aliases, the FOUR interrupt lines, chaining, address
// wrapping and the four address steps, THE COUNT MODES in TRANS_COUNT's
// top nibble, the pacing timers, the checksum sniffer, the abort with
// erratum RP2350-E5's workaround, a bus error - and the engines: the two
// in a UART's slots, this console's own transmit side among them, and on
// their own (letter l) the beat per block, a binding that reports errors
// alone, and the copy engine. ON BOTH ARCHITECTURES from one source.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// THE CONSOLE PRINTS THROUGH THE DMA: UART0's transport names a transmit
// engine on channel 0, so every character of every verdict below
// travelled by a block the engine started off the ring's run and released
// on the line-0 interrupt. Its receive side stays on the interrupt (a
// keystroke at a time wants no block). The INSTRUMENT is UART1 on GP4/GP5
// under the PL011's own loop-back (UARTCR.LBE, which ignores the RX pad),
// with an engine in EACH slot, channels 2 and 3. NOTHING TO WIRE.
//
// THE RULER IS THE PLATFORM TIMER (rp2350/mtime.hpp), the crystal's
// microseconds, the same on both halves.
//
// WHAT THIS CHIP ADDS TO THE RP2040'S SAME BLOCK, and which letters are
// for it: four more channels and TWO MORE INTERRUPT LINES (letter c walks
// all four), a MODE in the top four bits of TRANS_COUNT that makes a
// channel re-trigger itself or run forever (letter i), and an address
// step that can go BACKWARD or skip alternate cells (letter e). The two
// errata of the chapter are letters a (E5, the abort against a chain) and
// - by construction, since no verb here can start a zero-length sequence
// - E8, which is why letter a asks a zero count to be refused.
//
// What is exercised, letter by letter:
//   a  the block as found and the refusals: sixteen channels by the
//      silicon's own count, a channel idle at its reset word, a zero
//      count, a count past the 28 bits the mode leaves, a misaligned
//      word, a reserved mode, a chain to itself or to a seventeenth
//      channel refused; the security assignment READ BACK (every
//      resource SP, the console's own channel locked by its first
//      control write); then a channel STALLED on a pacing timer that
//      never requests (BUSY standing, every configuring verb refused),
//      aborted RP2350-E5's way - the ABORT register polled to zero, BUSY
//      down, nothing raised, the routes back
//   b  memory to memory at byte, half-word and word width: the copy
//      exact, TRANS_COUNT at zero, the raw status raised and cleared by
//      writing one, BUSY down; four kilobytes of words timed
//   c  THE FOUR LINES: a completion on channel 4 routed to each of them
//      in turn and counted in that line's handler alone, IRQ_QUIET
//      silencing a completion, and a null trigger under IRQ_QUIET
//      raising exactly one
//   d  chaining: channel 4 completes and triggers channel 5, prepared
//      and waiting - both copies exact, both raised
//   e  ADDRESS WRAPPING AND THE FOUR STEPS: a sixteen-byte source read
//      through a ring of sixteen fills sixty-four bytes four times over;
//      then the same run read BACKWARD (the copy reversed) and read by
//      TWOS (every other byte) - neither of which the RP2040 can do
//   f  a pacing timer: one transfer a microsecond at 150 MHz (X 1,
//      Y 150), a thousand bytes timed against the ruler
//   g  the sniffer: the sum, the CRC-16-CCITT and the CRC-32 of a buffer
//      against util/crc.hpp's own arithmetic; the reversed and inverted
//      result options
//   h  a bus error: a block reading SIO, which 12.6.7 says the DMA's bus
//      ports cannot see - what the silicon does is REPORTED (the error
//      bits, the halt, the interrupt), the verdict only that the channel
//      ended and came back clean
//   i  THE COUNT MODES: TRIGGER_SELF re-arming a paced ring lap after lap
//      with no software between them, each lap raising its interrupt and
//      reloading the count; then ENDLESS, whose count never decrements
//      and which raises nothing at all until it is aborted
//   j  the instrument's engines: 4096 bytes through UART1's loop-back at
//      3 Mbaud with an engine in each slot, harvested by the loop,
//      byte-exact, the line's interrupts counted against the bytes; then
//      1024 frames through the transmit engine timed against their wire
//      time at the rate ASKED - a loop shares one divisor at both ends and
//      cannot tell a wrong one
//   k  this console's transmit engine under a burst longer than its
//      ring: every byte out in order (the host reads it), the blocks
//      counted, no fault
//   l  THE ENGINES' OWN SURFACE: one channel carrying a byte, a
//      half-word and a word block in turn (the beat per block, CTRL
//      rewritten only when it changes), a binding that reports ERRORS
//      ALONE - a clean block raising nothing, a block reading the SIO
//      window raising its interrupt all the same (12.6.7.1), which is
//      what lets an SPI host take one interrupt a transaction - and the
//      copy engine: copy and fill at three beats, exact, the refusals
//
//   t  tx_idle() ON THE PAD under the transmit engine: eight frames on
//      GP4 with the loop-back off, the pad read through SIO, tx_idle()
//      turning true at the last stop bit's end, and the start bits seven
//      frames apart at the rate asked
//
//   v  K BREAKS IN N SLOTS on UART1's loop-back: 64 slots, four of them
//      a break, through the interrupt receiver (60 delivered, each break
//      an entry counted and dropped, the consumer looking once a slot as
//      the receive ring's skip wants) and through the receive engine
//      (every byte sent delivered in order, the breaks' zero bytes among
//      them, each break counted once by the error interrupt); each line
//      timed against the wire first
//   o  THE OVERRUN A STALLED RECEIVE ENGINE LEAVES: three episodes of a
//      ring nobody reads, the FIFO full behind the idle channel and four
//      thousand frames lost on it - each episode one UART1 entry and one
//      overrun counted, the vector quiet after it
//   r  A RELEASED REQUESTER (the release contract, no wire): UART1's and
//      SPI1's transmit, PWM slice 3's wrap and the ADC's FIFO, each
//      released six ways and the next owner's channel watched - what
//      outlives a release, at the peripheral and in the channel
//   p  (by name only, brio stress) THE CONSOLE'S RING BESIDE COPIES: the
//      host's stream into a receive engine at 115200, 1 Mbaud and
//      3 Mbaud, two 16 KB copies back to back beside it, the ring and the
//      copies at each level - every byte judged; on this chip a ring at
//      normal beside two HIGH_PRIORITY copies is starved, which the letter
//      reports and the transport's level forbids
//
//   m  (by name only) diagnostic: the transmit engine alone on the loop
//   n  (by name only) diagnostic: the receive engine alone on the loop
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2350/adc.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/pwm.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/crc.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P = Rp2350Platform<>;

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
using InstrumentRx = DmaRxEngine<3>;
using Instrument = Uart<1, instrument_pins, 1024, 1024, DmaTxEngine<2>, InstrumentRx>;


TestBench<Serial, 20> bench;

volatile uint32_t line_irqs[4] = {0, 0, 0, 0};
volatile uint32_t ch4_on_line[4] = {0, 0, 0, 0};
volatile uint32_t serial_blocks = 0;

// Letter i's lap counter: channel 5 in TRIGGER_SELF mode re-arms itself,
// so nothing but the handler knows how many laps have gone by - and the
// handler is what stops it, by clearing EN at the fourth.
volatile uint32_t self_laps = 0;
volatile bool self_running = false;

uint32_t us_now() { return Mtime::micros(); }

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

/// THE RATE ON THE WIRE, which a loop alone cannot tell: both ends share
/// one divisor, so a wrong one is a slower or a faster loop and nothing
/// else. `frames` frames of `bits` bit periods queued back to back, timed
/// on the ruler from the first write to tx_idle() - the last stop bit's
/// end (docs/pl011/README.md) - against their time at the rate ASKED; the
/// answer in thousandths. What the loop brings back is taken and thrown
/// away meanwhile, and the error counters are cleared after it.
template <typename Port>
uint32_t wire_permille(uint32_t baud, uint32_t bits = 10u, uint32_t frames = 1024u) {
    static uint8_t out[256];
    for (uint32_t i = 0; i < sizeof out; ++i) {
        out[i] = static_cast<uint8_t>(i * 37u + 11u);
    }
    uint8_t sink[64];
    const uint64_t wire_us = static_cast<uint64_t>(frames) * bits * 1'000'000u / baud;
    const uint32_t budget = static_cast<uint32_t>(wire_us * 2u) + 20'000u;
    uint32_t queued = 0;
    const uint32_t t0 = us_now();
    while (queued < frames && us_now() - t0 < budget) {
        const uint32_t want = frames - queued;
        queued += Port::write_bulk(std::span<const uint8_t>(out, want < sizeof out ? want : sizeof out));
        (void)Port::harvest();
        while (Port::read_bulk(sink) != 0u) {
        }
    }
    while (!Port::tx_idle() && us_now() - t0 < budget) {
        (void)Port::harvest();
        while (Port::read_bulk(sink) != 0u) {
        }
    }
    const uint32_t took = us_now() - t0;
    spin_us(static_cast<uint32_t>(64ull * bits * 1'000'000u / baud) + 1000u);
    (void)Port::harvest();
    while (Port::read_bulk(sink) != 0u) {
    }
    Port::clear_errors();
    return wire_us == 0u ? 0u : static_cast<uint32_t>(1000ull * took / wire_us);
}

/// The verdict on wire_permille's answer: never faster than the rate
/// asked by more than the divider's rounding and the ruler's tick, never
/// slower by more than three per cent.
bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

// The buffers every letter copies between: the source aligned to 64 so
// that letter e's rings (sixteen bytes) and letter i's (sixty-four) wrap
// inside their own windows, and both aligned for the word width.
alignas(64) uint8_t src[4096];
alignas(64) uint8_t dst[4096];

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

const char* security_name(DmaSecurity s) {
    switch (s) {
        case DmaSecurity::sp: return "SP";
        case DmaSecurity::su: return "SU";
        case DmaSecurity::nsp: return "NSP";
        default: return "NSU";
    }
}

// =============================================================================
// a - the block as found, the refusals, the security read-back, an abort
// =============================================================================
void ta_as_found() {
    print(serial, "  N_CHANNELS=", Dma::channels(), ", block ",
          Dma::released() ? "released" : "IN RESET", ", channel 4: enabled=", C4::enabled(),
          " busy=", C4::busy(), " errors=", hex(C4::errors()), crlf);
    bench.verdict("the silicon counts sixteen channels, four more than the RP2040's",
                  Dma::channels() == 16u);
    bench.verdict("channel 4 idles clean after the block's reset",
                  !C4::busy() && C4::errors() == 0u && !C4::raised());

    bench.verdict("a zero count is refused - which is also erratum RP2350-E8 out of reach, "
                  "a zero-length sequence being the one whose CHAIN_TO the silicon gets wrong",
                  !C4::load({.read = src, .write = dst, .count = 0}) && !C4::set_count(0));
    bench.verdict("a count past the 28 bits the mode field leaves is refused",
                  !C4::set_count(dma_count_max + 1u) &&
                      !C4::load({.read = src, .write = dst, .count = dma_count_max + 1u}));
    bench.verdict("a word transfer from an odd address is refused",
                  !C4::load({.read = src + 1, .write = dst, .count = 4,
                             .config = {.size = DmaSize::word}}));
    bench.verdict("a reserved count mode is refused",
                  !C4::set_count(4, static_cast<DmaCountMode>(7)) &&
                      !C4::load({.read = src, .write = dst, .count = 4,
                                 .mode = static_cast<DmaCountMode>(2)}));
    bench.verdict("a chain to the channel itself and to a seventeenth channel are refused, "
                  "and a chain to the sixteenth is not",
                  !C4::configure({.chain_to = 4}) && !C4::configure({.chain_to = 16}) &&
                      C4::configure({.chain_to = 15}));

    // THE SECURITY ASSIGNMENT, read and never written (12.6.6): every
    // resource is SP out of reset, and a write to a channel's control
    // registers locks that channel's level - which the console's own
    // engine did at init(), before this letter ran.
    print(serial, "  security: channel 0 ", security_name(Dma::channel_security(0)), "/",
          Dma::channel_security_locked(0) ? "locked" : "open", ", channel 4 ",
          security_name(Dma::channel_security(4)), ", lines ",
          security_name(Dma::line_security(0)), " ", security_name(Dma::line_security(3)),
          ", sniffer ", security_name(Dma::sniffer_security()), ", timer 0 ",
          security_name(Dma::timer_security(0)), "; MPU global ",
          security_name(DmaMpu::global_level()), ", region 0 ",
          DmaMpu::region(0).enabled ? "ENABLED" : "off", crlf);
    bool mpu_off = true;
    for (uint8_t i = 0; i < DmaMpu::regions; ++i) {
        mpu_off = mpu_off && !DmaMpu::region(i).enabled;
    }
    bench.verdict("every channel, every line, every timer and the sniffer are SECURE AND "
                  "PRIVILEGED out of reset, and no MPU region is enabled - which is why a "
                  "program that runs entirely Secure is never refused by any of it",
                  Dma::channel_security(0) == DmaSecurity::sp &&
                      Dma::channel_security(15) == DmaSecurity::sp &&
                      Dma::line_security(0) == DmaSecurity::sp &&
                      Dma::line_security(3) == DmaSecurity::sp &&
                      Dma::sniffer_security() == DmaSecurity::sp &&
                      Dma::timer_security(0) == DmaSecurity::sp &&
                      Dma::timer_security(3) == DmaSecurity::sp && mpu_off);
    bench.verdict("and a channel that has been programmed has LOCKED its own assignment: the "
                  "console's transmit engine owns channel 0",
                  Dma::channel_security_locked(0));

    // STALLED: timer 0 requests nothing, so the block never moves.
    DmaTimer<0>::set(0, 1);
    C4::route(0, true);
    const bool started = C4::load({.read = src, .write = dst, .count = 16,
                                   .config = {.treq = DmaTimer<0>::dreq()}});
    spin_us(100);
    const bool stalled = C4::busy() && C4::count() == 16u;
    const bool refused = !C4::configure({}) && !C4::set_count(8) &&
                         !C4::load({.read = src, .write = dst, .count = 8});
    print(serial, "  stalled on a silent pacing timer: busy=", C4::busy(), " count=", C4::count(),
          " mode=", static_cast<uint8_t>(C4::mode()), ", credits ", C4::debug_credits(),
          ", reload ", C4::debug_reload(), crlf);
    bench.verdict("a channel waiting on a request that never comes stays BUSY at its full "
                  "count, and every configuring verb refuses it",
                  started && stalled && refused);
    const uint32_t irqs_before = ch4_on_line[0];
    const uint32_t t0 = us_now();
    const bool aborted = C4::abort();
    const uint32_t abort_us = us_now() - t0;
    spin_us(100);
    print(serial, "  aborted (RP2350-E5's sequence, the ABORT register polled to zero in ",
          abort_us, " us): busy=", C4::busy(), " raised=", C4::raised(), " pending0=",
          C4::pending(0), " routed0=", C4::routed(0), " CHAN_ABORT=", hex(Dma::abort_pending()),
          " line-0 handler runs +", ch4_on_line[0] - irqs_before, crlf);
    bench.verdict("abort() brings BUSY down with the ABORT register clear, leaves nothing "
                  "raised or pending, runs no handler and puts the route back",
                  aborted && !C4::busy() && !C4::raised() && !C4::pending(0) && C4::routed(0) &&
                      Dma::abort_pending() == 0u && ch4_on_line[0] == irqs_before);
    DmaTimer<0>::stop();
    C4::stop();
}

// =============================================================================
// b - memory to memory at three widths, and timed
// =============================================================================
void tb_memory() {
    struct Leg { DmaSize size; uint32_t bytes; const char* name; };
    constexpr Leg legs[] = {{DmaSize::byte, 256, "256 bytes"}, {DmaSize::half, 256, "128 half-words"},
                            {DmaSize::word, 256, "64 words"}};
    for (const Leg& l : legs) {
        fill(src, l.bytes, 0x1234u + l.bytes + static_cast<uint32_t>(l.size));
        fill(dst, l.bytes, 0x9999u);
        const bool done = run_c4({.read = src, .write = dst,
                                  .count = l.bytes / dma_size_bytes(l.size),
                                  .config = {.size = l.size}});
        const bool raised = C4::raised();
        C4::clear_raised();
        print(serial, "  ", l.name, ": ", done ? "done" : "NOT DONE", ", count ", C4::count(),
              ", raw ", raised ? "raised" : "not raised", " then ",
              C4::raised() ? "STILL raised" : "cleared", crlf);
        bench.verdict(l.name, ": the copy is exact, TRANS_COUNT at zero, the raw status raised "
                      "and cleared by writing one",
                      done && same(src, dst, l.bytes) && C4::count() == 0u && raised &&
                          !C4::raised());
    }
    fill(src, 4096, 0x5A5Au);
    fill(dst, 4096, 0);
    const uint32_t t0 = us_now();
    const bool done = run_c4({.read = src, .write = dst, .count = 1024,
                              .config = {.size = DmaSize::word}});
    const uint32_t took = us_now() - t0;
    C4::clear_raised();
    print(serial, "  4096 bytes as 1024 words in ", took,
          " us (6.8 us would be one transfer a cycle at 150 MHz; the read and the write "
          "masters share the SRAM with this core, which is polling BUSY over the same bus)",
          crlf);
    bench.verdict("four kilobytes of words move exact, and in under a hundred microseconds",
                  done && same(src, dst, 4096) && took < 100u);
}

// =============================================================================
// c - the four interrupt lines
// =============================================================================
void tc_lines() {
    fill(src, 64, 0x77u);
    for (uint8_t line = 0; line < 4u; ++line) {
        uint32_t before[4];
        for (uint8_t i = 0; i < 4u; ++i) {
            before[i] = ch4_on_line[i];
        }
        C4::route(line, true);
        (void)run_c4({.read = src, .write = dst, .count = 64});
        spin_us(200);
        C4::route(line, false);
        uint32_t served = 0;
        uint32_t elsewhere = 0;
        for (uint8_t i = 0; i < 4u; ++i) {
            const uint32_t delta = ch4_on_line[i] - before[i];
            if (i == line) {
                served = delta;
            } else {
                elsewhere += delta;
            }
        }
        print(serial, "  routed to line ", line, ": that line's handler +", served,
              ", the other three +", elsewhere, crlf);
        bench.verdict("a completion routed to this line is served on it alone, exactly once "
                      "(and this chip has four, against the RP2040's two)",
                      served == 1u && elsewhere == 0u && !C4::pending(line));
    }

    C4::route(0, true);
    const uint32_t q0 = ch4_on_line[0];
    (void)run_c4({.read = src, .write = dst, .count = 64, .config = {.irq_quiet = true}});
    spin_us(200);
    const bool quiet_silent = ch4_on_line[0] == q0 && !C4::raised();
    C4::null_trigger();
    spin_us(200);
    print(serial, "  IRQ_QUIET: the completion raised ", quiet_silent ? 0u : 1u,
          " interrupt(s), the null trigger after it brought the total to ", ch4_on_line[0] - q0,
          crlf);
    bench.verdict("under IRQ_QUIET a completion is silent and a null trigger raises exactly one",
                  quiet_silent && ch4_on_line[0] == q0 + 1u);
    C4::stop();
}

// =============================================================================
// d - chaining
// =============================================================================
void td_chain() {
    fill(src, 512, 0xC4C5u);
    fill(dst, 512, 0);
    C5::stop();
    const bool prepared = C5::prepare({.read = src + 256, .write = dst + 256, .count = 256});
    const bool started = C4::load({.read = src, .write = dst, .count = 256,
                                   .config = {.chain_to = 5}});
    const uint32_t t0 = us_now();
    while ((C4::busy() || C5::busy() || C5::count() != 0u) && us_now() - t0 < 10'000u) {
    }
    print(serial, "  channel 4 chained to 5: 4 raised=", C4::raised(), " 5 raised=", C5::raised(),
          ", 5's count ", C5::count(), ", both halves ",
          same(src, dst, 512) ? "exact" : "WRONG", crlf);
    bench.verdict("the chained channel, prepared and waiting, ran at the first one's "
                  "completion: both halves exact, both raised",
                  prepared && started && C4::raised() && C5::raised() && same(src, dst, 512));
    C4::stop();
    C5::stop();
}

// =============================================================================
// e - address wrapping, and the four steps
// =============================================================================
void te_steps() {
    fill(src, 32, 0xABu);
    fill(dst, 64, 0);
    const bool ring_done = run_c4({.read = src, .write = dst, .count = 64,
                                   .config = {.ring_bits = 4}});
    bool four_times = true;
    for (uint32_t i = 0; i < 64u; ++i) {
        four_times = four_times && dst[i] == src[i & 15u];
    }
    print(serial, "  a sixteen-byte ring read into sixty-four bytes: ",
          four_times ? "the pattern four times" : "WRONG", crlf);
    bench.verdict("RING_SIZE wraps the read address every sixteen bytes",
                  ring_done && four_times);

    // BACKWARD, which the RP2040's controller cannot do: INCR_READ with
    // INCR_READ_REV, the read address decrementing by the transfer size.
    fill(dst, 64, 0);
    const bool back_done = run_c4({.read = src + 15, .write = dst, .count = 16,
                                   .config = {.read_step = DmaStep::backward}});
    bool reversed = true;
    for (uint32_t i = 0; i < 16u; ++i) {
        reversed = reversed && dst[i] == src[15u - i];
    }
    print(serial, "  read backward from the sixteenth byte: ",
          reversed ? "the copy is the source reversed" : "WRONG", crlf);
    bench.verdict("INCR_READ_REV walks the source DOWNWARD, one transfer size at a time",
                  back_done && reversed);

    // BY TWOS: INCR_READ clear with INCR_READ_REV set, the combination
    // that steps by twice the transfer size.
    fill(dst, 64, 0);
    const bool two_done = run_c4({.read = src, .write = dst, .count = 16,
                                  .config = {.read_step = DmaStep::forward_by_two}});
    bool alternate = true;
    for (uint32_t i = 0; i < 16u; ++i) {
        alternate = alternate && dst[i] == src[2u * i];
    }
    print(serial, "  read by twos: ", alternate ? "every other byte of the source" : "WRONG",
          crlf);
    bench.verdict("INCR_READ clear WITH its REV bit steps by twice the transfer size, "
                  "skipping alternate cells",
                  two_done && alternate);
    C4::stop();
}

// =============================================================================
// f - a pacing timer
// =============================================================================
void tf_pacing() {
    fill(src, 1000, 0x11u);
    DmaTimer<0>::set(1, 150);   // clk_sys / 150 = 1 MHz
    const uint32_t t0 = us_now();
    const bool done = run_c4({.read = src, .write = dst, .count = 1000,
                              .config = {.treq = DmaTimer<0>::dreq()}});
    const uint32_t took = us_now() - t0;
    print(serial, "  a thousand bytes at one request a microsecond: ", took, " us (the timer "
          "reads back X=", DmaTimer<0>::numerator(), " Y=", DmaTimer<0>::denominator(), ")",
          crlf);
    DmaTimer<0>::stop();
    bench.verdict("the pacing timer at X/Y = 1/150 moves one byte a microsecond, within 10 %",
                  done && same(src, dst, 1000) && took >= 990u && took <= 1100u);
    C4::stop();
}

// =============================================================================
// g - the sniffer
// =============================================================================
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
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc32}, crc32_ethernet_init);
    (void)run_c4({.read = src, .write = dst, .count = 256, .config = {.sniff = true}});
    const uint32_t got_crc32 = DmaSniffer::result();
    const uint32_t want_crc32 = crc32_ethernet_bytes(src, 256);
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc32, .out_reverse = true, .out_invert = true},
                      crc32_ethernet_init);
    (void)run_c4({.read = src, .write = dst, .count = 256, .config = {.sniff = true}});
    const uint32_t got_rev_inv = DmaSniffer::result();
    DmaSniffer::stop();
    print(serial, "  sum ", got_sum, " (", sum, "), crc16 ", hex(got_crc16), " (",
          hex(want_crc16), "), crc32 ", hex(got_crc32), " (", hex(want_crc32),
          "), reversed+inverted ", hex(got_rev_inv), " (", hex(~reverse32(want_crc32)), ")",
          crlf);
    bench.verdict("the sniffer's sum is the bytes' sum", got_sum == sum);
    bench.verdict("its CRC-16 is util/crc.hpp's CRC-16-CCITT at the same seed",
                  got_crc16 == want_crc16);
    bench.verdict("its CRC-32 is util/crc.hpp's CRC-32/MPEG-2 - the Ethernet polynomial, most "
                  "significant bit first, from the same seed",
                  got_crc32 == want_crc32);
    bench.verdict("OUT_REV and OUT_INV present the same result bit-reversed and inverted",
                  got_rev_inv == ~reverse32(want_crc32));
    C4::stop();
}

// =============================================================================
// h - a bus error
// =============================================================================
void th_bus_error() {
    fill(dst, 64, 0);
    C4::route(0, true);
    const uint32_t l0 = ch4_on_line[0];
    // 12.6.7: "SIO is not visible from the DMA bus ports as it is tightly
    // coupled to the processors" - the chapter's own example of an address
    // the fabric cannot decode.
    const auto* unreachable = reinterpret_cast<const volatile void*>(0xD0000000u);
    const bool started = C4::load({.read = unreachable, .write = dst, .count = 16,
                                   .config = {.size = DmaSize::word,
                                              .read_step = DmaStep::fixed}});
    const uint32_t t0 = us_now();
    while (C4::busy() && us_now() - t0 < 10'000u) {
    }
    spin_us(200);
    const uint32_t errors = C4::errors();
    print(serial, "  a word block reading the SIO window: busy=", C4::busy(), " count=",
          C4::count(), " errors=", hex(errors), " (AHB ", (errors & DmaError::ahb) != 0u,
          ", read ", (errors & DmaError::read) != 0u, ", write ",
          (errors & DmaError::write) != 0u, "), line-0 handler +", ch4_on_line[0] - l0,
          ", dst[0]=", hex(dst[0]), crlf);
    bench.verdict("the block ended one way or the other and the channel came back",
                  started && !C4::busy());
    C4::clear_errors();
    bench.verdict("the error bits clear by writing one", C4::errors() == 0u);
    fill(src, 64, 0x42u);
    const bool again = run_c4({.read = src, .write = dst, .count = 64});
    bench.verdict("and the channel copies again afterwards - 12.6.7.2's recovery is BUSY down "
                  "and the flags written back",
                  again && same(src, dst, 64));
    C4::stop();
}

// =============================================================================
// i - the count modes
// =============================================================================
void ti_count_modes() {
    // TRIGGER_SELF: a 64-byte source read through a ring of 64 into a
    // destination that goes on advancing. The channel re-triggers itself
    // at every zero; the handler counts the laps and clears EN at the
    // fourth, which is the only software in the loop. PACED AT 10 kHz -
    // a lap is 6.4 ms - so that the destination cannot run away: were the
    // handler never to fire, the loop's own budget below stops the letter
    // after some fifteen laps, a thousand bytes of the four the buffer
    // holds.
    fill(src, 64, 0x9E9Eu);
    fill(dst, 512, 0);
    self_laps = 0;
    self_running = true;
    DmaTimer<1>::set(1, 15000);
    C5::stop();
    C5::route(1, true);
    const bool started = C5::load({.read = src, .write = dst, .count = 64,
                                   .mode = DmaCountMode::trigger_self,
                                   .config = {.ring_bits = 6, .treq = DmaTimer<1>::dreq()}});
    const bool mode_reads_back = C5::mode() == DmaCountMode::trigger_self;
    const uint32_t t0 = us_now();
    while (self_laps < 4u && us_now() - t0 < 100'000u) {
    }
    const uint32_t took = us_now() - t0;
    const uint32_t laps = self_laps;
    self_running = false;
    spin_us(200);
    const bool aborted = C5::abort();
    DmaTimer<1>::stop();
    bool four_laps_written = true;
    for (uint32_t i = 0; i < 256u; ++i) {
        four_laps_written = four_laps_written && dst[i] == src[i & 63u];
    }
    print(serial, "  TRIGGER_SELF: ", laps, " laps of 64 bytes at 10 kHz in ", took,
          " us with no software between them, the count reloaded to ", C5::debug_reload(),
          " each time; the first 256 bytes are the source four times over: ",
          four_laps_written ? "yes" : "NO", crlf);
    bench.verdict("a channel in TRIGGER_SELF re-arms itself at every zero: four laps ran with "
                  "the processor only counting them, each raising its own interrupt, and the "
                  "mode reads back out of TRANS_COUNT's top nibble",
                  started && mode_reads_back && laps >= 4u && four_laps_written && aborted);

    // ENDLESS: the count does not decrement at all, and the register
    // description is explicit that such a channel raises no interrupt and
    // triggers nobody. One cell into one cell, so nothing runs away.
    static volatile uint32_t endless_src = 0xA5A5A5A5u;
    static volatile uint32_t endless_dst = 0;
    const uint32_t raised_before = ch4_on_line[0];
    C4::route(0, true);
    DmaTimer<1>::set(1, 150);
    const bool endless_started = C4::load({.read = &endless_src, .write = &endless_dst,
                                           .count = 8, .mode = DmaCountMode::endless,
                                           .config = {.size = DmaSize::word,
                                                      .read_step = DmaStep::fixed,
                                                      .write_step = DmaStep::fixed,
                                                      .treq = DmaTimer<1>::dreq()}});
    spin_us(5000);
    const uint32_t still = C4::count();
    const bool still_busy = C4::busy();
    const bool silent = ch4_on_line[0] == raised_before && !C4::raised();
    const bool endless_aborted = C4::abort();
    DmaTimer<1>::stop();
    print(serial, "  ENDLESS: after 5 ms of one transfer a microsecond the count still reads ",
          still, " (it was 8), busy=", still_busy, ", interrupts raised ",
          ch4_on_line[0] - raised_before, ", the destination holds ", hex(endless_dst),
          "; the abort ended it: busy=", C4::busy(), crlf);
    bench.verdict("an ENDLESS channel never decrements its count, raises nothing at all, and "
                  "is ended by an abort and nothing else",
                  endless_started && still == 8u && still_busy && silent && endless_aborted &&
                      !C4::busy() && endless_dst == 0xA5A5A5A5u);
    C4::stop();
    C5::stop();
}

// =============================================================================
// j - the instrument's two engines
// =============================================================================
void tj_instrument() {
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
    const uint32_t irqs0 = line_irqs[0];
    uint32_t sent = 0;
    uint32_t good = 0;
    bool wrong = false;
    uint8_t bad_got = 0;
    uint8_t bad_want = 0;
    bool bad_tx_idle = false;
    uint32_t bad_taken = 0;
    uint32_t bad_capacity = 0;
    uint32_t bad_gap = 0;
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
            const uint8_t want = next(rx_state);
            if (got[i] != want) {
                wrong = true;
                bad_got = got[i];
                bad_want = want;
                bad_tx_idle = Instrument::tx_idle();
                bad_taken = InstrumentRx::taken();
                bad_capacity = InstrumentRx::capacity();
                // WHICH byte of the stream did arrive? Stepping the same
                // generator forward from here names a GAP (bytes lost) and
                // its size; nothing found in 256 steps is not a gap.
                uint32_t look = rx_state;
                for (uint32_t k = 1; k <= 256u && bad_gap == 0u; ++k) {
                    if (next(look) == got[i]) {
                        bad_gap = k;
                    }
                }
            } else {
                ++good;
            }
        }
    }
    const uint32_t took = us_now() - t0;
    const uint32_t irqs = line_irqs[0] - irqs0;
    if (good != 4096u && !wrong) {
        // WHERE IS THE MISSING BYTE? The engine's own count says whether
        // the DMA ever took it out of the FIFO; one more harvest says
        // whether it was in memory and merely unpublished.
        const bool idle_now = InstrumentRx::idle();
        const uint32_t taken_now = InstrumentRx::taken();
        const uint32_t cap_now = InstrumentRx::capacity();
        const bool tx_done = Instrument::tx_idle();
        (void)Instrument::harvest();
        uint8_t late[4] = {};
        const uint32_t late_n = Instrument::read_bulk(late);
        print(serial, "  SHORT by ", 4096u - good, ": tx_idle ", tx_done ? 1 : 0,
              ", engine idle ", idle_now ? 1 : 0, " taken ", taken_now, " of ", cap_now,
              "; one more harvest yielded ", late_n, " byte(s) ", hex(late[0]), crlf);
    }
    if (wrong) {
        uint8_t got[8] = {};
        const uint32_t more = Instrument::read_bulk(got);
        print(serial, "  MISMATCH after ", good, " good bytes: sent ", sent, ", got ",
              hex(bad_got), " wanted ", hex(bad_want), " (the byte ", bad_gap,
              " ahead, 0 = not within 256); tx_idle ", bad_tx_idle ? 1 : 0,
              ", engine taken ", bad_taken, " of ", bad_capacity, "; ", more,
              " more byte(s) behind it: ", hex(got[0]), " ", hex(got[1]), " ", hex(got[2]),
              " ", hex(got[3]), crlf);
    }
    print(serial, "  4096 bytes through UART1's loop-back on two engines at 3 Mbaud: ", good,
          " exact in ", took, " us, ", irqs,
          " line-0 interrupts (the console's blocks among them), faults ",
          Instrument::dma_faults(), ", errors FE ", Instrument::frame_errors(), " OE ",
          Instrument::hw_overruns(), " ring ", Instrument::rx_overruns(), crlf);
    bench.verdict("every byte the transmit engine poured, the receive engine harvested, in order",
                  good == 4096u && !wrong);
    bench.verdict("no fault, no error, no overrun",
                  Instrument::dma_faults() == 0u && Instrument::frame_errors() == 0u &&
                      Instrument::hw_overruns() == 0u && Instrument::rx_overruns() == 0u);
    bench.verdict("the wire's time is what it took (13653 us of frames at 3 Mbaud), within 10 %",
                  took >= 13'000u && took <= 15'100u);
    const uint32_t pm = wire_permille<Instrument>(3'000'000);
    print(serial, "  1024 frames out through the transmit engine in ", pm,
          " thousandths of their wire time at 3 Mbaud", crlf);
    bench.verdict("THE LINE RUNS AT THE RATE ASKED: 1024 frames through the transmit engine in their "
                  "wire time at 3 Mbaud, -1 % to +3 % (the loop alone shares the divisor at both ends)",
                  wire_ok(pm));
    Instrument::release();
}

// =============================================================================
// k - the console's own transmit engine
// =============================================================================
void tk_console_engine() {
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
    print(serial, "  2064 bytes of burst went out in ", used,
          " block(s) of the console's engine (", blocks, " before this letter), faults ",
          Serial::dma_faults(), ", idle after the drain: ", idle, crlf);
    bench.verdict("the burst left in more than one block and the transport was idle again "
                  "after it",
                  used > 1u && idle);
    bench.verdict("no fault on the console's engine", Serial::dma_faults() == 0u);
}

// =============================================================================
// l - the engines' own surface
// =============================================================================
using BeatTx = DmaTxEngine<8, uint32_t>;
using QuietTx = DmaTxEngine<9, uint32_t>;
using Copy = DmaCopyEngine<10>;
volatile uint32_t quiet_entries = 0;   // line-0 entries QuietTx's service() answered
volatile uint8_t quiet_flags = 0;
alignas(4) volatile uint32_t beat_cell = 0;

/// Wait for a channel's BUSY to fall, bounded.
template <uint8_t ch>
bool settle(uint32_t budget_us = 10'000u) {
    const uint32_t t0 = us_now();
    while (DmaChannel<ch>::busy() && us_now() - t0 < budget_us) {
    }
    return !DmaChannel<ch>::busy();
}

void tl_engines() {
    // ONE CHANNEL, THREE BEATS: a byte run, a half-word run, a word run
    // poured in turn into one cell of RAM - an SRAM write of a byte or a
    // half-word writes that lane alone, so the cell shows which beat
    // wrote it last. The binding reports errors alone: the owner
    // completes each block once BUSY has fallen.
    alignas(4) static const uint8_t bytes4[4] = {0x11, 0x22, 0x33, 0x44};
    alignas(4) static const uint16_t halves2[2] = {0x5566, 0x7788};
    alignas(4) static const uint32_t words2[2] = {0x99AABBCCu, 0xDDEEFF01u};
    beat_cell = 0;
    BeatTx::arm(&beat_cell, Dreq::permanent, false, DmaReport::errors);
    const bool b1 = BeatTx::start(bytes4, 4) && settle<8>();
    (void)BeatTx::complete();
    const uint32_t after_bytes = beat_cell;
    const bool b2 = BeatTx::start(halves2, 2) && settle<8>();
    (void)BeatTx::complete();
    const uint32_t after_halves = beat_cell;
    const bool b3 = BeatTx::start(words2, 2) && settle<8>();
    (void)BeatTx::complete();
    const uint32_t after_words = beat_cell;
    print(serial, "  one channel, three beats into one cell: ", hex(after_bytes), " ",
          hex(after_halves), " ", hex(after_words), crlf);
    bench.verdict("one channel carries a byte, a half-word and a word block in turn, each at its "
                  "own beat",
                  b1 && b2 && b3 && after_bytes == 0x00000044u && after_halves == 0x00007788u &&
                      after_words == 0xDDEEFF01u);
    alignas(4) static uint16_t odd_halves[3];
    bench.verdict("a half-word run one byte off its beat, an empty run and a 29-bit count are "
                  "refused, and give the claim back",
                  !BeatTx::start(reinterpret_cast<const uint16_t*>(
                                     reinterpret_cast<const uint8_t*>(odd_halves) + 1),
                                 2) &&
                      !BeatTx::start(words2, 0) && !BeatTx::start(words2, dma_count_max + 1u) &&
                      !BeatTx::busy());
    BeatTx::stop();

    // ERRORS ALONE: IRQ_QUIET takes the completion off the line and leaves
    // the bus error on it.
    QuietTx::arm(&beat_cell, Dreq::permanent, false, DmaReport::errors);
    const uint32_t e0 = quiet_entries;
    const uint32_t l0 = line_irqs[0];
    const bool clean = QuietTx::start(words2, 2) && settle<9>();
    spin_us(100);
    (void)QuietTx::complete();
    const uint32_t clean_entries = quiet_entries - e0;
    const uint32_t clean_lines = line_irqs[0] - l0;
    quiet_flags = 0;
    const auto* unreachable = reinterpret_cast<const uint32_t*>(0xD0000000u);   // the SIO (12.6.7)
    const bool faulted = QuietTx::start(unreachable, 4) && settle<9>();
    spin_us(100);
    const uint32_t fault_entries = quiet_entries - e0;
    print(serial, "  errors alone: a clean block raised ", clean_entries, " (line 0 entered ",
          clean_lines, "), a block reading the SIO raised ", fault_entries, " with flags ",
          hex(quiet_flags), crlf);
    bench.verdict("a binding that reports errors alone: a clean block raises nothing",
                  clean && clean_entries == 0u && clean_lines == 0u);
    bench.verdict("and a bus error under it still raises the channel's interrupt, served as an "
                  "error",
                  faulted && fault_entries == 1u &&
                      (quiet_flags & QuietTx::flag_error) != 0u);
    (void)QuietTx::abandon();
    const bool again = QuietTx::start(words2, 2) && settle<9>();
    (void)QuietTx::complete();
    bench.verdict("abandon() hands the channel back, and it runs again", again &&
                  beat_cell == 0xDDEEFF01u);
    QuietTx::stop();

    // THE COPY ENGINE, polled: copy and fill at three beats.
    Copy::arm(DmaReport::errors);
    fill(src, 4096, 0x2468u);
    fill(dst, 4096, 0);
    const bool w = Copy::copy(reinterpret_cast<uint32_t*>(dst), reinterpret_cast<const uint32_t*>(src),
                              1024) && settle<10>();
    const bool word_copy = w && same(src, dst, 4096);
    fill(dst, 4096, 0);
    const bool h = Copy::copy(reinterpret_cast<uint16_t*>(dst + 2),
                              reinterpret_cast<const uint16_t*>(src + 2), 100) && settle<10>();
    const bool half_copy = h && same(src + 2, dst + 2, 200) && dst[0] == 0u && dst[1] == 0u;
    fill(dst, 4096, 0);
    const bool b = Copy::copy(dst + 1, src + 1, 333) && settle<10>();
    const bool byte_copy = b && same(src + 1, dst + 1, 333) && dst[0] == 0u;
    print(serial, "  copy: 1024 words ", word_copy ? "exact" : "WRONG", ", 100 half-words ",
          half_copy ? "exact" : "WRONG", ", 333 bytes at an odd address ",
          byte_copy ? "exact" : "WRONG", crlf);
    bench.verdict("the copy engine copies at three beats, exact", word_copy && half_copy && byte_copy);
    static const uint32_t word_cell = 0xA5C3E1F0u;
    static const uint16_t pixel = 0xF81Fu;
    static const uint8_t byte_cell = 0x5Au;
    const bool fw = Copy::fill(reinterpret_cast<uint32_t*>(dst), &word_cell, 64) && settle<10>();
    bool word_fill = fw;
    for (uint32_t i = 0; i < 64; ++i) {
        word_fill = word_fill && reinterpret_cast<const uint32_t*>(dst)[i] == word_cell;
    }
    const bool fh = Copy::fill(reinterpret_cast<uint16_t*>(dst + 512), &pixel, 64) && settle<10>();
    bool half_fill = fh;
    for (uint32_t i = 0; i < 64; ++i) {
        half_fill = half_fill && reinterpret_cast<const uint16_t*>(dst + 512)[i] == pixel;
    }
    const bool fb = Copy::fill(dst + 1001, &byte_cell, 77) && settle<10>();
    bool byte_fill = fb && dst[1000] != byte_cell;
    for (uint32_t i = 0; i < 77; ++i) {
        byte_fill = byte_fill && dst[1001 + i] == byte_cell;
    }
    print(serial, "  fill: 64 words ", word_fill ? "exact" : "WRONG", ", 64 half-words ",
          half_fill ? "exact" : "WRONG", ", 77 bytes ", byte_fill ? "exact" : "WRONG", crlf);
    bench.verdict("the copy engine fills from one cell at three beats, exact",
                  word_fill && half_fill && byte_fill);
    // A block thrown away in flight: abandon() is the abort polled to its
    // end, and the channel copies again afterwards.
    fill(dst, 4096, 0);
    const bool long_started = Copy::copy(reinterpret_cast<uint32_t*>(dst),
                                         reinterpret_cast<const uint32_t*>(src), 1024);
    const bool thrown = Copy::abandon();
    const bool partial = !same(src, dst, 4096);
    const bool after = Copy::copy(reinterpret_cast<uint32_t*>(dst),
                                  reinterpret_cast<const uint32_t*>(src), 1024) && settle<10>();
    print(serial, "  abandon: started ", long_started, ", thrown away ", thrown,
          ", the destination short of the source ", partial, ", then copied again ",
          after && same(src, dst, 4096) ? "exact" : "WRONG", crlf);
    bench.verdict("a copy abandoned in flight stops short, and the engine copies again exact",
                  long_started && thrown && partial && after && same(src, dst, 4096));
    bench.verdict("and refuses a misaligned word run and an empty one",
                  !Copy::copy(reinterpret_cast<uint32_t*>(dst + 2), reinterpret_cast<const uint32_t*>(src),
                              4) &&
                      !Copy::copy(reinterpret_cast<uint32_t*>(dst), reinterpret_cast<const uint32_t*>(src),
                                  0) &&
                      Copy::errors() == 0u);
    Copy::stop();
}

// =============================================================================
// m, n - the diagnostic halves (by name)
// =============================================================================
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
    print(serial, "  ", name, ": up=", up, " got ", got, " in ", us_now() - t0, " us, exact=",
          same(out, in, got), "; rx ch7 count ", DmaChannel<7>::count(), " busy ",
          DmaChannel<7>::busy(), " credits ", DmaChannel<7>::debug_credits(), " raw ",
          DmaChannel<7>::raised(), "; tx ch6 count ", DmaChannel<6>::count(), " busy ",
          DmaChannel<6>::busy(), crlf);
    bench.verdict(name, ": 512 bytes back exact", got == 512u && same(out, in, 512));
    const uint32_t pm = wire_permille<T>(3'000'000);
    print(serial, "  1024 frames out in ", pm, " thousandths of their wire time at 3 Mbaud", crlf);
    bench.verdict(name, ": 1024 frames in their wire time at 3 Mbaud, -1 % to +3 %", wire_ok(pm));
    T::release();
    uart1_isr = nullptr;
}
void tm_tx_only() { half_loop<TxOnly>("the transmit engine alone, the receiver on its interrupt"); }
void tn_rx_only() { half_loop<RxOnly>("the receive engine alone, the transmitter on its interrupt"); }

// ---------------------------------------------------------------------------
// t: tx_idle() is the wire's - the last stop bit off the pad
// ---------------------------------------------------------------------------
/// Eight frames of 0xFF at 9600 baud on GP4 with the loop-back OFF, so the
/// pad carries them, the pad read through SIO's GPIO_IN (which follows a
/// pad under any function): each frame has exactly one falling edge, its
/// start bit, so the last one places the last stop bit's end ten bit times
/// later; tx_idle() - the ring empty, no block in flight, UARTFR.BUSY clear - must
/// turn true at that end and not before.
void tt_tx_idle() {
    constexpr uint32_t bit_us = 1'000'000u / 9600u;
    uart1_isr = &TxOnly::isr;
    const bool up = TxOnly::init(clock, 9600);
    spin_us(5000);
    static const uint8_t ones[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    bool level = Pin<4>::read();
    uint32_t falls = 0;
    uint32_t first_fall = 0;
    uint32_t last_fall = 0;
    uint32_t idle_at = 0;
    bool idle_seen = false;
    (void)TxOnly::write_bulk(ones);
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 12u * 10u * bit_us) {
        const uint32_t t = us_now();
        const bool now_level = Pin<4>::read();
        if (level && !now_level) {
            if (falls == 0u) {
                first_fall = t;
            }
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
    print(serial, "  8 frames of 0xFF at 9600 on GP4 through the transmit engine: ", falls, " start bits; tx_idle() ",
          idle_seen ? "" : "NEVER ", "true ", after,
          " us from the last stop bit's end (a bit is ", bit_us, " us)", crlf);
    // Two microseconds of margin before the end: the fall is seen up to
    // one late, the idle up to one early.
    bench.verdict("tx_idle() IS THE WIRE'S through the transmit engine: never before the last stop "
                  "bit's end, and within a bit time after it",
                  up && idle_seen && falls == 8u && after >= -2 &&
                      after <= static_cast<int32_t>(bit_us));
    // THE PAD'S OWN CLOCK: eight start bits back to back are seven frames
    // apart, ten bit periods each at the rate asked.
    const uint32_t span = last_fall - first_fall;
    const uint32_t frames_us = 7u * 10u * 1'000'000u / 9600u;
    print(serial, "  the first to the last start bit: ", span, " us (seven frames at 9600: ", frames_us,
          " us)", crlf);
    bench.verdict("THE PAD RUNS AT THE RATE ASKED: seven frames between the first and the last start "
                  "bit, within a per cent of 9600 baud",
                  falls == 8u && span * 100u >= frames_us * 99u && span * 100u <= frames_us * 101u);
    TxOnly::release();
    uart1_isr = nullptr;
}

/// What the last breaks_through() measured of its line before the slots
/// (wire_permille).
uint32_t breaks_wire = 0;

/// K BREAKS IN N SLOTS on UART1's loop-back at 115200: 64 slots, four of
/// them a break (UARTLCR_H.BRK held two frames) instead of their byte.
/// Sent through the transmit engine; received by the interrupt receiver
/// (TxOnly) and by the receive engine (Instrument), the stream judged
/// against the 60 bytes that were sent.
template <typename T>
uint32_t breaks_through(uint8_t* got, uint32_t room, uint32_t& kept) {
    constexpr uint32_t frame_us = 10u * 1'000'000u / 115200u + 1u;
    const bool up = T::init(clock, 115200) && T::loopback(true);
    breaks_wire = wire_permille<T>(115200, 10u, 256u);
    spin_us(20 * frame_us);
    uint8_t junk[32];
    (void)T::harvest();
    while (T::read_bulk(junk) != 0u) {
    }
    T::clear_errors();
    kept = 0;
    uint32_t n = 0;
    for (uint32_t i = 0; i < 64u; ++i) {
        if (i == 9u || i == 24u || i == 41u || i == 58u) {
            const uint32_t t0 = us_now();
            while (!T::tx_idle() && us_now() - t0 < 100u * frame_us) {
            }
            T::Resource::break_send(true);
            spin_us(2u * frame_us);
            T::Resource::break_send(false);
            // The break's entry waited out and LOOKED AT before the next
            // byte lands: the look skips the drop with nothing clean queued
            // behind it.
            spin_us(5u * frame_us);
            (void)T::harvest();
            n += T::read_bulk({got + n, room - n});
            continue;
        }
        const uint8_t b = static_cast<uint8_t>(0x40u + i);
        while (T::write_bulk({&b, 1}) == 0u) {
        }
        ++kept;
        // THE CONSUMER KEEPS UP, a look a slot: the receive ring skips
        // what it holds at the look after a drop (util/ring.hpp's
        // SkipRing), so a stream read once at its end would lose every
        // clean byte queued before the last break - the slot is waited
        // out (its frame and the receive time-out) and taken.
        spin_us(5u * frame_us);
        (void)T::harvest();
        n += T::read_bulk({got + n, room - n});
    }
    spin_us(80u * frame_us);
    (void)T::harvest();
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
    print(serial, "  its line before the slots: 256 frames in ", breaks_wire,
          " thousandths of their wire time at 115200", crlf);
    bench.verdict("and its line runs at the rate asked: 256 frames in their wire time at 115200, "
                  "-1 % to +3 %", wire_ok(breaks_wire));
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
    print(serial, "  its line before the slots: 256 frames in ", breaks_wire,
          " thousandths of their wire time at 115200", crlf);
    bench.verdict("and its line runs at the rate asked through the transmit engine: 256 frames in "
                  "their wire time at 115200, -1 % to +3 %", wire_ok(breaks_wire));
    Instrument::release();
    uart1_isr = nullptr;
}

// -----------------------------------------------------------------------------
// o  THE OVERRUN A STALLED RECEIVE ENGINE LEAVES (the overrun-storm class)
//
// A handler that cannot serve a standing flag and never disarms it
// re-enters for ever; a flag that stands where the handler cannot see it
// hides every event after the first. Under a receive engine the PL011
// transport arms the four error interrupts and nothing else of the
// receiver. The overrun interrupt rises from UARTRSR's OE, which is
// STICKY (12.1.8: set "if data is received and the receive FIFO is already
// full", cleared by a write of the register), so the entry clears it - and
// masks the source until the next publish, because a FIFO that stays full
// under a consumer that does not read would raise it again with every
// frame that lands. The letter makes the worst case three times: a run
// that fills the ring whole and finds no room to re-arm (nobody consumes),
// the FIFO filling behind the idle channel, and four thousand frames more
// at 3 Mbaud landing on it; then the consumer comes back, and the next
// episode. Every UART1 entry is counted, during each and in the silence
// after it.

volatile uint32_t uart1_entries = 0;

/// One overrun episode: 5120 frames at 3 Mbaud into the ring with nobody
/// reading it - the ring fills, its engine finds no room to re-arm, the
/// FIFO fills behind the idle channel and the rest lands on it. Returns
/// the UART1 entries the episode cost.
uint32_t overrun_episode() {
    static uint8_t out[1024];
    fill(out, sizeof out, 0x5151u);
    const uint32_t e0 = uart1_entries;
    const uint32_t t0 = us_now();
    uint32_t queued = 0;
    while (queued < 5120u && us_now() - t0 < 100'000u) {
        const uint32_t want = 5120u - queued;
        queued += Instrument::write_bulk(std::span<const uint8_t>(out, want < sizeof out ? want : sizeof out));
    }
    while (!Instrument::tx_idle() && us_now() - t0 < 100'000u) {
    }
    spin_us(200);
    return uart1_entries - e0;
}

void to_stalled_overrun() {
    uart1_isr = &Instrument::isr;
    const bool up = Instrument::init(clock, 3'000'000) && Instrument::loopback(true);
    Instrument::clear_errors();
    uint32_t during[3] = {0, 0, 0};
    uint32_t after = 0;
    uint8_t oe[3] = {0, 0, 0};
    for (uint8_t k = 0; k < 3u; ++k) {
        during[k] = overrun_episode();
        const uint32_t e1 = uart1_entries;
        spin_us(10'000);
        after += uart1_entries - e1;
        oe[k] = Instrument::hw_overruns();
        // The consumer comes back: the ring read empty and the engine
        // re-armed by harvest(), the FIFO taken - the next episode starts
        // from a receiver that has room again.
        uint8_t sink[64];
        for (uint8_t pass = 0; pass < 4u; ++pass) {
            (void)Instrument::harvest();
            while (Instrument::read_bulk(sink) != 0u) {
            }
            spin_us(500);
        }
    }
    const uint8_t ring = Instrument::rx_overruns();
    print(serial, "  three episodes of 5120 frames at 3 Mbaud into a 1024-byte ring nobody reads: ",
          during[0], ", ", during[1], " and ", during[2], " UART1 entries, ", after,
          " in the silences after them; OE counted ", oe[0], ", ", oe[1], ", ", oe[2],
          " after each; ring overruns ", ring, ", faults ", Instrument::dma_faults(), crlf);
    bench.verdict("the ring filled and the engine found no room to re-arm: the ring overruns counted",
                  up && ring >= 3u);
    bench.verdict("EVERY OVERRUN IS COUNTED AND REPORTED: one OE an episode, three in three - the "
                  "entry clears the sticky status (UARTRSR) the next onset rises from, and the "
                  "publish after it wakes the source",
                  oe[0] == 1u && oe[1] == 2u && oe[2] == 3u);
    bench.verdict("NO STORM: four thousand frames lost on a full FIFO cost the vector one entry or "
                  "two an episode - the source masked from the loss to the next publish - and the "
                  "vector is quiet once the line is",
                  during[0] >= 1u && during[0] <= 4u && during[1] <= 4u && during[2] <= 4u &&
                      after == 0u);
    Instrument::release();
    uart1_isr = nullptr;
}

// -----------------------------------------------------------------------------
// p  THE CONSOLE'S RING BESIDE COPIES, host-fed (OUTSIDE z)
//
// docs/design/dma.md's priority rule on this controller's one bit: in each
// scheduling round every high channel and then ONE normal channel (12.6.10,
// HIGH_PRIORITY). The console UART0 takes a receive engine on channel 1
// over an 8 KB ring for the leg, the host pumps its pattern (brio stress,
// the sink op), and byte copies of 16 KB run back to back on channels 12
// and 13 the whole window: the ring armed at the transport's level (HIGH)
// and at NORMAL through an engine that ignores the level it is asked for,
// the copies at their own level and at HIGH_PRIORITY, one or two - two
// HIGH_PRIORITY copies beside a normal ring being the arbiter's worst
// case. Every byte the host sent is judged. MEASURED on both architectures:
// a ring at normal beside TWO HIGH_PRIORITY copies is starved - 7 to 17
// per cent of its bytes lost at 1 Mbaud, two fifths to two thirds at
// 3 Mbaud, every loss counted and skipped - where one such copy
// or two normal ones starve nothing, and the ring at HIGH loses nothing
// beside any of them (docs/rp2350/dma.md).
//
//   brio stress --letters p --beyond-vcp --board <board>

/// Channel 1's receive engine armed at the NORMAL level whatever the
/// transport asks: the arrangement the rule replaced.
struct NormalRx : DmaRxEngine<1> {
    NormalRx() = delete;
    static void arm(volatile void* data, Dreq dreq, bool, DmaReport report = DmaReport::blocks) {
        DmaRxEngine<1>::arm(data, dreq, false, report);
    }
};
constexpr uint32_t stress_ring = 8192;
using SinkHigh = Uart<0, console_pins, stress_ring, 256, DmaTxEngine<0>, DmaRxEngine<1>>;
using SinkNormal = Uart<0, console_pins, stress_ring, 256, DmaTxEngine<0>, NormalRx>;

/// Who owns UART0: 0 the console, 1 SinkHigh, 2 SinkNormal.
volatile uint8_t uart0_owner = 0;

using LoadA = DmaCopyEngine<12>;
using LoadB = DmaCopyEngine<13>;
alignas(4) uint8_t load_src[2][16384];
alignas(4) uint8_t load_dst[2][16384];
uint32_t load_blocks = 0;

template <typename E>
void keep_loaded(uint8_t i) {
    if (!E::busy() && E::copy(load_dst[i], load_src[i], 16384u)) {
        ++load_blocks;
    }
}

uint32_t lfsr_state = 0x12345678u;
uint32_t lfsr_step(uint32_t s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

struct SinkLeg {
    bool up = false;
    uint32_t in = 0;         ///< bytes received
    uint32_t gaps = 0;       ///< places the stream skipped positions
    uint32_t lost = 0;       ///< positions skipped there, in all
    uint32_t bad = 0;        ///< bytes matching no position near the expected one
    uint32_t blocks = 0;     ///< copy blocks run beside the leg
    bool high = false;       ///< the ring channel's HIGH_PRIORITY bit, read back
    uint8_t rx_overruns = 0;
    uint8_t hw_overruns = 0;
    uint8_t line_errors = 0;
    uint16_t faults = 0;
    uint32_t skips = 0;      ///< the receive ring's skips over the leg, modulo 256 (rx_skips())
};

/// A run of received bytes against the host's stream: a byte that is not
/// the next position's is looked for among the next 255, confirmed by the
/// byte after it; the positions stepped over were LOST, one GAP. A byte
/// that fits nowhere near is BAD and stands in for the position.
void judge(SinkLeg& leg, const uint8_t* run, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        ++leg.in;
        const uint32_t s = lfsr_step(lfsr_state);
        if (static_cast<uint8_t>(s) == run[i]) {
            lfsr_state = s;
            continue;
        }
        uint32_t t = s;
        bool found = false;
        for (uint32_t k = 1; k <= 255u; ++k) {
            t = lfsr_step(t);
            if (static_cast<uint8_t>(t) == run[i] &&
                (i + 1u == n || static_cast<uint8_t>(lfsr_step(t)) == run[i + 1u])) {
                leg.lost += k;
                ++leg.gaps;
                lfsr_state = t;
                found = true;
                break;
            }
        }
        if (!found) {
            ++leg.bad;
            lfsr_state = s;
        }
    }
}

template <typename Port>
SinkLeg sink_leg(uint8_t who, uint32_t baud, uint32_t window_ms, uint8_t copies, bool copies_high) {
    SinkLeg leg{};
    print(serial, "HOST sink 3 ", baud, " 8N1 ", window_ms, " 0", crlf);
    const uint32_t t_line = us_now();
    console_drain();
    Serial::release();
    uart0_owner = who;
    leg.up = Port::init(clock, baud);
    leg.high = (DmaChannel<1>::ctrl() & DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS) != 0u;
    LoadA::arm(DmaReport::errors, copies_high);
    LoadB::arm(DmaReport::errors, copies_high);
    load_blocks = 0;
    if (leg.up) {
        // The host settles 140 ms before it pumps: what comes before that
        // is the switch's, not the stream's.
        while (us_now() - t_line < 100'000u) {
        }
        uint8_t junk[64];
        (void)Port::harvest();
        while (Port::read_bulk(junk) != 0u) {
        }
        Port::clear_errors();
        const uint32_t skips0 = Port::rx_skips();
        lfsr_state = 0x12345678u;
        uint8_t chunk[64];
        while (us_now() - t_line < window_ms * 1000u) {
            if (copies >= 1u) {
                keep_loaded<LoadA>(0);
            }
            if (copies >= 2u) {
                keep_loaded<LoadB>(1);
            }
            (void)Port::harvest();
            for (;;) {
                const uint32_t n = Port::read_bulk(std::span<uint8_t>(chunk, sizeof chunk));
                if (n == 0u) {
                    break;
                }
                judge(leg, chunk, n);
            }
        }
        leg.rx_overruns = Port::rx_overruns();
        leg.hw_overruns = Port::hw_overruns();
        leg.line_errors = static_cast<uint8_t>(Port::frame_errors() + Port::parity_errors() +
                                               Port::break_errors());
        leg.faults = Port::dma_faults();
        leg.skips = (Port::rx_skips() - skips0) & 0xFFu;   // the ring's skip count is a byte
    }
    while (LoadA::busy() || LoadB::busy()) {
    }
    leg.blocks = load_blocks;
    LoadA::stop();
    LoadB::stop();
    Port::release();
    uart0_owner = 0;
    (void)Serial::init(clock, 115200);
    // A SILENCE BEFORE THE REPORT, the script's contract: whatever it
    // still had in flight lands in the console's ring and is no letter.
    spin_us(500'000);
    uint8_t c = 0;
    while (Serial::read_byte(c)) {
    }
    return leg;
}

/// The legs of letter p: the ring's level (1 the transport's, HIGH; 2
/// normal), how many copies run beside it and at which level.
struct LoadLeg {
    uint8_t who;
    uint8_t copies;
    bool copies_high;
};
constexpr LoadLeg load_legs[] = {
    {1, 2, false},   // the default arrangement: the ring HIGH, the copies at their normal
    {1, 2, true},    // the ring HIGH beside the worst the copies can be
    {2, 2, true},    // the arrangement the rule replaced, against the worst
    {2, 1, true},
    {2, 2, false},   // all three at normal: the round robin of equals
};
constexpr uint8_t load_leg_count = sizeof(load_legs) / sizeof(load_legs[0]);

void tp_ring_beside_copies() {
    print(serial, "  this letter needs brio stress on the other end of the console:", crlf,
          "  brio stress --letters p --beyond-vcp --board <board>", crlf);
    for (uint32_t k = 0; k < 2u; ++k) {
        fill(load_src[k], sizeof load_src[k], 0x7700u + k);
    }
    constexpr uint32_t rungs[] = {115200, 1'000'000, 3'000'000};
    constexpr uint32_t window_ms = 1500;
    bool fed = true;
    bool loaded = true;
    bool levels = true;
    bool reported = true;
    bool whole[load_leg_count];
    bool device_clean[load_leg_count];
    for (uint8_t l = 0; l < load_leg_count; ++l) {
        whole[l] = true;
        device_clean[l] = true;
    }
    for (const uint32_t baud : rungs) {
        for (uint8_t l = 0; l < load_leg_count; ++l) {
            const LoadLeg& g = load_legs[l];
            const SinkLeg leg = g.who == 1u ? sink_leg<SinkHigh>(g.who, baud, window_ms, g.copies, g.copies_high)
                                            : sink_leg<SinkNormal>(g.who, baud, window_ms, g.copies, g.copies_high);
            print(serial, "  sink at ", baud, ", the ring ", leg.high ? "HIGH" : "normal", " beside ", g.copies,
                  g.copies_high ? " HIGH_PRIORITY" : " normal", " 16 KB copies (", leg.blocks, " blocks): ",
                  leg.in, " byte(s) in, ", leg.gaps, " gap(s) of ", leg.lost, " lost, ", leg.bad,
                  " bad; ring overruns ", leg.rx_overruns, ", OE ", leg.hw_overruns, ", skips ",
                  leg.skips, ", line errors ", leg.line_errors, ", faults ", leg.faults, crlf);
            if (!leg.up || leg.in + leg.lost < baud / 10u / 2u) {
                fed = false;
            }
            if (leg.blocks < 10u) {
                loaded = false;
            }
            if (leg.high != (g.who == 1u)) {
                levels = false;
            }
            whole[l] = whole[l] && leg.gaps == 0u && leg.bad == 0u;
            if (leg.gaps != 0u && (leg.hw_overruns == 0u || leg.skips == 0u)) {
                reported = false;
            }
            device_clean[l] = device_clean[l] && leg.rx_overruns == 0u && leg.hw_overruns == 0u &&
                              leg.line_errors == 0u && leg.faults == 0u;
        }
    }
    bench.verdict("the host fed every leg (brio stress on the other end)", fed);
    bench.verdict("the copies ran the whole window, block after block", loaded);
    bench.verdict("the ring's channel carried the level asked: HIGH from the transport, normal "
                  "through the engine that ignores it", levels);
    bench.verdict("THE RING AT ITS DEFAULT LEVEL LOSES NOTHING, every byte in order at 115200, 1 Mbaud "
                  "and 3 Mbaud: beside two copies at their own level and beside two HIGH_PRIORITY ones",
                  whole[0] && whole[1] && device_clean[0] && device_clean[1]);
    bench.verdict("WHEREVER BYTES WERE LOST THE TRANSPORT SAID SO: every leg with a gap in the stream "
                  "counted an overrun and moved the ring's skips", reported);
    bench.verdict("A RING AT NORMAL LOSES NOTHING beside ONE HIGH_PRIORITY copy or two normal ones - "
                  "and beside TWO HIGH_PRIORITY copies it is starved on this chip, reported above and "
                  "not judged (docs/rp2350/dma.md): the level the transport arms it at is what keeps it",
                  whole[3] && whole[4] && device_clean[3] && device_clean[4]);
}

// -----------------------------------------------------------------------------
// r  A RELEASED REQUESTER (the release contract, no wire)
//
// docs/design/dma.md's release contract asks whether a peripheral's DMA
// request, once raised, outlives the peripheral's release. On this
// controller a request is a one-cycle PULSE the CHANNEL counts (12.6.4.2):
// each pulse on the DREQ the channel selects increments a counter of the
// channel's own, and a nonzero counter is what asks the arbiter for a
// transfer. So a request can outlive a release in two places - at the
// peripheral, if its DREQ goes on pulsing, and in the channel, whose count
// is no peripheral's. The letter measures both, with no wire.
//
// The requesters, none on a pad: UART1's transmit (TXDMAE over an empty
// FIFO's room), SPI1's transmit (TXDMAE, the same), PWM slice 3's wrap (a
// pulse a period, 100 cycles) and the ADC's FIFO (DREQ_EN over one
// sample). Each is brought up with its request standing and released one
// of four ways; then the INCOMING OWNER - channel 12 bound on the
// requester's DREQ, its credits read before the trigger, four items
// between memory and the requester's own register - is triggered and
// given 2000 cycles: every item it moves is a request that outlived the
// release.
//
//   A  the driver's release() - the request enable cleared, the block
//      disabled and held in reset - and then the incoming owner's own
//      reset: the hand-over as two transports make it (the PWM slice:
//      stopped, the block's reset being every slice's)
//   B  the request enable cleared, the block left enabled, no reset
//   C  the block disabled with its request enable left set, no reset
//   D  the control: nothing released - the request is live
//
// and two ways for the CHANNEL's half, the requester released as in A:
//
//   E  the channel bound on the live requester (EN set, untriggered) the
//      way an engine's arm() leaves it, then DmaChannel::stop() - which an
//      engine's stop() and arm() both run - and bound again on a request
//      that never comes (pacing timer 3 at X = 0): every item the incoming
//      owner moves is a credit the stop left in the channel
//   F  the same through the ENGINES' own verbs: the requester's engine
//      armed, then stop()ped, and the next owner's engine arm()ed on the
//      silent timer and started - what a driver hands its successor

using Det = DmaChannel<12>;
alignas(4) uint32_t ho_cells[4];
alignas(4) volatile uint32_t ho_sink = 0;

/// What the incoming owner saw: the credits on its channel before the
/// trigger, and the items it moved of four.
struct Seen {
    uint8_t credits = 0;
    uint8_t moved = 0;
    uint8_t stopped = 0;   ///< E and F: the credits the outgoing binding banked, then after stop()
    uint8_t banked = 0;
};

DmaTransfer toward(Dreq dreq, volatile void* reg, bool from_peripheral, DmaSize size) {
    DmaTransfer t{};
    if (from_peripheral) {
        t.read = reg;
        t.write = ho_cells;
        t.config.read_step = DmaStep::fixed;
    } else {
        t.read = ho_cells;
        t.write = reg;
        t.config.write_step = DmaStep::fixed;
    }
    t.count = 4;
    t.config.size = size;
    t.config.treq = dreq;
    return t;
}

Seen incoming(const DmaTransfer& t) {
    Seen s{};
    if (!Det::prepare(t)) {
        s.moved = 0xFFu;
        return s;
    }
    spin_us(2);
    s.credits = static_cast<uint8_t>(Det::debug_credits());
    Det::trigger();
    spin_us(16);   // 2400 cycles at 150 MHz
    s.moved = static_cast<uint8_t>(4u - Det::count());
    Det::stop();
    return s;
}

/// The channel's half: bound on the live requester, stop()ped, the
/// requester released, bound again on a silent request.
Seen channel_half(const DmaTransfer& live, void (*release)()) {
    Seen s{};
    (void)Det::prepare(live);
    spin_us(5);
    s.banked = static_cast<uint8_t>(Det::debug_credits());
    Det::stop();
    s.stopped = static_cast<uint8_t>(Det::debug_credits());
    release();
    DmaTimer<3>::set(0, 1);
    DmaTransfer silent{};
    silent.read = ho_cells;
    silent.write = &ho_sink;
    silent.count = 4;
    silent.config.size = DmaSize::word;
    silent.config.write_step = DmaStep::fixed;
    silent.config.treq = DmaTimer<3>::dreq();
    const Seen in = incoming(silent);
    s.credits = in.credits;
    s.moved = in.moved;
    return s;
}

/// The same through the engines' verbs: the requester's engine (a
/// transmit engine, or for the ADC a receive engine) armed and stopped,
/// the next owner's transmit engine armed on the silent timer and started.
using DetTx = DmaTxEngine<12, uint32_t>;
using DetRx = DmaRxEngine<12, uint16_t>;
template <typename Outgoing>
Seen engine_half(volatile void* reg, Dreq dreq, void (*release)()) {
    Seen s{};
    Outgoing::arm(reg, dreq, false, DmaReport::errors);
    spin_us(5);
    s.banked = static_cast<uint8_t>(Det::debug_credits());
    Outgoing::stop();
    s.stopped = static_cast<uint8_t>(Det::debug_credits());
    release();
    DmaTimer<3>::set(0, 1);
    DetTx::arm(&ho_sink, DmaTimer<3>::dreq(), false, DmaReport::errors);
    s.credits = static_cast<uint8_t>(Det::debug_credits());
    (void)DetTx::start(ho_cells, 4u);
    spin_us(16);
    s.moved = static_cast<uint8_t>(4u - Det::count());
    DetTx::stop();
    return s;
}

using U1 = Pl011<1>;
using S1 = Pl022<1>;
using Pw = PwmSlice<3>;

enum class Way : uint8_t { contract, enable_cleared, enable_left, live, channel, engines };

void u1_up() {
    (void)U1::reset();
    (void)U1::divisor(*uart_divisor(SysClock::pclk_hz, 115200));
    (void)U1::line_control({}, true);
    U1::enable(true);
    U1::dma_requests(true, false);
    spin_us(20);
}
void u1_release() {   // the transport's release(): requests off, disabled, held
    U1::dma_requests(false, false);
    U1::enable(false);
    U1::hold();
}
void s1_up() {
    (void)S1::reset();
    (void)S1::configure(SpiConfig{});
    S1::enable(true);
    S1::dma_requests(true, false);
    spin_us(20);
}
void s1_release() {   // the host's release(): disabled, held
    S1::enable(false);
    S1::hold();
}
void pw_up() {
    (void)Pwm::reset();
    (void)Pw::configure({.top = 99});
    Pw::enable(true);
    spin_us(20);
}
void pw_release() { Pw::enable(false); }
void adc_up() {
    (void)Adc::init(clock);
    Adc::select(AdcInput::temperature);
    Adc::fifo({.enable = true, .dreq = true, .threshold = 1});
    (void)Adc::read();
    spin_us(20);
}
void adc_release() { Adc::release(); }

Seen uart1_handover(Way how) {
    Det::stop();
    Det::clear_credits();
    u1_up();
    const DmaTransfer t = toward(Dreq::uart1_tx, &U1::regs().UARTDR, false, DmaSize::byte);
    Seen s{};
    switch (how) {
        case Way::contract: u1_release(); (void)U1::reset(); s = incoming(t); break;
        case Way::enable_cleared: U1::dma_requests(false, false); s = incoming(t); break;
        case Way::enable_left: U1::enable(false); s = incoming(t); break;
        case Way::live: s = incoming(t); break;
        case Way::channel: s = channel_half(t, u1_release); break;
        case Way::engines: s = engine_half<DmaTxEngine<12>>(&U1::regs().UARTDR, Dreq::uart1_tx, u1_release); break;
    }
    u1_release();
    return s;
}
Seen spi1_handover(Way how) {
    Det::stop();
    Det::clear_credits();
    s1_up();
    const DmaTransfer t = toward(Dreq::spi1_tx, S1::data_address(), false, DmaSize::half);
    Seen s{};
    switch (how) {
        case Way::contract: s1_release(); (void)S1::reset(); s = incoming(t); break;
        case Way::enable_cleared: S1::dma_requests(false, false); s = incoming(t); break;
        case Way::enable_left: S1::enable(false); s = incoming(t); break;
        case Way::live: s = incoming(t); break;
        case Way::channel: s = channel_half(t, s1_release); break;
        case Way::engines: s = engine_half<DmaTxEngine<12, uint16_t>>(S1::data_address(), Dreq::spi1_tx, s1_release); break;
    }
    s1_release();
    return s;
}
Seen pwm_handover(Way how) {
    Det::stop();
    Det::clear_credits();
    pw_up();
    const DmaTransfer t = toward(Pw::dreq, Pw::cc_address(), false, DmaSize::word);
    Seen s{};
    switch (how) {
        case Way::contract: pw_release(); s = incoming(t); break;
        case Way::enable_cleared:
        case Way::enable_left: s.moved = 0xFEu; break;   // no request enable: the wrap is the request
        case Way::live: s = incoming(t); break;
        case Way::channel: s = channel_half(t, pw_release); break;
        case Way::engines: s = engine_half<DmaTxEngine<12, uint32_t>>(Pw::cc_address(), Pw::dreq, pw_release); break;
    }
    pw_release();
    Pwm::hold();
    return s;
}
Seen adc_handover(Way how) {
    Det::stop();
    Det::clear_credits();
    adc_up();
    const DmaTransfer t = toward(Adc::dreq, Adc::fifo_address(), true, DmaSize::half);
    Seen s{};
    switch (how) {
        case Way::contract: adc_release(); (void)Adc::init(clock); s = incoming(t); break;
        case Way::enable_cleared: Adc::fifo({.enable = true, .dreq = false, .threshold = 1}); s = incoming(t); break;
        case Way::enable_left: Adc::enable(false); s = incoming(t); break;
        case Way::live: s = incoming(t); break;
        case Way::channel: s = channel_half(t, adc_release); break;
        case Way::engines: s = engine_half<DetRx>(Adc::fifo_address(), Adc::dreq, adc_release); break;
    }
    adc_release();
    return s;
}

void print_seen(const char* name, const Seen& s) {
    print(serial, "  ", name);
    if (s.moved == 0xFEu) {
        print(serial, "   -   ");
        return;
    }
    print(serial, " ", s.moved, "/", s.credits);
}

void tr_released_requester() {
    const char* names[6] = {"A release() + reset  ", "B enable cleared     ", "C disabled, enable on",
                            "D live (control)     ", "E channel stop()     ", "F engines stop/arm   "};
    Seen u[6];
    Seen sp[6];
    Seen pw[6];
    Seen ad[6];
    for (uint8_t i = 0; i < 6u; ++i) {
        u[i] = uart1_handover(static_cast<Way>(i));
        sp[i] = spi1_handover(static_cast<Way>(i));
        pw[i] = pwm_handover(static_cast<Way>(i));
        ad[i] = adc_handover(static_cast<Way>(i));
    }
    DmaTimer<3>::stop();
    print(serial, "  items moved of 4 / the incoming channel's credits before its trigger:", crlf);
    for (uint8_t i = 0; i < 6u; ++i) {
        print(serial, "  ", names[i], ":");
        print_seen("UART1_TX", u[i]);
        print_seen("SPI1_TX", sp[i]);
        print_seen("PWM_WRAP3", pw[i]);
        print_seen("ADC", ad[i]);
        print(serial, crlf);
    }
    for (uint8_t i = 4; i < 6u; ++i) {
        print(serial, "  ", names[i], ": banked by the outgoing binding / left after its stop():",
              " UART1_TX ", u[i].banked, "/", u[i].stopped, " SPI1_TX ", sp[i].banked, "/",
              sp[i].stopped, " PWM_WRAP3 ", pw[i].banked, "/", pw[i].stopped, " ADC ", ad[i].banked,
              "/", ad[i].stopped, crlf);
    }
    bench.verdict("THE CONTROL: a live request moves the incoming owner's items on all four "
                  "requesters - the detector sees a request (D)",
                  u[3].moved >= 1u && sp[3].moved >= 1u && pw[3].moved >= 1u && ad[3].moved >= 1u);
    bench.verdict("THE CONTRACT HANDS OVER CLEAN: a requester released by its driver and reset by "
                  "the next owner leaves that owner nothing (A)",
                  u[0].moved == 0u && sp[0].moved == 0u && pw[0].moved == 0u && ad[0].moved == 0u);
    bench.verdict("THE PL011 AND THE PL022 WITHDRAW A REQUEST: a cleared DMA enable or a disabled "
                  "block hands the next owner nothing, no reset needed (B, C) - 12.1.5's \"all "
                  "request signals are de-asserted\"",
                  u[1].moved == 0u && u[2].moved == 0u && sp[1].moved == 0u && sp[2].moved == 0u);
    bench.verdict("THE ADC HOLDS ONE: a converter disabled with DREQ_EN left set keeps its sample in "
                  "the FIFO and requests it - the next owner reads a stale sample (C); a cleared "
                  "DREQ_EN withdraws it (B)",
                  ad[2].moved >= 1u && ad[1].moved == 0u);
    bench.verdict("A STOPPED PWM SLICE RAISES NOTHING: its wrap is the request, and a slice that "
                  "does not count does not wrap (A)",
                  pw[0].moved == 0u);
    bench.verdict("THE CHANNEL'S CREDITS SURVIVE stop() AND A REBIND - banked by the PWM's pulses, "
                  "read on the incoming channel before its trigger - BUT THE TRIGGER DROPS THEM: "
                  "the next owner, on a request that never comes, moved nothing (E, F)",
                  pw[4].stopped >= 1u && pw[4].credits >= 1u && pw[5].credits >= 1u &&
                      u[4].moved == 0u && sp[4].moved == 0u && pw[4].moved == 0u &&
                      ad[4].moved == 0u && u[5].moved == 0u && sp[5].moved == 0u &&
                      pw[5].moved == 0u && ad[5].moved == 0u);
}


void banner() {
    print(serial, crlf, "test_rp2350_dma - the RP2350 DMA (datasheet 12.6) on ",
          core_kind == CoreKind::hazard3 ? "RISC-V Hazard3" : "Arm Cortex-M33",
          ", this console on a transmit engine, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() {
    if (uart0_owner == 0u) {
        (void)Serial::isr();
    } else if (uart0_owner == 1u) {
        (void)SinkHigh::isr();
    } else {
        (void)SinkNormal::isr();
    }
}
extern "C" void isr_uart1() {
    uart1_entries = uart1_entries + 1u;
    if (uart1_isr != nullptr) {
        (void)uart1_isr();
    }
}
extern "C" void isr_systick() { brio::Ticker::tick(); }

extern "C" void isr_dma_0() {
    line_irqs[0] = line_irqs[0] + 1u;
    if (C4::pending(0)) {
        C4::clear_pending(0);
        ch4_on_line[0] = ch4_on_line[0] + 1u;
    }
    if (brio::DmaChannel<0>::pending(0)) {
        serial_blocks = serial_blocks + 1u;   // counted before the transport clears it
    }
    if (uart0_owner == 0u) {
        (void)Serial::dma_isr();
    } else if (uart0_owner == 1u) {
        (void)SinkHigh::dma_isr();
    } else {
        (void)SinkNormal::dma_isr();
    }
    (void)Instrument::dma_isr();
    (void)TxOnly::dma_isr();
    (void)RxOnly::dma_isr();
    const uint8_t quiet = QuietTx::service();
    if (quiet != 0u) {
        quiet_flags = static_cast<uint8_t>(quiet_flags | quiet);
        quiet_entries = quiet_entries + 1u;
    }
}
extern "C" void isr_dma_1() {
    line_irqs[1] = line_irqs[1] + 1u;
    if (C4::pending(1)) {
        C4::clear_pending(1);
        ch4_on_line[1] = ch4_on_line[1] + 1u;
    }
    // Letter i's TRIGGER_SELF laps: the channel re-arms itself, so the
    // only way to stop it is to clear EN, which pauses it in place.
    if (C5::pending(1)) {
        C5::clear_pending(1);
        self_laps = self_laps + 1u;
        if (!self_running || self_laps >= 4u) {
            C5::enable(false);
        }
    }
}
extern "C" void isr_dma_2() {
    line_irqs[2] = line_irqs[2] + 1u;
    if (C4::pending(2)) {
        C4::clear_pending(2);
        ch4_on_line[2] = ch4_on_line[2] + 1u;
    }
}
extern "C" void isr_dma_3() {
    line_irqs[3] = line_irqs[3] + 1u;
    if (C4::pending(3)) {
        C4::clear_pending(3);
        ch4_on_line[3] = ch4_on_line[3] + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::DmaLine<0>::enable();
    brio::DmaLine<1>::enable();
    brio::DmaLine<2>::enable();
    brio::DmaLine<3>::enable();
    brio::enable_interrupts();

    bench.letter('a', "the block as found, the refusals, the security read-back, an abort",
                 ta_as_found);
    bench.letter('b', "memory to memory at three widths, and timed", tb_memory);
    bench.letter('c', "THE FOUR LINES, IRQ_QUIET, the null trigger", tc_lines);
    bench.letter('d', "chaining", td_chain);
    bench.letter('e', "address wrapping, and the four address steps", te_steps);
    bench.letter('f', "a pacing timer", tf_pacing);
    bench.letter('g', "the sniffer: sum, CRC-16, CRC-32", tg_sniffer);
    bench.letter('h', "a bus error", th_bus_error);
    bench.letter('i', "THE COUNT MODES: trigger-self, then endless", ti_count_modes);
    bench.letter('j', "the instrument's two engines on the loop-back", tj_instrument);
    bench.letter('k', "this console's transmit engine under a burst", tk_console_engine);
    bench.letter('l', "the engines: the beat per block, errors alone, copy and fill", tl_engines);
    bench.letter('t', "tx_idle() on the pad under the transmit engine", tt_tx_idle);
    bench.letter('v', "K breaks in N slots: the interrupt receiver and the engine", tv_breaks);
    bench.letter('o', "the overrun a stalled receive engine leaves: no storm", to_stalled_overrun);
    bench.letter('r', "a released requester: what reaches the next owner", tr_released_requester);
    bench.letter('p', "HOST: the console's ring beside copies at each level (brio stress)",
                 tp_ring_beside_copies, false);
    bench.letter('m', "diagnostic: the transmit engine alone", tm_tx_only, false);
    bench.letter('n', "diagnostic: the receive engine alone", tn_rx_only, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " dma=",
                    dma_ok ? "released" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    brio::crlf);
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
