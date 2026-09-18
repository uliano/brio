// test_rp2350_dma - the reference bench suite for the RP2350's DMA
// controller (rp2350/dma.hpp, datasheet 12.6): sixteen channels with
// their trigger aliases, the FOUR interrupt lines, chaining, address
// wrapping and the four address steps, THE COUNT MODES in TRANS_COUNT's
// top nibble, the pacing timers, the checksum sniffer, the abort with
// erratum RP2350-E5's workaround, a bus error - and the two engines in a
// UART's slots, this console's own transmit side among them. ON BOTH
// ARCHITECTURES from one source.
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
//      byte-exact, the line's interrupts counted against the bytes
//   k  this console's transmit engine under a burst longer than its
//      ring: every byte out in order (the host reads it), the blocks
//      counted, no fault
//
//   m  (by name only) diagnostic: the transmit engine alone on the loop
//   n  (by name only) diagnostic: the receive engine alone on the loop
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
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
using Instrument = Uart<1, instrument_pins, 1024, 1024, DmaTxEngine<2>, DmaRxEngine<3>>;

using Led = Pin<25>;

TestBench<Serial, 16> bench;

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
    const uint32_t irqs = line_irqs[0] - irqs0;
    if (wrong) {
        uint8_t got[8] = {};
        (void)Instrument::read_bulk(got);
        print(serial, "  MISMATCH after ", good, " good bytes: sent ", sent,
              ", the next received ", hex(got[0]), " ", hex(got[1]), " ", hex(got[2]), " ",
              hex(got[3]), crlf);
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
    T::release();
    uart1_isr = nullptr;
}
void tm_tx_only() { half_loop<TxOnly>("the transmit engine alone, the receiver on its interrupt"); }
void tn_rx_only() { half_loop<RxOnly>("the receive engine alone, the transmitter on its interrupt"); }

void banner() {
    print(serial, crlf, "test_rp2350_dma - the RP2350 DMA (datasheet 12.6) on ",
          core_kind == CoreKind::hazard3 ? "RISC-V Hazard3" : "Arm Cortex-M33",
          ", this console on a transmit engine, clk=", SysClock::hz, " Hz", crlf);
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
    line_irqs[0] = line_irqs[0] + 1u;
    if (C4::pending(0)) {
        C4::clear_pending(0);
        ch4_on_line[0] = ch4_on_line[0] + 1u;
    }
    if (brio::DmaChannel<0>::pending(0)) {
        serial_blocks = serial_blocks + 1u;   // counted before the transport clears it
    }
    (void)Serial::dma_isr();
    (void)Instrument::dma_isr();
    (void)TxOnly::dma_isr();
    (void)RxOnly::dma_isr();
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
    (void)Led::output();
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
