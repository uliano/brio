// test_stm32f4_serial - the reference bench suite for the STM32F4's serial
// TRANSPORT beyond the console personality: the Uart task over a USART
// that hears itself, with the interrupt receiver and with the receive
// engine, and what the transport promises about the wire - stm32f4/
// usart.hpp, with the engines of stm32f4/dma.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE: SINGLE-WIRE HALF DUPLEX IS THE LOOP. With CR3.HDSEL the
// TX and RX lines are joined inside the chip and the receiver listens on
// the TX pad (RM0390 25.4.10), so everything the instance sends it also
// receives. The loop is USART6 on PC6 (AF8, DS10693 table 11) - an APB2
// instance, so its top rate is 90 MHz / 16 (5.625 Mbaud) and 90 MHz / 8
// at OVER8 - on a pad whose other ends stay quiet while the suite runs
// (on the bench PC6 is also the I2C self-link's SCL, with its 2.2 kOhm
// pull-up), its engines on DMA2's stream 6 out and stream 1 in, channel 5
// (RM0390 table 29). The pad is driven PUSH-PULL while a frame goes out
// (UartOptions::single_wire_push_pull), the pull-up holding the line the
// transmitter releases between frames: the open drain rises on the
// pull-up alone, and letter a measures where it gives out. The pad's own
// input feeds EXTI line 6 in alternate-function mode, which is how letter
// e sees the stop bit start on the pad.
//
// What is exercised, letter by letter:
//   a  the loop proven: 256 bytes out and back whole and in order through
//      both receivers at 115200, 1 Mbaud and 5.625 Mbaud, and at OVER8's
//      11.25 Mbaud, timed against the wire - a loop alone cannot tell a
//      wrong rate; and the open drain's ceiling, where it gives out
//   b  the frame formats on the loop: 8N1, 8E1, 8O1, 7E1, 7O1, 8N2,
//      byte-exact through the receive engine
//   c  ONEBIT and OVER8 together at 2 Mbaud, timed against the wire, and
//      the error counters at zero
//   d  ERRORS UNDER THE ENGINE: breaks injected into a stream through the
//      receive engine - every data byte delivered intact and in order,
//      the frame errors counted one a break, no byte taken by a clear;
//      back-to-back breaks counting every other one (the bound the
//      channel's read leaves, usart.hpp's header); and the interrupt
//      receiver's breaks, dropped, counted and skipped - a reader that
//      looks between them handed every data byte, one that does not
//      handed nothing the ring held with a break behind it
//   e  tx_idle() IS THE WIRE'S: the moment it turns true against the last
//      stop bit's start on the pad (EXTI line 6, rising), for the
//      interrupt transmitter and the transmit engine - never before the
//      stop bit's end, within a bit time after it
//   f  THE BURST EDGE FROM THE VECTOR, nothing polled: a burst of one
//      frame told, a burst of 16 told within two frames of its last stop
//      bit, two interrupts a burst and none a byte, and a stream of four
//      laps with no silence in it read whole through the lap's marks
//   g  THE RING AGAINST COPIES ON ITS OWN CONTROLLER (DMA2): the receive
//      stream at its default level and at rx_priority = low, beside zero,
//      two, three and four copies at very_high, at 5.625 and 11.25 Mbaud
//      - the default never loses a byte, two copies starve nothing, four
//      starve the ring armed low; and the overrun a starved stream leaves
//      with nothing to take cleared, the vector quiet after it
//
// THE TWO-BOARD SET, under the second all-key t (z never runs it): USART1
// on PA9/PA10 (AF7) crossed to the desk's STM32G0B1RE running its
// usart_peer, driven in band with usart_link.hpp - every verdict of a rate
// is a FOREIGN receiver's, the far core tracked against its LSE crystal:
//   1  the rendezvous, the refusal path, and this crystal against the far
//      chip's tracked core both ways
//   2  the baud matrix to 6.4 Mbaud: an echo, this transmitter timed by
//      the far chip, the far blast on the polled receiver, the interrupt
//      receiver and the receive engine (DMA2 stream 2 channel 4)
//   3  the frame matrix across the wire, what this USART has no word for
//      SKIPPED with its reason, and the half stop bits timed as lengths
//   4  the receiver's errors from a foreign sender: PE, the FE onset, ORE,
//      the start bit's vote with ONEBIT clear and set, the break with and
//      without LIN
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: the far board's usart_peer
// compiles the same file.
#include "../../../avrdx/src/apps/usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 't'> bench;

// ---- the loop ---------------------------------------------------------------
constexpr UartPins loop_pins{.tx = {'C', 6, PinFunction::af8}, .rx = {'C', 7, PinFunction::af8}};
constexpr UartOptions loop_opts{.half_duplex = true, .tx_speed = PinSpeed::very_high,
                                .single_wire_push_pull = true};
constexpr UartOptions loop_opts_od{.half_duplex = true, .tx_speed = PinSpeed::very_high};
constexpr UartOptions loop_opts_fast{.over8 = true, .one_bit = true, .half_duplex = true,
                                     .tx_speed = PinSpeed::very_high,
                                     .single_wire_push_pull = true};
using TxEngine = DmaTxEngine<2, 6, 5>;
using RxEngine = DmaRxEngine<2, 1, 5>;
constexpr uint32_t ring = 64;   // a lap of the receive stream: letter f runs four of them
using LoopIrq = Uart<6, loop_pins, 512, 512, NoDmaEngine, NoDmaEngine, loop_opts>;
using LoopDma = Uart<6, loop_pins, ring, 512, TxEngine, RxEngine, loop_opts>;
using LoopOd = Uart<6, loop_pins, 512, 512, NoDmaEngine, NoDmaEngine, loop_opts_od>;
using LoopFast = Uart<6, loop_pins, 512, 512, TxEngine, RxEngine, loop_opts_fast>;
/// Letter g's two receivers: the receive engine alone (the transmitter is
/// the thread's, so a starved stream cannot slow the line), over a ring
/// that holds a whole leg, at the default level and ranked low.
constexpr uint32_t arb_ring = 8192;
constexpr UartOptions arb_opts_low{.over8 = true, .one_bit = true, .half_duplex = true,
                                   .tx_speed = PinSpeed::very_high,
                                   .single_wire_push_pull = true,
                                   .rx_priority = DmaPriority::low};
using RingHigh = Uart<6, loop_pins, arb_ring, 64, NoDmaEngine, RxEngine, loop_opts_fast>;
using RingLow = Uart<6, loop_pins, arb_ring, 64, NoDmaEngine, RxEngine, arb_opts_low>;
using Res = Usart<6>;

/// Who owns USART6 now: 0 nobody, 1 LoopIrq, 2 LoopDma, 3 LoopOd, 4 LoopFast,
/// 5 RingHigh, 6 RingLow.
volatile uint8_t owner = 0;
volatile uint32_t usart_entries = 0;
volatile uint32_t dma_entries = 0;
volatile uint32_t edges = 0;
volatile uint32_t edge_at = 0;   ///< the DWT at the last edge
volatile uint32_t pad_edge_at = 0;   ///< the DWT at PC6's last rising edge

void down() {
    const uint8_t was = owner;
    if (was == 1u) {
        LoopIrq::release();
    } else if (was == 2u) {
        LoopDma::release();
    } else if (was == 3u) {
        LoopOd::release();
    } else if (was == 4u) {
        LoopFast::release();
    } else if (was == 5u) {
        RingHigh::release();
    } else if (was == 6u) {
        RingLow::release();
    }
    owner = 0;   // after the release: a stopped stream raises its completion
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 <= ms) {
    }
}

template <typename Port>
bool up(uint8_t who, uint32_t baud, const UartFormat& f = {}) {
    down();
    owner = who;
    const bool ok = Port::init(clock, baud, f);
    wait_ms(1);
    uint8_t b = 0;
    while (Port::read_byte(b)) {
    }
    Port::clear_errors();
    edges = 0;
    usart_entries = 0;
    dma_entries = 0;
    return ok;
}

constexpr uint8_t pattern(uint32_t i) { return static_cast<uint8_t>(i * 151u + 7u); }

/// Send `n` bytes of the pattern from `from`, waiting on the wire.
template <typename Port>
void send(uint32_t from, uint32_t n) {
    static uint8_t block[256];
    uint32_t done = 0;
    while (done < n) {
        const uint32_t k = n - done < sizeof(block) ? n - done : static_cast<uint32_t>(sizeof(block));
        for (uint32_t i = 0; i < k; ++i) {
            block[i] = pattern(from + done + i);
        }
        uint32_t q = 0;
        while (q < k) {
            q += Port::write_bulk(std::span<const uint8_t>(block + q, k - q));
        }
        done += k;
    }
}

/// Everything the loop holds, against the pattern from `from` (masked to
/// `mask` for a seven-bit frame).
template <typename Port>
uint32_t drain(uint32_t from, bool& in_order, uint8_t mask = 0xFFu) {
    uint32_t got = 0;
    for (;;) {
        const auto run = Port::read_span();
        if (run.empty()) {
            return got;
        }
        for (uint32_t i = 0; i < run.size(); ++i) {
            if ((run[i] & mask) != (pattern(from + got + i) & mask)) {
                in_order = false;
            }
        }
        got += static_cast<uint32_t>(run.size());
        (void)Port::consume(static_cast<uint32_t>(run.size()));
    }
}

/// THE RATE ON THE WIRE, which a loop cannot tell: both ends share one
/// divisor, so a wrong one is a slower loop and nothing else. 1024 frames
/// sent back to back through the transmit engine are timed on the cycle
/// counter from the first store to TC, against 1024 frames of ten bits at
/// `baud` - enough that the run's fixed cost, some 1700 cycles, stays under
/// a per cent at 11.25 Mbaud; the answer is in thousandths of the wire's
/// time.
template <typename Port>
uint32_t wire_permille(uint32_t baud) {
    static uint8_t frames[1024];
    for (uint32_t i = 0; i < sizeof(frames); ++i) {
        frames[i] = pattern(i);
    }
    Res::clear_flags(UsartFlag::tc);
    const uint32_t t0 = CycleCounter::now();
    uint32_t q = 0;
    while (q < sizeof(frames)) {
        q += Port::write_bulk(std::span<const uint8_t>(frames + q, sizeof(frames) - q));
    }
    while (!Res::tx_complete() || !Port::tx_idle()) {
        if (CycleCounter::now() - t0 > SysClock::hz / 100u) {
            break;
        }
    }
    const uint32_t took = CycleCounter::now() - t0;
    const uint64_t wire = 10ull * sizeof(frames) * SysClock::hz / baud;
    wait_ms(1);
    uint8_t b = 0;
    while (Port::read_byte(b)) {
    }
    return static_cast<uint32_t>(1000ull * took / wire);
}

/// Send n, wait for the wire and a few frames more, read it all back.
template <typename Port>
bool round_trip(uint32_t n, uint32_t& got, uint8_t mask = 0xFFu) {
    send<Port>(0, n);
    const uint32_t t0 = Ticker::millis();
    got = 0;
    bool in_order = true;
    while (Ticker::millis() - t0 < 200u && got < n) {
        got += drain<Port>(got, in_order, mask);
    }
    return in_order && got == n;
}

// ---- a  the loop proven ----------------------------------------------------------
void ta_loop() {
    static constexpr uint32_t rates[] = {115200, 1'000'000, 5'625'000};
    for (const uint32_t baud : rates) {
        uint32_t got = 0;
        const bool up_i = up<LoopIrq>(1, baud);
        const bool ok_i = round_trip<LoopIrq>(256, got);
        print(serial, "  ", baud, " baud, interrupt receiver: ", got, " of 256 back, ORE ",
              LoopIrq::hw_overruns(), crlf);
        const bool up_d = up<LoopDma>(2, baud);
        uint32_t got_d = 0;
        const bool ok_d = round_trip<LoopDma>(256, got_d);
        print(serial, "  ", baud, " baud, receive engine:    ", got_d, " of 256 back, laps missed ",
              LoopDma::rx_overruns(), crlf);
        bench.verdict("the loop comes up at ", baud == 115200 ? "115200" : baud == 1'000'000 ? "1 Mbaud" : "5.625 Mbaud",
                      up_i && up_d);
        bench.verdict("256 bytes back whole and in order through both receivers", ok_i && ok_d);
    }
    // OVER8 doubles the top: 90 MHz / 8, with the receive engine.
    uint32_t got = 0;
    const bool up_f = up<LoopFast>(4, 11'250'000);
    const bool ok_f = round_trip<LoopFast>(256, got);
    const uint32_t pm = wire_permille<LoopFast>(11'250'000);
    print(serial, "  11.25 Mbaud (OVER8, ONEBIT), receive engine: ", got, " of 256 back, BRR ",
          hex(Res::brr()), "; 1024 frames out in ", pm, " thousandths of their wire time", crlf);
    bench.verdict("OVER8 reaches APB2 / 8 = 11.25 Mbaud and the loop carries it", up_f && ok_f);
    bench.verdict("and the line runs at the rate asked: 1024 frames in their wire time within "
                  "three per cent", pm >= 1000u && pm <= 1030u);
    // The open drain: where the pull-up's rise gives out.
    static constexpr uint32_t od_rates[] = {1'000'000, 2'812'500};
    bool od_slow = false;
    bool od_fast = false;
    for (const uint32_t baud : od_rates) {
        (void)up<LoopOd>(3, baud);
        const bool ok = round_trip<LoopOd>(64, got);
        print(serial, "  open drain at ", baud, ": ", got, " of 64 back", ok ? " whole" : "", crlf);
        if (baud == 1'000'000u) {
            od_slow = ok;
        } else {
            od_fast = ok;
        }
    }
    down();
    bench.verdict("the open drain on the internal pull-up carries 1 Mbaud", od_slow);
    print(serial, "  and ", od_fast ? "carries" : "does not carry", " 2.8125 Mbaud", crlf);
}

// ---- b  the frame formats ----------------------------------------------------------
void tb_formats() {
    struct Leg {
        const char* name;
        UartFormat f;
        uint8_t mask;
    };
    static constexpr Leg legs[] = {
        {"8N1", {UartBits::eight, UartParity::none, UartStop::one}, 0xFFu},
        {"8E1", {UartBits::eight, UartParity::even, UartStop::one}, 0xFFu},
        {"8O1", {UartBits::eight, UartParity::odd, UartStop::one}, 0xFFu},
        {"7E1", {UartBits::seven, UartParity::even, UartStop::one}, 0x7Fu},
        {"7O1", {UartBits::seven, UartParity::odd, UartStop::one}, 0x7Fu},
        {"8N2", {UartBits::eight, UartParity::none, UartStop::two}, 0xFFu},
    };
    for (const Leg& l : legs) {
        const bool ok_up = up<LoopDma>(2, 1'000'000, l.f);
        uint32_t got = 0;
        const bool ok = round_trip<LoopDma>(200, got, l.mask);
        print(serial, "  ", l.name, ": ", got, " of 200 back, FE ", LoopDma::frame_errors(), " PE ",
              LoopDma::parity_errors(), crlf);
        bench.verdict("byte-exact on the loop at 1 Mbaud: ", l.name,
                      ok_up && ok && LoopDma::frame_errors() == 0u && LoopDma::parity_errors() == 0u);
    }
    down();
}

// ---- c  ONEBIT and OVER8 -----------------------------------------------------------
void tc_sampling() {
    uint32_t got = 0;
    const bool ok_up = up<LoopFast>(4, 2'000'000);
    const bool ok = round_trip<LoopFast>(256, got);
    const uint32_t pm = wire_permille<LoopFast>(2'000'000);
    print(serial, "  OVER8 + ONEBIT at 2 Mbaud: BRR ", hex(Res::brr()), ", ", got, " of 256, NE ",
          LoopFast::noise_errors(), "; 1024 frames out in ", pm,
          " thousandths of their wire time", crlf);
    bench.verdict("USARTDIV is fCK / (8 x baud) at OVER8 (RM0390 25.4.4): 2 Mbaud on the wire, "
                  "1024 frames in their wire time within three per cent", pm >= 1000u && pm <= 1030u);
    bench.verdict("OVER8 lays the divisor out in eighths, bit 3 clear", (Res::brr() & 0x8u) == 0u &&
                                                                            Res::oversampling8());
    bench.verdict("ONEBIT and OVER8 carry the loop byte-exact, no noise counted",
                  ok_up && ok && LoopFast::noise_errors() == 0u);
    down();
}

// ---- d  errors under the engine -----------------------------------------------------
/// The waits below read no SR: a status read is half of every receive
/// clear on this block, and one landing between an errored frame's flag
/// and the stream's read of it would clear the flag before the vector
/// counts it (the window letter d's last leg measures). So a frame's end
/// is waited for on the cycle counter - two frame times at 115200, the
/// stream's read and the vector included.
void two_frames() {
    const uint32_t t0 = CycleCounter::now();
    while (CycleCounter::now() - t0 < 2u * SysClock::hz / 11'520u) {
    }
}

/// Send a break (SBK, cleared by the hardware during the break's stop
/// bit - a CR1 read) and wait it out.
void put_break() {
    Res::send_break();
    while (Res::break_pending()) {
    }
    two_frames();
}

/// Send one byte on an idle transmitter and wait it out.
void put_byte(uint8_t b) {
    Res::write_data(b);
    two_frames();
}

void td_errors() {
    // The stream: n data bytes, a break after every `every`-th of them -
    // each break followed by a clean frame, so the channel's read that
    // clears it is a clean frame's.
    constexpr uint32_t n = 120;
    constexpr uint32_t every = 8;
    (void)up<LoopDma>(2, 115200);
    uint32_t breaks = 0;
    uint32_t got = 0;
    uint32_t data = 0;
    uint32_t zeros = 0;
    bool in_order = true;
    for (uint32_t i = 0; i < n; ++i) {
        put_byte(pattern(i));
        if (i % every == every - 1u && i + 1u < n) {
            put_break();
            ++breaks;
        }
        // Read as it comes, a lap being 64.
        for (;;) {
            const auto run = LoopDma::read_span();
            if (run.empty()) {
                break;
            }
            for (const uint8_t b : run) {
                ++got;
                if (b == pattern(data)) {
                    ++data;
                } else if (b == 0u) {
                    ++zeros;   // the break's frame: 0x00 with FE, the stream stores it
                } else {
                    in_order = false;
                }
            }
            (void)LoopDma::consume(static_cast<uint32_t>(run.size()));
        }
    }
    wait_ms(2);
    {
        const auto run = LoopDma::read_span();
        for (const uint8_t b : run) {
            ++got;
            if (b == pattern(data)) {
                ++data;
            } else if (b == 0u) {
                ++zeros;
            } else {
                in_order = false;
            }
        }
        (void)LoopDma::consume(static_cast<uint32_t>(run.size()));
    }
    const uint8_t fe = LoopDma::frame_errors();
    print(serial, "  ", n, " data bytes and ", breaks, " breaks through the receive engine: ", data,
          " data bytes in order, ", zeros, " break frames, ", got, " stored; FE ", fe, ", ORE ",
          LoopDma::hw_overruns(), ", laps missed ", LoopDma::rx_overruns(), crlf);
    bench.verdict("every data byte delivered, intact and in order: no byte taken by a clear",
                  data == n && in_order);
    bench.verdict("each break stored as the frame it is (0x00)", zeros == breaks);
    bench.verdict("the frame errors counted one a break", fe == breaks);

    // Back to back: the channel's read of the second break clears the
    // flag the count's status read saw - the same bit. Two count one,
    // three count two (the bound in usart.hpp's header).
    (void)up<LoopDma>(2, 115200);
    put_byte(0x11);
    put_break();
    put_break();
    put_byte(0x22);
    wait_ms(2);
    const uint8_t two = LoopDma::frame_errors();
    (void)up<LoopDma>(2, 115200);
    put_byte(0x11);
    put_break();
    put_break();
    put_break();
    put_byte(0x22);
    wait_ms(2);
    const uint8_t three = LoopDma::frame_errors();
    print(serial, "  back to back: two breaks count ", two, ", three count ", three, crlf);
    bench.verdict("back-to-back breaks count every other one (two count 1, three count 2)",
                  two == 1u && three == 2u);

    // The window: the same ten breaks, each followed by a clean frame,
    // with the thread reading SR as fast as it can the whole time - a
    // status read between an errored frame's flag and the stream's read
    // of that frame clears the flag unseen.
    (void)up<LoopDma>(2, 115200);
    for (uint32_t i = 0; i < 10u; ++i) {
        Res::send_break();
        const uint32_t t0 = CycleCounter::now();
        while (CycleCounter::now() - t0 < 2u * SysClock::hz / 11'520u) {
            (void)Res::status();
        }
        put_byte(0x33);
    }
    print(serial, "  ten breaks under a thread that polls SR: ", LoopDma::frame_errors(),
          " counted", crlf);

    // The interrupt receiver drops what arrived with an error, counts it
    // and tells its ring, whose next look skips what it holds: a reader
    // that looks between the breaks is handed every data byte ...
    (void)up<LoopIrq>(1, 115200);
    breaks = 0;
    bool ok = true;
    uint32_t back = 0;
    const uint32_t skips0 = LoopIrq::rx_skips();
    for (uint32_t i = 0; i < 40u; ++i) {
        put_byte(pattern(i));
        if (i % 4u == 3u) {
            back += drain<LoopIrq>(back, ok);   // the four before the break
            put_break();
            ++breaks;
            back += drain<LoopIrq>(back, ok);   // the skip, with nothing queued
        }
    }
    const uint32_t skips = (LoopIrq::rx_skips() - skips0) & 0xFFu;
    print(serial, "  the interrupt receiver, read between: 40 bytes and ", breaks, " breaks, ",
          back, " delivered, FE ", LoopIrq::frame_errors(), ", skips ", skips, crlf);
    bench.verdict("the interrupt receiver drops each break's frame, counts it and skips it: "
                  "a reader that looks between the breaks is handed every data byte",
                  back == 40u && ok && LoopIrq::frame_errors() == breaks && skips == breaks);
    // ... and one that does not is handed nothing the ring held with a
    // break behind it: no run joins the two sides.
    const uint32_t skips1 = LoopIrq::rx_skips();
    for (uint32_t i = 0; i < 8u; ++i) {
        put_byte(pattern(40u + i));
        if (i == 3u) {
            put_break();
        }
    }
    wait_ms(2);
    bool joined_ok = true;
    const uint32_t joined = drain<LoopIrq>(40, joined_ok);
    const uint32_t skips_after = (LoopIrq::rx_skips() - skips1) & 0xFFu;
    print(serial, "  read after: 8 bytes around a break, ", joined, " delivered, FE ",
          LoopIrq::frame_errors(), ", skips ", skips_after, crlf);
    bench.verdict("bytes queued with a break among them are skipped whole, in one skip",
                  joined == 0u && skips_after == 1u && LoopIrq::frame_errors() == breaks + 1u);
    down();
}

// ---- e  tx_idle() is the wire's -----------------------------------------------------
using PadLine = ExtiLine<6>;

/// Four 0x55 frames (bit 7 low, so the stop bit starts with a rising
/// edge) through `Port`; the DWT where tx_idle() first answered true,
/// against PC6's last rising edge.
template <typename Port>
int32_t idle_after_stop(uint8_t who, uint32_t baud, uint32_t& bit_cycles) {
    static const uint8_t frames[4] = {0x55, 0x55, 0x55, 0x55};
    (void)up<Port>(who, baud);
    bit_cycles = SysClock::hz / Port::actual_baud(Port::template kernel_hz<SysClock>());
    (void)Exti::select(6, 'C');
    (void)PadLine::configure(ExtiSense::rising);
    (void)Exti::clear(6);
    (void)PadLine::arm(true);
    Nvic::enable(PadLine::irq());
    // The console quiet first: its transmit interrupt, entered just as
    // the stop bit rises, holds the edge's handler off and the stamp comes
    // late, which reads tx_idle() as true tens of cycles BEFORE the stop
    // bit's end (measured at 115200).
    for (uint32_t i = 0; i < 20'000'000u && !Serial::tx_idle(); ++i) {
    }
    (void)Port::write_bulk(std::span<const uint8_t>(frames, sizeof(frames)));
    uint32_t t_idle = 0;
    const uint32_t t0 = CycleCounter::now();
    // The stamp is taken AFTER the true answer: TC stood when SR was read,
    // so the count read after it is never early - late by a poll's few
    // cycles at most.
    for (;;) {
        if (Port::tx_idle()) {
            t_idle = CycleCounter::now();
            break;
        }
        if (CycleCounter::now() - t0 > SysClock::hz / 100u) {
            break;
        }
    }
    wait_ms(1);
    const uint32_t edge = pad_edge_at;
    // The edge's stamp is its handler's: the latency from a software
    // trigger of the same line to the stamp, taken off.
    const uint32_t t_sw = CycleCounter::now();
    (void)PadLine::trigger();
    wait_ms(1);
    const uint32_t latency = pad_edge_at - t_sw;
    (void)PadLine::arm(false);
    Nvic::disable(PadLine::irq());
    return static_cast<int32_t>(t_idle - edge + latency);
}

void te_tx_idle() {
    static constexpr uint32_t rates[] = {115200, 1'000'000};
    for (const uint32_t baud : rates) {
        uint32_t bit = 0;
        const int32_t d_irq = idle_after_stop<LoopIrq>(1, baud, bit);
        const int32_t d_dma = idle_after_stop<LoopDma>(2, baud, bit);
        print(serial, "  ", baud, " baud, a bit ", bit, " cycles: tx_idle() true ", d_irq,
              " cycles after the stop bit's rising edge (interrupt transmitter), ", d_dma,
              " (transmit engine)", crlf);
        // Never before the stop bit's end, within a bit time after it, the
        // edge's handler latency taken off; the thread's poll is a dozen
        // cycles a turn.
        const int32_t lo = static_cast<int32_t>(bit);
        const int32_t hi = static_cast<int32_t>(2u * bit);
        bench.verdict("tx_idle() never before the last stop bit is out, within a bit after it: ",
                      baud == 115200 ? "115200" : "1 Mbaud",
                      d_irq >= lo && d_irq <= hi && d_dma >= lo && d_dma <= hi);
    }
    down();
}

// ---- f  the burst edge from the vector ------------------------------------------------
void tf_edge() {
    (void)up<LoopDma>(2, 1'000'000);
    const uint32_t frame = SysClock::hz / 100'000u;   // ten bits at 1 Mbaud

    // One frame alone: no idle of its own follows a frame that clears an
    // idle the vector saw - its edge is the first-frame turn's.
    for (uint32_t k = 0; k < 3u; ++k) {   // the first burst after init, then two after an idle
        bool ok = true;
        (void)drain<LoopDma>(0, ok);
        const uint32_t e0 = edges;
        put_byte(0x42);
        wait_ms(1);
        const uint32_t e1 = edges;
        const uint32_t got = drain<LoopDma>(0, ok);
        print(serial, "  a burst of one frame: ", e1 - e0, " edge(s), ", got, " byte read", crlf);
        bench.verdict("a burst of one frame is told", e1 - e0 >= 1u && got == 1u);
    }

    // Sixteen frames: the vectors' entries, the last edge after the stop bit.
    bool ok = true;
    (void)drain<LoopDma>(0, ok);
    wait_ms(1);
    usart_entries = 0;
    dma_entries = 0;
    const uint32_t e0 = edges;
    Res::clear_flags(UsartFlag::tc);
    send<LoopDma>(0, 16);
    uint32_t got = 0;
    uint32_t seen0 = edges;
    while (!Res::tx_complete()) {
        if (edges != seen0) {   // a first frame's edge: the consumer drains, as it would
            seen0 = edges;
            got += drain<LoopDma>(got, ok);
        }
    }
    const uint32_t t_tc = CycleCounter::now();
    const uint32_t before_tc = edges;
    const uint32_t t0 = Ticker::millis();
    while (edges == before_tc && Ticker::millis() - t0 < 5u) {
    }
    const uint32_t late = edge_at - t_tc;
    wait_ms(1);
    got += drain<LoopDma>(got, ok);
    print(serial, "  16 frames: ", edges - e0, " edge(s), the last ", late, " cycles after the last stop bit (",
          late * 10u / frame, " tenths of a frame); ", usart_entries, " USART and ", dma_entries,
          " stream interrupt(s); ", got, " read", crlf);
    bench.verdict("the burst read whole, nothing polled", got == 16u && ok);
    bench.verdict("its edge within two frame times of the last stop bit",
                  edges != before_tc && late <= 2u * frame);
    bench.verdict("at most two USART interrupts a burst, none a byte",
                  usart_entries >= 1u && usart_entries <= 2u);

    // Four laps of the 64-byte ring with no silence: the marks tell the
    // consumer twice a lap, so it reads every byte.
    (void)drain<LoopDma>(0, ok);
    LoopDma::clear_errors();
    const uint32_t d0 = dma_entries;
    send<LoopDma>(0, 4u * ring);
    uint32_t read = 0;
    ok = true;
    const uint32_t t1 = Ticker::millis();
    uint32_t seen = edges;
    while (read < 4u * ring && Ticker::millis() - t1 < 50u) {
        if (edges != seen) {
            seen = edges;
            read += drain<LoopDma>(read, ok);
        }
    }
    print(serial, "  four laps without silence: ", read, " of ", 4u * ring, " read on the edges, ",
          dma_entries - d0, " stream interrupt(s), laps missed ", LoopDma::rx_overruns(), crlf);
    bench.verdict("a stream with no silence is read whole on the lap's marks",
                  read == 4u * ring && ok && LoopDma::rx_overruns() == 0u);
    down();
}

// ---- g  the receive ring against copies on its own controller -------------------------
//
// The loop's receive stream is DMA2's stream 1; DMA2 is also the one
// controller that copies memory to memory. Each leg sends `arb_chars`
// frames from the THREAD - a store into DR eleven bit times after the
// last one at the earliest, by the cycle counter, no SR read (a status
// read is half of every receive clear) and no stream (a starved transmit
// stream would only slow the line) - while zero to four copies run back
// to back on DMA2's streams 0, 4, 5 and 7 at very_high, and the ring -
// big enough for the whole leg - is read only at the end and judged
// against the pattern. The ring is armed at its default level and,
// through UartOptions::rx_priority, at low.

alignas(16) uint32_t copy_src[4][1024];
alignas(16) uint32_t copy_dst[4][1024];
using CopyA = DmaCopyEngine<2, 0>;   ///< below the ring's stream number
using CopyB = DmaCopyEngine<2, 4>;   ///< above it, as the other two
using CopyC = DmaCopyEngine<2, 5>;
using CopyD = DmaCopyEngine<2, 7>;
using RingStream = DmaStream<2, 1>;

constexpr uint32_t arb_chars = 4096;

struct ArbLeg {
    uint32_t got = 0;       ///< bytes the ring holds
    uint32_t lost = 0;      ///< positions of the pattern skipped over
    uint32_t gaps = 0;      ///< places they were skipped
    uint32_t copies = 0;    ///< copy blocks started under the stream
    uint32_t entries = 0;   ///< USART6 vector entries during the leg
    uint32_t after = 0;     ///< and in the millisecond of silence after it
    uint32_t late = 0;      ///< the longest a store into DR waited past its time, cycles
    uint8_t ore = 0;
    uint8_t faults = 0;
    DmaPriority level = DmaPriority::low;   ///< the ring stream's SxCR.PL, read back
};

/// The ring's content against the pattern: a byte that is not the next
/// position's is looked for further on (the pattern's period is 256), the
/// positions stepped over LOST.
template <typename Port>
void arb_judge(ArbLeg& leg) {
    uint32_t pos = 0;
    for (;;) {
        const auto run = Port::read_span();
        if (run.empty()) {
            break;
        }
        for (const uint8_t b : run) {
            uint32_t k = 0;
            while (k < 256u && pattern(pos + k) != b) {
                ++k;
            }
            if (k != 0u) {
                leg.lost += k;
                ++leg.gaps;
            }
            pos += k + 1u;
            ++leg.got;
        }
        (void)Port::consume(static_cast<uint32_t>(run.size()));
    }
}

/// One copy engine kept busy: a block started whenever the last ended,
/// 4 KB of word beats or of byte beats, sixteen bytes a burst.
template <typename E>
uint32_t keep_busy(uint8_t i, bool bytes) {
    if (E::busy()) {
        return 0;
    }
    const bool ok = bytes ? E::copy(reinterpret_cast<uint8_t*>(copy_dst[i]),
                                    reinterpret_cast<const uint8_t*>(copy_src[i]), 4096u)
                          : E::copy(copy_dst[i], copy_src[i], 1024u);
    return ok ? 1u : 0u;
}

template <typename Port>
ArbLeg arb_leg(uint8_t who, uint32_t baud, uint8_t copies, bool bytes) {
    ArbLeg leg{};
    (void)up<Port>(who, baud);
    CopyA::arm(DmaPriority::very_high);
    CopyB::arm(DmaPriority::very_high);
    CopyC::arm(DmaPriority::very_high);
    CopyD::arm(DmaPriority::very_high);
    leg.level = RingStream::priority();
    const uint32_t period = 11u * (SysClock::hz / baud);
    uint32_t next = CycleCounter::now() + period;
    for (uint32_t i = 0; i < arb_chars; ++i) {
        for (;;) {
            const int32_t d = static_cast<int32_t>(CycleCounter::now() - next);
            if (d >= 0) {
                if (static_cast<uint32_t>(d) > leg.late) {
                    leg.late = static_cast<uint32_t>(d);
                }
                break;
            }
            if (copies >= 1u) { leg.copies += keep_busy<CopyA>(0, bytes); }
            if (copies >= 2u) { leg.copies += keep_busy<CopyB>(1, bytes); }
            if (copies >= 3u) { leg.copies += keep_busy<CopyC>(2, bytes); }
            if (copies >= 4u) { leg.copies += keep_busy<CopyD>(3, bytes); }
        }
        Res::write_data(pattern(i));
        next = CycleCounter::now() + period;   // never two stores closer than a frame
    }
    while (CopyA::busy() || CopyB::busy() || CopyC::busy() || CopyD::busy()) {
    }
    wait_ms(1);
    leg.entries = usart_entries;
    wait_ms(1);
    leg.after = usart_entries - leg.entries;
    arb_judge<Port>(leg);
    leg.ore = Port::hw_overruns();
    leg.faults = Port::dma_faults();
    return leg;
}

void tg_arbitration() {
    for (uint32_t k = 0; k < 4u; ++k) {
        for (uint32_t i = 0; i < 1024u; ++i) {
            copy_src[k][i] = (i + k) * 2654435761u;
        }
    }
    bool levels = true;
    bool default_whole = true;
    bool two_whole = true;
    bool four_starve = true;
    bool quiet = true;
    static constexpr uint32_t rates[] = {5'625'000, 11'250'000};
    for (const uint32_t baud : rates) {
        for (uint8_t beat = 0; beat < 2u; ++beat) {
            for (uint8_t copies = 0; copies <= 4u; ++copies) {
                if (copies == 1u) {
                    continue;
                }
                const ArbLeg lo = arb_leg<RingLow>(6, baud, copies, beat == 1u);
                const ArbLeg hi = arb_leg<RingHigh>(5, baud, copies, beat == 1u);
                print(serial, "  ", baud, beat == 1u ? " byte" : " word", ", ", copies,
                      " copies | ring low: ", lo.lost, " of ", arb_chars, " lost in ", lo.gaps,
                      " gaps, ORE ", lo.ore, ", ", lo.entries, " vector entries, the thread ",
                      lo.late, " cycles late at worst | default: ", hi.lost, " lost, ORE ",
                      hi.ore, ", ", hi.entries, " entries, ", hi.late, " late", crlf);
                levels = levels && lo.level == DmaPriority::low &&
                         hi.level == DmaPriority::very_high;
                default_whole = default_whole && hi.got == arb_chars && hi.lost == 0u &&
                                hi.faults == 0u;
                if (copies <= 2u) {
                    two_whole = two_whole && lo.got == arb_chars && lo.lost == 0u;
                }
                if (copies == 4u) {
                    four_starve = four_starve && lo.lost > 0u;
                }
                quiet = quiet && lo.after == 0u && hi.after == 0u;
            }
        }
    }
    down();
    bench.verdict("rx_priority reaches the silicon: the ring's SxCR.PL read back 0 when the "
                  "options say low and 3 by default", levels);
    bench.verdict("THE DEFAULT RING LOSES NOTHING: at very_high, beside up to four copies at "
                  "very_high on its own controller, every byte in order at 5.625 and 11.25 "
                  "Mbaud, word and byte beats", default_whole);
    bench.verdict("two copies ranked above a ring armed low starve it of nothing: the arbiter "
                  "serves it between them", two_whole);
    bench.verdict("FOUR COPIES RANKED ABOVE A RING ARMED LOW DO STARVE IT - bytes lost in every "
                  "leg: the level decides correctness on this controller", four_starve);
    bench.verdict("THE OVERRUN WITH NOTHING TO TAKE IS CLEARED: after every starved leg the "
                  "vector is quiet - no entry in a millisecond of silence", quiet);
}

// ===========================================================================
// THE TWO-BOARD SET (key t): USART1 against the STM32G0B1RE's usart_peer
// ===========================================================================
//
// The desk's pairing: this board's USART1 crossed to the far board's -
// PA9 (TX, AF7) to its PA10, PA10 (RX, AF7) from its PA9 - the far board
// running usart_peer (stm32g0/src/apps/usart_peer.cpp), driven IN BAND
// over the link under test with usart_link.hpp's protocol: an ack before
// every action, a bounded action, the peer back in command mode by itself.
// The letters run under the second all-key alone: without the peer they
// FAIL, and the single-board letters above run with the far board dark.
//
// THE HARNESS. This end's link is the resource, polled: dut_link() turns
// a Cfg into the registers (the block reset first), the pads handed over
// only once the enable has sent its idle frame; command() retries three
// times, each after a quiet longer than the peer's partial-frame reset;
// peer_report() raises leg_refused on report_cfg_failed, and judge()
// prints SKIP for a refused leg and counts nothing. A leg this end's
// silicon cannot express is skipped the same way, by name.

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool leg_refused = false;
bool link_quiet = false;

using U1 = Usart<1>;
using U1Tx = Pin<'A', 9>;
using U1Rx = Pin<'A', 10>;
constexpr uint32_t u1_hz = SysClock::pclk2_hz;   // APB2, 90 MHz

/// The task over the same pads, for the receive legs: the interrupt
/// receiver, and the receive engine on DMA2 stream 2 channel 4 (RM0390
/// table 29, USART1_RX) - a stream no letter above uses.
constexpr UartPins link_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr UartOptions link_opts{.tx_speed = PinSpeed::very_high};
using LinkRxEngine = DmaRxEngine<2, 2, 4>;
using LinkIrq = Uart<1, link_pins, 2048, 64, NoDmaEngine, NoDmaEngine, link_opts>;
using LinkDma = Uart<1, link_pins, 2048, 64, NoDmaEngine, LinkRxEngine, link_opts>;
/// The same two above APB2 / 16 (5.625 Mbaud), where only OVER8 reaches.
constexpr UartOptions link_opts8{.over8 = true, .tx_speed = PinSpeed::very_high};
using LinkIrq8 = Uart<1, link_pins, 2048, 64, NoDmaEngine, NoDmaEngine, link_opts8>;
using LinkDma8 = Uart<1, link_pins, 2048, 64, NoDmaEngine, LinkRxEngine, link_opts8>;
/// Who owns USART1's vector: 0 nobody (the resource, polled), 1 LinkIrq,
/// 2 LinkDma, 3 LinkIrq8, 4 LinkDma8.
volatile uint8_t link_owner = 0;

void link_task_down() {
    if (link_owner == 1u) {
        LinkIrq::release();
    } else if (link_owner == 2u) {
        LinkDma::release();
    } else if (link_owner == 3u) {
        LinkIrq8::release();
    } else if (link_owner == 4u) {
        LinkDma8::release();
    }
    link_owner = 0;
}

void wait_us(uint32_t us) {
    const uint32_t c0 = CycleCounter::now();
    const uint32_t n = us * (SysClock::hz / 1'000'000u);
    while (CycleCounter::now() - c0 < n) {
    }
}

void put1(uint16_t b) {
    const uint32_t t0 = Ticker::millis();
    while (!U1::tx_empty() && Ticker::millis() - t0 < 50u) {
    }
    U1::write_word(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::millis();
    while (!U1::tx_complete() && Ticker::millis() - t0 < 50u) {
    }
}

void u1_flush() {
    for (uint8_t k = 0; k < 4u; ++k) {
        U1::clear_by_read();
    }
}

/// What this end can express of a Cfg: the frame and the divisor, or
/// nothing - M counts the parity bit (25.4.3), so 7+P, 8, 8+P and 9 and
/// the four stop codes; the oversampling closer to the rate.
struct DutSetup {
    UartFormat format{};
    uint16_t brr = 0;
    bool over8 = false;
    uint32_t actual = 0;   ///< the rate the divisor gives
};

std::optional<DutSetup> dut_setup(const link::Cfg& c) {
    DutSetup s{};
    const bool parity = c.parity != 0u;
    s.format.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    switch (c.bits) {
        case 7: if (!parity) return std::nullopt; s.format.bits = UartBits::seven; break;
        case 8: s.format.bits = UartBits::eight; break;
        case link::bits9_low:
        case link::bits9_high: if (parity) return std::nullopt; s.format.bits = UartBits::nine; break;
        default: return std::nullopt;
    }
    s.format.stop = c.stop == 2u ? UartStop::two
                  : c.stop == link::stop_half ? UartStop::half
                  : c.stop == link::stop_one_half ? UartStop::one_and_half : UartStop::one;
    const std::optional<uint16_t> b16 = usart_brr(u1_hz, c.rate);
    const std::optional<uint16_t> b8 = usart_brr_over8(u1_hz, c.rate);
    auto err = [&c](uint32_t a) { return a > c.rate ? a - c.rate : c.rate - a; };
    if (b16 && (!b8 || err(usart_actual_baud(u1_hz, *b16)) <= err(usart_actual_baud_over8(u1_hz, *b8)))) {
        s.brr = *b16;
        s.actual = usart_actual_baud(u1_hz, *b16);
    } else if (b8) {
        s.brr = *b8;
        s.over8 = true;
        s.actual = usart_actual_baud_over8(u1_hz, *b8);
    } else {
        return std::nullopt;
    }
    return s;
}

/// This end's link in a commanded configuration: the block reset and
/// programmed with the pads off it (PA9 an input on its pull-up, so the
/// line stays high through TE's idle frame), then the pads handed over.
bool dut_link(const link::Cfg& c, bool lin = false) {
    link_task_down();
    const std::optional<DutSetup> s = dut_setup(c);
    if (!s) {
        return false;
    }
    U1Tx::input(PinPull::up);
    U1::bus_clock(true);
    U1::enable(false);
    U1::reset();
    if (!U1::configure(s->format, s->brr) || (s->over8 && !U1::oversampling8(true))) {
        return false;
    }
    if (lin && !U1::lin(LinConfig{})) {
        return false;
    }
    U1::transmitter(true);
    U1::receiver(true);
    U1::enable(true);
    wait_us(2u * (1'000'000u / c.rate) + 20u);
    U1Tx::function(PinFunction::af7, {.speed = PinSpeed::very_high});
    U1Rx::function(PinFunction::af7, {.pull = PinPull::up});
    u1_flush();
    return true;
}

bool link_command_mode() {
    dec.reset();
    return dut_link(link::Cfg{});
}

/// The link's pads given back (analog) and the block off: what every
/// two-board letter leaves, so the single-board half finds them as reset
/// left them.
void link_release() {
    link_task_down();
    U1::enable(false);
    U1::reset();
    U1Tx::release();
    U1Rx::release();
}

bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    while (Ticker::millis() - t0 < ms) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::rxne) == 0u) {
            continue;
        }
        const uint8_t b = U1::read_data();
        if ((st & (UsartFlag::fe | UsartFlag::pe | UsartFlag::ne)) != 0u) {
            dec.reset();
            continue;
        }
        if (dec.feed(b) == link::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

void put_link(uint8_t b) { put1(b); }

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    u1_flush();
    link::write_frame(put_link, op, p, len);
    wait_tc();
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each after a quiet
/// interval longer than the peer's own partial-frame reset.
bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len, ms)) {
            return true;
        }
        (void)link_command_mode();
        wait_ms(70);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(link::byte_of(op)),
              ": no ack - is the far board running usart_peer?", crlf);
    }
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    return command(op, no_payload, 0, ms) && recv_frame(data, ms);
}

/// Find the peer: the crossed pair is the desk's only wiring, so this pings
/// until the peer's dark search hears a frame that checks out.
bool ensure_link() {
    link_quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = link_command_mode() && command(link::Op::ping);
    }
    link_quiet = false;
    if (!found) {
        print(serial, "  the peer did not answer: the STM32G0B1RE runs usart_peer on USART1?", crlf);
    }
    return found;
}

bool peer_ident(link::Ident& d) {
    link::Frame f;
    if (!query(link::Op::ident, f) || f.op != link::Op::ident_data || f.len != link::ident_size) {
        return false;
    }
    d = link::get_ident(f.data);
    return true;
}

/// The report of the last action, after its bound has had time to run. A
/// refusal raises leg_refused and says so once.
bool peer_report(link::Report& r, uint16_t after_ms) {
    link_task_down();
    (void)link_command_mode();
    if (after_ms != 0u) {
        wait_ms(after_ms);
    }
    for (uint8_t k = 0; k < 4u; ++k) {
        link::Frame f;
        const bool answered = query(link::Op::report, f) && f.op == link::Op::report_data;
        if (answered && f.len != link::report_size) {
            print(serial, "  the peer's REPORT is ", f.len, " bytes, this DUT reads ", link::report_size,
                  " - flash both boards from one tree", crlf);
            return false;
        }
        if (answered) {
            r = link::get_report(f.data);
            if ((r.flags & link::report_cfg_failed) != 0u && !leg_refused) {
                leg_refused = true;
                print(serial, "  the peer REFUSED op ", hex(r.op),
                      " (report_cfg_failed): a capability its silicon has not got", crlf);
            }
            return true;
        }
        wait_ms(40);
    }
    return false;
}

/// Command an action; the peer starts its own traffic link::settle_ms + 2
/// ms after its ack, so this end has settle_ms to take up its end.
bool peer_act(link::Op op, const link::Params& a, const uint8_t* extra = nullptr, uint8_t n = 0) {
    leg_refused = false;
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    for (uint8_t i = 0; i < n; ++i) {
        p[link::params_size + i] = extra[i];
    }
    if (!command(op, p, static_cast<uint8_t>(link::params_size + n))) {
        return false;
    }
    wait_us(static_cast<uint32_t>(link::settle_ms) * 1000u);
    return true;
}

/// A verdict, unless the leg was refused: then SKIP, counting nothing.
void judge(const char* name, bool ok) {
    if (leg_refused) {
        print(serial, "  SKIP  ", name, " (the peer refused the leg)", crlf);
        return;
    }
    bench.verdict(name, ok);
}
void judge(const char* name, const char* suffix, bool ok) {
    if (leg_refused) {
        print(serial, "  SKIP  ", name, suffix, " (the peer refused the leg)", crlf);
        return;
    }
    bench.verdict(name, suffix, ok);
}

/// Milliseconds `frames` frames need at `rate`, four times over plus a
/// floor, inside the protocol's 16-bit bound.
uint16_t bound_ms(uint32_t rate, uint32_t frames) {
    const uint32_t ms = (12u * frames * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
    return ms > 4000u ? 4000u : static_cast<uint16_t>(ms);
}

/// The peer opens its receive window settle_ms + 2 ms after its ack and
/// flushes what this end's reconfiguration left on the line.
void wait_for_peer_window() { wait_ms(6); }

/// `n` frames back to back: DR written on TXE, a frame ahead of the
/// shifter, so the line never idles.
void dut_blast(uint16_t n, uint16_t mask) {
    for (uint16_t i = 0; i < n; ++i) {
        while (!U1::tx_empty()) {
        }
        U1::write_word(static_cast<uint16_t>((0x55u + i) & mask));
    }
    wait_tc();
}

/// What this end's receiver made of a run, polled: frames, the errors, the
/// first and last data word, the first-to-last span on the DWT.
struct Drain {
    uint16_t frames = 0;
    uint8_t fe = 0, pe = 0, ne = 0, ore = 0;
    uint16_t first = 0, last = 0;
    uint32_t span = 0;   ///< DWT cycles, first frame to last
    bool lbd = false;
};

Drain collect(uint16_t ms, uint16_t want = 0xFFFFu) {
    Drain d{};
    uint32_t c_first = 0, c_last = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms && d.frames < want) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::lbd) != 0u) {
            d.lbd = true;
            U1::clear_flags(UsartFlag::lbd);
        }
        if ((st & UsartFlag::rxne) == 0u) {
            continue;
        }
        const uint32_t now = CycleCounter::now();
        const uint16_t w = U1::read_word();
        if (d.frames == 0u) {
            d.first = w;
            c_first = now;
        }
        d.last = w;
        c_last = now;
        ++d.frames;
        if ((st & UsartFlag::fe) != 0u && d.fe < 255u) ++d.fe;
        if ((st & UsartFlag::pe) != 0u && d.pe < 255u) ++d.pe;
        if ((st & UsartFlag::ne) != 0u && d.ne < 255u) ++d.ne;
        if ((st & UsartFlag::ore) != 0u && d.ore < 255u) ++d.ore;
    }
    d.span = c_last - c_first;
    return d;
}

/// One calibrate leg: the peer counts and times `n` gapless frames of this
/// end at `c`; the answer is this end's rate against the peer's time base.
struct Calib {
    bool ok = false;
    int16_t ppm = 0;
    uint32_t span = 0;
    uint16_t count = 0;
};

Calib calibrate(const link::Cfg& c, uint16_t n) {
    Calib k{};
    link::Params a{};
    a.cfg = c;
    a.count = n;
    a.ms = bound_ms(c.rate, n);
    if (!peer_act(link::Op::calibrate, a)) {
        return k;
    }
    if (!dut_link(c)) {
        return k;
    }
    wait_for_peer_window();
    dut_blast(n, c.bits >= link::bits9_low ? 0x1FFu : c.bits == 7u ? 0x7Fu : 0xFFu);
    link::Report r{};
    if (!peer_report(r, 5) || leg_refused) {
        return k;
    }
    k.ok = r.count == n && r.span != 0u && r.ferr == 0u && r.ovf == 0u;
    k.count = r.count;
    if (!k.ok) {
        print(serial, "    calibrate at ", c.rate, ": the peer counted ", r.count, " of ", n, ", ferr ", r.ferr,
              " ovf ", r.ovf, " span ", r.span, " flags ", hex(r.flags), crlf);
    }
    k.ppm = static_cast<int16_t>(r.aux16);
    k.span = r.span;
    return k;
}

void print_ppm(int32_t ppm) {
    const uint32_t m = static_cast<uint32_t>(ppm < 0 ? -ppm : ppm);
    print(serial, ppm < 0 ? "-" : "+", m, " ppm");
}

bool near(int32_t a, int32_t b, int32_t tol) { return a - b <= tol && b - a <= tol; }

/// Decimal for a verdict label.
char num_buf[12];
const char* num(uint32_t v) {
    uint8_t i = 11;
    num_buf[i] = 0;
    if (v == 0u) num_buf[--i] = '0';
    while (v != 0u && i != 0u) {
        num_buf[--i] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    return &num_buf[i];
}

/// The rate this end's divisor gives against the rate asked, in ppm.
int32_t divisor_ppm(uint32_t rate) {
    link::Cfg c{};
    c.rate = rate;
    const std::optional<DutSetup> s = dut_setup(c);
    if (!s) return 0;
    // Twice the kernel cycles a bit, exactly: the register read back the
    // way the generator divides (no integer rate rounded in between).
    const uint32_t twice = s->over8 ? (2u * ((static_cast<uint32_t>(s->brr >> 4) << 3) | (s->brr & 0x7u))) : 2u * s->brr;
    const int64_t num = static_cast<int64_t>(2ull * u1_hz) * 1'000'000;
    return static_cast<int32_t>(num / (static_cast<int64_t>(twice) * rate) - 1'000'000);
}

/// The ppm of a span against `frames - 1` frames of `half_bits` at `rate`,
/// measured on THIS end's DWT: positive when the far sender runs fast.
int32_t span_ppm(uint32_t span_cycles, uint32_t rate, uint16_t half_bits, uint16_t frames) {
    if (span_cycles == 0u || frames < 2u) return 0;
    const uint64_t nominal = (static_cast<uint64_t>(frames - 1u) * half_bits * SysClock::hz) / (2u * rate);
    return static_cast<int32_t>((static_cast<int64_t>(nominal) - span_cycles) * 1'000'000 /
                                static_cast<int64_t>(span_cycles));
}

constexpr uint32_t calib_rate = 1'000'000;   ///< exact on both ends' divisors here: the clocks alone
constexpr uint16_t calib_frames = 4000;      ///< 40 ms, a million ticks of span
int32_t base_ppm = 0;                        ///< letter 1's figure: this crystal against the peer's
bool base_known = false;

int32_t clock_baseline() {
    if (!base_known) {
        link::Cfg c{};
        c.rate = calib_rate;
        const Calib k = calibrate(c, calib_frames);
        if (k.ok) {
            base_ppm = k.ppm;
            base_known = true;
        }
    }
    return base_ppm;
}

// ---------------------------------------------------------------------------
// 1 - the rendezvous, and this crystal against the peer's LSE-measured core
// ---------------------------------------------------------------------------
void t1_link() {
    const bool found = ensure_link();
    bench.verdict("command mode on USART1 (8N1, 115200) and the peer answers a ping", found);
    if (!found) {
        link_release();
        return;
    }
    link::Ident d{};
    const bool got = peer_ident(d);
    bench.verdict("IDENT answered", got);
    char label[9] = {};
    if (got) {
        for (uint8_t i = 0; i < 8u; ++i) label[i] = d.label[i];
        print(serial, "  peer '", label, "' kind ", hex(link::peer_kind(d.version)), " fw ", hex(d.version),
              " xtal=", d.xtal, " sanity=", hex(d.sanity), crlf);
    }
    bench.verdict("the peer is the STM32G0's usart_peer", got && d.sanity == link::ident_sanity &&
                                                             link::peer_kind(d.version) == link::peer_stm32g0);
    // The label is the peer's UID word: not this die's own.
    const uint32_t w0 = DeviceUid::read().word[0];
    bool same = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        same = same && label[i] == static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }
    bench.verdict("and it is another die: its label is not this chip's unique ID", got && !same);
    bench.verdict("its time base is crystal-referenced (the core tracked against its LSE)", got && d.xtal == 1u);

    u1_flush();
    put1(link::magic);
    put1(link::byte_of(link::Op::ping));
    put1(0);
    put1(0x00);   // not the checksum
    wait_tc();
    link::Frame bad;
    bench.verdict("a bad checksum comes back as a NAK", recv_frame(bad, 120) && bad.op == link::Op::nak);
    for (uint8_t i = 0; i < 12u; ++i) put1(static_cast<uint8_t>(0x11u * i));
    wait_tc();
    wait_ms(120);
    (void)link_command_mode();
    bench.verdict("the link recovers after a desync", command(link::Op::ping));

    // THE REFUSAL PATH: a synchronous client (no CK wire on this desk) is
    // acked, reported refused, and the peer is back in command mode at once.
    {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.mode = link::Mode::sync_client;
        a.ms = 50;
        link::Report r{};
        const bool refused = peer_act(link::Op::set_link, a) && peer_report(r, 0) && leg_refused &&
                             (r.flags & link::report_ran) == 0u;
        leg_refused = false;
        bench.verdict("the peer refuses a synchronous client and answers at once after",
                      refused && command(link::Op::ping));
    }

    // THE CLOCKS. This end's 1 Mbaud is 90 MHz / 90, exact: what the peer
    // times is this crystal against its LSE-measured core.
    link::Cfg c{};
    c.rate = calib_rate;
    Calib k[3];
    bool all = true;
    for (Calib& x : k) {
        x = calibrate(c, calib_frames);
        all = all && x.ok;
    }
    print(serial, "  this crystal against the peer's LSE-measured core: ");
    print_ppm(k[0].ppm);
    print(serial, ", then ", k[1].ppm, " and ", k[2].ppm, " ppm; span ", k[0].span, " ticks for ",
          calib_frames - 1u, " frames", crlf);
    bench.verdict("the peer timed three gapless runs of 4000 frames at 1 Mbaud", all);
    bench.verdict("the three agree within 20 ppm: two crystals, no RC in the loop",
                  all && near(k[0].ppm, k[1].ppm, 20) && near(k[1].ppm, k[2].ppm, 20));
    // D4's check: the peer's calibration holds when two crystals meet
    // within their tolerances - the ST-LINK's 8 MHz and the LSE, each
    // tens of ppm - where the G0's uncorrected HSI would read ~0.2 %.
    bench.verdict("and within 150 ppm of each other: the peer's time base is its crystal's",
                  all && near(k[0].ppm, 0, 150));
    if (all) {
        base_ppm = k[0].ppm;
        base_known = true;
    }

    // The other direction, on this end's ruler: the peer's 1 Mbaud blast
    // timed on the DWT. The peer's divisor is its own (64 of its measured
    // 64.1 MHz is not exactly 1 Mbaud): a figure, judged only inside the
    // receivers' tolerance.
    {
        link::Params a{};
        a.cfg.rate = calib_rate;
        a.count = calib_frames;
        a.ms = bound_ms(calib_rate, calib_frames);
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(a.cfg);
        const Drain dr = up ? collect(a.ms, calib_frames) : Drain{};
        const int32_t ppm = span_ppm(dr.span, calib_rate, 20, dr.frames);
        print(serial, "  the peer's 1 Mbaud on this crystal: ", dr.frames, " frames, ");
        print_ppm(ppm);
        print(serial, " (its divisor on its measured core, 0 ppm being exact)", crlf);
        bench.verdict("the peer's blast arrives whole and inside a per cent of 1 Mbaud on this ruler",
                      up && dr.frames == calib_frames && dr.fe == 0u && near(ppm, 0, 10'000));
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 2 - the baud matrix across the wire, judged by a foreign receiver
// ---------------------------------------------------------------------------
uint16_t echo_mismatches(uint16_t n, uint16_t mask) {
    uint16_t bad = 0;
    wait_for_peer_window();
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>((0x5Au + i * 37u) & mask);
        while (!U1::tx_empty()) {
        }
        U1::write_word(v);
        bool ok = false;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 20u) {
            const uint32_t st = U1::status();
            if ((st & UsartFlag::rxne) != 0u) {
                ok = (U1::read_word() & mask) == v && (st & UsartFlag::receive_errors) == 0u;
                break;
            }
        }
        if (!ok) ++bad;
    }
    return bad;
}

/// THE LADDER BOTH DIVIDERS CAN MEET. This USART's BRR divides PCLK2 by
/// any whole k from 8 (OVER8's 3-bit fraction is eighths of 8 x USARTDIV,
/// 25.4.4), and the far STM32G0's divides its kernel by any whole n from 8
/// - its OVER8 adds no resolution, since BRR[2:0] = USARTDIV[3:0] >> 1
/// drops USARTDIV's last bit (RM0444 33.5.7). So the two lattices are
/// 90 MHz / k and 64 MHz / n, and they meet where k / n = 7 / 5: above
/// 2 Mbaud the rungs are 64 MHz / 5m - 2.56, 3.2, 4.27 and 6.4 Mbaud,
/// 0.45 % above the rate on this end, 0.15 % from the far end. A rate
/// between them is one or both ends a per cent or more off, 7.5 Mbaud
/// five per cent on the far end, past its receiver's tolerance.
constexpr uint32_t matrix_rates[] = {9600, 115'200, 921'600, 1'000'000, 1'500'000, 2'000'000,
                                     2'560'000, 3'200'000, 4'266'667, 6'400'000};

/// Frames enough for a span of some 25 ms at `rate`: the peer's stamps
/// jitter by its loop, a microsecond, so the reading resolves 40 ppm.
uint16_t span_frames(uint32_t rate) {
    const uint32_t n = rate / 400u;
    return static_cast<uint16_t>(n < 64u ? 64u : (n > 20'000u ? 20'000u : n));
}

/// `n` bytes of the peer's counting blast through a Uart task, read as
/// runs: how many arrived, in order.
template <typename Port>
uint32_t task_receive(uint32_t n, uint16_t ms, uint8_t& ore) {
    uint32_t got = 0;
    bool broken = false;
    const uint32_t t0 = Ticker::millis();
    while (got < n && !broken && Ticker::millis() - t0 < ms) {
        const std::span<const uint8_t> run = Port::read_span();
        uint32_t k = 0;
        for (const uint8_t b : run) {
            if (b != static_cast<uint8_t>(got)) {
                broken = true;
                break;
            }
            ++got;
            ++k;
        }
        (void)Port::consume(k);
    }
    ore = Port::hw_overruns();
    return got;
}

void t2_rates() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    const int32_t base = clock_baseline();
    print(serial, "  this crystal against the peer's: ");
    print_ppm(base);
    print(serial, crlf);
    for (const uint32_t rate : matrix_rates) {
        link::Cfg c{};
        c.rate = rate;
        // a. An echo of 16 frames.
        {
            link::Params a{};
            a.cfg = c;
            a.count = 16;
            a.ms = bound_ms(rate, 32);
            const bool acted = peer_act(link::Op::echo, a);
            const bool up = acted && dut_link(c);
            const uint16_t bad = up ? echo_mismatches(16, 0xFFu) : 16u;
            link::Report r{};
            const bool have = peer_report(r, 5);
            judge(num(rate), " baud: 16 frames echoed by the far chip, every one equal", up && bad == 0u);
            judge(num(rate), " baud: the far chip counted 16 clean frames",
                  have && r.count == 16u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u);
        }
        // b. This end's transmitter timed by the FOREIGN receiver.
        const uint16_t n = span_frames(rate);
        const Calib k = calibrate(c, n);
        const int32_t expect = base + divisor_ppm(rate);
        print(serial, "  ", rate, ": ", n, " frames, the far receiver times ");
        print_ppm(k.ppm);
        print(serial, " against this divisor's ", divisor_ppm(rate), " and the clocks' ", base, crlf);
        judge(num(rate), " baud: this transmitter's rate on the wire, timed by the far chip, within 200 ppm "
                         "of the divisor's",
              k.ok && near(k.ppm, expect, 200));
        // c. The peer's blast into this end: the polled receiver timing it,
        //    then the interrupt receiver and the receive engine.
        {
            link::Params a{};
            a.cfg = c;
            a.count = n;
            a.ms = bound_ms(rate, n);
            const bool acted = peer_act(link::Op::blast, a);
            const bool up = acted && dut_link(c);
            const Drain dr = up ? collect(a.ms, n) : Drain{};
            link::Report r{};
            (void)peer_report(r, 0);
            const int32_t ppm = span_ppm(dr.span, rate, 20, dr.frames);
            print(serial, "    the far blast on this ruler: ", dr.frames, " of ", n, ", ORE ", dr.ore, ", ");
            print_ppm(ppm);
            print(serial, crlf);
            judge(num(rate), " baud: the far blast arrives whole on the polled receiver, inside a per cent of "
                             "its rate",
                  up && dr.frames == n && dr.fe == 0u && dr.ore == 0u && near(ppm, 0, 10'000));
        }
        for (uint8_t which = 1; which <= 2u; ++which) {
            link::Params a{};
            a.cfg = c;
            a.count = 1024;
            a.ms = bound_ms(rate, 1024);
            a.value = 0;
            a.pattern = link::pattern_counting;
            const bool acted = peer_act(link::Op::blast, a);
            link_task_down();
            const bool fast = !LinkIrq::can_baud(u1_hz, rate);
            link_owner = static_cast<uint8_t>(which + (fast ? 2u : 0u));
            bool up = acted;
            if (up) {
                up = fast ? (which == 1u ? LinkIrq8::init(clock, rate) : LinkDma8::init(clock, rate))
                          : (which == 1u ? LinkIrq::init(clock, rate) : LinkDma::init(clock, rate));
            }
            uint8_t ore = 0;
            uint32_t got = 0;
            if (up) {
                if (fast) {
                    got = which == 1u ? task_receive<LinkIrq8>(1024, a.ms, ore)
                                      : task_receive<LinkDma8>(1024, a.ms, ore);
                } else {
                    got = which == 1u ? task_receive<LinkIrq>(1024, a.ms, ore)
                                      : task_receive<LinkDma>(1024, a.ms, ore);
                }
            }
            link_task_down();
            link::Report r{};
            (void)peer_report(r, 0);
            if (got != 1024u) {
                print(serial, "    ", which == 1u ? "the interrupt receiver" : "the receive engine", ": ", got,
                      " of 1024 in order, ORE ", ore, crlf);
            }
            judge(num(rate), which == 1u ? " baud: 1024 frames whole through the interrupt receiver"
                                         : " baud: 1024 frames whole through the receive engine",
                  up && got == 1024u);
        }
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 3 - the frame matrix across the wire
// ---------------------------------------------------------------------------
struct FmtCase {
    uint8_t bits;
    uint8_t parity;
    uint8_t stop;
    const char* name;
};

void t3_frames() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    static constexpr FmtCase cases[] = {
        {8, 0, 1, "8N1"}, {8, 1, 1, "8E1"}, {8, 2, 1, "8O1"}, {7, 1, 1, "7E1"}, {7, 2, 1, "7O1"},
        {link::bits9_low, 0, 1, "9N1"}, {8, 0, 2, "8N2"}, {8, 0, link::stop_half, "8N0.5"},
        {8, 0, link::stop_one_half, "8N1.5"}, {7, 0, 1, "7N1"}, {6, 1, 1, "6E1"},
        {link::bits9_low, 1, 1, "9E1"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
        if (!dut_setup(c)) {
            print(serial, "  SKIP  ", fc.name, " (this USART has no word length for it, RM0390 25.4.3)", crlf);
            continue;
        }
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(115'200, 24);
        const uint16_t mask = fc.bits >= link::bits9_low ? 0x1FFu : fc.bits == 7u ? 0x7Fu : 0xFFu;
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(8, mask) : 8u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        if (bad != 0u && !leg_refused) {
            print(serial, "    ", fc.name, ": wrong=", bad, " peer count=", r.count, " ferr=", r.ferr,
                  " perr=", r.perr, crlf);
        }
        judge(fc.name, " across the wire, echoed by the far chip",
              up && bad == 0u && have && r.count == 8u && r.ferr == 0u && r.perr == 0u);
    }
    // The two odd stops as LENGTHS: a gapless run timed by the peer is a
    // frame of 19 and of 21 half bits, the clocks' offset and nothing else.
    for (const uint8_t stop : {link::stop_half, link::stop_one_half}) {
        link::Cfg ref{};
        ref.rate = calib_rate;
        const Calib r0 = calibrate(ref, 2000);
        link::Cfg c{};
        c.rate = calib_rate;
        c.stop = stop;
        const Calib k = calibrate(c, 2000);
        print(serial, "  ", stop == link::stop_half ? "8N0.5" : "8N1.5", " at 1 Mbaud: ");
        print_ppm(k.ppm);
        print(serial, " against frames of ", link::frame_half_bits(8, 0, stop), " half bits (8N1: ", r0.ppm,
              ")", crlf);
        judge(stop == link::stop_half ? "half a stop bit" : "one and a half stop bits",
              ": the frame on the wire is that long, within 200 ppm of an 8N1 run's",
              r0.ok && k.ok && near(k.ppm, r0.ppm, 200));
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 4 - the receiver's errors, from a foreign sender
// ---------------------------------------------------------------------------
void t4_errors() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    // 1. Odd parity into an even receiver: PE on each, the data readable.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;
        link::Cfg dc = pc;
        dc.parity = 1;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        a.pattern = link::pattern_fixed;
        const bool acted = peer_act(link::Op::send, a);
        const bool up = acted && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 12), 6);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  odd into even: frames=", d.frames, " pe=", d.pe, " fe=", d.fe, " data ", hex(d.last),
              crlf);
        judge("every frame of the wrong parity raises PE, and its data is still read",
              up && d.frames == 6u && d.pe == 6u && d.fe == 0u && (d.last & 0xFFu) == 0x5Au);
    }
    // 2. The FE onset: 0x00 frames from the peer's divisor a few per cent
    //    off 9600, into a receiver at 9600 (25.4.5's tolerance, 3.75 % at
    //    OVER16 with no fraction: 9375 here). BACK TO BACK: a fast sender's
    //    frames with idle between them never fail the stop bit - its sample
    //    lands in the idle - so the blast's gapless run is what shows both
    //    edges of the tolerance.
    {
        static constexpr int8_t errs[] = {0, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 8, -8};
        int8_t onset_pos = 0, onset_neg = 0;
        bool inside_clean = true, outside_raised = true;
        link::Cfg dc{};
        dc.rate = 9600;
        for (const int8_t e : errs) {
            link::Params a{};
            a.cfg.rate = static_cast<uint32_t>(9600 + (9600 * e) / 100);
            a.count = 6;
            a.ms = 400;
            a.value = 0x00;
            a.pattern = link::pattern_fixed;
            const bool acted = peer_act(link::Op::blast, a);
            const bool up = acted && dut_link(dc);
            const Drain d = collect(40, 6);
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  ", e, " %: frames=", d.frames, " fe=", d.fe, " ne=", d.ne, crlf);
            const bool clean = up && d.frames == 6u && d.fe == 0u;
            if (up && !clean) {
                if (e > 0 && onset_pos == 0) onset_pos = e;
                if (e < 0 && onset_neg == 0) onset_neg = e;
            }
            if (e >= -3 && e <= 3) inside_clean = inside_clean && clean;
            if (e == 8 || e == -8) outside_raised = outside_raised && up && !clean;
        }
        print(serial, "  FE onset: +", onset_pos, " %, ", onset_neg, " %", crlf);
        judge("every rate within 3 % of the receiver's is clean (25.4.5's 3.75 %)", inside_clean);
        judge("and 8 % off either way raises FE", outside_raised);
    }
    // 3. A run into a receiver that reads late: ORE, DR keeps the first.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(115'200, 16);
        a.value = 0xB0;
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(pc);
        wait_ms(6 + 2);
        const uint32_t st = U1::status();
        const uint16_t kept = U1::read_word();
        const Drain d = collect(5);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  a run of 8 read late: SR ", hex(st), ", DR ", hex(kept), ", then ", d.frames, " more",
              crlf);
        judge("a run read late raises ORE", up && (st & UsartFlag::ore) != 0u && (st & UsartFlag::rxne) != 0u);
        judge("and DR keeps the first frame of the run, the rest lost (one data register, 25.6.1)",
              up && kept == 0xB0u && d.frames == 0u);
    }
    // 4. NE from a glitch: one low pulse of w ticks on the idle line at
    //    9600 (a bit is 2500 ticks, a sixteenth 156), with ONEBIT clear -
    //    the start bit's 3rd, 5th and 7th samples voted, NF on a split vote
    //    (25.4.5) - and set.
    {
        static constexpr uint16_t widths[] = {120, 600, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1700, 2500};
        uint16_t first_taken[2] = {0, 0};
        bool tiny_ignored = true, whole_taken = true, ne_seen = false, ne_onebit = false;
        for (uint8_t onebit = 0; onebit < 2u; ++onebit) {
            print(serial, "  ONEBIT ", onebit, ":");
            for (const uint16_t w : widths) {
                link::Params a{};
                a.cfg.rate = 9600;
                a.count = 1;
                a.ms = 100;
                a.aux16 = w;
                a.aux8 = 10;
                link::Cfg dc{};
                dc.rate = 9600;
                const bool acted = peer_act(link::Op::glitch, a);
                bool up = acted && dut_link(dc);
                if (up && onebit != 0u) {
                    U1::enable(false);
                    U1::one_bit_sampling(true);
                    U1::enable(true);
                    u1_flush();
                }
                const Drain d = collect(15);
                link::Report r{};
                (void)peer_report(r, 0);
                print(serial, " ", w, ":", d.frames, "/", d.ne, "/", d.fe);
                if (up && d.frames != 0u && first_taken[onebit] == 0u) first_taken[onebit] = w;
                if (w == 120u) tiny_ignored = tiny_ignored && up && d.frames == 0u;
                if (w == 2500u) whole_taken = whole_taken && up && d.frames == 1u;
                if (onebit == 0u && d.ne != 0u) ne_seen = true;
                if (onebit != 0u && d.ne != 0u) ne_onebit = true;
            }
            print(serial, "   (ticks:frames/NE/FE)", crlf);
        }
        print(serial, "  the narrowest pulse taken as a start bit: ", first_taken[0], " ticks, ", first_taken[1],
              " with ONEBIT", crlf);
        judge("a pulse under a sample (120 ticks) is never a start bit", tiny_ignored);
        judge("a whole bit low is one, either way", whole_taken);
        judge("and with ONEBIT clear a pulse that splits the start bit's vote raises NE", ne_seen);
        judge("with ONEBIT set no width raises NE (25.4.5: the one-sample method has no vote)", !ne_onebit);
    }
    // 5. A break of 20 bit times: one frame, FE, data 0x00 - and in LIN
    //    mode LBD beside it (25.4.7's break detection).
    for (const bool lin : {false, true}) {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 20;
        a.ms = 200;
        const bool acted = peer_act(link::Op::brk, a);
        const bool up = acted && dut_link(pc, lin);
        const Drain d = collect(30);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  a break of 20 bit times", lin ? " (LIN mode)" : "", ": frames=", d.frames, " fe=", d.fe,
              " data ", hex(d.first), " lbd=", d.lbd ? 1 : 0, "; the engine ",
              (r.flags & link::report_mismatch) != 0u ? "BROKEN" : "whole", crlf);
        if (!lin) {
            judge("a break arrives as one frame with FE and data 0x00",
                  up && d.frames == 1u && d.fe == 1u && d.first == 0u);
        } else {
            judge("and in LIN mode it raises LBD as well", up && d.lbd);
        }
    }
    link_release();
}

void banner() {
    print(serial, crlf, "test_stm32f4_serial - the transport on USART6's single-wire loop", crlf);
    bench.menu();
}

}   // namespace

extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
extern "C" void USART1_IRQHandler() {
    const uint8_t o = link_owner;
    if (o == 1u) {
        (void)LinkIrq::isr();
    } else if (o == 2u) {
        (void)LinkDma::isr();
    } else if (o == 3u) {
        (void)LinkIrq8::isr();
    } else if (o == 4u) {
        (void)LinkDma8::isr();
    }
}
extern "C" void DMA2_Stream2_IRQHandler() {
    if (link_owner == 2u) {
        (void)LinkDma::dma_isr();
    } else if (link_owner == 4u) {
        (void)LinkDma8::dma_isr();
    }
}
extern "C" void USART6_IRQHandler() {
    usart_entries = usart_entries + 1u;
    bool edge = false;
    if (owner == 1u) {
        edge = LoopIrq::isr();
    } else if (owner == 2u) {
        edge = LoopDma::isr();
    } else if (owner == 3u) {
        edge = LoopOd::isr();
    } else if (owner == 4u) {
        edge = LoopFast::isr();
    } else if (owner == 5u) {
        edge = RingHigh::isr();
    } else if (owner == 6u) {
        edge = RingLow::isr();
    }
    if (edge) {
        edge_at = CycleCounter::now();
        edges = edges + 1u;
    }
}
[[gnu::always_inline]] inline void loop_streams() {
    dma_entries = dma_entries + 1u;
    bool edge = false;
    if (owner == 2u) {
        edge = LoopDma::dma_isr();
    } else if (owner == 4u) {
        edge = LoopFast::dma_isr();
    } else if (owner == 5u) {
        edge = RingHigh::dma_isr();
    } else if (owner == 6u) {
        edge = RingLow::dma_isr();
    }
    if (edge) {
        edge_at = CycleCounter::now();
        edges = edges + 1u;
    }
}
extern "C" void DMA2_Stream1_IRQHandler() { loop_streams(); }
extern "C" void DMA2_Stream6_IRQHandler() { loop_streams(); }
extern "C" void EXTI9_5_IRQHandler() {
    pad_edge_at = CycleCounter::now();
    (void)Exti::clear(6);
}
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)CycleCounter::init();
    brio::enable_interrupts();

    bench.letter('a', "the loop proven: both receivers, every rate, the open drain's ceiling", ta_loop);
    bench.letter('b', "the frame formats on the loop", tb_formats);
    bench.letter('c', "ONEBIT and OVER8", tc_sampling);
    bench.letter('d', "errors under the engine: breaks in a stream, nothing stolen", td_errors);
    bench.letter('e', "tx_idle() against the last stop bit on the pad", te_tx_idle);
    bench.letter('f', "the burst edge from the vector, nothing polled", tf_edge);
    bench.letter('g', "the receive ring against copies on its own controller", tg_arbitration);
    bench.second_letter('1', "the rendezvous, and this crystal against the far chip's", t1_link);
    bench.second_letter('2', "the baud matrix across the wire, timed by the far chip", t2_rates);
    bench.second_letter('3', "the frame matrix across the wire", t3_frames);
    bench.second_letter('4', "the receiver's errors from a foreign sender", t4_errors);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
              tick_ok ? "SysTick" : "FAILED", crlf);
        banner();
        bench.prompt();
    }
    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c) || c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
