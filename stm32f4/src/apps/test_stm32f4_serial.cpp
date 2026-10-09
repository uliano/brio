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
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;
TestBench<Serial> bench;

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

void banner() {
    print(serial, crlf, "test_stm32f4_serial - the transport on USART6's single-wire loop", crlf);
    bench.menu();
}

}   // namespace

extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
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
