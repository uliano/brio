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
//   h  THE TRANSMIT ROOM AND EDGE, AND in_handler(): tx_room() the ring's
//      capacity on an idle port, less a run written with interrupts
//      masked, the capacity again after the wire time; tx_drained() false
//      before a write, true once after it has drained, false again - for
//      the interrupt transmitter and the transmit engine, and read by the
//      vector's glue, one edge a write; the platform's in_handler() true
//      in the USART6 vector, false in the letter's body and under a mask
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
//   5  ES0298 2.13.2 staged: odd parity in while this end transmits, in
//      the erratum's order, the workaround's and through the task
//   6  the far engine's shapes: where a start is believed, the operational
//      range of a gapped frame, one stretched cell
//   7  ES0298 2.13.1 staged: IDLE and the receive engine's edge from a
//      sender up to 3 % faster, at gaps of one frame and more; and the
//      erratum's own case drawn by the far engine (the stage op) - an
//      idle of 9.5 to 11 of the SENDER's bits, the sender 0 to 3 % fast
//   8  letter g's ring against copies, the far chip as the sender at the
//      ladder's top rungs
//   9  half duplex against a second board: drive_rxd, one open-drain wire
//      between the chips, a collision
//   0  LIN both ways: the far engine's breaks detected at LBDL's two
//      lengths, the same breaks with LIN mode OFF (LBD still rises -
//      letter 4's finding, staged), this end's break timed by the far
//      receiver with LIN and without
//   r  IrDA SIR both ways against the far ST ENDEC (its TXINV and RXINV
//      the one inversion each way needs), the 3/16 and the low-power
//      pulse on the pad, normal-mode PSC = 1 refused otherwise, the far
//      chip's 115200 ceiling; RS-485 refused (this USART has no DE)
//   s  mute mode from a foreign sender: the idle-line wake (RWU after a
//      burst's first frame, and before any frame), the address-mark wake
//      on the far send_mpcm with this USART's 4-bit address on 8- and
//      9-bit words, and ES0298 2.13.3 staged with its controls - after
//      a software mute and after the hardware's own
//   u  set_baud() at run time: a task brought up at 115200 and moved to
//      9600, 921600 and 2 Mbaud by set_baud() alone, the far send received
//      whole and its own frames timed by the far chip; TCIE's vector
//      between one and two frames after the last write; LBDIE's vector on
//      a far break in LIN mode
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
#include "../../../avrxt/src/apps/usart_link.hpp"

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
/// 5 RingHigh, 6 RingLow, 7 LoopIrq with letter h's glue (the transmit
/// edge read in the vector, in_handler() asked there).
volatile uint8_t owner = 0;
volatile uint32_t tx_edges = 0;      ///< the transmit edges owner 7's glue read
volatile uint8_t handler_said = 0;   ///< in_handler() in owner 7's vector: 0 unasked, 1 false, 2 true
volatile uint32_t usart_entries = 0;
volatile uint32_t dma_entries = 0;
volatile uint32_t edges = 0;
volatile uint32_t edge_at = 0;   ///< the DWT at the last edge
volatile uint32_t pad_edge_at = 0;   ///< the DWT at PC6's last rising edge

void down() {
    const uint8_t was = owner;
    if (was == 1u || was == 7u) {
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
uint32_t last_latency = 0, last_latency_lo = 0, last_latency_hi = 0;   ///< the stamp's correction, cycles

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
    // trigger of the same line to the stamp, taken off - the MEDIAN of
    // seven triggers, since one trigger can meet the SysTick's entry or a
    // flash line's fill and read tens of cycles long (a single sample read
    // the transmitter's figure bimodal run to run).
    uint32_t lat[7];
    for (uint32_t& l : lat) {
        const uint32_t before = pad_edge_at;
        const uint32_t t_sw = CycleCounter::now();
        (void)PadLine::trigger();
        const uint32_t t_wait = CycleCounter::now();
        while (pad_edge_at == before && CycleCounter::now() - t_wait < SysClock::hz / 1000u) {
        }
        l = pad_edge_at - t_sw;
    }
    for (uint8_t i = 1; i < 7u; ++i) {   // insertion sort of seven
        const uint32_t v = lat[i];
        uint8_t j = i;
        while (j > 0u && lat[j - 1u] > v) {
            lat[j] = lat[j - 1u];
            --j;
        }
        lat[j] = v;
    }
    const uint32_t latency = lat[3];
    last_latency_lo = lat[0];
    last_latency_hi = lat[6];
    last_latency = latency;
    (void)PadLine::arm(false);
    Nvic::disable(PadLine::irq());
    return static_cast<int32_t>(t_idle - edge + latency);
}

void te_tx_idle() {
    static constexpr uint32_t rates[] = {115200, 1'000'000};
    for (const uint32_t baud : rates) {
        uint32_t bit = 0;
        const int32_t d_irq = idle_after_stop<LoopIrq>(1, baud, bit);
        const uint32_t l_irq = last_latency, l_irq_lo = last_latency_lo, l_irq_hi = last_latency_hi;
        const int32_t d_dma = idle_after_stop<LoopDma>(2, baud, bit);
        print(serial, "  ", baud, " baud, a bit ", bit, " cycles: tx_idle() true ", d_irq,
              " cycles after the stop bit's rising edge (interrupt transmitter), ", d_dma,
              " (transmit engine); the stamp's latency a median of 7: ", l_irq, " (", l_irq_lo, "..",
              l_irq_hi, ") and ", last_latency, " (", last_latency_lo, "..", last_latency_hi, ")", crlf);
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

// ---- h  the transmit room and edge, and in_handler() -----------------------------------
//
// A run of 48 frames is written with interrupts MASKED, so no consumer
// of the transmit ring can run between the write and the look that
// follows it: the TXE handler is held off, and the transmit engine,
// whose stream does run, releases its block's bytes only in its
// completion handler, held off too. So the room read under the mask is
// exact - capacity less 48, no bound to derive - and so is the edge
// (none while masked). Then the wire runs: 48 frames of ten bits at
// 1 Mbaud are 480 us, waited for on tx_idle() with a bound of 10 ms
// (twenty times the wire time: a miss is a stuck transmitter, never a
// slow one). The loop hears what it sends, so the 48 frames are read
// back after a millisecond more (the last one's stop bit) and checked
// against the pattern: the run went out whole, not just out of the
// ring.

constexpr uint32_t room_run = 48;
constexpr uint32_t room_capacity = 511;   ///< LoopIrq's and LoopDma's 512-slot transmit ring

/// Wait, bounded, for the port's wire to fall idle; false when the bound
/// ran out. The bound is the caller's, derived from its run.
template <typename Port>
bool idle_within(uint32_t cycles) {
    const uint32_t t0 = CycleCounter::now();
    while (!Port::tx_idle()) {
        if (CycleCounter::now() - t0 > cycles) {
            return false;
        }
    }
    return true;
}

struct RoomLeg {
    bool up = false;
    uint32_t idle_room = 0;   ///< tx_room() on the idle port
    bool edge_before = true;  ///< tx_drained() before the write
    uint32_t taken = 0;       ///< what write_bulk() took, masked
    uint32_t held_room = 0;   ///< tx_room() right after it, masked
    bool edge_held = true;    ///< tx_drained() then, masked
    bool idle = false;        ///< the wire idle within the bound
    uint32_t after_room = 0;  ///< tx_room() after the wire time
    bool edge_first = false;  ///< tx_drained() then
    bool edge_second = true;  ///< and read again
    uint32_t back = 0;        ///< frames the loop heard back
    bool in_order = false;
};

template <typename Port>
RoomLeg room_leg(uint8_t who) {
    static uint8_t run[room_run];
    for (uint32_t i = 0; i < room_run; ++i) {
        run[i] = pattern(i);
    }
    RoomLeg l;
    l.up = up<Port>(who, 1'000'000);
    l.idle_room = Port::tx_room();
    l.edge_before = Port::tx_drained();
    {
        Stm32f4Platform<>::CriticalSection cs;
        l.taken = Port::write_bulk(std::span<const uint8_t>(run, room_run));
        l.held_room = Port::tx_room();
        l.edge_held = Port::tx_drained();
    }
    l.idle = idle_within<Port>(SysClock::hz / 100u);
    l.after_room = Port::tx_room();
    l.edge_first = Port::tx_drained();
    l.edge_second = Port::tx_drained();
    wait_ms(1);
    l.in_order = true;
    l.back = drain<Port>(0, l.in_order);
    return l;
}

bool room_leg_ok(const RoomLeg& l) {
    return l.up && l.idle_room == room_capacity && !l.edge_before && l.taken == room_run &&
           l.held_room == room_capacity - room_run && !l.edge_held && l.idle &&
           l.after_room == room_capacity && l.edge_first && !l.edge_second &&
           l.back == room_run && l.in_order;
}

void print_room_leg(const char* name, const RoomLeg& l) {
    print(serial, "  ", name, ": room ", l.idle_room, " idle, ", l.held_room, " after ", l.taken,
          " written masked, ", l.after_room, " after the wire; edge ", l.edge_before ? "true" : "false",
          " before, ", l.edge_held ? "true" : "false", " masked, ", l.edge_first ? "true" : "false",
          " then ", l.edge_second ? "true" : "false", " after; ", l.back, " of ", room_run,
          " back", l.in_order ? " in order" : " OUT OF ORDER", crlf);
}

void th_room_edge() {
    const RoomLeg irq = room_leg<LoopIrq>(1);
    print_room_leg("interrupt transmitter", irq);
    bench.verdict("the interrupt transmitter: tx_room() the capacity idle, less the run masked, the "
                  "capacity after the wire; tx_drained() false, false, then true once",
                  room_leg_ok(irq));
    const RoomLeg dma = room_leg<LoopDma>(2);
    print_room_leg("transmit engine      ", dma);
    bench.verdict("the transmit engine: the block holds its run in the ring until its completion, "
                  "the completion that empties the ring is the edge, told once",
                  room_leg_ok(dma));

    // The glue: the vector reads the edge after isr(), one edge a write.
    static uint8_t run[room_run];
    for (uint32_t i = 0; i < room_run; ++i) {
        run[i] = pattern(i);
    }
    (void)up<LoopIrq>(7, 1'000'000);
    tx_edges = 0;
    handler_said = 0;
    constexpr uint32_t writes = 3;
    bool all_idle = true;
    for (uint32_t k = 0; k < writes; ++k) {
        {
            Stm32f4Platform<>::CriticalSection cs;
            (void)LoopIrq::write_bulk(std::span<const uint8_t>(run, room_run));
        }
        all_idle = idle_within<LoopIrq>(SysClock::hz / 100u) && all_idle;
        wait_ms(1);
        bool ok = true;
        (void)drain<LoopIrq>(0, ok);
    }
    const uint32_t glue_edges = tx_edges;
    const bool body_edge = LoopIrq::tx_drained();   // the glue read every one
    const uint8_t said = handler_said;
    const bool body_says = Stm32f4Platform<>::in_handler();
    bool masked_says = true;
    {
        Stm32f4Platform<>::CriticalSection cs;
        masked_says = Stm32f4Platform<>::in_handler();
    }
    down();
    print(serial, "  the vector's glue: ", glue_edges, " edge(s) for ", writes, " writes, ",
          body_edge ? "one" : "none", " left for the body; in_handler() ",
          said == 2u ? "true" : said == 1u ? "false" : "unasked", " in the vector, ",
          body_says ? "true" : "false", " in the body, ", masked_says ? "true" : "false",
          " under a mask", crlf);
    bench.verdict("the edge read in the vector's glue: one a write, none left behind",
                  all_idle && glue_edges == writes && !body_edge);
    bench.verdict("in_handler() is IPSR's: true in the USART6 vector, false in the letter's body "
                  "and under a critical section", said == 2u && !body_says && !masked_says);
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
/// 2 LinkDma, 3 LinkIrq8, 4 LinkDma8; letter 8's rings 5 LinkRing, 6
/// LinkRing8, 7 LinkRingLow, 8 LinkRingLow8; letter u's probe 9.
volatile uint8_t link_owner = 0;

void link_rings_down(uint8_t o);

void link_task_down() {
    if (link_owner == 1u) {
        LinkIrq::release();
    } else if (link_owner == 2u) {
        LinkDma::release();
    } else if (link_owner == 3u) {
        LinkIrq8::release();
    } else if (link_owner == 4u) {
        LinkDma8::release();
    } else if (link_owner >= 5u) {
        link_rings_down(link_owner);
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
    // OVER8 alone under the protocol's clk2x flag, the closer of the two
    // otherwise.
    const bool force8 = (c.flags & link::flag_clk2x) != 0u;
    const std::optional<uint16_t> b16 = force8 ? std::nullopt : usart_brr(u1_hz, c.rate);
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
/// `lin_eleven` is LBDL, in LIN mode.
bool dut_link(const link::Cfg& c, bool lin = false, bool lin_eleven = false) {
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
    if (lin && !U1::lin(LinConfig{.break_11bit = lin_eleven})) {
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
constexpr uint16_t agree_frames = 40'000;    ///< letter 1's three runs: 0.4 s each

/// THE BOUND LETTER 1'S THREE RUNS AGREE WITHIN, DERIVED (docs/stm32f4/usart.md,
/// "The rendezvous's bound"). This crystal and the peer's LSE move apart by
/// well under a ppm over the second the runs take, so two figures differ by
/// the peer's measurement error alone - a TIME at each end of its span,
/// whatever the span's length, in nanoseconds:
///  - its two stamps: each frame seen at the next turn of a 20-cycle poll,
///    behind its receiver's 1/16-bit start sampling (4 cycles at 1 Mbaud)
///    and up to 5 cycles of DMA on the bus - 29 cycles of its 64.2 MHz;
///  - the tracker's conversion (stm32g0/lse_tracker.hpp): exact for its
///    window's mean, which starts 0.5 to 0.8 ms ahead of the span and stops
///    up to an interval short of it, and the HSI16 wanders hundreds of ppm
///    over such slivers; with the captures' 8-cycle grain, within 0.47 us
///    measured over 90 windows;
///  - one SysTick handler on a stamp: some 55 cycles of the M0+'s entry,
///    the twelve instructions and the return.
/// Either figure is off by at most their sum; two figures by twice that,
/// and the report's whole ppm adds one.
constexpr uint32_t agree_stamp_ns = 450, agree_tracker_ns = 470, agree_isr_ns = 860;
constexpr uint32_t agree_span_us = (agree_frames - 1u) * 10u;   // 8N1 at 1 Mbaud
constexpr int32_t agree_ppm = static_cast<int32_t>(
    (2u * (agree_stamp_ns + agree_tracker_ns + agree_isr_ns) * 1000u + agree_span_us - 1u) / agree_span_us + 1u);
static_assert(agree_ppm == 10, "letter 1's verdict states the derived bound: move its text with it");
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
        x = calibrate(c, agree_frames);
        all = all && x.ok;
    }
    print(serial, "  this crystal against the peer's LSE-measured core: ");
    print_ppm(k[0].ppm);
    print(serial, ", then ", k[1].ppm, " and ", k[2].ppm, " ppm; span ", k[0].span, " ticks for ",
          agree_frames - 1u, " frames", crlf);
    bench.verdict("the peer timed three gapless runs of 40000 frames at 1 Mbaud", all);
    // 3.56 us over 0.4 s is 9 ppm, and the whole ppm: 10. The same 3.56 us
    // over the 40 ms of a 4000-frame run would be 89 ppm, more than the
    // peer's uncorrected HSI16 moves between two runs (median 74 ppm,
    // measured) - hence 40000 frames, 1.2 s for the three.
    bench.verdict("the three agree within 10 ppm (the bound derived above): two crystals, no RC in the loop",
                  all && near(k[0].ppm, k[1].ppm, agree_ppm) && near(k[1].ppm, k[2].ppm, agree_ppm) &&
                      near(k[0].ppm, k[2].ppm, agree_ppm));
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
        link::Report r{};
        const bool have = peer_report(r, 0);
        const int32_t ppm = span_ppm(dr.span, calib_rate, 20, dr.frames);
        // The peer states its divisor's rate on its tracked core (the
        // blast's aux16): this ruler reads that less this crystal's offset
        // against the peer's (the three runs above).
        const int32_t far = static_cast<int16_t>(r.aux16);
        print(serial, "  the peer's 1 Mbaud on this crystal: ", dr.frames, " frames, ");
        print_ppm(ppm);
        print(serial, "; its divisor's own ", far, " ppm, this crystal against its ", base_ppm, crlf);
        bench.verdict("the peer's blast arrives whole, its rate on this ruler within 200 ppm of its divisor's "
                      "less the clocks' offset",
                      up && have && dr.frames == calib_frames && dr.fe == 0u && all &&
                          (r.flags & link::report_rate_stated) != 0u && near(ppm, far - base_ppm, 200));
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

/// Frames enough for a span of some 25 ms at `rate`, and a thousand at
/// least: a far receiver's first or last stamp moves by some 0.4 bit from
/// run to run (measured on both peers: +600 ppm on 64 frames at 9600,
/// +140 on 288 at 115200, a third of the runs), so a span of N frames
/// resolves 0.04 / N - under 40 ppm from a thousand. The window stays
/// inside nine tenths of the trackers' rings (this end's and the far G0
/// peer's, a second each), so the rate it is judged against is the
/// window's own.
uint16_t span_frames(uint32_t rate) {
    uint32_t n = rate / 400u;
    if (n < 1000u) n = 1000u;
    const uint32_t cap = static_cast<uint32_t>((uint64_t{rate} * (1000u * 9u / 10u)) / 10'000u);
    if (n > cap) n = cap;
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
            const bool have = peer_report(r, 0);
            const int32_t ppm = span_ppm(dr.span, rate, 20, dr.frames);
            // The far divisor's rate on its tracked core (the blast's
            // aux16 with report_rate_stated - 0 then exact; the flag clear
            // from a peer that cannot tell) less the clocks' offset: the
            // far transmitter judged as tightly as this one.
            const int32_t far = static_cast<int16_t>(r.aux16);
            const bool known = have && (r.flags & link::report_rate_stated) != 0u;
            print(serial, "    the far blast on this ruler: ", dr.frames, " of ", n, ", ORE ", dr.ore, ", ");
            print_ppm(ppm);
            print(serial, " against the far divisor's ", far, " less the clocks' ", base, crlf);
            judge(num(rate), known ? " baud: the far blast arrives whole on the polled receiver, its rate within "
                                     "200 ppm of the far divisor's less the clocks' offset"
                                   : " baud: the far blast arrives whole on the polled receiver, inside a per "
                                     "cent of its rate (the far end does not state its own)",
                  up && dr.frames == n && dr.fe == 0u && dr.ore == 0u &&
                      (known ? near(ppm, far - base, 200) : near(ppm, 0, 10'000)));
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

// ---------------------------------------------------------------------------
// The core letters past the first four: what they share
// ---------------------------------------------------------------------------

/// A counting stream judged with gaps allowed: the peer's counting pattern
/// runs on across a blast's repeats, so a byte out of sequence is either a
/// corrupt byte (the one after it is back in sequence) or the stream k
/// places further on - a gap of k frames, modulo the pattern's 256.
struct CountCheck {
    uint8_t expect = 0;
    uint8_t held = 0;
    bool holding = false;
    uint32_t got = 0, gaps = 0, lost = 0, bad = 0;

    void take(uint8_t b) {
        ++got;
        if (holding) {
            holding = false;
            if (b == static_cast<uint8_t>(expect + 1u)) {
                ++bad;   // the held byte was corrupt, the stream in step behind it
                expect = static_cast<uint8_t>(expect + 2u);
                return;
            }
            const uint8_t k = static_cast<uint8_t>(held - expect);
            ++gaps;
            lost += k;
            expect = static_cast<uint8_t>(held + 1u);
        }
        if (b == expect) {
            expect = static_cast<uint8_t>(expect + 1u);
            return;
        }
        held = b;
        holding = true;
    }
    void close() {
        if (holding) {
            holding = false;
            ++gaps;
            lost += static_cast<uint8_t>(held - expect);
        }
    }
};

template <typename Port>
uint32_t ring_drain(CountCheck& c) {
    uint32_t n = 0;
    for (;;) {
        const std::span<const uint8_t> run = Port::read_span();
        if (run.empty()) {
            return n;
        }
        for (const uint8_t b : run) {
            c.take(b);
        }
        (void)Port::consume(static_cast<uint32_t>(run.size()));
        n += static_cast<uint32_t>(run.size());
    }
}

/// USART1 as a SINGLE WIRE on PA9 (HDSEL, 25.4.10): the pad that meets the
/// far board's PA10, where the far G0 swaps its own single wire - open
/// drain on its pull-up, PA10 an input with no pull. The resource, polled.
bool wire_up(uint32_t baud) {
    link_task_down();
    link::Cfg c{};
    c.rate = baud;
    const std::optional<DutSetup> s = dut_setup(c);
    if (!s) {
        return false;
    }
    U1Tx::input(PinPull::up);
    U1::enable(false);
    U1::reset();
    if (!U1::configure(s->format, s->brr) || (s->over8 && !U1::oversampling8(true)) || !U1::half_duplex(true)) {
        return false;
    }
    U1::transmitter(true);
    U1::receiver(true);
    U1::enable(true);
    wait_us(2u * (1'000'000u / baud) + 20u);
    U1Tx::function(PinFunction::af7, {.pull = PinPull::up, .open_drain = true, .speed = PinSpeed::high});
    U1Rx::input(PinPull::none);
    u1_flush();
    return true;
}

/// One received word within `us`, polled; its SR errors in `err`.
std::optional<uint16_t> wire_word(uint32_t us, uint32_t& err) {
    const uint32_t c0 = CycleCounter::now();
    const uint32_t n = us * (SysClock::hz / 1'000'000u);
    while (CycleCounter::now() - c0 < n) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::rxne) != 0u) {
            err = st & UsartFlag::receive_errors;
            return U1::read_word();
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// 5 - ES0298 2.13.2 staged: PE in full duplex while this end transmits
// ---------------------------------------------------------------------------
//
// "In full-duplex mode, when the Parity Error flag is set by the receiver
// at the end of a reception, it may be cleared while transmitting by
// reading the USART_SR register to check the TXE or TC flags and writing
// data to the data register" - with no workaround but the order. The far
// chip sends 64 odd-parity frames into this end at 8E1 while it transmits
// back to back, three ways: the polled loop in the ERRATUM'S ORDER (an SR
// read and a DR write every turn, then the SR read for the receive); the
// polled loop in the WORKAROUND'S (one SR read, DR read, then the write);
// and the Uart task (one SR read in its vector decides both). A half
// duplex loop never transmits while it receives, which is why the parity
// legs of the loop letters could not reach it.

struct PeLeg {
    bool up = false;
    uint16_t frames = 0, pe = 0;
};

PeLeg pe_polled(bool erratum_order) {
    PeLeg l{};
    link::Params a{};
    a.cfg.rate = 115'200;
    a.cfg.parity = 2;
    a.count = 64;
    a.ms = bound_ms(115'200, 128);
    a.value = 0x5A;
    a.pattern = link::pattern_fixed;
    link::Cfg dc = a.cfg;
    dc.parity = 1;
    const bool acted = peer_act(link::Op::send, a);
    l.up = acted && dut_link(dc);
    USART_TypeDef& u = U1::regs();
    const uint32_t t0 = Ticker::millis();
    uint32_t last = t0;
    while (l.up && Ticker::millis() - t0 < 60u) {
        if (erratum_order) {
            // The transmit path's SR read and DR write land between the
            // error and the receive path's SR read EVERY turn: written
            // unconditionally (a write over a full DR replaces the byte
            // queued, nothing else), since gating it on TXE would meet the
            // window once in hundreds of frames.
            (void)u.SR;
            u.DR = 0x33u;
            const uint32_t st2 = u.SR;
            if ((st2 & UsartFlag::rxne) != 0u) {
                if ((st2 & UsartFlag::pe) != 0u) ++l.pe;
                (void)u.DR;
                ++l.frames;
                last = Ticker::millis();
            }
        } else {
            const uint32_t st = u.SR;
            if ((st & UsartFlag::rxne) != 0u) {
                if ((st & UsartFlag::pe) != 0u) ++l.pe;
                (void)u.DR;
                ++l.frames;
                last = Ticker::millis();
            }
            if ((st & UsartFlag::txe) != 0u) {
                u.DR = 0x33u;
            }
        }
        if (l.frames != 0u && Ticker::millis() - last > 5u) {
            break;
        }
    }
    wait_tc();
    link::Report r{};
    (void)peer_report(r, 0);
    return l;
}

void t5_parity_full_duplex() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    const PeLeg wrong = pe_polled(true);
    const PeLeg right = pe_polled(false);
    print(serial, "  64 odd-parity frames into 8E1 while transmitting: the erratum's order counted ", wrong.pe,
          " PE in ", wrong.frames, " frames, the workaround's ", right.pe, " in ", right.frames, crlf);
    judge("the workaround's order (SR once, DR read, then the write) counts every parity error",
          right.up && right.frames == 64u && right.pe == 64u);
    judge("ES0298 2.13.2 STAGED: in the erratum's order (an SR read and a DR write between the error and the "
          "receive's SR read) PE is lost on frames - REPRODUCED", wrong.up && wrong.frames == 64u && wrong.pe < 64u);

    // The task: the far end's 64 frames while the task's own transmitter
    // runs back to back (the ring kept full from the thread).
    {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.parity = 2;
        a.count = 64;
        a.ms = bound_ms(115'200, 128);
        a.value = 0x5A;
        a.pattern = link::pattern_fixed;
        const bool acted = peer_act(link::Op::send, a);
        link_task_down();
        link_owner = 1;
        const bool up = acted && LinkIrq::init(clock, 115'200, {.bits = UartBits::eight, .parity = UartParity::even});
        static const uint8_t fill[64] = {};
        const uint32_t t0 = Ticker::millis();
        while (up && Ticker::millis() - t0 < 30u) {
            (void)LinkIrq::write_bulk(std::span<const uint8_t>(fill, sizeof fill));
        }
        const uint32_t pending = LinkIrq::rx_pending();
        const uint8_t pe = LinkIrq::parity_errors();
        link_task_down();
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  through the task, transmitting the whole time: ", pe, " parity errors counted of 64, ",
              pending, " bytes delivered", crlf);
        judge("the task's one SR read a vector decides receive and transmit with: every parity error counted "
              "while it transmits, none delivered", up && pe == 64u && pending == 0u);
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 6 - the engine's shapes: where a start is believed, the operational
//     range, one stretched cell
// ---------------------------------------------------------------------------
//
// The far engine (TIM1 on the STM32G0B1RE's PA9, its core tracked against
// its LSE) draws pulses of any width and frames at any rate with no
// divisor's rounding. This receiver is on the crystal; its own divisor's
// error is taken out of every rate below.

struct BangLeg {
    bool up = false;
    Drain d{};
    bool clean = false;     ///< every frame the value, no error flag at all
    bool correct = false;   ///< every frame the value, no FE or PE (a noise flag allowed)
};

BangLeg bang_leg(uint32_t rate, uint32_t dut_rate, uint8_t value, uint16_t count, int8_t cell, int8_t pct) {
    BangLeg l{};
    link::Params a{};
    a.cfg.rate = rate;
    a.count = count;
    a.value = value;
    a.cell = cell;
    a.pct = pct;
    a.ms = 400;
    link::Cfg dc{};
    dc.rate = dut_rate;
    const bool acted = peer_act(link::Op::bitbang, a);
    l.up = acted && dut_link(dc);
    l.d = l.up ? collect(static_cast<uint16_t>(20u + (16u * count * 1000u) / rate), count) : Drain{};
    link::Report r{};
    (void)peer_report(r, 0);
    l.correct = l.up && l.d.frames == count && l.d.fe == 0u && l.d.pe == 0u && (l.d.first & 0xFFu) == value &&
                (l.d.last & 0xFFu) == value && (r.flags & link::report_mismatch) == 0u;
    l.clean = l.correct && l.d.ne == 0u;
    return l;
}

void t6_shapes() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    // a. WHERE A START IS BELIEVED at 115200 (a bit is 208 ticks): eight
    //    low pulses of w ticks, 255 us apart, OVER16 and OVER8.
    static constexpr uint16_t widths[] = {6, 13, 26, 52, 78, 91, 104, 110, 117, 123, 130,
                                          136, 143, 149, 156, 169, 182, 208};
    uint16_t first_taken[2] = {0, 0}, first_clean[2] = {0, 0};
    bool under_half = true, whole_clean[2] = {true, true};
    for (uint8_t o8 = 0; o8 < 2u; ++o8) {
        print(serial, "  ", o8 != 0u ? "OVER8 " : "OVER16", " at 115200, 8 pulses a width (ticks:frames/NE/FE):");
        for (const uint16_t w : widths) {
            link::Params a{};
            a.cfg.rate = 115'200;
            a.count = 8;
            a.ms = 100;
            a.aux16 = w;
            a.aux8 = 255;
            link::Cfg dc{};
            dc.rate = 115'200;
            if (o8 != 0u) {
                dc.flags = link::flag_clk2x;
            }
            const bool acted = peer_act(link::Op::glitch, a);
            bool up = acted && dut_link(dc);
            if (up && o8 != 0u && !U1::oversampling8()) {
                up = false;
            }
            const Drain d = up ? collect(20) : Drain{};
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, " ", w, ":", d.frames, "/", d.ne, "/", d.fe);
            if (up && d.frames != 0u && first_taken[o8] == 0u) first_taken[o8] = w;
            if (up && d.frames == 8u && d.ne == 0u && d.fe == 0u && first_clean[o8] == 0u) first_clean[o8] = w;
            if (o8 == 0u && w < 104u) under_half = under_half && up && d.frames == 0u;
            if (w == 208u) whole_clean[o8] = whole_clean[o8] && up && d.frames == 8u && d.ne == 0u &&
                                             d.fe == 0u && (d.last & 0xFFu) == 0xFFu;
        }
        print(serial, crlf);
    }
    print(serial, "  the narrowest pulse taken: ", first_taken[0], " ticks at OVER16 (",
          (first_taken[0] * 16u + 104u) / 208u, " sixteenths), ", first_taken[1], " at OVER8; taken clean eight "
          "of eight from ", first_clean[0], " and ", first_clean[1], crlf);
    judge("a pulse under half a bit is never a start at OVER16 (25.4.5: the 3rd, 5th and 7th samples and the "
          "8th, 9th and 10th vote)", under_half);
    judge("a whole bit low is a clean 0xFF frame, eight of eight, in either oversampling",
          whole_clean[0] && whole_clean[1]);
    judge("and the narrowest start sits between 8 and 11 sixteenths of a bit at OVER16",
          first_taken[0] >= 104u && first_taken[0] <= 143u);

    // b. THE OPERATIONAL RANGE of a gapped 0x00 frame at 9600 and 115200,
    //    against this receiver's real rate (its divisor's error out).
    for (const uint32_t base : {9600u, 115'200u}) {
        const int32_t real_ppm = divisor_ppm(base);
        // Each side walked in steps of 5 per mille until a frame is wrong
        // or flagged FE/PE; the first leg with a noise flag on the way is
        // the vote's onset, printed beside the range.
        int16_t lo = 0, hi = 0, ne_lo = 0, ne_hi = 0;
        for (const int16_t sgn : {static_cast<int16_t>(1), static_cast<int16_t>(-1)}) {
            for (int16_t p = 5; p <= 120; p += 5) {
                const int32_t off = real_ppm + sgn * p * 1000;
                const uint32_t rate =
                    static_cast<uint32_t>((static_cast<int64_t>(base) * (1'000'000 + off)) / 1'000'000);
                const BangLeg l = bang_leg(rate, base, 0x00, 4, -1, 0);
                int16_t& ne_at = sgn > 0 ? ne_hi : ne_lo;
                if (!l.clean && ne_at == 0) {
                    ne_at = static_cast<int16_t>(sgn * p);
                }
                if (!l.correct) {
                    print(serial, "    broken at ", sgn * p, " per mille: ", l.d.frames, " frames, FE ", l.d.fe, " NE ",
                          l.d.ne, " PE ", l.d.pe, ", data ", hex(l.d.first), "..", hex(l.d.last), crlf);
                    (sgn > 0 ? hi : lo) = static_cast<int16_t>(sgn * p);
                    break;
                }
            }
        }
        const bool lo_found = lo != 0, hi_found = hi != 0;
        print(serial, "  ", base, " baud (the divisor ", real_ppm, " ppm off): four 0x00 frames decoded from ",
              lo + 5, " to +", hi - 5, " per mille of its rate, broken at ", lo, " and +", hi,
              "; the noise flag from ", ne_lo, " and +", ne_hi, crlf);
        // 25.4.5's table (OVER16, fraction != 0): 3.75 % at DIV_Fraction 0,
        // 3.41 % otherwise - whatever the sender.
        judge(base == 9600u ? "9600" : "115200",
              " baud: the gapped frame decodes across 25.4.5's 3.41 % both ways, measured against this "
              "receiver's real rate",
              lo_found && hi_found && lo <= -35 && hi >= 35);
    }

    // c. ONE STRETCHED CELL at 115200: four 0x55 frames, the start cell,
    //    data bit 3 or the stop cell longer or shorter by pct.
    static constexpr int8_t pcts[] = {-60, -45, -30, -15, 15, 30, 45, 60};
    bool thirty_ok = true, sixty_data_broken = true, stop_free = true;
    const uint32_t rate = static_cast<uint32_t>((static_cast<int64_t>(115'200) * (1'000'000 + divisor_ppm(115'200))) /
                                                1'000'000);
    for (const int8_t cell : {static_cast<int8_t>(0), static_cast<int8_t>(4), static_cast<int8_t>(9)}) {
        print(serial, "  cell ", cell, cell == 0 ? " (start)" : cell == 9 ? " (stop)" : " (data bit 3)", ":");
        for (const int8_t pct : pcts) {
            const BangLeg l = bang_leg(rate, 115'200, 0x55, 4, cell, pct);
            print(serial, " ", pct, "%:", l.clean ? "ok" : "BAD");
            if (cell != 9 && (pct == 30 || pct == -30 || pct == 15 || pct == -15)) thirty_ok = thirty_ok && l.clean;
            if (cell == 4 && (pct == 60 || pct == -60)) sixty_data_broken = sixty_data_broken && l.up && !l.clean;
            if (cell == 9) stop_free = stop_free && l.clean;
        }
        print(serial, crlf);
    }
    judge("a start or data cell 30 % longer or shorter is decoded: every later sample stays inside its cell",
          thirty_ok);
    judge("a data cell 60 % longer or shorter is not: the next sample lands in the neighbouring cell",
          sixty_data_broken);
    judge("and the stop cell's length is free when an idle follows it", stop_free);
    link_release();
}

// ---------------------------------------------------------------------------
// 7 - ES0298 2.13.1 staged: the line's IDLE, and the receive engine's burst
//     edge, from a FASTER sender
// ---------------------------------------------------------------------------
//
// "If the USART receives an idle frame followed by a character, and the
// clock of the transmitter device is faster than the USART receiver clock,
// the USART receive signal falls too early when receiving the character
// start bit, with the result that the idle frame is not detected" - no
// workaround. And THE RECEIVE ENGINE'S BURST EDGE IS IDLE (usart.md): its
// contract is a burst told within two frames of its last stop bit. Twenty
// bursts of sixteen frames at 50000 baud (a frame is 200 us), gaps of one
// frame, a frame and a half and five, the far chip 0 to 3 % faster: the
// polled resource counts IDLE itself, the receive engine its edges.

struct IdleLeg {
    bool up = false;
    uint32_t frames = 0, idles = 0, edges = 0;
    CountCheck c{};
};

volatile uint32_t link_edges = 0;   ///< the link task's vectors' true answers

IdleLeg idle_leg(uint32_t peer_rate, uint8_t gap_units, bool engine) {
    IdleLeg l{};
    link::Params a{};
    a.cfg.rate = peer_rate;
    a.count = 16;
    a.aux16 = 20;
    a.aux8 = gap_units;
    a.ms = 1000;
    a.pattern = link::pattern_counting;
    const bool acted = peer_act(link::Op::blast, a);
    if (!engine) {
        link::Cfg dc{};
        dc.rate = 50'000;
        l.up = acted && dut_link(dc);
        USART_TypeDef& u = U1::regs();
        const uint32_t t0 = Ticker::millis();
        uint32_t last = t0;
        while (l.up && Ticker::millis() - t0 < 600u) {
            const uint32_t st = u.SR;
            if ((st & UsartFlag::rxne) != 0u) {
                (void)u.DR;
                ++l.frames;
                last = Ticker::millis();
                continue;
            }
            if ((st & UsartFlag::idle) != 0u) {
                (void)u.DR;   // SR then DR: IDLE's clear (25.6.1)
                ++l.idles;
            }
            if (l.frames != 0u && Ticker::millis() - last > 50u) {
                break;
            }
        }
    } else {
        link_task_down();
        link_owner = 2;
        l.up = acted && LinkDma::init(clock, 50'000);
        link_edges = 0;
        const uint32_t t0 = Ticker::millis();
        uint32_t last = t0;
        while (l.up && Ticker::millis() - t0 < 600u) {
            if (ring_drain<LinkDma>(l.c) != 0u) {
                last = Ticker::millis();
            }
            if (l.c.got != 0u && Ticker::millis() - last > 50u) {
                break;
            }
        }
        (void)ring_drain<LinkDma>(l.c);
        l.c.close();
        l.frames = l.c.got;
        l.edges = link_edges;
        link_task_down();
    }
    link::Report r{};
    (void)peer_report(r, 0);
    return l;
}

/// One train the far engine DRAWS (usart_link.hpp's stage): frames of one
/// fixed value, gaps in sixteenths of the SENDER's bit - read polled, the
/// IDLE flags counted from the first frame on (SR then DR clears both).
struct StageLeg {
    bool up = false;
    bool drawn = false;   ///< the far engine played the train whole
    uint16_t frames = 0, wrong = 0, idles = 0;
};

StageLeg stage_leg(const link::Params& a, const link::Cfg& dc) {
    StageLeg l{};
    const bool acted = peer_act(link::Op::stage, a);
    l.up = acted && dut_link(dc);
    USART_TypeDef& u = U1::regs();
    const uint32_t t0 = Ticker::millis();
    uint32_t last = t0;
    while (l.up && Ticker::millis() - t0 < 400u) {
        const uint32_t st = u.SR;
        if ((st & UsartFlag::rxne) != 0u) {
            if ((static_cast<uint16_t>(u.DR) & 0xFFu) != a.value) ++l.wrong;
            ++l.frames;
            last = Ticker::millis();
            continue;
        }
        if ((st & UsartFlag::idle) != 0u) {
            (void)u.DR;   // SR then DR: IDLE's clear (25.6.1)
            if (l.frames != 0u) ++l.idles;
        }
        if (l.frames != 0u && Ticker::millis() - last > 30u) {
            break;
        }
    }
    link::Report r{};
    l.drawn = peer_report(r, 0) && (r.flags & link::report_ran) != 0u && (r.flags & link::report_mismatch) == 0u &&
              r.count == a.count;
    return l;
}

void t7_idle_faster() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    static constexpr int16_t offsets[] = {0, 5, 10, 20, 30};   // per mille, the far end faster
    static constexpr uint8_t gaps[] = {2, 3, 10};               // 100 us units: 1, 1.5 and 5 frames
    bool whole = true, idle_long = true, edge_long = true, idle_one = true;
    // The engine's edges a run: the first frame's turn and the line's IDLE
    // for every burst (letter f's two a burst), the very first frame's turn
    // merged with the receiver's start - so the reference is the count with
    // the far end exact and the longest gap, and a missed IDLE is a leg
    // short of it.
    uint32_t edge_ref = 0;
    for (uint8_t engine = 0; engine < 2u; ++engine) {
        if (engine != 0u) {
            edge_ref = idle_leg(50'000, 10, true).edges;
        }
        for (const uint8_t g : gaps) {
            print(serial, "  ", engine != 0u ? "receive engine, edges" : "polled, IDLE flags", " per 20 bursts, gap ",
                  g * 100u, " us, the far end +0, +5, +10, +20, +30 per mille:");
            for (const int16_t e : offsets) {
                const uint32_t rate =
                    static_cast<uint32_t>((static_cast<int64_t>(50'000) * (1'000'000 + e * 1000)) / 1'000'000);
                const IdleLeg l = idle_leg(rate, g, engine != 0u);
                print(serial, " ", engine != 0u ? l.edges : l.idles);
                whole = whole && l.up && l.frames == 320u && (engine == 0u || (l.c.gaps == 0u && l.c.bad == 0u));
                if (engine != 0u) {
                    edge_long = edge_long && edge_ref >= 20u && l.edges == edge_ref;
                } else if (g != 2u) {
                    idle_long = idle_long && l.idles == 20u;
                } else {
                    idle_one = idle_one && l.idles == 20u;
                }
            }
            print(serial, crlf);
        }
    }
    judge("every burst arrives whole at every offset and every gap, through both receivers", whole);
    judge("IDLE is told once a burst after a gap of a frame and a half or more, the far end up to 3 % faster",
          idle_long);
    print(serial, "  the engine's reference: ", edge_ref, " edges for 20 bursts (each burst's first frame and its "
          "IDLE)", crlf);
    judge("THE ENGINE'S CONTRACT HOLDS: no burst's IDLE edge missed at any gap or offset, the far end up to 3 % "
          "faster", edge_long);
    judge("ES0298 2.13.1 staged at a gap of one frame (200 us at 50000 baud) from a sender up to 3 % fast: every "
          "IDLE told - the erratum NOT reproduced there", idle_one);

    // THE ERRATUM'S OWN CASE, gaps stated in the SENDER's bit times (the
    // stage op): sixteen 0xF0 frames at 50000 baud from a far end 0, 1 and
    // 3 % faster, an idle of 9.5 to 11 of ITS bits after each - "an idle
    // frame followed by a character" is ten. Fifteen gaps a train and its
    // end: sixteen IDLE where every gap is told, printed with the gap in
    // THIS receiver's sixteenths (this crystal's divisor is exact here).
    static constexpr uint16_t idle16[] = {152, 156, 160, 164, 168, 176};
    static constexpr int16_t fast[] = {0, 10, 30};
    bool trains_whole = true, eleven_told = true;
    uint16_t own_case = 0xFFFFu, past_case = 0xFFFFu, exact_long = 0, exact_short = 0xFFFFu;
    for (const int16_t e : fast) {
        print(serial, "  the far end +", e, " per mille, gap (sender sixteenths = this receiver's): IDLE of 16:");
        for (const uint16_t g : idle16) {
            link::Params a{};
            a.cfg.rate = static_cast<uint32_t>((static_cast<int64_t>(50'000) * (1'000 + e)) / 1'000);
            a.count = 16;
            a.value = 0xF0;
            a.pattern = link::pattern_fixed;
            a.aux16 = g;
            link::Cfg dc{};
            dc.rate = 50'000;
            const StageLeg l = stage_leg(a, dc);
            const uint32_t here = (static_cast<uint32_t>(g) * 1000u + (1000u + static_cast<uint32_t>(e)) / 2u) /
                                  (1000u + static_cast<uint32_t>(e));
            print(serial, " ", g, "=", here, ":", l.idles);
            trains_whole = trains_whole && l.up && l.drawn && l.frames == 16u && l.wrong == 0u;
            if (g == 176u) eleven_told = eleven_told && l.idles == 16u;
            if (g == 160u && e == 30) own_case = l.idles;
            if (g == 168u && e == 30) past_case = l.idles;
            if (g == 164u && e == 0) exact_long = l.idles;
            if (g == 156u && e == 0) exact_short = l.idles;
        }
        print(serial, crlf);
    }
    judge("every drawn train arrives whole, sixteen frames of sixteen", trains_whole);
    judge("an idle of eleven sender bits is told by IDLE after every frame, the far end up to 3 % faster",
          eleven_told);
    judge("from an exact sender the idle frame is ten bits: told after every frame at 10.25, after none at "
          "9.75", exact_long == 16u && exact_short == 1u);
    // The receiver times its idle frame from the stop bit AS IT SAMPLED
    // it, which a faster sender's frame ends ahead of by the sender's
    // error over the frame - the far G0's letter 9 reads the same table.
    judge("ES0298 2.13.1 REPRODUCES in its own case: one idle frame and then a character from a sender 3 % "
          "fast, and the idle frame is not told - nor at 10.5 sender bits, 10.2 of this receiver's, past the "
          "exact sender's threshold", own_case == 1u && past_case == 1u);
    link_release();
}

// ---------------------------------------------------------------------------
// 8 - the ring under copies, with a real sender
// ---------------------------------------------------------------------------
//
// Letter g's arrangement on USART1's receive engine (DMA2 stream 2): zero,
// two, three or four copies at very_high on DMA2's streams 0, 4, 5 and 7,
// word or byte beats, beside the ring at its default level and ranked low
// - the sender now the far chip, 4096 frames back to back at 4.27 and 6.4
// Mbaud (the common ladder's top rungs), the ring holding the whole leg
// and read at its end.

constexpr uint32_t f8_frames = 4096;
constexpr UartOptions ring_low{.tx_speed = PinSpeed::very_high, .rx_priority = DmaPriority::low};
constexpr UartOptions ring_low8{.over8 = true, .tx_speed = PinSpeed::very_high, .rx_priority = DmaPriority::low};
using LinkRing = Uart<1, link_pins, arb_ring, 64, NoDmaEngine, LinkRxEngine, link_opts>;
using LinkRing8 = Uart<1, link_pins, arb_ring, 64, NoDmaEngine, LinkRxEngine, link_opts8>;
using LinkRingLow = Uart<1, link_pins, arb_ring, 64, NoDmaEngine, LinkRxEngine, ring_low>;
using LinkRingLow8 = Uart<1, link_pins, arb_ring, 64, NoDmaEngine, LinkRxEngine, ring_low8>;
using LinkStream = DmaStream<2, 2>;

void link_rings_down(uint8_t o) {
    if (o == 5u) {
        LinkRing::release();
    } else if (o == 6u) {
        LinkRing8::release();
    } else if (o == 7u) {
        LinkRingLow::release();
    } else if (o == 8u) {
        LinkRingLow8::release();
    }
}

struct RingLeg {
    bool up = false;
    CountCheck c{};
    uint32_t copies = 0;
    uint8_t ore = 0, faults = 0;
    DmaPriority level = DmaPriority::low;
};

template <typename Port>
RingLeg ring_leg_on(uint8_t who, uint32_t rate, uint8_t copies, bool bytes) {
    RingLeg l{};
    link::Params a{};
    a.cfg.rate = rate;
    a.count = f8_frames;
    a.ms = bound_ms(rate, f8_frames);
    a.pattern = link::pattern_counting;
    const bool acted = peer_act(link::Op::blast, a);
    link_task_down();
    link_owner = who;
    l.up = acted && Port::init(clock, rate);
    CopyA::arm(DmaPriority::very_high);
    CopyB::arm(DmaPriority::very_high);
    CopyC::arm(DmaPriority::very_high);
    CopyD::arm(DmaPriority::very_high);
    l.level = LinkStream::priority();
    // The blast starts some 5 ms after the ack and lasts 4096 frame times.
    const uint32_t c0 = CycleCounter::now();
    const uint32_t span = SysClock::hz / 1000u * 8u + static_cast<uint32_t>((10ull * f8_frames * SysClock::hz) / rate);
    while (l.up && CycleCounter::now() - c0 < span) {
        if (copies >= 1u) { l.copies += keep_busy<CopyA>(0, bytes); }
        if (copies >= 2u) { l.copies += keep_busy<CopyB>(1, bytes); }
        if (copies >= 3u) { l.copies += keep_busy<CopyC>(2, bytes); }
        if (copies >= 4u) { l.copies += keep_busy<CopyD>(3, bytes); }
    }
    while (CopyA::busy() || CopyB::busy() || CopyC::busy() || CopyD::busy()) {
    }
    wait_ms(2);
    (void)ring_drain<Port>(l.c);
    l.c.close();
    l.ore = Port::hw_overruns();
    l.faults = Port::dma_faults();
    link_task_down();
    link::Report r{};
    (void)peer_report(r, 0);
    return l;
}

RingLeg ring_leg(uint32_t rate, uint8_t copies, bool bytes, bool low) {
    const bool fast = !LinkIrq::can_baud(u1_hz, rate);
    if (low) {
        return fast ? ring_leg_on<LinkRingLow8>(8, rate, copies, bytes) : ring_leg_on<LinkRingLow>(7, rate, copies, bytes);
    }
    return fast ? ring_leg_on<LinkRing8>(6, rate, copies, bytes) : ring_leg_on<LinkRing>(5, rate, copies, bytes);
}

void t8_ring_copies() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    for (uint32_t k = 0; k < 4u; ++k) {
        for (uint32_t i = 0; i < 1024u; ++i) {
            copy_src[k][i] = (i + k) * 2654435761u;
        }
    }
    bool levels = true, default_whole = true, two_whole = true, four_starve = true, honest = true;
    static constexpr uint32_t rates[] = {4'266'667, 6'400'000};
    for (const uint32_t rate : rates) {
        for (uint8_t beat = 0; beat < 2u; ++beat) {
            for (uint8_t copies = 0; copies <= 4u; ++copies) {
                if (copies == 1u) {
                    continue;
                }
                const RingLeg lo = ring_leg(rate, copies, beat == 1u, true);
                const RingLeg hi = ring_leg(rate, copies, beat == 1u, false);
                print(serial, "  ", rate, beat == 1u ? " byte" : " word", ", ", copies, " copies | ring low: ",
                      lo.c.got, " in, ", lo.c.lost, " lost in ", lo.c.gaps, " gaps, ", lo.c.bad, " bad, ORE ", lo.ore,
                      " | default: ", hi.c.got, " in, ", hi.c.lost, " lost, ", hi.c.bad, " bad, ORE ", hi.ore, crlf);
                levels = levels && lo.level == DmaPriority::low && hi.level == DmaPriority::very_high;
                default_whole = default_whole && hi.up && hi.c.got == f8_frames && hi.c.gaps == 0u &&
                                hi.c.bad == 0u && hi.faults == 0u;
                if (copies <= 2u) {
                    two_whole = two_whole && lo.up && lo.c.got == f8_frames && lo.c.gaps == 0u;
                }
                if (copies == 4u) {
                    four_starve = four_starve && lo.c.lost > 0u;
                }
                honest = honest && lo.c.bad == 0u && (lo.c.lost == 0u || lo.ore != 0u);
            }
        }
    }
    judge("rx_priority reaches the silicon on USART1's ring: SxCR.PL read back low and very_high", levels);
    judge("THE DEFAULT RING LOSES NOTHING to a real sender at 4.27 and 6.4 Mbaud beside up to four copies at "
          "very_high, word and byte beats", default_whole);
    judge("two copies ranked above a ring armed low starve it of nothing", two_whole);
    judge("FOUR COPIES RANKED ABOVE A RING ARMED LOW DO STARVE IT, a real sender as the loop's thread", four_starve);
    judge("and a starved ring loses frames to an overrun it reports, never a corrupt byte", honest);
    link_release();
}

// ---------------------------------------------------------------------------
// 9 - half duplex against a second board: drive_rxd, one wire, a collision
// ---------------------------------------------------------------------------
//
// This end's HDSEL single wire on PA9, the far G0's PA10 - where the far
// end's receive pad (drive_rxd: its engine driving that wire) and its
// SWAP + HDSEL single wire (onewire, collide) both sit.

void t9_half_duplex() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    bool heard = true;
    for (const uint32_t rate : {9600u, 115'200u, 1'000'000u}) {
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 8;
        a.value = 0xA6;
        a.ms = 300;
        const bool acted = peer_act(link::Op::drive_rxd, a);
        const bool up = acted && wire_up(rate);
        const Drain d = up ? collect(static_cast<uint16_t>(30u + 8u * 16u * 1000u / rate), 8) : Drain{};
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  drive_rxd at ", rate, ": ", d.frames, " of 8, ", hex(d.first), "..", hex(d.last), ", FE ", d.fe,
              " NE ", d.ne, "; the engine ", (r.flags & link::report_mismatch) != 0u ? "BROKEN" : "whole", crlf);
        heard = heard && up && d.frames == 8u && d.fe == 0u && d.ne == 0u && (d.first & 0xFFu) == 0xA6u &&
                (d.last & 0xFFu) == 0xA6u;
    }
    judge("the HDSEL receiver hears a FOREIGN driver on its own pad: eight frames whole at 9600, 115200 and "
          "1 Mbaud", heard);

    bool shared = true;
    for (const uint32_t rate : {9600u, 115'200u, 460'800u}) {
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 8;
        a.ms = 400;
        const bool acted = peer_act(link::Op::onewire, a);
        const bool up = acted && wire_up(rate);
        wait_for_peer_window();
        uint8_t echoes = 0, replies = 0;
        const uint32_t frame_us = 12'000'000u / rate + 50u;
        for (uint8_t i = 0; up && i < 8u; ++i) {
            const uint16_t v = static_cast<uint16_t>(0x31u + 17u * i);
            U1::write_word(v);
            uint32_t e1 = 0, e2 = 0;
            const std::optional<uint16_t> echo = wire_word(4u * frame_us, e1);
            const std::optional<uint16_t> reply = wire_word(8u * frame_us, e2);
            if (echo && (*echo & 0xFFu) == v && e1 == 0u) ++echoes;
            if (reply && (*reply & 0xFFu) == ((v ^ 0xFFu) & 0xFFu) && e2 == 0u) ++replies;
            // THE TURNAROUND IS THIS END'S TOO: the reply's word is in at its
            // stop bit's middle, and a frame started then pulls the far
            // end's stop bit low under it (measured: the far receiver lost
            // step and answered inside this end's next frame). Two bit
            // times, the far end's own guard's half.
            wait_us(2'000'000u / rate + 1u);
        }
        link::Report r{};
        (void)peer_report(r, 5);
        print(serial, "  onewire at ", rate, ": ", echoes, " own echoes and ", replies, " far replies of 8; the far end "
              "heard ", r.count, crlf);
        shared = shared && up && echoes == 8u && replies == 8u && r.count == 8u;
    }
    judge("ONE WIRE, TWO CHIPS: every frame out heard back as itself and answered by the far end, both open "
          "drain, at 9600, 115200 and 460800", shared);

    {
        link::Params a{};
        a.cfg.rate = 9600;
        a.value = 0x00;
        a.aux16 = 0;
        a.ms = 400;
        const bool acted = peer_act(link::Op::collide, a);
        const bool up = acted && wire_up(9600);
        wait_for_peer_window();
        uint32_t e1 = 0, e2 = 0;
        std::optional<uint16_t> trig, hit;
        if (up) {
            U1::write_word(0xA5u);
            while (!U1::tx_empty()) {
            }
            U1::write_word(0xFFu);   // into the shifter behind the trigger: back to back
            trig = wire_word(5000u, e1);
            hit = wire_word(5000u, e2);
        }
        link::Report r{};
        (void)peer_report(r, 5);
        print(serial, "  collide at 9600: the trigger came back ", trig ? hex(static_cast<uint32_t>(*trig)) : hex(0u),
              " (errors ", hex(e1), "), the 0xFF frame ", hit ? hex(static_cast<uint32_t>(*hit)) : hex(0u),
              " (errors ", hex(e2), "); the far end heard ", r.count, crlf);
        judge("the trigger frame is heard whole", up && trig && (*trig & 0xFFu) == 0xA5u && e1 == 0u);
        judge("THE WIRED AND: the far frame pulled the shared wire under this end's 0xFF, which came back "
              "changed or with an error", up && r.count >= 1u && (!hit || (*hit & 0xFFu) != 0xFFu || e2 != 0u));
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 0 - LIN both ways against the far STM32G0
// ---------------------------------------------------------------------------
//
// 25.4.8: SBK in LIN mode sends thirteen zero bits and one bit of value 1;
// the detector, "totally independent from the normal USART receiver",
// raises LBD on ten (LBDL = 0) or eleven (LBDL = 1) zero bits and a
// delimiter - and "if the LIN mode is disabled (LINEN = 0), the receiver
// continues working as normal USART, without taking into account the
// break detection". Letter 4 measured LBD rise with LIN mode off; this
// letter stages that finding against the far engine's breaks of exact
// lengths, beside the same breaks with LIN on, and times this end's own
// breaks on the far receiver.

/// One break of `bits` bit times from the far engine at `rate` into this
/// receiver, LIN mode on (LBDL eleven or ten) or off: whether LBD rose.
struct BreakLeg {
    bool up = false;
    bool lbd = false;
    Drain d{};
    bool engine_ok = false;
};

BreakLeg break_leg(uint32_t rate, uint16_t bits, bool lin, bool eleven) {
    BreakLeg l{};
    link::Params a{};
    a.cfg.rate = rate;
    a.count = bits;
    a.ms = 200;
    link::Cfg c{};
    c.rate = rate;
    const bool acted = peer_act(link::Op::brk, a);
    l.up = acted && dut_link(c, lin, eleven);
    l.d = l.up ? collect(30) : Drain{};
    l.lbd = l.d.lbd || (U1::status() & UsartFlag::lbd) != 0u;
    link::Report r{};
    l.engine_ok = peer_report(r, 0) && (r.flags & link::report_mismatch) == 0u && r.count == bits;
    return l;
}

/// A hundredths figure, its two decimals always printed (7.05, not 7.5).
void print_x100(uint32_t v) {
    print(serial, v / 100u, (v % 100u) < 10u ? ".0" : ".", v % 100u);
}

/// THIS END'S BREAK, timed by the far receiver: SBK, then a 0x55 written
/// the moment SBK clears (during the break's stop bit, 25.6.4), so the far
/// chip stamps the break (a 0x00 with FE) and the 0x55 behind it - their
/// span, in bit times x100 on this crystal's rate, is the break and the
/// high bit that follows it.
struct SentBreak {
    bool ok = false;
    uint32_t bits_x100 = 0;
};

SentBreak sent_break(uint32_t rate, bool lin) {
    SentBreak b{};
    link::Params a{};
    a.cfg.rate = rate;
    a.count = 2;
    a.ms = 100;
    link::Cfg c{};
    c.rate = rate;
    if (!peer_act(link::Op::sink, a) || !dut_link(c, lin)) {
        return b;
    }
    wait_for_peer_window();
    U1::send_break();
    const uint32_t t1 = Ticker::millis();
    while (U1::break_pending() && Ticker::millis() - t1 < 20u) {
    }
    U1::write_word(0x55);
    wait_tc();
    link::Report r{};
    if (!peer_report(r, 5)) {
        return b;
    }
    b.ok = r.count == 2u && r.ferr == 1u && r.sum == 0x55u && r.span != 0u;
    b.bits_x100 = static_cast<uint32_t>((static_cast<uint64_t>(r.span) * rate * 100u) / 24'000'000u);
    return b;
}

void t0_lin() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    // a. The detector, LIN on: ten or eleven zero bits and a delimiter,
    //    one bit short refused; and the same breaks with LIN OFF.
    struct Case { uint16_t bits; bool eleven; bool expect; };
    static constexpr Case on_cases[] = {
        {9, false, false}, {10, false, true}, {11, false, true}, {13, false, true},
        {10, true, false}, {11, true, true}, {13, true, true},
    };
    static constexpr uint16_t off_bits[] = {9, 10, 11, 13, 20};
    bool as_told = true, engines = true;
    bool off_lbd[5] = {};
    print(serial, "  breaks at 19200, LIN on (bits:LBDL:LBD):");
    for (const Case& k : on_cases) {
        const BreakLeg l = break_leg(19'200, k.bits, true, k.eleven);
        print(serial, " ", k.bits, k.eleven ? ":11:" : ":10:", l.lbd ? "1" : "0");
        engines = engines && l.engine_ok;
        as_told = as_told && l.up && l.lbd == k.expect;
    }
    print(serial, crlf, "  the same with LIN OFF, LBDL = 0 (bits:LBD/frames/FE):");
    bool off_up = true;
    for (uint8_t i = 0; i < 5u; ++i) {
        const BreakLeg l = break_leg(19'200, off_bits[i], false, false);
        print(serial, " ", off_bits[i], ":", l.lbd ? "1" : "0", "/", l.d.frames, "/", l.d.fe);
        engines = engines && l.engine_ok;
        off_up = off_up && l.up;
        off_lbd[i] = l.lbd;
    }
    print(serial, crlf);
    judge("the far engine drew every break whole", engines);
    judge("LIN on: LBD rises for ten zero bits at LBDL = 0 and eleven at LBDL = 1, never one bit short", as_told);
    judge("LIN OFF, against 25.4.8: LBD still rises, at the same lengths as with LIN on (nine no, ten and more yes) "
          "- the detector runs whatever LINEN says (letter 4's finding, staged)",
          off_up && !off_lbd[0] && off_lbd[1] && off_lbd[2] && off_lbd[3] && off_lbd[4]);
    // b. This end's break timed by the far receiver: thirteen and one in
    //    LIN mode (25.4.8), a frame's ten and one without it (25.4.2:
    //    "a logic 1 bit at the end of the last break frame").
    const SentBreak lin = sent_break(19'200, true);
    const SentBreak plain = sent_break(19'200, false);
    print(serial, "  this end's break and the 0x55 behind it, far span at 19200: LIN ");
    print_x100(lin.bits_x100);
    print(serial, " bit times, without LIN ");
    print_x100(plain.bits_x100);
    print(serial, crlf);
    judge("the LIN break is thirteen bits and its one high bit, timed by a foreign receiver (14 +- 0.5)",
          lin.ok && lin.bits_x100 >= 1350u && lin.bits_x100 <= 1450u);
    judge("without LIN a break is a frame of ten and its one high bit (11 +- 0.5)",
          plain.ok && plain.bits_x100 >= 1050u && plain.bits_x100 <= 1150u);
    link_release();
}

// ---------------------------------------------------------------------------
// r - IrDA SIR both ways against the far STM32G0 (and RS-485, refused)
// ---------------------------------------------------------------------------
//
// THIS USART HAS NO DRIVER ENABLE (25.3, table 148 has no such row): the
// pair's RS-485 letter is the far G0's, timed on its own PB3, this board's
// PB13 left an input under it. IrDA SIR (25.4.12) against the far ST
// ENDEC: "the transmit encoder output has the opposite polarity to the
// decoder input", so a bare wire between two of them needs one inversion
// each way - the far G0's TXINV and RXINV, which its usart_peer applies in
// IrDA (its own suite's letter r reads them on the pad). Normal mode runs
// at PSC = 1, the one value 25.6.7 allows there; low power at PSC 49 -
// 1.84 MHz, inside the 1.42..2.12 MHz the chapter names, a pulse of three
// periods.

/// The link as IrDA SIR at `rate` through the resource: the block reset,
/// 8N1, IREN, then the pads.
bool dut_irda(uint32_t rate, const IrdaConfig& cfg) {
    link_task_down();
    U1Tx::input(PinPull::up);
    U1::bus_clock(true);
    U1::enable(false);
    U1::reset();
    const std::optional<uint16_t> brr = usart_brr(u1_hz, rate);
    if (!brr || !U1::configure(UartFormat{}, *brr) || !U1::irda(cfg)) {
        return false;
    }
    U1::transmitter(true);
    U1::receiver(true);
    U1::enable(true);
    wait_us(2u * (1'000'000u / rate) + 20u);
    U1Tx::function(PinFunction::af7, {.speed = PinSpeed::very_high});
    U1Rx::function(PinFunction::af7, {.pull = PinPull::up});
    u1_flush();
    return true;
}

/// The transmit pad's idle level, then the first pulse after a write: its
/// level and width in DWT cycles.
struct PadPulse {
    bool seen = false;
    bool idle_high = false;
    bool pulse_high = false;
    uint32_t width = 0;
};

PadPulse ir_first_pulse(uint8_t v) {
    PadPulse p{};
    InterruptGuard guard;
    volatile uint32_t& idr = GPIOA->IDR;
    const uint32_t mask = U1Tx::mask;
    const uint32_t idle = idr & mask;
    p.idle_high = idle != 0u;
    while (!U1::tx_empty()) {
    }
    U1::write_word(v);
    // Two tight loops of one load and one compare each, the cycle counter
    // read as each ends: the width is good to a loop's period.
    uint32_t k = 2'000'000u;
    while ((idr & mask) == idle && --k != 0u) {
    }
    const uint32_t a = CycleCounter::now();
    if (k == 0u) {
        return p;
    }
    k = 2'000'000u;
    while ((idr & mask) != idle && --k != 0u) {
    }
    const uint32_t e = CycleCounter::now();
    if (k == 0u) {
        return p;
    }
    p.width = e - a;
    p.pulse_high = !p.idle_high;
    p.seen = true;
    return p;
}

struct IrTx {
    bool up = false;
    PadPulse pad{};
    link::Report r{};
    bool whole = false;
};

/// This end transmits SIR, the far decoder sinks: 32 counting frames from
/// 0x30, the first one's pulse read on the pad.
IrTx ir_tx_leg(uint32_t rate, const IrdaConfig& cfg) {
    IrTx t{};
    link::Params a{};
    a.cfg.mode = link::Mode::ircom;
    a.cfg.rate = rate;
    a.count = 32;
    a.ms = bound_ms(rate, 64);
    const bool acted = peer_act(link::Op::sink, a);
    t.up = acted && dut_irda(rate, cfg);
    uint16_t sum = 0;
    if (t.up) {
        wait_for_peer_window();
        t.pad = ir_first_pulse(0x30);
        sum = 0x30;
        for (uint8_t i = 1; i < 32u; ++i) {
            const uint8_t v = static_cast<uint8_t>(0x30u + i);
            put1(v);
            sum = static_cast<uint16_t>(sum + v);
        }
        wait_tc();
    }
    (void)peer_report(t.r, 5);
    t.whole = t.up && t.r.count == 32u && t.r.sum == sum && t.r.ferr == 0u && !leg_refused;
    return t;
}

struct IrRx {
    bool up = false;
    uint16_t got = 0, good = 0;
    uint32_t errs = 0;
    uint8_t first = 0;
};

/// The far end transmits SIR (`txpl` != 0 its low-power pulse), this end
/// decodes in `cfg`'s mode: 32 counting frames from 0x60.
IrRx ir_rx_leg(uint32_t rate, uint8_t txpl, const IrdaConfig& cfg = {}) {
    IrRx x{};
    link::Params a{};
    a.cfg.mode = link::Mode::ircom;
    a.cfg.rate = rate;
    a.cfg.txpl = txpl;
    a.count = 32;
    a.value = 0x60;
    // THE TURNAROUND: until this end's encoder is up, its UART idle (high)
    // stands on the far decoder's input - behind the far RXINV, a standing
    // pulse its receiver chews as frames - and an ST ENDEC does not encode
    // while its receiver is busy (25.4.12, RM0444 33.5.18: half duplex).
    // So the far sender's first frame waits 20 ms (send's lead-in) past
    // the settle, long after this end is up and the far receiver quiet:
    // without it, the first start pulse was missed one leg in a few
    // (measured, the first byte read one bit late).
    a.aux16 = 20;
    a.ms = static_cast<uint16_t>(bound_ms(rate, 64) + 20u);
    const bool acted = peer_act(link::Op::send, a);
    x.up = acted && dut_irda(rate, cfg);
    const uint32_t t1 = Ticker::millis();
    // Thirty-two and no more: the far chip's return to command mode after
    // them is one more edge on the line, a frame after the last.
    while (x.up && x.got < 32u && Ticker::millis() - t1 < a.ms) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::rxne) == 0u) {
            continue;
        }
        const uint8_t b = U1::read_data();
        const uint32_t flags = st & (UsartFlag::fe | UsartFlag::pe | UsartFlag::ne | UsartFlag::ore);
        if (flags == 0u && b == static_cast<uint8_t>(0x60u + x.good) && x.got == x.good) {
            ++x.good;
        }
        if (x.got == 0u) {
            x.first = b;
        }
        x.errs |= flags;
        ++x.got;
    }
    link::Report r{};
    (void)peer_report(r, 0);
    return x;
}

void print_pulse(const PadPulse& p, uint32_t bit_cycles) {
    if (!p.seen) {
        print(serial, "no pulse");
        return;
    }
    const uint32_t x10 = (p.width * 160u) / bit_cycles;
    print(serial, "idle ", p.idle_high ? "HIGH" : "LOW", ", a ", p.pulse_high ? "HIGH" : "LOW", " pulse of ",
          p.width, " cycles (", x10 / 10u, ".", x10 % 10u, " sixteenths of a bit)");
}

void tr_irda() {
    print(serial, "  RS-485: SKIPPED, no verdict claimed - this USART has no driver enable (RM0390 25.3, table 148); "
          "the pair's RS-485 letter is the far G0's", crlf);
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    // a. The driver's own refusal: normal mode takes PSC = 1 alone (25.6.7).
    U1::enable(false);
    U1::reset();
    const bool refused = !U1::irda(IrdaConfig{false, 4}) && !U1::irda_enabled();
    bench.verdict("irda() refuses normal mode with a prescaler other than 1 (25.6.7: \"PSC must be set to "
                  "00000001\")", refused);
    // b. This end to the far end, normal and low-power, at 9600 and 115200.
    {
        constexpr uint32_t bit9600 = SysClock::hz / 9'600u;
        constexpr uint32_t bit115 = SysClock::hz / 115'200u;
        const IrTx n96 = ir_tx_leg(9'600, IrdaConfig{});
        const IrTx n115 = ir_tx_leg(115'200, IrdaConfig{});
        const IrTx lp96 = ir_tx_leg(9'600, IrdaConfig{true, 49});
        const IrTx lp115 = ir_tx_leg(115'200, IrdaConfig{true, 49});
        print(serial, "  SIR out at 9600: ");
        print_pulse(n96.pad, bit9600);
        print(serial, "; the far decoder took ", n96.r.count, " frames, ferr ", n96.r.ferr, crlf);
        print(serial, "  at 115200: ");
        print_pulse(n115.pad, bit115);
        print(serial, "; ", n115.r.count, " frames", crlf);
        print(serial, "  low power (PSC 49, three periods = 294 cycles) at 9600: ");
        print_pulse(lp96.pad, bit9600);
        print(serial, "; ", lp96.r.count, " frames; at 115200 a pulse of ", lp115.pad.width, " cycles, ", lp115.r.count,
              " frames", crlf);
        judge("the SIR output idles LOW and every zero is a HIGH pulse of 3/16 of a bit (2.5..3.5 sixteenths at "
              "9600 and 115200)",
              n96.pad.seen && !n96.pad.idle_high && n96.pad.pulse_high && n115.pad.seen &&
                  (n96.pad.width * 160u) / bit9600 >= 25u && (n96.pad.width * 160u) / bit9600 <= 35u &&
                  (n115.pad.width * 160u) / bit115 >= 25u && (n115.pad.width * 160u) / bit115 <= 35u);
        judge("the low-power pulse is three PSC periods whatever the rate (294 cycles, +-12)",
              lp96.pad.seen && lp115.pad.seen && lp96.pad.width >= 282u && lp96.pad.width <= 306u &&
                  lp115.pad.width >= 282u && lp115.pad.width <= 306u);
        judge("the far ST decoder, behind its RXINV, takes every frame, normal and low-power, at 9600 and 115200",
              n96.whole && n115.whole && lp96.whole && lp115.whole);
    }
    // c. The far end to this end: its encoder behind TXINV, normal and
    //    low-power (txpl 40 ticks, 1.69 us on the far kernel). A low-power
    //    pulse is three periods of a clock no slower than 1.42 MHz, so at
    //    9600 it is a sixtieth of a bit: this receiver takes it in normal
    //    mode (PSC 1) and in its own low-power mode (PSC 49, 25.4.12: "a
    //    valid low is accepted only if its duration is greater than 2
    //    periods of the IrDA low-power baud clock").
    {
        const IrRx n96 = ir_rx_leg(9'600, 0);
        const IrRx n115 = ir_rx_leg(115'200, 0);
        const IrRx lp115 = ir_rx_leg(115'200, 40);
        const IrRx lp96 = ir_rx_leg(9'600, 40);
        const IrRx lp96_lp = ir_rx_leg(9'600, 40, IrdaConfig{true, 49});
        print(serial, "  SIR in, good/received (errors, first byte): 9600 ", n96.good, "/", n96.got, " (",
              hex(n96.errs),
              ", ", hex(n96.first), "); 115200 ", n115.good, "/", n115.got, " (", hex(n115.errs), ", ", hex(n115.first),
              "); low-power pulses at 115200 ", lp115.good, "/", lp115.got, " (", hex(lp115.errs), ", ",
              hex(lp115.first), "), at 9600 ", lp96.good, "/", lp96.got, " (", hex(lp96.errs), ", ", hex(lp96.first),
              ") received in normal mode, ", lp96_lp.good, "/", lp96_lp.got, " (", hex(lp96_lp.errs), ", ",
              hex(lp96_lp.first), ") in low-power mode", crlf);
        judge("this decoder takes the far encoder's frames whole: normal pulses at 9600 and 115200, low-power at "
              "115200",
              n96.good == 32u && n96.got == 32u && n115.good == 32u && n115.got == 32u && lp115.good == 32u &&
                  lp115.got == 32u);
        judge("at 9600 the low-power pulse, a sixtieth of a bit, is taken whole in normal mode and in low-power mode "
              "(PSC 49)",
              lp96.good == 32u && lp96.got == 32u && lp96_lp.good == 32u && lp96_lp.got == 32u);
    }
    // d. The far chip's ceiling: SIR to 115.2 kbaud (RM0444 33.5.18) - a
    //    sink asked at 230400 is refused.
    {
        link::Params a{};
        a.cfg.mode = link::Mode::ircom;
        a.cfg.rate = 230'400;
        a.count = 4;
        a.ms = 50;
        const bool acted = peer_act(link::Op::sink, a);
        link::Report r{};
        const bool reported = acted && peer_report(r, 60);
        print(serial, "  SIR at 230400 asked of the far chip: ",
              (r.flags & link::report_cfg_failed) != 0u ? "refused" : "SERVED", crlf);
        leg_refused = false;
        bench.verdict("the far STM32G0B1RE refuses SIR above 115200, its chapter's ceiling",
                      reported && (r.flags & link::report_cfg_failed) != 0u);
    }
    link_release();
}

// ---------------------------------------------------------------------------
// s - mute mode and the multiprocessor wake from a foreign sender, and
//     ES0298 2.13.3
// ---------------------------------------------------------------------------
//
// 25.4.6 across the wire, the far G0 the sender: the IDLE-LINE wake (RWU
// set after a burst's first frame, the rest of the burst dropped, the idle
// frame the wake - and RWU set before ANY frame, which 25.6.4's RWU note
// says leaves the idle-line wake unable to work); the ADDRESS-MARK wake on
// the far `send_mpcm` - four groups of an address and its data - with this
// USART's one address width, FOUR bits, on an 8-bit word (the mark bit 7)
// and a 9-bit one (the mark bit 8), the same groups the far G0's letter `s`
// reads in its four- and seven-bit modes; and ES0298 2.13.3: "the USART
// receiver recognizes a valid address with a parity error, it exits from
// Mute mode without setting the Parity Error flag" - no workaround - with
// the controls that show the parity error is real.

struct MuteLeg {
    bool up = false;
    bool rwu_set = false;          ///< RWU read right after mute()
    uint8_t n = 0;
    uint16_t v[24] = {};
    uint32_t pe = 0;               ///< frame i carried PE: bit i
    uint16_t idles_muted = 0;
};

/// What a muted receiver lets through. `mute_after` frames are taken before
/// RWU is set; 0 sets it before the stream, 0xFF never.
MuteLeg mute_collect(uint8_t mute_after, uint16_t mask) {
    MuteLeg l{};
    USART_TypeDef& u = U1::regs();
    if (mute_after == 0u) {
        l.rwu_set = U1::mute() && U1::muted();
    }
    const uint32_t t0 = Ticker::millis();
    uint32_t last = t0;
    uint8_t taken = 0;
    while (Ticker::millis() - t0 < 400u) {
        const uint32_t st = u.SR;
        if ((st & UsartFlag::rxne) != 0u) {
            const uint16_t w = static_cast<uint16_t>(u.DR & mask);   // SR then DR: this frame's flags
            if (l.n < 24u) {
                if ((st & UsartFlag::pe) != 0u) l.pe |= 1UL << l.n;
                l.v[l.n++] = w;
            }
            if (++taken == mute_after) {
                l.rwu_set = U1::mute() && U1::muted();
            }
            last = Ticker::millis();
            continue;
        }
        if ((st & UsartFlag::idle) != 0u) {
            (void)u.DR;
            if (U1::muted()) ++l.idles_muted;
        }
        if (taken != 0u && Ticker::millis() - last > 40u) {
            break;
        }
    }
    return l;
}

bool seq_is(const MuteLeg& l, std::initializer_list<uint16_t> want) {
    if (l.n != want.size()) {
        return false;
    }
    uint8_t i = 0;
    for (const uint16_t w : want) {
        if (l.v[i++] != w) {
            return false;
        }
    }
    return true;
}

void print_seq(const MuteLeg& l) {
    for (uint8_t i = 0; i < l.n; ++i) {
        print(serial, " ", hex(l.v[i]), (l.pe >> i) & 1u ? "(PE)" : "");
    }
    print(serial, crlf);
}

/// The far send_mpcm's four groups (address, data frames), its data
/// counting up from 0x10, into this end at `here` in mute mode `m`, RWU
/// set before the stream unless `mute_first` is false.
MuteLeg mpcm_leg(const link::Cfg& far, const link::Cfg& here, const MuteConfig& m, const uint8_t (&groups)[8],
                 uint16_t mask, bool mute_first = true) {
    link::Params a{};
    a.cfg = far;
    a.groups = 4;
    a.value = 0x10;
    a.ms = 200;
    const bool acted = peer_act(link::Op::send_mpcm, a, groups, 8);
    MuteLeg l{};
    const bool up = acted && dut_link(here) && U1::mute_mode(m);
    if (up) {
        l = mute_collect(mute_first ? 0u : 0xFFu, mask);
    }
    l.up = up;
    U1::unmute();
    (void)U1::mute_mode({});
    link::Report r{};
    (void)peer_report(r, 0);
    return l;
}

void ts_mute() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    link::Cfg c{};
    c.rate = 115'200;
    // a. THE IDLE-LINE WAKE: three bursts of eight counting frames a
    //    millisecond apart, RWU set after the first frame - and the same
    //    with RWU set before any frame.
    for (uint8_t after = 1; after <= 2u; ++after) {
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.aux16 = 3;
        a.aux8 = 10;
        a.ms = 200;
        a.pattern = link::pattern_counting;
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(c) && U1::mute_mode({.wake = MuteWake::idle_line});
        const MuteLeg l = up ? mute_collect(after == 1u ? 1u : 0u, 0xFFu) : MuteLeg{};
        const bool rwu_end = U1::muted();
        U1::unmute();
        link::Report r{};
        (void)peer_report(r, 0);
        if (after == 1u) {
            print(serial, "  idle-line wake, RWU set after the first frame (", l.rwu_set ? "set" : "REFUSED",
                  ", IDLE flags while muted ", l.idles_muted, ", RWU at the end ", rwu_end ? "SET" : "clear", "):");
            print_seq(l);
            bool tail = l.n == 17u;
            for (uint8_t i = 0; tail && i < 16u; ++i) {
                tail = l.v[1u + i] == 8u + i;
            }
            judge("the idle-line wake: RWU drops the rest of the burst, the idle frame wakes the receiver - both "
                  "later bursts whole and no IDLE flag for the wake",
                  up && l.rwu_set && l.v[0] == 0u && tail && l.idles_muted == 0u);
        } else {
            print(serial, "  idle-line wake, RWU set BEFORE any frame (", l.rwu_set ? "set" : "REFUSED",
                  ", RWU at the end ", rwu_end ? "SET" : "clear", "): ", l.n, " frames:");
            print_seq(l);
            // 25.6.4's RWU note: an idle-line wake wants a byte received
            // before RWU is set - on an idle line the bit does not hold.
            judge("RWU written before any frame does not hold (25.6.4's note): the receiver stays awake and "
                  "takes all three bursts",
                  up && !l.rwu_set && l.n == 24u && l.v[0] == 0u && l.v[23] == 23u);
        }
    }
    // b. THE ADDRESS MARK on an 8-bit word, a 4-bit address 5: groups
    //    (0x83, 2) (0x85, 3) (0x86, 2) (0x95, 2) - 0x85 and 0x95 both
    //    match four bits (the far G0's seven-bit mode tells them apart).
    {
        static constexpr uint8_t groups[8] = {0x83, 2, 0x85, 3, 0x86, 2, 0x95, 2};
        const MuteLeg l = mpcm_leg(c, c, {.wake = MuteWake::address_mark, .address = 0x5}, groups, 0xFFu);
        print(serial, "  8-bit words, address 5:");
        print_seq(l);
        judge("a 4-bit address on an 8-bit word: 0x85 and 0x95 wake the receiver with their data, 0x83 and 0x86 "
              "mute it",
              l.up && l.rwu_set && seq_is(l, {0x85, 0x12, 0x13, 0x14, 0x95, 0x17, 0x18}));
    }
    // c. A 9-BIT WORD, the mark in bit 8: groups 0x05, 0x85, 0x45, 0x0A -
    //    the first three match four bits.
    {
        link::Cfg c9 = c;
        c9.bits = link::bits9_low;
        static constexpr uint8_t groups[8] = {0x05, 2, 0x85, 3, 0x45, 2, 0x0A, 2};
        const MuteLeg l = mpcm_leg(c9, c9, {.wake = MuteWake::address_mark, .address = 0x5}, groups, 0x1FFu);
        print(serial, "  9-bit words, address 5:");
        print_seq(l);
        judge("a 9-bit word's mark is bit 8, and the 4-bit address takes every group whose low nibble is 5",
              l.up && seq_is(l, {0x105, 0x10, 0x11, 0x185, 0x12, 0x13, 0x14, 0x145, 0x15, 0x16}));
    }
    // d. ES0298 2.13.3: this end 8E1 (the mark bit 7, before the parity),
    //    the far end 8O1 - every frame a parity error - at address 5. Muted
    //    by SOFTWARE (RWU written) before groups (0x83, 1) (0x85, 2) (0x86,
    //    1) (0x95, 1); muted by the HARDWARE (a non-matching address sets
    //    RWU) in groups (0x86, 1) (0x85, 1) (0x83, 1) (0x95, 1), the
    //    receiver awake at the start. The controls: the same frames with no
    //    mute mode (PE on every one), and the address mark armed with the
    //    receiver awake when a matching address comes.
    {
        link::Cfg ce = c;
        ce.parity = 1;
        link::Cfg co = c;
        co.parity = 2;
        static constexpr uint8_t groups[8] = {0x83, 1, 0x85, 2, 0x86, 1, 0x95, 1};
        const MuteLeg soft = mpcm_leg(co, ce, {.wake = MuteWake::address_mark, .address = 0x5}, groups, 0xFFu);
        static constexpr uint8_t hw_groups[8] = {0x86, 1, 0x85, 1, 0x83, 1, 0x95, 1};
        const MuteLeg hard =
            mpcm_leg(co, ce, {.wake = MuteWake::address_mark, .address = 0x5}, hw_groups, 0xFFu, false);
        static constexpr uint8_t awake_groups[8] = {0x85, 2, 0x86, 1, 0x95, 1, 0x83, 1};
        const MuteLeg awake =
            mpcm_leg(co, ce, {.wake = MuteWake::address_mark, .address = 0x5}, awake_groups, 0xFFu, false);
        const MuteLeg plain = mpcm_leg(co, ce, {.wake = MuteWake::idle_line}, groups, 0xFFu, false);
        print(serial, "  8E1 here, 8O1 there, address 5, muted by software first:");
        print_seq(soft);
        print(serial, "  the same, awake at the start, muted by 0x86 and by 0x83:");
        print_seq(hard);
        print(serial, "  the same, the address mark armed and the receiver awake for 0x85:");
        print_seq(awake);
        print(serial, "  the same with no mute at all:");
        print_seq(plain);
        judge("the control: with no mute mode every one of the far frames carries PE",
              plain.up && plain.n == 9u && plain.pe == 0x1FFu);
        judge("ES0298 2.13.3 REPRODUCES after a SOFTWARE mute: the first matching address with a parity error "
              "wakes the receiver with NO PE, its data and the next address (0x95, after the hardware's mute) "
              "with PE",
              soft.up && seq_is(soft, {0x85, 0x11, 0x12, 0x95, 0x14}) && soft.pe == 0x1Eu);
        judge("and NOT after the hardware's own mute: a non-matching address mutes the receiver, and the "
              "matching one after it wakes it WITH PE - every address of the leg flagged",
              hard.up && seq_is(hard, {0x85, 0x11, 0x95, 0x13}) && hard.pe == 0xFu);
        judge("awake under the address mark the matching address is checked as any frame: 0x85 with PE",
              awake.up && awake.n != 0u && awake.v[0] == 0x85u && (awake.pe & 1u) != 0u);
    }
    link_release();
}

// ---------------------------------------------------------------------------
// u - set_baud at run time, and the TC and LBD interrupt enables
// ---------------------------------------------------------------------------
//
// set_baud() moves a running Uart task to another rate with the clock
// standing still. Each rung brings the task up at 115200 - a rate the far
// chip is NOT speaking - and moves it with set_baud() alone, then judges
// both directions by the far chip: its `send` of 64 counting frames
// received whole and in order through the task's ring, and n frames the
// task sends gapless timed on the far ruler (the calibrate op, letter 2's
// bound). Then two interrupt enables no transport arms: TCIE, its vector
// timed against the last frame on the wire, and LBDIE, its vector on a
// break from the far engine in LIN mode. (CTSIE needs a CTS wire, and the
// link has none.)

constexpr uint8_t owner_probe = 9;   ///< USART1's vector is this letter's
volatile uint16_t probe_tc = 0;
volatile uint16_t probe_lbd = 0;
volatile uint32_t probe_tc_at = 0;

/// The letter's body of USART1's vector: each enabled source counted and
/// consumed by disarming it (TC) or clearing it (LBD).
void probe_usart1() {
    const uint32_t st = U1::status();
    const uint32_t cr1 = U1::regs().CR1;
    if ((cr1 & USART_CR1_TCIE) != 0u && (st & UsartFlag::tc) != 0u) {
        probe_tc_at = CycleCounter::now();
        probe_tc = static_cast<uint16_t>(probe_tc + 1u);
        U1::tc_interrupt(false);
    }
    if ((U1::regs().CR2 & USART_CR2_LBDIE) != 0u && (st & UsartFlag::lbd) != 0u) {
        probe_lbd = static_cast<uint16_t>(probe_lbd + 1u);
        U1::clear_flags(UsartFlag::lbd);
    }
}

void tu_baud_irqs() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    const int32_t base = clock_baseline();
    static constexpr uint32_t rungs[] = {9600, 921'600, 2'000'000};
    for (const uint32_t rate : rungs) {
        link::Cfg c{};
        c.rate = rate;
        // a. The far `send` into the task, moved there by set_baud().
        {
            link::Params a{};
            a.cfg = c;
            a.count = 64;
            a.value = 0;
            a.pattern = link::pattern_counting;
            a.aux16 = 100;   // ms after the ack: the task's bring-up fits in it
            a.ms = 2000;
            const bool acted = peer_act(link::Op::send, a);
            link_task_down();
            link_owner = 1;
            const bool up = acted && LinkIrq::init(clock, 115'200);
            const bool moved = up && LinkIrq::set_baud(SysClock::hz, rate);
            uint8_t ore = 0;
            const uint32_t got = moved ? task_receive<LinkIrq>(64, 100u + bound_ms(rate, 64) + 200u, ore) : 0u;
            const uint32_t now_baud = LinkIrq::actual_baud(u1_hz);
            link_task_down();
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  ", rate, ": the task up at 115200, set_baud() -> ", now_baud, " by its register; ",
                  got, " of 64 of the far send in order, ORE ", ore, crlf);
            judge(num(rate), " baud: a task brought up at 115200 and moved by set_baud() alone receives the far "
                             "chip's 64 frames whole and in order",
                  moved && got == 64u);
        }
        // b. The task's own frames, timed by the far receiver.
        {
            const uint16_t n = span_frames(rate);
            link::Params a{};
            a.cfg = c;
            a.count = n;
            a.ms = bound_ms(rate, n);
            const bool acted = peer_act(link::Op::calibrate, a);
            link_task_down();
            link_owner = 1;
            const bool up = acted && LinkIrq::init(clock, 115'200);
            const bool moved = up && LinkIrq::set_baud(SysClock::hz, rate);
            if (moved) {
                wait_for_peer_window();
                for (uint16_t i = 0; i < n; ++i) {
                    const uint32_t t0 = Ticker::millis();
                    while (!LinkIrq::write_byte(static_cast<uint8_t>(0x55u + i)) && Ticker::millis() - t0 < 50u) {
                    }
                }
                const uint32_t t0 = Ticker::millis();
                while (!LinkIrq::tx_idle() && Ticker::millis() - t0 < 100u) {
                }
            }
            link_task_down();
            link::Report r{};
            const bool have = peer_report(r, 5);
            const int32_t ppm = static_cast<int16_t>(r.aux16);
            const int32_t expect = base + divisor_ppm(rate);
            print(serial, "  ", rate, ": ", n, " frames from the task, the far chip counted ", r.count, " (ferr ",
                  r.ferr, ") and times them ");
            print_ppm(ppm);
            print(serial, " against the divisor's ", divisor_ppm(rate), " and the clocks' ", base, crlf);
            judge(num(rate), " baud: the task's transmitter after set_baud(), gapless and on the rate - the far "
                             "chip counts every frame and times them within 200 ppm of the divisor's",
                  moved && have && r.count == n && r.ferr == 0u && r.ovf == 0u && near(ppm, expect, 200));
        }
    }

    // c. TCIE: sixteen frames written gapless at 115200, TCIE armed after
    //    the last write. TXE was set at that write, so the shifter held the
    //    fifteenth (started up to one frame earlier) and TDR takes the
    //    sixteenth: TC rises between ONE and TWO frame times after the
    //    write (86.8 and 173.6 us), and the vector lands a microsecond
    //    after it at most (its entry and the stamp, well under 180 cycles).
    {
        link::Cfg c{};
        c.rate = 115'200;
        link::Params a{};
        a.cfg = c;
        a.count = 16;
        a.ms = bound_ms(c.rate, 16);
        const bool acted = peer_act(link::Op::calibrate, a);
        const bool up = acted && dut_link(c);
        uint32_t last_write = 0;
        if (up) {
            wait_for_peer_window();
            probe_tc = 0;
            link_owner = owner_probe;
            Nvic::enable(U1::irq);
            for (uint16_t i = 0; i < 16u; ++i) {
                const uint32_t t0 = Ticker::millis();
                while (!U1::tx_empty() && Ticker::millis() - t0 < 5u) {
                }
                U1::write_word(static_cast<uint16_t>(0x55u + i));
            }
            last_write = CycleCounter::now();
            U1::tc_interrupt(true);
            const uint32_t t0 = Ticker::millis();
            while (probe_tc == 0u && Ticker::millis() - t0 < 10u) {
            }
            Nvic::disable(U1::irq);
            U1::tc_interrupt(false);
            link_owner = 0;
        }
        link::Report r{};
        const bool have = peer_report(r, 5);
        const uint32_t after_ns = static_cast<uint32_t>(
            (static_cast<uint64_t>(probe_tc_at - last_write) * 1'000'000'000ull) / SysClock::hz);
        print(serial, "  TCIE: the vector ran ", probe_tc, " time(s), ", after_ns / 1000u, ".",
              (after_ns % 1000u) / 100u, " us after the last write; the far chip counted ", r.count, crlf);
        judge("TCIE: the TRANSMISSION-COMPLETE vector runs once, between one and two frame times after the last "
              "write (86.8..174.6 us at 115200) - the last frame through, the far chip counting all sixteen",
              up && probe_tc == 1u && after_ns >= 86'800u && after_ns <= 174'600u && have && r.count == 16u);
    }

    // d. LBDIE: one 13-bit break from the far engine at 19200, LIN mode on
    //    with LBDL at eleven bits (letter 0's detection), the flag counted
    //    by the vector alone.
    {
        link::Params a{};
        a.cfg.rate = 19'200;
        a.count = 13;
        a.ms = 200;
        link::Cfg c{};
        c.rate = 19'200;
        const bool acted = peer_act(link::Op::brk, a);
        const bool up = acted && dut_link(c, true, true);
        if (up) {
            probe_lbd = 0;
            link_owner = owner_probe;
            U1::break_interrupt(true);
            Nvic::enable(U1::irq);
            const uint32_t t0 = Ticker::millis();
            while (probe_lbd == 0u && Ticker::millis() - t0 < 30u) {
            }
            wait_ms(5);
            Nvic::disable(U1::irq);
            U1::break_interrupt(false);
            link_owner = 0;
        }
        const bool flag_left = (U1::status() & UsartFlag::lbd) != 0u;
        link::Report r{};
        const bool have = peer_report(r, 0);
        print(serial, "  LBDIE: the far break of 13 bits, the vector ran ", probe_lbd, " time(s), LBD left standing ",
              flag_left, crlf);
        judge("LBDIE: the far chip's break reaches the vector ONCE in LIN mode, and the vector's clear leaves no "
              "LBD standing",
              up && have && r.count == 13u && probe_lbd == 1u && !flag_left);
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
    bool edge = false;
    if (o == owner_probe) {
        probe_usart1();
        return;
    }
    if (o == 1u) {
        edge = LinkIrq::isr();
    } else if (o == 2u) {
        edge = LinkDma::isr();
    } else if (o == 3u) {
        edge = LinkIrq8::isr();
    } else if (o == 4u) {
        edge = LinkDma8::isr();
    } else if (o == 5u) {
        edge = LinkRing::isr();
    } else if (o == 6u) {
        edge = LinkRing8::isr();
    } else if (o == 7u) {
        edge = LinkRingLow::isr();
    } else if (o == 8u) {
        edge = LinkRingLow8::isr();
    }
    if (edge) {
        link_edges = link_edges + 1u;
    }
}
extern "C" void DMA2_Stream2_IRQHandler() {
    const uint8_t o = link_owner;
    bool edge = false;
    if (o == 2u) {
        edge = LinkDma::dma_isr();
    } else if (o == 4u) {
        edge = LinkDma8::dma_isr();
    } else if (o == 5u) {
        edge = LinkRing::dma_isr();
    } else if (o == 6u) {
        edge = LinkRing8::dma_isr();
    } else if (o == 7u) {
        edge = LinkRingLow::dma_isr();
    } else if (o == 8u) {
        edge = LinkRingLow8::dma_isr();
    }
    if (edge) {
        link_edges = link_edges + 1u;
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
    } else if (owner == 7u) {
        // The glue a transmit-side writer binds (util/stream.hpp's
        // EdgeSink): the edge read right after the body that raises it.
        handler_said = Stm32f4Platform<>::in_handler() ? 2u : 1u;
        edge = LoopIrq::isr();
        if (LoopIrq::tx_drained()) {
            tx_edges = tx_edges + 1u;
        }
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
    bench.letter('h', "the transmit room and edge, and in_handler()", th_room_edge);
    bench.second_letter('1', "the rendezvous, and this crystal against the far chip's", t1_link);
    bench.second_letter('2', "the baud matrix across the wire, timed by the far chip", t2_rates);
    bench.second_letter('3', "the frame matrix across the wire", t3_frames);
    bench.second_letter('4', "the receiver's errors from a foreign sender", t4_errors);
    bench.second_letter('5', "ES0298 2.13.2 staged: PE in full duplex while transmitting", t5_parity_full_duplex);
    bench.second_letter('6', "the engine's shapes: the start's width, the range, one stretched cell", t6_shapes);
    bench.second_letter('7', "ES0298 2.13.1 staged: IDLE and the engine's edge from a faster sender", t7_idle_faster);
    bench.second_letter('8', "the ring under copies, with a real sender", t8_ring_copies);
    bench.second_letter('9', "half duplex against a second board: drive_rxd, one wire, a collision", t9_half_duplex);
    bench.second_letter('0', "LIN both ways: the break at LBDL's two lengths, with LIN off, sent and timed", t0_lin);
    bench.second_letter('r', "IrDA SIR both ways against the far ST ENDEC; RS-485 refused (no DE)", tr_irda);
    bench.second_letter('s', "mute and the multiprocessor wake from a foreign sender; ES0298 2.13.3", ts_mute);
    bench.second_letter('u', "set_baud() at run time, and the TC and LBD interrupt enables", tu_baud_irqs);

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
