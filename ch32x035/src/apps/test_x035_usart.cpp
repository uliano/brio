// test_x035_usart - the reference bench suite for the USARTs of the
// CH32X035 (RM ch. 14): the divisor in HCLK, the frame formats and the
// refusals, the modes and the exclusions the chapter states between them,
// a frame's own length on the wire timed by the transmitter, the break,
// the column the probe's pads forbid - and, across the desk's link to a
// CH32V006's usart_peer, the bytes, the rates as a foreign receiver times
// them, the HSI against a crystal, and what only a second chip can drive.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// THE CONSOLE IS USART2 on PA2/PA3 over the probe's serial. The instance
// under test is USART4 on its default column, PB0 (TX) and PB1 (RX) -
// the desk's link: PB0 wired to the CH32V006's PB1 (its USART2 RX on
// column 4), PB1 to its PB0 (TX). The suite has TWO SETS:
//
//  - `z`, THE SINGLE-BOARD LETTERS, runnable with the far board dark (its
//    console, or its usart_peer searching: either leaves both link pads
//    floating inputs, usart_link.hpp's dark discipline): letters d, e and
//    h transmit on PB0 into that input and judge nothing the far end does.
//  - `y`, THE TWO-BOARD LETTERS, against the CH32V006 running usart_peer
//    on its 24 MHz crystal: driven in band over the link itself
//    (usart_link.hpp), its rendezvous a letter, and every leg judged from
//    what the far chip reports - FAILING loudly when nobody answers. A
//    leg the peer REFUSES (a capability its silicon has not got) is
//    SKIPPED with the reason and judges nothing.
//
// What is exercised, letter by letter:
//   a  the divisor: the console's BRR for 115200 at 48 MHz, the rate it
//      really gives, and the arithmetic at the chapter's 3 Mbaud edge
//   b  the frame, register by register on USART4 with the port disabled:
//      8N1, 8E1, 7O1, 9N1 and the stop codes as CTLR1 and CTLR2 hold them,
//      and the refusals - 7N1, a divisor below 16
//   c  the modes and their exclusions, register by register: LIN, half
//      duplex, IrDA, the smartcard and the synchronous clock, each refused
//      where 14.4 .. 14.7 say it must be and each read back where it is not
//   d  a frame's length: one byte of the console timed from the DATAR write
//      to TC on the STK counter, against ten bit times of the divisor; and
//      a run of frames timed against its wire at the rate asked, on the
//      console at 115200 and on USART4 (PB0) at 115200, 1 and 3 Mbaud
//   e  the break: USART4 opened on PB0/PB1, SBK set, and the time until the
//      hardware clears it on the break frame's stop bit
//   f  USART3's code 1 puts TX and RX on PC18/PC19, the debug port's pads:
//      init() answers false while the probe owns them, and touches nothing
//   h  half duplex on PB0 alone: the receiver does NOT hear the instance's
//      own frames (14.5's "the TX and RX pins connected internally" is the
//      receive path letter p proves with a second chip), measured
//   j  (y) the rendezvous - ping, IDENT (a CH32V006 peer on a crystal), a
//      bad checksum, a desync - and THE HSI AGAINST THE CRYSTAL: the
//      peer's span of a gapless run at 1 Mbaud (a divisor of 48, exact),
//      three times; then HSITRIM stepped a step at a time and each step
//      timed the same way, the trim back at its centre after
//   k  (y) the rates across the wire: 115200 to 3 Mbaud, echoed byte by
//      byte, then timed by the FOREIGN receiver against HSI / BRR; the
//      transport's bytes at 115200, 1 and 3 Mbaud
//   l  (y) the frame matrix across the wire: 8N1, 8E1, 8O1, 7E1, 7O1, 9N1,
//      8N2, and half and one-and-a-half stop bits - each echoed, and the
//      two odd stops timed as frames of 9.5 and 10.5 bits
//   m  (y) the receiver's errors from a crystal sender: PE, FE onset
//      against a uniform rate error (the chapter's "not less than 3 %",
//      14.3), glitches on the idle line, ORE from a run read late, a break
//   n  (y) the peer's engine shapes: a bit-banged frame, one stretched
//      cell, a LIN break ahead of a frame
//   o  (y) the modes on a wire: mute woken by an address mark and by an
//      idle line, IrDA between two WCH SIR codecs, LIN's break detection
//      at both lengths
//   p  (y) half duplex on PB0 hearing a frame the peer drives onto that
//      wire from its RX pad (drive_rxd)
//
// THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
// that drives a link pad from a sagging rail refuses every letter and
// says so.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>
#include <span>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#include "../../../avrdx/src/apps/usart_link.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;

using Loop = Uart<4, P>;                 // PB0 TX, PB1 RX
using Tx4 = Pin<'B', 0>;
using Rx4 = Pin<'B', 1>;
using U4 = Usart<4>;
using Debug3 = Uart<3, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 1>;

TwoKeyTestBench<Serial, 'y'> bench;

uint32_t cycles_now() { return stk()->CNTL; }
uint32_t cycles_between(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMPLR + 1u;
    return to >= from ? to - from : to + period - from;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);
}

// ---------------------------------------------------------------------------
// a - the divisor
// ---------------------------------------------------------------------------
void ta_divisor() {
    const uint16_t brr = Usart<2>::brr();
    print(serial, "  USART2 BRR=", brr, " (", brr >> 4, " + ", brr & 0xFu, "/16) -> ",
          Serial::actual_baud(SysClock::hz), " baud", crlf);
    bench.verdict("the console's divisor is HCLK / baud, rounded: 417 at 48 MHz", brr == 417u);
    bench.verdict("the rate it gives is within 0.1 per cent of 115200",
                  Serial::actual_baud(SysClock::hz) > 115085u && Serial::actual_baud(SysClock::hz) < 115315u);
    bench.verdict("3 Mbaud is reachable at 48 MHz (a divisor of exactly 16) and 3.2 is not",
                  Serial::can_baud(SysClock::hz, 3'000'000) && !Serial::can_baud(SysClock::hz, 3'200'000));
}

// ---------------------------------------------------------------------------
// b - the frame, register by register
// ---------------------------------------------------------------------------
void tb_frames() {
    U4::bus_clock(true);
    U4::reset();
    struct Case {
        UartFormat f;
        uint16_t ctlr1;
        uint16_t ctlr2;
        const char* name;
    };
    const Case cases[] = {
        {{UartBits::eight, UartParity::none, UartStop::one}, 0, 0, "8N1"},
        {{UartBits::eight, UartParity::even, UartStop::one}, usart_m | usart_pce, 0, "8E1"},
        {{UartBits::seven, UartParity::odd, UartStop::two}, static_cast<uint16_t>(usart_pce | usart_ps),
         static_cast<uint16_t>(2u << usart_stop_shift), "7O2"},
        {{UartBits::nine, UartParity::none, UartStop::one_and_half}, usart_m,
         static_cast<uint16_t>(3u << usart_stop_shift), "9N1.5"},
    };
    bool all = true;
    for (const Case& c : cases) {
        const bool ok = U4::configure(c.f, 417);
        const uint16_t c1 = U4::regs().CTLR1;
        const uint16_t c2 = U4::regs().CTLR2;
        print(serial, "  ", c.name, ": CTLR1=", hex(c1), " CTLR2=", hex(c2), crlf);
        all = ok && c1 == c.ctlr1 && c2 == c.ctlr2 && U4::brr() == 417u && all;
    }
    bench.verdict("every frame lands in CTLR1 and CTLR2 as the chapter spells it, and BRR too", all);
    bench.verdict("seven data bits with no parity bit are refused",
                  !U4::configure(UartFormat{UartBits::seven, UartParity::none, UartStop::one}, 417));
    bench.verdict("a divisor below 16 is refused", !U4::set_brr(15) && U4::brr() == 417u);
    U4::reset();
    U4::bus_clock(false);
}

// ---------------------------------------------------------------------------
// c - the modes and their exclusions
// ---------------------------------------------------------------------------
void tc_modes() {
    U4::bus_clock(true);
    U4::reset();
    (void)U4::configure(UartFormat{}, 417);

    const bool lin = U4::lin(LinConfig{true, false}) && U4::lin_enabled() &&
                     (U4::regs().CTLR2 & usart_lbdl) != 0u;
    const bool hd_refused_under_lin = !U4::half_duplex(true);
    const bool irda_refused_under_lin = !U4::irda(IrdaConfig{});
    U4::lin_off();
    bench.verdict("LIN takes its bits, and half duplex and IrDA are refused under it",
                  lin && hd_refused_under_lin && irda_refused_under_lin);

    const bool hd = U4::half_duplex(true) && U4::half_duplex();
    const bool lin_refused_under_hd = !U4::lin(LinConfig{});
    const bool sc_refused_under_hd = !U4::smartcard(SmartcardConfig{});
    (void)U4::half_duplex(false);
    bench.verdict("half duplex takes HDSEL, and LIN and the smartcard are refused under it",
                  hd && lin_refused_under_hd && sc_refused_under_hd);

    (void)U4::stop_bits(UartStop::two);
    const bool irda_refused_two_stops = !U4::irda(IrdaConfig{});
    (void)U4::stop_bits(UartStop::one);
    const bool irda = U4::irda(IrdaConfig{true, 7}) && U4::irda_enabled() && U4::prescaler() == 7u &&
                      (U4::regs().CTLR3 & usart_irlp) != 0u;
    const bool stops_refused_under_irda = !U4::stop_bits(UartStop::two);
    const bool normal_psc_refused = !irda_valid(IrdaConfig{false, 2});
    U4::irda_off();
    bench.verdict("IrDA refuses two stop bits, takes the low-power prescaler, and holds "
                  "the stop bits at one while it is on",
                  irda_refused_two_stops && irda && stops_refused_under_irda && normal_psc_refused);

    const bool sc = U4::smartcard(SmartcardConfig{true, 12, 5}) && U4::smartcard_enabled() &&
                    U4::guard_time() == 12u && U4::prescaler() == 5u &&
                    (U4::regs().CTLR2 & usart_stop_mask) == (3u << usart_stop_shift) &&
                    (U4::regs().CTLR2 & usart_clken) != 0u &&
                    (U4::regs().CTLR3 & usart_nack) != 0u;
    U4::smartcard_off();
    const bool clock_left_running = (U4::regs().CTLR2 & usart_clken) != 0u;
    const bool sync_off = U4::synchronous_off();
    bench.verdict("the smartcard takes its guard time, prescaler, 1.5 stops, CLKEN and NACK",
                  sc);
    bench.verdict("and smartcard_off() leaves the card clock to synchronous_off()",
                  clock_left_running && sync_off && !U4::synchronous_enabled());

    U4::transmitter(true);
    const bool sync_refused_under_te = !U4::synchronous(UsartSyncConfig{});
    U4::transmitter(false);
    const bool sync = U4::synchronous(UsartSyncConfig{true, true, true}) && U4::synchronous_enabled() &&
                      (U4::regs().CTLR2 & (usart_cpol | usart_cpha | usart_lbcl)) ==
                          (usart_cpol | usart_cpha | usart_lbcl);
    (void)U4::synchronous_off();
    bench.verdict("the synchronous clock is refused while TE is set, and takes CPOL, CPHA "
                  "and LBCL with the port disabled",
                  sync_refused_under_te && sync);

    (void)U4::flow_control(true, true);
    const bool flow = U4::rts_enabled() && U4::cts_enabled();
    (void)U4::flow_control(false, false);
    bench.verdict("the flow-control pair takes RTSE and CTSE", flow);

    const bool mute_cfg = U4::mute_mode(MuteConfig{MuteWake::address_mark, 9}) &&
                          (U4::regs().CTLR2 & usart_add_mask) == 9u &&
                          (U4::regs().CTLR1 & usart_wake) != 0u;
    const bool muted = U4::mute() && U4::muted();
    U4::unmute();
    bench.verdict("mute mode takes the address-mark wake and the address, and RWU",
                  mute_cfg && muted && !U4::muted());
    U4::reset();
    U4::bus_clock(false);
}

// ---------------------------------------------------------------------------
// d - a frame's length, and the rate on the wire
// ---------------------------------------------------------------------------

/// THE RATE ON THE WIRE, which neither a frame against its own divisor nor
/// a loop can tell - both ends of a loop share one divisor, and a frame
/// measured against BRR is the divisor measuring itself: `n` 8N1 frames
/// polled into `U`'s DATAR on TXE - the data register one frame ahead of
/// the shifter, so the line never idles - timed on the ticker's cycle
/// count from the first store to TC, against `n` x 10 bit times at the
/// rate ASKED, HCLK / `baud`; in thousandths. The port must be idle, its
/// transport's ring empty (the transmit interrupt disarmed).
template <typename U>
uint32_t wire_permille(uint32_t baud, uint32_t n) {
    const uint64_t wire = 10ull * n * SysClock::hz / baud;
    U::clear_flags(usart_tc);
    const uint32_t t0 = Ticker::cycles();
    for (uint32_t i = 0; i < n; ++i) {
        while (!U::tx_empty() && Ticker::cycles() - t0 < 4u * wire) {
        }
        U::write_data(static_cast<uint8_t>('0' + (i % 64u)));
    }
    while (!U::tx_complete() && Ticker::cycles() - t0 < 4u * wire) {
    }
    return static_cast<uint32_t>(1000ull * (Ticker::cycles() - t0) / wire);
}

bool on_the_wire(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

void td_frame_time() {
    console_drain();
    uint32_t took = 0;
    {
        P::CriticalSection cs;   // nothing of ours lands inside the frame
        Usart<2>::clear_flags(usart_tc);
        const uint32_t c0 = cycles_now();
        Usart<2>::write_data('U');
        uint32_t spin = 0;
        while (!Usart<2>::tx_complete() && spin < 100'000u) {
            ++spin;
        }
        took = cycles_between(c0, cycles_now());
    }
    const uint32_t expected = 10u * Usart<2>::brr();   // ten bits of BRR cycles each
    print(serial, crlf, "  one 8N1 frame ('U' above) took ", took, " HCLK cycles, ten bit times of "
          "the divisor are ", expected, crlf);
    bench.verdict("the frame lasts ten bit times of the divisor, within two bit times",
                  took + 2u * Usart<2>::brr() >= expected && took <= expected + 2u * Usart<2>::brr());

    // The runs: the console's 128 frames at the 115200 its init() asked
    // (BRR 417 for 416.7, 0.8 thousandths slow by the divisor's own
    // rounding), and USART4 on PB0 - a wire into the far board's floating
    // input - at three rates of letter k's, 1024 frames each: 115200, 1
    // Mbaud (48, exact) and 3 Mbaud (16, exact).
    console_drain();
    const uint32_t console_pm = wire_permille<Usart<2>>(115200, 128);
    print(serial, crlf, "  the console: 128 frames in ", console_pm, " thousandths of their wire time at 115200",
          crlf);
    bench.verdict("the console's line runs at the rate asked: 128 frames at 115200 within -1 % to +3 % of "
                  "their wire time", on_the_wire(console_pm));
    const bool opened = Loop::init(clock, 115200);
    // THE RX PAD PULLED UP: it is a wire to the far board's pad, which a
    // dark board leaves a floating input, and a floating receiver invents
    // frames whose interrupts take the core from the run (measured: the 3
    // Mbaud run at twice its wire time with the far board on its console).
    (void)Rx4::input(PinPull::up);
    (void)delay_us(clock, 200);   // TE's idle frame
    static constexpr uint32_t rates[] = {115200, 1'000'000, 3'000'000};
    uint32_t pm[3] = {};
    bool moved = opened;
    for (uint8_t k = 0; k < 3u; ++k) {
        moved = moved && Loop::set_baud(SysClock::hz, rates[k]);
        pm[k] = moved ? wire_permille<U4>(rates[k], 1024) : 0u;
    }
    Loop::release();
    print(serial, "  USART4 on PB0, 1024 frames: ", pm[0], " thousandths of their wire time at 115200, ", pm[1],
          " at 1 Mbaud, ", pm[2], " at 3 Mbaud", crlf);
    bench.verdict("and USART4's at each rate asked: 1024 frames at 115200, 1 Mbaud and 3 Mbaud, each within "
                  "-1 % to +3 % of its wire time",
                  moved && on_the_wire(pm[0]) && on_the_wire(pm[1]) && on_the_wire(pm[2]));
}

// ---------------------------------------------------------------------------
// e - the break
// ---------------------------------------------------------------------------
void te_break() {
    const bool opened = Loop::init(clock, 115200);
    (void)Rx4::input(PinPull::up);   // the wire to a dark board's floating pad (letter d)
    bench.verdict("USART4 opens on PB0/PB1", opened);
    (void)delay_us(clock, 200);   // TE's idle frame
    uint32_t took = 0;
    bool cleared = false;
    {
        P::CriticalSection cs;
        const uint32_t c0 = cycles_now();
        U4::send_break();
        uint32_t spin = 0;
        while (U4::break_pending() && spin < 100'000u) {
            ++spin;
        }
        took = cycles_between(c0, cycles_now());
        cleared = !U4::break_pending();
    }
    const uint32_t bit = U4::brr();
    print(serial, "  SBK cleared after ", took, " cycles = ", took / bit, " bit times", crlf);
    bench.verdict("SBK clears itself on the break frame's stop bit, within 20 bit times",
                  cleared && took <= 20u * bit);
    Loop::release();
}

// ---------------------------------------------------------------------------
// f - the column on the probe's pads
// ---------------------------------------------------------------------------
void tf_debug_column() {
    const bool gate_before = Rcc::enabled(Bus::pb1, rcc_pb1_usart3);
    const bool opened = Debug3::init(clock, 115200);
    print(serial, "  SW_CFG=", Afio::debug_config(), "; USART3 on PC18/PC19 init() answered ",
          opened, crlf);
    bench.verdict("USART3's code 1 is refused while the debug port is the probe's",
                  Afio::debug_port_enabled() && !opened);
    bench.verdict("and nothing was touched: the gate is as it was, the remap at its reset code",
                  Rcc::enabled(Bus::pb1, rcc_pb1_usart3) == gate_before &&
                      Afio::remap_code(Remap::usart3) == 0u);
}

// ---------------------------------------------------------------------------
// h - half duplex on PB0 alone: its own frames
// ---------------------------------------------------------------------------
/// USART4 open with the frame and rate, the port's reset line pulsed
/// first, the pads as the caller sets them after. The TX pad is parked as
/// a GPIO output high across the reset: a USART in reset drives its
/// alternate output low, an edge the far receiver takes for a start bit.
bool u4_open(const UartFormat& f, uint32_t rate) {
    Tx4::output(true);
    U4::bus_clock(true);
    U4::reset();
    if (!U4::configure(f, usart_divisor(SysClock::hz, rate))) {
        return false;
    }
    U4::enable(true);
    U4::transmitter(true);
    U4::receiver(true);
    return true;
}

void u4_flush() {
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)U4::status();
        (void)U4::read_word();
    }
}

void th_half_duplex_self() {
    // 14.5 asks the TX pad as "output mode plus pull"; this series' one
    // output stage is push-pull, so the pad is the function's, driven.
    const bool opened = u4_open(UartFormat{}, 115200) && U4::half_duplex(true);
    Tx4::function();
    (void)delay_us(clock, 200);
    u4_flush();
    static constexpr uint8_t bytes[4] = {0x55, 0xA3, 0x0F, 0xF0};
    uint8_t heard = 0;
    uint8_t same = 0;
    for (const uint8_t b : bytes) {
        U4::write_data(b);
        uint32_t spin = 0;
        while (!U4::tx_complete() && spin < 100'000u) {
            ++spin;
        }
        const uint16_t st = U4::status();
        if ((st & usart_rxne) != 0u) {
            ++heard;
            same = static_cast<uint8_t>(same + ((U4::read_word() & 0xFFu) == b ? 1u : 0u));
        }
    }
    (void)U4::half_duplex(false);
    U4::enable(false);
    U4::bus_clock(false);
    Tx4::release();
    print(serial, "  HDSEL on PB0: ", heard, " of 4 own frames heard, ", same, " of them equal", crlf);
    bench.verdict("half duplex opens on PB0", opened);
    // MEASURED: none. 14.5's "the TX and RX pins connected internally"
    // is the single wire's receive path, which hears another node
    // (letter p) and not the instance's own frames - the CH32V006's
    // answer too (its usart.hpp) - so this die has no jumperless loop.
    bench.verdict("its receiver does not hear its own frames: no jumperless self-loop on this die",
                  heard == 0u);
}

// ===========================================================================
// THE TWO-BOARD SET: the link to the CH32V006's usart_peer
// ===========================================================================

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool leg_refused = false;
bool link_quiet = false;

/// A wait of whole milliseconds on the tick count: delay_us serves less
/// than one tick period and refuses the rest (delay.hpp).
void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms + 1u) {
    }
}

/// Exactly `us` microseconds on the cycle count: what the rendezvous
/// needs, where wait_ms()'s tick granularity would add up to one.
void wait_us(uint32_t us) {
    const uint32_t c0 = Ticker::cycles();
    const uint32_t n = us * (SysClock::hz / 1'000'000u);
    while (Ticker::cycles() - c0 < n) {
    }
}

void put4(uint8_t b) {
    while (!U4::tx_empty()) {
    }
    U4::write_data(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::ticks();
    while (!U4::tx_complete() && Ticker::ticks() - t0 < 50u) {
    }
}

/// The DUT's end in a commanded configuration: the link pads the
/// function's TX and a pulled-up RX (a peer between answers drives
/// nothing until its wiring is proven, and a floating RX invents bytes).
bool dut_link(const link::Cfg& c) {
    UartFormat f{};
    switch (c.bits) {
        case 7: f.bits = UartBits::seven; break;
        case link::bits9_low:
        case link::bits9_high: f.bits = UartBits::nine; break;
        default: f.bits = UartBits::eight; break;
    }
    f.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    f.stop = c.stop == 2u ? UartStop::two
           : c.stop == link::stop_half ? UartStop::half
           : c.stop == link::stop_one_half ? UartStop::one_and_half : UartStop::one;
    if (!u4_open(f, c.rate)) {
        return false;
    }
    if (c.mode == link::Mode::ircom && !U4::irda(IrdaConfig{})) {
        return false;
    }
    // TE's idle frame first: a pad handed to the function the moment TE
    // rises shows the far receiver a low pulse (measured on the far
    // chip's USART, the same block's lineage), so two bit times and 20 us
    // pass before it.
    (void)delay_us(clock, 2u * (usart_divisor(SysClock::hz, c.rate) / 48u) + 20u);
    Tx4::function();
    (void)Rx4::input(PinPull::up);
    u4_flush();
    return true;
}

bool link_command_mode() {
    dec.reset();
    return dut_link(link::Cfg{});
}

bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::ticks();
    dec.reset();
    while (Ticker::ticks() - t0 < ms) {
        const uint16_t st = U4::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        const uint8_t b = U4::read_data();
        if ((st & (usart_fe | usart_pe | usart_ne)) != 0u) {
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

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    u4_flush();
    link::write_frame(put4, op, p, len);
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

/// Find the peer: full duplex is the desk's only wiring, so this pings
/// until the peer's dark search hears a frame that checks out.
bool ensure_link() {
    link_quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = link_command_mode() && command(link::Op::ping);
    }
    link_quiet = false;
    if (!found) {
        print(serial, "  the peer did not answer: the CH32V006 runs usart_peer on its column 4?", crlf);
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

/// The report of the last action, after its bound has had time to run.
/// A refusal raises leg_refused and says so once.
bool peer_report(link::Report& r, uint16_t after_ms) {
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

/// Command an action; the peer starts its own traffic link::settle_ms +
/// 2 ms after its ack, so the DUT has that window to take up its end.
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
    // settle_ms EXACTLY: the peer starts its own traffic settle_ms + 2 ms
    // after its ack in its ticks, as early as 4 ms, and this end's
    // reconfiguration must be done by then (a frame lost to the race was
    // measured with a tick-granular wait here).
    wait_us(static_cast<uint32_t>(link::settle_ms) * 1000u);
    return true;
}

/// A verdict, unless the peer refused the leg: then SKIP, counting nothing.
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

/// Milliseconds a frame count needs at a rate, four times over plus a floor.
uint16_t bound_ms(uint32_t rate, uint32_t frames) {
    const uint32_t ms = (12u * frames * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
    return ms > 4000u ? 4000u : static_cast<uint16_t>(ms);
}

/// The peer opens its window link::settle_ms + 2 ms after its ack and
/// flushes what the DUT's reconfiguration left on the line.
void wait_for_peer_window() { wait_ms(6); }

/// `n` frames back to back: DATAR written on TXE, a frame ahead of the
/// shifter, so the line never idles.
void dut_blast(uint16_t n, uint16_t mask) {
    for (uint16_t i = 0; i < n; ++i) {
        while (!U4::tx_empty()) {
        }
        U4::write_word(static_cast<uint16_t>((0x55u + i) & mask));
    }
    wait_tc();
}

/// What the DUT's receiver made of a run, polled: frames, the errors, the
/// last data word.
struct Drain {
    uint16_t frames = 0;
    uint8_t fe = 0, pe = 0, ne = 0, ore = 0;
    uint16_t last = 0;
    uint16_t first = 0;
    bool lbd = false;   ///< LIN's break flag, read before the link goes back to command mode
};

Drain collect(uint16_t ms) {
    Drain d{};
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        const uint16_t st = U4::status();
        if ((st & usart_rxne) == 0u) {
            continue;
        }
        const uint16_t w = U4::read_word();
        if (d.frames == 0u) d.first = w;
        d.last = w;
        ++d.frames;
        if ((st & usart_fe) != 0u) ++d.fe;
        if ((st & usart_pe) != 0u) ++d.pe;
        if ((st & usart_ne) != 0u) ++d.ne;
        if ((st & usart_ore) != 0u) ++d.ore;
    }
    return d;
}

/// One calibrate leg: the peer counts and times `n` gapless frames of the
/// DUT at `c`; the answer is the DUT's rate against the peer's crystal.
struct Calib {
    bool ok = false;
    int16_t ppm = 0;
    uint32_t span = 0;
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
    if (!k.ok) {
        print(serial, "    calibrate at ", c.rate, ": the peer counted ", r.count, " of ", n, ", ferr ", r.ferr,
              " ovf ", r.ovf, " span ", r.span, " flags ", hex(r.flags), crlf);
    }
    k.ppm = static_cast<int16_t>(r.aux16);
    k.span = r.span;
    return k;
}

/// ppm printed with its sign and as per cent.
void print_ppm(int32_t ppm) {
    const uint32_t m = static_cast<uint32_t>(ppm < 0 ? -ppm : ppm);
    print(serial, ppm < 0 ? "-" : "+", m, " ppm (", ppm < 0 ? "-" : "+", m / 10000u, ".",
          (m / 1000u) % 10u, (m / 100u) % 10u, " %)");
}

/// The rate a divisor gives against the rate asked, in ppm.
int32_t divisor_ppm(uint32_t rate) {
    const uint32_t brr = usart_divisor(SysClock::hz, rate);
    return static_cast<int32_t>((static_cast<int64_t>(SysClock::hz) * 1'000'000 / brr) / rate) - 1'000'000;
}

bool near(int32_t a, int32_t b, int32_t tol) { return a - b <= tol && b - a <= tol; }

/// Decimal for a verdict label (one call per expression).
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

constexpr uint32_t calib_rate = 1'000'000;   ///< a divisor of 48: the HSI's error alone
constexpr uint16_t calib_frames = 4000;      ///< 40 ms, about a million ticks of span
int32_t hsi_ppm = 0;                         ///< letter j's figure, used by the later letters
bool hsi_known = false;

/// The HSI's error, measured once if letter j has not.
int32_t hsi_error() {
    if (!hsi_known) {
        link::Cfg c{};
        c.rate = calib_rate;
        const Calib k = calibrate(c, calib_frames);
        if (k.ok) {
            hsi_ppm = k.ppm;
            hsi_known = true;
        }
    }
    return hsi_ppm;
}

// ---------------------------------------------------------------------------
// j - the rendezvous, and the HSI against the crystal
// ---------------------------------------------------------------------------
void tj_link() {
    const bool found = ensure_link();
    bench.verdict("command mode on USART4 (8N1, 115200) and the peer answers a ping", found);
    if (!found) {
        return;
    }
    link::Ident d{};
    const bool got = peer_ident(d);
    bench.verdict("IDENT answered", got);
    if (got) {
        char label[9] = {};
        for (uint8_t i = 0; i < 8u; ++i) label[i] = d.label[i];
        print(serial, "  peer '", label, "' kind ", hex(link::peer_kind(d.version)), " fw ", hex(d.version),
              " xtal=", d.xtal, " sanity=", hex(d.sanity), crlf);
    }
    bench.verdict("the peer is the CH32V006's usart_peer", got && d.sanity == link::ident_sanity &&
                                                                link::peer_kind(d.version) == link::peer_ch32v00x);
    bench.verdict("its time base is crystal-referenced", got && d.xtal == 1u);

    u4_flush();
    put4(link::magic);
    put4(link::byte_of(link::Op::ping));
    put4(0);
    put4(0x00);   // not the checksum
    wait_tc();
    link::Frame bad;
    bench.verdict("a bad checksum comes back as a NAK", recv_frame(bad, 120) && bad.op == link::Op::nak);
    for (uint8_t i = 0; i < 12u; ++i) put4(static_cast<uint8_t>(0x11u * i));
    wait_tc();
    wait_ms(120);
    (void)link_command_mode();
    bench.verdict("the link recovers after a desync", command(link::Op::ping));

    // THE REFUSAL PATH, which a leg the peer cannot serve takes: onewire
    // (no USART TX of the CH32V006 reaches its own RX pad) and the
    // synchronous client (no synchronous mode on that part) are acked,
    // reported refused, and the peer is back in command mode at once.
    bool refused_both = true;
    for (const link::Op op : {link::Op::onewire, link::Op::set_link}) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.mode = op == link::Op::set_link ? link::Mode::sync_client : link::Mode::async;
        a.ms = 50;
        link::Report r{};
        refused_both = refused_both && peer_act(op, a) && peer_report(r, 0) && leg_refused &&
                       (r.flags & link::report_ran) == 0u;
    }
    leg_refused = false;
    bench.verdict("the peer refuses onewire and a synchronous client, and answers at once after", refused_both &&
                                                                                                  command(link::Op::ping));

    // THE HSI, three times at its factory calibration.
    link::Cfg c{};
    c.rate = calib_rate;
    Calib k[3];
    bool all = true;
    for (Calib& x : k) {
        x = calibrate(c, calib_frames);
        all = all && x.ok;
    }
    print(serial, "  the HSI at trim ", Rcc::hsi_trim(), " (HSICAL ", hex(Rcc::hsi_calibration()), "): ");
    print_ppm(k[0].ppm);
    print(serial, ", then ", k[1].ppm, " and ", k[2].ppm, " ppm; span ", k[0].span, " ticks for ",
          calib_frames - 1u, " frames", crlf);
    // The record for the document: the HSI's trim with the smallest error.
    bench.verdict("the peer timed three gapless runs of 4000 frames at 1 Mbaud", all);
    // The instrument resolves a trim step many times over (its stamps
    // jitter by a few ticks in a million); the HSI itself wanders between
    // runs a second apart, which is the oscillator's and the figure printed.
    bench.verdict("the three runs agree within one trim step (0.23 %), the HSI's wander printed",
                  all && near(k[0].ppm, k[1].ppm, 2300) && near(k[1].ppm, k[2].ppm, 2300) &&
                      near(k[0].ppm, k[2].ppm, 2300));
    bench.verdict("the HSI lies inside table 3-9's -1.7 % to +1.6 % (0 to 70 C)",
                  all && k[0].ppm >= -17'000 && k[0].ppm <= 16'000);
    if (all) {
        hsi_ppm = k[0].ppm;
        hsi_known = true;
    }

    if (!all) {
        bench.verdict("the trim walked, the HSI measured first", false);
        (void)link_command_mode();
        return;
    }
    // THE TRIM, a step at a time - kept where the command channel's own
    // 115200 stays inside both receivers' tolerance (2.5 % each way at
    // the manual's 110 kHz a step, 0.23 %).
    const uint8_t centre = Rcc::hsi_trim();
    int32_t ppm[32] = {};
    bool timed[32] = {};
    const int32_t step_guess = 2300;
    uint8_t lo = centre, hi = centre;
    while (lo > 0u && near(hsi_ppm + (static_cast<int32_t>(lo) - 1 - centre) * step_guess, 0, 25'000)) --lo;
    while (hi < 31u && near(hsi_ppm + (static_cast<int32_t>(hi) + 1 - centre) * step_guess, 0, 25'000)) ++hi;
    for (uint8_t t = lo; t <= hi; ++t) {
        Rcc::hsi_trim(t);
        wait_ms(1);
        const Calib x = calibrate(c, calib_frames);
        timed[t] = x.ok;
        ppm[t] = x.ppm;
    }
    Rcc::hsi_trim(centre);
    wait_ms(1);
    const Calib back = calibrate(c, calib_frames);
    print(serial, "  trim  ppm      Hz off 48 MHz   step Hz", crlf);
    bool monotonic = true;
    bool every = true;
    int32_t steps = 0;
    for (uint8_t t = lo; t <= hi; ++t) {
        every = every && timed[t];
        const int32_t hz = ppm[t] * 48;
        print(serial, "  ", t, "    ", ppm[t], "    ", hz);
        if (t > lo) {
            const int32_t step = (ppm[t] - ppm[t - 1u]) * 48;
            print(serial, "    ", step);
            monotonic = monotonic && step > 0;
        }
        print(serial, crlf);
    }
    if (hi > lo) {
        steps = ((ppm[hi] - ppm[lo]) * 48) / static_cast<int32_t>(hi - lo);
    }
    print(serial, "  trims ", lo, "..", hi, ": the mean step is ", steps, " Hz; back at ", centre, ": ",
          back.ppm, " ppm", crlf);
    bench.verdict("every trim step was timed", every && hi > lo);
    bench.verdict("each step moves the HSI up", every && monotonic);
    bench.verdict("the mean step lies within a factor of two of the manual's about 110 kHz",
                  every && steps >= 55'000 && steps <= 220'000);
    bench.verdict("back at the centre the HSI reads what it read before, within one step",
                  back.ok && near(back.ppm, hsi_ppm, 2300));
    (void)link_command_mode();
}

// ---------------------------------------------------------------------------
// k - the rates across the wire
// ---------------------------------------------------------------------------
uint16_t echo_mismatches(uint16_t n, uint16_t mask) {
    uint16_t bad = 0;
    wait_for_peer_window();
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>((0x5Au + i * 37u) & mask);
        while (!U4::tx_empty()) {
        }
        U4::write_word(v);
        bool ok = false;
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < 20u) {
            const uint16_t st = U4::status();
            if ((st & usart_rxne) != 0u) {
                ok = (U4::read_word() & mask) == v && (st & (usart_fe | usart_pe | usart_ne)) == 0u;
                break;
            }
        }
        if (!ok) ++bad;
    }
    return bad;
}

void tk_rates() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return;
    }
    const int32_t hsi = hsi_error();
    print(serial, "  the HSI: ");
    print_ppm(hsi);
    print(serial, crlf);
    static constexpr uint32_t rates[] = {115'200, 921'600, 1'000'000, 1'500'000, 2'000'000, 2'400'000, 3'000'000};
    for (const uint32_t rate : rates) {
        link::Cfg c{};
        c.rate = rate;
        link::Params a{};
        a.cfg = c;
        a.count = 16;
        a.ms = bound_ms(rate, 32);
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(16, 0xFFu) : 16u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        judge(num(rate), " baud: 16 bytes echoed, every one equal", up && bad == 0u);
        judge(num(rate), " baud: the peer counted 16 clean frames",
              have && r.count == 16u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u);
        // The reference first, back to back: the HSI wanders by a few
        // hundred ppm over seconds (letter j), so each rate is judged
        // against the 1 Mbaud figure taken just before it. Half the
        // smallest divisor step anywhere on this ladder (1/417, 2400 ppm)
        // is the bound: a wrong divisor cannot hide inside it.
        link::Cfg ref{};
        ref.rate = calib_rate;
        const Calib r0 = calibrate(ref, 2000);
        const Calib k = calibrate(c, 2000);
        const int32_t expect = r0.ppm + divisor_ppm(rate);
        print(serial, "  ", rate, ": BRR ", usart_divisor(SysClock::hz, rate), ", the foreign receiver times ");
        print_ppm(k.ppm);
        print(serial, " against HSI / BRR's ", expect, " ppm", crlf);
        judge(num(rate), " baud: the rate on the wire is HSI / BRR within 500 ppm, timed by the peer",
              r0.ok && k.ok && near(k.ppm, expect, 500));
    }

    // The transport's bytes, the old jumper letter's, now through the far
    // chip: an echo, full duplex, at 115200, 1 and 3 Mbaud - and at 3
    // Mbaud each direction alone as well, because the echo there is the
    // transport's measured ceiling and not its contract: the far chip's
    // echo lags its frame, so the vector is entered twice a frame, once
    // for TXE and once for RXNE, some 100 cycles each against a frame of
    // 160 (docs/ch32x035/usart.md).
    static constexpr uint32_t transport_rates[] = {115'200, 1'000'000, 3'000'000};
    static const uint8_t pattern[8] = {0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7E, 0x81};
    for (const uint32_t rate : transport_rates) {
        link::Cfg c{};
        c.rate = rate;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(rate, 16);
        const bool acted = peer_act(link::Op::echo, a);
        bool same = acted && Loop::init(clock, rate);
        (void)Rx4::input(PinPull::up);
        wait_for_peer_window();
        while (Loop::rx_pending() != 0u) {
            uint8_t discard = 0;
            (void)Loop::read_byte(discard);
        }
        Loop::clear_errors();
        (void)Loop::write_bulk(pattern);
        const uint32_t t0 = Ticker::ticks();
        while (Loop::rx_pending() < 8u && Ticker::ticks() - t0 < 30u) {
        }
        uint8_t got = 0;
        for (uint8_t i = 0; i < 8u; ++i) {
            uint8_t b = 0;
            if (Loop::read_byte(b) && b == pattern[i]) ++got;
        }
        same = same && got == 8u;
        const uint16_t ore = Loop::hw_overruns();
        Loop::release();
        link::Report r{};
        (void)peer_report(r, 5);
        print(serial, "  the transport's echo at ", rate, ": ", got, " of 8 back equal, ORE ", ore,
              "; the far chip received ", r.count, crlf);
        if (rate != 3'000'000u) {
            judge(num(rate), " baud: eight bytes through the transport come back through the far chip", same);
        }
    }
    // 3 Mbaud the other way alone: the far chip sinks a run the
    // transport sends.
    {
        link::Cfg c{};
        c.rate = 3'000'000;
        link::Params a{};
        a.cfg = c;
        a.count = 64;
        a.ms = bound_ms(c.rate, 128);
        const bool acted = peer_act(link::Op::sink, a);
        bool up = acted && Loop::init(clock, c.rate);
        (void)Rx4::input(PinPull::up);
        wait_for_peer_window();
        uint8_t run[64];
        uint16_t sum = 0;
        for (uint8_t i = 0; i < 64u; ++i) {
            run[i] = static_cast<uint8_t>(0x30u + i);
            sum = static_cast<uint16_t>(sum + run[i]);
        }
        (void)Loop::write_bulk(run);
        const uint32_t t0 = Ticker::ticks();
        while (!Loop::tx_idle() && Ticker::ticks() - t0 < 30u) {
        }
        Loop::release();
        link::Report r{};
        const bool have = peer_report(r, 5);
        judge("3000000 baud: 64 bytes the transport sends, all received by the far chip",
              up && have && r.count == 64u && r.sum == sum && r.ferr == 0u && r.ovf == 0u);
    }
    // Gapless runs INTO the transport: one vector entry a frame, the
    // hardware prologue's round trip and the ring's push against the
    // frame's 160 cycles at 3 Mbaud - the sustained ceiling, swept.
    static constexpr uint32_t sweep[] = {1'000'000, 1'500'000, 2'000'000, 2'400'000, 3'000'000};
    uint32_t ceiling = 0;
    bool clean_so_far = true;
    for (const uint32_t rate : sweep) {
        link::Cfg c{};
        c.rate = rate;
        link::Params a{};
        a.cfg = c;
        a.count = 256;
        a.ms = bound_ms(rate, 512);
        a.value = 0x30;
        a.pattern = link::pattern_counting;
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && Loop::init(clock, rate);
        (void)Rx4::input(PinPull::up);
        Loop::clear_errors();
        // The consumer takes RUNS (read_span, consume): one look a run, so
        // the main loop's share stays a few cycles a byte beside the
        // vector's one entry a frame.
        uint16_t got = 0;
        uint8_t wrong = 0;
        bool broken = false;
        const uint32_t t1 = Ticker::ticks();
        while (got < 256u && !broken && Ticker::ticks() - t1 < 40u) {
            const std::span<const uint8_t> run = Loop::read_span();
            uint32_t n = 0;
            for (const uint8_t b : run) {
                if (b != static_cast<uint8_t>(0x30u + got)) {
                    wrong = b;
                    broken = true;
                    break;
                }
                ++got;
                ++n;
            }
            Loop::consume(n);
        }
        const uint16_t ore = Loop::hw_overruns();
        if (got != 256u) {
            print(serial, "    at ", rate, ": byte ", got, " read ", hex(wrong), " for ", hex(static_cast<uint8_t>(0x30u + got)),
                  "; fe ", Loop::frame_errors(), " ne ", Loop::noise_errors(), " pe ", Loop::parity_errors(),
                  " ring full ", Loop::rx_overruns(), " skips ", Loop::rx_skips(), crlf);
        }
        Loop::release();
        link::Report r{};
        (void)peer_report(r, 5);
        print(serial, "  a gapless run of 256 into the transport at ", rate, ": ", got, " in order, ORE ", ore, crlf);
        clean_so_far = clean_so_far && up && got == 256u;
        if (clean_so_far) {
            ceiling = rate;
        }
        if (rate == 2'000'000u) {
            judge("2000000 baud: a gapless run of 256 from the far chip, every byte received in order",
                  up && got == 256u);
        }
    }
    print(serial, "  the transport's sustained receive: clean up to ", ceiling, " baud", crlf);
    (void)link_command_mode();
}

// ---------------------------------------------------------------------------
// l - the frame matrix across the wire
// ---------------------------------------------------------------------------
struct FmtCase {
    uint8_t bits;
    uint8_t parity;
    uint8_t stop;
    const char* name;
};

void tl_frames() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return;
    }
    static constexpr FmtCase cases[] = {
        {8, 0, 1, "8N1"}, {8, 1, 1, "8E1"}, {8, 2, 1, "8O1"}, {7, 1, 1, "7E1"}, {7, 2, 1, "7O1"},
        {link::bits9_low, 0, 1, "9N1"}, {8, 0, 2, "8N2"}, {8, 0, link::stop_half, "8N0.5"},
        {8, 0, link::stop_one_half, "8N1.5"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
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
        if (bad != 0u) {
            print(serial, "    ", fc.name, ": wrong=", bad, " peer count=", r.count, " ferr=", r.ferr,
                  " perr=", r.perr, crlf);
        }
        judge(fc.name, " across the wire, echoed",
              up && bad == 0u && have && r.count == 8u && r.ferr == 0u && r.perr == 0u);
    }
    // The two odd stops as LENGTHS: a gapless run timed by the peer is a
    // frame of 19 and of 21 half bits, the HSI's error and nothing else.
    for (const uint8_t stop : {link::stop_half, link::stop_one_half}) {
        // The 8N1 reference back to back (the HSI wanders, letter j); a
        // frame a half bit off would be 5 % off, the bound 500 ppm.
        link::Cfg ref{};
        ref.rate = calib_rate;
        const Calib r0 = calibrate(ref, 2000);
        const int32_t hsi = r0.ppm;
        link::Cfg c{};
        c.rate = calib_rate;
        c.stop = stop;
        const Calib k = calibrate(c, 2000);
        print(serial, "  ", stop == link::stop_half ? "8N0.5" : "8N1.5", " at 1 Mbaud: ");
        print_ppm(k.ppm);
        print(serial, " against frames of ", link::frame_half_bits(8, 0, stop), " half bits", crlf);
        judge(stop == link::stop_half ? "half a stop bit" : "one and a half stop bits",
              ": the frame on the wire is that long, within 500 ppm of an 8N1 run's",
              r0.ok && k.ok && near(k.ppm, hsi, 500));
    }
    (void)link_command_mode();
}

// ---------------------------------------------------------------------------
// m - the receiver's errors, from a crystal sender
// ---------------------------------------------------------------------------
void tm_errors() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return;
    }
    const int32_t hsi = hsi_error();
    // 1. Odd parity into an even receiver.
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
        const bool acted = peer_act(link::Op::send, a);
        const bool up = acted && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 12));
        print(serial, "  odd into even: frames=", d.frames, " pe=", d.pe, " fe=", d.fe, crlf);
        judge("every frame of the wrong parity raises PE", up && d.frames == 6u && d.pe == 6u && d.fe == 0u);
        (void)link_command_mode();
    }
    // 2. A uniform rate error from the peer's engine: 0x00 frames at 9600
    //    off by e per cent, into a receiver at 9600 whose own clock is off
    //    by the HSI's error.
    {
        static constexpr int8_t errs[] = {0, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 8, -8};
        int8_t onset_pos = 0, onset_neg = 0;
        bool inside_clean = true;
        link::Cfg dc{};
        dc.rate = 9600;
        for (const int8_t e : errs) {
            link::Params a{};
            a.cfg.rate = static_cast<uint32_t>(9600 + (9600 * e) / 100);
            a.count = 6;
            a.ms = 400;
            a.value = 0x00;
            const bool acted = peer_act(link::Op::bitbang, a);
            const bool up = acted && dut_link(dc);
            const Drain d = collect(60);
            link::Report r{};
            (void)peer_report(r, 0);
            const int32_t effective = static_cast<int32_t>(e) * 10'000 - hsi;
            print(serial, "  ", e, " % (", effective / 100, " per ten thousand against this receiver): frames=",
                  d.frames, " fe=", d.fe, "; the peer banged ", r.count, crlf);
            if (up && (d.fe != 0u || d.frames != 6u)) {
                if (e > 0 && onset_pos == 0) onset_pos = e;
                if (e < 0 && onset_neg == 0) onset_neg = e;
            }
            if (near(effective, 0, 30'000)) {
                inside_clean = inside_clean && up && d.frames == 6u && d.fe == 0u;
            }
            (void)link_command_mode();
        }
        print(serial, "  FE onset: +", onset_pos, " %, ", onset_neg, " %", crlf);
        judge("every rate within 3 % of the receiver's own is clean (14.3)", inside_clean);
        judge("and a rate far enough off does raise FE", onset_pos != 0 || onset_neg != 0);
    }
    // 3. Low pulses on the idle line: where a start bit starts being believed.
    {
        static constexpr uint16_t widths[] = {6, 60, 300, 625, 1250, 1875, 2500};
        uint16_t first_taken = 0;
        link::Cfg dc{};
        dc.rate = 9600;
        bool tiny_ignored = false, whole_taken = false;
        for (const uint16_t w : widths) {
            link::Params a{};
            a.cfg.rate = 9600;
            a.count = 4;
            a.ms = 300;
            a.aux16 = w;
            a.aux8 = 200;
            const bool acted = peer_act(link::Op::glitch, a);
            const bool up = acted && dut_link(dc);
            const Drain d = collect(60);
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  a low pulse of ", w, " ticks (a bit is 2500): frames=", d.frames, " fe=", d.fe,
                  " ne=", d.ne, crlf);
            if (up && d.frames != 0u && first_taken == 0u) first_taken = w;
            if (w == 6u) tiny_ignored = up && d.frames == 0u;
            if (w == 2500u) whole_taken = up && d.frames != 0u;
            (void)link_command_mode();
        }
        print(serial, "  the narrowest pulse taken as a start bit: ", first_taken, " ticks", crlf);
        judge("a pulse of 6 ticks is not a start bit", tiny_ignored);
        judge("a whole bit low is one", whole_taken);
    }
    // 4. A run into a receiver that reads late: ORE.
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
        wait_ms(6 + 2);   // the window, and the run, unread
        const uint16_t st = U4::status();
        const uint16_t kept = U4::read_word();
        const Drain d = collect(5);
        print(serial, "  a run of 8 read late: STATR ", hex(st), ", DATAR ", hex(kept), ", then ", d.frames,
              " more", crlf);
        judge("a run read late raises ORE", up && (st & usart_ore) != 0u && (st & usart_rxne) != 0u);
        judge("and DATAR keeps the first frame of the run, the rest lost",
              up && kept == 0xB0u && d.frames == 0u);
        (void)link_command_mode();
    }
    // 5. A break: FE with data zero.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 20;
        a.ms = 200;
        const bool acted = peer_act(link::Op::brk, a);
        const bool up = acted && dut_link(pc);
        const Drain d = collect(30);
        print(serial, "  a break of 20 bit times: frames=", d.frames, " fe=", d.fe, " data ", hex(d.first), crlf);
        judge("a break arrives as one frame with FE and data 0x00",
              up && d.frames == 1u && d.fe == 1u && d.first == 0u);
        (void)link_command_mode();
    }
    (void)link_command_mode();
}

// ---------------------------------------------------------------------------
// n - the peer's engine shapes
// ---------------------------------------------------------------------------
Drain bitbang_run(uint8_t value, uint16_t frames, int8_t cell, int8_t pct, uint16_t break_bits,
                  const link::Cfg& dut_cfg) {
    link::Params a{};
    a.cfg.rate = 9600;
    a.count = frames;
    a.ms = 600;
    a.value = value;
    a.cell = cell;
    a.pct = pct;
    a.aux16 = break_bits;
    if (!peer_act(link::Op::bitbang, a)) return Drain{};
    if (!dut_link(dut_cfg)) return Drain{};
    if (dut_cfg.mode == link::Mode::async && break_bits != 0u) {
        U4::enable(false);
        (void)U4::lin(LinConfig{});
        U4::enable(true);
        U4::clear_flags(usart_lbd);
    }
    Drain d = collect(static_cast<uint16_t>(30u + 15u * frames + break_bits / 4u));
    d.lbd = U4::flag(usart_lbd);
    link::Report r{};
    (void)peer_report(r, 0);
    return d;
}

void tn_shapes() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return;
    }
    link::Cfg base{};
    base.rate = 9600;
    const Drain clean = bitbang_run(0x55, 6, -1, 0, 0, base);
    print(serial, "  six banged 0x55 at 9600: frames=", clean.frames, " fe=", clean.fe, " last ", hex(clean.last), crlf);
    judge("a frame from the peer's edge list is a clean 0x55, six times",
          clean.frames == 6u && clean.fe == 0u && clean.ne == 0u && clean.last == 0x55u);
    (void)link_command_mode();

    int8_t onset = 0;
    for (const int8_t p : {10, 20, 30, 40, 50, 60, 70}) {
        const Drain d = bitbang_run(0x00, 4, 5, p, 0, base);
        print(serial, "  cell 5 stretched +", p, " %: frames=", d.frames, " fe=", d.fe, crlf);
        if ((d.fe != 0u || d.frames != 4u) && onset == 0) onset = p;
        (void)link_command_mode();
    }
    print(serial, "  one stretched cell breaks the frame from +", onset, " %", crlf);
    judge("one stretched cell survives far past the rate tolerance (30 % and more)", onset == 0 || onset >= 30);

    const Drain lin = bitbang_run(0x55, 1, -1, 0, 13, base);
    const bool lbd = lin.lbd;
    print(serial, "  a 13-bit break then 0x55: LBD ", lbd, ", frames=", lin.frames, " last ", hex(lin.last), crlf);
    judge("a LIN break ahead of a frame: the break detected and the frame after it received",
          lbd && lin.last == 0x55u);
    (void)link_command_mode();
}

// ---------------------------------------------------------------------------
// o - the modes on a wire
// ---------------------------------------------------------------------------
void to_modes() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return;
    }
    // 1. Mute woken by an address mark: 9-bit words, this node 5.
    {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = link::bits9_low;
        link::Params a{};
        a.cfg = c;
        a.value = 0x10;
        a.groups = 3;
        const uint8_t pairs[6] = {3, 2, 5, 3, 3, 2};
        const bool acted = peer_act(link::Op::send_mpcm, a, pairs, 6);
        bool up = acted && dut_link(c);
        up = up && U4::mute_mode(MuteConfig{MuteWake::address_mark, 5}) && U4::mute();
        const Drain d = collect(40);
        print(serial, "  address mark: frames=", d.frames, " first ", hex(d.first), " last ", hex(d.last), crlf);
        link::Report r{};
        (void)peer_report(r, 0);
        judge("a muted receiver wakes on its address alone: the address frame and its three data",
              up && d.frames == 4u && d.first == 0x105u && d.last == 0x14u);
        U4::unmute();
        (void)link_command_mode();
    }
    // 2. Mute woken by an idle line: three runs of four, 3 ms apart; the
    //    receiver muted after the first frame wakes on the idle after the
    //    first run.
    {
        link::Cfg c{};
        c.rate = 9600;
        link::Params a{};
        a.cfg = c;
        a.count = 4;
        a.aux16 = 3;
        a.aux8 = 30;
        a.ms = 400;
        a.pattern = link::pattern_counting;
        a.value = 0;
        const bool acted = peer_act(link::Op::blast, a);
        bool up = acted && dut_link(c) && U4::mute_mode(MuteConfig{MuteWake::idle_line, 0});
        uint16_t seen[12] = {};
        uint8_t n = 0;
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < 60u) {
            if (!U4::rx_ready()) continue;
            const uint16_t w = U4::read_word();
            if (n < 12u) seen[n++] = w;
            if (n == 1u) up = up && U4::mute();
        }
        print(serial, "  idle line: ", n, " frames:");
        for (uint8_t i = 0; i < n; ++i) print(serial, " ", seen[i]);
        print(serial, crlf);
        bool expected = n == 9u && seen[0] == 0u;
        for (uint8_t i = 1; expected && i < 9u; ++i) expected = seen[i] == static_cast<uint16_t>(i + 3u);
        link::Report r{};
        (void)peer_report(r, 0);
        judge("muted after the first frame, the receiver drops the rest of the run and wakes on the idle line",
              up && expected);
        U4::unmute();
        (void)link_command_mode();
    }
    // 3. IrDA, normal mode, between two WCH SIR codecs.
    {
        link::Cfg c{};
        c.rate = 115'200;
        c.mode = link::Mode::ircom;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(115'200, 24);
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(8, 0xFFu) : 8u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        print(serial, "  IrDA at 115200: wrong=", bad, " peer count=", r.count, " ferr=", r.ferr, crlf);
        judge("IrDA's SIR frames echoed by a second WCH codec, every one equal",
              up && bad == 0u && have && r.count == 8u);
        U4::irda_off();
        (void)link_command_mode();
    }
    // 4. LIN's break detection at both lengths.
    {
        struct BreakCase {
            uint16_t bits;
            bool eleven;
            bool detect;
        };
        static constexpr BreakCase cases[] = {{9, false, false}, {13, false, true}, {10, true, false}, {13, true, true}};
        bool all = true;
        for (const BreakCase& bc : cases) {
            link::Cfg c{};
            c.rate = 9600;
            link::Params a{};
            a.cfg = c;
            a.count = bc.bits;
            a.ms = 200;
            const bool acted = peer_act(link::Op::brk, a);
            bool up = acted && dut_link(c);
            U4::enable(false);
            up = up && U4::lin(LinConfig{bc.eleven, false});
            U4::enable(true);
            U4::clear_flags(usart_lbd);
            const Drain d = collect(30);
            const bool lbd = U4::flag(usart_lbd);
            U4::lin_off();
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  a ", bc.bits, "-bit break, ", bc.eleven ? "11" : "10", "-bit detection: LBD ", lbd,
                  ", frames ", d.frames, crlf);
            all = all && up && lbd == bc.detect;
            (void)link_command_mode();
        }
        judge("LIN detects a 13-bit break at both lengths, a 9-bit low at neither, 10 bits not at 11", all);
    }
    (void)link_command_mode();
}

// ---------------------------------------------------------------------------
// p - half duplex hearing a second chip
// ---------------------------------------------------------------------------
void tp_half_duplex_wire() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        return;
    }
    // PB0 must not drive while the peer does: the TX pad is an INPUT here,
    // floating and then pulled up, the receiver listening under HDSEL.
    struct PadCase {
        PinPull pull;
        const char* name;
    };
    static constexpr PadCase pads[] = {{PinPull::none, "floating"}, {PinPull::up, "pulled up"}};
    uint16_t heard[2] = {};
    for (uint8_t k = 0; k < 2u; ++k) {
        link::Cfg c{};
        c.rate = 9600;
        link::Params a{};
        a.cfg = c;
        a.count = 4;
        a.value = 0x5A;
        a.ms = 400;
        const bool acted = peer_act(link::Op::drive_rxd, a);
        bool up = acted && u4_open(UartFormat{}, 9600);
        U4::transmitter(false);
        up = up && U4::half_duplex(true);
        (void)Tx4::input(pads[k].pull);
        u4_flush();
        const Drain d = collect(80);
        (void)U4::half_duplex(false);
        link::Report r{};
        (void)peer_report(r, 0);
        heard[k] = up && d.fe == 0u && d.last == 0x5Au ? d.frames : 0u;
        print(serial, "  HDSEL, PB0 an input ", pads[k].name, ": ", d.frames, " frames, last ", hex(d.last),
              ", fe ", d.fe, crlf);
        (void)link_command_mode();
    }
    judge("half duplex on PB0 as a floating input hears the four frames a second chip drives onto it",
          heard[0] == 4u);
    (void)link_command_mode();
}





/// The supply as the voltage detector read it at boot (main()).
bool supply_ok = true;

void banner() {
    print(serial, crlf, "test_x035_usart on ", device::part_name, " - the USARTs", crlf, crlf);
    print(serial, "  supply at boot: ", supply_ok ? "above 3.0 V" : "UNDER 3.0 V, every letter refused",
          " (the voltage detector at PLS 10)", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void usart4_handler() { (void)Loop::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // THE SUPPLY before any link pad is driven (the desk's rule).
    brio::Pwr::pvd(brio::PvdLevel::v3_0);
    (void)brio::delay_us(clock, 200);
    supply_ok = !brio::Pwr::supply_low();

    bench.letter('a', "the divisor in HCLK", ta_divisor);
    bench.letter('b', "the frame, register by register", tb_frames);
    bench.letter('c', "the modes and their exclusions", tc_modes);
    bench.letter('d', "a frame's length on the console", td_frame_time);
    bench.letter('e', "the break, on USART4", te_break);
    bench.letter('f', "the column on the probe's pads", tf_debug_column);
    bench.letter('h', "half duplex on PB0: its own frames", th_half_duplex_self);
    bench.second_letter('j', "the rendezvous, and the HSI against the crystal", tj_link);
    bench.second_letter('k', "the rates across the wire, timed by the far chip", tk_rates);
    bench.second_letter('l', "the frame matrix across the wire", tl_frames);
    bench.second_letter('m', "the receiver's errors from a crystal sender", tm_errors);
    bench.second_letter('n', "the peer's engine shapes", tn_shapes);
    bench.second_letter('o', "mute, IrDA and LIN on a wire", to_modes);
    bench.second_letter('p', "half duplex hearing a second chip", tp_half_duplex_wire);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW", brio::crlf);
        if (!supply_ok) {
            brio::print(serial, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): every letter refused, the link "
                                "pads left floating", brio::crlf);
        }
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
        } else if (!supply_ok) {
            brio::print(serial, "refused: the supply is under 3.0 V", brio::crlf);
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
