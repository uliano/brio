// test_x035_usart - the reference bench suite for the USARTs of the
// CH32X035 (RM ch. 14): the divisor in HCLK, the frame formats and the
// refusals, the modes and the exclusions the chapter states between them,
// a frame's own length on the wire timed by the transmitter, the break,
// the column the probe's pads forbid - and, across one jumper, the bytes
// themselves.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// THE CONSOLE IS USART2 on PA2/PA3 over the probe's serial. The instance
// under test is USART4 on its default column, PB0 (TX) and PB1 (RX), two
// pads WCH's QFN20 evaluation board brings to its pin header and nowhere
// else. NOTHING TO WIRE for `z`; letter `w` wants a jumper PB0-PB1 and
// DETECTS it first, and without it says so and judges nothing.
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
//      to TC on the STK counter, against ten bit times of the divisor
//   e  the break: USART4 opened on PB0/PB1, SBK set, and the time until the
//      hardware clears it on the break frame's stop bit
//   f  USART3's code 1 puts TX and RX on PC18/PC19, the debug port's pads:
//      init() answers false while the probe owns them, and touches nothing
//   w  (a jumper PB0-PB1, detected first) USART4 talking to itself: bytes
//      at 115200, 1 Mbaud and 3 Mbaud through the transport, 8E1 and 7O1
//      and 9N1 frames polled on the resource, a LIN break detected
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;
using Led = Pin<'A', 0>;

using Loop = Uart<4, P>;                 // PB0 TX, PB1 RX
using U4 = Usart<4>;
using Debug3 = Uart<3, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 1>;

TestBench<Serial> bench;

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
// d - a frame's length
// ---------------------------------------------------------------------------
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
}

// ---------------------------------------------------------------------------
// e - the break
// ---------------------------------------------------------------------------
void te_break() {
    const bool opened = Loop::init(clock, 115200);
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
// w - USART4 talking to itself across PB0-PB1
// ---------------------------------------------------------------------------
using Tx4 = Pin<'B', 0>;
using Rx4 = Pin<'B', 1>;

bool wire_present() {
    (void)Rx4::input(PinPull::up);   // PB1 has no pull-down: the up is the one to beat
    Tx4::output(false);
    (void)delay_us(clock, 20);
    const bool follows_low = !Rx4::read();
    Tx4::set();
    (void)delay_us(clock, 20);
    const bool follows_high = Rx4::read();
    Tx4::release();
    Rx4::release();
    return follows_low && follows_high;
}

bool loop_bytes(uint32_t baud) {
    if (!Loop::set_baud(SysClock::hz, baud)) {
        return false;
    }
    while (Loop::rx_pending() != 0u) {
        uint8_t discard = 0;
        (void)Loop::read_byte(discard);
    }
    static const uint8_t pattern[8] = {0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7E, 0x81};
    (void)Loop::write(pattern, 8);
    const uint32_t t0 = Ticker::ticks();
    while (Loop::rx_pending() < 8u && Ticker::ticks() - t0 < 20u) {
    }
    bool same = Loop::rx_pending() == 8u;
    for (uint8_t i = 0; i < 8u && same; ++i) {
        uint8_t b = 0;
        same = Loop::read_byte(b) && b == pattern[i];
    }
    return same;
}

bool polled_frame(UartFormat f, uint16_t word) {
    U4::enable(false);
    if (!U4::configure(f, usart_divisor(SysClock::hz, 115200))) {
        return false;
    }
    U4::regs().CTLR1 = static_cast<uint16_t>(U4::regs().CTLR1 | usart_te | usart_re | usart_ue);
    (void)delay_us(clock, 200);
    U4::clear_by_read();
    U4::write_word(word);
    uint32_t spin = 0;
    while (!U4::rx_ready() && spin < 200'000u) {
        ++spin;
    }
    const uint16_t errors = static_cast<uint16_t>(U4::status() & (usart_pe | usart_fe | usart_ne | usart_ore));
    const uint16_t got = static_cast<uint16_t>(U4::read_word() & uart_data_mask(f));
    return errors == 0u && got == (word & uart_data_mask(f));
}

void tw_loop() {
    if (!wire_present()) {
        print(serial, "  no wire: PB0-PB1 is not strapped, nothing judged", crlf);
        return;
    }
    const bool opened = Loop::init(clock, 115200);
    bench.verdict("USART4 opens on PB0/PB1", opened);
    bench.verdict("eight bytes loop back at 115200", loop_bytes(115200));
    bench.verdict("and at 1 Mbaud", loop_bytes(1'000'000));
    bench.verdict("and at 3 Mbaud, a divisor of 16", loop_bytes(3'000'000));
    print(serial, "  errors: frame ", Loop::frame_errors(), " noise ", Loop::noise_errors(),
          " parity ", Loop::parity_errors(), " overrun ", Loop::hw_overruns(), crlf);

    U4::rxne_interrupt(false);   // the polled frames are the resource's, not the ring's
    bench.verdict("an 8E1 frame loops back with no error",
                  polled_frame(UartFormat{UartBits::eight, UartParity::even, UartStop::one}, 0xA5));
    bench.verdict("a 7O1 frame loops back with no error",
                  polled_frame(UartFormat{UartBits::seven, UartParity::odd, UartStop::one}, 0x5A));
    bench.verdict("a 9N1 frame carries its ninth bit",
                  polled_frame(UartFormat{UartBits::nine, UartParity::none, UartStop::one}, 0x1C3));

    (void)polled_frame(UartFormat{}, 0x00);
    U4::enable(false);
    (void)U4::lin(LinConfig{false, false});
    U4::enable(true);
    U4::clear_flags(usart_lbd);
    U4::send_break();
    uint32_t spin = 0;
    while (!U4::flag(usart_lbd) && spin < 200'000u) {
        ++spin;
    }
    const bool lbd = U4::flag(usart_lbd);
    U4::clear_flags(usart_lbd);
    U4::lin_off();
    bench.verdict("in LIN mode the receiver detects the break the transmitter sends (LBD)", lbd);
    Loop::release();
}

void banner() {
    print(serial, crlf, "test_x035_usart on ", device::part_name, " - the USARTs", crlf, crlf);
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
    Led::output(true);
    brio::enable_interrupts();

    bench.letter('a', "the divisor in HCLK", ta_divisor);
    bench.letter('b', "the frame, register by register", tb_frames);
    bench.letter('c', "the modes and their exclusions", tc_modes);
    bench.letter('d', "a frame's length on the console", td_frame_time);
    bench.letter('e', "the break, on USART4", te_break);
    bench.letter('f', "the column on the probe's pads", tf_debug_column);
    bench.letter('w', "USART4 across a jumper PB0-PB1, detected first", tw_loop);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
