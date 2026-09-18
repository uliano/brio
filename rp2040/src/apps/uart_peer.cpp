// uart_peer - THE FAR END OF A SERIAL LINK, for a board that has no
// console of its own.
//
// Every two-board serial letter of this project so far has been driven by
// typing one letter on each board's console. That needs a console on
// both, and a board whose console pins have been given to the link (or
// whose bridge is not cabled) has none. This app is the answer: a
// FIXED-ROLE firmware with no console, no kernel and no menu, whose ROLE
// is chosen by the board under test over a WIRE rather than by a command
// over a port that does not exist.
//
// The link: UART1 on GP4 (TX) / GP5 (RX) at 115200 8N1, crossed to the
// other board's UART1.
// The mode line: GP20, an input pulled down, driven by the other board.
// The LED (GP25) follows it.
//
//   mode line LOW   ECHO: every byte that arrives on GP5 goes back out on
//                   GP4, unchanged and as fast as the rings allow. This is
//                   what proves a link byte-exact in both directions.
//   mode line HIGH  SEND: once per rising edge, 64 bytes of the pattern
//                   below go out on GP4 and nothing is echoed. This is
//                   what a receiver testing its own FRAME FORMAT against a
//                   known one needs - an echo cannot serve there, because
//                   an echo returns the bits it was given and a test of
//                   the wrong format has to be given bits it did not send.
//
// The pattern is the xorshift every stress tool of this project
// generates: a 32-bit state seeded 0x12345678, the low byte of each step.
//
// Nothing here is specific to the chip at the other end: it is a serial
// peer, and the board under test may be of any family.
//
// build: boards = pico,picow,weact2040

#include <stdint.h>

#include <span>

#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins link_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using Link = Uart<1, link_pins, 512, 512>;

using ModeLine = Pin<20>;
using Led = Pin<25>;

/// The pattern both ends of every stream of this project generate.
struct Lfsr {
    uint32_t state = 0x12345678u;
    uint8_t next() {
        uint32_t s = state;
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        state = s;
        return static_cast<uint8_t>(s & 0xFFu);
    }
};

void send_burst() {
    Lfsr tx;
    uint8_t out[64];
    for (uint8_t& b : out) {
        b = tx.next();
    }
    (void)Link::write_bulk(out);
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart1() { (void)Link::isr(); }

int main() {
    (void)SysClock::init();
    (void)Timer::init(clock);
    const bool link_ok = Link::init(clock, 115200);
    ModeLine::input(PinPull::down);
    Led::output();
    enable_interrupts();

    bool sending = false;
    for (;;) {
        const bool want_send = ModeLine::read();
        Led::duty(want_send ? 1 : 0);
        if (want_send && !sending) {
            // The rising edge: one burst, then silence until it falls.
            sending = true;
            if (link_ok) {
                send_burst();
            }
        } else if (!want_send) {
            sending = false;
        }
        if (!want_send && link_ok) {
            uint8_t buf[64];
            const uint32_t n = Link::read_bulk(buf);
            if (n != 0u) {
                (void)Link::write_bulk(std::span<const uint8_t>(buf, n));
            }
        }
    }
}
