// chatter - a desk identification beacon: it does nothing but transmit
// on the console UART continuously, so the board running it is the one
// whose on-board TX LED is lit. Flash it on the board you are trying to
// tell apart (the two bench boards are physically identical); every
// line carries the USERROW label, so the console names the board too.
//
// Wiring: none. Console on USART2 ALT1 (PF4/PF5) at 460800, like every
// bench app.

#include <avr/interrupt.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    sei();
    auto board = brio::board_id();
    if (board.empty()) board = "?";
    uint32_t n = 0;
    for (;;) {
        brio::print(serial, "chatter ", n++, " - this board is ", board,
                    " (clk=", xtal ? "XTAL" : "OSCHF", "), watch my TX LED",
                    brio::crlf);
        brio::delay_us(clock, 50'000);
    }
}
