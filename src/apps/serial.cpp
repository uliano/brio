// serial - bring-up app for the AVR128DB48 board, on the brio framework.
//
// Brings up USART2 on PF4(TXD)/PF5(RXD) (ALT1 position, PORTMUX-routed) at
// 460800 8N1 and prints an incrementing, RTC-timestamped counter once every
// 500 ms, toggling PF2 in step. Each line also reports the main clock source
// as detected at boot (XTAL = 24 MHz crystal on PA0/PA1, OSCHF = internal
// fallback), so the crystal health is visible from any serial monitor at any
// time: `pio device monitor -e serial`.
//
// Framework pieces exercised:
//  - brio::Uart<2, Route::alt1> : static (monostate) interrupt-driven USART
//  - brio::Ticker               : static RTC/PIT timebase at 1024 Hz
//  - brio::print(...)           : variadic formatting over any ByteSink
//
// Clock: 24 MHz crystal on PA0/PA1 via brio::init_clock_24mhz(), with
// automatic OSCHF fallback; F_CPU=24000000 (board JSON) sets the UART BAUD
// divisor and is correct for both sources.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "clock.hpp"
#include "pin.hpp"
#include "uart.hpp"
#include "ticker.hpp"
#include "print.hpp"

namespace {
using Led = brio::Pin<'F', 2>;  // PF2, blinks in step with each printed line
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;      // zero-cost tag for print(serial, ...)
}  // namespace

ISR(USART2_RXC_vect) { Serial::rxc(); }      // byte received -> RX ring
ISR(USART2_DRE_vect) { Serial::dre(); }      // TX reg empty  -> next byte from TX ring
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }  // 1024 Hz timebase tick

int main() {
    const bool xtal = brio::init_clock_24mhz();  // 24 MHz BEFORE Serial::init (BAUD divisor)
    const char *clk = xtal ? "XTAL" : "OSCHF";
    Led::output();
    Serial::init(460800);
    brio::Ticker::init();  // RTC PIT timebase (internal 32.768 kHz oscillator)
    sei();               // the UART and the ticker are interrupt-driven

    brio::print(serial, brio::crlf,
              "AVR128DB48 serial up: USART2 PF4/PF5, 460800 8N1, 24 MHz, clk=",
              clk, brio::crlf);

    brio::TimeStamp ts;
    uint32_t count = 0;
    for (;;) {
        brio::Ticker::now(ts);
        brio::print(serial, "[", ts, "] ", clk, " count = ", count, brio::crlf);

        Led::toggle();
        _delay_ms(500);
        ++count;
    }
}
