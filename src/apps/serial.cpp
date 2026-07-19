// serial - placeholder bring-up app for the AVR128DB48 board.
//
// Brings up USART2 on PF4(TXD)/PF5(RXD) (ALT1 position, PORTMUX-routed) at 460800 8N1 and prints an
// incrementing, RTC-timestamped counter once every 500 ms, toggling PF2 in
// step. Each line also reports the main clock source as detected at boot
// (XTAL = 24 MHz crystal on PA0/PA1, OSCHF = internal fallback), so the
// crystal health is visible from any serial monitor at any time:
// `pio device monitor -e serial`.
//
// Ported from the avr128db28_experiments twin (in turn from AVR-Multislope):
//  - Uart<>   : interrupt-driven UART transport with ring buffers (uart.hpp).
//               PF4/PF5 is USART2 ALT1 -> UART_ALTERNATE (PORTMUX routing done by Uart<>).
//  - Ticker   : RTC/PIT timebase at 1024 Hz on the internal 32 kHz osc
//               (ticker.hpp); gives the [seconds.ticks] stamp on each line.
//
// Clock: 24 MHz crystal on PA0/PA1 via init_clock_24mhz(), with automatic
// OSCHF fallback; F_CPU=24000000 (board JSON) sets the UART BAUD divisor and
// is correct for both sources.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "clock.hpp"
#include "pin.hpp"
#include "uart.hpp"
#include "ticker.hpp"

namespace {
using Led = Pin<'F', 2>;  // PF2, blinks in step with each printed line
}  // namespace

// Global so the USART2 + RTC interrupt vectors below can reach them.
Uart<2, UART_ALTERNATE> serial(460800);

ISR(USART2_RXC_vect) { serial.rxc(); }        // byte received -> RX ring
ISR(USART2_DRE_vect) { serial.dre(); }        // TX reg empty  -> next byte from TX ring
ISR(RTC_PIT_vect)    { Ticker::ptr->pit(); }  // 1024 Hz timebase tick

int main() {
    const bool xtal = init_clock_24mhz();  // 24 MHz BEFORE we transmit (BAUD divisor)
    const char *clk = xtal ? "XTAL" : "OSCHF";
    Led::output();
    init_ticker();       // RTC PIT timebase (internal 32.768 kHz oscillator)
    sei();               // the UART and the ticker are interrupt-driven

    serial.print("\r\nAVR128DB48 serial up: USART2 PF4/PF5, 460800 8N1, 24 MHz, clk=");
    serial.print(clk);
    serial.newline(true);

    TimeStamp ts;
    uint32_t count = 0;
    for (;;) {
        Ticker::ptr->now(ts);
        serial.print("[");
        serial.print(ts);        // "<seconds>s.<ticks>t"
        serial.print("] ");
        serial.print(clk);
        serial.print(" count = ");
        serial.print(count, 10);
        serial.newline(true);    // CR+LF

        Led::toggle();
        _delay_ms(500);
        ++count;
    }
}
