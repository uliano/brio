// display_9bit - tests the "panel strapped for 3-wire SPI" hypothesis:
// same inversion blink as display_inv, but bit-banged in 9-BIT frames
// (D/C travels as the FIRST bit of each word, the RS pin is ignored by
// a 3-wire-strapped controller).
//
//   panel pulses white<->black -> mystery solved: 3-wire interface,
//                                 the SPI engine grows a 9-bit story
//   still frozen white         -> next suspects: SPI mode 3, module LDO,
//                                 buffer, or a genuinely dead panel
//
// Bit-banged at ~100 kHz on the same wires (PA4=SDA, PA6=SCL, PD0=CS,
// PD2=RST; PD1/RS parked high, deliberately unused). Kernel-free and
// blocking: diagnostics want few moving parts.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/uart.hpp"
#include "util/print.hpp"

namespace {

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

using Sda = brio::Pin<'A', 4>;
using Scl = brio::Pin<'A', 6>;
using Cs  = brio::Pin<'D', 0>;
using Rs  = brio::Pin<'D', 1>;   // parked, ignored in 3-wire mode
using Rst = brio::Pin<'D', 2>;

void send9(bool data_not_cmd, uint8_t byte) {
    Cs::clear();
    _delay_us(1);
    for (int8_t bit = 8; bit >= 0; --bit) {
        const bool level =
            (bit == 8) ? data_not_cmd : ((byte >> bit) & 1) != 0;
        if (level) {
            Sda::set();
        } else {
            Sda::clear();
        }
        _delay_us(2);
        Scl::set();
        _delay_us(2);
        Scl::clear();
    }
    _delay_us(1);
    Cs::set();
    _delay_us(2);
}

void cmd(uint8_t c) { send9(false, c); }

} // namespace

ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    sei();

    Sda::clear(); Sda::output();
    Scl::clear(); Scl::output();               // idle low, mode-0 style
    Cs::set();    Cs::output();
    Rs::set();    Rs::output();
    Rst::set();   Rst::output();
    brio::Pin<'D', 3>::set();                  // deselect the MCP3550
    brio::Pin<'D', 3>::output();

    brio::print(serial, brio::crlf, "9-bit (3-wire SPI) blink probe",
                brio::crlf);

    Rst::clear();
    _delay_ms(10);
    Rst::set();
    _delay_ms(150);

    cmd(0x11);                                 // SLPOUT
    _delay_ms(150);
    cmd(0x29);                                 // DISPON
    _delay_ms(25);

    bool inverted = false;
    for (;;) {
        inverted = !inverted;
        cmd(inverted ? 0x21 : 0x20);           // INVON / INVOFF
        brio::print(serial, inverted ? "INVON" : "INVOFF", brio::crlf);
        _delay_ms(500);
    }
}
