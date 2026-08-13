// display_9225 - tests the REGISTER-BASED controller hypothesis (the
// "RS" pin lexicon: ILI9225-family, not MIPI-DCS). Serial protocol:
// 16-bit register INDEX with RS low, 16-bit DATA with RS high, each in
// its own CS window, MSB first. Full ILI9225 init, red fill (176x220),
// then blink via the display-control register R07 once per second:
//
//   red screen / any visible pulsing -> register-based controller found
//   still frozen white               -> with a healthy LDO reading, the
//                                       panel/module earns its verdict
//
// Bit-banged, kernel-free, blocking: diagnostics want few moving parts.

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
using Rs  = brio::Pin<'D', 1>;
using Rst = brio::Pin<'D', 2>;

void wr16(uint16_t v) {
    for (int8_t bit = 15; bit >= 0; --bit) {
        if ((v >> bit) & 1) {
            Sda::set();
        } else {
            Sda::clear();
        }
        Scl::set();
        Scl::clear();
    }
}

void write_index(uint16_t idx) {
    Rs::clear();
    Cs::clear();
    wr16(idx);
    Cs::set();
    Rs::set();
}

void write_data(uint16_t val) {
    Rs::set();
    Cs::clear();
    wr16(val);
    Cs::set();
}

void reg(uint16_t idx, uint16_t val) {
    write_index(idx);
    write_data(val);
}

constexpr uint16_t width = 176;
constexpr uint16_t height = 220;

void init_9225() {
    // power off
    reg(0x10, 0x0000); reg(0x11, 0x0000); reg(0x12, 0x0000);
    reg(0x13, 0x0000); reg(0x14, 0x0000);
    _delay_ms(40);
    // power on sequence
    reg(0x11, 0x0018); reg(0x12, 0x6121); reg(0x13, 0x006F);
    reg(0x14, 0x495F); reg(0x10, 0x0800);
    _delay_ms(10);
    reg(0x11, 0x103B);
    _delay_ms(50);
    // driver / interface setup
    reg(0x01, 0x011C); reg(0x02, 0x0100); reg(0x03, 0x1030);
    reg(0x07, 0x0000); reg(0x08, 0x0808); reg(0x0B, 0x1100);
    reg(0x0C, 0x0000); reg(0x0F, 0x0D01); reg(0x15, 0x0020);
    reg(0x20, 0x0000); reg(0x21, 0x0000);
    // window = full screen
    reg(0x36, width - 1); reg(0x37, 0x0000);
    reg(0x38, height - 1); reg(0x39, 0x0000);
    // gamma (typical values)
    reg(0x50, 0x0400); reg(0x51, 0x060B); reg(0x52, 0x0C0A);
    reg(0x53, 0x0105); reg(0x54, 0x0A0C); reg(0x55, 0x0B06);
    reg(0x56, 0x0004); reg(0x57, 0x0501); reg(0x58, 0x0E00);
    reg(0x59, 0x000E);
    _delay_ms(50);
    reg(0x07, 0x1017);                 // display ON
    _delay_ms(50);
}

} // namespace

ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    sei();

    Sda::clear(); Sda::output();
    Scl::clear(); Scl::output();
    Cs::set();    Cs::output();
    Rs::set();    Rs::output();
    Rst::set();   Rst::output();
    brio::Pin<'D', 3>::set();
    brio::Pin<'D', 3>::output();

    brio::print(serial, brio::crlf, "ILI9225 register-protocol probe",
                brio::crlf);

    Rst::clear();
    _delay_ms(10);
    Rst::set();
    _delay_ms(50);

    init_9225();
    brio::print(serial, "init done, filling red...", brio::crlf);

    reg(0x20, 0x0000);                 // GRAM address 0,0
    reg(0x21, 0x0000);
    write_index(0x22);                 // GRAM write
    for (uint32_t i = 0; i < static_cast<uint32_t>(width) * height; ++i) {
        write_data(0xF800);            // red
    }
    brio::print(serial, "filled - blinking display on/off", brio::crlf);

    bool on = true;
    for (;;) {
        _delay_ms(1000);
        on = !on;
        reg(0x07, on ? 0x1017 : 0x0000);
        brio::print(serial, on ? "R07 ON" : "R07 OFF", brio::crlf);
    }
}
