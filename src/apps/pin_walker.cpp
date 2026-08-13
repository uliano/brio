// pin_walker - multimeter-friendly continuity/level test for the display
// bench wiring. Each signal of the bench map toggles slowly (2 s high,
// 2 s low, twice) while the serial console announces which one; probe
// each pin ON THE MODULE HEADER and confirm it swings rail-to-rail.
//
//   walking CS (PD0): HIGH ... LOW ... HIGH ... LOW
//   walking RS (PD1): ...
//
// A pin that does not swing at the module side = dead wire / bad contact.
// Also measure while you are there: VCC pin ~5 V steady, GND pin 0 V,
// and the module's 3.3 V LDO output if reachable.
//
// Deliberately kernel-free and blocking: a diagnostic should have as few
// moving parts as possible.

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

struct Walk {
    const char* name;
    volatile VPORT_t* vport;
    uint8_t mask;
};

const Walk walks[] = {
    {"SDA  (PA4)", &VPORTA, PIN4_bm},
    {"SCLK (PA6)", &VPORTA, PIN6_bm},
    {"CS   (PD0)", &VPORTD, PIN0_bm},
    {"RS   (PD1)", &VPORTD, PIN1_bm},
    {"RST  (PD2)", &VPORTD, PIN2_bm},
};

} // namespace

ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    brio::init_clock_24mhz();
    Serial::init(460800);
    sei();

    for (const Walk& w : walks) {           // everything driven, idle high
        w.vport->OUT |= w.mask;
        w.vport->DIR |= w.mask;
    }

    brio::print(serial, brio::crlf,
                "pin walker: probe each MODULE pin against board GND",
                brio::crlf);

    for (;;) {
        for (const Walk& w : walks) {
            brio::print(serial, "walking ", w.name, brio::crlf);
            for (uint8_t i = 0; i < 2; ++i) {
                brio::print(serial, "  HIGH", brio::crlf);
                w.vport->OUT |= w.mask;
                _delay_ms(2000);
                brio::print(serial, "  LOW", brio::crlf);
                w.vport->OUT &= ~w.mask;
                _delay_ms(2000);
            }
            w.vport->OUT |= w.mask;         // park high
        }
        brio::print(serial, "--- lap done, starting over ---", brio::crlf);
    }
}
