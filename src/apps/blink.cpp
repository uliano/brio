// blink - toggles an LED on PF2 at ~1 Hz (500 ms on / 500 ms off).
//
// The board runs at 24 MHz from the external crystal on PA0/PA1 (XOSCHF),
// set up by dx::init_clock_24mhz(), which falls back to the internal OSCHF
// if the crystal fails to start (CLK_PER is 24 MHz either way).
// F_CPU (24000000) comes from boards/AVR128DB48.json and feeds <util/delay.h>.
//
// Wiring: LED from PF2 -> resistor (~330 ohm) -> GND, so set() = on. toggle()
// blinks regardless of LED polarity, so either orientation is fine here.

#include <avr/io.h>
#include <util/delay.h>
#include "clock.hpp"
#include "pin.hpp"

namespace {
using Led = dx::Pin<'F', 2>;  // PF2
}  // namespace

int main() {
    dx::init_clock_24mhz();  // PA0/PA1 crystal -> CLK_PER = 24 MHz (OSCHF fallback)
    Led::output();           // PF2 as output (PORTF.DIRSET = PIN2_bm)

    for (;;) {
        Led::toggle();       // PORTF.OUTTGL = PIN2_bm
        _delay_ms(500);
    }
}
