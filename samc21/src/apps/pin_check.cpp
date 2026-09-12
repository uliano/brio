// pin_check - the solder-joint check of a self-built ATSAMC21J18A
// board: every GPIO the 64-pin package bonds out carries a 100 Hz
// square wave, so each pin can be visited with a scope probe (a joint
// that is open or bridged shows at once), the two crystals are tested
// first and their verdict is readable on their own pins, and the
// console's two pins are tested by the console itself.
//
// THE CONSOLE IS PART OF THE CHECK, WHICH IS WHY IT IS HERE: the
// banner arriving proves PB30 (TX), and every character typed is
// echoed with its code - "rx 'a' 0x61" - which proves PB31 (RX) and
// proves it in the one way a terminal's own local echo cannot fake. A
// wrong code for the key pressed is a baud or a frame fault, not a
// joint. So PB30/PB31 are the two pins NOT in the square wave: they
// carry the console instead.
//
// It stays a tool with no kernel and no timer interrupt: a board that
// says nothing and waves nowhere is a board for the debugger, not for
// a message it could not deliver. The CPU stays on OSC48M throughout -
// the crystals are measured, never run from, so the baud rate does not
// depend on them - and SysTick is a bare counter for the delays, with
// no handler bound.
//
// The crystals. The 24 MHz crystal on PA14/PA15 is started and waited
// for: ready, the oscillator keeps XIN/XOUT (20.5.1 - a scope on PA15
// sees the swing) and those two pins are left out of the wave; not
// ready within its wait, it is stopped, the pins return to PORT and
// join the wave - so A SQUARE WAVE ON XIN/XOUT MEANS A CRYSTAL THAT
// DID NOT START. The same for the 32.768 kHz crystal on PA00/PA01,
// optional on the design: started, the pins are the oscillator's; not
// started, PA00/PA01 toggle with the rest. The banner says which
// happened, so the pins only confirm it.
//
// What toggles: PA00..PA13, PA16..PA25, PA27, PA28, PB00..PB17, PB22,
// PB23 - the crystal pins as above. PB23 is the board's LED, lit at
// half brightness by the wave. PB22 is the user button, fitted the
// pull-down way, so it is driven high against that resistor and must
// NOT be pressed during the check.
//
// Not toggling: PB30/PB31, the console; PA30/PA31, SWCLK/SWDIO -
// driven as GPIO they would lock the probe out of the next flash and
// out of the debugger this program is meant to leave a door open for,
// and the flash that put this image on the board is their check.
// RESETN is not a GPIO.
//
// The wave is not a frequency reference: echoing a character spends
// the echo's wire time inside the half-period that is running, so
// typing stretches that one edge by a fraction of a millisecond.
// Nobody types while reading the scope.
//
// Console: SERCOM5 PAD0/PAD1 (PB30/PB31) 115200 8N1. '?' repeats the
// banner, every other character is echoed.
//
// Wiring: a scope probe on the pin under check, moved from pin to pin
// (an LED with ~330 ohm to GND still tells a live pin from a dead one,
// at half brightness).
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc21/clock.hpp"
#include "samc21/delay.hpp"
#include "samc21/nvic.hpp"
#include "samc21/nvm.hpp"
#include "samc21/osc32kctrl.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;

constexpr uint32_t bit(uint8_t n) { return 1UL << n; }

constexpr uint32_t xosc_pins = bit(14) | bit(15);     // XIN/XOUT
constexpr uint32_t xosc32k_pins = bit(0) | bit(1);    // XIN32/XOUT32
constexpr uint32_t swd_pins = bit(30) | bit(31);      // SWCLK/SWDIO
constexpr uint32_t console_pins = bit(30) | bit(31);  // PB30/PB31

// The J package's bonded pads, group by group (the device header's
// PIN_Pxnn set): PA00..PA25, PA27, PA28, PA30, PA31 and PB00..PB17,
// PB22, PB23, PB30, PB31.
constexpr uint32_t bonded_a = 0x03FFFFFFUL | bit(27) | bit(28) | bit(30) | bit(31);
constexpr uint32_t bonded_b = 0x0003FFFFUL | bit(22) | bit(23) | bit(30) | bit(31);

uint32_t wave_a = 0;
uint32_t wave_b = 0;
bool on_xosc = false;
bool on_xosc32k = false;

/// Eight zero-padded lowercase hex digits - the format the bench
/// manifest records a die serial in, which a bare hex() cannot produce.
/// A board being checked for the first time is also a board whose
/// identity has to be written down somewhere.
const char* hex8(uint32_t v, char* buf) {
    static const char digits[] = "0123456789abcdef";
    for (uint8_t i = 0; i < 8u; ++i) {
        buf[7u - i] = digits[(v >> (4u * i)) & 0xFu];
    }
    buf[8] = '\0';
    return buf;
}

void banner() {
    const DeviceSerial die = DeviceSerial::read();
    char b0[9], b1[9], b2[9], b3[9];
    print(serial, crlf, "pin_check - die serial ", hex8(die.word[0], b0), "-",
          hex8(die.word[1], b1), "-", hex8(die.word[2], b2), "-",
          hex8(die.word[3], b3), crlf);
    print(serial, "cpu: OSC48M throughout (the crystals are measured, not run from)",
          crlf);
    print(serial, "24 MHz crystal: ",
          on_xosc ? "running (PA14/PA15 held)"
                  : "DID NOT START (PA14/PA15 waving)",
          crlf);
    print(serial, "32k crystal: ",
          on_xosc32k ? "running (PA00/PA01 held)"
                     : "not running (PA00/PA01 waving) - none fitted, or a joint",
          crlf);
    print(serial, "wave 100 Hz, bit = pin: PA=", hex(wave_a), " PB=", hex(wave_b),
          crlf);
    print(serial, "not driven: PB30/PB31 (this console), PA30/PA31 (SWD), RESETN",
          crlf);
    print(serial, "PB23 is the LED, PB22 the button - do not press it now", crlf);
    print(serial, "rx errors: frame ", Serial::frame_errors(), " parity ",
          Serial::parity_errors(), " hw-overrun ", Serial::hw_overruns(),
          " ring-overrun ", Serial::rx_overruns(), crlf);
    print(serial, "type anything (echoed with its code = PB31 proven), "
                  "'?' repeats this", crlf);
}

/// Drain whatever the RX ring holds. The echo carries the code as well
/// as the character: a terminal's local echo cannot produce it, and a
/// code that does not match the key pressed is a baud or frame fault.
void poll_console() {
    uint8_t c = 0;
    while (Serial::read_byte(c)) {
        if (c == '?') {
            banner();
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, "rx ");
        if (c >= 0x20 && c < 0x7F) {
            print(serial, "'", static_cast<char>(c), "' ");
        }
        print(serial, hex(c), crlf);
    }
}

}  // namespace

// ---- target glue ------------------------------------------------------------
// ONE vector for the whole peripheral: isr() reads INTFLAG against
// INTENSET and serves whatever is genuinely pending.
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

int main() {
    (void)SysClock::init();
    (void)SysTickCounter::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    enable_interrupts();

    // Said BEFORE the waits below, which are seconds long for a crystal
    // that is absent or dead: without this line such a board looks dead
    // for those seconds, which is the wrong first impression for a tool
    // whose whole job is telling dead from alive.
    if (serial_ok) {
        print(serial, crlf, "pin_check - testing the crystals, "
                            "a few seconds each if one is missing", crlf);
    }

    // The 24 MHz crystal: a bounded wait, false means "not oscillating"
    // and the pins are given back for the wave.
    on_xosc = Xosc::init(XoscConfig{.hz = 24'000'000UL, .startup = 4});
    if (!on_xosc) {
        Xosc::stop();
    }
    // The 32.768 kHz crystal: about four seconds of wait at most.
    on_xosc32k = Xosc32k::init(Xosc32kConfig{.crystal = true}, 40'000'000UL);
    if (!on_xosc32k) {
        Xosc32k::stop();
    }

    wave_a = bonded_a & ~swd_pins & ~(on_xosc ? xosc_pins : 0u) &
             ~(on_xosc32k ? xosc32k_pins : 0u);
    wave_b = bonded_b & ~console_pins;

    Port<'A'>::dir_set(wave_a);
    Port<'B'>::dir_set(wave_b);

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that did not come up would never return -
    // the wave would then be the only sign of life, which is exactly
    // the diagnosis one wants at that point.
    if (serial_ok) {
        banner();
    }

    for (;;) {
        Port<'A'>::out_toggle(wave_a);
        Port<'B'>::out_toggle(wave_b);
        // Half a period, sliced so the console is answered promptly
        // (and because delay_us refuses a millisecond or more). The RX
        // ring holds milliseconds of traffic at this rate, so nothing
        // is lost between two polls.
        for (uint8_t i = 0; i < 10; ++i) {
            (void)delay_us(clock, 500u);
            if (serial_ok) {
                poll_console();
            }
        }
    }
}
