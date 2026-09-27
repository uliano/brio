// pin_check - the solder-joint check of a self-built AVR-Dx 48-pin board,
// the AVR128DB48 boards and the REV 1.2 board with an AVR128DA48: every
// GPIO the package bonds out that the board gives no other job carries a
// 100 Hz square wave, so each pin can be visited with a scope probe (a
// joint that is open or bridged shows at once); the clocks are tested
// first and their verdict is readable on their own pins; the console's
// two pins are tested by the console itself, and the board's LED and
// button by each other.
//
// THE CONSOLE IS PART OF THE CHECK, WHICH IS WHY IT IS HERE: the
// banner arriving proves PF4 (TX), and every character typed is echoed
// with its code - "rx 'a' 0x61" - which proves PF5 (RX) and proves it
// in the one way a terminal's own local echo cannot fake. A wrong code
// for the key pressed is a baud or a frame fault, not a joint. So
// PF4/PF5 are two pins NOT in the square wave: they carry the console
// instead.
//
// THE LED AND THE BUTTON: the LED on PF2 blinks, and every press of the
// button on PF3 (to ground, the pin pulled up) moves it to the next of
// four rates - 1, 2, 5 and 10 Hz - and says so on the console, which
// proves the button's joint, the LED's and the jumpers between them and
// the pins. So PF2/PF3 are not in the square wave either.
//
// It stays a tool with no kernel, no timer interrupt and no verdict
// grammar: a board that says nothing and waves nowhere is a board for
// the debugger, not for a message it could not deliver. The clock is
// the one thing it needs, and it needs it either way - the HF source and
// the internal oscillator are asked for the same 24 MHz, so the baud
// rate is right whichever of them answers.
//
// The HF clock. On a DB the main clock is asked for the 24 MHz crystal
// on PA0/PA1; on a DA - which has no crystal oscillator - for an
// external 24 MHz clock on PA0 (the REV 1.2 board makes one with a
// quartz and two inverters). When it starts the CPU runs from it and
// its pins stay the clock's; when it does not the CPU stays on OSCHF at
// the same rate and a DB's crystal pins join the square wave - so A
// SQUARE WAVE ON A CRYSTAL PIN MEANS A CRYSTAL THAT DID NOT START. A DA's
// PA0 is never driven, clock or not: the board's clock there is an
// ACTIVE output, and driving the pin against it would be two outputs
// fighting. The 32.768 kHz crystal
// on PF0/PF1 the same way: started, the pins are the oscillator's; not
// started within its wait, PF0/PF1 toggle with the rest. The banner says
// which happened, so the pins only confirm it.
//
// What toggles: PA0..7, PB0..5, PC0..7, PD0..7, PE0..3 - the clock pins
// as above. On a DB, PORTC is the MVIO domain: its eight pins toggle
// only when VDDIO2 is powered, so a flat PORTC with a live rest of the
// board is a VDDIO2 finding; on a DA, PORTC runs from VDD like the rest.
//
// Not toggling: PF2 (the LED) and PF3 (the button), as above; PF4/PF5,
// the console; PF6, the RESET input by fuse (grounding it restarts the
// program - which is its check); UPDI, a dedicated pin proved by the
// flash itself.
//
// The wave is not a frequency reference: echoing a character spends
// the echo's wire time inside the half-period that is running, so
// typing stretches that one edge by a fraction of a millisecond.
// Nobody types while reading the scope.
//
// Console: USART2 ALT1 (PF4/PF5) 460800. '?' repeats the banner, every
// other character is echoed.
//
// Wiring: a scope probe on the pin under check, moved from pin to pin
// (an LED with ~330 ohm to GND still tells a live pin from a dead one,
// at half brightness).

// build: boards = db48,da48
// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

// The DB's crystal, or the DA's external clock on PA0 (brio::has_xoschf).
using SysClock = brio::Clock<brio::has_xoschf ? brio::ClockSource::crystal
                                              : brio::ClockSource::external,
                             24'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

namespace {

using namespace brio;

constexpr uint8_t hf_pins = has_xoschf ? 0x03 : 0x01;   // the crystal's PA0/PA1, or EXTCLK's PA0
constexpr uint8_t xosc32k_pins = 0x03;  // PF0/PF1
constexpr uint8_t console_pins = 0x30;  // PF4/PF5
constexpr uint8_t reset_pin = 0x40;     // PF6
constexpr uint8_t led_pin = 0x04;       // PF2
constexpr uint8_t button_pin = 0x08;    // PF3

using Led = Pin<'F', 2>;
using Button = Pin<'F', 3>;

/// The LED's four rates, as half periods of the 5 ms wave step.
constexpr uint8_t led_rates_hz[] = {1, 2, 5, 10};
constexpr uint8_t led_half_steps[] = {100, 50, 20, 10};

/// The pins driven, port by port (bit n = pin n). Built once in main()
/// from the two crystal verdicts, then never changed.
struct Wave {
    uint8_t a, b, c, d, e, f;
};

Wave wave{};
bool on_hf = false;
bool on_xosc32k = false;

uint8_t led_rate = 0;       // index into led_rates_hz
uint8_t led_steps = 0;      // wave steps since the LED's last toggle
uint16_t presses = 0;
uint8_t button_low = 0;     // consecutive steps the button read pressed
bool button_down = false;

void toggle_all() {
    Port<'A'>::out_toggle(wave.a);
    Port<'B'>::out_toggle(wave.b);
    Port<'C'>::out_toggle(wave.c);
    Port<'D'>::out_toggle(wave.d);
    Port<'E'>::out_toggle(wave.e);
    Port<'F'>::out_toggle(wave.f);
}

void banner() {
    auto board = board_id();
    if (board.empty()) {
        board = "?";
    }
    print(serial, crlf, "pin_check - board ", board, ", silicon rev ",
          hex(SYSCFG.REVID), crlf);
    if constexpr (has_xoschf) {
        print(serial, "main clock: 24 MHz from ",
              on_hf ? "the crystal (PA0/PA1 held)"
                    : "OSCHF - THE CRYSTAL DID NOT START (PA0/PA1 waving)",
              crlf);
    } else {
        print(serial, "main clock: 24 MHz from ",
              on_hf ? "the external clock on PA0 (PA0 an input, PA1 waving)"
                    : "OSCHF - NO EXTERNAL CLOCK SEEN ON PA0 (PA0 an input, PA1 waving)",
              crlf);
    }
    print(serial, "32k crystal: ",
          on_xosc32k ? "running (PF0/PF1 held)"
                     : "not running (PF0/PF1 waving) - none fitted, or a joint",
          crlf);
    print(serial, "wave 100 Hz, bit = pin: PA=", hex(wave.a), " PB=", hex(wave.b),
          " PC=", hex(wave.c), " PD=", hex(wave.d), " PE=", hex(wave.e),
          " PF=", hex(wave.f), crlf);
    print(serial, "not driven: PF2 (LED), PF3 (button), PF4/PF5 (this console), "
                  "PF6 (RESET), UPDI", crlf);
    if constexpr (has_mvio) {
        print(serial, "PORTC is MVIO: flat PORTC with the rest alive = VDDIO2", crlf);
    }
    print(serial, "LED PF2 at ", led_rates_hz[led_rate], " Hz; button PF3 pressed ", presses,
          " time(s) - each press moves the LED to the next rate", crlf);
    print(serial, "rx errors: frame ", Serial::frame_errors(), " parity ",
          Serial::parity_errors(), " hw-overrun ", Serial::hw_overruns(),
          " ring-overrun ", Serial::rx_overruns(), crlf);
    print(serial, "type anything (echoed with its code = PF5 proven), "
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

/// One wave step (5 ms): the LED's half period counted, the button
/// sampled - a press is two steps low after a release, so a bounce
/// shorter than 10 ms is one press.
void step_led_and_button() {
    if (++led_steps >= led_half_steps[led_rate]) {
        led_steps = 0;
        Led::toggle();
    }
    if (!Button::read()) {
        if (button_low < 2u) {
            ++button_low;
        }
        if (button_low == 2u && !button_down) {
            button_down = true;
            ++presses;
            led_rate = static_cast<uint8_t>((led_rate + 1u) % sizeof(led_rates_hz));
            led_steps = 0;
            print(serial, "button PF3 pressed (", presses, "): LED PF2 at ",
                  led_rates_hz[led_rate], " Hz", crlf);
        }
    } else {
        button_low = 0;
        button_down = false;
    }
}

}  // namespace

int main() {
    // The 24 MHz HF source: started and switched to by init(); on failure
    // the CPU keeps OSCHF at the same 24 MHz (and a DB's crystal is
    // stopped, PA0/PA1 the PORT's again), so the baud divisor below - and
    // the wave's rate - are the same either way.
    on_hf = SysClock::init();

    Serial::init(clock, 460800);
    sei();

    // Said BEFORE the wait below, which is seconds long on a board with
    // no 32 kHz crystal fitted: without this line such a board looks
    // dead for those seconds, which is the wrong first impression for a
    // tool whose whole job is telling dead from alive.
    print(serial, crlf, "pin_check - testing the 32 kHz crystal, "
                        "a few seconds if none is fitted", crlf);

    // The 32.768 kHz crystal, FORCED ON: nothing here requests it, and
    // unrequested it would not run at all (clock.hpp's Xosc32k). The
    // default wait is longer than any crystal needs, and a board with
    // none fitted simply fails it.
    Xosc32k::start_crystal(Xosc32kStartup::cycles64k, false, true);
    on_xosc32k = Xosc32k::wait_stable();
    if (!on_xosc32k) {
        Xosc32k::stop();  // PF0/PF1 back to the PORT
    }

    wave = Wave{
        // A DA's PA0 is never driven: the board may carry an ACTIVE clock
        // there, and a pin driven against an oscillator's output is two
        // outputs fighting. A DB's crystal pins are passive and may wave.
        .a = static_cast<uint8_t>(0xFF & ~(on_hf || !has_xoschf ? hf_pins : 0)),
        .b = 0x3F,
        .c = 0xFF,
        .d = 0xFF,
        .e = 0x0F,
        .f = static_cast<uint8_t>(0x7F & ~reset_pin & ~console_pins & ~led_pin &
                                  ~button_pin & ~(on_xosc32k ? xosc32k_pins : 0)),
    };

    Port<'A'>::dir_set(wave.a);
    Port<'B'>::dir_set(wave.b);
    Port<'C'>::dir_set(wave.c);
    Port<'D'>::dir_set(wave.d);
    Port<'E'>::dir_set(wave.e);
    Port<'F'>::dir_set(wave.f);
    Led::output();
    Button::input(PinPull::up);

    banner();

    for (;;) {
        toggle_all();
        step_led_and_button();
        // Half a period, sliced so the console is answered promptly.
        // The RX ring holds 1.4 ms of full-rate traffic, so nothing is
        // lost between two polls whatever arrives.
        for (uint8_t i = 0; i < 10; ++i) {
            delay_us(clock, 500);
            poll_console();
        }
    }
}
