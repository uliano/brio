// pin_check - the solder-joint check of a self-built AVR128DB48 board:
// every GPIO the package bonds out carries a 100 Hz square wave, so
// each pin can be visited with a scope probe (a joint that is open or
// bridged shows at once), the two crystals are tested first and their
// verdict is readable on their own pins, and the console's two pins
// are tested by the console itself.
//
// THE CONSOLE IS PART OF THE CHECK, WHICH IS WHY IT IS HERE: the
// banner arriving proves PF4 (TX), and every character typed is echoed
// with its code - "rx 'a' 0x61" - which proves PF5 (RX) and proves it
// in the one way a terminal's own local echo cannot fake. A wrong code
// for the key pressed is a baud or a frame fault, not a joint. So
// PF4/PF5 are the two pins NOT in the square wave: they carry the
// console instead.
//
// It stays a tool with no kernel, no timer interrupt and no verdict
// grammar: a board that says nothing and waves nowhere is a board for
// the debugger, not for a message it could not deliver. The clock is
// the one thing it needs, and it needs it either way - the crystal and
// the internal oscillator are asked for the same 24 MHz, so the baud
// rate is right whichever of them answers.
//
// The crystals. The main clock is asked for the 24 MHz crystal on
// PA0/PA1; when it starts the CPU runs from it and those two pins stay
// the oscillator's (a scope on PA1 sees the swing), when it does not
// the CPU stays on OSCHF at the same rate and PA0/PA1 join the square
// wave - so A SQUARE WAVE ON XIN/XOUT MEANS A CRYSTAL THAT DID NOT
// START. The same for the 32.768 kHz crystal on PF0/PF1, which the
// board may or may not carry: started, the pins are the oscillator's;
// not started within its wait, PF0/PF1 toggle with the rest. The
// banner says which happened, so the pins only confirm it.
//
// What toggles: PA0..7, PB0..5, PC0..7, PD0..7, PE0..3, PF2, PF3 - the
// crystal pins as above. PORTC is the MVIO domain: its eight pins
// toggle only when VDDIO2 is powered, so a flat PORTC with a live rest
// of the board is a VDDIO2 finding.
//
// Not toggling: PF4/PF5, the console; PF6, the RESET input by fuse
// (grounding it restarts the program - which is its check); UPDI, a
// dedicated pin proved by the flash itself.
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

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

namespace {

using namespace brio;

constexpr uint8_t xoschf_pins = 0x03;   // PA0/PA1
constexpr uint8_t xosc32k_pins = 0x03;  // PF0/PF1
constexpr uint8_t console_pins = 0x30;  // PF4/PF5
constexpr uint8_t reset_pin = 0x40;     // PF6

/// The pins driven, port by port (bit n = pin n). Built once in main()
/// from the two crystal verdicts, then never changed.
struct Wave {
    uint8_t a, b, c, d, e, f;
};

Wave wave{};
bool on_crystal = false;
bool on_xosc32k = false;

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
    print(serial, "main clock: 24 MHz from ",
          on_crystal ? "the crystal (PA0/PA1 held)"
                     : "OSCHF - THE CRYSTAL DID NOT START (PA0/PA1 waving)",
          crlf);
    print(serial, "32k crystal: ",
          on_xosc32k ? "running (PF0/PF1 held)"
                     : "not running (PF0/PF1 waving) - none fitted, or a joint",
          crlf);
    print(serial, "wave 100 Hz, bit = pin: PA=", hex(wave.a), " PB=", hex(wave.b),
          " PC=", hex(wave.c), " PD=", hex(wave.d), " PE=", hex(wave.e),
          " PF=", hex(wave.f), crlf);
    print(serial, "not driven: PF4/PF5 (this console), PF6 (RESET), UPDI", crlf);
    print(serial, "PORTC is MVIO: flat PORTC with the rest alive = VDDIO2", crlf);
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

}  // namespace

int main() {
    // The 24 MHz crystal: started and switched to by init(); on failure
    // init() stops it, PA0/PA1 are the PORT's again and the CPU keeps
    // OSCHF at the same 24 MHz, so the baud divisor below - and the
    // wave's rate - are the same either way.
    on_crystal = SysClock::init();

    Serial::init(clock, 460800);
    sei();

    // Said BEFORE the wait below, which is seconds long on a board with
    // no 32 kHz crystal fitted: without this line such a board looks
    // dead for those seconds, which is the wrong first impression for a
    // tool whose whole job is telling dead from alive.
    print(serial, crlf, "pin_check - testing the 32 kHz crystal, "
                        "a few seconds if none is fitted", crlf);

    // The 32.768 kHz crystal: the default wait is longer than any
    // crystal needs, and a board with none fitted simply fails it.
    Xosc32k::start_crystal();
    on_xosc32k = Xosc32k::wait_stable();
    if (!on_xosc32k) {
        Xosc32k::stop();  // PF0/PF1 back to the PORT
    }

    wave = Wave{
        .a = static_cast<uint8_t>(0xFF & ~(on_crystal ? xoschf_pins : 0)),
        .b = 0x3F,
        .c = 0xFF,
        .d = 0xFF,
        .e = 0x0F,
        .f = static_cast<uint8_t>(0x7F & ~reset_pin & ~console_pins &
                                  ~(on_xosc32k ? xosc32k_pins : 0)),
    };

    Port<'A'>::dir_set(wave.a);
    Port<'B'>::dir_set(wave.b);
    Port<'C'>::dir_set(wave.c);
    Port<'D'>::dir_set(wave.d);
    Port<'E'>::dir_set(wave.e);
    Port<'F'>::dir_set(wave.f);

    banner();

    for (;;) {
        toggle_all();
        // Half a period, sliced so the console is answered promptly.
        // The RX ring holds 1.4 ms of full-rate traffic, so nothing is
        // lost between two polls whatever arrives.
        for (uint8_t i = 0; i < 10; ++i) {
            delay_us(clock, 500);
            poll_console();
        }
    }
}
