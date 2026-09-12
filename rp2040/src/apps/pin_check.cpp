// pin_check - the solder-joint and first-light check of an RP2040 board:
// every user GPIO the board offers carries a 100 Hz square wave, so each
// pin can be visited with a scope probe (a joint that is open or bridged
// shows at once), the crystal is tested first and its verdict is
// readable on every pin, and the console's two pins are tested by the
// console itself.
//
// THE CONSOLE IS PART OF THE CHECK, WHICH IS WHY IT IS HERE: the banner
// arriving proves GP0 (UART0 TX), and every character typed is echoed
// with its code - "rx 'a' 0x61" - which proves GP1 (UART0 RX) and proves
// it in the one way a terminal's own local echo cannot fake. A wrong
// code for the key pressed is a baud or a frame fault, not a joint. So
// GP0/GP1 are two pins NOT in the square wave: they carry the console
// instead.
//
// It stays a tool with no kernel and no timer interrupt: a board that
// says nothing and waves nowhere is a board for the debugger, not for
// a message it could not deliver. SysTick is a bare counter for the
// delays, with no handler bound.
//
// THE CRYSTAL. SysClock::init() starts the 12 MHz crystal and the PLL
// to 125 MHz; when the crystal does not come up the tree stays on the
// ring oscillator (about 6.5 MHz, neither exact nor stable) and every
// delay computed for 125 MHz stretches by twenty times - so A WAVE FAR
// SLOWER THAN 100 HZ MEANS A CRYSTAL THAT DID NOT START, on every pin
// at once, and the console is mute (its divisor was computed for a
// clock the core is not on). A wave at 100.0 Hz is the crystal's own
// verdict: the ring oscillator could not hold it.
//
// What toggles: GP2..GP22 and GP24..GP29. GP25 is the LED on a Pico and
// on a WeAct board, lit at half brightness by the wave; GP26..GP29 are
// the ADC's pads, waved as digital outputs here. GP24 and GP29 are a
// Pico's VBUS sense and VSYS/3 dividers (hundreds of kilohms into a
// driven pin: harmless) and plain header pins on a WeAct. On a Pico W
// GP24, GP25 and GP29 are the radio chip's SPI lines and reach no
// header: the wave lands on an unpowered chip (GP23 below keeps it
// so) and no LED - that board's LED is on the radio.
//
// Not toggling: GP0/GP1, the console; GP23 - a WeAct board's user KEY
// (a button to ground on a driven output would be a short), a Pico's
// regulator PS pin (toggling its power-save mode at 100 Hz says
// nothing about a joint) and a Pico W's WL_REG_ON, which left low
// keeps the radio off. SWCLK/SWDIO and the six QSPI flash pins are
// not user GPIOs.
//
// The banner carries the chip id (manufacturer, part, silicon
// revision) and the clock's state.
//
// Console: UART0 on GP0 (TX) / GP1 (RX), 115200 8N1 - the Debug
// Probe's UART bridge, crossed. '?' repeats the banner, every other
// character is echoed.
//
// Wiring: a scope probe on the pin under check, moved from pin to pin.
//
// build: boards = pico,picow,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/sysinfo.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

constexpr uint32_t bit(uint8_t n) { return 1UL << n; }

constexpr uint32_t all_pins = (1UL << gpio_count) - 1u;
constexpr uint32_t console_pins_mask = bit(0) | bit(1);
constexpr uint32_t key_pin = bit(23);
constexpr uint32_t wave = all_pins & ~console_pins_mask & ~key_pin;

bool on_crystal = false;

void banner() {
    const ChipId id = ChipId::read();
    print(serial, crlf, "pin_check - chip ", hex(id.manufacturer), "/", hex(id.part),
          " rev B", id.revision, crlf);
    print(serial, "clock: ",
          on_crystal ? "12 MHz crystal x PLL = 125 MHz"
                     : "CRYSTAL DID NOT START - on the ring oscillator, the wave is slow",
          crlf);
    print(serial, "wave 100 Hz, bit = pin: ", hex(wave), crlf);
    print(serial, "not driven: GP0/GP1 (this console), GP23 (KEY / PS)", crlf);
    print(serial, "GP25 is the LED - half brightness is the wave", crlf);
    print(serial, "rx errors: frame ", Serial::frame_errors(), " parity ",
          Serial::parity_errors(), " break ", Serial::break_errors(), " hw-overrun ",
          Serial::hw_overruns(), " ring-overrun ", Serial::rx_overruns(), crlf);
    print(serial, "type anything (echoed with its code = GP1 proven), '?' repeats this",
          crlf);
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
extern "C" void isr_uart0() { (void)Serial::isr(); }

int main() {
    on_crystal = SysClock::init();
    (void)SysTickCounter::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    enable_interrupts();

    Gpio::outputs(wave);

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that did not come up would never return -
    // the wave would then be the only sign of life, which is exactly
    // the diagnosis one wants at that point.
    if (serial_ok) {
        banner();
    }

    for (;;) {
        Gpio::out_toggle(wave);
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
