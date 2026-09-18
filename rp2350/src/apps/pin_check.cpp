// pin_check - the solder-joint and first-light check of an RP2350 board:
// every user GPIO the package bonds carries a 100 Hz square wave, so each
// pin can be visited with a scope probe (a joint that is open or bridged
// shows at once), and the console's two pins are tested by the console
// itself.
//
// THE CONSOLE IS PART OF THE CHECK, WHICH IS WHY IT IS HERE: the banner
// arriving proves GP0 (UART0 TX), and every character typed is echoed
// with its code - "rx 'a' 0x61" - which proves GP1 (UART0 RX) and proves
// it in the one way a terminal's own local echo cannot fake. A wrong code
// for the key pressed is a baud or a frame fault, not a joint. So GP0/GP1
// are two pins NOT in the square wave: they carry the console instead.
//
// It stays a tool with no kernel and no timer interrupt: a board that
// says nothing and waves nowhere is a board for the debugger, not for a
// message it could not deliver. The microsecond ruler (rp2350/mtime.hpp)
// is a bare counter here, with no comparator armed and no trap bound.
//
// THE CRYSTAL, AND WHY THE WAVE IS NOT ITS WITNESS ON THIS CHIP.
// SysClock::init() starts the 12 MHz crystal and the system PLL to
// 150 MHz; when the crystal does not come up the tree is left on the ring
// oscillator, which is neither exact nor stable. On the RP2040 that
// stretched every delay twenty-fold and the WAVE was the verdict. Here
// the wave is timed by the ruler, and the ruler is divided out of clk_ref
// - which is the ring oscillator in exactly that case - so the wave keeps
// roughly its rate and says nothing. WHAT SAYS IT IS THE CONSOLE: the
// divisor was computed for a clk_peri of 150 MHz and the core is running
// at a fraction of that, so not a readable character arrives. The ladder
// to read this board by:
//
//   banner and echo, wave at 100 Hz     everything is good, and the
//                                       banner states the rates the chip
//                                       measured of itself
//   no banner, but the wave is there    THE CRYSTAL DID NOT START - the
//                                       core, the pads and the ruler are
//                                       alive, the clock tree is not
//   neither                             a board for the debugger
//
// What toggles: every pin of the bank except GP0/GP1 and GP23 - GP2..GP22
// and GP24..GP29 on the QFN-60, and up to GP47 on the QFN-80, the eight
// ADC pads among them (GP40..GP47), waved as digital outputs. GP25 is the
// LED on the WeAct RP2350B board, lit at half brightness by the wave.
//
// Not toggling: GP0/GP1, the console; GP23, that board's user KEY - a
// button to ground on a driven output would be a short. SWCLK/SWDIO and
// the six QSPI flash pins are not user GPIOs and are not in this bank.
//
// PINS WIRED TO EACH OTHER ARE SAFE HERE, which is worth stating because
// a bench board usually has a few: the whole of a half is toggled by ONE
// word write, so two pins of the same half that are strapped together sit
// at the same level at all times and no current flows between them. The
// two halves are two writes, one instruction apart, so a pin of the low
// half strapped to one of the high half is briefly opposed - a few
// nanoseconds into a 4 mA driver, which is what the pad is specified to
// survive.
//
// The banner carries the chip's identity (manufacturer, part, STEPPING -
// which is what the errata are keyed by), the package the die reports,
// WHICH ARCHITECTURE this image is running, and the three clock rates the
// chip's own frequency counter measured against its crystal.
//
// Console: UART0 on GP0 (TX) / GP1 (RX), 115200 8N1 - a Debug Probe's
// UART bridge, crossed. '?' repeats the banner, every other character is
// echoed.
//
// Wiring: a scope probe on the pin under check, moved from pin to pin.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
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

/// The low half's mask of pins this package has: every bit below 32 that
/// the bank reaches. Written as a function because `1UL << 32` is not a
/// shift this language defines.
constexpr uint32_t low_half_pins() {
    return gpio_count >= 32u ? 0xFFFFFFFFUL : ((1UL << gpio_count) - 1u);
}
/// And the high half's, bit 0 being GP32 - empty on the QFN-60.
constexpr uint32_t high_half_pins() {
    return gpio_count > 32u ? ((1UL << (gpio_count - 32u)) - 1u) : 0u;
}

constexpr uint32_t console_mask = bit(0) | bit(1);
constexpr uint32_t key_mask = bit(23);
constexpr uint32_t wave_low = low_half_pins() & ~console_mask & ~key_mask;
constexpr uint32_t wave_high = high_half_pins();

bool on_crystal = false;

/// Half a period of the 100 Hz wave.
constexpr uint32_t half_period_us = 5000;

void banner() {
    const ChipId id = ChipId::read();
    print(serial, crlf, "pin_check - chip ", hex(id.manufacturer), "/", hex(id.part),
          " stepping ", hex(id.revision), " (A2 = 0x2, A3 = 0x3, A4 = 0x8)", crlf);
    print(serial, "package: the die reports ",
          ChipId::package_sel() == Package::qfn60 ? "QFN-60" : "QFN-80",
          ", this image is built for ", gpio_count, " GPIO", crlf);
    print(serial, "architecture: ",
          core_kind == CoreKind::hazard3 ? "RISC-V Hazard3" : "Arm Cortex-M33",
          ", core ", core_id(), crlf);
    print(serial, "clock: ",
          on_crystal ? "12 MHz crystal x PLL = 150 MHz"
                     : "THE CRYSTAL DID NOT START - on the ring oscillator",
          crlf);
    print(serial, "  clk_sys ", SysClock::count_hz(CountSource::clk_sys).value_or(0u),
          " Hz, clk_ref ", SysClock::count_hz(CountSource::clk_ref).value_or(0u),
          " Hz, clk_peri ", SysClock::count_hz(CountSource::clk_peri).value_or(0u),
          " Hz (the chip's own counter)", crlf);
    print(serial, "wave 100 Hz, bit = pin: low half ", hex(wave_low), ", high half ",
          hex(wave_high), " (bit 0 = GP32)", crlf);
    print(serial, "not driven: GP0/GP1 (this console), GP23 (the KEY)", crlf);
    print(serial, "GP25 is the LED - half brightness is the wave", crlf);
    print(serial, "rx errors: frame ", Serial::frame_errors(), " parity ",
          Serial::parity_errors(), " break ", Serial::break_errors(), " hw-overrun ",
          Serial::hw_overruns(), " ring-overrun ", Serial::rx_overruns(), crlf);
    print(serial, "type anything (echoed with its code = GP1 proven), '?' repeats this",
          crlf);
}

/// Drain whatever the RX ring holds. The echo carries the code as well as
/// the character: a terminal's local echo cannot produce it, and a code
/// that does not match the key pressed is a baud or frame fault.
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

/// Spin on the ruler for `us`, answering the console as we go. The ring
/// holds milliseconds of traffic at this rate, so nothing is lost between
/// two polls.
void wait_answering(uint32_t us, bool serial_ok) {
    const uint32_t t0 = Mtime::micros();
    while (Mtime::micros() - t0 < us) {
        if (serial_ok) {
            poll_console();
        }
    }
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES: a Cortex-M vector-table slot on one half,
// an entry of Hazard3's own dispatch on the other.
extern "C" void isr_uart0() { (void)Serial::isr(); }

int main() {
    on_crystal = SysClock::init();
    const bool ruler_ok = Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    enable_interrupts();

    Gpio::outputs(wave_low);
    Gpio::outputs_hi(wave_high);

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that did not come up would never return - the
    // wave would then be the only sign of life, which is exactly the
    // diagnosis one wants at that point.
    if (serial_ok) {
        banner();
        if (!ruler_ok) {
            print(serial, "THE RULER DID NOT START: clk_ref is not a whole number of "
                  "megahertz, so the wave below has no rate to keep", crlf);
        }
    }

    for (;;) {
        Gpio::out_toggle(wave_low);
        Gpio::out_toggle_hi(wave_high);
        wait_answering(half_period_us, serial_ok);
    }
}
