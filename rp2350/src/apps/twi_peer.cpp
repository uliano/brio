// twi_peer - a scriptable second chip on the bus for the far end of a
// two-board I2C test, on an RP2350: the far board is the DUT and the bus
// host, and it drives this one IN BAND over the very bus both are testing
// (avrdx/src/apps/twi_link.hpp). The body is the one both Raspberry Pi
// chips share over the Synopsys DW_apb_i2c,
// rp2040/src/apps/rp_twi_peer.hpp, whose header says what this silicon
// makes of the protocol; this file binds it to the RP2350, ONE SOURCE FOR
// BOTH ARCHITECTURES (twi_link.hpp's one kind, 0x09, the label naming the
// half: "rp2350a" on the Cortex-M33 pair, "rp2350r" on the Hazard3 pair):
//
//  - THE BUS IS I2C0 ON GP36 (SDA) / GP37 (SCL) (9.4, function 3), above
//    GP31, the pair's I2C wires to the WeAct RP2040's GP2/GP3, each line
//    with 1.5 kOhm to THAT board's 3V3 (docs/boards/weact-rp2350b.md).
//  - THE PARTNER'S RAIL FIRST (pair_power.hpp): the client is brought up
//    only once GP36/GP37 read high through those pull-ups - an RP2040 pad
//    is not failsafe, and this image's pads carry the driver's pull-up -
//    and the rail is asked again once a second while it reads down.
//  - THE CLOCK is the 12 MHz crystal through the PLL to 150 MHz, so
//    Ident.xtal is 1 when that came up.
//  - ONE DRIVER PER WIRE: no other link pad is touched (GP4, GP5, GP20,
//    GP21, GP28, GP30, GP31, GP41 stay at reset, isolated).
//
// Console: UART0 on GP0/GP1 at 115200 - observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/i2c.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (rp_twi_peer.hpp's header).
#include "../../../avrdx/src/apps/twi_link.hpp"

#include "pair_power.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Console = Uart<0, console_pins>;
constexpr Console console;

constexpr I2cPins bus_pins{.scl = 37, .sda = 36};
using Client = I2cClient<0, bus_pins>;
using Host = I2cHost<0, bus_pins>;
using Raw = DwApbI2c<0>;
using SdaPin = Pin<36>;
using SclPin = Pin<37>;

uint32_t us_now() { return Timer<0>::now_low(); }

constexpr uint8_t peer_kind = twilink::peer_rp2350;
constexpr bool hazard3 = core_kind == CoreKind::hazard3;
constexpr char peer_label[8] = {'r', 'p', '2', '3', '5', '0', hazard3 ? 'r' : 'a', '\0'};
constexpr const char* bus_text = "I2C0 on GP36 (SDA) / GP37 (SCL)";
bool xtal_ok = false;

/// The partner's rail through the link's own pull-ups (pair_power.hpp):
/// the two pads released whatever it read.
bool link_pads_free() { return pair::partner_rail(clock).up(); }

// THE BODY, over the names above.
#include "../../../rp2040/src/apps/rp_twi_peer.hpp"
}  // namespace

extern "C" void isr_uart0() { (void)Console::isr(); }
extern "C" void isr_i2c0() { peer_vector(); }

int main() {
    xtal_ok = SysClock::init();
    const bool timer_ok = Timer<0>::init(clock);
    // The partner-rail sense paces itself by the platform timer
    // (rp2350/delay.hpp), which nothing else starts on the Cortex-M33 half:
    // started here, the two halves are one program.
    const bool ruler_ok = Mtime::start(clock);
    const bool console_ok = Console::init(clock, 115200);
    enable_interrupts();
    const bool rail = link_pads_free();
    const bool client_ok = rail && go_command();
    print(console, crlf, "twi_peer - I2C instrument client (RP2350 on the ", hazard3 ? "Hazard3" : "Cortex-M33",
          " half, label '", peer_label, "', fw ", hex(firmware_version), ")", crlf);
    print(console, "client ", bus_text, ", 1.5 kOhm to the RP2040's 3V3 on each; command address ",
          hex(twilink::command_addr), " exactly - no general call, no second address; ic_clk ",
          SysClock::hz / 1'000'000u, " MHz", crlf);
    print(console, "boot: clk=", xtal_ok ? "PLL150 on the crystal" : "FAILED", " timer0=", timer_ok ? "1us" : "FAILED",
          " ruler=", ruler_ok ? "1us" : "FAILED", " console=", console_ok ? "UART0" : "FAILED",
          " partner rail=", rail ? "up" : "DOWN", " client=", client_ok ? "up" : "OFF", crlf);
    if (!rail) {
        print(console, "THE PARTNER IS UNPOWERED: GP36/GP37 read low through their pull-ups to the RP2040's 3V3 - "
                       "the client stays off, asked again once a second",
              crlf);
    }
    help();
    print(console, "> ");
    peer_loop();
}
