// twi_peer - a scriptable second chip on the bus for the far end of a
// two-board I2C test, on an RP2040: the far board is the DUT and the bus
// host, and it drives this one IN BAND over the very bus both are testing
// (avrxt/src/apps/twi_link.hpp). The body is the one both Raspberry Pi
// chips share over the Synopsys DW_apb_i2c, rp_twi_peer.hpp beside this
// file, whose header says what this silicon makes of the protocol; this
// file binds it to the RP2040:
//
//  - THE BUS IS I2C1 ON GP2 (SDA) / GP3 (SCL) (table 279, function 3), the
//    pair's I2C wires to the WeAct RP2350B's GP36/GP37, each line with
//    1.5 kOhm to THIS board's 3V3 (docs/boards/weact-rp2040.md). The pads
//    take the driver's pull-up, slew limit and Schmitt trigger.
//  - THE CLOCK is the 12 MHz crystal through the PLL to 125 MHz, so
//    Ident.xtal is 1 when that came up, and the protocol's microseconds
//    are the system timer's, counted off the crystal.
//  - NO GUARD: this board's pads are the ones that are not failsafe, and
//    the pull-ups hang from its own rail - nothing here can feed the
//    partner (an RP2350 pad is 3.3 V-failsafe unpowered).
//  - ONE DRIVER PER WIRE: no other link pad is touched (GP4, GP5, GP8,
//    GP10, GP11, GP20, GP21, GP29 stay at reset).
//
// Console: UART0 on GP0/GP1 at 115200 - the Debug Probe's bridge -
// observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (rp_twi_peer.hpp's header).
#include "../../../avrxt/src/apps/twi_link.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Console = Uart<0, console_pins>;
constexpr Console console;

constexpr I2cPins bus_pins{.scl = 3, .sda = 2};
using Client = I2cClient<1, bus_pins>;
using Host = I2cHost<1, bus_pins>;
using Raw = DwApbI2c<1>;
using SdaPin = Pin<2>;
using SclPin = Pin<3>;

uint32_t us_now() { return Timer::now_low(); }

constexpr uint8_t peer_kind = twilink::peer_rp2040;
constexpr char peer_label[8] = {'r', 'p', '2', '0', '4', '0', '\0', '\0'};
constexpr const char* bus_text = "I2C1 on GP2 (SDA) / GP3 (SCL)";
bool xtal_ok = false;

/// This board's pads are the ones that are not failsafe: nothing to guard.
bool link_pads_free() { return true; }

// THE BODY, over the names above.
#include "rp_twi_peer.hpp"
}  // namespace

extern "C" void isr_uart0() { (void)Console::isr(); }
extern "C" void isr_i2c1() { peer_vector(); }

int main() {
    xtal_ok = SysClock::init();
    const bool timer_ok = Timer::init(clock);
    const bool console_ok = Console::init(clock, 115200);
    enable_interrupts();
    const bool client_ok = go_command();
    print(console, crlf, "twi_peer - I2C instrument client (RP2040, label 'rp2040', fw ", hex(firmware_version), ")",
          crlf);
    print(console, "client ", bus_text, ", 1.5 kOhm to this board's 3V3 on each; command address ",
          hex(twilink::command_addr), " exactly - no general call, no second address; ic_clk ",
          SysClock::hz / 1'000'000u, " MHz", crlf);
    print(console, "boot: clk=", xtal_ok ? "PLL125 on the crystal" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
          " console=", console_ok ? "UART0" : "FAILED", " client=", client_ok ? "up" : "OFF", crlf);
    help();
    print(console, "> ");
    peer_loop();
}
