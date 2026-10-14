// uart_peer - THE FAR END OF A SERIAL LINK, for a board that has no
// console of its own, on an RP2350, ONE SOURCE FOR BOTH ARCHITECTURES (it
// runs on the Cortex-M33 half by default). The body is the one both
// Raspberry Pi chips share over the ARM PL011,
// rp2040/src/apps/rp_uart_peer.hpp, whose header says what the two roles
// are and how the mode line picks one; this file binds it to the RP2350:
//
//  - THE LINK is UART1 on GP4 (TX) / GP5 (RX) under the first function
//    column, at 115200 8N1, crossed to the other board's UART (on the pair
//    desk, the WeAct RP2040's UART1 - docs/boards/weact-rp2350b.md).
//  - THE MODE LINE is GP20, an input pulled down, driven by the other
//    board (on the pair desk, the RP2040's GP21). The LED (GP25) follows
//    it. IT IS READ WITH ITS BUFFER PULSED (`Pin::read_pulsed()`, erratum
//    RP2350-E9's workaround, rp2350/pin.hpp): a pad whose buffer stays on
//    keeps the last level the far end drove once that end lets go, which
//    would leave this peer SENDING with nobody asking; with the buffer off
//    between reads the pull-down takes a released line low.
//  - THE PARTNER'S RAIL FIRST (pair_power.hpp): the transmitter idles high
//    on GP4, and an RP2040 pad is not failsafe, so UART1 comes up only once
//    the I2C link's pull-ups read that board powered, asked again once a
//    second while they read down.
//  - ONE DRIVER PER WIRE: no other link pad is touched (GP21, GP28, GP30,
//    GP31, GP36, GP37 between their sense reads, GP41 stay at reset).
//
// build: boards = weact2350b,weact2350b-rv

#include <stdint.h>

#include <span>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/uart.hpp"

#include "pair_power.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

constexpr UartPins link_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using Link = Uart<1, link_pins, 512, 512>;

using ModeLine = Pin<20>;
using Led = Pin<25>;

void mode_line_up() { (void)ModeLine::input({.pull = PinPull::down, .input_enable = false}); }
bool mode_high() { return ModeLine::read_pulsed(); }
uint32_t us_now() { return Mtime::micros(); }
/// The partner's rail through the I2C link's pull-ups (pair_power.hpp);
/// the two sense pads released whatever it read.
bool link_pads_free() { return pair::partner_rail(clock).up(); }

// THE BODY, over the names above.
#include "../../../rp2040/src/apps/rp_uart_peer.hpp"
}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart1() { (void)Link::isr(); }

int main() {
    (void)SysClock::init();
    (void)Mtime::start(clock);
    peer_loop();
}
