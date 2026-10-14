// uart_peer - THE FAR END OF A SERIAL LINK, for a board that has no
// console of its own, on an RP2040. The body is the one both Raspberry Pi
// chips share over the ARM PL011, rp2040/src/apps/rp_uart_peer.hpp, whose
// header says what the two roles are and how the mode line picks one;
// this file binds it to the RP2040:
//
//  - THE LINK is UART1 on GP4 (TX) / GP5 (RX) at 115200 8N1, crossed to
//    the other board's UART (on the pair desk, the WeAct RP2350B's UART1 -
//    docs/boards/weact-rp2040.md).
//  - THE MODE LINE is GP20, an input pulled down, driven by the other
//    board (on the pair desk, the RP2350B's GP21). The LED (GP25) follows
//    it.
//  - NO GUARD: this board's pads are the ones that are not failsafe, and
//    the far board's are (RP2350 datasheet chapter 1), so driving the link
//    with the partner unpowered feeds nobody.
//
// build: boards = pico,picow,weact2040

#include <stdint.h>

#include <span>

#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins link_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using Link = Uart<1, link_pins, 512, 512>;

using ModeLine = Pin<20>;
using Led = Pin<25>;

void mode_line_up() { ModeLine::input(PinPull::down); }
bool mode_high() { return ModeLine::read(); }
uint32_t us_now() { return Timer::now_low(); }
/// This board's pads are the ones that are not failsafe: nothing to guard.
bool link_pads_free() { return true; }

// THE BODY, over the names above.
#include "rp_uart_peer.hpp"
}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart1() { (void)Link::isr(); }

int main() {
    (void)SysClock::init();
    (void)Timer::init(clock);
    peer_loop();
}
