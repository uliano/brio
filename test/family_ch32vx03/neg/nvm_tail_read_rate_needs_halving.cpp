// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303rc ch32v303vc
// A READ OF THE TAIL AT A RATE THAT WANTS HCLK HALVED. RM 32.1's note 2
// names the non-zero-wait area among the flash operations that want the
// tree divided above 120 MHz, and the tail is read out of the array at
// the access clock - 72 MHz at 144 with SCKMOD at its reset value, over
// the 60 the chapter allows. The window reads at any rate; the tail's
// read takes the clock and refuses this one, at compile time under a
// static clock.
#include "ch32vx03/nvm.hpp"

using Full = brio::Clock<brio::ClockSource::pll, 144'000'000>;

static uint8_t buffer[16];

void f() {
    (void)brio::Flash::read_tail(Full{}, brio::Flash::tail_base,
                                 std::span<uint8_t>(buffer, sizeof buffer));
}
