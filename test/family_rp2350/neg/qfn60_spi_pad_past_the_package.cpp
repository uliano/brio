// GP42/GP43/GP40 are SPI1's pads on the die and are bonded on the QFN-80
// alone: in a build that states the QFN-60, an SPI on them must not
// compile. The refusal is the pin chapter's own - the SPI table is the
// DIE's, and what the smaller package has not got is the bond wire.
#include "rp2350/spi.hpp"
constexpr brio::SpiPins high_pins{.sck = 42, .tx = 43, .rx = 40};
using Bad = brio::SpiHost<1, high_pins>;
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
void f() { (void)Bad::init(SysClock{}); }
