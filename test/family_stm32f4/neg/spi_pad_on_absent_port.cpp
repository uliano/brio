// Port F is bonded on the 144-pin classes; the F411's line has A, B, C,
// D, E and H, so SPI5's pads there are refused rather than configuring a
// port the device header does not declare.
// mcu: stm32f411xe
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'F', 7, PinFunction::af5},
                       .miso = {'F', 8, PinFunction::af5},
                       .mosi = {'F', 9, PinFunction::af5}};
using Bus = SpiHost<5, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
