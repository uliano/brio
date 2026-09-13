// Two signals of one link on one pad is not a link: the second claim
// would take the pad away from the first, silently.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 5, PinFunction::af5}};
using Bus = SpiHost<1, pins>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
