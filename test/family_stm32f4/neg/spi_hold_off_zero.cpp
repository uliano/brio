// A host's hold-off is the longest the image keeps its vector waiting,
// and no image has none: the tick's handler is there even where nothing
// else is, so a declared 0 is refused.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using Bus = SpiHost<1, pins, NoDmaEngine, NoDmaEngine, 0>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
