// mcu: stm32g0b1xx stm32g031xx
// The host's placement is util/placement.hpp's InFlash or InRam and
// nothing else: a tag of the program's own invention is refused where the
// host is named, never taken for one of the two.
#include "stm32g0/spi.hpp"
struct InTheCloud {};
constexpr brio::SpiPins pins{
    .sck = {'B', 3, brio::PinFunction::af0},
    .miso = {'B', 4, brio::PinFunction::af0},
    .mosi = {'B', 5, brio::PinFunction::af0},
    .nss = {},
};
using H = brio::SpiHost<1, pins, brio::NoDmaEngine, brio::NoDmaEngine, InTheCloud>;
void f();
void f() { (void)H::status(); }
