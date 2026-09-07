// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A host with no MOSI pad has no way to say anything: SCK and MOSI are
// the two pads this engine cannot do without.
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {},
                             .nss = {}};
using Mute = brio::SpiHost<1, pins>;
void use() { (void)Mute::has_engines; }
