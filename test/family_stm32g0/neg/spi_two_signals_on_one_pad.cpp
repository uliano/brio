// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Two of the four signals on the same pad is a wiring mistake no
// datasheet table can catch, and the one thing about a pad list this
// stratum CAN check.
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 4, brio::PinFunction::af0},
                             .nss = {}};
using Clash = brio::SpiHost<1, pins>;
void use() { (void)Clash::has_engines; }
