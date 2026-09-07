// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A client must be able to RECEIVE: MOSI is the line the host talks on,
// and a client that cannot hear it has nothing to answer.
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 10, brio::PinFunction::af5},
                             .miso = {'C', 2, brio::PinFunction::af1},
                             .mosi = {},
                             .nss = {'B', 12, brio::PinFunction::af0}};
using Deaf = brio::SpiClient<2, pins>;
void use() { (void)Deaf::has_nss_pad; }
