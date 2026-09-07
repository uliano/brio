// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The data phase is full-duplex and its completion is the RECEIVE
// block's, so a transmit engine alone has no edge to complete on: name
// both slots or neither.
#include "stm32g0/dma.hpp"
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {}};
using Half = brio::SpiHost<1, pins, brio::DmaTxEngine<1, 1>, brio::NoDmaEngine>;
void use() { (void)Half::has_engines; }
