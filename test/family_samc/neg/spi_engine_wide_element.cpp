// mcu: samc21j18a
// The SPI host's Request is byte-oriented, so its engines must carry
// uint8_t elements: a halfword beat would move two Request bytes per
// character and the descriptor arithmetic would lie about both.

#include "samc/dmac.hpp"
#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads pads{
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

using Bad = SpiHost<0, pads, 0, DmaTxEngine<0, uint16_t>, DmaRxEngine<1, uint16_t>>;

void use() { (void)Bad::status(); }
