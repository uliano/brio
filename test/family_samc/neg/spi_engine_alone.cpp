// mcu: samc21j18a
// The engine slots are BOTH OR NEITHER: the data phase is full-duplex
// and its completion is the RECEIVE block's, so a TX engine alone has
// no edge to complete on, and an RX engine alone would race the byte
// pump for DATA.

#include "samc/dmac.hpp"
#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads pads{
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

using Bad = SpiHost<0, pads, 0, DmaTxEngine<0>, NoDmaEngine>;

void use() { (void)Bad::status(); }
