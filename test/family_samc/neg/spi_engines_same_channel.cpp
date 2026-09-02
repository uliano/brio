// mcu: samc21j18a
// The two engines of one SpiHost must ride two different DMA channels:
// a channel moves bytes ONE way, and pointing both directions at it
// would have each re-programming the other's descriptor.

#include "samc/dmac.hpp"
#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads pads{
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

using Bad = SpiHost<0, pads, 0, DmaTxEngine<3>, DmaRxEngine<3>>;

void use() { (void)Bad::status(); }
