// mcu: samc21j18a
// Two engines on ONE DMA channel: a channel moves bytes one way, and the
// two directions would reprogram each other's descriptor.
#include "samc21/dmac.hpp"
#include "samc21/i2c.hpp"
using namespace brio;
constexpr I2cPads pads{.sda_pin = {'A', 8, PinFunction::c},
                       .scl_pin = {'A', 9, PinFunction::c}};
using H = I2cHost<0, pads, 0, DmaTxEngine<6>, DmaRxEngine<6>>;
bool f() { return H::isr(); }
