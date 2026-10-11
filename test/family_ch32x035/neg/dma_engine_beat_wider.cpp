// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// An engine's Elem is the WIDEST beat its binding moves (the peripheral
// register's width): a byte-wide transmit engine refuses a run of
// half-words.
#include <span>

#include "ch32x035/dma.hpp"

void f() {
    static const uint16_t run[4] = {};
    (void)brio::DmaTxEngine<3, uint8_t>::start(std::span<const uint16_t>(run));
}
