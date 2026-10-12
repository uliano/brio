// mcu: avr128db48 avr128da28
// The AVR DA/DB SPI moves eight-bit characters (28.3): a client run of
// sixteen-bit elements has no frame to carry them and is refused.
#include "avrdx/spi.hpp"
using namespace brio;
uint16_t tx[4];
uint16_t rx[4];
void f() {
    (void)SpiClient<0>::exchange(std::span<const uint16_t>(tx), std::span<uint16_t>(rx), 100u);
}
