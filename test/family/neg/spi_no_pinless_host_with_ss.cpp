// mcu: avr128db48 avr128da48 avr128db28 avr128da64
// DA errata 2.10.1: with PORTMUX.SPIROUTE at NONE a host must disable
// Client Select (SSD = 1) or it does not stay a host. There is no pin
// to hold SS high there.
#include "avrdx/spi.hpp"
using namespace brio;
void f() { (void)Spi<0>::init<SpiConfig{.route = SpiRoute::none,
                                        .client_select_disable = false}>(); }
