// mcu: avr128db48 avr128da48 avr128db32 avr128da64
// A client is clocked by the host: with no SCK, MOSI or SS pin there is
// nothing for a pinless client to do.
#include "avrdx/spi.hpp"
using namespace brio;
void f() { (void)Spi<0>::init<SpiConfig{.route = SpiRoute::none,
                                        .role = SpiRole::client}>(); }
