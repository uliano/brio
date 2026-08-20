// mcu: avr128db28 avr128da28
#include "avrdx/tca.hpp"
using namespace brio;
void f() { Tca<0>::init<TcaConfig{.mode = TcaMode::normal, .period = 1, .route = 0x42}>(); }
