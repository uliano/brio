// mcu: avr128db48 avr128da48
#include "avrdx/tca.hpp"
using namespace brio;
void f() { Tca<0>::init<TcaConfig{.mode = TcaMode::normal, .period = 1, .route = 0x47}>(); }
