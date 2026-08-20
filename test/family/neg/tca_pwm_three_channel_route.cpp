// mcu: avr128db48 avr128da48
#include "avrdx/tca.hpp"
using namespace brio;
void f() { TcaPwm<1, 0x43>::init(); }
