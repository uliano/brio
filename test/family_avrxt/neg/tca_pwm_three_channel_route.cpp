// mcu: avr128db48 avr128da48
#include "avrxt/tca.hpp"
using namespace brio;
void f() { TcaPwm<1, 0x43>::init(); }
