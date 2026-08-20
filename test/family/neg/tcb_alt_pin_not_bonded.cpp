// mcu: avr128db28
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { Tcb<2>::init<TcbConfig{.mode = TcbMode::pwm8, .compare = 0x80FF, .output = true, .alt_pin = true}>(); }
