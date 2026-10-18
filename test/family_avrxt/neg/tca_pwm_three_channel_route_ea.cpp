// mcu: avr64ea28 avr64ea32 avr64ea48
// TCA1's PORTA route on the AVR EA carries three channels (PA4..PA6):
// TcaPwm wants six.
#include "avrxt/tca.hpp"
using namespace brio;
void f() { TcaPwm<1, 'A'>::init(); }
