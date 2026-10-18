// mcu: avr64ea28 avr64ea32
// TCA1 exists on every AVR EA, but its PORTC route needs the 48-pin part.
#include "avrxt/tca.hpp"
using namespace brio;
void f() { Tca<1>::init<TcaConfig{.mode = TcaMode::normal, .period = 0xFFFF, .route = 'C'}>(); }
