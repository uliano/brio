// mcu: avr128db48 avr128da48 avr128db64
// TCA1 -> PORTD is the AVR EA's route (DS40002443A 17.3.7): the AVR
// DA/DB's TCAROUTEA has no such code.
#include "avrxt/tca.hpp"
using namespace brio;
void f() { Tca<1>::init<TcaConfig{.mode = TcaMode::normal, .period = 1, .route = 'D'}>(); }
