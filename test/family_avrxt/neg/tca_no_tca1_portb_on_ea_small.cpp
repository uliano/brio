// mcu: avr64ea28 avr64ea32
// The 28/32-pin AVR EA has TCA1 but no PORTB: its header keeps TCA1's
// PORTB code as "not connected to any pins", refused as a route.
#include "avrxt/tca.hpp"
using namespace brio;
void f() { Tca<1>::init<TcaConfig{.mode = TcaMode::normal, .period = 1, .route = 'B'}>(); }
