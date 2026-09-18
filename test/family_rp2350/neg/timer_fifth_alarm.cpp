// Each system timer has four alarms, 0..3, and the index is a build
// fact: what claims one is the app binding its vector by name.
#include "rp2350/timer.hpp"
void f() { brio::Timer<0>::alarm_in<4>(1000u); }
