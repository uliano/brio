// mcu: avr128db48 avr128da28
// The PIT divides a 32.768 kHz clock by powers of two: 2048 ticks/s is
// not a period the hardware has (and 1024 Hz is the fastest brio ticks).
#include "avrdx/ticker.hpp"
using namespace brio;
void f() { BasicTicker<2048>::init(); }
