// mcu: avr64ea48 avr64ea32
// A crystal at a rate the fuse cannot have: the fuse's rate must be
// named beside it (the fallback's claim).
#include "avrdx/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::crystal, 8'000'000>::init(); }
