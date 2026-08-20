// mcu: avr128da48 avr128da28
// The DA has no XOSCHF: the HF crystal must be refused there.
#include "avrdx/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::crystal, 24'000'000>::init(); }
