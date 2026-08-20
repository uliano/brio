// mcu: avr128db48
// Same for the meters: hz()/us() would convert with the old rate.
#include "avrdx/clock.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot>;
void f() { FrequencyMeter<Tcb<0>>::init(Dyn{}, EventChannel<2>{}); }
