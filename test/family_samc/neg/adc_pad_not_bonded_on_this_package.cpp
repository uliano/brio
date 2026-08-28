// mcu: samc21e18a
// The E variant bonds NO PORT B pad to either converter, so ADC0's AIN2
// (PB08) does not exist there. The J and the G both have it, which is
// why this negative names the E alone.

#include "samc/adc.hpp"

using namespace brio;

void use() { Adc<0>::select(AnalogIn<Pin<'B', 8>>{}); }
