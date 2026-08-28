// mcu: samc21e18a samc21g18a
// ADC1's AIN0 is PB00, which only the J bonds: on the E there is no PORT
// B analog pad at all and on the G the ADC1 rows start at PB02.

#include "samc/adc.hpp"

using namespace brio;

void use() { Adc<1>::select(AnalogIn<Pin<'B', 0>>{}); }
