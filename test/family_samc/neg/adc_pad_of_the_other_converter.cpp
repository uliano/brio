// mcu: samc21e18a samc21g18a samc21j18a
// The two converters have DIFFERENT MAPS over overlapping pads: PA04 is
// ADC0/AIN4 on every package and reaches ADC1 on none of them. Handing
// it to ADC1 must not compile - on any variant, because this one is not
// about bonding but about which converter the pad goes to.

#include "samc/adc.hpp"

using namespace brio;

void use() { Adc<1>::select(AnalogIn<Pin<'A', 4>>{}); }
