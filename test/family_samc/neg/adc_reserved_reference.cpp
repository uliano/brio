// mcu: samc21j18a
// REFCTRL.REFSEL (38.8.3) implements six codes; 0x6..0xF are Reserved.

#include "samc/adc.hpp"

using namespace brio;

constexpr AdcConfig bad_cfg{
    .reference = static_cast<Ref>(6),
};

void use() { (void)Adc<0>::init<bad_cfg>(0); }
