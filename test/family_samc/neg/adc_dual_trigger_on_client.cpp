// mcu: samc21j18a
// CTRLC.DUALSEL (38.8.10) is the HOST's knob: "These bits are available
// in the Host ADC and have no effect if the Host-Client operation is
// disabled". Asking the client to choose the pair's trigger mode is a
// write into a field that does nothing.

#include "samc/adc.hpp"

using namespace brio;

constexpr AdcConfig bad_cfg{
    .dual = AdcDual::interleave,
};

void use() { (void)Adc<1>::init<bad_cfg>(0); }
