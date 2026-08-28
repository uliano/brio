// mcu: samc21j18a
// CTRLA.SLAVEEN (38.8.1) exists only on the CLIENT of the pair - "This
// bit can be set only for the Client ADC (ADC1). For the Host ADC
// (ADC0), this bit is always read zero" - and the device header says the
// same thing as ADC0_MASTER_SLAVE_MODE = 1.

#include "samc/adc.hpp"

using namespace brio;

constexpr AdcConfig bad_cfg{
    .client_enable = true,
};

void use() { (void)Adc<0>::init<bad_cfg>(0); }
