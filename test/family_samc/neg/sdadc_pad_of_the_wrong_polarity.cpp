// mcu: samc21e18a samc21g18a samc21j18a
// An SDADC input is a PAIR of pads and each pad has ONE polarity: PA06
// is AINN0 and PA07 is AINP0, never the other way round. The device
// header defines PIN_PA06B_SDADC_INN0 and no PIN_PA06B_SDADC_INP<k>.

#include "samc/sdadc.hpp"

using namespace brio;

void use() { Sdadc::claim_positive<Pin<'A', 6>>(); }
