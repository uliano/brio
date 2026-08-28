// mcu: samc21j18a
// THE CHAPTER'S CLASSIC TRAP (38.6.2.9, 38.6.2.10 and 38.8.11 all say
// it): accumulating or averaging more than one sample REQUIRES
// CTRLC.RESSEL = 16BIT. Asking for 16 samples at 12-bit resolution
// produces a RESULT the register cannot hold and arithmetic nothing can
// interpret, so it must not compile.

#include "samc/adc.hpp"

using namespace brio;

constexpr AdcConfig bad_cfg{
    .resolution = AdcRes::bits12,
    .average = AdcAverage::samples16,
    .adjust = 4,
};

void use() { (void)Adc<0>::init<bad_cfg>(0); }
