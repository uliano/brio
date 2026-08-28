// mcu: samc21e18a samc21g18a samc21j18a
// 39.6.3.4's formula is (Data0 + OFFSETCORR) x GAINCORR / 2^SHIFTCORR,
// so a GAINCORR of zero multiplies EVERY result away. GAINCORR's reset
// value in this revision is 1 - which is erratum 1.18.3's workaround
// baked into the silicon, that item being about the revision where the
// reset value was zero.

#include "samc/sdadc.hpp"

using namespace brio;

constexpr SdadcConfig bad_cfg{
    .gain_correction = 0,
};

void use() { (void)Sdadc::init<bad_cfg>(0); }
