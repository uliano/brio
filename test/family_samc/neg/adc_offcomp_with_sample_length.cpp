// mcu: samc21j18a
// 38.8.12, on SAMPCTRL.OFFCOMP: "This bit must be set to zero to
// validate the SAMPLEN value. It's not possible to use OFFCOMP=1 and
// SAMPLEN>0." Offset compensation fixes the sampling period at four
// cycles, so a SAMPLEN beside it is a request the silicon discards.

#include "samc/adc.hpp"

using namespace brio;

constexpr AdcConfig bad_cfg{
    .sample_length = 7,
    .offset_compensation = true,
};

void use() { (void)Adc<0>::init<bad_cfg>(0); }
