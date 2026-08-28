// mcu: samc21e18a samc21g18a samc21j18a
// TCC2 has two compare/capture channels (TCC2_CC_NUM), so channel 2 does
// not exist and a PWM task on it must not compile.

#include "samc/tcc.hpp"

using namespace brio;

using Pwm = TccPwm<Tcc<2>, 2, 255>;
static_assert(Pwm::max == 255,
              "this assertion is meant to FAIL: TCC2 has two channels");
