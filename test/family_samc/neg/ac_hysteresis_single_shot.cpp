// mcu: samc21e18a samc21g18a samc21j18a
// 40.6.6: "Hysteresis is available only in continuous mode
// (COMPCTRLx.SINGLE=0)." Asking for both is asking the silicon for
// something it does not implement, so it must not compile.

#include "samc/ac.hpp"

using namespace brio;

constexpr AcConfig bad_cfg{.single_shot = true, .hysteresis = true};
static_assert(ac_config_valid(0, bad_cfg),
              "this assertion is meant to FAIL: hysteresis is continuous-mode "
              "only");
