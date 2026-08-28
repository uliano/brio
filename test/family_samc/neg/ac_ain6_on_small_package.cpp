// mcu: samc21e18a samc21g18a
// 40.1: "The input selection includes four shared analog port pins (only
// two, AIN[5:4], for CMP2 and CMP3 on E and G variants)". AIN6 and AIN7
// are PB05 and PB06, bonded on the J alone - and the device header says
// so in symbols, which is what the driver reads. Asking COMP2 for its
// PIN2 (= AIN6) on a smaller package must not compile.

#include "samc/ac.hpp"

using namespace brio;

constexpr AcConfig bad_cfg{.positive = AcPositive::pin2};
static_assert(AcComparator<2>::config_valid(bad_cfg),
              "this assertion is meant to FAIL: COMP2's PIN2 is AIN6, which "
              "the E and G packages do not bond");
