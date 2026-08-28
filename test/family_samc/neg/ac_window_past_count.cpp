// mcu: samc21e18a samc21g18a samc21j18a
// This family pairs its four comparators into exactly two windows
// (40.1). A third one does not exist and must not compile.

#include "samc/ac.hpp"

using namespace brio;

using Window2 = AcWindow<2>;
static_assert(Window2::index == 2,
              "this assertion is meant to FAIL: there are two windows, 0 and 1");
