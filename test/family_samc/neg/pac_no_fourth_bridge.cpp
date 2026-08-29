// mcu: samc21e18a samc21g18a samc21j18a
// The reserve probes the PAC's bridge count from the register map's own
// symbols (PAC_STATUSD_Msk). On every variant vendored here the answer
// is three; a four would mean the E/G/J header had grown the C21N's
// fourth bridge, which would need the driver's switch statements to
// grow with it.

#include "samc/pac.hpp"

using namespace brio;

static_assert(brio::Pac::bridge_count == 4,
              "this assertion is meant to FAIL: the E/G/J PAC has three "
              "peripheral bridges, not four");
