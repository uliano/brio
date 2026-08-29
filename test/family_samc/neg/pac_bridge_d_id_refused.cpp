// mcu: samc21e18a samc21g18a samc21j18a
// PERID = 32 x bridge + index (11.7.1), and this device's PAC register
// map stops at bridge C: there is no STATUSD and no INTFLAGD. An
// identifier of 96 or more names the fourth bridge, which only the C21N
// has - accepting it would aim a protection request at a register that
// is not there.

#include "samc/pac.hpp"

using namespace brio;

static_assert(brio::Pac::id_valid(96),
              "this assertion is meant to FAIL: bridge D belongs to the "
              "C21N and this device's PAC has three bridges");
