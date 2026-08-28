// mcu: samc21e18a samc21g18a samc21j18a
// 35.6.2.4: "TC0 is paired with TC1, TC2 is paired with TC3. TC4, TC5,
// TC6 and TC7 cannot be paired. When paired, the TC peripherals are
// configured using the registers of the even-numbered TC." TC1 is the
// CLIENT of a pair, never its master - the device header says so with
// TC1_MASTER_SLAVE_MODE = 2 - so asking TC1 for a 32-bit counter must
// not compile.

#include "samc/tc.hpp"

using namespace brio;

constexpr TcConfig bad_cfg{.mode = TcMode::count32};
static_assert(Tc<1>::config_valid(bad_cfg),
              "this assertion is meant to FAIL: TC1 is a pair CLIENT and "
              "cannot host a 32-bit counter");
