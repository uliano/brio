// mcu: samc21j18a
// STDBYCFG.VREGSMOD has three modes and a Reserved fourth (19.8.2):
// AUTO 0x0, PERFORMANCE 0x1, LP 0x2, Reserved 0x3. Asking for the
// Reserved one through the compile-time twin must not compile.

#include "samc/sleep.hpp"

using namespace brio;

constexpr StandbyConfig bad_cfg{.regulator = static_cast<VregStandbyMode>(3)};

bool configure_a_reserved_regulator() { return Pm::configure_standby<bad_cfg>(); }
