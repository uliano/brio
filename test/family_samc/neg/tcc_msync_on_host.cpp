// mcu: samc21e18a samc21g18a samc21j18a
// 36.8.1: CTRLA.MSYNC is "only for TCC Client instance", and
// TCCn_MASTER_SLAVE_MODE says which instance that is - TCC1 here. Asking
// the HOST to synchronize on itself must be refused.

#include "samc/tcc.hpp"

using namespace brio;

static_assert(tcc_config_valid(0, TccConfig{.host_sync = true}),
              "this assertion is meant to FAIL: TCC0 is the pair host, not "
              "its client");
