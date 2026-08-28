// mcu: samc21e18a samc21g18a samc21j18a
// ERRATUM 1.21.10 (DS80000740S, E/G/J, every revision): "ALOCK feature is
// not functional", workaround "None". It is the one item in this chapter
// that can be turned into a compile error instead of a comment, and this
// is that compile error.

#include "samc/tcc.hpp"

using namespace brio;

static_assert(tcc_config_valid(0, TccConfig{.auto_lock = true}),
              "this assertion is meant to FAIL: erratum 1.21.10 makes ALOCK "
              "non-functional, so the driver never writes it");
