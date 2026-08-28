// mcu: samc21j18a
// CFGA.REFNUM must be non-zero (44.8.3), and the measurement's own
// arithmetic divides by it. A configuration that asks for zero must not
// reach the silicon - here it is caught in a constant expression.

#include "samc/freqm.hpp"

using namespace brio;

constexpr FreqmConfig bad_cfg{.refnum = 0};
static_assert(brio::Freqm::config_valid(bad_cfg),
              "this assertion is meant to FAIL: REFNUM 0 is not a valid "
              "configuration");
