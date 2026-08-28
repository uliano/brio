// mcu: samc21e18a samc21g18a samc21j18a
// 26.8.10's own note on FILTENx: "The filter must be disabled if the
// asynchronous detection is enabled." The filter needs the EIC clock to
// sample with, and asynchronous detection is the mode that has none, so
// the two cannot both be asked for.

#include "samc/eic.hpp"

using namespace brio;

constexpr EicLineConfig bad_cfg{
    .sense = EicSense::rising,
    .filter = true,
    .asynchronous = true,
};
static_assert(eic_line_config_valid(bad_cfg),
              "this assertion is meant to FAIL: an asynchronously detected "
              "line has no clock for the filter to sample with");
