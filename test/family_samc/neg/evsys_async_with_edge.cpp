// mcu: samc21j18a
// On the asynchronous path 29.6.2.6 says the edge detection "is not
// required and must be disabled by software". Asking for an edge there
// is a configuration the silicon cannot honour, so it must not compile.

#include "samc/evsys.hpp"

using namespace brio;

constexpr EventChannelConfig bad_cfg{
    .path = EventPath::asynchronous,
    .edge = EventEdge::rising,
};
static_assert(brio::Evsys::config_valid(bad_cfg),
              "this assertion is meant to FAIL: an asynchronous channel has "
              "no edge detector to ask");
