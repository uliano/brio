// mcu: samc21j18a
// A zero-word CRC32 or MBIST is a command with nothing to do that would
// still set STATUSA.DONE and hand back a checksum of nothing.

#include "samc/dsu.hpp"

using namespace brio;

static_assert(brio::Dsu::range_valid(0x20000000UL, 0),
              "this assertion is meant to FAIL: a command range must "
              "contain at least one word");
