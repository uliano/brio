// mcu: samc21j18a
// The smallest MASK value, zero, already spans 16 bytes - two packets.
// There is no encoding for a smaller buffer.

#include "samc/mtb.hpp"

using namespace brio;

static_assert(brio::Mtb::geometry_valid(8),
              "this assertion is meant to FAIL: 16 bytes is the smallest "
              "trace buffer MASTER.MASK can describe");
