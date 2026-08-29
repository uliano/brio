// mcu: samc21j18a
// MASTER.MASK holds log2(bytes) - 4, so the trace buffer's size is a
// power of two or it is nothing. A size of 1000 bytes has no MASK value
// and, worse, would silently trace into a region of a different size.

#include "samc/mtb.hpp"

using namespace brio;

static_assert(brio::Mtb::geometry_valid(1000),
              "this assertion is meant to FAIL: the trace buffer's size "
              "must be a power of two");
