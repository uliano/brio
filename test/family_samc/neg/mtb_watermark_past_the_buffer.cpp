// mcu: samc21j18a
// FLOW.WATERMARK is an offset inside the trace buffer: a watermark past
// its end is a stop point the write pointer reaches only after wrapping
// over the packets it was meant to preserve.

#include "samc/mtb.hpp"

using namespace brio;

static_assert(brio::Mtb::geometry_valid(1024, 129),
              "this assertion is meant to FAIL: a 1024-byte buffer holds "
              "128 packets and the watermark must be one of them");
