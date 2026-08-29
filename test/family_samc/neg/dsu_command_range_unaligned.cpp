// mcu: samc21j18a
// The DSU's shared command interface walks WORDS through the bus matrix
// and ADDR holds the address in bits 31:2, with AMOD in the low two
// (13.14.4). An odd start address is not slow, it is meaningless - the
// low bits would be read as an access mode.

#include "samc/dsu.hpp"

using namespace brio;

static_assert(brio::Dsu::range_valid(0x20000002UL, 16),
              "this assertion is meant to FAIL: a command range must be "
              "word aligned");
