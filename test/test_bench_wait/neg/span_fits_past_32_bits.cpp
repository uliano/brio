// A counter wider than 32 bits is no counter span_fits() reasons about:
// every budget here is a 32-bit count.
// refused: a counter of 2 to 32 bits
#include <stdint.h>
#include "util/test_wait.hpp"

static_assert(brio::span_fits<40>(1000));
