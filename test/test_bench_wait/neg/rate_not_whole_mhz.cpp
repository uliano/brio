// A ruler whose rate is not a whole number of megahertz is refused: the
// conversion multiplies by the whole cycles in one microsecond, and a
// 1.5 MHz ruler has none to multiply by.
// refused: the ruler's rate must be a whole number of megahertz
#include <stdint.h>
#include "util/test_wait.hpp"

struct Ruler {
    static uint32_t now() { return 0; }
    static constexpr uint32_t hz() { return 1'500'000; }
};

void f() { brio::spin_us<Ruler>(10); }
