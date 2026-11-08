// A ruler whose hz() is not a constant expression is refused: the
// microseconds of a budget convert to cycles at compile time, and a rate
// read at run time would put a division in the image.
// refused: the ruler's hz.. must be a constant expression
#include <stdint.h>
#include "util/test_wait.hpp"

struct Ruler {
    static uint32_t now() { return 0; }
    static uint32_t hz() { return 48'000'000; }
};

void f() { brio::spin_us<Ruler>(10); }
