// A constant budget longer than half the ruler's wrap is refused: 2^31
// cycles at 48 MHz is 44739242 us, and a budget past it could never be
// told from a counter that wrapped. (In an optimized build the same
// refusal comes from gcc's error attribute where the constant reaches a
// verb; a constant evaluation, as here, refuses it in any build.)
// refused: budget_past_half_the_wrap
#include <stdint.h>
#include "util/test_wait.hpp"

struct Ruler {
    static uint32_t now() { return 0; }
    static constexpr uint32_t hz() { return 48'000'000; }
};

constexpr uint32_t too_long = brio::us_cycles<Ruler>(44'739'243);
