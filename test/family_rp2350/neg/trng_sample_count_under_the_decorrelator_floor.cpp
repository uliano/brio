// With the von Neumann decorrelator bypassed, SAMPLE_CNT1 may not be
// below seventeen.
#include "rp2350/trng.hpp"
void f() {
    (void)brio::Trng::init<brio::TrngConfig{.sample_cycles = 16, .von_neumann = false}>();
}
