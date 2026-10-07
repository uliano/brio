// guard: fail vector tim7_handler (entry 71) is bound by none of the stratum's macros
//
// A vector bound by the fast attribute written by hand, not through the
// stratum's macros: a leaf here, but nothing marks it as one, so nothing
// would hold it to being one - the next call added to it would save the
// twenty f-registers unseen. The vector guard must refuse the image and
// name the vector.
#include <stdint.h>

#include "ch32vx03/pfic.hpp"

volatile uint32_t hits = 0;

extern "C" [[gnu::interrupt("WCH-Interrupt-fast")]] void tim7_handler() {
    hits = hits + 1u;
    brio::Pfic::clear_pending(brio::Irq::tim7);
}

int main() {
    for (;;) {
    }
}
