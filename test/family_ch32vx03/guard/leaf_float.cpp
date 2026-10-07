// guard: fail vector tim6_handler (entry 70) is bound leaf and is none
//
// A vector bound LEAF that calls nothing but does float work: the
// attributed handler saves the f-registers it uses itself, which is
// correct and is not the leaf the binding promises (a float body binds
// with BRIO_CH32_VECTOR_FLOAT), so the vector guard must refuse it.
#include <stdint.h>

#include "ch32vx03/pfic.hpp"

volatile float gain = 1.5f;
volatile float out = 0.0f;

BRIO_CH32_LEAF_VECTOR(tim6_handler) {
    out = gain * gain + out;
    brio::Pfic::clear_pending(brio::Irq::tim6);
}

int main() {
    for (;;) {
    }
}
