// guard: fail vector tim7_handler (entry 71) is bound leaf and is none
//
// A vector bound LEAF whose braces call a function the compiler may not
// inline: bound by the attribute, the handler then saves the twenty
// caller-saved f-registers around the call - the cost the trampoline
// exists to spare - so the vector guard must refuse the image and name
// the vector.
#include <stdint.h>

#include "ch32vx03/pfic.hpp"

volatile uint32_t level = 7;

[[gnu::noinline]] void step() { level = level * 3u + 1u; }

BRIO_CH32_LEAF_VECTOR(tim7_handler) {
    step();
    brio::Pfic::clear_pending(brio::Irq::tim7);
}

int main() {
    for (;;) {
    }
}
