// guard: pass vector tim7_handler (entry 71): its body, tim7_handler_body, is a leaf
//
// The leaf shapes the vector guard must let through, and its report: a
// vector bound LEAF that is one, dispatching through a switch's jump
// table inside itself, passes; a plain trampoline whose body came out a
// leaf passes too and is REPORTED as a candidate for the leaf form,
// naming the vector.
#include <stdint.h>

#include "ch32vx03/pfic.hpp"

volatile uint32_t code = 0;
volatile uint32_t level = 0;
volatile uint32_t hits = 0;

BRIO_CH32_LEAF_VECTOR(tim6_handler) {
    switch (code) {
    case 0: level = level + 3u; break;
    case 1: level = level ^ 0x55u; break;
    case 2: level = level * 5u; break;
    case 3: level = level << 2; break;
    case 4: level = level - 9u; break;
    case 5: level = level | 0x100u; break;
    case 6: level = level & 0xF0F0u; break;
    case 7: level = level >> 3; break;
    case 8: level = level * 7u; break;
    default: break;
    }
    brio::Pfic::clear_pending(brio::Irq::tim6);
}

BRIO_CH32_VECTOR(tim7_handler) {
    hits = hits + 1u;
    brio::Pfic::clear_pending(brio::Irq::tim7);
}

int main() {
    for (;;) {
    }
}
