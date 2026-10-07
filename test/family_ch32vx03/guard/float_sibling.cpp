// guard: pass
//
// The shapes the vector guard must let through: float work under the
// float trampoline, which stores the twenty caller-saved f-registers
// itself; and a plain trampoline whose integer body calls out, tail
// jumps and dispatches through a switch's jump table, every entry of
// which lands inside the body.
#include <stdint.h>

#include "ch32vx03/pfic.hpp"

volatile float gain = 1.5f;
volatile float out = 0.0f;
volatile uint32_t code = 0;
volatile uint32_t level = 0;

[[gnu::noinline]] uint32_t step(uint32_t x) { return x * 3u + 1u; }

BRIO_CH32_VECTOR_FLOAT(tim6_handler) {
    out = gain * gain + out;
    brio::Pfic::clear_pending(brio::Irq::tim6);
}

BRIO_CH32_VECTOR(tim7_handler) {
    switch (code) {
    case 0: level = level + 3u; break;
    case 1: level = level ^ 0x55u; break;
    case 2: level = step(level); break;
    case 3: level = level << 2; break;
    case 4: level = level - 9u; break;
    case 5: level = level | 0x100u; break;
    case 6: level = level & 0xF0F0u; break;
    case 7: level = level >> 3; break;
    case 8: level = level * 7u; break;
    default: break;
    }
    brio::Pfic::clear_pending(brio::Irq::tim7);
}

int main() {
    for (;;) {
    }
}
