// guard: fail scale
//
// A plain trampoline's body that reaches float work through a call: the
// body itself is integer code, the function it calls converts and
// multiplies in f-registers, and nothing saves the interrupted
// program's. The vector guard must refuse the image and name the
// function.
#include <stdint.h>

#include "ch32vx03/pfic.hpp"

volatile float gain = 1.5f;
volatile uint32_t level = 7;

[[gnu::noinline]] uint32_t scale(uint32_t x) {
    return static_cast<uint32_t>(static_cast<float>(x) * gain);
}

BRIO_CH32_VECTOR(tim6_handler) {
    level = scale(level);
    brio::Pfic::clear_pending(brio::Irq::tim6);
}

int main() {
    for (;;) {
    }
}
