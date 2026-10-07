// guard: fail an indirect call
//
// A plain trampoline's body that calls through a pointer: the walk
// cannot know what the call reaches, so the vector guard must refuse
// the image rather than pass a body it has not read.
#include "ch32vx03/pfic.hpp"

void (*volatile hook)() = nullptr;

BRIO_CH32_VECTOR(tim7_handler) {
    if (hook != nullptr) {
        hook();
    }
    brio::Pfic::clear_pending(brio::Irq::tim7);
}

int main() {
    for (;;) {
    }
}
