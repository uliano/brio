// guard: fail vector software_handler (entry 14) is bound leaf and is a trampoline
//
// A body bound with the scheduler's vector binding that calls a function:
// the binding marks the vector a leaf, and a naked vector that calls out
// under the hardware prologue would run the callee with the interrupted
// program's f-registers unsaved. The guard must refuse the image and name
// the vector.
#include <stdint.h>

#include "ch32vx03/schedule.hpp"

volatile uint32_t hits = 0;

extern "C" [[gnu::noinline]] void count_hit() { hits = hits + 1u; }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) {
    __asm__("call count_hit\n\t"
            "mret");
}

int main() {
    for (;;) {
    }
}
