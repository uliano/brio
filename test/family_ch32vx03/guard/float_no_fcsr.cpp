// guard: fail is a float trampoline that stores the twenty f-registers but does not save and restore fcsr
//
// A float trampoline written by hand in the shape BRIO_CH32_VECTOR_FLOAT
// had before it saved fcsr: the twenty caller-saved f-registers stored
// and loaded around the call, fcsr left to whatever the body does to it.
// A body that raises a flag or sets a rounding mode leaves it so for the
// program it interrupted, so the vector guard must refuse it.
#include <stdint.h>

volatile float gain = 1.5f;
volatile float out = 0.0f;

extern "C" void tim6_handler_body() { out = gain * gain + out; }

extern "C" [[gnu::naked]] void tim6_handler() {
    __asm__("addi sp, sp, -80\n\t"
            "fsw ft0, 0(sp)\n\t"  "fsw ft1, 4(sp)\n\t"  "fsw ft2, 8(sp)\n\t"
            "fsw ft3, 12(sp)\n\t" "fsw ft4, 16(sp)\n\t" "fsw ft5, 20(sp)\n\t"
            "fsw ft6, 24(sp)\n\t" "fsw ft7, 28(sp)\n\t" "fsw fa0, 32(sp)\n\t"
            "fsw fa1, 36(sp)\n\t" "fsw fa2, 40(sp)\n\t" "fsw fa3, 44(sp)\n\t"
            "fsw fa4, 48(sp)\n\t" "fsw fa5, 52(sp)\n\t" "fsw fa6, 56(sp)\n\t"
            "fsw fa7, 60(sp)\n\t" "fsw ft8, 64(sp)\n\t" "fsw ft9, 68(sp)\n\t"
            "fsw ft10, 72(sp)\n\t" "fsw ft11, 76(sp)\n\t"
            "call tim6_handler_body\n\t"
            "flw ft0, 0(sp)\n\t"  "flw ft1, 4(sp)\n\t"  "flw ft2, 8(sp)\n\t"
            "flw ft3, 12(sp)\n\t" "flw ft4, 16(sp)\n\t" "flw ft5, 20(sp)\n\t"
            "flw ft6, 24(sp)\n\t" "flw ft7, 28(sp)\n\t" "flw fa0, 32(sp)\n\t"
            "flw fa1, 36(sp)\n\t" "flw fa2, 40(sp)\n\t" "flw fa3, 44(sp)\n\t"
            "flw fa4, 48(sp)\n\t" "flw fa5, 52(sp)\n\t" "flw fa6, 56(sp)\n\t"
            "flw fa7, 60(sp)\n\t" "flw ft8, 64(sp)\n\t" "flw ft9, 68(sp)\n\t"
            "flw ft10, 72(sp)\n\t" "flw ft11, 76(sp)\n\t"
            "addi sp, sp, 80\n\t"
            "mret");
}

int main() {
    for (;;) {
    }
}
