// mcu: stm32g0b1xx
// 36.3.4's note: "The restricted operation mode must not be combined
// with the loop-back mode (internal or external)." ASM is its own CCCR
// bit, so the illegal pairing is representable and has to be refused.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = *brio::fdcan_bit_timing_for(64'000'000u, 500'000u),
        .mode = brio::FdcanMode::internal_loop_back,
        .restricted = true}>();
}
