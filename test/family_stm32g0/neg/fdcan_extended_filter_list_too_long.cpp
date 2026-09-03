// mcu: stm32g0b1xx
// The extended list is eight elements of two words (36.3.12) and the
// same clamp applies to LSE.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = *brio::fdcan_bit_timing_for(64'000'000u, 500'000u),
        .extended_filters = 9}>();
}
