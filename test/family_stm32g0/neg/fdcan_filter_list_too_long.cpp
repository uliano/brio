// mcu: stm32g0b1xx
// 36.4.19 says values above 28 "are interpreted as 28" - the silicon
// CLAMPS. A program that asks for 31 standard filter elements has a bug
// and a clamp hides it, so the driver refuses instead.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = *brio::fdcan_bit_timing_for(64'000'000u, 500'000u),
        .standard_filters = 31}>();
}
