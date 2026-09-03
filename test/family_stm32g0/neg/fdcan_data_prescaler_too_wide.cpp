// mcu: stm32g0b1xx
// DBRP is FIVE bits where NBRP is nine (36.4.3 against 36.4.7): the same
// FdcanBitTiming struct serves both phases and only the validator knows
// which fields it is being measured against.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = *brio::fdcan_bit_timing_for(64'000'000u, 500'000u),
        .data = brio::FdcanBitTiming{32, 10, 3, 3},
        .fd = brio::FdcanFd::on}>();
}
