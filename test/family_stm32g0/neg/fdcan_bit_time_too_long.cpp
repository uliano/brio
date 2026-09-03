// mcu: stm32g0b1xx
// The other end of 36.4.7's window: NTSEG1 and NTSEG2 reach 256 + 128
// quanta between them, and a bit time of 82 tq is already illegal.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = brio::FdcanBitTiming{0, 59, 20, 3}}>();
}
