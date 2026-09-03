// mcu: stm32g0b1xx
// 36.4.7: "The CAN bit time can be programed in the range of 4 to
// 81 x tq". A timing of 1 + 1 + 1 quanta fits every field of NBTP and is
// still outside the chapter's window, which is why the driver checks the
// window and not only the fields.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = brio::FdcanBitTiming{0, 0, 0, 0}}>();
}
