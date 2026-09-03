// mcu: stm32g0b1xx
// 36.4.3's note: "The data phase bit rate must be higher than or equal
// to the nominal bit rate". Both phases count the same time-quantum
// clock, so the comparison needs no frequency - and this configuration
// asks for a data phase four times slower than its arbitration phase.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = *brio::fdcan_data_timing_for(64'000'000u, 2'000'000u),
        .data = brio::FdcanBitTiming{3, 26, 3, 3},
        .fd = brio::FdcanFd::on_with_bit_rate_switch}>();
}
