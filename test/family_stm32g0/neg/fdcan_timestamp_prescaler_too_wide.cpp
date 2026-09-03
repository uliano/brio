// mcu: stm32g0b1xx
// 36.4.8: TCP "configures the timestamp and timeout counters time unit
// in multiples of CAN bit times [1...16]" - and the register holds one
// less than the value, so 17 has nowhere to go.
#include "stm32g0/fdcan.hpp"
void f() {
    (void)brio::Fdcan<1>::enter<brio::FdcanConfig{
        .nominal = *brio::fdcan_bit_timing_for(64'000'000u, 500'000u),
        .timestamp_prescaler = 17}>();
}
