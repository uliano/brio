// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Table 212's DLC codes 0..8 and then 12/16/20/24/32/48/64 bytes: there
// is no thirteen-byte CAN frame, so the encoder has no word to build and
// hands back an empty optional - which dereferencing in a constant
// expression is an error.
#include "stm32g0/fdcan.hpp"
constexpr auto words =
    *brio::fdcan_encode_tx(brio::FdcanFrame{.id = 1, .length = 13, .fd = true});
