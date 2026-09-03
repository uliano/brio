// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 14.4.5: with POLYSIZE under 32 bits "the least significant bits have
// to be used" - a 16-bit polynomial declared 8-bit would simply have its
// top half ignored, producing a checksum nobody asked for.
#include "stm32g0/crc.hpp"
constexpr brio::CrcConfig cfg{.polynomial = 0x1021u, .size = brio::CrcWidth::w8};
static_assert(brio::crc_config_valid(cfg), "must be refused");
