// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// An element IS the bus access width, and CCR's PSIZE/MSIZE have exactly
// three codes: a three-byte or eight-byte element is not a narrower
// transfer, it is no transfer at all.
#include <stdint.h>
#include "stm32g0/dma.hpp"
struct Triple { uint8_t a, b, c; };
using Wrong = brio::DmaPingPongEngine<1, 1, Triple>;
static_assert(Wrong::channel == 1);
static constexpr brio::DmaWidth w = brio::DmaPingPongEngine<1, 1, Triple>::width;
static_assert(w == brio::DmaWidth::byte);
