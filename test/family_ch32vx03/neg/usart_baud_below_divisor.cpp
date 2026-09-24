// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A RATE THE GENERATOR CANNOT SERVE: BRR counts peripheral clocks per
// SIXTEENTH of a bit, so 8 Mbaud from a PB1 at its 72 MHz ceiling asks
// for a divisor of 9 - below the sixteen the chapter's generator must
// divide by. init() answers false at run time, because a clock can be
// dynamic; a program whose rate is fixed guards it at compile time,
// which is the line below and which must NOT hold here.
#include "ch32vx03/usart.hpp"

static_assert(brio::usart_divisor_valid(brio::usart_divisor(72'000'000UL, 8'000'000UL)),
              "8 Mbaud is out of the reach of a 72 MHz peripheral clock");
void f() {}
