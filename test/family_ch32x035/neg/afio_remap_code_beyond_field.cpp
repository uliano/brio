// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// USART1's remap field is two bits (RM 8.3.2.1): a code of 4 is no column
// and is refused where it is a constant.
#include "ch32x035/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::usart1, 4>(); }
