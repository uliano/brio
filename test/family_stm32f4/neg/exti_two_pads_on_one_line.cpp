// One line per PIN NUMBER, shared by every port: an application that
// declares its own set of lines is told at compile time when two of them
// are the same line (Exti::select() refuses the second claim at run time,
// but this is the answer that arrives before the board is powered).
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/exti.hpp"
using Left = brio::ExtInt<brio::Pin<'A', 5>>;
using Right = brio::ExtInt<brio::Pin<'B', 5>>;
static_assert(brio::exti_lines_distinct<Left, Right>(), "PA5 and PB5 are both line 5");
