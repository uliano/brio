// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// THE ONE-PIN-PER-LINE RULE. PA3 and PB3 are the same EXTI line and
// only one of them can own it; an application that declares its set of
// lines gets the collision refused instead of losing a pad silently.
#include "stm32g0/exti.hpp"
using namespace brio;
static_assert(exti_lines_distinct<ExtInt<Pin<'A', 3>>, ExtInt<Pin<'B', 3>>>(),
              "two pads on one EXTI line");
