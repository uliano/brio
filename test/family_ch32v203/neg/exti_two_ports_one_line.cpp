// mcu: ch32v203f6 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// Two pads of the same PIN NUMBER are one EXTI line (10.2.3): PA1 and
// PB1 cannot both raise an edge, and an application that states its own
// set is told so at compile time. (The CH32V203F8 is not in the list
// because its package bonds no PB1 - there the refusal would be the
// pad's and not the line's.)
#include "ch32v203/exti.hpp"

using OnA = brio::ExtInt<brio::Pin<'A', 1>>;
using OnB = brio::ExtInt<brio::Pin<'B', 1>>;

static_assert(brio::exti_lines_distinct<OnA, OnB>(), "two pads on one EXTI line");
