// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// CTLR4 ON A CH32V203. The MARK and SPACE parity is USARTx_CTLR4's, and
// 18.10.8's note gives that register to the CH32F20x_D8, the CH32V30x and
// the CH32V31x alone - neither of the CH32V203's two classes. The probe
// and every verb behind it are refused at compile time there, before a
// program could write an address that is not a register.
#include "ch32vx03/usart.hpp"

using Second = brio::Usart<2>;
void f() { (void)Second::mark_space(brio::UsartMarkSpace::mark); }
