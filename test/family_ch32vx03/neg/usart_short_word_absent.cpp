// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A FIVE-BIT WORD ON A CH32V203. CTLR1's M_EXT carries the same note as
// CTLR4 (18.10.4): the CH32F20x_D8's, the CH32V30x's and the CH32V31x's,
// and on a CH32V203 bits 15:14 of CTLR1 are no field at all. Refused at
// compile time.
#include "ch32vx03/usart.hpp"

using Second = brio::Usart<2>;
void f() { (void)Second::short_word(brio::UsartShortWord::five); }
