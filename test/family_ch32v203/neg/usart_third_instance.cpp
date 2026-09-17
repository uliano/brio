// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6
// USART3 is on the two parts the datasheet gives four usarts and nowhere
// else, so the instance is refused on the seven below them even where
// the package bonds its pads: the CH32V203C6 brings out PB10 and PB11
// and has no USART3 behind them.
#include "ch32v203/usart.hpp"

using Third = brio::Usart<3>;
void f() { Third::bus_clock(true); }
