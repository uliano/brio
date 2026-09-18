// The third pad of a group carries that instance's TRANSMIT signal under
// function 11 and its CTS under function 2 (datasheet 9.4). Naming GP2
// under function 2 asks for the flow-control signal, so it must not
// compile as a transmit pad.
#include "rp2350/uart.hpp"
constexpr brio::UartPins wrong_column{.tx = {2, brio::PinFunction::uart},
                                      .rx = {3, brio::PinFunction::uart}};
using Bad = brio::Uart<0, wrong_column>;
void f() { (void)sizeof(Bad); }
