// GP32/GP33 are UART0's pads on the die and are bonded on the QFN-80
// alone: in a build that states the QFN-60, a UART on them must not
// compile. The refusal is the pin chapter's own - the UART table is the
// DIE's, and what the smaller package has not got is the bond wire.
#include "rp2350/uart.hpp"
constexpr brio::UartPins high_pins{.tx = {32, brio::PinFunction::uart},
                                   .rx = {33, brio::PinFunction::uart}};
using Bad = brio::Uart<0, high_pins>;
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
void f() { (void)Bad::init(SysClock{}, 115200); }
