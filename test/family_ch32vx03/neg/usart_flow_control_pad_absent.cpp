// mcu: ch32v203f8
// THE SIGNAL EXISTS AND THE PIN DOES NOT. USART1 is a full USART on
// every part that has it, and its default column puts CTS on PA11 and
// RTS on PA12 - two pads this twenty-pin package does not bond. The
// pair is refused by the BONDING and not by the instance.
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

inline constexpr brio::UartOptions pair{.rts = true, .cts = true};
using Flow = brio::Uart<1, brio::Ch32vx03Platform<>, 64, 64, brio::UartFormat{},
                        brio::NoDmaEngine, brio::NoDmaEngine, 0, pair>;
void f() { (void)Flow::baud(); }
