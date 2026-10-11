// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// THE DRIVEN SINGLE WIRE IS A SINGLE WIRE'S. UartOptions::
// single_wire_push_pull drives the ONE pad of a half-duplex line (RM0444
// 33.5.15); on a full-duplex link the TX pad is push-pull already and the
// option names nothing - refused rather than ignored.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 9, brio::PinFunction::af1},
                           .rx = {'A', 10, brio::PinFunction::af1}};
constexpr brio::UartOptions driven{.single_wire_push_pull = true};
using Full = brio::Uart<1, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, driven>;
void f() { (void)Full::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 115200); }
