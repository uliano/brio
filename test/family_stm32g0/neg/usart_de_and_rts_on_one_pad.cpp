// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// DE and RTS are the SAME pad and the same driver (RM0444 33.5.20): an
// instance does one or the other, never both.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 9, brio::PinFunction::af1},
                           .rx = {'A', 10, brio::PinFunction::af1}};
constexpr brio::PinSel de{'A', 12, brio::PinFunction::af1};
constexpr brio::UartOptions o{.driver_enable = true, .de_pin = de, .rts = true,
                              .rts_pin = de};
using Bad = brio::Uart<1, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, o>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 9600); }
