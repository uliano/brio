// mcu: ch32v006k8 ch32v003f4
// InRam on a port with no transmit engine: the placement moves the engine's
// block restart and nothing else, so there is nothing to place - REFUSED,
// never a tag that silently does nothing (the interrupt body is the
// vector's: a program places the vector).
#include "ch32v00x/clock.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Port = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::NoDmaEngine, brio::NoDmaEngine, 0,
                        brio::UartOptions{}, brio::InRam>;
void f() { (void)Port::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 115200); }
