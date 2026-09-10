// mcu: ch32v006k8
// USART2 exists on the CH32V005/006/007 but this stratum implements
// USART1 alone until the AFIO remaps exist: the instance must be REFUSED
// at compile time, not half-built.
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Second = brio::Uart<2, brio::Ch32v00xPlatform<>>;
void f() { (void)Second::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 9600); }
