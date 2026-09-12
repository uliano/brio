// mcu: ch32v006k8 ch32v003f4
// USART2's default column puts TX on PA7, the CH32V006K8's reset pin:
// the instance at remap code 0 must be REFUSED on this part (codes 1..6
// are its).
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Second = brio::Uart<2, brio::Ch32v00xPlatform<>>;
void f() { (void)Second::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 9600); }
