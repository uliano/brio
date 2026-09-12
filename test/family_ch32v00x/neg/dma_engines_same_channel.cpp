// mcu: ch32v006k8 ch32v003f4
// A channel moves data ONE way: a Uart whose two engines name the same
// channel must be REFUSED at the application's line.
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Bad = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::DmaTxEngine<4>, brio::DmaRxEngine<4>>;
void f() { (void)Bad::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 9600); }
