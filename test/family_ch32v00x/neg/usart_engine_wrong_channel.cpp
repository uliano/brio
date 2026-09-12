// mcu: ch32v006k8 ch32v003f4
// Table 8-2: USART1 transmits on DMA channel 4 - a transmit engine on
// any other channel must be REFUSED.
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Wrong = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::DmaTxEngine<3, uint8_t>>;
void f() { (void)Wrong::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 9600); }
