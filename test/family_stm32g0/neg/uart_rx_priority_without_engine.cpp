// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// THE RECEIVE ENGINE'S LEVEL IS A RECEIVE ENGINE'S. UartOptions::rx_priority
// ranks the channel a receive ring runs on (docs/design/dma.md); on a port
// with no receive engine there is no channel to rank, and a level named
// there must be REFUSED rather than ignored.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins p{.tx = {'A', 2, brio::PinFunction::af1},
                           .rx = {'A', 3, brio::PinFunction::af1}};
constexpr brio::UartOptions low_ring{.rx_priority = brio::DmaPriority::low};
using Plain = brio::Uart<2, p, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, low_ring>;
void f() { (void)Plain::init(brio::Clock<brio::ClockSource::pll, 64'000'000>{}, 115200); }
