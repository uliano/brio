// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE RECEIVE ENGINE'S LEVEL IS A RECEIVE ENGINE'S. UartOptions::rx_priority
// ranks the channel a receive ring runs on (docs/design/dma.md); on a port
// with no receive engine there is no channel to rank, and a level named
// there must be REFUSED rather than ignored.
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

inline constexpr brio::UartOptions low_ring{.rx_priority = brio::DmaPriority::low};
using Plain = brio::Uart<2, brio::Ch32vx03Platform<>, 64, 64, brio::UartFormat{},
                         brio::NoDmaEngine, brio::NoDmaEngine, 0, low_ring>;
void f() { (void)Plain::baud(); }
