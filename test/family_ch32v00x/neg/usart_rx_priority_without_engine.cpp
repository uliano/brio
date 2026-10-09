// mcu: ch32v006k8 ch32v003f4
// THE RECEIVE ENGINE'S LEVEL IS A RECEIVE ENGINE'S. UartOptions::rx_priority
// ranks the channel a receive ring runs on (docs/design/dma.md); on a port
// with no receive engine there is no channel to rank, and a level named
// there must be REFUSED rather than ignored.
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

inline constexpr brio::UartOptions low_ring{.rx_priority = brio::DmaPriority::low};
using Plain = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::NoDmaEngine,
                         brio::NoDmaEngine, 0, low_ring>;
void f() { (void)Plain::actual_baud(48'000'000); }
