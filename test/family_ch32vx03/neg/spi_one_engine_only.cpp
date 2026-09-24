// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// BOTH ENGINES OR NEITHER: the data phase is full duplex and the
// transaction's completion is the RECEIVE block's, so a host with a
// transmit engine and no receive one would clock its frames out and
// never learn that they came back.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/spi.hpp"

using Half = brio::SpiHost<1, brio::spi_default_pins<1>, brio::DmaTxEngine<3>>;
void f() { (void)Half::status(); }
