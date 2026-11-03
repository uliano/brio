// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A channel has one owner for the whole image (docs/design/dma.md, "The
// ownership rule"). Table 11-5 ORs USART2's transmit request and I2C1's
// receive request onto DMA1's channel 7 (table 11-2 on the CH32V303), so a
// USART2 transport with its transmit engine there and an I2C1 host with
// its engines cannot share an image, even used in turn: the second claim
// names DmaSlotId<1, 7> and both owners. Every part with I2C1 and USART2.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/i2c.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Usart2 = brio::Uart<2, P, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<1, 7>>;
using I2c1 = brio::I2cHost<1, brio::i2c_default_pins<1>, brio::DmaTxEngine<1, 6>, brio::DmaRxEngine<1, 7>>;

void both() {
    Usart2::release();
    I2c1::release();
}
