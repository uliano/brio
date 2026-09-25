// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE CHANNEL IS THE REQUEST (RM 11.2.3, table 11-5): I2C1 transmits on
// DMA channel 6 and receives on 7, so a transmit engine sitting on
// channel 4 - which is I2C2's transmit, or USART1's - would be armed
// for a request that never comes.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/i2c.hpp"

using Wrong = brio::I2cHost<1, brio::i2c_default_pins<1>, brio::DmaTxEngine<1, 4>,
                            brio::DmaRxEngine<1, 7>>;
void f() { (void)Wrong::status(); }
