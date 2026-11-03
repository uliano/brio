// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// One owner a DMA channel (util/dma_owner.hpp, docs/design/dma.md): figure
// 9-1 ORs USART2's receive and I2C1's transmit onto channel 6, so a Uart on
// USART2 with its receive engine and an I2C host with its transmit engine
// are two owners of one channel - REFUSED, the error naming DmaSlotId<1, 6>
// and both requests.
#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/i2c.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using Serial = brio::Uart<2, brio::Ch32x035Platform<>, 64, 64, brio::UartFormat{}, brio::NoDmaEngine,
                          brio::DmaRxEngine<6>>;
using Bus = brio::I2cHost<1, brio::i2c1_default_pins, brio::DmaTxEngine<6>, brio::DmaRxEngine<7>>;
void f() {
    (void)Serial::init(Clk{}, 115200);
    (void)Bus::init(Clk{});
}
