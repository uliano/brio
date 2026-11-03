// mcu: ch32v006k8
// One owner a DMA channel (util/dma_owner.hpp, docs/design/dma.md): table
// 8-2 ORs USART2's transmit and I2C1's transmit onto channel 6, so a Uart on
// USART2 and an I2C host, each with its transmit engine there, are two
// owners of one channel - REFUSED, the error naming DmaSlotId<1, 6> and both
// requests.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Clk = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using Serial = brio::Uart<2, brio::Ch32v00xPlatform<>, 64, 64, brio::DmaTxEngine<6>, brio::NoDmaEngine, 1>;
using Bus = brio::I2cHost<1, brio::i2c1_default_pins, brio::DmaTxEngine<6>, brio::DmaRxEngine<7>>;
void f() {
    (void)Serial::init(Clk{}, 115200);
    (void)Bus::init(Clk{});
}
