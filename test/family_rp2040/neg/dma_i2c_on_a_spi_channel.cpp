// Must FAIL: one owner a channel, on the I2C host's claim: channel 5 is SPI0's
// receive, so I2C0's receive cannot have it too.
#include "rp2040/dma.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/spi.hpp"
constexpr brio::SpiPins s0{.sck = 18, .tx = 19, .rx = 16};
constexpr brio::I2cPins i0{.scl = 13, .sda = 12};
using Bus = brio::SpiHost<0, s0, brio::DmaTxEngine<4, uint16_t>, brio::DmaRxEngine<5, uint16_t>>;
using Sensor = brio::I2cHost<0, i0, brio::DmaTxEngine<6, uint16_t>, brio::DmaRxEngine<5>>;
void f() { (void)sizeof(Bus); (void)sizeof(Sensor); }
