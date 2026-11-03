// mcu: samc21e18a samc21g18a samc21j18a
// An image runs two DMA engines at most (docs/samc21/dmac.md, "The rule"),
// and the compiler keeps the budget: with channel numbers 0 and 1 alone and
// one owner a channel, a third engined transport - here an I2C host's
// transmit engine beside a console's and an SPI host's - names a channel
// another request owns and fails to compile, whatever the transports' kinds.
#include "samc21/dmac.hpp"
#include "samc21/i2c.hpp"
#include "samc21/sercom.hpp"
#include "samc21/spi.hpp"
using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};
constexpr SpiPads spi_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 19, PinFunction::c},
};
constexpr I2cPads i2c_pads{
    .sda_pin = {'A', 8, PinFunction::d},
    .scl_pin = {'A', 9, PinFunction::d},
};

void f() {
    (void)Uart<0, console_pads, 64, 256, DmaTxEngine<0>>::tx_idle();
    (void)SpiHost<1, spi_pads, 0, DmaTxEngine<1>>::status();
    (void)I2cHost<2, i2c_pads, 0, DmaTxEngine<1>>::status();
}
