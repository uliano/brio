// The DCS serial link over this family's SpiHost: every package must
// compile it (instantiation only - the function below is never called).
//
// WHAT THIS PROVES. devices/dcs_link.hpp copies a PROTOTYPE REQUEST and
// supplies only the command byte, the spans and the completion style,
// so it must compile over a Request it knows nothing about. On this
// family the rate is a `uint8_t baud` DIVISOR and not an enum, and the
// Request carries no frame size - the frame is a byte by construction.
#include <span>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "samc21/pin.hpp"
#include "samc21/spi.hpp"

using namespace brio;

// The pad row every package of this family bonds, as the SPI fixture
// wires it: DO(MOSI) PAD[0], SCK PAD[1], SS PAD[2], DI(MISO) PAD[3].
constexpr SpiPads panel_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

using PanelHost = SpiHost<0, panel_pads>;
using PanelLink = DcsSerialLink<PanelHost>;
static_assert(DcsLink<PanelLink>);

/// The two tenures, filled as this stratum fills a request: the select
/// and the D/C pin, the setup time, the BAUD REGISTER VALUE and the
/// mode - and the read one slower, because a panel's read ceiling is
/// not its write ceiling. A bigger divisor is a slower clock here.
PanelLink::Config panel_link_config() {
    PanelHost::Request write{};
    write.cs = Pin<'B', 0>::ref();
    write.dc = Pin<'B', 1>::ref();
    write.cs_setup_us = 1;
    write.baud = 1;
    write.mode = SpiMode::mode0;
    PanelHost::Request read = write;
    read.baud = 5;
    return PanelLink::Config{write, read};
}

void devices_link_verbs() {
    PanelLink link{panel_link_config()};
    const uint8_t params[4] = {0x00, 0x00, 0x01, 0x3F};
    uint8_t in[8] = {};
    (void)link.valid();
    (void)link.write_prototype_valid();
    (void)link.read_prototype_valid();
    (void)link.status();
    (void)link.command(Dcs::caset, params);
    (void)link.write(Dcs::ramwr, params);
    (void)link.read(Dcs::ramrd, in);
}
