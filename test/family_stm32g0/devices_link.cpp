// The DCS serial link over this family's SpiHost: every header of the
// family must compile it (instantiation only - the function below is
// never called).
//
// WHAT THIS PROVES. devices/dcs_link.hpp copies a PROTOTYPE REQUEST and
// supplies only the command byte, the spans and the completion style,
// so it must compile over a Request it knows nothing about. On this
// family the Request carries a FRAME SIZE of four to sixteen bits, and
// `cmd_len` and `len` are counted in frames - which is why the link
// refuses a prototype whose frame is not eight bits, and why the
// prototypes below spell `bits8`.
#include <span>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/spi.hpp"

using namespace brio;

// SPI1 on the pads the SPI fixture wires.
constexpr SpiPins panel_pins{
    .sck = {'B', 3, PinFunction::af0},
    .miso = {'B', 4, PinFunction::af0},
    .mosi = {'B', 5, PinFunction::af0},
    .nss = {},
};

using PanelHost = SpiHost<1, panel_pins>;
using PanelLink = DcsSerialLink<PanelHost>;
static_assert(DcsLink<PanelLink>);

/// The two tenures, filled as this stratum fills a request: the select
/// and the D/C pin, the setup time, the division, the mode and the
/// frame - and the read one slower, because a panel's read ceiling is
/// not its write ceiling.
PanelLink::Config panel_link_config() {
    PanelHost::Request write{};
    write.cs = Pin<'A', 4>::ref();
    write.dc = Pin<'A', 3>::ref();
    write.cs_setup_us = 1;
    write.clock = SpiClock::div4;
    write.mode = SpiMode::mode0;
    write.bits = SpiDataSize::bits8;
    PanelHost::Request read = write;
    read.clock = SpiClock::div16;
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
