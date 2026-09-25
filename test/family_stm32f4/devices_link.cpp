// The DCS serial link over this family's SpiHost: every header of the
// family must compile it (instantiation only - the function below is
// never called).
//
// WHAT THIS PROVES. devices/dcs_link.hpp copies a PROTOTYPE REQUEST and
// supplies only the command byte, the spans and the completion style,
// so it must compile over a Request it knows nothing about. On this
// family the Request carries a frame size of eight or sixteen bits and
// counts `cmd_len` and `len` in FRAMES - which is why the link refuses
// a prototype whose frame is not a byte, and why both prototypes below
// spell `bits8`.
#include <span>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/spi.hpp"

using namespace brio;

// SPI1 on the pads every package bonds, as the SPI fixture wires them.
constexpr SpiPins panel_pins{.sck = {'A', 5, PinFunction::af5},
                             .miso = {'A', 6, PinFunction::af5},
                             .mosi = {'A', 7, PinFunction::af5}};

using PanelHost = SpiHost<1, panel_pins>;
using PanelLink = DcsSerialLink<PanelHost>;
static_assert(DcsLink<PanelLink>);

/// The two tenures, filled as this stratum fills a request: the select
/// and the D/C pin, the setup time, the division off the instance's own
/// APB clock, the mode and the frame - and the read one slower, because
/// a panel's read ceiling is not its write ceiling.
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
