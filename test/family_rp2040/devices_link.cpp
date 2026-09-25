// The DCS serial link over this family's SpiHost: the chip must compile
// it (instantiation only - the function below is never called).
//
// WHAT THIS PROVES. devices/dcs_link.hpp copies a PROTOTYPE REQUEST and
// supplies only the command byte, the spans and the completion style,
// so it must compile over a Request it knows nothing about. Here the
// engine is the IP stratum's PL022 and the rate is a PAIR - an even
// prescaler and a serial clock rate - which is the shape that proves
// the link reads no rate field of its own; the frame size is eight or
// sixteen bits and `cmd_len` and `len` are counted in FRAMES.
#include <span>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "rp2040/spi.hpp"

using namespace brio;

constexpr SpiPins panel_pins{.sck = 18, .tx = 19, .rx = 16};

using PanelHost = SpiHost<0, panel_pins>;
using PanelLink = DcsSerialLink<PanelHost>;
static_assert(DcsLink<PanelLink>);

/// The two tenures, filled as this stratum fills a request: the select
/// and the D/C pin, the setup time, the prescaler pair off clk_peri,
/// the mode and the frame - and the read one slower, because a panel's
/// read ceiling is not its write ceiling.
PanelLink::Config panel_link_config() {
    PanelHost::Request write{};
    write.cs = Pin<17>::ref();
    write.dc = Pin<20>::ref();
    write.cs_setup_us = 1;
    write.clock = SpiClocks::div4;
    write.mode = SpiMode::mode0;
    write.bits = SpiDataSize::bits8;
    PanelHost::Request read = write;
    read.clock = SpiClocks::div16;
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
