// The DCS serial link over this family's SpiHost: every package must
// compile it (instantiation only - the function below is never called).
//
// WHAT THIS PROVES. devices/dcs_link.hpp copies a PROTOTYPE REQUEST and
// supplies only the command byte, the spans and the completion style,
// so it must compile over a Request it knows nothing about. On this
// family the rate is a SpiClock division enum and there is NO frame size
// at all - the frame is a byte by construction - which is the case the
// link's `requires` detection has to get right.
#include <span>

#include "avrdx/pin.hpp"
#include "avrdx/spi.hpp"
#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"

using namespace brio;

using PanelHost = SpiHost<0>;   // the DEFAULT route, on every package
using PanelLink = DcsSerialLink<PanelHost>;
static_assert(DcsLink<PanelLink>);

/// The two tenures, filled as this stratum fills a request: the select
/// and the D/C pin, the setup time, the division and the mode - and the
/// read one slower, because a panel's read ceiling is not its write
/// ceiling.
PanelLink::Config panel_link_config() {
    PanelHost::Request write{};
    write.cs = Pin<'A', 2>::ref();
    write.dc = Pin<'A', 3>::ref();
    write.cs_setup_us = 1;
    write.clock = SpiClock::div4;
    write.mode = SpiMode::mode0;
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
