// The panel driver over this family's SpiHost: every package must
// compile it (instantiation only - the function below is never
// called).
//
// WHAT THIS PROVES. devices/dcs_panel.hpp is the vocabulary over a
// controller's traits over a link, and it names no family: all that
// reaches it from down here is the link's three verbs and a Pin-like
// reset line. So this file is the same everywhere but the pads and the
// prototype's own fields, and what it judges is that the driver's two
// buffers, its rotation map, its window arithmetic and its oracle
// compile for every part the family has.
#include <span>

#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "devices/dcs_panel.hpp"
#include "devices/ili9481.hpp"
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

using PanelDriver = DcsPanel<Ili9481, PanelLink>;
static_assert(Surface<PanelDriver>);
static_assert(ReadableSurface<PanelDriver>);

/// The module's own facts, as a board file hands them over beside the
/// pins: this glass mirrored, its channels crossed, wanting the
/// inversion.
constexpr DcsModule panel_module{.column_mirror = true, .bgr = true, .wants_inversion = true};

/// The milliseconds the wake asked for, added up and never spent: a
/// compile fixture has no clock tree, and what the driver needs of one
/// is only that it be callable.
volatile uint32_t waited_ms = 0;

void devices_panel_verbs() {
    PanelLink link{panel_link_config()};
    PanelDriver panel{link, panel_module};

    // The reset line is a pad of this family's own, named at run time
    // as a board file would name it: every stratum's PinRef carries the
    // set() and clear() the wake asks for. The clock is the
    // application's, and a compile fixture has none.
    auto reset = Pin<'B', 2>::ref();
    (void)panel.reset_and_wake(reset, [](uint16_t ms) { waited_ms = waited_ms + ms; });
    (void)panel.rotation(DcsRotation::r90);
    (void)panel.rotation();
    (void)panel.madctl();
    (void)panel.width();
    (void)panel.height();

    panel.fill_rect(0, 0, 8, 4, 0x00FF8000u);
    const uint32_t run[4] = {0x00FF0000u, 0x0000FF00u, 0x000000FFu, 0x00FFFFFFu};
    panel.write_run(2, 3, run);
    uint32_t back[4] = {};
    (void)panel.read_run(2, 3, back);
    (void)panel.get_pixel(1, 1);
    (void)panel.link_failures();
    panel.reset_counters();
}
