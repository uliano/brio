// The panel driver over this family's SpiHost: every part of both
// series, under both ISAs must compile it (instantiation only - the
// function below is never called).
//
// WHAT THIS PROVES. devices/dcs_panel.hpp is the vocabulary over a
// controller's traits over a link, and it names no family: all that
// reaches it from down here is the link's three verbs and a Pin-like
// reset line. So this file is the same everywhere but the pads and the
// prototype's own fields, and what it judges is that the driver's two
// buffers, its rotation map, its window arithmetic and its oracle
// compile for every part the family has.
#include <span>

#include "ch32vx03/pin.hpp"
#include "ch32vx03/spi.hpp"
#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "devices/dcs_panel.hpp"
#include "devices/ili9481.hpp"

using namespace brio;

using PanelHost = SpiHost<1>;   // SPI1, on its own default column
using PanelLink = DcsSerialLink<PanelHost>;

/// The two tenures, filled as this stratum fills a request: the select
/// and the D/C pin, the setup time, the division of THE INSTANCE'S OWN
/// BUS, the mode and the frame - and the read one slower, because a
/// panel's read ceiling is not its write ceiling.
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
    auto reset = Pin<'A', 2>::ref();
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
