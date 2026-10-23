// The panel driver over this family's SpiHost: every part must compile
// it (instantiation only - the function below is never called).
//
// WHAT THIS PROVES. devices/dcs_panel.hpp is the vocabulary over a
// controller's traits over a link, and it names no family: all that
// reaches it from down here is the link's three verbs and a Pin-like
// reset line. So this file is the same everywhere but the pads and the
// prototype's own fields, and what it judges is that the driver's two
// buffers, its rotation map, its window arithmetic and its oracle
// compile for every part the family has.
//
// One die, seven packages: a package whose SPI1 column shares its MOSI
// pin with another pad (PA7 with PB0 on the CH32X033F8, table 2-1's
// note 7) forms no host on that column, as test/family_ch32x035/spi.cpp
// pins. On such a part the panel is not formed either, and this file
// proves the DcsLink, the panel and their oracle only where the host
// exists - the same test the SPI fixture makes of itself.
#include <span>

#include "ch32x035/pin.hpp"
#include "ch32x035/spi.hpp"
#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"
#include "devices/dcs_panel.hpp"
#include "devices/ili9481.hpp"

using namespace brio;

/// The host is formed where the package lets it drive SCK and MOSI.
constexpr bool panel_column_ok(const SpiPins& p) {
    return spi_pins_valid(p) && pad_can_drive(p.sck) && pad_can_drive(p.mosi);
}

/// The milliseconds the wake asked for, added up and never spent: a
/// compile fixture has no clock tree, and what the driver needs of one
/// is only that it be callable.
volatile uint32_t waited_ms = 0;

/// The panel over SPI1 on one column, formed only where the package lets
/// the host drive it - the pins a template parameter so that nothing
/// below is instantiated on a part that forms no host.
template <SpiPins pins, bool formed = panel_column_ok(pins)>
struct PanelFixture {
    static void verbs() {}
};

template <SpiPins pins>
struct PanelFixture<pins, true> {
    using PanelHost = SpiHost<1, pins>;
    using PanelLink = DcsSerialLink<PanelHost>;

    /// The two tenures, filled as this stratum fills a request: the
    /// select and the D/C pin, the setup time, the division of HCLK, the
    /// mode and the frame - and the read one slower, because a panel's
    /// read ceiling is not its write ceiling.
    static typename PanelLink::Config panel_link_config() {
        typename PanelHost::Request write{};
        write.cs = Pin<'A', 4>::ref();
        write.dc = Pin<'A', 3>::ref();
        write.cs_setup_us = 1;
        write.clock = SpiClock::div4;
        write.mode = SpiMode::mode0;
        write.bits = SpiDataSize::bits8;
        typename PanelHost::Request read = write;
        read.clock = SpiClock::div16;
        return typename PanelLink::Config{write, read};
    }

    using PanelDriver = DcsPanel<Ili9481, PanelLink>;
    static_assert(Surface<PanelDriver>);
    static_assert(ReadableSurface<PanelDriver>);

    /// The module's own facts, as a program hands them over beside the
    /// pins: this glass mirrored, its channels crossed, wanting the
    /// inversion.
    static constexpr DcsModule panel_module{.column_mirror = true, .bgr = true,
                                            .wants_inversion = true};

    static void verbs() {
        PanelLink link{panel_link_config()};
        PanelDriver panel{link, panel_module};

        // The reset line is a pad of this family's own, named at run
        // time as a program would name it: every stratum's PinRef
        // carries the set() and clear() the wake asks for. The clock is
        // the application's, and a compile fixture has none.
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
};

void devices_panel_verbs() { PanelFixture<spi1_default_pins>::verbs(); }
