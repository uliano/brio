// The DCS serial link over this family's SpiHost: every part must
// compile it (instantiation only - the function below is never called).
//
// WHAT THIS PROVES. devices/dcs_link.hpp copies a PROTOTYPE REQUEST and
// supplies only the command byte, the spans and the completion style,
// so it must compile over a Request it knows nothing about. On this
// family the Request carries a frame size of eight or sixteen bits and
// counts `cmd_len` and `len` in FRAMES - which is why the link refuses
// a prototype whose frame is not a byte, and why both prototypes below
// spell `bits8`.
//
// One die, seven packages: a package whose SPI1 column shares its MOSI
// pin with another pad (PA7 with PB0 on the CH32X033F8, table 2-1's
// note 7) forms no host on that column, as test/family_ch32x035/spi.cpp
// pins. On such a part the link is not formed either, and this file
// proves it only where the host exists - the same test the SPI fixture
// makes of itself.
#include <span>

#include "ch32x035/pin.hpp"
#include "ch32x035/spi.hpp"
#include "devices/dcs.hpp"
#include "devices/dcs_link.hpp"

using namespace brio;

/// The host is formed where the package lets it drive SCK and MOSI.
constexpr bool link_column_ok(const SpiPins& p) {
    return spi_pins_valid(p) && pad_can_drive(p.sck) && pad_can_drive(p.mosi);
}

/// The link over SPI1 on one column, formed only where the package lets
/// the host drive it - the pins a template parameter so that nothing
/// below is instantiated on a part that forms no host.
template <SpiPins pins, bool formed = link_column_ok(pins)>
struct LinkFixture {
    static void verbs() {}
};

template <SpiPins pins>
struct LinkFixture<pins, true> {
    using PanelHost = SpiHost<1, pins>;
    using PanelLink = DcsSerialLink<PanelHost>;
    static_assert(DcsLink<PanelLink>);

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

    static void verbs() {
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
};

void devices_link_verbs() { LinkFixture<spi1_default_pins>::verbs(); }
