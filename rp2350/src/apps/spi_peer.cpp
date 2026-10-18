// spi_peer - a scriptable SPI CLIENT for the far end of a two-board SPI
// test, on an RP2350: the far board is the DUT and the bus host, and it
// drives this one IN BAND over the very bus both are testing
// (avrxt/src/apps/spi_link.hpp). The body is the one both Raspberry Pi chips
// share, rp2040/src/apps/rp_spi_peer.hpp - the client a PIO program, the
// roles inverted on the PL022 - whose header says what the silicon makes of
// the protocol; this file binds it to the RP2350, ONE SOURCE FOR BOTH
// ARCHITECTURES (spi_link's one kind, 0x09, the label naming the half:
// "rp2350a" on the Cortex-M33 pair, "rp2350r" on the Hazard3 pair):
//
//  - THE BUS IS SPI1's PADS: GP30 SCK, GP28 this chip's receive pad (the
//    host's MOSI), GP31 its transmit pad (the answer line), GP41 the select
//    (1.5 kOhm to the RP2040's 3V3), the pair's SPI wires to the WeAct
//    RP2040's GP10 / GP11 / GP8 / GP29 (docs/boards/weact-rp2350b.md). As
//    the client they belong to PIO0's machine 0 (function 6) with the
//    block's window at GPIO16..47 (GPIOBASE 16, 11.1.1: GP41 is out of the
//    first window); for host_burst to the PL022 (function 1), the select its
//    frame signal.
//  - THE PARTNER'S RAIL FIRST (pair_power.hpp): nothing is handed over until
//    GP36/GP37 read high through their pull-ups - an RP2040 pad is not
//    failsafe - and the rail is asked again once a second while it reads
//    down.
//  - THE EXCHANGE'S ENGINES are DMA channels 0 and 1, bound per window to
//    the machine's FIFOs; they report a bus error alone, on DMA_IRQ_0.
//  - THE CLOCK is the 12 MHz crystal through the PLL to 150 MHz: the
//    machine runs at that clk_sys, and Ident.xtal is 1 when it came up.
//  - ONE DRIVER PER WIRE: no other link pad is touched (GP4, GP5, GP20,
//    GP21, GP36, GP37 stay at reset, isolated).
//
// Console: UART0 on GP0/GP1 at 115200 - observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//   | b the input synchronizers bypassed or not
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/pio.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (rp_spi_peer.hpp's header).
#include "../../../avrxt/src/apps/spi_link.hpp"

#include "pair_power.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Console = Uart<0, console_pins>;
constexpr Console console;

constexpr uint8_t sck_gpio = 30;
constexpr uint8_t rx_gpio = 28;
constexpr uint8_t tx_gpio = 31;
constexpr uint8_t cs_gpio = 41;
using SckPin = Pin<sck_gpio>;
using RxPin = Pin<rx_gpio>;
using TxPin = Pin<tx_gpio>;
using CsPin = Pin<cs_gpio>;

using Blk = Pio<0>;
using Sm = PioSm<0, 0>;
constexpr PinFunction pio_function = PinFunction::pio0;
/// The block's window at GPIO16..47 (GPIOBASE 16): every pin field and WAIT
/// GPIO index is the system GPIO less sixteen.
constexpr uint8_t pio_window_base = 16;
constexpr uint8_t pio_index(uint8_t gpio) { return static_cast<uint8_t>(gpio - pio_window_base); }
void open_pio_window() { (void)Blk::gpio_base(pio_window_base); }

constexpr SpiPins host_pins{.sck = sck_gpio, .tx = tx_gpio, .rx = rx_gpio, .cs = cs_gpio};
using Host = SpiHost<1, host_pins>;
using PeerTx = DmaTxEngine<0, uint16_t>;
using PeerRx = DmaRxEngine<1, uint16_t>;

uint32_t us_now() { return Timer<0>::now_low(); }

constexpr uint8_t peer_kind = spilink::peer_rp2350;
constexpr bool hazard3 = core_kind == CoreKind::hazard3;
constexpr char peer_label[8] = {'r', 'p', '2', '3', '5', '0', hazard3 ? 'r' : 'a', '\0'};
constexpr const char* link_text = "SPI1's pads GP30 (SCK) / GP28 (in) / GP31 (out) / GP41 (CS)";
bool xtal_ok = false;

/// The partner's rail through the I2C link's pull-ups (pair_power.hpp).
bool link_pads_free() { return pair::partner_rail(clock).up(); }

// THE BODY, over the names above.
#include "../../../rp2040/src/apps/rp_spi_peer.hpp"
}  // namespace

extern "C" void isr_uart0() { (void)Console::isr(); }
extern "C" void isr_dma_0() { peer_dma_vector(); }
/// host_burst is one polled request: the line carries nothing, and a
/// stray level is disarmed rather than looped on.
extern "C" void isr_spi1() { brio::Pl022<1>::interrupts(brio::SpiInterrupt::all, false); }

int main() {
    xtal_ok = SysClock::init();
    const bool timer_ok = Timer<0>::init(clock);
    // The partner-rail sense paces itself by the platform timer
    // (rp2350/delay.hpp), which nothing else starts on the Cortex-M33 half.
    const bool ruler_ok = Mtime::start(clock);
    const bool dma_ok = Dma::init();
    const bool console_ok = Console::init(clock, 115200);
    enable_interrupts();
    const bool rail = link_pads_free();
    const bool client_ok = rail && bring_up();
    print(console, crlf, "spi_peer - SPI instrument client on PIO0 (RP2350 on the ",
          hazard3 ? "Hazard3" : "Cortex-M33", " half, label '", peer_label, "', fw ", hex(firmware_version), ")",
          crlf);
    print(console, link_text, "; command mode = SPI mode 0, MSb first; DARK by default - the answer line "
                   "driven only for one window after a frame that checked out", crlf);
    print(console, "boot: clk=", xtal_ok ? "PLL150 on the crystal" : "FAILED", " timer0=", timer_ok ? "1us" : "FAILED",
          " ruler=", ruler_ok ? "1us" : "FAILED", " dma=", dma_ok ? "released" : "FAILED",
          " console=", console_ok ? "UART0" : "FAILED", " partner rail=", rail ? "up" : "DOWN",
          " client=", client_ok ? "up" : "OFF", crlf);
    if (!rail) {
        print(console, "THE PARTNER IS UNPOWERED: GP36/GP37 read low through their pull-ups to the RP2040's 3V3 - "
                       "the client stays off, asked again once a second",
              crlf);
    }
    help();
    print(console, "> ");
    peer_loop();
}
