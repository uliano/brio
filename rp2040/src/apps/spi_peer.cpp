// spi_peer - a scriptable SPI CLIENT for the far end of a two-board SPI
// test, on an RP2040: the far board is the DUT and the bus host, and it
// drives this one IN BAND over the very bus both are testing
// (avrdx/src/apps/spi_link.hpp). The body is the one both Raspberry Pi chips
// share, rp_spi_peer.hpp beside this file - the client a PIO program, the
// roles inverted on the PL022 - whose header says what the silicon makes of
// the protocol; this file binds it to the RP2040:
//
//  - THE BUS IS SPI1's PADS: GP10 SCK, GP8 this chip's receive pad (the
//    host's MOSI), GP11 its transmit pad (the answer line), GP29 the select
//    (1.5 kOhm to THIS board's 3V3), the pair's SPI wires to the WeAct
//    RP2350B's GP30 / GP31 / GP28 / GP41 (docs/boards/weact-rp2040.md). As
//    the client they belong to PIO0's machine 0 (function 6); for
//    host_burst to the PL022 (function 1), the select its frame signal.
//  - THE EXCHANGE'S ENGINES are DMA channels 0 and 1, bound per window to
//    the machine's FIFOs; they report a bus error alone, on DMA_IRQ_0.
//  - THE CLOCK is the 12 MHz crystal through the PLL to 125 MHz: the
//    machine runs at that clk_sys, and Ident.xtal is 1 when it came up.
//  - NO GUARD: this board's pads are the ones that are not failsafe, and
//    the select's pull-up hangs from its own rail.
//  - ONE DRIVER PER WIRE: no other link pad is touched (GP2, GP3, GP4,
//    GP5, GP20, GP21 stay at reset).
//
// Console: UART0 on GP0/GP1 at 115200 - the Debug Probe's bridge -
// observability only.
//   ? help | i status and counters | 0 back to the dark client | 3 trace
//   | b the input synchronizers bypassed or not
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/pio.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/spi.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (rp_spi_peer.hpp's header).
#include "../../../avrdx/src/apps/spi_link.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Console = Uart<0, console_pins>;
constexpr Console console;

constexpr uint8_t sck_gpio = 10;
constexpr uint8_t rx_gpio = 8;
constexpr uint8_t tx_gpio = 11;
constexpr uint8_t cs_gpio = 29;
using SckPin = Pin<sck_gpio>;
using RxPin = Pin<rx_gpio>;
using TxPin = Pin<tx_gpio>;
using CsPin = Pin<cs_gpio>;

using Blk = Pio<0>;
using Sm = PioSm<0, 0>;
constexpr PinFunction pio_function = PinFunction::pio0;
/// The RP2040's PIO sees GPIO 0..29 as itself.
constexpr uint8_t pio_index(uint8_t gpio) { return gpio; }
void open_pio_window() {}

constexpr SpiPins host_pins{.sck = sck_gpio, .tx = tx_gpio, .rx = rx_gpio, .cs = cs_gpio};
using Host = SpiHost<1, host_pins>;
using PeerTx = DmaTxEngine<0, uint16_t>;
using PeerRx = DmaRxEngine<1, uint16_t>;

uint32_t us_now() { return Timer::now_low(); }

constexpr uint8_t peer_kind = spilink::peer_rp2040;
constexpr char peer_label[8] = {'r', 'p', '2', '0', '4', '0', '\0', '\0'};
constexpr const char* link_text = "SPI1's pads GP10 (SCK) / GP8 (in) / GP11 (out) / GP29 (CS)";
bool xtal_ok = false;

/// This board's pads are the ones that are not failsafe: nothing to guard.
bool link_pads_free() { return true; }

// THE BODY, over the names above.
#include "rp_spi_peer.hpp"
}  // namespace

extern "C" void isr_uart0() { (void)Console::isr(); }
extern "C" void isr_dma_0() { peer_dma_vector(); }
/// host_burst is one polled request: the line carries nothing, and a
/// stray level is disarmed rather than looped on.
extern "C" void isr_spi1() { brio::Pl022<1>::interrupts(brio::SpiInterrupt::all, false); }

int main() {
    xtal_ok = SysClock::init();
    const bool timer_ok = Timer::init(clock);
    const bool dma_ok = Dma::init();
    const bool console_ok = Console::init(clock, 115200);
    enable_interrupts();
    const bool client_ok = bring_up();
    print(console, crlf, "spi_peer - SPI instrument client on PIO0 (RP2040, label 'rp2040', fw ",
          hex(firmware_version), ")", crlf);
    print(console, link_text, "; command mode = SPI mode 0, MSb first; DARK by default - the answer line "
                   "driven only for one window after a frame that checked out", crlf);
    print(console, "boot: clk=", xtal_ok ? "PLL125 on the crystal" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
          " dma=", dma_ok ? "released" : "FAILED", " console=", console_ok ? "UART0" : "FAILED",
          " client=", client_ok ? "up" : "OFF", crlf);
    help();
    print(console, "> ");
    peer_loop();
}
