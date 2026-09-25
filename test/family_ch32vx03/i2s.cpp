// mcu: ch32v303rc ch32v303vc
// I2S family smoke TU: the audio face of SPI2 and SPI3 on the two parts
// the datasheet gives it (table 2-1-1's I2S row) - the configuration in
// every field of I2S_CFGR and I2SPR, the divider arithmetic against the
// clock this class feeds a master (SYSCLK, figure 3-3), the columns of
// table 3-4 claimed for every mode, every flag and verb of the resource,
// its ISR body and its DMA slots. A neg TU proves the face refused on
// every other part.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 144'000'000>;

using Out = I2s<2>;
using In = I2s<3>;

static_assert(i2s_present(2) && i2s_present(3));
static_assert(Out::number == 2 && In::number == 3);
// The face shares the SPI face's vector and its two DMA slots (20.3.9).
static_assert(Out::irq == Irq::spi2 && In::irq == Irq::spi3);
static_assert(Out::dma_tx_slot == DmaSlot{1, 5} && Out::dma_rx_slot == DmaSlot{1, 4});
static_assert(In::dma_tx_slot == DmaSlot{2, 2} && In::dma_rx_slot == DmaSlot{2, 1});
// The clock a master divides is SYSCLK here - not HCLK, not PCLK1.
static_assert(i2s_clock_hz(SysClock{}) == SysClock::sysclk_hz);
static_assert(i2s_clock_hz(SysClock{}) == 144'000'000UL);

/// A master transmitting 16-bit Philips frames at 48 kHz, the divider
/// asked of the arithmetic at compile time.
constexpr auto pre48 = *i2s_prescaler_for(i2s_clock_hz(SysClock{}), 48'000, false, false);
constexpr I2sConfig host_tx{.mode = I2sMode::host_transmit, .div = pre48.div, .odd = pre48.odd};
constexpr I2sConfig client_rx{.mode = I2sMode::client_receive};
constexpr I2sConfig pcm_long{.mode = I2sMode::host_receive, .standard = I2sStandard::pcm,
                             .pcm_long_frame = true, .data = I2sDataLength::bits24,
                             .channel = I2sChannelLength::bits32, .clock_idle_high = true,
                             .master_clock_out = true, .div = 3, .odd = true,
                             .dma_receive = true};
static_assert(i2s_config_valid(host_tx) && i2s_config_valid(client_rx) &&
              i2s_config_valid(pcm_long));
static_assert(i2s_fs_hz(i2s_clock_hz(SysClock{}), host_tx) == 47'872UL);

/// The columns: I2S2's one, I2S3's two, with and without the master
/// clock's pad.
constexpr I2sPins out_pins = i2s_default_pins<2>;
constexpr I2sPins out_pins_mck = i2s_pins_for(2, 0, true);
constexpr I2sPins in_pins = i2s_default_pins<3>;
constexpr I2sPins in_moved = i2s_pins_for(3, 1, true);
static_assert(i2s_pins_valid(2, out_pins) && i2s_pins_valid(2, out_pins_mck));
static_assert(i2s_pins_valid(3, in_pins) && i2s_pins_valid(3, in_moved));
// SPI2's NSS/SCK/MOSI are I2S2's WS/CK/SD; I2S3's are SPI3's on its
// column.
static_assert(out_pins.ws == spi2_default_pins.nss && out_pins.ck == spi2_default_pins.sck &&
              out_pins.sd == spi2_default_pins.mosi);
static_assert(in_moved.ws == spi_pins_for(3, 1).nss && in_moved.sd == spi_pins_for(3, 1).mosi);

void resource_verbs() {
    Out::bus_clock(true);
    Out::reset();
    (void)Out::claim_pads<out_pins_mck>(I2sMode::host_transmit);
    (void)Out::configure(host_tx);
    Out::configure<pcm_long>();
    (void)Out::i2s_mode_selected();
    (void)Out::mode();
    (void)Out::standard();
    (void)Out::prescaler(47, false);
    (void)Out::prescaler_div();
    (void)Out::prescaler_odd();
    (void)Out::master_clock_out();
    Out::enable();
    (void)Out::enabled();
    Out::data(0x1234u);
    (void)Out::tx_empty();
    (void)Out::busy();
    (void)Out::right_channel();
    (void)Out::underrun();
    Out::clear_underrun();
    Out::txe_interrupt(true);
    Out::error_interrupt(true);
    Out::dma_requests(true, false);
    (void)Out::isr();
    (void)Out::data_address();
    (void)Out::disable();
    Out::select_spi_mode();
    Out::release_pads<out_pins_mck>();
    Out::bus_clock(false);

    In::bus_clock(true);
    In::reset();
    (void)In::remap(0);
    (void)In::claim_pads<in_pins>(I2sMode::client_receive);
    (void)In::configure(client_rx);
    In::enable();
    (void)In::rx_ready();
    (void)In::data();
    (void)In::overrun();
    In::clear_overrun();
    (void)In::status();
    In::rxne_interrupt(true);
    (void)In::isr();
    (void)In::disable();
    In::release_pads<in_pins>();
    (void)In::claim_pads<in_moved>(I2sMode::host_receive);
    In::release_pads<in_moved>();
    In::bus_clock(false);
}

extern "C" BRIO_CH32_INTERRUPT void spi2_handler() { (void)Out::isr(); }
extern "C" BRIO_CH32_INTERRUPT void spi3_handler() { (void)In::isr(); }
