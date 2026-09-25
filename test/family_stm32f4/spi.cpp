// SPI/I2S family smoke TU: the resource in both of its faces, the two
// bus tasks and the audio PLL verbs, on every header of the pack. SPI1
// and SPI2 exist on every F4 but the 36-pin F410Tx (SPI1 alone); SPI3,
// SPI4, SPI5 and SPI6 are asked of the header. Every verb of every type
// is called once, and the facts the reserve derives are asserted against
// the header's own symbols.
#include <type_traits>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"

using namespace brio;

// ---- the reserve's presence facts ------------------------------------------------

static_assert(spi_present(1));
static_assert(!spi_present(0) && !spi_present(7));
static_assert(spi_on_apb2(1) && !spi_on_apb2(2) && !spi_on_apb2(3));
static_assert(spi_clock_mask(1) == RCC_APB2ENR_SPI1EN);
static_assert(spi_irq(1) == SPI1_IRQn);
#if defined(SPI2_BASE)
static_assert(spi_present(2) && spi_clock_mask(2) == RCC_APB1ENR_SPI2EN && spi_irq(2) == SPI2_IRQn);
#else
static_assert(!spi_present(2));
#endif
#if defined(SPI3_BASE)
static_assert(spi_present(3) && spi_clock_mask(3) == RCC_APB1ENR_SPI3EN && spi_irq(3) == SPI3_IRQn);
#else
static_assert(!spi_present(3));
#endif
#if defined(SPI4_BASE)
static_assert(spi_present(4) && spi_on_apb2(4) && spi_irq(4) == SPI4_IRQn);
#else
static_assert(!spi_present(4));
#endif
#if defined(SPI5_BASE)
static_assert(spi_present(5) && spi_on_apb2(5) && spi_irq(5) == SPI5_IRQn);
#else
static_assert(!spi_present(5));
#endif
#if defined(SPI6_BASE)
static_assert(spi_present(6) && spi_on_apb2(6) && spi_irq(6) == SPI6_IRQn);
#else
static_assert(!spi_present(6));
#endif

// The full-duplex extension blocks, and the classes that pair whole
// instances instead.
#if defined(I2S2ext_BASE)
static_assert(i2s_ext_present(2) && i2s_ext_base(2) == I2S2ext_BASE);
#else
static_assert(!i2s_ext_present(2));
#endif
#if defined(I2S3ext_BASE)
static_assert(i2s_ext_present(3));
#else
static_assert(!i2s_ext_present(3));
#endif
static_assert(!i2s_ext_present(1) && !i2s_ext_present(4));

// The audio PLL and the I2S clock selector, as the header spells them.
#if defined(RCC_CR_PLLI2SON)
static_assert(plli2s_present());
#else
static_assert(!plli2s_present() && i2s_clock_select() != I2sClockSelect::cfgr);
#endif
#if defined(RCC_DCKCFGR_I2S1SRC)
static_assert(i2s_clock_select() == I2sClockSelect::dckcfgr_pair);
#elif defined(RCC_DCKCFGR_I2SSRC)
static_assert(i2s_clock_select() == I2sClockSelect::dckcfgr_one);
#elif defined(RCC_CFGR_I2SSRC)
static_assert(i2s_clock_select() == I2sClockSelect::cfgr);
#else
static_assert(i2s_clock_select() == I2sClockSelect::none);
#endif
#if defined(RCC_PLLI2SCFGR_PLLI2SM_Pos)
static_assert(plli2s_has_m());
#else
static_assert(!plli2s_has_m());
#endif

// Where the I2S face is, per part class - and the refusal on a class
// whose manual was not read.
#if defined(STM32F411xE)
static_assert(spi_i2s_facts().known && spi_i2s_capable(1) && spi_i2s_capable(5));
#elif defined(STM32F446xx)
static_assert(spi_i2s_facts().known && !spi_i2s_facts().ext_blocks);
static_assert(spi_i2s_capable(2) && spi_i2s_capable(3) && !spi_i2s_capable(1));
#elif defined(STM32F429xx) || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
      defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
      defined(STM32F469xx) || defined(STM32F479xx)
static_assert(spi_i2s_facts().known && spi_i2s_facts().ext_blocks);
static_assert(spi_i2s_capable(2) && spi_i2s_capable(3) && !spi_i2s_capable(1));
#else
static_assert(!spi_i2s_facts().known && !spi_i2s_capable(2));
#endif

// The DMA request cells, and the three-cell rows only one manual shows.
#if defined(STM32F411xE)
static_assert(spi_dma_placements(1, true).count == 3);
static_assert(spi_dma_placement_valid(1, true, 2, 2, 2));
static_assert(spi_dma_placements(4, false).count == 3 && spi_dma_placement_valid(4, false, 2, 4, 4));
static_assert(spi_dma_placements(5, true).count == 3 && spi_dma_placement_valid(5, true, 2, 5, 5));
#elif defined(STM32F429xx) || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
      defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
      defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
static_assert(spi_dma_placements(1, true).count == 2 && !spi_dma_placement_valid(1, true, 2, 2, 2));
static_assert(spi_dma_placement_valid(1, false, 2, 0, 3) && spi_dma_placement_valid(1, false, 2, 2, 3));
static_assert(spi_dma_placement_valid(2, true, 1, 4, 0) && spi_dma_placement_valid(2, false, 1, 3, 0));
static_assert(spi_dma_placement_valid(3, true, 1, 7, 0) && spi_dma_placement_valid(3, false, 1, 0, 0));
#else
static_assert(!spi_dma_placements(1, true).known);
static_assert(!spi_dma_placement_valid(1, true, 2, 3, 3));
#endif
#if defined(SPI5_BASE) && (defined(STM32F429xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
                           defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx))
static_assert(spi_dma_placement_valid(5, true, 2, 4, 2) && spi_dma_placement_valid(5, false, 2, 3, 2));
static_assert(spi_dma_placement_valid(6, true, 2, 5, 1) && spi_dma_placement_valid(6, false, 2, 6, 1));
#endif
// An instance the read table has no row for is refused, not guessed.
#if !defined(SPI6_BASE)
static_assert(!spi_dma_placement_valid(6, true, 2, 5, 1));
#endif

using SysClock = Clock<ClockSource::hsi, 16'000'000>;
constexpr SysClock sys_clock;

// ---- the audio PLL's arithmetic ---------------------------------------------------

// A 1 MHz VCO input times 192 over R = 5 is 38.4 MHz - the chapter's own
// table 127 row for a 16-bit 48 kHz stream.
static_assert(plli2s_config_for(8'000'000UL, 38'400'000UL, 8).n == 192);
static_assert(plli2s_config_for(8'000'000UL, 38'400'000UL, 8).r == 5);
static_assert(plli2s_config_for(8'000'000UL, 38'400'000UL, 8).m == 0);
static_assert(plli2s_r_hz(8'000'000UL, plli2s_config_for(8'000'000UL, 38'400'000UL, 8), 8) ==
              38'400'000UL);
// With an M of its own the search picks one.
static_assert(plli2s_config_for(8'000'000UL, 38'400'000UL).m != 0);
// A rate the limits cannot make.
static_assert(plli2s_config_for(8'000'000UL, 7'000'000UL, 8).n == 0);
static_assert(plli2s_r_hz(8'000'000UL, PllI2sConfig{}, 8) == 0);

// ---- the pads ----------------------------------------------------------------------

// SPI1 on the pads every package bonds (DS10693 table 11 / the F429's
// table 12 / the F411's table 9: AF5).
constexpr SpiPins spi1_pins{.sck = {'A', 5, PinFunction::af5},
                            .miso = {'A', 6, PinFunction::af5},
                            .mosi = {'A', 7, PinFunction::af5},
                            .nss = {'A', 4, PinFunction::af5}};
static_assert(spi_pins_valid(spi1_pins));
// A link needs SCK, and two signals may not share a pad.
static_assert(!spi_pins_valid(SpiPins{.miso = {'A', 6, PinFunction::af5}}));
static_assert(!spi_pins_valid(SpiPins{.sck = {'A', 5, PinFunction::af5},
                                      .mosi = {'A', 5, PinFunction::af5}}));
// A write-only bus: no MISO, no NSS.
constexpr SpiPins spi1_write_only{.sck = {'A', 5, PinFunction::af5},
                                  .mosi = {'A', 7, PinFunction::af5}};
static_assert(spi_pins_valid(spi1_write_only));

constexpr I2sPins i2s_pads{.ck = {'B', 10, PinFunction::af5},
                           .ws = {'B', 12, PinFunction::af5},
                           .sd = {'B', 15, PinFunction::af5},
                           .mck = {'C', 6, PinFunction::af5}};
static_assert(i2s_pins_valid(i2s_pads));
static_assert(!i2s_pins_valid(I2sPins{.ck = {'B', 10, PinFunction::af5},
                                      .ws = {'B', 10, PinFunction::af5},
                                      .sd = {'B', 15, PinFunction::af5}}));

// ---- the resource, verb by verb -------------------------------------------------------

template <uint8_t n>
void resource_smoke() {
    using S = Spi<n>;
    static_assert(S::number == n);
    static_assert(S::irq == spi_irq(n));

    S::bus_clock(true);
    (void)S::bus_clock();
    S::reset();

    const SpiConfig host{.role = SpiRole::host,
                         .mode = SpiMode::mode3,
                         .clock = SpiClock::div32,
                         .bits = SpiDataSize::bits16,
                         .lsb_first = true,
                         .nss = SpiNss::software,
                         .direction = SpiDirection::full_duplex,
                         .frame_format = SpiFrameFormat::motorola,
                         .crc = true,
                         .crc_polynomial = 0x1021u | 1u,
                         .dma_transmit = true,
                         .dma_receive = true};
    (void)S::configure(host);
    (void)S::configure(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_input});
    (void)S::configure(SpiConfig{.direction = SpiDirection::receive_only});
    (void)S::configure(SpiConfig{.direction = SpiDirection::half_duplex_in});
    (void)S::configure(SpiConfig{.nss = SpiNss::hardware_output,
                                 .frame_format = SpiFrameFormat::ti});
    S::enable();
    (void)S::enabled();
    (void)S::disable();
    (void)S::master_receive_only();
    S::bidirectional_output(true);
    (void)S::bidirectional();

    S::data(0x00A5u);
    (void)S::data(SpiDataSize::bits8);
    (void)S::data();
    S::data8(0x5A);
    (void)S::data8();
    (void)S::data_address();
    (void)S::rx_ready();
    (void)S::tx_empty();
    (void)S::busy();
    (void)S::status();
    (void)S::flag(SpiFlag::overrun);
    S::flush_rx();

    (void)S::overrun();
    S::clear_overrun();
    (void)S::mode_fault();
    S::clear_mode_fault();
    (void)S::crc_error();
    S::clear_crc_error();
    (void)S::frame_error();
    S::clear_frame_error();

    (void)S::crc_polynomial(0x8005u | 1u);
    (void)S::crc_polynomial();
    (void)S::crc_enable(true);
    (void)S::crc_enabled();
    S::crc_next();
    (void)S::crc_next_pending();
    (void)S::rx_crc();
    (void)S::tx_crc();

    S::software_select(false);
    (void)S::software_selected();
    S::nss_output(true);
    (void)S::nss_output();

    S::dma_requests(true, true);
    S::dma_transmit(false);
    S::dma_receive(false);
    S::rxne_interrupt(true);
    (void)S::rxne_interrupt();
    S::txe_interrupt(true);
    S::error_interrupt(true);
    (void)S::isr();
    S::bus_clock(false);
}

// ---- the I2S face ---------------------------------------------------------------------

/// `ok` is false on a part class whose manual was not read, where every
/// I2S type refuses. The guarded types name the TEMPLATE PARAMETER `n`,
/// which is what makes them dependent and keeps the discarded branch
/// uninstantiated - a non-dependent name inside `if constexpr` is
/// instantiated at the template's definition, refusals and all.
template <uint8_t n, bool ok>
void i2s_instance_smoke() {
    if constexpr (!ok) {
        return;
    } else {
    using Audio = I2s<n>;
    static_assert(Audio::number == n && !Audio::is_extension);

    Audio::bus_clock(true);
    (void)Audio::bus_clock();
    Audio::reset();
    const I2sConfig cfg{.mode = I2sMode::host_transmit,
                        .standard = I2sStandard::philips,
                        .pcm_long_frame = false,
                        .data = I2sDataLength::bits16,
                        .channel = I2sChannelLength::bits32,
                        .clock_idle_high = true,
                        .master_clock_out = true,
                        .div = 12,
                        .odd = true,
                        .dma_transmit = true,
                        .dma_receive = false};
    (void)Audio::configure(cfg);
    (void)Audio::configure(I2sConfig{.mode = I2sMode::client_receive,
                                     .standard = I2sStandard::pcm,
                                     .pcm_long_frame = true,
                                     .data = I2sDataLength::bits24});
    (void)Audio::configure(I2sConfig{.mode = I2sMode::host_receive,
                                     .standard = I2sStandard::msb_justified,
                                     .data = I2sDataLength::bits32});
    (void)Audio::configure(I2sConfig{.mode = I2sMode::client_transmit,
                                     .standard = I2sStandard::lsb_justified});
    Audio::enable();
    (void)Audio::enabled();
    (void)Audio::disable();
    (void)Audio::mode();
    (void)Audio::i2s_mode_selected();
    (void)Audio::prescaler(7, false);
    (void)Audio::prescaler_div();
    (void)Audio::prescaler_odd();
    (void)Audio::master_clock_out();
    Audio::data(0x1234u);
    (void)Audio::data();
    (void)Audio::data_address();
    (void)Audio::tx_empty();
    (void)Audio::rx_ready();
    (void)Audio::busy();
    (void)Audio::status();
    (void)Audio::right_channel();
    (void)Audio::underrun();
    Audio::clear_underrun();
    (void)Audio::overrun();
    Audio::clear_overrun();
    (void)Audio::frame_error();
    Audio::clear_frame_error();
    Audio::dma_requests(true, false);
    Audio::rxne_interrupt(true);
    Audio::txe_interrupt(true);
    Audio::error_interrupt(true);
    (void)Audio::isr();
    Audio::select_spi_mode();
    Audio::bus_clock(false);
    }
}

template <uint8_t n, bool ok>
void i2s_ext_smoke() {
    if constexpr (!ok) {
        return;
    } else {
    using Ext = I2sExt<n>;
    static_assert(Ext::is_extension);
    (void)Ext::configure(I2sConfig{.mode = I2sMode::client_receive});
    // A master mode is refused on the extension block: it is a slave by
    // construction (28.4.2).
    (void)Ext::configure(I2sConfig{.mode = I2sMode::host_transmit});
    Ext::enable();
    (void)Ext::disable();
    Ext::data(0x4321u);
    (void)Ext::data();
    (void)Ext::right_channel();
    (void)Ext::isr();
    Ext::select_spi_mode();
    }
}

/// The whole I2S half is behind the reserve's `known`: on a part class
/// whose manual was not read the types refuse, and the fixture must not
/// name them there.
void i2s_smoke() {
    i2s_instance_smoke<2, spi_i2s_facts().known>();
#if defined(I2S2ext_BASE)
    i2s_ext_smoke<2, spi_i2s_facts().known>();
#endif
}

// ---- the RCC verbs the I2S needs --------------------------------------------------------

void audio_pll_smoke() {
    constexpr PllI2sConfig cfg = plli2s_config_for(8'000'000UL, 38'400'000UL,
                                                   plli2s_has_m() ? 0u : 8u);
    Rcc::plli2s_enable(false);
    (void)Rcc::plli2s_configure(cfg);
    Rcc::plli2s_enable(true);
    (void)Rcc::plli2s_ready();
    (void)Rcc::plli2s_wait(true);
    (void)Rcc::plli2s_config();
    (void)Rcc::pll_m();
    (void)Rcc::i2s_source(I2sSource::plli2s_r, false);
    (void)Rcc::i2s_source(I2sSource::ckin, true);
    (void)Rcc::i2s_source(I2sSource::pll_r);
    (void)Rcc::i2s_source(I2sSource::root);
    (void)Rcc::i2s_source(true);
}

// ---- the tasks ----------------------------------------------------------------------------

using Bus = SpiHost<1, spi1_pins>;
using WriteOnlyBus = SpiHost<1, spi1_write_only>;
using Peer = SpiClient<1, spi1_pins>;

/// The engined host: the cells are the reserve's, so the engines only
/// exist where the class's table was read. SPI1's transmit is DMA2 stream
/// 3 channel 3 and its receive DMA2 stream 0 channel 3 on every class of
/// the four; elsewhere the slots fall back to the empty tag, so the same
/// code exercises the engineless path on the other nineteen headers.
template <bool known = spi_dma_placements(1, true).known>
void engined_smoke() {
    using Tx = std::conditional_t<known, DmaTxEngine<2, 3, 3>, NoDmaEngine>;
    using Rx = std::conditional_t<known, DmaRxEngine<2, 0, 3>, NoDmaEngine>;
    using EnginedBus = SpiHost<1, spi1_pins, Tx, Rx>;
    static_assert(EnginedBus::has_engines == known);
    (void)EnginedBus::init(sys_clock, 1'000'000u);
    typename EnginedBus::Request r{};
    r.len = 4;
    (void)EnginedBus::start(r);
    (void)EnginedBus::dma_isr();
    (void)EnginedBus::status();
    (void)EnginedBus::recover();
    EnginedBus::release();
}

void host_smoke() {
    static_assert(!Bus::has_engines);
    (void)Bus::init(sys_clock);
    (void)Bus::init(sys_clock, 5'000'000u);
    Bus::rebase(16'000'000u);
    (void)Bus::clock_for(1'000'000u);
    (void)Bus::sck_hz(SpiClock::div8);
    (void)Bus::max_sck_hz();
    (void)Bus::ceiling_clock();
    (void)Bus::reference_hz();
    Bus::prime(SpiMode::mode3, SpiClock::div4, SpiDataSize::bits16);
    (void)Bus::bit_order(true);
    (void)Bus::lsb_first();
    Bus::sck_speed(PinSpeed::medium);
    (void)Bus::sck_speed();
    (void)Bus::errata_apb_ceiling_hz();
    (void)Bus::within_errata_ceiling();

    static uint8_t out[4] = {1, 2, 3, 4};
    static uint8_t in[4] = {};
    static const uint8_t cmd[1] = {0x8Fu};
    typename Bus::Request req{};
    req.cmd = Borrowed<const uint8_t, Lease::reply>{cmd};
    req.cmd_len = 1;
    req.tx = Borrowed<const uint8_t, Lease::reply>{out};
    req.rx = Borrowed<uint8_t, Lease::reply>{in};
    req.len = 4;
    req.clock = SpiClock::div16;
    req.mode = SpiMode::mode3;
    req.bits = SpiDataSize::bits8;
    req.cs_setup_us = 2;
    req.polled = true;
    (void)Bus::start(req);
    req.polled = false;
    (void)Bus::start(req);
    (void)Bus::isr();
    (void)Bus::dma_isr();
    (void)Bus::status();
    (void)Bus::recover();
    Bus::claim_nss_pad(true);
    Bus::claim_nss_pad(false);
    Bus::release();

    (void)WriteOnlyBus::init(sys_clock);
    WriteOnlyBus::release();
}

void client_smoke() {
    static_assert(Peer::frames_ahead == 1);
    static_assert(Peer::has_nss_pad);
    (void)Peer::init(sys_clock);
    (void)Peer::init(sys_clock, typename Peer::Config{.mode = SpiMode::mode1,
                                                      .bits = SpiDataSize::bits16,
                                                      .lsb_first = true,
                                                      .nss = SpiNss::software,
                                                      .direction = SpiDirection::full_duplex,
                                                      .frame_format = SpiFrameFormat::motorola,
                                                      .crc = true,
                                                      .crc_polynomial = 0x1021u | 1u,
                                                      .drive_output = false});
    Peer::drive_output(true);
    Peer::enable(0xA5u);
    (void)Peer::disable();
    Peer::write(0x33u);
    (void)Peer::writable();
    (void)Peer::poll();
    (void)Peer::selected();
    Peer::select(true);
    (void)Peer::overrun();
    Peer::clear_overrun();
    (void)Peer::crc_error();
    Peer::clear_crc_error();
    (void)Peer::frame_error();
    (void)Peer::status();
    (void)Peer::isr();
    Peer::rxne_interrupt(true);
    Peer::txe_interrupt(true);
    Peer::error_interrupt(true);
    (void)Peer::bits();
    Peer::release();
}

int main() {
    resource_smoke<1>();
#if defined(SPI2_BASE)
    resource_smoke<2>();
#endif
#if defined(SPI3_BASE)
    resource_smoke<3>();
#endif
#if defined(SPI4_BASE)
    resource_smoke<4>();
#endif
#if defined(SPI5_BASE)
    resource_smoke<5>();
#endif
#if defined(SPI6_BASE)
    resource_smoke<6>();
#endif
    i2s_smoke();
    audio_pll_smoke();
    host_smoke();
    engined_smoke();
    client_smoke();
    return 0;
}
