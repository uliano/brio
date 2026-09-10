// SPI/I2S family smoke TU: the Spi<n> resource, both tasks above it and
// the arithmetic between them - on SPI1 and SPI2, which every G0 has,
// and on SPI3 where the header declares one (the G0B1/G0C1). The pads
// are the Nucleo-G0B1RE self-link's (DS13560 tables 13, 15, 17, 18):
// SPI1 PB3/PB4/PB5 with PA15 as NSS, SPI2 PB10/PC2/PD4 with PB12 - all
// on ports every package of the family carries.
#include "stm32g0/clock.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/spi.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

// ---- the reserve ------------------------------------------------------------

static_assert(spi_present(1) && spi_present(2));
static_assert(!spi_present(0) && !spi_present(4));
static_assert(spi_bus_clock(1).apb2 && spi_bus_clock(1).mask == RCC_APBENR2_SPI1EN);
static_assert(!spi_bus_clock(2).apb2 && spi_bus_clock(2).mask == RCC_APBENR1_SPI2EN);
static_assert(spi_dma_rx_request(1) == 16 && spi_dma_tx_request(1) == 17);
static_assert(spi_dma_rx_request(2) == 18 && spi_dma_tx_request(2) == 19);
static_assert(spi_dma_rx_request(3) == 66 && spi_dma_tx_request(3) == 67);
static_assert(spi_has_i2s(1));
static_assert(!spi_has_i2s(3));
static_assert(i2s_clock_select(1).pos != 0xFF);
static_assert(i2s_clock_select(3).pos == 0xFF);
#if defined(SPI3_BASE)
static_assert(spi_present(3));
static_assert(spi_bus_clock(3).mask == RCC_APBENR1_SPI3EN);
static_assert(spi_has_i2s(2));
static_assert(i2s_clock_select(2).pos != 0xFF && i2s_clock_select(2).ccipr2);
static_assert(i2s_clock_select(1).ccipr2);
#else
static_assert(!spi_present(3));
static_assert(!spi_has_i2s(2));
static_assert(i2s_clock_select(2).pos == 0xFF);
static_assert(!i2s_clock_select(1).ccipr2);
#endif

// ---- the baud arithmetic, all eight codes -----------------------------------

static_assert(spi_division(SpiClock::div2) == 2 && spi_division(SpiClock::div256) == 256);
static_assert(spi_sck_hz(64'000'000, SpiClock::div2) == 32'000'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div4) == 16'000'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div8) == 8'000'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div16) == 4'000'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div32) == 2'000'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div64) == 1'000'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div128) == 500'000);
static_assert(spi_sck_hz(64'000'000, SpiClock::div256) == 250'000);
static_assert(spi_max_sck_hz(64'000'000) == 32'000'000);
static_assert(spi_min_sck_hz(64'000'000) == 250'000);
static_assert(spi_rate_for(64'000'000, 10'000'000) == SpiClock::div8);
static_assert(spi_rate_for(16'000'000, 1'000'000) == SpiClock::div16);
static_assert(!spi_rate_for(64'000'000, 100'000).has_value());

// ---- the frame sizes --------------------------------------------------------

static_assert(spi_data_size_valid(SpiDataSize::bits4));
static_assert(spi_data_size_valid(SpiDataSize::bits16));
static_assert(!spi_data_size_valid(static_cast<SpiDataSize>(2)));
static_assert(spi_data_bits(SpiDataSize::bits12) == 12);
static_assert(spi_frame_mask(SpiDataSize::bits12) == 0x0FFFu);
static_assert(spi_frame_is_halfword(SpiDataSize::bits16));
static_assert(!spi_frame_is_halfword(SpiDataSize::bits4));

// ---- the register words -----------------------------------------------------

static_assert((spi_cr1(SpiConfig{}) & SPI_CR1_MSTR) != 0u);
static_assert((spi_cr1(SpiConfig{}) & SPI_CR1_SSI) != 0u);   // a master selects nothing
static_assert((spi_cr1(SpiConfig{.role = SpiRole::client}) & SPI_CR1_SSI) == 0u);
static_assert((spi_cr1(SpiConfig{.mode = SpiMode::mode3}) & (SPI_CR1_CPOL | SPI_CR1_CPHA)) ==
              (SPI_CR1_CPOL | SPI_CR1_CPHA));
static_assert((spi_cr2(SpiConfig{}) & SPI_CR2_DS) == (7u << SPI_CR2_DS_Pos));
static_assert((spi_cr2(SpiConfig{}) & SPI_CR2_FRXTH) != 0u);   // 8-bit: quarter
static_assert((spi_cr2(SpiConfig{.bits = SpiDataSize::bits16}) & SPI_CR2_FRXTH) == 0u);
static_assert((spi_cr2(SpiConfig{.nss = SpiNss::hardware_pulse}) &
               (SPI_CR2_NSSP | SPI_CR2_SSOE)) == (SPI_CR2_NSSP | SPI_CR2_SSOE));
static_assert((spi_cr1(SpiConfig{.nss = SpiNss::hardware_input}) & SPI_CR1_SSM) == 0u);

// ---- the I2S arithmetic, against 35.7.4's own formulas ----------------------

static_assert(i2s_data_bits(I2sDataLength::bits24) == 24);
static_assert(i2s_channel_bits(I2sDataLength::bits16, I2sChannelLength::bits16) == 16);
static_assert(i2s_channel_bits(I2sDataLength::bits32, I2sChannelLength::bits16) == 32);
static_assert((i2s_cfgr(I2sConfig{}) & SPI_I2SCFGR_I2SMOD) != 0u);
static_assert((i2s_cfgr(I2sConfig{.mode = I2sMode::slave_receive}) & SPI_I2SCFGR_I2SCFG) ==
              (1u << SPI_I2SCFGR_I2SCFG_Pos));
static_assert((i2s_pr(I2sConfig{.divider = 21, .master_clock_out = true}) &
               SPI_I2SPR_MCKOE) != 0u);
static_assert((i2s_pr(I2sConfig{.divider = 21}) & 0xFFu) == 21u);
// Fs = f / (32 x (CHLEN + 1) x (2 x DIV + ODD)) without MCK.
static_assert(i2s_sampling_hz(64'000'000, I2sConfig{.divider = 21}) == 47'619);
// ...and f / (256 x (2 x DIV + ODD)) with it.
static_assert(i2s_sampling_hz(64'000'000,
                              I2sConfig{.divider = 3, .master_clock_out = true}) == 41'666);
// PCM halves both denominators.
static_assert(i2s_sampling_hz(64'000'000,
                              I2sConfig{.standard = I2sStandard::pcm, .divider = 21}) ==
              95'238);
static_assert(!i2s_config_valid(I2sConfig{.divider = 0}));
static_assert(i2s_config_valid(I2sConfig{.mode = I2sMode::slave_receive, .divider = 0}));

// ---- the pads ---------------------------------------------------------------

constexpr SpiPins host_pins{
    .sck = {'B', 3, PinFunction::af0},    // SPI1_SCK
    .miso = {'B', 4, PinFunction::af0},   // SPI1_MISO
    .mosi = {'B', 5, PinFunction::af0},   // SPI1_MOSI
    .nss = {'A', 15, PinFunction::af0},   // SPI1_NSS
};
constexpr SpiPins client_pins{
    .sck = {'B', 10, PinFunction::af5},   // SPI2_SCK
    .miso = {'C', 2, PinFunction::af1},   // SPI2_MISO
    .mosi = {'D', 4, PinFunction::af1},   // SPI2_MOSI
    .nss = {'B', 12, PinFunction::af0},   // SPI2_NSS
};
static_assert(spi_pins_valid(host_pins) && spi_pins_valid(client_pins));
static_assert(host_pins.has_nss() && client_pins.has_nss());
static_assert(!spi_pins_valid(SpiPins{}));                       // no SCK
static_assert(!spi_pins_valid(SpiPins{.sck = {'B', 3}, .miso = {'B', 3}, .mosi = {}, .nss = {}}));
static_assert(spi_pins_valid(SpiPins{.sck = {'B', 3}, .miso = {}, .mosi = {'B', 5}, .nss = {}}));

using Host = SpiHost<1, host_pins>;
using Peer = SpiClient<2, client_pins>;
static_assert(Peer::frames_ahead == 2);
using Tx = DmaTxEngine<1, 1>;
using Rx = DmaRxEngine<1, 2>;
using EngineHost = SpiHost<1, host_pins, Tx, Rx>;
#if defined(SPI3_BASE)
constexpr SpiPins third_pins{
    .sck = {'C', 10, PinFunction::af4},    // SPI3_SCK
    .miso = {'C', 11, PinFunction::af4},   // SPI3_MISO
    .mosi = {'C', 12, PinFunction::af4},   // SPI3_MOSI
    .nss = {},                             // software select
};
using Third = SpiHost<3, third_pins>;
#endif

static_assert(!Host::has_engines && EngineHost::has_engines);
static_assert(Spi<1>::has_i2s_mode);
static_assert(Spi<1>::has_crc && Spi<1>::has_ti_mode && Spi<1>::has_nss_pulse);
static_assert(Spi<1>::fifo_bits == 32);
static_assert(std::is_trivially_copyable_v<Host::Request>);

// ---- every resource verb, instantiated ---------------------------------------

void spi_resource_verbs() {
    using S = Spi<1>;
    S::bus_clock(true);
    (void)S::bus_clock();
    S::reset();
    (void)S::regs();
    (void)S::irq();
    (void)S::dma_rx_request();
    (void)S::dma_tx_request();
    (void)S::data_address();
    (void)S::has_i2s();
    (void)S::i2s_kernel_clock(I2sClock::hsi16);
    (void)S::i2s_kernel_clock();

    (void)S::configure(SpiConfig{});
    (void)S::enabled();
    S::enable();
    (void)S::disable();
    (void)S::disable_receive_only();
    S::flush_rx();

    (void)S::role(SpiRole::client);
    (void)S::role();
    (void)S::mode(SpiMode::mode2);
    (void)S::mode();
    (void)S::clock(SpiClock::div64);
    (void)S::clock();
    (void)S::bit_order(true);
    (void)S::lsb_first();
    (void)S::data_size(SpiDataSize::bits16);
    (void)S::data_size();
    S::rx_threshold(SpiRxThreshold::half);
    (void)S::rx_threshold();
    (void)S::direction(SpiDirection::bidi_transmit);
    (void)S::direction();
    S::bidi_output(false);
    (void)S::nss(SpiNss::hardware_input);
    (void)S::nss();
    S::software_select(true);
    (void)S::software_selected();
    (void)S::frame_format(SpiFrameFormat::ti);
    (void)S::frame_format();

    (void)S::crc(true, SpiCrcLength::crc16);
    (void)S::crc();
    (void)S::crc_length();
    (void)S::crc_polynomial(0x1021u);
    (void)S::crc_polynomial();
    S::crc_next();
    (void)S::crc_next_pending();
    (void)S::rx_crc();
    (void)S::tx_crc();
    (void)S::restart_crc();

    (void)S::dma_transmit(true);
    (void)S::dma_receive(true);
    (void)S::dma_transmit();
    (void)S::dma_receive();
    (void)S::last_dma_transmit_odd(true);
    (void)S::last_dma_receive_odd(true);

    S::data_byte(0x5A);
    (void)S::data_byte();
    S::data_halfword(0x1234);
    (void)S::data_halfword();
    S::data(SpiDataSize::bits16, 0x0FFF);
    (void)S::data(SpiDataSize::bits8);

    (void)S::status();
    (void)S::rxne();
    (void)S::txe();
    (void)S::busy();
    (void)S::channel_right();
    (void)S::tx_level();
    (void)S::rx_level();
    (void)S::overrun();
    (void)S::mode_fault();
    (void)S::crc_error();
    (void)S::frame_error();
    (void)S::underrun();
    S::clear_overrun();
    S::clear_mode_fault();
    S::clear_crc_error();
    S::clear_frame_error();
    S::clear_underrun();

    S::interrupts(SpiInterrupt::all, false);
    (void)S::interrupts();
    S::rxne_interrupt(true);
    S::txe_interrupt(true);
    S::error_interrupt(true);
    (void)S::isr();

    (void)S::i2s_mode();
    (void)S::i2s_enabled();
    (void)S::i2s_configure(I2sConfig{});
    (void)S::spi_mode();
    (void)S::i2s_enable();
    (void)S::i2s_stop_transmit();
    S::i2s_disable();
    (void)S::i2s_configuration();
    (void)S::i2s_standard();
    (void)S::i2s_data_length();
    (void)S::i2s_channel_length();
    (void)S::i2s_divider();
    (void)S::i2s_divider_odd();
    (void)S::i2s_master_clock_out();
}

// SPI2's own verbs, so the shared vector and the second APB register are
// instantiated too; and SPI3's where it exists.
void spi_second_instance() {
    Spi<2>::bus_clock(true);
    Spi<2>::reset();
    (void)Spi<2>::configure(SpiConfig{.role = SpiRole::client});
    (void)Spi<2>::isr();
    (void)Spi<2>::has_i2s();
    (void)Spi<2>::i2s_configure(I2sConfig{.mode = I2sMode::slave_receive});
#if defined(SPI3_BASE)
    Spi<3>::bus_clock(true);
    Spi<3>::reset();
    (void)Spi<3>::configure(SpiConfig{});
    (void)Spi<3>::isr();
    (void)Spi<3>::i2s_configure(I2sConfig{});   // refused: no I2S on SPI3
#endif
}

// ---- both tasks --------------------------------------------------------------

void spi_tasks() {
    constexpr SysClock clock;
    (void)Host::init(clock, 8'000'000);
    (void)Host::clock_for(1'000'000);
    (void)Host::sck_hz(SpiClock::div8);
    (void)Host::max_sck_hz();
    (void)Host::ceiling_clock();
    (void)Host::reference_hz();
    Host::rebase(16'000'000);
    Host::prime(SpiMode::mode3, SpiClock::div4);
    Host::prime(SpiMode::mode0, SpiClock::div4, SpiDataSize::bits12);

    static const uint8_t cmd[2] = {0x9F, 0x00};
    static uint8_t rx[8]{};
    Host::Request r{};
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = 2;
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.clock = SpiClock::div8;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    (void)Host::start(r);
    (void)Host::isr();
    (void)Host::dma_isr();
    (void)Host::status();
    (void)Host::recover();
    Host::release();

    (void)EngineHost::init(clock);
    EngineHost::Request e{};
    e.tx = lend<Lease::reply>(cmd);
    e.len = 2;
    (void)EngineHost::start(e);
    (void)EngineHost::isr();
    (void)EngineHost::dma_isr();
    EngineHost::release();

#if defined(SPI3_BASE)
    (void)Third::init(clock);
    Third::release();
#endif

    (void)Peer::init(clock, {.mode = SpiMode::mode0, .bits = SpiDataSize::bits8});
    Peer::enable(0xA5, 0x5A);
    Peer::write(0x33);
    (void)Peer::writable();
    (void)Peer::poll();
    (void)Peer::selected();
    Peer::select(true);
    Peer::drive_output(false);
    (void)Peer::overrun();
    Peer::clear_overrun();
    (void)Peer::crc_error();
    Peer::clear_crc_error();
    (void)Peer::frame_error();
    Peer::clear_frame_error();
    (void)Peer::status();
    (void)Peer::isr();
    Peer::rxne_interrupt(true);
    Peer::error_interrupt(true);
    (void)Peer::bits();
    (void)Peer::disable();
    Peer::release();
}

// ---- the vector name the reserve derives -------------------------------------

extern "C" void BRIO_STM32G0_SPI2_HANDLER();
