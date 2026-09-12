// SPI family smoke TU: the vocabulary and its arithmetic, the pin
// table, the resource's verbs, a host with and without engines, a
// client.
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/spi.hpp"

using namespace brio;

static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));
static_assert(SpiClocks::div16.divisor() == 16u && SpiClocks::div2.divisor() == 2u);
static_assert(spi_sck_hz(125'000'000, SpiClocks::div2) == 62'500'000u);
static_assert(spi_clock_for(125'000'000, 62'500'000).has_value());
static_assert(spi_clock_for(125'000'000, 62'500'000)->divisor() == 2u);
static_assert(spi_clock_for(125'000'000, 1'000'000)->divisor() == 126u);   // 992 kHz, the fastest under
static_assert(spi_clock_for(125'000'000, 10'000'000)->divisor() == 14u);   // 8.9 MHz: 2 x 7
static_assert(!spi_clock_for(125'000'000, 1'000).has_value());            // below 254 x 256
static_assert(!SpiClock{3, 0}.valid());
static_assert(spi_sck_pin(0, 18) && spi_tx_pin(0, 19) && spi_rx_pin(0, 16) && spi_cs_pin(0, 17));
static_assert(spi_sck_pin(1, 10) && spi_tx_pin(1, 11) && spi_rx_pin(1, 8) && spi_cs_pin(1, 9));
static_assert(!spi_sck_pin(1, 18) && !spi_tx_pin(0, 11));
static_assert(spi_pins_valid(0, {.sck = 18, .tx = 19, .rx = 16}));
static_assert(!spi_pins_valid(0, {.sck = 10, .tx = 19}));
static_assert(spi_config_valid({}) && !spi_config_valid({.bits = 3}) && !spi_config_valid({.bits = 17}));
static_assert(!spi_config_valid({.role = SpiRole::host, .output_disabled = true}));
static_assert((spi_cr0_of({.mode = SpiMode::mode3, .bits = 16}) & (SPI_SSPCR0_SPO_BITS | SPI_SSPCR0_SPH_BITS | SPI_SSPCR0_DSS_BITS)) ==
              (SPI_SSPCR0_SPO_BITS | SPI_SSPCR0_SPH_BITS | 15u));
static_assert((spi_cr1_of({.role = SpiRole::client, .loopback = true}) & (SPI_SSPCR1_MS_BITS | SPI_SSPCR1_LBM_BITS)) ==
              (SPI_SSPCR1_MS_BITS | SPI_SSPCR1_LBM_BITS));
static_assert(Pl022<1>::dreq_rx == Dreq::spi1_rx);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SpiPins host_pins{.sck = 18, .tx = 19, .rx = 16};
constexpr SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
using Host = SpiHost<0, host_pins>;
using DmaHost = SpiHost<0, host_pins, DmaTxEngine<4>, DmaRxEngine<5>>;
using Client = SpiClient<1, client_pins>;
static_assert(!Host::has_engines && DmaHost::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);
static_assert(Client::frames_ahead == 8u);

uint8_t buf[16];

void spi_verbs() {
    constexpr SysClock clock;
    using S = Pl022<0>;
    (void)S::reset();
    (void)S::released();
    S::hold();
    (void)S::enabled();
    S::enable(false);
    (void)S::configure({.role = SpiRole::client, .mode = SpiMode::mode1, .format = SpiFormat::ti, .bits = 12});
    S::loopback(true);
    (void)S::loopback();
    S::output_disabled(false);
    (void)S::clock();
    (void)S::bits();
    (void)S::flags();
    (void)S::tx_not_full();
    (void)S::tx_empty();
    (void)S::rx_not_empty();
    (void)S::rx_full();
    (void)S::busy();
    S::write_data(0x55);
    (void)S::read_data();
    S::flush_rx();
    S::interrupts(SpiInterrupt::rx, true);
    (void)S::interrupts();
    (void)S::raw_pending();
    (void)S::pending();
    S::clear_pending(SpiInterrupt::overrun);
    (void)S::isr();
    S::dma_requests(true, true);

    (void)Host::init(clock, 10'000'000);
    Host::rebase(12'000'000, 48'000'000);
    (void)Host::clock_for(1'000'000);
    (void)Host::sck_hz(SpiClocks::div4);
    (void)Host::max_sck_hz();
    (void)Host::ceiling_clock();
    (void)Host::reference_hz();
    Host::prime(SpiMode::mode3, SpiClocks::div8, SpiDataSize::bits16);
    Host::loopback(true);
    (void)Host::loopback();
    Host::Request r{};
    r.cs = Pin<17>::ref();
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    r.rx = lend<Lease::reply>(buf);
    r.len = 16;
    (void)Host::start(r);
    (void)Host::isr();
    (void)Host::dma_isr();
    (void)Host::status();
    (void)Host::recover();
    Host::release();
    (void)DmaHost::init(clock);
    DmaHost::Request d{};
    d.cs = Pin<17>::ref();
    d.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    d.len = 16;
    d.polled = true;
    (void)DmaHost::start(d);
    (void)DmaHost::isr();
    (void)DmaHost::dma_isr();
    (void)DmaHost::recover();
    DmaHost::release();

    (void)Client::init(clock, {.mode = SpiMode::mode0, .bits = 8, .drive_output = false});
    Client::drive_output(true);
    Client::sod(false);
    Client::enable(0xA5);
    Client::write(1);
    (void)Client::writable();
    (void)Client::poll();
    (void)Client::selected();
    (void)Client::overrun();
    Client::clear_overrun();
    (void)Client::flags();
    (void)Client::isr();
    Client::interrupts(SpiInterrupt::rx, true);
    Client::disable();
    Client::release();
}
