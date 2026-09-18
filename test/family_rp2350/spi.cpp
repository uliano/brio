// SPI family smoke TU: this chip's half of the PL022 contract - the pin
// table of 9.4 (ONE function column, unlike the UART's two), the chip
// traits against the IP stratum's concept, and every verb of the
// resource, of the host with both engine slots empty and both filled,
// and of the client. Compiled for both packages and both architectures,
// so the same source proves that the table is the die's and stops at
// GP47, that the traits satisfy `Pl022Chip` whichever interrupt
// controller `Irq` names, and that the public aliases instantiate over
// each.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"

using namespace brio;

// ---- the table of 9.4 (table 645) -------------------------------------------

// The groups of four cycle SPI0, SPI0, SPI1, SPI1 over the bank: the
// instance is bit 3 of the pad number.
static_assert(spi_pad_instance(0) == 0u && spi_pad_instance(7) == 0u &&
              spi_pad_instance(8) == 1u && spi_pad_instance(15) == 1u &&
              spi_pad_instance(16) == 0u && spi_pad_instance(40) == 1u &&
              spi_pad_instance(47) == 1u);

// The four signals in the table's order: RX, CSn, SCK, TX.
static_assert(spi_rx_pin(0, 0) && spi_cs_pin(0, 1) && spi_sck_pin(0, 2) && spi_tx_pin(0, 3));
static_assert(spi_rx_pin(0, 4) && spi_cs_pin(0, 5) && spi_sck_pin(0, 6) && spi_tx_pin(0, 7));
static_assert(spi_rx_pin(1, 8) && spi_cs_pin(1, 9) && spi_sck_pin(1, 10) && spi_tx_pin(1, 11));
static_assert(spi_rx_pin(1, 12) && spi_cs_pin(1, 13) && spi_sck_pin(1, 14) && spi_tx_pin(1, 15));
// SPI0's SCK pads: 2, 6, 18, 22, 34, 38 - and SPI1's: 10, 14, 26, 30, 42, 46.
static_assert(spi_sck_pin(0, 18) && spi_sck_pin(0, 22) && spi_sck_pin(0, 34) && spi_sck_pin(0, 38));
static_assert(spi_sck_pin(1, 26) && spi_sck_pin(1, 30) && spi_sck_pin(1, 42) && spi_sck_pin(1, 46));
// The three QFN-80-only groups, which no QFN-60 bonds.
static_assert(spi_rx_pin(1, 28) && spi_tx_pin(1, 31) && spi_rx_pin(0, 32) && spi_tx_pin(0, 39) &&
              spi_rx_pin(1, 40) && spi_cs_pin(1, 45) && spi_tx_pin(1, 47));
// No pad is both instances', and no signal is another signal's.
static_assert(!spi_sck_pin(1, 18) && !spi_tx_pin(0, 11) && !spi_sck_pin(0, 3) &&
              !spi_rx_pin(0, 2) && !spi_cs_pin(1, 8));
// Nothing above the bank is an SPI pad on any package.
static_assert(!spi_rx_pin(0, 48) && !spi_tx_pin(1, 200));

constexpr SpiPins host_pins{.sck = 18, .tx = 19, .rx = 16};
constexpr SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
constexpr SpiPins write_only_pins{.sck = 2, .tx = 3};
static_assert(spi_pins_valid(0, host_pins) && spi_pins_valid(1, client_pins) &&
              spi_pins_valid(0, write_only_pins));
// A pin set of the other instance, a signal on the wrong pad, and a
// third instance.
static_assert(!spi_pins_valid(1, host_pins));
static_assert(!spi_pins_valid(0, {.sck = 10, .tx = 19}));
static_assert(!spi_pins_valid(0, {.sck = 18, .tx = 17}));
static_assert(!spi_pins_valid(0, {.tx = 19, .rx = 16}));   // no SCK
static_assert(!spi_pins_valid(2, host_pins));

// ---- the traits ------------------------------------------------------------

static_assert(Pl022Chip<Rp2350Pl022>);
static_assert(Rp2350Pl022::instances == 2u);
static_assert(Rp2350Pl022::fifo_depth == 8u);
static_assert(Rp2350Pl022::no_pad == 0xFFu);
static_assert(Rp2350Pl022::irq<0>() == SPI0_IRQ_IRQn && Rp2350Pl022::irq<1>() == SPI1_IRQ_IRQn);
static_assert(static_cast<int>(SPI0_IRQ_IRQn) == 31 && static_cast<int>(SPI1_IRQ_IRQn) == 32);
static_assert(Rp2350Pl022::reset_bit<0>() == ResetBlock::spi0 &&
              Rp2350Pl022::reset_bit<1>() == ResetBlock::spi1);
// 12.6.4.1's rows, which are NOT the RP2040's 16..19.
static_assert(Rp2350Pl022::tx_request<0>() == Dreq::spi0_tx &&
              Rp2350Pl022::rx_request<0>() == Dreq::spi0_rx &&
              Rp2350Pl022::tx_request<1>() == Dreq::spi1_tx &&
              Rp2350Pl022::rx_request<1>() == Dreq::spi1_rx);
static_assert(static_cast<uint8_t>(Dreq::spi0_tx) == 24u &&
              static_cast<uint8_t>(Dreq::spi1_rx) == 27u);
static_assert(Rp2350Pl022::sck_pad(host_pins) == 18u && Rp2350Pl022::tx_pad(host_pins) == 19u &&
              Rp2350Pl022::rx_pad(host_pins) == 16u &&
              Rp2350Pl022::cs_pad(host_pins) == Rp2350Pl022::no_pad);
static_assert(Rp2350Pl022::cs_pad(client_pins) == 9u);
// ONE function column: F1 carries every SPI signal of both packages.
static_assert(Rp2350Pl022::pad_function == PinFunction::spi &&
              static_cast<uint8_t>(PinFunction::spi) == 1u);
// The host's answer line and the client's select come up pulled UP (the
// pads of this chip reset pulled DOWN); the client's command line, which
// the host drives, takes no pull.
static_assert(Rp2350Pl022::host_rx_pad_config().pull == PinPull::up &&
              Rp2350Pl022::cs_pad_config().pull == PinPull::up &&
              Rp2350Pl022::client_rx_pad_config().pull == PinPull::none &&
              Rp2350Pl022::tx_pad_config().pull == PinPull::none &&
              Rp2350Pl022::sck_pad_config().pull == PinPull::none);
static_assert(Rp2350Pl022::engines_distinct<NoDmaEngine, NoDmaEngine>());
static_assert(Rp2350Pl022::engines_distinct<DmaTxEngine<4>, DmaRxEngine<5>>());
static_assert(!Rp2350Pl022::engines_distinct<DmaTxEngine<4>, DmaRxEngine<4>>());
// The busy-wait's factor carries no rate here, only whether there was one.
static_assert(Rp2350Pl022::delay_rate(150'000'000).usable && !Rp2350Pl022::delay_rate(0).usable);
static_assert(Pl022ChipDelay<Rp2350Pl022>);

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
using PeriClock = Clock<ClockSource::pll, 150'000'000UL, 12'000'000UL, PeriSource::crystal>;
static_assert(Pl022ChipClock<Rp2350Pl022, SysClock>);
// SSPCLK is clk_peri, which follows the peri source and not clk_sys.
static_assert(Rp2350Pl022::sspclk_hz(SysClock{}) == 150'000'000UL);
static_assert(Rp2350Pl022::sspclk_hz(PeriClock{}) == 12'000'000UL);

// The rate arithmetic AT THIS CHIP'S clk_peri: the top of the ladder is
// 75 Mbit/s (CPSDVSR 2, SCR 0), and the client's ceiling is / 12.
static_assert(spi_sck_hz(150'000'000, SpiClocks::div2) == 75'000'000u);
static_assert(spi_clock_for(150'000'000, 75'000'000)->divisor() == 2u);
static_assert(spi_clock_for(150'000'000, 12'500'000)->divisor() == 12u);
static_assert(spi_clock_for(150'000'000, 10'000'000)->divisor() == 16u);
static_assert(spi_clock_for(150'000'000, 1'000'000)->divisor() == 150u);
static_assert(!spi_clock_for(150'000'000, 1'000).has_value());   // below 254 x 256

// ---- the public names ------------------------------------------------------

using S0 = Pl022<0>;
using S1 = Pl022<1>;
using Host = SpiHost<0, host_pins>;
using DmaHost = SpiHost<0, host_pins, DmaTxEngine<4>, DmaRxEngine<5>>;
using WriteOnly = SpiHost<0, write_only_pins>;
using Client = SpiClient<1, client_pins>;

static_assert(S0::index == 0u && S1::index == 1u);
static_assert(S0::fifo_depth == 8u);
static_assert(S1::dreq_tx == Dreq::spi1_tx && S1::dreq_rx == Dreq::spi1_rx);
static_assert(!Host::has_engines && DmaHost::has_engines);
static_assert(std::is_trivially_copyable_v<Host::Request>);
static_assert(Client::frames_ahead == 8u);

uint8_t buf[32];
uint8_t cmd[2];

void spi_resource_verbs() {
    (void)S1::regs().SSPDR;
    (void)S1::irq();
    (void)S1::data_address();
    (void)S1::reset();
    (void)S1::released();
    S1::hold();
    (void)S1::enabled();
    S1::enable(false);
    (void)S1::configure({.role = SpiRole::client, .mode = SpiMode::mode1,
                         .format = SpiFormat::ti, .bits = 12});
    S1::loopback(true);
    (void)S1::loopback();
    S1::output_disabled(false);
    (void)S1::clock();
    (void)S1::bits();
    (void)S1::flags();
    (void)S1::tx_not_full();
    (void)S1::tx_empty();
    (void)S1::rx_not_empty();
    (void)S1::rx_full();
    (void)S1::busy();
    S1::write_data(0x55);
    (void)S1::read_data();
    S1::flush_rx();
    S1::interrupts(SpiInterrupt::rx, true);
    (void)S1::interrupts();
    (void)S1::raw_pending();
    (void)S1::pending();
    S1::clear_pending(SpiInterrupt::overrun);
    (void)S1::isr();
    S1::dma_requests(true, true);
    // The PrimeCell identity, which the device description states as the
    // registers' own reset values.
    (void)S1::regs().SSPPERIPHID0;
    (void)S1::regs().SSPPCELLID3;
}

void spi_host_verbs() {
    constexpr SysClock clock;
    (void)Host::init(clock, 10'000'000);
    (void)Host::init(clock);
    Host::rebase(SysClock::pclk_hz, SysClock::hz);
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
    r.dc = Pin<22>::ref();
    r.cs_setup_us = 2;
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    r.rx = lend<Lease::reply>(buf);
    r.len = 32;
    (void)Host::start(r);
    (void)Host::isr();
    (void)Host::dma_isr();
    (void)Host::status();
    (void)Host::recover();
    Host::release();

    // A write-only bus: SCK and TX, no answer line.
    (void)WriteOnly::init(clock);
    WriteOnly::release();
}

void spi_engine_verbs() {
    constexpr SysClock clock;
    (void)DmaHost::init(clock);
    DmaHost::Request d{};
    d.cs = Pin<17>::ref();
    d.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    d.len = 32;
    d.polled = true;
    (void)DmaHost::start(d);
    (void)DmaHost::isr();
    (void)DmaHost::dma_isr();
    (void)DmaHost::status();
    (void)DmaHost::recover();
    DmaHost::release();
}

void spi_client_verbs() {
    constexpr SysClock clock;
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
    Client::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, true);
    Client::disable();
    Client::release();
}
