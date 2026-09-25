// SPI family smoke TU: the resource's verbs over the whole of RM
// chapter 20, the rate arithmetic on the TWO DIFFERENT BUSES the two
// instances hang on, the columns table 10-32 gives SPI1 and the single
// one SPI2 has, the host engine with its Request and its DMA slots, the
// client with its dark listener, and the arbiter over the whole of it.
//
// HOW MANY INSTANCES a part has is device::spi_count and nothing else
// (datasheet table 2-1): the parts below the CH32V203C8 carry SPI1
// alone. So this TU names SPI2 only through `second_spi`, which is the
// instance where there is one and SPI1 where there is not - and a neg TU
// proves that Spi<2> is refused on a part with one.
//
// WHICH COLUMN a program may name is two questions with two answers: the
// instance must HAVE the column (SPI1 has two, SPI2 has none - no remap
// field exists for it), and the package must BOND its pads. The default
// column of each instance is what this TU instantiates, because that is
// the one every part of the family can answer for.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "kernel/tenuto.hpp"
#include "util/spi_bus.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 144'000'000>;

/// The second instance where the part has one, and the first where it
/// has not: a TU that must compile on nine parts names no literal 2.
inline constexpr uint8_t second_spi = device::spi_count >= 2u ? 2u : 1u;

using One = Spi<1>;
using Two = Spi<second_spi>;

// ---- the family's own facts ------------------------------------------------

// SPI1 is the PB2 instance and SPI2 the PB1 one, which is what decides
// the gate, the reset line and - the thing that matters most here - the
// clock a BR code divides.
static_assert(spi_bus_for(1) == Bus::pb2 && spi_bus_for(2) == Bus::pb1);
static_assert(spi_gate_for(1) == rcc_pb2_spi1 && spi_gate_for(2) == rcc_pb1_spi2);
static_assert(spi_irq_for(1) == Irq::spi1 && spi_irq_for(2) == Irq::spi2);
static_assert(spi_base_for(1) == pb2_base + 0x3000 && spi_base_for(2) == pb1_base + 0x3800);
static_assert(spi_base_for(3) == 0, "this family addresses two SPI instances");

static_assert(One::number == 1 && One::bus == Bus::pb2 && One::irq == Irq::spi1);
static_assert(!One::has_i2s_prescaler, "table 20-1 lists no I2SPR for SPI1");
static_assert(Spi<1>::dma_rx_channel == 2 && Spi<1>::dma_tx_channel == 3);

// The instance count is the part's, and nothing else in the stratum says
// it (datasheet table 2-1).
static_assert(spi_present(1));
static_assert(spi_present(2) == (device::spi_count >= 2u));
static_assert(!spi_present(0) && !spi_present(3));

// The rate table is PER BUS: at 144 MHz of HCLK, SPI1 divides 144 MHz
// and SPI2 divides 72 MHz, so the same code is two frequencies.
static_assert(spi_bus_hz_at<1>(SysClock::hz) == SysClock::pclk2_hz);
static_assert(spi_bus_hz_at<2>(SysClock::hz) == SysClock::pclk1_hz);
static_assert(spi_bus_hz<1>(SysClock{}) == 144'000'000UL);
static_assert(spi_bus_hz<2>(SysClock{}) == 72'000'000UL);
static_assert(spi_sck_hz(spi_bus_hz<1>(SysClock{}), SpiClock::div2) == 72'000'000UL);
static_assert(spi_sck_hz(spi_bus_hz<2>(SysClock{}), SpiClock::div2) == 36'000'000UL);

// The whole BR ladder of 20.4.1, and the chooser's refusal.
static_assert(spi_division(SpiClock::div8) == 8 && spi_division(SpiClock::div128) == 128);
static_assert(spi_rate_for(72'000'000UL, 36'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(72'000'000UL, 300'000UL) == SpiClock::div256);
static_assert(!spi_rate_for(72'000'000UL, 281'249UL).has_value());

// The four modes, the two frame sizes.
static_assert(spi_mode_cpol(SpiMode::mode3) && spi_mode_cpha(SpiMode::mode3));
static_assert(!spi_mode_cpol(SpiMode::mode0) && !spi_mode_cpha(SpiMode::mode0));
static_assert(spi_data_bits(SpiDataSize::bits8) == 8);
static_assert(spi_frame_mask(SpiDataSize::bits16) == 0xFFFFu);

// ---- the columns (table 10-32) ---------------------------------------------

// SPI1 has two columns and SPI2 one; every signal of every column is
// named, and the remap code travels with the pads.
static_assert(spi_column_count(1) == afio_spi1_codes && spi_column_count(2) == 1);
static_assert(spi_pins_for(1, 0).nss == Pad{'A', 4} && spi_pins_for(1, 0).sck == Pad{'A', 5} &&
              spi_pins_for(1, 0).miso == Pad{'A', 6} && spi_pins_for(1, 0).mosi == Pad{'A', 7});
static_assert(spi_pins_for(1, 1).nss == Pad{'A', 15} && spi_pins_for(1, 1).sck == Pad{'B', 3} &&
              spi_pins_for(1, 1).miso == Pad{'B', 4} && spi_pins_for(1, 1).mosi == Pad{'B', 5});
static_assert(spi_pins_for(2, 0).nss == Pad{'B', 12} && spi_pins_for(2, 0).miso == Pad{'B', 14});
static_assert(spi_default_pins<1>.remap == 0u && spi_default_pins<2>.remap == 0u);
static_assert(spi1_default_pins.sck == Pad{'A', 5} && spi2_default_pins.sck == Pad{'B', 13});

// A column SPI2 has not got, a pad named twice and a link with no clock.
static_assert(!spi_pins_valid(2, SpiPins{.sck = {'B', 13}, .mosi = {'B', 15}, .remap = 1}));
static_assert(!spi_pins_valid(1, SpiPins{.sck = {'A', 5}, .miso = {'A', 5}}));
static_assert(!spi_pins_valid(1, SpiPins{.miso = {'A', 6}, .mosi = {'A', 7}}));

// afio.hpp judges the same column, by the same two questions.
static_assert(afio_remap_has_code(Remap::spi1, 0) == spi_pins_valid(1, spi_pins_for(1, 0)) ||
                  !device::has_port('A'),
              "the driver's pin check and the remap table's must agree about column 0");

// ---- the configuration words -----------------------------------------------

static_assert(spi_config_valid(SpiConfig{}));
static_assert(spi_config_valid(SpiConfig{.role = SpiRole::client,
                                         .nss = SpiNss::hardware_input}));
// CRC is a full-duplex feature (20.4.1) and a zero polynomial computes
// nothing; SSOE is a host's verb.
static_assert(!spi_config_valid(SpiConfig{.direction = SpiDirection::receive_only, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));

static_assert(spi_ctlr1_of(SpiConfig{.mode = SpiMode::mode1, .clock = SpiClock::div64,
                                     .bits = SpiDataSize::bits16, .lsb_first = true}) ==
              (spi_cpha | spi_mstr | (5u << spi_br_shift) | spi_lsbfirst | spi_ssm | spi_ssi |
               spi_dff));
static_assert(spi_ctlr1_of(SpiConfig{.nss = SpiNss::hardware_input}) ==
              (spi_mstr | (3u << spi_br_shift)));
static_assert(spi_ctlr2_of(SpiConfig{.dma_transmit = true, .dma_receive = true}) ==
              (spi_txdmaen | spi_rxdmaen));
static_assert(spi_ctlr2_enables == (spi_errie | spi_rxneie | spi_txeie));

// The compile-time rate chooser, which REFUSES rather than reports.
static_assert(SpiRateOf<SysClock::pclk1_hz, 1'000'000>::clock == SpiClock::div128);
static_assert(SpiRateOf<SysClock::pclk2_hz, 1'000'000>::clock == SpiClock::div256);

// ---- the tasks -------------------------------------------------------------

/// The plain host and client on each instance's own default column.
using Host1 = SpiHost<1>;
using Client1 = SpiClient<1>;
using Host2 = SpiHost<second_spi>;
using Client2 = SpiClient<second_spi>;

/// The engines, on the channels table 11-5 wires to each instance.
using Host1Dma = SpiHost<1, spi_default_pins<1>, DmaTxEngine<1, Spi<1>::dma_tx_channel>,
                         DmaRxEngine<1, Spi<1>::dma_rx_channel>>;
using Host2Dma = SpiHost<second_spi, spi_default_pins<second_spi>,
                         DmaTxEngine<1, Spi<second_spi>::dma_tx_channel>,
                         DmaRxEngine<1, Spi<second_spi>::dma_rx_channel>>;

static_assert(!Host1::has_engines && Host1Dma::has_engines);
static_assert(Client1::frames_ahead == 1, "one buffer, no FIFO (figure 20-1)");
static_assert(Client1::has_nss_pad == pad_bonded(spi_default_pins<1>.nss));
static_assert(Host1::number == 1 && Host2::number == second_spi);

/// The arbiter over the engine, which is what an application posts to.
using Arb = SpiBus<Host1, P, 4>;

class Sink {
public:
    using Event = SpiDone;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

using Kernel = Tenuto<P, Sink, Arb>;

uint8_t out_buf[8];
uint8_t in_buf[8];

/// Every verb of the resource, on both instances.
void resource()
{
    One::bus_clock(true);
    One::reset();
    (void)One::remap(0);
    // The template form, whose refusals are compile errors (the negs).
    One::configure<SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_input}>();
    (void)One::configure(SpiConfig{.role = SpiRole::host, .mode = SpiMode::mode2,
                                   .clock = SpiClock::div32, .bits = SpiDataSize::bits16,
                                   .lsb_first = true, .nss = SpiNss::hardware_output,
                                   .direction = SpiDirection::full_duplex, .crc = true,
                                   .crc_polynomial = 0x1021u, .dma_transmit = true,
                                   .dma_receive = true});
    One::enable();
    (void)One::enabled();
    (void)One::receive_only_host();
    One::data(SpiDataSize::bits16, 0x1234u);
    (void)One::data(SpiDataSize::bits16);
    One::data8(0x5Au);
    (void)One::data8();
    (void)One::rxne();
    (void)One::txe();
    (void)One::busy();
    (void)One::status();
    (void)One::role();
    (void)One::mode();
    (void)One::clock();
    (void)One::bits();
    One::flush_rx();
    (void)One::overrun();
    One::clear_overrun();
    (void)One::mode_fault();
    One::clear_mode_fault();
    (void)One::crc_error();
    One::clear_crc_error();
    One::crc_next();
    One::crc_polynomial(0x0007u);
    (void)One::crc_polynomial();
    (void)One::rx_crc();
    (void)One::tx_crc();
    One::software_select(true);
    One::half_duplex_output(false);
    (void)One::high_speed_read(true);
    (void)One::i2s_config_writable();
    One::dma_requests(true, true);
    One::rxne_interrupt(true);
    One::txe_interrupt(false);
    One::error_interrupt(true);
    (void)One::isr();
    (void)One::stop_receive_only(SpiDataSize::bits8);
    (void)One::disable();
    (void)One::bus_hz(SysClock{});
    One::bus_clock(false);

    Two::bus_clock(true);
    Two::reset();
    (void)Two::configure(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_input,
                                   .direction = SpiDirection::half_duplex_in});
    Two::enable();
    (void)Two::disable();
    (void)Two::bus_hz(SysClock{});
    Two::bus_clock(false);
}

/// The host engine: both instances, engined and not, every verb.
void hosts()
{
    constexpr SysClock clock{};
    (void)Host1::init(clock);
    (void)Host1::init(clock, 1'000'000UL);
    Host1::rebase(SysClock::hz);
    (void)Host1::clock_for(500'000UL);
    (void)Host1::sck_hz(SpiClock::div16);
    (void)Host1::max_sck_hz();
    (void)Host1::ceiling_clock();
    (void)Host1::reference_hz();
    (void)Host1::hclk_hz();
    Host1::pad_speed(PinSpeed::medium);
    (void)Host1::pad_speed();
    Host1::prime(SpiMode::mode3, SpiClock::div8, SpiDataSize::bits16);
    (void)Host1::bit_order(true);
    (void)Host1::lsb_first();
    (void)Host1::busy();

    Host1::Request r{};
    r.cs = Pin<'A', 4>::ref();
    r.dc = Pin<'A', 3>::ref();
    r.cs_setup_us = 2;
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.rx = lend<Lease::reply>(in_buf);
    r.len = 8;
    r.clock = SpiClock::div16;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    (void)Host1::start(r);
    (void)Host1::isr();
    (void)Host1::dma_isr();
    (void)Host1::status();
    (void)Host1::recover();
    Host1::claim_nss_pad(true);
    Host1::release();

    (void)Host1Dma::init(clock);
    (void)Host1Dma::dma_isr();
    Host1Dma::release();

    (void)Host2::init(clock);
    Host2::release();
    (void)Host2Dma::init(clock);
    Host2Dma::release();
}

/// The client: the dark listener, the one-ahead write, the select read.
void clients()
{
    constexpr SysClock clock{};
    (void)Client1::init(clock, {.mode = SpiMode::mode1, .bits = SpiDataSize::bits8,
                                .lsb_first = false, .nss = SpiNss::hardware_input,
                                .direction = SpiDirection::full_duplex, .crc = false,
                                .crc_polynomial = 0x0007u, .drive_output = false});
    Client1::drive_output(true);
    (void)Client1::output_driven();
    Client1::pad_speed(PinSpeed::slow);
    (void)Client1::pad_speed();
    Client1::enable(0xA5u);
    (void)Client1::writable();
    Client1::write(0x5Au);
    (void)Client1::poll();
    (void)Client1::selected();
    Client1::select(true);
    (void)Client1::overrun();
    Client1::clear_overrun();
    (void)Client1::crc_error();
    Client1::clear_crc_error();
    (void)Client1::status();
    (void)Client1::isr();
    Client1::rxne_interrupt(true);
    Client1::txe_interrupt(true);
    Client1::error_interrupt(true);
    (void)Client1::bits();
    (void)Client1::disable();
    Client1::release();

    (void)Client2::init(clock);
    Client2::release();
}

/// The arbiter, posted to and stepped - util/spi_bus.hpp unchanged.
void kernel()
{
    Kernel::init_all();
    Host1::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.len = 8;
    r.reply = reply_to<Sink, SpiDone>();
    post<Arb>(r);
    (void)Kernel::step();
    post<Arb>(PrepareSleep{.depth = SleepDepth::standby, .reply = {}});
    (void)Kernel::step();
}

int main()
{
    resource();
    hosts();
    clients();
    kernel();
    return 0;
}

/// The vectors an application binds: the instance's own, and the two DMA
/// channels its engines ride.
extern "C" BRIO_CH32_INTERRUPT void spi1_handler()
{
    if (Host1::isr()) {
        post<Arb>(TransferDone{Host1::status()});
    }
}
extern "C" BRIO_CH32_INTERRUPT void spi2_handler() { (void)Client2::isr(); }
extern "C" BRIO_CH32_INTERRUPT void spi_dma_tx_handler() { (void)Host1Dma::dma_isr(); }
