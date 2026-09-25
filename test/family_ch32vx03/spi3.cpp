// mcu: ch32v303rc ch32v303vc
// SPI3 family smoke TU: the third synchronous port of the CH32V303RC and
// VC - on PB1 beside SPI2, its vector at the class's own tail (67), its
// two requests on DMA2 (channels 1 and 2, table 11-3), its two columns
// behind AFIO_PCFR1's bit 28 (table 10-33) - through the resource, the
// host engine with and without its engines, the client, and the arbiter;
// and the high-speed read's second mode, which only SPI1's bus can ask
// for.
//
// WHY THESE TWO PARTS: table 2-1-1 gives three SPI to the 256 KB CH32V303
// and two to the 128 KB ones; the CH32V203 has two at most. A neg TU
// proves SPI3 refused everywhere else.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "kernel/tenuto.hpp"
#include "util/spi_bus.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 144'000'000>;

using Three = Spi<3>;

static_assert(spi_present(3) && Three::number == 3);
static_assert(Three::bus == Bus::pb1 && static_cast<uint8_t>(Three::irq) == 67);
static_assert(Three::dma_rx_slot == DmaSlot{2, 1} && Three::dma_tx_slot == DmaSlot{2, 2});
static_assert(Three::has_i2s_prescaler);
// SPI3 divides PB1, as SPI2 does: at 144 MHz of HCLK its ladder tops at
// 36 MHz where SPI1's tops at 72.
static_assert(spi_bus_hz<3>(SysClock{}) == 72'000'000UL);
static_assert(spi_sck_hz(spi_bus_hz<3>(SysClock{}), SpiClock::div2) == 36'000'000UL);
// Its two columns - the first is SPI1's second, pad for pad.
static_assert(spi_column_count(3) == 2);
static_assert(spi_pins_for(3, 0).sck == spi_pins_for(1, 1).sck &&
              spi_pins_for(3, 0).mosi == spi_pins_for(1, 1).mosi);
static_assert(spi_pins_valid(3, spi3_default_pins) && spi_pins_valid(3, spi_pins_for(3, 1)));
static_assert(afio_remap_has_code(Remap::spi3, 0) && afio_remap_has_code(Remap::spi3, 1));

using Host3 = SpiHost<3>;
using Host3Moved = SpiHost<3, spi_pins_for(3, 1)>;
using Host3Dma = SpiHost<3, spi3_default_pins, DmaTxEngine<2, 2>, DmaRxEngine<2, 1>>;
using Client3 = SpiClient<3>;
using Arb = SpiBus<Host3, P, 4>;

static_assert(Host3Dma::has_engines && !Host3::has_engines);
static_assert(Client3::frames_ahead == 1 && Client3::has_nss_pad);

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

void resource() {
    Three::bus_clock(true);
    Three::reset();
    (void)Three::remap(1);
    (void)Three::remap(0);
    (void)Three::configure(SpiConfig{.role = SpiRole::host, .clock = SpiClock::div8,
                                     .bits = SpiDataSize::bits16});
    Three::configure<SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_input}>();
    Three::configure_high_speed_read<SpiConfig{.clock = SpiClock::div2}>();
    (void)Three::high_speed_read(false);
    Three::enable();
    Three::data(SpiDataSize::bits16, 0xBEEFu);
    (void)Three::data(SpiDataSize::bits16);
    (void)Three::status();
    (void)Three::isr();
    (void)Three::i2s_config_writable();
    Three::dma_requests(true, true);
    (void)Three::disable();
    Three::bus_clock(false);

    // HSRXEN2: the class's, asked of the die; only SPI1's bus reaches the
    // 120 MHz it wants, so SPI3's answer is always no.
    (void)Spi<1>::high_speed_read2(true, SysClock::pclk2_hz);
    (void)Spi<1>::high_speed_read2(false, SysClock::pclk2_hz);
    (void)Three::high_speed_read2(true, SysClock::pclk1_hz);
}

void tasks() {
    constexpr SysClock clock{};
    (void)Host3::init(clock, 4'000'000UL);
    Host3::Request r{};
    r.cs = Pin<'A', 15>::ref();
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.rx = lend<Lease::reply>(in_buf);
    r.len = 8;
    r.polled = true;
    (void)Host3::start(r);
    (void)Host3::isr();
    (void)Host3::recover();
    Host3::release();
    (void)Host3Moved::init(clock);
    Host3Moved::release();
    (void)Host3Dma::init(clock);
    (void)Host3Dma::dma_isr();
    Host3Dma::release();

    (void)Client3::init(clock, {.mode = SpiMode::mode3});
    Client3::enable(0x5Au);
    (void)Client3::poll();
    Client3::write(0xA5u);
    (void)Client3::selected();
    Client3::drive_output(false);
    Client3::release();

    Kernel::init_all();
    r.polled = false;
    r.reply = reply_to<Sink, SpiDone>();
    post<Arb>(r);
    (void)Kernel::step();
}

extern "C" BRIO_CH32_INTERRUPT void spi3_handler() { (void)Host3::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel1_handler() { (void)Host3Dma::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel2_handler() { (void)Host3Dma::dma_isr(); }
