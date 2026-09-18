// THE IP STRATUM THROUGH THE HAZARD3 COMPILER, over a chip that is not a
// chip: brio/pl022/spi.hpp instantiated on brio/host/sim_pl022.hpp - two
// register blocks in RAM, a reset that memsets them, an interrupt
// controller that counts, pads that remember, a run-time pin that records
// which way it was driven and a busy-wait that counts instead of
// spending. The RP2040's fixture makes the same proof with one compiler;
// this one makes it with the OTHER INSTRUCTION SET'S too, which is the
// thing this target adds - a file that had smuggled in anything of a
// Cortex-M, of an NVIC or of a vendor header would fail here and nowhere
// else.
//
// The arithmetic below is judged at the rate THIS chip runs: the IP
// file's prescaler pair is pure arithmetic and owes no chip an answer, so
// the two families ask it different questions and both must be right.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

using namespace brio;

using Clock = SimPl022Clock<150'000'000>;

constexpr SimPl022Pins host_pins{.sck = 0, .tx = 1, .rx = 2};
constexpr SimPl022Pins client_pins{.sck = 4, .tx = 5, .rx = 6, .cs = 7};
static_assert(SimPl022::pins_valid(0, host_pins) && SimPl022::pins_valid(1, client_pins));
static_assert(!SimPl022::pins_valid(0, {.sck = 0, .tx = 0}));
static_assert(!SimPl022::pins_valid(2, host_pins));

using Block = Pl022Ssp<SimPl022, 1>;
using Plain = Pl022Host<SimPl022, 0, host_pins, SimPl022NoEngine, SimPl022NoEngine>;
using Engined = Pl022Host<SimPl022, 0, host_pins, SimPl022Engine<0>, SimPl022Engine<1>>;
using Client = Pl022Client<SimPl022, 1, client_pins>;

static_assert(Block::fifo_depth == 8u);
static_assert(Block::index == 1u);
static_assert(Block::dreq_tx == SimPl022Request::spi1_tx &&
              Block::dreq_rx == SimPl022Request::spi1_rx);
static_assert(!Plain::has_engines && Engined::has_engines);
static_assert(Client::frames_ahead == 8u);
static_assert(std::is_trivially_copyable_v<Plain::Request>);
static_assert(Pl022ChipClock<SimPl022, Clock>);
static_assert(Pl022ChipDelay<SimPl022>);

uint8_t buf[16];

void pl022_resource_verbs() {
    (void)Block::regs().SSPDR;
    (void)Block::irq();
    (void)Block::data_address();
    (void)Block::reset();
    (void)Block::released();
    Block::hold();
    (void)Block::enabled();
    Block::enable(false);
    (void)Block::configure({.role = SpiRole::client, .mode = SpiMode::mode1,
                            .format = SpiFormat::ti, .bits = 12});
    Block::loopback(true);
    (void)Block::loopback();
    Block::output_disabled(false);
    (void)Block::clock();
    (void)Block::bits();
    (void)Block::flags();
    (void)Block::tx_not_full();
    (void)Block::tx_empty();
    (void)Block::rx_not_empty();
    (void)Block::rx_full();
    (void)Block::busy();
    Block::write_data(0x55);
    (void)Block::read_data();
    Block::flush_rx();
    Block::interrupts(SpiInterrupt::rx, true);
    (void)Block::interrupts();
    (void)Block::raw_pending();
    (void)Block::pending();
    Block::clear_pending(SpiInterrupt::overrun);
    (void)Block::isr();
    Block::dma_requests(true, true);
}

void pl022_host_verbs() {
    constexpr Clock clock;
    (void)Plain::init(clock, 10'000'000);
    (void)Plain::init(clock);
    Plain::rebase(12'000'000, 48'000'000);
    (void)Plain::clock_for(1'000'000);
    (void)Plain::sck_hz(SpiClocks::div4);
    (void)Plain::max_sck_hz();
    (void)Plain::ceiling_clock();
    (void)Plain::reference_hz();
    Plain::prime(SpiMode::mode3, SpiClocks::div8, SpiDataSize::bits16);
    Plain::loopback(true);
    (void)Plain::loopback();
    Plain::Request r{};
    r.cs = SimPl022PinRef{3};
    r.cs_setup_us = 2;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    r.rx = lend<Lease::reply>(buf);
    r.len = 16;
    (void)Plain::start(r);
    (void)Plain::isr();
    (void)Plain::dma_isr();
    (void)Plain::status();
    (void)Plain::recover();
    Plain::release();
}

void pl022_engine_verbs() {
    constexpr Clock clock;
    (void)Engined::init(clock);
    Engined::Request d{};
    d.cs = SimPl022PinRef{3};
    d.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    d.cmd_len = 1;
    d.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buf));
    d.len = 8;
    (void)Engined::start(d);
    (void)Engined::isr();
    (void)Engined::dma_isr();
    (void)Engined::status();
    (void)Engined::recover();
    Engined::release();
}

void pl022_client_verbs() {
    constexpr Clock clock;
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

// The IP's own arithmetic and its register words, judged where no chip
// can colour them - at 150 MHz, which is this chip's clk_peri.
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(SpiClocks::div16.divisor() == 16u && SpiClocks::div256.divisor() == 256u);
static_assert(!SpiClock{3, 0}.valid() && SpiClock{2, 0}.valid());
// The ladder at 150 MHz: 75 Mbit/s at the top, 585937 at the bottom of
// the named divisors, and the client's twelfth is exact.
static_assert(spi_sck_hz(150'000'000, SpiClocks::div2) == 75'000'000u);
static_assert(spi_sck_hz(150'000'000, SpiClocks::div256) == 585'937u);
static_assert(spi_clock_for(150'000'000, 75'000'000)->divisor() == 2u);
static_assert(spi_clock_for(150'000'000, 12'500'000)->divisor() == 12u);
static_assert(spi_clock_for(150'000'000, 10'000'000)->divisor() == 16u);
static_assert(spi_clock_for(150'000'000, 1'000'000)->divisor() == 150u);
// The slowest the pair reaches from this clock is 2306 Hz (254 x 256),
// so a ceiling below that is refused rather than rounded up to it.
static_assert(spi_clock_for(150'000'000, 2'307)->divisor() == 65024u);
static_assert(!spi_clock_for(150'000'000, 2'306).has_value());
static_assert(!spi_clock_for(0, 1'000'000).has_value() &&
              !spi_clock_for(150'000'000, 0).has_value());
static_assert(spi_data_bits(SpiDataSize::bits16) == 16u &&
              spi_frame_mask(SpiDataSize::bits8) == 0x00FFu);
static_assert(spi_frame_is_halfword(SpiDataSize::bits16));
static_assert(spi_config_valid({}) && !spi_config_valid({.bits = 3}) &&
              !spi_config_valid({.bits = 17}));
static_assert(!spi_config_valid({.role = SpiRole::host, .output_disabled = true}));
static_assert(spi_config_valid({.role = SpiRole::client, .output_disabled = true}));
static_assert(spi_cr0_of({.mode = SpiMode::mode3, .bits = 16, .clock = SpiClocks::div4}) ==
              ((1u << SpiControl0::rate_lsb) | SpiControl0::cpol | SpiControl0::cpha | 15u));
static_assert(spi_cr1_of({.role = SpiRole::client, .loopback = true}) ==
              (SpiControl1::client | SpiControl1::loopback));
static_assert(SpiInterrupt::all == 0xFu);
static_assert(SpiFlag::busy == 0x10u && SpiFlag::rx_not_empty == 0x4u);
static_assert(SpiDmaControl::tx == 0x2u && SpiDmaControl::rx == 0x1u);
