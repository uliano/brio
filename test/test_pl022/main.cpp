// Host tests for the ARM PrimeCell SSP driver (brio/pl022/spi.hpp) over
// a chip that is not a chip: two register blocks in RAM
// (brio/host/sim_pl022.hpp). What is judged here is what a bench cannot
// see cheaply - the exact WORDS a configuration leaves in the block, the
// frames a pump wrote, and the ORDER of the acts of a bring-up - and
// what a bench cannot see at all: that the driver compiles and runs with
// no silicon under it. The receive half is not here, because a register
// block of plain memory never hands a frame back.
// Run with: ctest --preset host (or ctest --preset host -R test_pl022)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

using namespace brio;

namespace {

using Clock = SimPl022Clock<125'000'000>;

constexpr SimPl022Pins host_pins{.sck = 0, .tx = 1, .rx = 2};
constexpr SimPl022Pins client_pins{.sck = 4, .tx = 5, .rx = 6, .cs = 7};
constexpr uint8_t cs_pin = 3;

using Host = Pl022Host<SimPl022, 0, host_pins, SimPl022NoEngine, SimPl022NoEngine>;
using TxEngine = SimPl022Engine<0>;
using RxEngine = SimPl022Engine<1>;
using Engined = Pl022Host<SimPl022, 0, host_pins, TxEngine, RxEngine>;
using Client = Pl022Client<SimPl022, 1, client_pins>;
using Block = Pl022Ssp<SimPl022, 0>;

SimPl022Regs& regs() { return SimPl022::regs<0>(); }

void fresh() {
    SimPl022::reset_all();
    TxEngine::reset();
    RxEngine::reset();
}

uint8_t out_buf[32];
uint8_t in_buf[32];

Host::Request plain_request(uint16_t len) {
    Host::Request r{};
    r.cs = SimPl022PinRef{cs_pin};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.rx = lend<Lease::reply>(in_buf);
    r.len = len;
    return r;
}

}   // namespace

TEST_CASE("the rate arithmetic is the prescaler pair's own") {
    // The divisor is cpsdvsr x (1 + scr), the prescaler EVEN.
    CHECK(SpiClocks::div2.divisor() == 2u);
    CHECK(SpiClocks::div256.divisor() == 256u);
    CHECK_FALSE(SpiClock{3, 0}.valid());
    CHECK(SpiClock{254, 255}.divisor() == 65'024u);
    CHECK(spi_sck_hz(125'000'000, SpiClocks::div2) == 62'500'000u);

    // The chooser takes the FASTEST setting at or under the ceiling, and
    // refuses rather than rounding up past the slowest.
    const auto ten = spi_clock_for(125'000'000, 10'000'000);
    REQUIRE(ten.has_value());
    CHECK(ten->cpsdvsr == 2);
    CHECK(ten->scr == 6);
    CHECK(spi_sck_hz(125'000'000, *ten) == 8'928'571u);
    CHECK(spi_clock_for(125'000'000, 62'500'000)->divisor() == 2u);
    CHECK(spi_sck_hz(125'000'000, *spi_clock_for(125'000'000, 1'000'000)) == 992'063u);
    CHECK_FALSE(spi_clock_for(125'000'000, 1'000).has_value());
    CHECK_FALSE(spi_clock_for(0, 1'000'000).has_value());
    CHECK_FALSE(spi_clock_for(125'000'000, 0).has_value());
}

TEST_CASE("the two control words are what the configuration says") {
    // SSPCR0: SCR in 15:8, FRF in 5:4, SPO, SPH, DSS - 1 in 3:0.
    CHECK(spi_cr0_of({.mode = SpiMode::mode3, .bits = 16, .clock = SpiClocks::div4}) ==
          ((1u << 8) | (1u << 6) | (1u << 7) | 15u));
    CHECK(spi_cr0_of({.mode = SpiMode::mode2, .bits = 8, .clock = SpiClocks::div2}) ==
          ((1u << 6) | 7u));
    // A non-Motorola framing has no phase pair at all: TI frames on a
    // pulse, and the two bits mean nothing there.
    CHECK(spi_cr0_of({.mode = SpiMode::mode3, .format = SpiFormat::ti, .bits = 12,
                      .clock = SpiClocks::div2}) == ((1u << 4) | 11u));

    // SSPCR1: the role, and the two live bits.
    CHECK(spi_cr1_of({}) == 0u);
    CHECK(spi_cr1_of({.role = SpiRole::client}) == SpiControl1::client);
    CHECK(spi_cr1_of({.loopback = true}) == SpiControl1::loopback);
    CHECK(spi_cr1_of({.role = SpiRole::client, .output_disabled = true}) ==
          (SpiControl1::client | SpiControl1::output_disable));
    // And SSE is never among them: configure() writes with the port down.
    CHECK((spi_cr1_of({.role = SpiRole::client, .loopback = true}) & SpiControl1::enable) == 0u);

    // What the block has not got is refused before any register is touched.
    CHECK(spi_config_valid({}));
    CHECK_FALSE(spi_config_valid({.bits = 3}));
    CHECK_FALSE(spi_config_valid({.bits = 17}));
    CHECK_FALSE(spi_config_valid({.clock = SpiClock{3, 0}}));
    CHECK_FALSE(spi_config_valid({.role = SpiRole::host, .output_disabled = true}));
    CHECK(spi_config_valid({.role = SpiRole::client, .output_disabled = true}));
}

TEST_CASE("the resource writes three registers, and refuses while enabled") {
    fresh();
    REQUIRE(Block::reset());
    CHECK(Block::released());
    CHECK_FALSE(Block::enabled());
    CHECK(Block::tx_empty());
    CHECK(Block::tx_not_full());
    CHECK_FALSE(Block::rx_not_empty());
    CHECK_FALSE(Block::busy());

    REQUIRE(Block::configure({.mode = SpiMode::mode1, .bits = 12, .clock = SpiClocks::div8}));
    CHECK(regs().SSPCPSR == 2u);
    CHECK(regs().SSPCR0 == ((3u << 8) | (1u << 7) | 11u));
    CHECK(regs().SSPCR1 == 0u);
    CHECK(Block::clock() == SpiClocks::div8);
    CHECK(Block::bits() == 12u);

    // The live bits go under the enable; the whole configuration does not.
    Block::enable(true);
    CHECK(Block::enabled());
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    Block::loopback(true);
    CHECK(Block::loopback());
    Block::output_disabled(true);
    CHECK(regs().SSPCR1 == (SpiControl1::enable | SpiControl1::loopback | SpiControl1::output_disable));
    const uint32_t before = regs().SSPCR0;
    CHECK_FALSE(Block::configure({.bits = 8}));
    CHECK(regs().SSPCR0 == before);
    Block::enable(false);
    CHECK(regs().SSPCR1 == (SpiControl1::loopback | SpiControl1::output_disable));
    CHECK_FALSE(Block::configure({.bits = 3}));   // refused for what it is, not for the enable
    CHECK(regs().SSPCR0 == before);
}

TEST_CASE("only two of the four sources clear through the clear register") {
    fresh();
    REQUIRE(Block::reset());

    Block::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, true);
    CHECK(regs().SSPIMSC == (SpiInterrupt::rx | SpiInterrupt::rx_timeout));
    Block::interrupts(SpiInterrupt::rx, false);
    CHECK(Block::interrupts() == SpiInterrupt::rx_timeout);

    // The two FIFO levels clear by moving data, so a write of "all" puts
    // only the timeout and the overrun in SSPICR.
    Block::clear_pending(SpiInterrupt::all);
    CHECK(regs().SSPICR == (SpiInterrupt::rx_timeout | SpiInterrupt::overrun));

    // isr() answers the raised-AND-enabled sources and clears those two.
    regs().SSPICR = 0;
    regs().SSPMIS = SpiInterrupt::rx | SpiInterrupt::overrun;
    CHECK(Block::isr() == (SpiInterrupt::rx | SpiInterrupt::overrun));
    CHECK(regs().SSPICR == SpiInterrupt::overrun);

    // A masked-status word with no clearable source writes nothing.
    regs().SSPICR = 0;
    regs().SSPMIS = SpiInterrupt::rx;
    CHECK(Block::isr() == SpiInterrupt::rx);
    CHECK(regs().SSPICR == 0u);

    Block::dma_requests(true, false);
    CHECK(regs().SSPDMACR == SpiDmaControl::tx);
    Block::dma_requests(false, true);
    CHECK(regs().SSPDMACR == SpiDmaControl::rx);
}

TEST_CASE("a host bring-up leaves the block set up, in the order it promises") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock, 10'000'000));

    // The boot configuration is the CEILING, since one was asked for:
    // 8.93 MHz, eight bits, mode 0, Motorola.
    CHECK(Host::max_sck_hz() == 10'000'000u);
    REQUIRE(Host::ceiling_clock().has_value());
    CHECK(Host::ceiling_clock()->divisor() == 14u);
    CHECK(Host::reference_hz() == 125'000'000u);
    CHECK(regs().SSPCPSR == 2u);
    CHECK(regs().SSPCR0 == ((6u << 8) | 7u));
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    CHECK(regs().SSPIMSC == 0u);
    CHECK(SimPl022Interrupts::line_enabled[0]);

    // THE ORDER init() promises: the pads go over only once the
    // registers hold the idle polarity, and SSE comes after them.
    CHECK(SimPl022Bench::pad_function[host_pins.sck] == SimPl022::pad_function);
    CHECK(SimPl022Bench::pad_function[host_pins.tx] == SimPl022::pad_function);
    CHECK(SimPl022Bench::pad_function[host_pins.rx] == SimPl022::pad_function);
    CHECK(SimPl022Bench::pad_claimed_at[host_pins.sck] < SimPl022Bench::pad_claimed_at[host_pins.tx]);
    CHECK(SimPl022Bench::pad_claimed_at[host_pins.tx] < SimPl022Bench::pad_claimed_at[host_pins.rx]);
    CHECK(SimPl022Bench::pad_claimed_at[host_pins.rx] < SimPl022Bench::enabled_at);
    // A HOST'S ANSWER LINE IS PULLED, its clock and its output are not:
    // nothing may be driving MISO, and an undriven one must not read as
    // a stream of zeros.
    CHECK(SimPl022Bench::pad_pulled_up[host_pins.rx]);
    CHECK_FALSE(SimPl022Bench::pad_pulled_up[host_pins.sck]);
    CHECK_FALSE(SimPl022Bench::pad_pulled_up[host_pins.tx]);

    // A ceiling this reference rate cannot reach is refused, and says so.
    CHECK_FALSE(Host::init(clock, 1'000));
    // With no ceiling at all the boot rate is the named default.
    fresh();
    REQUIRE(Host::init(clock));
    CHECK(Host::max_sck_hz() == 0u);
    CHECK_FALSE(Host::ceiling_clock().has_value());
    CHECK(regs().SSPCR0 == ((7u << 8) | 7u));   // div16
    CHECK(Host::sck_hz(SpiClocks::div16) == 7'812'500u);
}

TEST_CASE("the ceiling clamps from below, and a no-op re-application writes nothing") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock, 10'000'000));

    // A request faster than the ceiling is served at the ceiling; a
    // slower one is served as asked.
    CHECK(Host::sck_hz(SpiClocks::div2) == 8'928'571u);
    CHECK(Host::sck_hz(SpiClocks::div64) == 1'953'125u);
    CHECK(Host::clock_for(1'000'000)->divisor() == 126u);

    // prime() re-applies only what MOVED: asking for what is already
    // applied does not cycle the enable.
    const uint32_t enabled_at = SimPl022Bench::enabled_at;
    Host::prime(SpiMode::mode0, *Host::ceiling_clock(), SpiDataSize::bits8);
    CHECK(SimPl022Bench::enabled_at == enabled_at);
    Host::prime(SpiMode::mode3, SpiClocks::div64, SpiDataSize::bits16);
    CHECK(SimPl022Bench::enabled_at > enabled_at);
    CHECK(regs().SSPCR0 == ((31u << 8) | (1u << 6) | (1u << 7) | 15u));
    CHECK(regs().SSPCR1 == SpiControl1::enable);

    // The loop-back is part of the APPLIED state, so it survives the
    // re-application a changed request costs.
    Host::loopback(true);
    CHECK(Host::loopback());
    CHECK((regs().SSPCR1 & SpiControl1::loopback) != 0u);
    Host::prime(SpiMode::mode0, SpiClocks::div16, SpiDataSize::bits8);
    CHECK((regs().SSPCR1 & SpiControl1::loopback) != 0u);
    Host::loopback(false);
    CHECK((regs().SSPCR1 & SpiControl1::loopback) == 0u);

    // A rebase moves the ceiling and the busy-wait's factor with the tree.
    Host::rebase(48'000'000, 48'000'000);
    CHECK(Host::reference_hz() == 48'000'000u);
    CHECK(Host::ceiling_clock()->divisor() == 6u);   // 8 MHz, the fastest under 10
}

TEST_CASE("a transaction asserts its select, waits its setup and fills the FIFO once") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    for (uint8_t i = 0; i < 32u; ++i) {
        out_buf[i] = static_cast<uint8_t>(0xA0u + i);
    }

    // An EMPTY request completes on the spot and never touches the wire.
    Host::Request empty = plain_request(0);
    CHECK(Host::start(empty));
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == 0u);

    // A real one: the select asserted LOW, the D/C line high (no command
    // phase), the setup time asked for, and the transmit FIFO filled to
    // its depth and no further.
    Host::Request r = plain_request(16);
    r.cs_setup_us = 5;
    r.clock = SpiClocks::div32;
    CHECK_FALSE(Host::start(r));   // it runs on the interrupt
    CHECK_FALSE(SimPl022Bench::pin_level[cs_pin]);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] != 0u);
    CHECK(SimPl022Bench::waited_us == 5u);
    CHECK(SimPl022Bench::waited_cycles == 125u);
    CHECK(regs().SSPIMSC == (SpiInterrupt::rx | SpiInterrupt::rx_timeout));
    CHECK(regs().SSPDR == out_buf[7]);   // eight frames in flight, the FIFO's depth
    CHECK(regs().SSPCR0 == ((15u << 8) | 7u));

    // The interrupt body answers nothing when nothing of its own is up.
    regs().SSPMIS = 0;
    CHECK_FALSE(Host::isr());

    // recover() releases the select FIRST and puts the block back where
    // start() is legal, with the applied configuration standing.
    CHECK(Host::recover());
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(regs().SSPCR0 == ((15u << 8) | 7u));
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    CHECK(regs().SSPIMSC == 0u);
    CHECK(SimPl022Interrupts::line_enabled[0]);

    // A request with a command phase drives D/C LOW first, and sends the
    // command frames alone - the data waits for them to come back.
    fresh();
    REQUIRE(Host::init(clock));
    Host::Request cmd = plain_request(16);
    cmd.dc = SimPl022PinRef{8};
    cmd.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    cmd.cmd_len = 2;
    CHECK_FALSE(Host::start(cmd));
    CHECK_FALSE(SimPl022Bench::pin_level[8]);
    CHECK(regs().SSPDR == out_buf[1]);   // the second command frame, and no data
}

TEST_CASE("release puts the pads back and the block into reset") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    Host::release();

    CHECK_FALSE(SimPl022Interrupts::line_enabled[0]);
    CHECK(regs().SSPIMSC == 0u);
    CHECK(regs().SSPCR1 == 0u);   // SSE down
    CHECK(SimPl022Bench::pad_function[host_pins.sck] == 0u);
    CHECK(SimPl022Bench::pad_function[host_pins.tx] == 0u);
    CHECK(SimPl022Bench::pad_function[host_pins.rx] == 0u);
    CHECK(SimPl022Bench::pad_released_at[host_pins.rx] != 0u);
    CHECK_FALSE(Block::released());
}

TEST_CASE("the engines carry the data phase, and the receive block ends it") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));

    // Both engines armed on the data register at the bring-up, and the
    // requests left DOWN until a block stands behind them.
    CHECK(TxEngine::armed == 1u);
    CHECK(RxEngine::armed == 1u);
    CHECK(regs().SSPDMACR == 0u);

    Engined::Request r{};
    r.cs = SimPl022PinRef{cs_pin};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.rx = lend<Lease::reply>(in_buf);
    r.len = 24;
    CHECK_FALSE(Engined::start(r));
    CHECK(TxEngine::blocks == 1u);
    CHECK(RxEngine::blocks == 1u);
    CHECK(TxEngine::length == 24u);
    CHECK(regs().SSPDMACR == (SpiDmaControl::tx | SpiDmaControl::rx));
    CHECK(regs().SSPIMSC == 0u);   // the pump is not in this path

    // The RECEIVE block's completion is the transaction's: the requests
    // go down, the select goes up, and the status is clean.
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(regs().SSPDMACR == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(Engined::status() == spi_ok);

    // A request with no out buffer clocks the fixed dummy cell; one with
    // no in buffer lands in the sink.
    Engined::Request blind{};
    blind.cs = SimPl022PinRef{cs_pin};
    blind.len = 8;
    CHECK_FALSE(Engined::start(blind));
    CHECK(TxEngine::fixed);
    CHECK(RxEngine::discarded);

    // A bus error on the transmit channel ends the transaction with the
    // engine's own status, and the block is thrown away - twice, since
    // the error path abandons at its own exit and again at the one all
    // failed transactions share. Abandoning is idempotent, and the count
    // is what says so.
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Engined::dma_isr());
    CHECK(TxEngine::faults == 2u);
    CHECK(Engined::status() == spi_dma_fault);
    CHECK(regs().SSPDMACR == 0u);

    // Sixteen-bit frames fall back to the pump: the engines carry bytes.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request wide{};
    wide.cs = SimPl022PinRef{cs_pin};
    wide.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    wide.len = 8;
    wide.bits = SpiDataSize::bits16;
    CHECK_FALSE(Engined::start(wide));
    CHECK(TxEngine::blocks == 0u);
    CHECK(regs().SSPIMSC == (SpiInterrupt::rx | SpiInterrupt::rx_timeout));
    // Two bytes low-first make one frame, and the FIFO took eight.
    CHECK(regs().SSPDR == static_cast<uint32_t>(out_buf[14] | (out_buf[15] << 8)));

    Engined::release();
    CHECK(TxEngine::stops == 1u);
    CHECK(RxEngine::stops == 1u);
}

TEST_CASE("a client frames on its own select pad, and may listen dark") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Client::init(clock, {.mode = SpiMode::mode1, .bits = 12}));

    SimPl022Regs& one = SimPl022::regs<1>();
    CHECK(one.SSPCR1 == SpiControl1::client);
    // A client's own Config names no rate - the host sets the pace - so
    // SCR is whatever SpiConfig's default carries, and only the phase
    // pair and the width are this side's business.
    CHECK(one.SSPCR0 == ((7u << 8) | (1u << 7) | 11u));
    CHECK(one.SSPIMSC == 0u);
    CHECK(SimPl022Interrupts::line_enabled[1]);
    CHECK(Client::frames_ahead == 8u);

    // THE ORDER init() promises: the inputs first, the answer line last.
    CHECK(SimPl022Bench::pad_claimed_at[client_pins.sck] < SimPl022Bench::pad_claimed_at[client_pins.rx]);
    CHECK(SimPl022Bench::pad_claimed_at[client_pins.rx] < SimPl022Bench::pad_claimed_at[client_pins.cs]);
    CHECK(SimPl022Bench::pad_claimed_at[client_pins.cs] < SimPl022Bench::pad_claimed_at[client_pins.tx]);
    // THE SELECT IS PULLED, THE COMMAND LINE IS NOT: an unwired select
    // must read not-selected, while the host drives the command line
    // whenever it matters.
    CHECK(SimPl022Bench::pad_pulled_up[client_pins.cs]);
    CHECK_FALSE(SimPl022Bench::pad_pulled_up[client_pins.rx]);

    // selected() is a live read of that same pad, active low.
    SimPl022Bench::pad_level[client_pins.cs] = true;
    CHECK_FALSE(Client::selected());
    SimPl022Bench::pad_level[client_pins.cs] = false;
    CHECK(Client::selected());

    // enable() puts the first answer in the FIFO with SSE already up, so
    // the host's first clock has something to shift.
    Client::enable(0x5A5);
    CHECK((one.SSPCR1 & SpiControl1::enable) != 0u);
    CHECK(one.SSPDR == 0x5A5u);
    CHECK(Client::writable());

    // THE DARK LISTENER IS THE PAD: drive_output(false) gives the answer
    // line back, and SOD is left as the block's own bit.
    Client::drive_output(false);
    CHECK(SimPl022Bench::pad_function[client_pins.tx] == 0u);
    CHECK(SimPl022Bench::pad_released_at[client_pins.tx] != 0u);
    CHECK((one.SSPCR1 & SpiControl1::output_disable) == 0u);
    Client::sod(true);
    CHECK((one.SSPCR1 & SpiControl1::output_disable) != 0u);
    Client::drive_output(true);
    CHECK(SimPl022Bench::pad_function[client_pins.tx] == SimPl022::pad_function);

    // The overrun is the one flag a client watches, and it clears by
    // writing one.
    one.SSPRIS = SpiInterrupt::overrun;
    CHECK(Client::overrun());
    Client::clear_overrun();
    CHECK(one.SSPICR == SpiInterrupt::overrun);

    // A frame is masked to the configured width, and nothing is offered
    // while the receive FIFO reads empty.
    CHECK_FALSE(Client::poll().has_value());

    Client::disable();
    CHECK((one.SSPCR1 & SpiControl1::enable) == 0u);
    Client::release();
    CHECK_FALSE(SimPl022Interrupts::line_enabled[1]);
    CHECK(SimPl022Bench::pad_function[client_pins.cs] == 0u);
}
