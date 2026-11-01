// Host tests for the ARM PrimeCell SSP driver (brio/pl022/spi.hpp) over
// a chip that is not a chip: two register blocks in RAM
// (brio/host/sim_pl022.hpp). What is judged here is what a bench cannot
// see cheaply - the exact WORDS a configuration leaves in the block, the
// way each request was run, and the ORDER of the acts of a bring-up - and
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
using WideTx = SimPl022Engine<2, uint16_t>;
using WideRx = SimPl022Engine<3, uint16_t>;
using WideHost = Pl022Host<SimPl022, 0, host_pins, WideTx, WideRx>;
using Client = Pl022Client<SimPl022, 1, client_pins>;
using Block = Pl022Ssp<SimPl022, 0>;

SimPl022Regs& regs() { return SimPl022::regs<0>(); }

void fresh() {
    SimPl022::reset_all();
    TxEngine::reset();
    RxEngine::reset();
    WideTx::reset();
    WideRx::reset();
}

alignas(4) uint8_t out_buf[32];
alignas(4) uint8_t in_buf[32];

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
    // The host arms no source of the instance's line, and leaves the line
    // itself disabled: that vector is a client's.
    CHECK_FALSE(SimPl022Interrupts::line_enabled[0]);

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

/// Let the polled loops run to their end: SSPSR says a frame waits, and
/// SSPDR reads back whatever was written last (brio/host/sim_pl022.hpp).
/// Nothing that drains the receive FIFO may run meanwhile.
void frames_come_back(bool on) {
    if (on) {
        regs().SSPSR = regs().SSPSR | SpiFlag::rx_not_empty;
    } else {
        regs().SSPSR = regs().SSPSR & ~SpiFlag::rx_not_empty;
    }
}

TEST_CASE("an asynchronous request on a host with no engines is refused, and touches nothing") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    static_assert(!Host::has_dma && Engined::has_dma);
    for (uint8_t i = 0; i < 32u; ++i) {
        out_buf[i] = static_cast<uint8_t>(0xA0u + i);
    }
    const uint32_t cr0 = regs().SSPCR0;

    // Asynchronous is DMA, and there are no engines: start() answers at
    // once with spi_rejected, before the block, the select, the D/C or the
    // setup time is touched - even for a request whose mode would flip
    // the clock's idle level, and even for an empty one.
    Host::Request r = plain_request(16);
    r.dc = SimPl022PinRef{8};
    r.cs_setup_us = 5;
    r.mode = SpiMode::mode3;
    r.clock = SpiClocks::div2;
    CHECK(Host::start(r));
    CHECK(Host::status() == spi_rejected);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == 0u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] == 0u);
    CHECK(SimPl022Bench::pin_clear_at[8] == 0u);
    CHECK(SimPl022Bench::pin_set_at[8] == 0u);
    CHECK(SimPl022Bench::waited_us == 0u);
    CHECK(regs().SSPCR0 == cr0);
    CHECK(regs().SSPDR == 0u);
    Host::Request empty = plain_request(0);
    CHECK(Host::start(empty));
    CHECK(Host::status() == spi_rejected);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == 0u);
}

TEST_CASE("a polled transaction asserts its select, waits its setup and runs inside start()") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    for (uint8_t i = 0; i < 32u; ++i) {
        out_buf[i] = static_cast<uint8_t>(0xA0u + i);
    }

    // An EMPTY polled request completes on the spot and never touches the
    // wire.
    Host::Request empty = plain_request(0);
    empty.polled = true;
    CHECK(Host::start(empty));
    CHECK(Host::status() == spi_ok);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == 0u);

    // A real one: the select asserted LOW and released at the end, the
    // setup time asked for, the rate applied, and no interrupt source armed.
    Host::Request r = plain_request(16);
    r.cs_setup_us = 5;
    r.clock = SpiClocks::div32;
    r.polled = true;
    frames_come_back(true);
    CHECK(Host::start(r));
    frames_come_back(false);
    CHECK(Host::status() == spi_ok);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] != 0u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] > SimPl022Bench::pin_clear_at[cs_pin]);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(SimPl022Bench::waited_us == 5u);
    CHECK(SimPl022Bench::waited_cycles == 125u);
    CHECK(regs().SSPIMSC == 0u);
    CHECK(regs().SSPDR == out_buf[15]);   // the last frame written
    CHECK(regs().SSPCR0 == ((15u << 8) | 7u));

    // A loop whose frames never come back runs out of its budget:
    // spi_stalled, as every family's host answers it, never the engines'
    // spi_dma_fault; the select released, the block left for recover(). The
    // drain is not judged here: a receive FIFO of plain memory holds nothing
    // to drain (brio/host/sim_pl022.hpp); the suites' letter c judges it on
    // silicon.
    const uint32_t stalled_from = SimPl022Bench::pin_clear_at[cs_pin];
    CHECK(Host::start(r));
    CHECK(Host::status() == spi_stalled);
    CHECK(Host::status() != spi_dma_fault);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] > stalled_from);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] > SimPl022Bench::pin_clear_at[cs_pin]);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(regs().SSPIMSC == 0u);

    // recover() releases the select FIRST and puts the block back where
    // start() is legal, with the applied configuration standing and the
    // line still the host's to leave alone.
    CHECK(Host::recover());
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(regs().SSPCR0 == ((15u << 8) | 7u));
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    CHECK(regs().SSPIMSC == 0u);
    CHECK_FALSE(SimPl022Interrupts::line_enabled[0]);

    // A request with a command phase drives D/C LOW first and raises it
    // between the phases.
    fresh();
    REQUIRE(Host::init(clock));
    Host::Request cmd = plain_request(16);
    cmd.dc = SimPl022PinRef{8};
    cmd.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    cmd.cmd_len = 2;
    cmd.polled = true;
    frames_come_back(true);
    CHECK(Host::start(cmd));
    frames_come_back(false);
    CHECK(Host::status() == spi_ok);
    CHECK(SimPl022Bench::pin_clear_at[8] != 0u);
    CHECK(SimPl022Bench::pin_set_at[8] > SimPl022Bench::pin_clear_at[8]);
    CHECK(SimPl022Bench::pin_set_at[8] < SimPl022Bench::pin_set_at[cs_pin]);
}

// cmd_len + len of 65536, the first sum a 16-bit total cannot hold: it would read
// as an empty request, spi_ok with nothing on the wire. One command frame and
// 65535 data frames, the most a Request's fields carry with a command phase of
// one; 65536 frames of 8 bits at clk_peri / 2 are 1 048 576 cycles of wire, so a
// request that really ran had to walk both buffers to their last frame.
alignas(4) uint8_t wrap_out[65535];
alignas(4) uint8_t wrap_in[65535];

TEST_CASE("a request of 65536 frames in all is a request, not an empty one") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    out_buf[0] = 0xC1;
    wrap_out[0] = 0x11;
    wrap_out[65534] = 0x5A;
    wrap_in[65534] = 0;

    Host::Request r{};
    r.cs = SimPl022PinRef{cs_pin};
    r.dc = SimPl022PinRef{8};
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(wrap_out));
    r.rx = lend<Lease::reply>(wrap_in);
    r.len = 65535;
    r.clock = SpiClocks::div16;
    r.polled = true;
    frames_come_back(true);
    CHECK(Host::start(r));
    frames_come_back(false);
    CHECK(Host::status() == spi_ok);
    // The select framed it and D/C rose between the phases.
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] != 0u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] > SimPl022Bench::pin_clear_at[cs_pin]);
    CHECK(SimPl022Bench::pin_set_at[8] > SimPl022Bench::pin_clear_at[8]);
    // The last frame written is the out buffer's last, and it read back into
    // the in buffer's last byte: the loop ran the whole length.
    CHECK(regs().SSPDR == 0x5Au);
    CHECK(wrap_in[65534] == 0x5Au);

    // The same sum asynchronously on a host with engines goes to them: at /16 the
    // one command frame repays a block of its own (152 cycles of frame against the
    // fixture's budget), chained to the data block of 65535 from the vector.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request a{};
    a.cs = SimPl022PinRef{cs_pin};
    a.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    a.cmd_len = 1;
    a.tx = lend<Lease::reply>(static_cast<const uint8_t*>(wrap_out));
    a.rx = lend<Lease::reply>(wrap_in);
    a.len = 65535;
    a.clock = SpiClocks::div16;
    CHECK_FALSE(Engined::start(a));
    CHECK(TxEngine::length == 1u);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK_FALSE(Engined::dma_isr());
    CHECK(TxEngine::length == 65535u);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(Engined::status() == spi_ok);
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

TEST_CASE("a host whose pin set names cs drives the block's own frame signal there") {
    // THE SELECTS (the file header): the pad goes to the block after the
    // three others and before SSE, and goes back at release - while a
    // host with no cs in its pin set never touches that pad.
    constexpr SimPl022Pins framed_pins{.sck = 8, .tx = 9, .rx = 10, .cs = 11};
    using Framed = Pl022Host<SimPl022, 0, framed_pins, SimPl022NoEngine, SimPl022NoEngine>;
    static_assert(Framed::frame_select && !Host::frame_select);
    fresh();
    constexpr Clock clock;
    REQUIRE(Framed::init(clock));
    CHECK(SimPl022Bench::pad_function[framed_pins.cs] == SimPl022::pad_function);
    CHECK(SimPl022Bench::pad_claimed_at[framed_pins.rx] < SimPl022Bench::pad_claimed_at[framed_pins.cs]);
    CHECK(SimPl022Bench::pad_claimed_at[framed_pins.cs] < SimPl022Bench::enabled_at);
    Framed::release();
    CHECK(SimPl022Bench::pad_function[framed_pins.cs] == 0u);
    CHECK(SimPl022Bench::pad_released_at[framed_pins.cs] != 0u);

    fresh();
    REQUIRE(Host::init(clock));
    CHECK(SimPl022Bench::pad_function[cs_pin] == 0u);
    CHECK(SimPl022Bench::pad_claimed_at[cs_pin] == 0u);
}

TEST_CASE("the engines carry the data phase, and the receive block ends it") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));

    // Both engines armed on the data register at the bring-up, and the
    // requests left DOWN until a block stands behind them. The transmit
    // engine reports a bus error and nothing else: the receive block's
    // completion is the transaction's, ONE interrupt a transaction.
    CHECK(TxEngine::armed == 1u);
    CHECK(RxEngine::armed == 1u);
    CHECK(TxEngine::errors_only);
    CHECK_FALSE(RxEngine::errors_only);
    // The levels are the rule's (docs/design/dma.md): the receive engine
    // high, its FIFO the one that overruns; the transmit engine normal.
    CHECK(RxEngine::high);
    CHECK_FALSE(TxEngine::high);
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
    CHECK(TxEngine::beat == 1u);
    CHECK(regs().SSPDMACR == (SpiDmaControl::tx | SpiDmaControl::rx));
    CHECK(regs().SSPIMSC == 0u);   // the block's own line is not in this path

    // The RECEIVE block's completion is the transaction's: the requests
    // go down, the select goes up, the status is clean - and the transmit
    // block, which raised nothing, is completed by that proof.
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(regs().SSPDMACR == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(Engined::status() == spi_ok);
    CHECK_FALSE(TxEngine::busy());

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

    // THE SHORT ASYNCHRONOUS REQUEST (pl022/spi.hpp's arithmetic over
    // SimPl022's figures): at clk / 2 in mode 3 a byte frame is 16 cycles,
    // so a read rides the engines from 5 frames and a write from 7.
    fresh();
    REQUIRE(Engined::init(clock));
    CHECK(Engined::async_min_frames(SpiClocks::div2, SpiMode::mode3, SpiDataSize::bits8, true) == 5u);
    CHECK(Engined::async_min_frames(SpiClocks::div2, SpiMode::mode3, SpiDataSize::bits8, false) == 7u);
    // At clk / 16 in mode 0 a frame is 152 cycles: one frame repays them.
    CHECK(Engined::async_min_frames(SpiClocks::div16, SpiMode::mode0, SpiDataSize::bits8, true) == 1u);
    Engined::Request brief{};
    brief.cs = SimPl022PinRef{cs_pin};
    brief.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    brief.rx = lend<Lease::reply>(in_buf);
    brief.clock = SpiClocks::div2;
    brief.mode = SpiMode::mode3;
    // One frame short: polled inside start(), no block, no interrupt.
    brief.len = 4;
    frames_come_back(true);
    CHECK(Engined::start(brief));
    frames_come_back(false);
    CHECK(Engined::status() == spi_ok);
    CHECK(TxEngine::blocks == 0u);
    CHECK(regs().SSPDMACR == 0u);
    CHECK(regs().SSPIMSC == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    // At the count: the engines.
    brief.len = 5;
    CHECK_FALSE(Engined::start(brief));
    CHECK(TxEngine::blocks == 1u);
    CHECK(TxEngine::length == 5u);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    // A write of six is short, of seven is not.
    Engined::Request write = brief;
    write.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    write.len = 6;
    frames_come_back(true);
    CHECK(Engined::start(write));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 1u);
    write.len = 7;
    CHECK_FALSE(Engined::start(write));
    CHECK(TxEngine::blocks == 2u);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());

    // A COMMAND PHASE THAT REPAYS A BLOCK OF ITS OWN (8 frames written,
    // 128 cycles against 100) rides the engines first, D/C low, its echo
    // discarded; its completion raises D/C and launches the data block
    // from the vector - one interrupt a block, the transaction ending on
    // the second.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request two{};
    two.cs = SimPl022PinRef{cs_pin};
    two.dc = SimPl022PinRef{8};
    two.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    two.cmd_len = 8;
    two.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf + 8));
    two.rx = lend<Lease::reply>(in_buf);
    two.len = 16;
    two.clock = SpiClocks::div2;
    two.mode = SpiMode::mode3;
    CHECK_FALSE(Engined::start(two));
    CHECK(TxEngine::blocks == 1u);
    CHECK(TxEngine::length == 8u);
    CHECK(RxEngine::discarded);
    CHECK_FALSE(SimPl022Bench::pin_level[8]);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK_FALSE(Engined::dma_isr());
    CHECK(SimPl022Bench::pin_level[8]);
    CHECK(TxEngine::blocks == 2u);
    CHECK(TxEngine::length == 16u);
    CHECK_FALSE(RxEngine::discarded);
    CHECK_FALSE(SimPl022Bench::pin_level[cs_pin]);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(Engined::status() == spi_ok);
    // A command phase too short for a block of its own (one frame) runs
    // polled inside start(), and the data block follows it there.
    two.cmd_len = 1;
    frames_come_back(true);
    CHECK_FALSE(Engined::start(two));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 3u);
    CHECK(TxEngine::length == 16u);
    CHECK(SimPl022Bench::pin_set_at[8] > SimPl022Bench::pin_clear_at[8]);
    CHECK_FALSE(SimPl022Bench::pin_level[cs_pin]);   // held across both phases
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());

    // A POLLED REQUEST IS THE LOOP, at any length, on a host with engines
    // too: a read and a write of the whole buffer, past every count above,
    // no block on either engine, no request raised, inside start(), and so
    // no vector needed. Its command phase and data phase alike.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request held = brief;
    held.polled = true;
    held.len = 32;
    frames_come_back(true);
    CHECK(Engined::start(held));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 0u);
    CHECK(RxEngine::blocks == 0u);
    CHECK(regs().SSPDMACR == 0u);
    CHECK(Engined::status() == spi_ok);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    Engined::Request held_write = held;
    held_write.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    held_write.mode = SpiMode::mode3;
    held_write.clock = SpiClocks::div16;
    frames_come_back(true);
    CHECK(Engined::start(held_write));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 0u);
    CHECK(Engined::status() == spi_ok);
    Engined::Request held_cmd = held;
    held_cmd.dc = SimPl022PinRef{8};
    held_cmd.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    held_cmd.cmd_len = 8;
    frames_come_back(true);
    CHECK(Engined::start(held_cmd));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 0u);
    CHECK(Engined::status() == spi_ok);
    CHECK(SimPl022Bench::pin_set_at[8] > SimPl022Bench::pin_clear_at[8]);
    // Its budget running out on the engined host is the same answer as on
    // the host with none: spi_stalled, and no engine touched.
    CHECK(Engined::start(held));
    CHECK(Engined::status() == spi_stalled);
    CHECK(TxEngine::blocks == 0u);
    CHECK(TxEngine::faults == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(Engined::recover());
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    // And a command phase polled ahead of an engined data block that runs
    // out ends the request there with the same answer: no block launched,
    // true from start(), the select released.
    Engined::Request two_stalled = brief;
    two_stalled.dc = SimPl022PinRef{8};
    two_stalled.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    two_stalled.cmd_len = 1;
    two_stalled.len = 16;
    CHECK(Engined::start(two_stalled));
    CHECK(Engined::status() == spi_stalled);
    CHECK(TxEngine::blocks == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);

    // Sixteen-bit frames on BYTE engines: the engines cannot carry them,
    // so the request runs polled inside start() whichever way it asked.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request wide{};
    wide.cs = SimPl022PinRef{cs_pin};
    wide.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    wide.len = 8;
    wide.bits = SpiDataSize::bits16;
    frames_come_back(true);
    CHECK(Engined::start(wide));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 0u);
    CHECK(regs().SSPIMSC == 0u);
    // Two bytes low-first make one frame, and the loop wrote eight.
    CHECK(regs().SSPDR == static_cast<uint32_t>(out_buf[14] | (out_buf[15] << 8)));

    Engined::release();
    CHECK(TxEngine::stops == 1u);
    CHECK(RxEngine::stops == 1u);
}

TEST_CASE("half-word engines carry both frame sizes, each at its own beat") {
    fresh();
    constexpr Clock clock;
    REQUIRE(WideHost::init(clock));
    CHECK(WideTx::errors_only);

    // A 16-bit request on aligned buffers rides the engines, one
    // half-word beat a frame, the length in FRAMES.
    WideHost::Request r{};
    r.cs = SimPl022PinRef{cs_pin};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.rx = lend<Lease::reply>(in_buf);
    r.len = 8;
    r.bits = SpiDataSize::bits16;
    CHECK_FALSE(WideHost::start(r));
    CHECK(WideTx::blocks == 1u);
    CHECK(WideRx::blocks == 1u);
    CHECK(WideTx::beat == 2u);
    CHECK(WideRx::beat == 2u);
    CHECK(WideTx::length == 8u);
    CHECK(regs().SSPIMSC == 0u);
    WideRx::next_flags = WideRx::flag_complete;
    CHECK(WideHost::dma_isr());
    CHECK(WideHost::status() == spi_ok);

    // The same width with no buffers clocks the fixed cell and fills the
    // sink at the frame's beat.
    WideHost::Request blind{};
    blind.cs = SimPl022PinRef{cs_pin};
    blind.len = 4;
    blind.bits = SpiDataSize::bits16;
    CHECK_FALSE(WideHost::start(blind));
    CHECK(WideTx::fixed);
    CHECK(WideRx::discarded);
    CHECK(WideTx::beat == 2u);
    WideRx::next_flags = WideRx::flag_complete;
    CHECK(WideHost::dma_isr());

    // 8-bit frames on the same engines: a byte beat.
    WideHost::Request narrow = r;
    narrow.bits = SpiDataSize::bits8;
    CHECK_FALSE(WideHost::start(narrow));
    CHECK(WideTx::beat == 1u);
    CHECK(WideRx::beat == 1u);
    WideRx::next_flags = WideRx::flag_complete;
    CHECK(WideHost::dma_isr());

    // A buffer one byte off a half-word is not one the DMA can take: the
    // request runs polled inside start() and no block starts.
    const uint32_t blocks = WideTx::blocks;
    WideHost::Request odd{};
    odd.cs = SimPl022PinRef{cs_pin};
    odd.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf + 1));
    odd.len = 4;
    odd.bits = SpiDataSize::bits16;
    frames_come_back(true);
    CHECK(WideHost::start(odd));
    frames_come_back(false);
    CHECK(WideHost::status() == spi_ok);
    CHECK(WideTx::blocks == blocks);
    CHECK(regs().SSPIMSC == 0u);

    WideHost::release();
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

    // THE DARK LISTENER IS THE PAD: drive_output(false) takes the answer
    // line away to no function with NO pull - not to the pad's reset state,
    // whose pull would fight the far host's - and SOD is left as the
    // block's own bit.
    const uint32_t before = SimPl022Bench::pad_claimed_at[client_pins.tx];
    Client::drive_output(false);
    CHECK(SimPl022Bench::pad_function[client_pins.tx] == SimPl022::no_function);
    CHECK(SimPl022Bench::pad_claimed_at[client_pins.tx] > before);
    CHECK_FALSE(SimPl022Bench::pad_pulled_up[client_pins.tx]);
    CHECK(SimPl022Bench::pad_released_at[client_pins.tx] == 0u);
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
