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

#include <variant>

#include "host/platform.hpp"
#include "host/sim_pl022.hpp"
#include "kernel/tenuto.hpp"
#include "pl022/spi.hpp"
#include "util/spi_bus.hpp"

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
    r.engine = SpiEngine::dma;
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
    SimPl022Bench::status_after_reset = on ? SpiFlag::rx_not_empty : 0u;
    if (on) {
        regs().SSPSR = regs().SSPSR | SpiFlag::rx_not_empty;
    } else {
        regs().SSPSR = regs().SSPSR & ~SpiFlag::rx_not_empty;
    }
}

TEST_CASE("a request naming dma on a host with no engines is refused, and touches nothing") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    static_assert(!Host::has_dma && Engined::has_dma);
    for (uint8_t i = 0; i < 32u; ++i) {
        out_buf[i] = static_cast<uint8_t>(0xA0u + i);
    }
    const uint32_t cr0 = regs().SSPCR0;

    // dma names the engines, and there are none: start() answers at once
    // with spi_rejected, before the block, the select, the D/C or the
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
    empty.engine = SpiEngine::polled;
    CHECK(Host::start(empty));
    CHECK(Host::status() == spi_ok);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == 0u);

    // A real one: the select asserted LOW and released at the end, the
    // setup time asked for, the rate applied, and no interrupt source armed.
    Host::Request r = plain_request(16);
    r.cs_setup_us = 5;
    r.clock = SpiClocks::div32;
    r.engine = SpiEngine::polled;
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
    // spi_dma_fault; the select released and THEN the block reset, since the
    // reset can move the SCK pad in modes 2 and 3; the state applied written
    // back and SSE up, with nothing asked of the program. What the reset does
    // to the FIFOs is not judged here: plain memory has none (brio/host/
    // sim_pl022.hpp); the suites' letter g judges it on silicon.
    const uint32_t stalled_from = SimPl022Bench::pin_clear_at[cs_pin];
    const uint32_t resets_before = SimPl022Bench::resets;
    CHECK(Host::start(r));
    CHECK(Host::status() == spi_stalled);
    CHECK(Host::status() != spi_dma_fault);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] > stalled_from);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] > SimPl022Bench::pin_clear_at[cs_pin]);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] < SimPl022Bench::reset_at);
    CHECK(regs().SSPCR0 == ((15u << 8) | 7u));
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    CHECK(regs().SSPIMSC == 0u);

    // The next request needs no recover() and pays no second reset.
    frames_come_back(true);
    CHECK(Host::start(r));
    frames_come_back(false);
    CHECK(Host::status() == spi_ok);
    CHECK(SimPl022Bench::resets == resets_before + 1u);

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
    cmd.engine = SpiEngine::polled;
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
    r.engine = SpiEngine::polled;
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

    // The same sum naming dma on a host with engines: the one command frame in
    // the loop inside start(), then the data block of 65535 on the engines, one
    // interrupt its end.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request a{};
    a.engine = SpiEngine::dma;
    a.cs = SimPl022PinRef{cs_pin};
    a.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    a.cmd_len = 1;
    a.tx = lend<Lease::reply>(static_cast<const uint8_t*>(wrap_out));
    a.rx = lend<Lease::reply>(wrap_in);
    a.len = 65535;
    a.clock = SpiClocks::div16;
    frames_come_back(true);
    CHECK_FALSE(Engined::start(a));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 1u);
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
    r.engine = SpiEngine::dma;
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
    blind.engine = SpiEngine::dma;
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
    (void)Engined::recover();   // the case below judges what the fault leaves

    // dma MEANS DMA AT THE SHORTEST LENGTH: one frame, read or written, at the
    // fastest rate, goes to the engines; nothing else decides.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request brief{};
    brief.engine = SpiEngine::dma;
    brief.cs = SimPl022PinRef{cs_pin};
    brief.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    brief.rx = lend<Lease::reply>(in_buf);
    brief.clock = SpiClocks::div2;
    brief.mode = SpiMode::mode3;
    brief.len = 1;
    CHECK_FALSE(Engined::start(brief));
    CHECK(TxEngine::blocks == 1u);
    CHECK(TxEngine::length == 1u);
    CHECK(regs().SSPDMACR == (SpiDmaControl::tx | SpiDmaControl::rx));
    CHECK_FALSE(SimPl022Bench::pin_level[cs_pin]);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    Engined::Request write = brief;
    write.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    CHECK_FALSE(Engined::start(write));
    CHECK(TxEngine::blocks == 2u);
    CHECK(RxEngine::discarded);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());

    // THE COMMAND RULE. A command ahead of a data phase runs in the loop inside
    // start(), D/C low, and D/C rises before the data block goes onto the
    // engines; one interrupt ends the request, whatever the command's length.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request two{};
    two.engine = SpiEngine::dma;
    two.cs = SimPl022PinRef{cs_pin};
    two.dc = SimPl022PinRef{8};
    two.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    two.cmd_len = 16;
    two.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf + 16));
    two.rx = lend<Lease::reply>(in_buf);
    two.len = 16;
    two.clock = SpiClocks::div4;   // the receive shape, whose loop the bench's frames can end
    two.mode = SpiMode::mode3;
    frames_come_back(true);
    CHECK_FALSE(Engined::start(two));
    frames_come_back(false);
    CHECK(TxEngine::blocks == 1u);
    CHECK(TxEngine::length == 16u);
    CHECK_FALSE(RxEngine::discarded);
    CHECK(SimPl022Bench::pin_set_at[8] > SimPl022Bench::pin_clear_at[8]);
    CHECK(SimPl022Bench::pin_level[8]);
    CHECK_FALSE(SimPl022Bench::pin_level[cs_pin]);   // held across both phases
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(Engined::status() == spi_ok);
    // A command with no data phase rides the engines as the whole request, its
    // echoes discarded, D/C low on its way and raised at its end, before the
    // select - where a polled command leaves it.
    Engined::Request alone = two;
    alone.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    alone.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    alone.len = 0;
    CHECK_FALSE(Engined::start(alone));
    CHECK(TxEngine::blocks == 2u);
    CHECK(TxEngine::length == 16u);
    CHECK(RxEngine::discarded);
    CHECK_FALSE(SimPl022Bench::pin_level[8]);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(SimPl022Bench::pin_level[8]);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(SimPl022Bench::pin_set_at[8] < SimPl022Bench::pin_set_at[cs_pin]);

    // A POLLED REQUEST IS THE LOOP, at any length, on a host with engines
    // too: a read and a write of the whole buffer, past every count above,
    // no block on either engine, no request raised, inside start(), and so
    // no vector needed. Its command phase and data phase alike.
    fresh();
    REQUIRE(Engined::init(clock));
    Engined::Request held = brief;
    held.engine = SpiEngine::polled;
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
    // the host with none: spi_stalled, no engine touched, the select released
    // and then the block reset and enabled.
    uint32_t resets_before = SimPl022Bench::resets;
    CHECK(Engined::start(held));
    CHECK(Engined::status() == spi_stalled);
    CHECK(TxEngine::blocks == 0u);
    CHECK(TxEngine::faults == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] < SimPl022Bench::reset_at);
    CHECK(regs().SSPCR1 == SpiControl1::enable);
    // And a command phase polled ahead of an engined data block that runs
    // out ends the request there with the same answer: no block launched,
    // true from start(), the select released, then the same reset.
    resets_before = SimPl022Bench::resets;
    Engined::Request two_stalled = brief;
    two_stalled.dc = SimPl022PinRef{8};
    two_stalled.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    two_stalled.cmd_len = 1;
    two_stalled.len = 16;
    CHECK(Engined::start(two_stalled));
    CHECK(Engined::status() == spi_stalled);
    CHECK(TxEngine::blocks == 0u);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] < SimPl022Bench::reset_at);
    CHECK(regs().SSPCR1 == SpiControl1::enable);

    // Sixteen-bit frames on BYTE engines: the engines cannot carry them, so a
    // request naming dma is refused - before the block, the select or the D/C
    // is touched - and the same request polled runs the loop.
    fresh();
    REQUIRE(Engined::init(clock));
    const uint32_t cr0_before = regs().SSPCR0;
    Engined::Request wide{};
    wide.engine = SpiEngine::dma;
    wide.cs = SimPl022PinRef{cs_pin};
    wide.dc = SimPl022PinRef{8};
    wide.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    wide.len = 8;
    wide.bits = SpiDataSize::bits16;
    wide.mode = SpiMode::mode3;
    CHECK(Engined::start(wide));
    CHECK(Engined::status() == spi_rejected);
    CHECK(TxEngine::blocks == 0u);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == 0u);
    CHECK(SimPl022Bench::pin_set_at[8] == 0u);
    CHECK(SimPl022Bench::pin_clear_at[8] == 0u);
    CHECK(regs().SSPCR0 == cr0_before);
    CHECK(regs().SSPDR == 0u);
    wide.engine = SpiEngine::polled;
    frames_come_back(true);
    CHECK(Engined::start(wide));
    frames_come_back(false);
    CHECK(Engined::status() == spi_ok);
    CHECK(TxEngine::blocks == 0u);
    CHECK(regs().SSPIMSC == 0u);
    // Two bytes low-first make one frame, and the loop wrote eight.
    CHECK(regs().SSPDR == static_cast<uint32_t>(out_buf[14] | (out_buf[15] << 8)));

    Engined::release();
    CHECK(TxEngine::stops == 1u);
    CHECK(RxEngine::stops == 1u);
}

namespace {

/// A request that rides the engines: 24 frames naming dma.
Engined::Request engined_request() {
    Engined::Request r{};
    r.engine = SpiEngine::dma;
    r.cs = SimPl022PinRef{cs_pin};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    r.rx = lend<Lease::reply>(in_buf);
    r.len = 24;
    return r;
}

}   // namespace

TEST_CASE("a bus error leaves the block's reset owed, and the next start() pays it first") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));
    Engined::prime(SpiMode::mode3, SpiClocks::div16);
    const uint32_t cr0 = regs().SSPCR0;
    Engined::Request r = engined_request();
    r.mode = SpiMode::mode3;
    r.clock = SpiClocks::div16;

    // Either channel. The interrupt decides and does not work: the requests
    // down, the channels put away, the select released, and not one register
    // of the block's control touched - no reset and no disable.
    for (const bool on_tx : {true, false}) {
        CHECK_FALSE(Engined::start(r));
        const uint32_t resets_before = SimPl022Bench::resets;
        const uint32_t armed_before = RxEngine::armed;
        if (on_tx) {
            TxEngine::next_flags = TxEngine::flag_error;
        } else {
            RxEngine::next_flags = RxEngine::flag_error;
        }
        CHECK(Engined::dma_isr());
        CHECK(Engined::status() == spi_dma_fault);
        CHECK(regs().SSPDMACR == 0u);
        CHECK_FALSE(TxEngine::busy());
        CHECK_FALSE(RxEngine::busy());
        CHECK(RxEngine::armed == armed_before + 1u);
        CHECK(SimPl022Bench::pin_level[cs_pin]);
        CHECK(SimPl022Bench::resets == resets_before);
        CHECK((regs().SSPCR1 & SpiControl1::enable) != 0u);

        // The next start() resets the block before anything else, its own
        // select after, and the state applied stands; once.
        CHECK_FALSE(Engined::start(r));
        CHECK(SimPl022Bench::resets == resets_before + 1u);
        CHECK(SimPl022Bench::reset_at < SimPl022Bench::pin_clear_at[cs_pin]);
        CHECK(regs().SSPCR0 == cr0);
        CHECK((regs().SSPCR1 & SpiControl1::enable) != 0u);
        CHECK(Engined::status() == spi_ok);
        RxEngine::next_flags = RxEngine::flag_complete;
        CHECK(Engined::dma_isr());
        CHECK(Engined::status() == spi_ok);
        CHECK_FALSE(Engined::start(r));
        CHECK(SimPl022Bench::resets == resets_before + 1u);
        RxEngine::next_flags = RxEngine::flag_complete;
        CHECK(Engined::dma_isr());
    }

    // The polled way pays it too, and so does the empty request: the test
    // comes before anything else start() does.
    CHECK_FALSE(Engined::start(r));
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Engined::dma_isr());
    uint32_t resets_before = SimPl022Bench::resets;
    Engined::Request polled = r;
    polled.engine = SpiEngine::polled;
    frames_come_back(true);
    CHECK(Engined::start(polled));
    frames_come_back(false);
    CHECK(Engined::status() == spi_ok);
    CHECK(SimPl022Bench::resets == resets_before + 1u);

    CHECK_FALSE(Engined::start(r));
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Engined::dma_isr());
    resets_before = SimPl022Bench::resets;
    Engined::Request empty{};
    empty.engine = SpiEngine::dma;
    empty.cs = SimPl022PinRef{cs_pin};
    CHECK(Engined::start(empty));
    CHECK(Engined::status() == spi_ok);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
}

TEST_CASE("prime() pays a reset that is owed, for a caller framing its own select") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));
    Engined::prime(SpiMode::mode3, SpiClocks::div16);
    Engined::Request r = engined_request();
    r.mode = SpiMode::mode3;
    r.clock = SpiClocks::div16;
    CHECK_FALSE(Engined::start(r));
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Engined::dma_isr());

    // The program's select falls after prime() and before start(), so the reset
    // is prime()'s, once, and the request behind it pays none.
    const uint32_t resets_before = SimPl022Bench::resets;
    const SimPl022PinRef own{cs_pin};
    Engined::prime(SpiMode::mode3, SpiClocks::div16);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK((regs().SSPCR1 & SpiControl1::enable) != 0u);
    own.clear();
    const uint32_t lowered = SimPl022Bench::pin_clear_at[cs_pin];
    CHECK(SimPl022Bench::reset_at < lowered);
    r.cs = SimPl022PinRef{};
    CHECK_FALSE(Engined::start(r));
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == lowered);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    own.set();

    // With no reset owed prime() is what it always was.
    Engined::prime(SpiMode::mode1, SpiClocks::div8);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(regs().SSPCR0 == spi_cr0_of({.mode = SpiMode::mode1, .bits = 8, .clock = SpiClocks::div8}));
}

TEST_CASE("recover(), init() and release() each clear a reset that is owed") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));
    const Engined::Request r = engined_request();
    const auto fault = [&] {
        CHECK_FALSE(Engined::start(r));
        TxEngine::next_flags = TxEngine::flag_error;
        CHECK(Engined::dma_isr());
        CHECK(Engined::status() == spi_dma_fault);
    };
    const auto next_pays_nothing = [&] {
        const uint32_t resets_before = SimPl022Bench::resets;
        CHECK_FALSE(Engined::start(r));
        CHECK(SimPl022Bench::resets == resets_before);
        RxEngine::next_flags = RxEngine::flag_complete;
        CHECK(Engined::dma_isr());
    };

    // recover() answers true and resets once, the select first.
    fault();
    uint32_t resets_before = SimPl022Bench::resets;
    CHECK(Engined::recover());
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] < SimPl022Bench::reset_at);
    CHECK((regs().SSPCR1 & SpiControl1::enable) != 0u);
    next_pays_nothing();

    fault();
    REQUIRE(Engined::init(clock));
    next_pays_nothing();

    fault();
    Engined::release();
    REQUIRE(Engined::init(clock));
    next_pays_nothing();
}

namespace {

/// One client over the arbiter over the engined host: what is queued behind a
/// stall or a bus error starts in the dispatch that answers it, before the
/// client has run.
using QueueBus = SpiBus<Engined, HostPlatform, 4>;

struct Go {};

struct QueueClient {
    using Event = std::variant<Go, SpiDone>;
    static inline EventQueue<Event, 6, HostPlatform> queue;
    static inline Engined::Request first = [] { Engined::Request r{}; r.engine = SpiEngine::dma; return r; }();
    static inline Engined::Request second = [] { Engined::Request r{}; r.engine = SpiEngine::dma; return r; }();
    static inline uint8_t replies[2] = {};
    static inline uint8_t n = 0;

    static void init() { n = 0; }
    static void dispatch(const Event& e) {
        match(e,
              [](const Go&) {
                  post<QueueBus>(first);
                  post<QueueBus>(second);
              },
              [](const SpiDone& d) {
                  REQUIRE(n < 2u);
                  replies[n] = d.status;
                  ++n;
              });
    }
};

using QueueKernel = Tenuto<HostPlatform, QueueClient, QueueBus>;

void run_queue() {
    while (QueueKernel::step()) {
    }
}

}   // namespace

TEST_CASE("a request queued behind a stall or a bus error finds a clean block") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));
    QueueKernel::init_all();
    Engined::prime(SpiMode::mode3, SpiClocks::div16);
    const uint32_t cr0 = regs().SSPCR0;

    Engined::Request good = engined_request();
    good.mode = SpiMode::mode3;
    good.clock = SpiClocks::div16;
    good.reply = reply_to<QueueClient, SpiDone>();

    // Behind a stall: the first request's frames never come back, the second
    // rides the engines, both started by one dispatch of the arbiter's. No
    // recover() is anywhere, and the arbiter calls none.
    Engined::Request stalling = good;
    stalling.engine = SpiEngine::polled;
    stalling.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    stalling.len = 2;
    QueueClient::first = stalling;
    QueueClient::second = good;
    QueueClient::n = 0;
    const uint32_t blocks_before = TxEngine::blocks;
    uint32_t resets_before = SimPl022Bench::resets;
    post<QueueClient>(Go{});
    run_queue();
    CHECK(QueueClient::n == 1u);
    CHECK(QueueClient::replies[0] == spi_stalled);
    CHECK(TxEngine::blocks == blocks_before + 1u);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(regs().SSPCR0 == cr0);
    CHECK((regs().SSPCR1 & SpiControl1::enable) != 0u);
    RxEngine::next_flags = RxEngine::flag_complete;
    REQUIRE(Engined::dma_isr());
    post<QueueBus>(TransferDone{Engined::status()});
    run_queue();
    CHECK(QueueClient::n == 2u);
    CHECK(QueueClient::replies[1] == spi_ok);

    // Behind a bus error: the first on the engines fails in the interrupt, the
    // glue posts its code, and the dispatch that answers it starts the second,
    // whose start() pays the reset before it launches.
    QueueClient::first = good;
    QueueClient::second = good;
    QueueClient::n = 0;
    resets_before = SimPl022Bench::resets;
    post<QueueClient>(Go{});
    run_queue();
    CHECK(QueueClient::n == 0u);
    TxEngine::next_flags = TxEngine::flag_error;
    REQUIRE(Engined::dma_isr());
    CHECK(SimPl022Bench::resets == resets_before);
    post<QueueBus>(TransferDone{Engined::status()});
    run_queue();
    CHECK(QueueClient::n == 1u);
    CHECK(QueueClient::replies[0] == spi_dma_fault);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::reset_at < SimPl022Bench::pin_clear_at[cs_pin]);
    RxEngine::next_flags = RxEngine::flag_complete;
    REQUIRE(Engined::dma_isr());
    post<QueueBus>(TransferDone{Engined::status()});
    run_queue();
    CHECK(QueueClient::n == 2u);
    CHECK(QueueClient::replies[1] == spi_ok);
}

TEST_CASE("half-word engines carry both frame sizes, each at its own beat") {
    fresh();
    constexpr Clock clock;
    REQUIRE(WideHost::init(clock));
    CHECK(WideTx::errors_only);

    // A 16-bit request on aligned buffers rides the engines, one
    // half-word beat a frame, the length in FRAMES.
    WideHost::Request r{};
    r.engine = SpiEngine::dma;
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
    blind.engine = SpiEngine::dma;
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

    // A buffer one byte off a half-word is not one the DMA can take: a request
    // naming dma is refused, no block starts and the select is not touched -
    // the out buffer, the in buffer, a command alone, a command ahead of such
    // data (refused before the command is clocked).
    const uint32_t blocks = WideTx::blocks;
    const uint32_t lowered = SimPl022Bench::pin_clear_at[cs_pin];
    const uint32_t dr = regs().SSPDR;
    WideHost::Request odd{};
    odd.engine = SpiEngine::dma;
    odd.cs = SimPl022PinRef{cs_pin};
    odd.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf + 1));
    odd.len = 4;
    odd.bits = SpiDataSize::bits16;
    CHECK(WideHost::start(odd));
    CHECK(WideHost::status() == spi_rejected);
    WideHost::Request odd_in = r;
    odd_in.rx = lend<Lease::reply>(in_buf + 1);
    CHECK(WideHost::start(odd_in));
    CHECK(WideHost::status() == spi_rejected);
    WideHost::Request odd_cmd{};
    odd_cmd.engine = SpiEngine::dma;
    odd_cmd.cs = SimPl022PinRef{cs_pin};
    odd_cmd.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf + 1));
    odd_cmd.cmd_len = 2;
    odd_cmd.bits = SpiDataSize::bits16;
    CHECK(WideHost::start(odd_cmd));
    CHECK(WideHost::status() == spi_rejected);
    WideHost::Request cmd_then_odd = odd;
    cmd_then_odd.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf));
    cmd_then_odd.cmd_len = 2;
    CHECK(WideHost::start(cmd_then_odd));
    CHECK(WideHost::status() == spi_rejected);
    CHECK(WideTx::blocks == blocks);
    CHECK(SimPl022Bench::pin_clear_at[cs_pin] == lowered);
    CHECK(regs().SSPDR == dr);
    CHECK(regs().SSPIMSC == 0u);
    // A command off a half-word ahead of aligned 16-bit data is carried: the
    // command runs in the loop, a frame at a time, and the data rides.
    WideHost::Request odd_ahead = r;
    odd_ahead.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out_buf + 1));
    odd_ahead.cmd_len = 2;
    frames_come_back(true);
    CHECK_FALSE(WideHost::start(odd_ahead));
    frames_come_back(false);
    CHECK(WideTx::blocks == blocks + 1u);
    CHECK(WideTx::beat == 2u);
    WideRx::next_flags = WideRx::flag_complete;
    CHECK(WideHost::dma_isr());
    CHECK(WideHost::status() == spi_ok);

    WideHost::release();
}

TEST_CASE("a refused request leaves a reset that is owed for the next one") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));
    Engined::Request r = engined_request();
    CHECK_FALSE(Engined::start(r));
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Engined::dma_isr());
    // Byte engines refuse 16-bit frames: answered before anything moves, the
    // block's reset still owed, then paid by the next request, once.
    const uint32_t resets_before = SimPl022Bench::resets;
    Engined::Request wide = r;
    wide.bits = SpiDataSize::bits16;
    CHECK(Engined::start(wide));
    CHECK(Engined::status() == spi_rejected);
    CHECK(SimPl022Bench::resets == resets_before);
    CHECK_FALSE(Engined::start(r));
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(Engined::status() == spi_ok);
}

TEST_CASE("when a request ends, the wire has ended: BSY waited for with SPH = 0, and bounded") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Engined::init(clock));
    const auto busy = [](bool on) {
        regs().SSPSR = on ? (regs().SSPSR | SpiFlag::busy) : (regs().SSPSR & ~SpiFlag::busy);
    };

    // POLLED, SPH = 0: the loop's tail waits for BSY on the request's budget. A
    // shifter that never stops runs the budget out: spi_stalled, the select
    // released first, the block reset.
    Engined::Request polled = engined_request();
    polled.engine = SpiEngine::polled;
    polled.mode = SpiMode::mode0;
    frames_come_back(true);
    busy(true);
    uint32_t resets_before = SimPl022Bench::resets;
    CHECK(Engined::start(polled));
    CHECK(Engined::status() == spi_stalled);
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    CHECK(SimPl022Bench::pin_set_at[cs_pin] < SimPl022Bench::reset_at);
    // With SPH = 1 the end is not early and nothing waits for BSY.
    frames_come_back(true);
    busy(true);
    polled.mode = SpiMode::mode1;
    CHECK(Engined::start(polled));
    CHECK(Engined::status() == spi_ok);
    busy(false);
    frames_come_back(false);

    // THE VECTOR, SPH = 0: the completion waits for BSY on a bound of one SCK
    // period and a margin; run out, the request ends spi_stalled, the select
    // released, the reset owed and paid by the next start().
    Engined::Request dma = engined_request();
    dma.mode = SpiMode::mode2;
    dma.clock = SpiClocks::div16;
    CHECK_FALSE(Engined::start(dma));
    busy(true);
    RxEngine::next_flags = RxEngine::flag_complete;
    resets_before = SimPl022Bench::resets;
    CHECK(Engined::dma_isr());
    CHECK(Engined::status() == spi_stalled);
    CHECK(SimPl022Bench::pin_level[cs_pin]);
    CHECK(regs().SSPDMACR == 0u);
    CHECK(SimPl022Bench::resets == resets_before);
    busy(false);
    CHECK_FALSE(Engined::start(dma));
    CHECK(SimPl022Bench::resets == resets_before + 1u);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(Engined::status() == spi_ok);
    // SPH = 1: BSY up is not waited for.
    dma.mode = SpiMode::mode3;
    CHECK_FALSE(Engined::start(dma));
    busy(true);
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Engined::dma_isr());
    CHECK(Engined::status() == spi_ok);
    busy(false);
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
