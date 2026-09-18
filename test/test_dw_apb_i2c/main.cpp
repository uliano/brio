// Host tests for the Synopsys DesignWare I2C driver
// (brio/dw_apb_i2c/i2c.hpp) over a chip that is not a chip: two register
// blocks in RAM with a modelled wire under their pads
// (brio/host/sim_dw_apb_i2c.hpp). What is judged here is what a bench
// cannot see cheaply - the exact WORDS a bring-up leaves in the block,
// the ORDER of the acts that make it, the entry a tenure ends with - and
// what a bench cannot stage at all: a block that will not disable, an
// abort of every reason, and a client that releases SDA on the n-th
// hand-driven pulse.
// Run with: ctest --preset host (or ctest --preset host -R test_dw_apb_i2c)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include "dw_apb_i2c/i2c.hpp"
#include "host/sim_dw_apb_i2c.hpp"

using namespace brio;

namespace {

using Clock = SimDwApbI2cClock<125'000'000>;

constexpr SimDwApbI2cPins pins{.scl = 1, .sda = 0};
constexpr SimDwApbI2cPins peer_pins{.scl = 3, .sda = 2};

using Host = DwApbI2cHost<SimDwApbI2c, 0, pins, SimI2cNoEngine, SimI2cNoEngine>;
using Client = DwApbI2cClient<SimDwApbI2c, 1, peer_pins>;
using Block = DwApbI2cBlock<SimDwApbI2c, 0>;

using TxEngine = SimDwApbI2cEngine<0, uint16_t>;
using RxEngine = SimDwApbI2cEngine<1, uint8_t>;
using Streamed = DwApbI2cHost<SimDwApbI2c, 1, peer_pins, TxEngine, RxEngine>;

SimDwApbI2cRegs& regs() { return SimDwApbI2c::regs<0>(); }
SimDwApbI2cRegs& peer_regs() { return SimDwApbI2c::regs<1>(); }

void fresh() { SimDwApbI2c::reset_all(); }

/// What a finished tenure left standing in the block.
uint32_t last_entry() { return regs().IC_DATA_CMD; }

}   // namespace

TEST_CASE("the counts are the wire's period less what the block adds") {
    // At 125 MHz the spike filter is 50 ns rounded up - seven cycles -
    // and each speed's period is split with the low taking the larger
    // share; the programmed counts are that split LESS the cycles the
    // block adds (SPKLEN + 7 high, 1 low), so the wire runs at the rate
    // that was asked for.
    const auto std_100k = i2c_timing_for(125'000'000, I2cSpeed::standard_100k);
    REQUIRE(std_100k.has_value());
    CHECK(std_100k->spklen == 7);
    CHECK(std_100k->hcnt == 486);
    CHECK(std_100k->lcnt == 749);
    CHECK((std_100k->hcnt + std_100k->spklen + 7u) + (std_100k->lcnt + 1u) == 1250u);
    CHECK(i2c_scl_hz(125'000'000, *std_100k) == 100'000u);

    const auto fast = i2c_timing_for(125'000'000, I2cSpeed::fast_400k);
    const auto plus = i2c_timing_for(125'000'000, I2cSpeed::fast_plus_1m);
    REQUIRE(fast.has_value());
    REQUIRE(plus.has_value());
    CHECK(i2c_scl_hz(125'000'000, *fast) == 399'361u);
    CHECK(i2c_scl_hz(125'000'000, *plus) == 1'000'000u);
    // The same split programmed WHOLE - the added cycles not taken off -
    // runs slow, which is what subtracting them is for.
    CHECK(i2c_scl_hz(125'000'000, {.hcnt = 500, .lcnt = 750, .spklen = 7}) == 98'814u);

    // The hold is the specification's 300 ns (120 in fast-mode-plus),
    // one cycle over, and the setup its tSU;DAT (250 ns here) the same
    // way.
    CHECK(std_100k->sda_tx_hold == 39);
    CHECK(plus->sda_tx_hold == 16);
    CHECK(std_100k->sda_setup == 33);
    CHECK(plus->sda_setup == 8);
}

TEST_CASE("a clock too slow for a speed makes it unreachable, not slow") {
    // The block's own floors at each speed's minimum clock: HCNT =
    // SPKLEN + 5, LCNT = SPKLEN + 7, with SPKLEN 1 there.
    CHECK(i2c_timing_for(12'000'000, I2cSpeed::fast_400k)->spklen == 1);
    CHECK(i2c_timing_for(12'000'000, I2cSpeed::fast_400k)->hcnt == 6);
    CHECK(i2c_timing_for(12'000'000, I2cSpeed::fast_400k)->lcnt == 15);
    CHECK(i2c_timing_for(2'700'000, I2cSpeed::standard_100k)->hcnt == 6);
    CHECK(i2c_timing_for(2'700'000, I2cSpeed::standard_100k)->lcnt == 12);

    CHECK_FALSE(i2c_timing_for(2'699'999, I2cSpeed::standard_100k).has_value());
    CHECK_FALSE(i2c_timing_for(11'999'999, I2cSpeed::fast_400k).has_value());
    CHECK_FALSE(i2c_timing_for(31'999'999, I2cSpeed::fast_plus_1m).has_value());
    CHECK(i2c_timing_for(32'000'000, I2cSpeed::fast_plus_1m).has_value());
    CHECK(i2c_min_clk_hz(I2cSpeed::fast_plus_1m) == 32'000'000u);

    // An empty timing produces nothing at all, rather than dividing by
    // zero.
    CHECK(i2c_scl_hz(125'000'000, {.hcnt = 0, .lcnt = 0, .spklen = 0}) == 15'625'000u);
    CHECK(i2c_scl_hz(0, {.hcnt = 1, .lcnt = 1}) == 0u);
}

TEST_CASE("the control word, the entries and the abort's own reason") {
    // A host disables its client half; every behaviour bit of the
    // default configuration is in the word, and the speed code sits in
    // bits 2:1 (standard is 1, both fast modes 2).
    CHECK(i2c_con_of({}) == (I2cControl::host_mode | I2cControl::client_disable |
                             I2cControl::restart_enable | I2cControl::stop_det_if_addressed |
                             I2cControl::tx_empty_late | I2cControl::hold_when_rx_full |
                             (I2cControl::speed_standard << I2cControl::speed_lsb)));
    CHECK(i2c_con_of({}) == 0x3E3u);
    CHECK(i2c_con_of({.speed = I2cSpeed::fast_400k}) == 0x3E5u);
    CHECK(i2c_con_of({.speed = I2cSpeed::fast_plus_1m}) == 0x3E5u);
    CHECK((i2c_con_of({.role = I2cRole::client}) & I2cControl::host_mode) == 0u);
    CHECK((i2c_con_of({.role = I2cRole::client}) & I2cControl::client_disable) == 0u);
    CHECK((i2c_con_of({.ten_bit_host = true}) & I2cControl::ten_bit_host) != 0u);
    CHECK((i2c_con_of({.restart_enabled = false}) & I2cControl::restart_enable) == 0u);

    // An entry is a byte or a read command, with its two flags above it.
    CHECK(i2c_write_entry(0xA5) == 0x00A5u);
    CHECK(i2c_write_entry(0xA5, {.stop = true}) == 0x02A5u);
    CHECK(i2c_read_entry() == 0x0100u);
    CHECK(i2c_read_entry({.restart = true}) == 0x0500u);
    CHECK(i2c_read_entry({.restart = true, .stop = true}) == 0x0700u);

    // One interrupt carries every failure; the source says which, and
    // a lost arbitration outranks the NACK it also raises.
    CHECK(i2c_status_of_abort(I2cAbort::addr_noack) == i2c_nack_addr);
    CHECK(i2c_status_of_abort(I2cAbort::data_noack) == i2c_nack_data);
    CHECK(i2c_status_of_abort(I2cAbort::arb_lost) == i2c_arb_lost);
    CHECK(i2c_status_of_abort(I2cAbort::arb_lost | I2cAbort::addr_noack) == i2c_arb_lost);
    CHECK(i2c_status_of_abort(I2cAbort::user_abort) == i2c_bus_error);
    CHECK(i2c_status_of_abort(0) == i2c_bus_error);
}

TEST_CASE("a bring-up leaves the block set up, the pads last and the line after them") {
    fresh();
    // What the block comes up as, and what init() starts from: a host at
    // the fast speed code with restarts enabled, both transmit-FIFO
    // flags set and the receive FIFO empty.
    CHECK(regs().IC_CON == 0x65u);
    CHECK(regs().IC_STATUS == (I2cFlag::tx_not_full | I2cFlag::tx_empty));
    CHECK(regs().IC_FS_SPKLEN == 7u);

    constexpr Clock clock;
    REQUIRE(Host::init(clock));

    // The block was cycled through its reset, configured as a host at
    // the standard speed, and given that speed's counts - in the
    // STANDARD pair, the fast one left at its reset value.
    CHECK(SimDwApbI2cBench::resets == 1u);
    CHECK(regs().IC_CON == i2c_con_of({.role = I2cRole::host}));
    CHECK(regs().IC_SS_SCL_HCNT == 486u);
    CHECK(regs().IC_SS_SCL_LCNT == 749u);
    CHECK(regs().IC_FS_SCL_HCNT == 0x06u);
    CHECK(regs().IC_FS_SPKLEN == 7u);
    CHECK(regs().IC_SDA_HOLD == 39u);
    CHECK(regs().IC_SDA_SETUP == 33u);

    // The thresholds: every byte raises RX_FULL, the pump refills at
    // the FIFO's half.
    CHECK(regs().IC_RX_TL == 0u);
    CHECK(regs().IC_TX_TL == Block::fifo_depth / 2u);

    // Nothing armed until a tenure, the block enabled, the line on.
    CHECK(regs().IC_INTR_MASK == 0u);
    CHECK((regs().IC_ENABLE & I2cEnable::enable) != 0u);
    CHECK(SimDwApbI2cInterrupts::line_enabled[0]);
    CHECK(Host::idle());
    CHECK(Host::reference_hz() == Clock::hz);
    CHECK(Host::speed_ok(I2cSpeed::fast_plus_1m));

    // THE ORDER init() promises: both pads go to the peripheral with
    // the block still DISABLED, so nothing of theirs is driven before
    // the pull-ups own the wire.
    CHECK(SimDwApbI2cBench::pad_function[pins.scl] == SimDwApbI2c::pad_function);
    CHECK(SimDwApbI2cBench::pad_function[pins.sda] == SimDwApbI2c::pad_function);
    CHECK(SimDwApbI2cBench::pad_pulled_up[pins.scl]);
    CHECK(SimDwApbI2cBench::pad_claimed_at[pins.scl] < SimDwApbI2cBench::enabled_at);
    CHECK(SimDwApbI2cBench::pad_claimed_at[pins.sda] < SimDwApbI2cBench::enabled_at);
}

TEST_CASE("a clock that cannot carry a speed marks it unreachable and nothing else") {
    fresh();
    // Eight megahertz: standard mode is legal, both fast ones are not.
    REQUIRE(Host::init(SimDwApbI2cClock<8'000'000>{}));
    CHECK(Host::speed_ok(I2cSpeed::standard_100k));
    CHECK_FALSE(Host::speed_ok(I2cSpeed::fast_400k));
    CHECK_FALSE(Host::speed_ok(I2cSpeed::fast_plus_1m));

    Host::Request r{};
    r.addr = 0x42;
    r.speed = I2cSpeed::fast_400k;
    const uint32_t entry_before = last_entry();
    // TRUE is "nothing moved": the arbiter answers with status() alone.
    CHECK(Host::start(r));
    CHECK(Host::status() == i2c_rejected);
    CHECK(Host::idle());
    CHECK(last_entry() == entry_before);
    CHECK(regs().IC_TAR != 0x42u);

    // And a clock under the slowest speed's floor refuses the bring-up
    // outright.
    fresh();
    CHECK_FALSE(Host::init(SimDwApbI2cClock<2'000'000>{}));
}

TEST_CASE("a block that will not disable is answered, not waited on for ever") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    // The one refusal a bench cannot stage: the hardware keeps saying it
    // is running, which is what a command with no STOP in sight does.
    SimDwApbI2c::stuck_enabled = true;

    Host::Request r{};
    r.addr = 0x42;
    r.rx_len = 1;
    CHECK(Host::start(r));
    CHECK(Host::status() == i2c_bus_error);
    CHECK(Host::idle());
    SimDwApbI2c::stuck_enabled = false;
}

TEST_CASE("every tenure's last entry carries STOP, and the address is the target's") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    const uint8_t out[3] = {0x11, 0x22, 0x33};

    SUBCASE("a write ends on the last byte") {
        Host::Request r{};
        r.addr = 0x42;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
        r.tx_len = 3;
        CHECK_FALSE(Host::start(r));
        CHECK(regs().IC_TAR == 0x42u);
        CHECK(last_entry() == i2c_write_entry(0x33, {.stop = true}));
        // A write ends on its STOP: the sources armed are the abort, the
        // stop and the receive level, and TX_EMPTY is NOT among them
        // because every entry fit.
        CHECK(regs().IC_INTR_MASK ==
              (I2cInterrupt::tx_abrt | I2cInterrupt::stop_det | I2cInterrupt::rx_full));
        CHECK_FALSE(Host::idle());
    }

    SUBCASE("a read ends on the last read command") {
        Host::Request r{};
        r.addr = 0x48;
        r.rx_len = 4;
        CHECK_FALSE(Host::start(r));
        CHECK(last_entry() == i2c_read_entry({.stop = true}));
    }

    SUBCASE("a write-then-read turns direction with a repeated START") {
        Host::Request r{};
        r.addr = 0x48;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
        r.tx_len = 1;
        r.rx_len = 1;
        CHECK_FALSE(Host::start(r));
        // Two entries: the byte (no stop, the read follows) and the one
        // read command, which carries BOTH the repeated START and the
        // STOP.
        CHECK(last_entry() == i2c_read_entry({.restart = true, .stop = true}));
    }

    SUBCASE("the probe is a one-byte read, since no entry stands for an address alone") {
        Host::Request r{};
        r.addr = 0x50;
        CHECK_FALSE(Host::start(r));
        CHECK(regs().IC_TAR == 0x50u);
        CHECK(last_entry() == i2c_read_entry({.stop = true}));
    }
}

TEST_CASE("the speed of a tenure is written into the block, once") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    CHECK(regs().IC_CON == i2c_con_of({}));

    Host::Request r{};
    r.addr = 0x42;
    r.rx_len = 1;
    r.speed = I2cSpeed::fast_plus_1m;
    CHECK_FALSE(Host::start(r));
    // The fast pair now holds fast-mode-plus's counts and IC_CON its
    // code; the standard pair is left as it was.
    CHECK(regs().IC_CON == i2c_con_of({.speed = I2cSpeed::fast_plus_1m}));
    CHECK(regs().IC_FS_SCL_HCNT == 36u);
    CHECK(regs().IC_FS_SCL_LCNT == 74u);
    CHECK(regs().IC_SS_SCL_HCNT == 486u);
    CHECK(Host::scl_hz(I2cSpeed::fast_plus_1m) == 1'000'000u);
}

TEST_CASE("an abort ends the tenure with the wire's own reason") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));

    Host::Request r{};
    r.addr = 0x42;
    r.rx_len = 2;
    REQUIRE_FALSE(Host::start(r));

    regs().IC_INTR_STAT = I2cInterrupt::tx_abrt;
    regs().IC_TX_ABRT_SOURCE = I2cAbort::addr_noack;
    CHECK(Host::isr());                     // the tenure completed
    CHECK(Host::status() == i2c_nack_addr);
    CHECK(Host::idle());
    CHECK(regs().IC_INTR_MASK == 0u);       // and disarmed itself

    // A stale source on an idle engine is swept away and reported to
    // nobody.
    regs().IC_INTR_STAT = I2cInterrupt::stop_det;
    CHECK_FALSE(Host::isr());
    CHECK(regs().IC_INTR_MASK == 0u);

    // Every other reason, through the same one interrupt.
    REQUIRE_FALSE(Host::start(r));
    regs().IC_INTR_STAT = I2cInterrupt::tx_abrt;
    regs().IC_TX_ABRT_SOURCE = I2cAbort::data_noack;
    CHECK(Host::isr());
    CHECK(Host::status() == i2c_nack_data);

    REQUIRE_FALSE(Host::start(r));
    regs().IC_INTR_STAT = I2cInterrupt::tx_abrt;
    regs().IC_TX_ABRT_SOURCE = I2cAbort::arb_lost | I2cAbort::data_noack;
    CHECK(Host::isr());
    CHECK(Host::status() == i2c_arb_lost);
}

TEST_CASE("the unstick owns the wire, counts the pulses and hands the pads back") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    SimDwApbI2cBench::scl_pad = pins.scl;
    SimDwApbI2cBench::sda_pad = pins.sda;

    SUBCASE("a healthy wire is left alone") {
        SimDwApbI2cBench::release_after = 0;   // SDA reads high at once
        CHECK(Host::unstick() == 0u);
        CHECK(SimDwApbI2cBench::pulses == 0u);
        CHECK(SimDwApbI2cBench::spins == 0u);
    }

    SUBCASE("a client holding SDA is clocked out, and the pulses are the answer") {
        SimDwApbI2cBench::release_after = 3;
        CHECK(Host::unstick() == 3u);
        CHECK(SimDwApbI2cBench::pulses == 3u);
        // Two half-bits a pulse and two for the STOP, five
        // microseconds each: half a bit of standard mode.
        CHECK(SimDwApbI2cBench::spins == 8u);
        CHECK(SimDwApbI2cBench::micros == 40u);
    }

    SUBCASE("a line nine pulses and a STOP cannot free is a short, and says so") {
        SimDwApbI2cBench::release_after = 20;
        CHECK(Host::unstick() == 0xFFu);
        CHECK(SimDwApbI2cBench::pulses == 9u);
        CHECK(SimDwApbI2cBench::spins == 20u);
    }

    // Whatever the wire answered, the pads go back to the peripheral and
    // the line comes back on.
    CHECK(SimDwApbI2cBench::pad_function[pins.scl] == SimDwApbI2c::pad_function);
    CHECK(SimDwApbI2cBench::pad_function[pins.sda] == SimDwApbI2c::pad_function);
    CHECK_FALSE(SimDwApbI2cBench::pad_driven[pins.scl]);
    CHECK_FALSE(SimDwApbI2cBench::pad_driven[pins.sda]);
    CHECK(SimDwApbI2cInterrupts::line_enabled[0]);
}

TEST_CASE("recover puts the engine back where a start is legal; release puts it away") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    Host::Request r{};
    r.addr = 0x42;
    r.rx_len = 2;
    REQUIRE_FALSE(Host::start(r));
    CHECK_FALSE(Host::idle());

    CHECK(Host::recover());
    CHECK(SimDwApbI2cBench::resets == 2u);
    CHECK(Host::idle());
    CHECK(regs().IC_CON == i2c_con_of({}));
    CHECK(regs().IC_SS_SCL_HCNT == 486u);
    CHECK(regs().IC_TX_TL == Block::fifo_depth / 2u);
    CHECK(regs().IC_INTR_MASK == 0u);
    CHECK((regs().IC_ENABLE & I2cEnable::enable) != 0u);
    CHECK(SimDwApbI2cInterrupts::line_enabled[0]);

    Host::release();
    CHECK_FALSE(SimDwApbI2cInterrupts::line_enabled[0]);
    CHECK(regs().IC_INTR_MASK == 0u);
    CHECK((regs().IC_ENABLE & I2cEnable::enable) == 0u);
    CHECK(SimDwApbI2cBench::pad_function[pins.scl] == 0u);
    CHECK(SimDwApbI2cBench::pad_released_at[pins.sda] != 0u);
    CHECK_FALSE(Block::released());
}

TEST_CASE("the engines take the middle of a read phase, the pump its two ends") {
    fresh();
    TxEngine::reset();
    RxEngine::reset();
    constexpr Clock clock;
    REQUIRE(Streamed::init(clock));
    CHECK(TxEngine::armed == 1u);
    CHECK(RxEngine::armed == 1u);
    CHECK(TxEngine::blocks == 0u);   // armed is not started

    uint8_t in[8]{};
    Streamed::Request r{};
    r.addr = 0x48;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = 8;
    REQUIRE_FALSE(Streamed::start(r));

    // The pump wrote the FIRST read command; then the engines took over:
    // every byte of the phase on the receive channel, the plain command
    // poured from ONE cell for the entries between the first and the
    // last, and both requests enabled.
    CHECK(RxEngine::length == 8u);
    CHECK_FALSE(RxEngine::discarded);
    CHECK(TxEngine::length == 6u);
    CHECK(TxEngine::fixed);
    CHECK(peer_regs().IC_DMA_CR == (I2cDmaControl::tx | I2cDmaControl::rx));
    CHECK((peer_regs().IC_INTR_MASK & (I2cInterrupt::tx_empty | I2cInterrupt::rx_full)) == 0u);

    // The block of commands is in: the transmit request goes off and the
    // LAST entry - the one that carries STOP - goes through the pump.
    TxEngine::next_flags = TxEngine::flag_complete;
    CHECK_FALSE(Streamed::dma_isr());
    CHECK(peer_regs().IC_DMA_CR == I2cDmaControl::rx);
    CHECK(peer_regs().IC_DATA_CMD == i2c_read_entry({.stop = true}));

    // The receive block ending IS the tenure's end.
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Streamed::dma_isr());
    CHECK(Streamed::status() == i2c_ok);
    CHECK(Streamed::idle());
    CHECK(peer_regs().IC_DMA_CR == 0u);

    Streamed::release();
}

TEST_CASE("a short read stays on the pump, and a bus error ends the tenure") {
    fresh();
    TxEngine::reset();
    RxEngine::reset();
    constexpr Clock clock;
    REQUIRE(Streamed::init(clock));

    // Two bytes are the first entry and the last: nothing for an engine
    // to pour.
    uint8_t in[2]{};
    Streamed::Request r{};
    r.addr = 0x48;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = 2;
    REQUIRE_FALSE(Streamed::start(r));
    CHECK(TxEngine::blocks == 0u);
    CHECK(RxEngine::blocks == 0u);
    CHECK(peer_regs().IC_DATA_CMD == i2c_read_entry({.stop = true}));

    // A long one, and a bus error on the transmit channel: the engines
    // are put away, the block told to abort, and the tenure ends with
    // the engine's own code.
    Streamed::Request big{};
    big.addr = 0x48;
    big.rx_len = 8;   // no buffer: the bytes are read and thrown away
    REQUIRE_FALSE(Streamed::start(big));
    CHECK(RxEngine::discarded);
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Streamed::dma_isr());
    CHECK(Streamed::status() == i2c_dma_fault);
    CHECK(Streamed::idle());
    CHECK(TxEngine::faults == 1u);
    CHECK((peer_regs().IC_ENABLE & I2cEnable::abort) != 0u);
    CHECK(peer_regs().IC_DMA_CR == 0u);

    Streamed::release();
}

TEST_CASE("a client answers its own address and reports one event per call") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Client::init(clock, {.address = 0x42, .ten_bit = true, .general_call = true,
                                 .speed = I2cSpeed::fast_400k}));
    CHECK(peer_regs().IC_SAR == 0x42u);
    CHECK(peer_regs().IC_ACK_GENERAL_CALL == 1u);
    CHECK((peer_regs().IC_CON & I2cControl::host_mode) == 0u);
    CHECK((peer_regs().IC_CON & I2cControl::ten_bit_client) != 0u);
    CHECK(peer_regs().IC_FS_SCL_HCNT == 111u);
    CHECK(peer_regs().IC_RX_TL == 0u);
    CHECK(SimDwApbI2cInterrupts::line_enabled[1]);

    Client::interrupts(Client::events, true);
    CHECK(peer_regs().IC_INTR_MASK == Client::events);

    // Nothing raised is no event.
    peer_regs().IC_INTR_STAT = 0;
    CHECK(Client::service() == I2cClientEvent::none);

    // THE LADDER, in the order the body reads it: an abort first,
    // whatever else stands with it.
    peer_regs().IC_INTR_STAT = I2cInterrupt::tx_abrt | I2cInterrupt::rd_req;
    peer_regs().IC_TX_ABRT_SOURCE = I2cAbort::client_flush;
    CHECK(Client::service() == I2cClientEvent::flushed);
    peer_regs().IC_INTR_STAT = I2cInterrupt::tx_abrt;
    peer_regs().IC_TX_ABRT_SOURCE = I2cAbort::client_arb_lost;
    CHECK(Client::service() == I2cClientEvent::error);

    peer_regs().IC_INTR_STAT = I2cInterrupt::rx_over | I2cInterrupt::rd_req;
    CHECK(Client::service() == I2cClientEvent::overrun);
    peer_regs().IC_INTR_STAT = I2cInterrupt::gen_call | I2cInterrupt::rd_req;
    CHECK(Client::service() == I2cClientEvent::general_call);

    // A read request outranks a byte received, and each says which way
    // the tenure is going.
    peer_regs().IC_INTR_STAT = I2cInterrupt::rd_req | I2cInterrupt::rx_full;
    CHECK(Client::service() == I2cClientEvent::byte_wanted);
    CHECK(Client::host_reads());
    peer_regs().IC_INTR_STAT = I2cInterrupt::rx_full;
    CHECK(Client::service() == I2cClientEvent::byte_received);
    CHECK_FALSE(Client::host_reads());

    peer_regs().IC_INTR_STAT = I2cInterrupt::rx_done;
    CHECK(Client::service() == I2cClientEvent::nacked);
    peer_regs().IC_INTR_STAT = I2cInterrupt::restart_det | I2cInterrupt::stop_det;
    CHECK(Client::service() == I2cClientEvent::restart);
    peer_regs().IC_INTR_STAT = I2cInterrupt::stop_det;
    CHECK(Client::service() == I2cClientEvent::stop);

    // The one verb that cycles the block: refusing every data byte is a
    // disable, a store and an enable.
    CHECK(Client::acknowledge(false));
    CHECK(peer_regs().IC_SLV_DATA_NACK_ONLY == 1u);
    CHECK((peer_regs().IC_ENABLE & I2cEnable::enable) != 0u);
    CHECK(Client::acknowledge(true));
    CHECK(peer_regs().IC_SLV_DATA_NACK_ONLY == 0u);

    Client::release();
    CHECK_FALSE(SimDwApbI2cInterrupts::line_enabled[1]);
    CHECK(SimDwApbI2cBench::pad_function[peer_pins.sda] == 0u);
}

TEST_CASE("a rate change re-solves the table and rewrites the applied row") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Host::init(clock));
    CHECK(Host::speed_ok(I2cSpeed::fast_plus_1m));

    // Down to a clock fast-mode-plus cannot reach: the row goes
    // unreachable and the applied one is rewritten for the new rate.
    Host::rebase(12'000'000);
    CHECK(Host::reference_hz() == 12'000'000u);
    CHECK_FALSE(Host::speed_ok(I2cSpeed::fast_plus_1m));
    CHECK(Host::speed_ok(I2cSpeed::fast_400k));
    CHECK(regs().IC_SS_SCL_HCNT == Host::timing_of(I2cSpeed::standard_100k).hcnt);
    CHECK(regs().IC_SS_SCL_LCNT == Host::timing_of(I2cSpeed::standard_100k).lcnt);
    CHECK((regs().IC_ENABLE & I2cEnable::enable) != 0u);
    CHECK(Host::scl_hz(I2cSpeed::standard_100k) == 100'000u);
}
