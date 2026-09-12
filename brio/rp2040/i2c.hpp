/*
 * i2c.hpp
 *
 * The RP2040's two I2C controllers (datasheet 4.3): the Synopsys
 * DW_apb_i2c, a host or a client (never both at once), standard, fast
 * and fast-mode-plus, 7-bit addresses on both sides and 10-bit ones,
 * two 16-deep FIFOs, one interrupt line per instance with thirteen
 * sources, a DMA request per FIFO. Three layers, the other strata's
 * arrangement:
 *
 *  - `DwApbI2c<n>` is the RESOURCE: the register block behind the reset
 *    controller's gate, the configuration under the block's rule
 *    (IC_CON, the counts, the addresses and the hold times take a write
 *    only with ENABLE clear), the command register, the flags, the
 *    interrupt sources and their read-to-clear registers, the abort
 *    source decoded. It decides nothing.
 *  - `I2cHost<n, pins, TxEngine, RxEngine>` is the ENGINE util/
 *    i2c_bus.hpp's I2cBus drives: the Request is the other strata's
 *    VERBATIM - {addr, tx span, rx span, reply, speed}, one bus tenure
 *    that is a write, a read, a write-then-read with a repeated START,
 *    or the probe - and the outcome vocabulary is i2c_bus.hpp's, read
 *    off IC_TX_ABRT_SOURCE.
 *  - `I2cClient<n, pins>` is the target side: a polled surface and one
 *    ISR body reporting one event per call, deliberately thin, because
 *    a client is a protocol and which protocol is the application's.
 *
 * THE MACHINE THIS CHAPTER PRESCRIBES IS A COMMAND FIFO, not an event
 * machine: a host writes IC_DATA_CMD entries - a data byte to write, or
 * a read command - each carrying a RESTART bit and a STOP bit, and the
 * controller strings them into a transaction: the first entry issues
 * START and the address held in IC_TAR, a direction change (or the
 * RESTART bit) issues a repeated START, the STOP bit issues STOP after
 * that entry. An EMPTY TRANSMIT FIFO WITH NO STOP HOLDS SCL LOW (4.3.7:
 * the bus stalls until the next entry), which is why the last entry of
 * every tenure carries STOP and the pump keeps the FIFO ahead of the
 * wire. Nothing stands for an address-only frame: the controller
 * cannot issue START, the address and STOP with no data entry, so THE
 * PROBE ON THIS SILICON IS A ONE-BYTE READ - START, the address with
 * the read bit, one byte taken and not acknowledged, STOP - the answer
 * still being the acknowledge on the address (i2c_nack_addr when
 * nobody is home). A NACK, a lost arbitration and every other failure
 * come back as ONE interrupt, TX_ABRT, with the reason in
 * IC_TX_ABRT_SOURCE, and the FIFOs are flushed and held so until
 * IC_CLR_TX_ABRT is read.
 *
 * THE SCL TIMING is two counts per speed (HCNT, LCNT in ic_clk cycles,
 * ic_clk = clk_sys), the spike filter (SPKLEN), and the SDA hold and
 * setup times, under the block's floors (4.3.14.1 with table 450: LCNT
 * at least SPKLEN + 7, HCNT at least SPKLEN + 5) and WITH THE CYCLES
 * THE BLOCK ADDS (4.3.14.1: the
 * low period is LCNT + 1, the high period is HCNT + SPKLEN + 7), so
 * `i2c_timing_for` subtracts them and a bus asked for 100 kHz runs at
 * 100 kHz, not at 96. The speed's minimum high and low times (the I2C
 * specification's, 4.3.14.3) bound the split, and a clock too slow to
 * meet them (table 450: 2.7 MHz for standard, 12 for fast, 32 for
 * fast-mode-plus) makes the speed unreachable - answered i2c_rejected.
 *
 * THE PINS are fixed per instance under function 3 (2.19.2, table 279):
 * I2C0 has SDA on GPIO 0, 4, 8, 12, 16, 20, 24, 28 and SCL on 1, 5, 9,
 * 13, 17, 21, 25, 29; I2C1 has SDA on 2, 6, 10, 14, 18, 22, 26 and SCL
 * on 3, 7, 11, 15, 19, 23, 27. The pads take the pull-up, the slew
 * limit and the Schmitt trigger 4.3.1.3 asks for; the board's own
 * pull-ups do the pulling.
 *
 * THE DMA ENGINES serve the READ phase: the read commands are one
 * constant halfword the transmit engine pours from a fixed cell (the
 * command register ignores the width of a write and replicates a byte
 * across the word, 2.1.4, so a command is written as a halfword), the
 * bytes come back through the receive engine, and the first command
 * (RESTART) and the last (STOP) are the pump's - the last written on
 * TX_EMPTY when the engine's block left no room, because the request
 * is a level the DMA banks credits on while the FIFO is empty and
 * spends in a burst that can fill it. A write phase stays on the pump:
 * its entries are the request's bytes with a flag each, and a byte
 * buffer cannot carry the flag.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <type_traits>

#include "rp2040/device.hpp"

#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

/// The three speeds the block runs (4.3.2; no high-speed mode).
enum class I2cSpeed : uint8_t { standard_100k = 0, fast_400k = 1, fast_plus_1m = 2 };

inline constexpr uint8_t i2c_speed_count = 3;

constexpr uint32_t i2c_speed_hz(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::fast_plus_1m: return 1'000'000UL;
        case I2cSpeed::fast_400k: return 400'000UL;
        default: return 100'000UL;
    }
}

/// IC_CON.SPEED: standard is 1, fast and fast-mode-plus share 2.
constexpr uint8_t i2c_speed_code(I2cSpeed s) {
    return s == I2cSpeed::standard_100k ? I2C_IC_CON_SPEED_VALUE_STANDARD : I2C_IC_CON_SPEED_VALUE_FAST;
}

/// The minimum ic_clk a speed needs (table 450).
constexpr uint32_t i2c_min_clk_hz(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::fast_plus_1m: return 32'000'000UL;
        case I2cSpeed::fast_400k: return 12'000'000UL;
        default: return 2'700'000UL;
    }
}

/// The specification's minimum high and low periods per speed, in ns
/// (4.3.14.3).
constexpr uint32_t i2c_min_high_ns(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::fast_plus_1m: return 260;
        case I2cSpeed::fast_400k: return 600;
        default: return 4000;
    }
}
constexpr uint32_t i2c_min_low_ns(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::fast_plus_1m: return 500;
        case I2cSpeed::fast_400k: return 1300;
        default: return 4700;
    }
}

/// The data hold this side provides as a transmitter (the
/// specification's 300 ns bridge across SCL's falling edge; 120 ns in
/// fast-mode-plus, the vendor's choice) and the data setup a client
/// transmitter gives before SCL rises (tSU;DAT: 250 / 100 / 50 ns).
constexpr uint32_t i2c_sda_hold_ns(I2cSpeed s) { return s == I2cSpeed::fast_plus_1m ? 120 : 300; }
constexpr uint32_t i2c_sda_setup_ns(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::fast_plus_1m: return 50;
        case I2cSpeed::fast_400k: return 100;
        default: return 250;
    }
}

/// The maximum spike the filter swallows, the specification's tSP.
inline constexpr uint32_t i2c_spike_ns = 50;

/// What the count registers hold for one speed at one ic_clk.
struct I2cTiming {
    uint16_t hcnt = 0;        ///< IC_*_SCL_HCNT: the high period less SPKLEN + 7
    uint16_t lcnt = 0;        ///< IC_*_SCL_LCNT: the low period less 1
    uint8_t spklen = 1;       ///< IC_FS_SPKLEN
    uint16_t sda_tx_hold = 1; ///< IC_SDA_HOLD[15:0]
    uint8_t sda_setup = 2;    ///< IC_SDA_SETUP
    I2cSpeed speed = I2cSpeed::standard_100k;
};

constexpr uint32_t i2c_cycles_ceil(uint32_t hz, uint32_t ns) {
    return static_cast<uint32_t>((static_cast<uint64_t>(hz) * ns + 999'999'999ULL) / 1'000'000'000ULL);
}

/// The chapter's arithmetic. The wire period is hz / speed cycles,
/// split into a high of at least the specification's minimum (and at
/// least what HCNT's floor allows) and a low of the rest, the low
/// taking the larger share (3/5, the vendor's ratio) when the period
/// leaves room; the cycles the block adds (SPKLEN + 7 to the high, 1
/// to the low) are subtracted from the programmed counts. Nullopt when
/// the clock is below the speed's floor, when the split cannot meet
/// both minima, or when the hold could not fit the low period
/// (IC_SDA_HOLD's rule: at most LCNT - 2).
constexpr std::optional<I2cTiming> i2c_timing_for(uint32_t hz, I2cSpeed speed) {
    if (hz < i2c_min_clk_hz(speed)) {
        return {};
    }
    const uint32_t baud = i2c_speed_hz(speed);
    const uint32_t period = (hz + baud / 2u) / baud;
    uint32_t spklen = i2c_cycles_ceil(hz, i2c_spike_ns);
    if (spklen < 1u) { spklen = 1u; }
    if (spklen > 255u) { return {}; }
    // The floors, as table 450's own rows have them (HCNT = SPKLEN + 5,
    // LCNT = SPKLEN + 7 at every speed's minimum clock; 4.3.14.1's
    // "larger than" is read as the table sets them): high >= 2 x SPKLEN
    // + 12, low >= SPKLEN + 8.
    const uint32_t high_floor = 2u * spklen + 12u;
    const uint32_t low_floor = spklen + 8u;
    uint32_t high_min = i2c_cycles_ceil(hz, i2c_min_high_ns(speed));
    if (high_min < high_floor) { high_min = high_floor; }
    uint32_t low_min = i2c_cycles_ceil(hz, i2c_min_low_ns(speed));
    if (low_min < low_floor) { low_min = low_floor; }
    uint32_t high = period * 2u / 5u;
    if (high < high_min) { high = high_min; }
    uint32_t low = period > high ? period - high : 0u;
    if (low < low_min) {
        low = low_min;
        high = period > low ? period - low : 0u;
        if (high < high_min) {
            return {};
        }
    }
    const uint32_t hcnt = high - (spklen + 7u);
    const uint32_t lcnt = low - 1u;
    if (hcnt > 0xFFFFu || lcnt > 0xFFFFu) {
        return {};
    }
    uint32_t hold = i2c_cycles_ceil(hz, i2c_sda_hold_ns(speed)) + 1u;
    if (hold + 2u > lcnt) {
        return {};
    }
    uint32_t setup = i2c_cycles_ceil(hz, i2c_sda_setup_ns(speed)) + 1u;
    if (setup < 2u) { setup = 2u; }
    if (setup > 255u) { setup = 255u; }
    return I2cTiming{static_cast<uint16_t>(hcnt), static_cast<uint16_t>(lcnt), static_cast<uint8_t>(spklen),
                     static_cast<uint16_t>(hold), static_cast<uint8_t>(setup), speed};
}

/// What a timing really produces on an ideal wire (figure 86 with the
/// rise and fall times at zero): hz over the two periods the block
/// generates.
constexpr uint32_t i2c_scl_hz(uint32_t hz, const I2cTiming& t) {
    const uint32_t cycles = (static_cast<uint32_t>(t.hcnt) + t.spklen + 7u) + (static_cast<uint32_t>(t.lcnt) + 1u);
    return cycles == 0u ? 0u : hz / cycles;
}

/// Which pins carry the two lines: each the instance's own under
/// function 3 (table 279).
struct I2cPins {
    uint8_t scl = 0xFFu;
    uint8_t sda = 0xFFu;
};

constexpr bool i2c_sda_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 1u) == 0u && ((p >> 1) & 1u) == n; }
constexpr bool i2c_scl_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 1u) == 1u && ((p >> 1) & 1u) == n; }
constexpr bool i2c_pins_valid(uint8_t n, const I2cPins& p) {
    return n < 2u && i2c_sda_pin(n, p.sda) && i2c_scl_pin(n, p.scl);
}

/// The pad configuration 4.3.1.3 asks for: pull-up, slew limited,
/// Schmitt trigger. The board's pull-ups do the pulling; the pad's is
/// a courtesy.
inline constexpr PinConfig i2c_pad_config{.pull = PinPull::up, .drive = PinDrive::ma4, .slew_fast = false,
                                          .schmitt = true, .input_enable = true};

enum class I2cRole : uint8_t { client = 0, host = 1 };

/// The whole of IC_CON.
struct I2cConfig {
    I2cRole role = I2cRole::host;
    I2cSpeed speed = I2cSpeed::standard_100k;
    bool ten_bit_host = false;      ///< the host addresses in 10 bits
    bool ten_bit_client = false;    ///< the client answers a 10-bit address
    bool restart_enabled = true;    ///< repeated STARTs allowed (a direction change needs one)
    bool stop_if_addressed = true;  ///< the client's STOP_DET only for its own tenures
    bool tx_empty_late = true;      ///< TX_EMPTY only once the last popped entry has left the shifter
    bool hold_when_rx_full = true;  ///< SCL held instead of an overrun with the receive FIFO full
};

constexpr uint32_t i2c_con_of(const I2cConfig& c) {
    uint32_t v = static_cast<uint32_t>(i2c_speed_code(c.speed)) << I2C_IC_CON_SPEED_LSB;
    if (c.role == I2cRole::host) {
        v |= I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS;
    }
    if (c.ten_bit_host) { v |= I2C_IC_CON_IC_10BITADDR_MASTER_BITS; }
    if (c.ten_bit_client) { v |= I2C_IC_CON_IC_10BITADDR_SLAVE_BITS; }
    if (c.restart_enabled) { v |= I2C_IC_CON_IC_RESTART_EN_BITS; }
    if (c.stop_if_addressed) { v |= I2C_IC_CON_STOP_DET_IFADDRESSED_BITS; }
    if (c.tx_empty_late) { v |= I2C_IC_CON_TX_EMPTY_CTRL_BITS; }
    if (c.hold_when_rx_full) { v |= I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_BITS; }
    return v;
}

/// One IC_DATA_CMD entry's flags.
struct I2cCmd {
    bool restart = false;
    bool stop = false;
};

constexpr uint16_t i2c_write_entry(uint8_t byte, I2cCmd c = {}) {
    return static_cast<uint16_t>(byte | (c.restart ? I2C_IC_DATA_CMD_RESTART_BITS : 0u) |
                                 (c.stop ? I2C_IC_DATA_CMD_STOP_BITS : 0u));
}
constexpr uint16_t i2c_read_entry(I2cCmd c = {}) {
    return static_cast<uint16_t>(I2C_IC_DATA_CMD_CMD_BITS | (c.restart ? I2C_IC_DATA_CMD_RESTART_BITS : 0u) |
                                 (c.stop ? I2C_IC_DATA_CMD_STOP_BITS : 0u));
}

/// IC_STATUS's flags.
struct I2cFlag {
    static constexpr uint32_t activity = I2C_IC_STATUS_ACTIVITY_BITS;
    static constexpr uint32_t tx_not_full = I2C_IC_STATUS_TFNF_BITS;
    static constexpr uint32_t tx_empty = I2C_IC_STATUS_TFE_BITS;
    static constexpr uint32_t rx_not_empty = I2C_IC_STATUS_RFNE_BITS;
    static constexpr uint32_t rx_full = I2C_IC_STATUS_RFF_BITS;
    static constexpr uint32_t host_active = I2C_IC_STATUS_MST_ACTIVITY_BITS;
    static constexpr uint32_t client_active = I2C_IC_STATUS_SLV_ACTIVITY_BITS;
};

/// The thirteen interrupt sources: one layout for IC_INTR_MASK,
/// IC_RAW_INTR_STAT and IC_INTR_STAT. TX_EMPTY and RX_FULL are levels
/// the hardware sets and clears; every other one clears by reading its
/// IC_CLR_* register (table 451).
struct I2cInterrupt {
    static constexpr uint32_t rx_under = I2C_IC_INTR_MASK_M_RX_UNDER_BITS;
    static constexpr uint32_t rx_over = I2C_IC_INTR_MASK_M_RX_OVER_BITS;
    static constexpr uint32_t rx_full = I2C_IC_INTR_MASK_M_RX_FULL_BITS;      ///< the receive FIFO at or above RX_TL + 1
    static constexpr uint32_t tx_over = I2C_IC_INTR_MASK_M_TX_OVER_BITS;
    static constexpr uint32_t tx_empty = I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;    ///< the transmit FIFO at or below TX_TL
    static constexpr uint32_t rd_req = I2C_IC_INTR_MASK_M_RD_REQ_BITS;        ///< a host reads this client: SCL held until a byte is given
    static constexpr uint32_t tx_abrt = I2C_IC_INTR_MASK_M_TX_ABRT_BITS;      ///< the transaction failed: the source says why
    static constexpr uint32_t rx_done = I2C_IC_INTR_MASK_M_RX_DONE_BITS;      ///< the host did not acknowledge a byte this client gave
    static constexpr uint32_t activity = I2C_IC_INTR_MASK_M_ACTIVITY_BITS;
    static constexpr uint32_t stop_det = I2C_IC_INTR_MASK_M_STOP_DET_BITS;
    static constexpr uint32_t start_det = I2C_IC_INTR_MASK_M_START_DET_BITS;
    static constexpr uint32_t gen_call = I2C_IC_INTR_MASK_M_GEN_CALL_BITS;
    static constexpr uint32_t restart_det = I2C_IC_INTR_MASK_M_RESTART_DET_BITS;
    static constexpr uint32_t all = 0x1FFFu;
};

/// The reasons IC_TX_ABRT_SOURCE names, as the resource decodes them.
struct I2cAbort {
    static constexpr uint32_t addr_noack = I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS |
                                           I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_BITS |
                                           I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_BITS |
                                           I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_BITS;
    static constexpr uint32_t data_noack = I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS;
    static constexpr uint32_t arb_lost = I2C_IC_TX_ABRT_SOURCE_ARB_LOST_BITS;
    static constexpr uint32_t user_abort = I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_BITS;
    static constexpr uint32_t client_flush = I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_BITS;   ///< a read arrived with stale bytes queued
    static constexpr uint32_t client_arb_lost = I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_BITS;
    static constexpr uint32_t client_read_cmd = I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_BITS;    ///< a client wrote a READ command
    static constexpr uint32_t flush_count_lsb = 23;
};

/// i2c_bus.hpp's code for an abort source.
constexpr uint8_t i2c_status_of_abort(uint32_t source) {
    if ((source & I2cAbort::arb_lost) != 0u) { return i2c_arb_lost; }
    if ((source & I2cAbort::addr_noack) != 0u) { return i2c_nack_addr; }
    if ((source & I2cAbort::data_noack) != 0u) { return i2c_nack_data; }
    return i2c_bus_error;
}

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
struct DwApbI2c {
    static_assert(n < 2, "the RP2040 has I2C0 and I2C1");
    DwApbI2c() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint32_t reset_bit = n == 0 ? ResetBlock::i2c0 : ResetBlock::i2c1;
    static constexpr uint8_t fifo_depth = 16;
    static constexpr Dreq dreq_tx = n == 0 ? Dreq::i2c0_tx : Dreq::i2c1_tx;
    static constexpr Dreq dreq_rx = n == 0 ? Dreq::i2c0_rx : Dreq::i2c1_rx;

    static I2C0_Type& regs() { return *(n == 0 ? I2C0 : I2C1); }
    static constexpr IRQn_Type irq() { return n == 0 ? I2C0_IRQ_IRQn : I2C1_IRQ_IRQn; }
    static volatile void* data_address() { return &regs().IC_DATA_CMD; }

    /// The block from its reset state (2.14): held, released, ready.
    static bool reset() { return Resets::cycle(reset_bit); }
    static bool released() { return Resets::released(reset_bit); }
    static void hold() { Resets::hold(reset_bit); }

    // ---- enable ---------------------------------------------------------------

    static bool enabled() { return (regs().IC_ENABLE & I2C_IC_ENABLE_ENABLE_BITS) != 0u; }
    /// What the hardware says (IC_ENABLE_STATUS.IC_EN): a disable takes
    /// until the bus activity under way completes (4.3.10.3).
    static bool running() { return (regs().IC_ENABLE_STATUS & I2C_IC_ENABLE_STATUS_IC_EN_BITS) != 0u; }
    static void enable() { hw_set(regs().IC_ENABLE, I2C_IC_ENABLE_ENABLE_BITS); }
    /// Disable and wait, bounded, for the hardware to report it: true
    /// when it did. A host mid-command with no STOP in sight never
    /// disables (the note in 4.3.10.3) - abort() first.
    static bool disable(uint32_t spins = 100'000u) {
        hw_clear(regs().IC_ENABLE, I2C_IC_ENABLE_ENABLE_BITS);
        while (running() && spins-- != 0u) {
        }
        return !running();
    }
    /// IC_ENABLE.ABORT: STOP after the entry under way, the transmit
    /// FIFO flushed, TX_ABRT raised with ABRT_USER_ABRT (4.3.10.4). A
    /// host verb; ignored unless enabled.
    static void abort() { hw_set(regs().IC_ENABLE, I2C_IC_ENABLE_ABORT_BITS); }
    static bool aborting() { return (regs().IC_ENABLE & I2C_IC_ENABLE_ABORT_BITS) != 0u; }
    /// The two facts a disable reports about a client tenure it cut.
    static bool client_disabled_while_busy() {
        return (regs().IC_ENABLE_STATUS & I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_BITS) != 0u;
    }
    static bool client_rx_data_lost() {
        return (regs().IC_ENABLE_STATUS & I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_BITS) != 0u;
    }

    // ---- configuration (ENABLE clear) ------------------------------------------

    /// IC_CON whole; refused while enabled (the block's rule: a write
    /// then has no effect).
    static bool configure(const I2cConfig& c) {
        if (enabled()) {
            return false;
        }
        regs().IC_CON = i2c_con_of(c);
        return true;
    }
    static uint32_t con() { return regs().IC_CON; }

    /// The counts of a speed into that speed's pair (standard into
    /// IC_SS_*, the two fast ones into IC_FS_*), the spike filter, the
    /// hold and the setup; refused while enabled.
    static bool timing(const I2cTiming& t) {
        if (enabled()) {
            return false;
        }
        if (t.speed == I2cSpeed::standard_100k) {
            regs().IC_SS_SCL_HCNT = t.hcnt;
            regs().IC_SS_SCL_LCNT = t.lcnt;
        } else {
            regs().IC_FS_SCL_HCNT = t.hcnt;
            regs().IC_FS_SCL_LCNT = t.lcnt;
        }
        regs().IC_FS_SPKLEN = t.spklen;
        regs().IC_SDA_HOLD = (regs().IC_SDA_HOLD & I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_BITS) | t.sda_tx_hold;
        regs().IC_SDA_SETUP = t.sda_setup;
        return true;
    }
    /// The receive-side hold (IC_SDA_HOLD[23:16]), 0 by default.
    static bool sda_rx_hold(uint8_t cycles) {
        if (enabled()) {
            return false;
        }
        regs().IC_SDA_HOLD = (regs().IC_SDA_HOLD & I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS) |
                             (static_cast<uint32_t>(cycles) << I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_LSB);
        return true;
    }
    static I2cTiming timing(I2cSpeed s) {
        I2cTiming t{};
        t.speed = s;
        if (s == I2cSpeed::standard_100k) {
            t.hcnt = static_cast<uint16_t>(regs().IC_SS_SCL_HCNT);
            t.lcnt = static_cast<uint16_t>(regs().IC_SS_SCL_LCNT);
        } else {
            t.hcnt = static_cast<uint16_t>(regs().IC_FS_SCL_HCNT);
            t.lcnt = static_cast<uint16_t>(regs().IC_FS_SCL_LCNT);
        }
        t.spklen = static_cast<uint8_t>(regs().IC_FS_SPKLEN);
        t.sda_tx_hold = static_cast<uint16_t>(regs().IC_SDA_HOLD & I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS);
        t.sda_setup = static_cast<uint8_t>(regs().IC_SDA_SETUP);
        return t;
    }

    /// The address the host's next transaction goes to (IC_TAR, 7 or 10
    /// bits; `general_call` sends the general call address instead);
    /// refused while enabled.
    static bool target(uint16_t addr, bool general_call = false) {
        if (enabled()) {
            return false;
        }
        regs().IC_TAR = (static_cast<uint32_t>(addr) & I2C_IC_TAR_IC_TAR_BITS) |
                        (general_call ? I2C_IC_TAR_SPECIAL_BITS : 0u);
        return true;
    }
    static uint16_t target() { return static_cast<uint16_t>(regs().IC_TAR & I2C_IC_TAR_IC_TAR_BITS); }
    /// The client's own address (IC_SAR); refused while enabled.
    static bool own_address(uint16_t addr) {
        if (enabled()) {
            return false;
        }
        regs().IC_SAR = static_cast<uint32_t>(addr) & I2C_IC_SAR_IC_SAR_BITS;
        return true;
    }
    static uint16_t own_address() { return static_cast<uint16_t>(regs().IC_SAR & I2C_IC_SAR_IC_SAR_BITS); }
    /// Whether the client answers the general call (0x00).
    static void ack_general_call(bool on) { regs().IC_ACK_GENERAL_CALL = on ? 1u : 0u; }
    /// IC_SLV_DATA_NACK_ONLY: a client that refuses every data byte
    /// (the byte is not stored); written with the block disabled and
    /// the client idle, refused otherwise.
    static bool nack_data(bool on) {
        if (enabled() || (flags() & I2cFlag::client_active) != 0u) {
            return false;
        }
        regs().IC_SLV_DATA_NACK_ONLY = on ? 1u : 0u;
        return true;
    }

    /// The FIFO thresholds: RX_FULL at `rx` + 1 entries or more,
    /// TX_EMPTY at `tx` entries or fewer. Live.
    static void rx_threshold(uint8_t entries_minus_one) { regs().IC_RX_TL = entries_minus_one; }
    static void tx_threshold(uint8_t entries) { regs().IC_TX_TL = entries; }

    // ---- data and status --------------------------------------------------------

    static uint32_t flags() { return regs().IC_STATUS; }
    static bool tx_not_full() { return (flags() & I2cFlag::tx_not_full) != 0u; }
    static bool tx_empty() { return (flags() & I2cFlag::tx_empty) != 0u; }
    static bool rx_not_empty() { return (flags() & I2cFlag::rx_not_empty) != 0u; }
    static bool rx_full() { return (flags() & I2cFlag::rx_full) != 0u; }
    static bool activity() { return (flags() & I2cFlag::activity) != 0u; }
    static bool host_active() { return (flags() & I2cFlag::host_active) != 0u; }
    static bool client_active() { return (flags() & I2cFlag::client_active) != 0u; }
    static uint8_t tx_count() { return static_cast<uint8_t>(regs().IC_TXFLR & I2C_IC_TXFLR_TXFLR_BITS); }
    static uint8_t rx_count() { return static_cast<uint8_t>(regs().IC_RXFLR & I2C_IC_RXFLR_RXFLR_BITS); }

    /// One IC_DATA_CMD entry (i2c_write_entry / i2c_read_entry).
    static void push(uint16_t entry) { regs().IC_DATA_CMD = entry; }
    /// One received byte.
    static uint8_t pop() { return static_cast<uint8_t>(regs().IC_DATA_CMD & I2C_IC_DATA_CMD_DAT_BITS); }
    /// Whether the last pop() was the first byte after an address phase.
    static bool first_data_byte() { return (regs().IC_DATA_CMD & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) != 0u; }
    static void flush_rx() {
        while (rx_not_empty()) {
            (void)regs().IC_DATA_CMD;
        }
    }

    // ---- interrupts ---------------------------------------------------------------

    static void interrupts(uint32_t mask, bool on) {
        if (on) { hw_set(regs().IC_INTR_MASK, mask); } else { hw_clear(regs().IC_INTR_MASK, mask); }
    }
    static void interrupts_only(uint32_t mask) { regs().IC_INTR_MASK = mask; }
    static uint32_t interrupts() { return regs().IC_INTR_MASK; }
    static uint32_t raw_pending() { return regs().IC_RAW_INTR_STAT; }
    static uint32_t pending() { return regs().IC_INTR_STAT; }

    /// Clear the read-to-clear sources named in `mask` (the two levels
    /// clear by moving data and are ignored here).
    static void clear_pending(uint32_t mask) {
        if ((mask & I2cInterrupt::rx_under) != 0u) { (void)regs().IC_CLR_RX_UNDER; }
        if ((mask & I2cInterrupt::rx_over) != 0u) { (void)regs().IC_CLR_RX_OVER; }
        if ((mask & I2cInterrupt::tx_over) != 0u) { (void)regs().IC_CLR_TX_OVER; }
        if ((mask & I2cInterrupt::rd_req) != 0u) { (void)regs().IC_CLR_RD_REQ; }
        if ((mask & I2cInterrupt::tx_abrt) != 0u) { (void)regs().IC_CLR_TX_ABRT; }
        if ((mask & I2cInterrupt::rx_done) != 0u) { (void)regs().IC_CLR_RX_DONE; }
        if ((mask & I2cInterrupt::activity) != 0u) { (void)regs().IC_CLR_ACTIVITY; }
        if ((mask & I2cInterrupt::stop_det) != 0u) { (void)regs().IC_CLR_STOP_DET; }
        if ((mask & I2cInterrupt::start_det) != 0u) { (void)regs().IC_CLR_START_DET; }
        if ((mask & I2cInterrupt::gen_call) != 0u) { (void)regs().IC_CLR_GEN_CALL; }
        if ((mask & I2cInterrupt::restart_det) != 0u) { (void)regs().IC_CLR_RESTART_DET; }
    }
    /// Every read-to-clear source at once (IC_CLR_INTR), the abort
    /// source with them.
    static void clear_all() { (void)regs().IC_CLR_INTR; }

    /// Why the last TX_ABRT happened (I2cAbort's bits; the count of
    /// flushed entries in the top nine). Standing until the abort is
    /// cleared.
    static uint32_t abort_source() { return regs().IC_TX_ABRT_SOURCE; }
    static uint16_t flushed_entries() { return static_cast<uint16_t>(abort_source() >> I2cAbort::flush_count_lsb); }
    /// Read the abort away: the FIFOs come out of their flushed state
    /// (4.3.10.1.2's note). Returns the source it stood for.
    static uint32_t clear_abort() {
        const uint32_t src = abort_source();
        (void)regs().IC_CLR_TX_ABRT;
        return src;
    }

    /// The raised-and-enabled sources, the way the other strata's ISR
    /// bodies answer; nothing cleared - each source's owner clears it.
    [[gnu::always_inline]] static uint32_t isr() { return pending(); }

    /// The DMA request enables (IC_DMA_CR).
    static void dma_requests(bool tx, bool rx) {
        regs().IC_DMA_CR = (tx ? I2C_IC_DMA_CR_TDMAE_BITS : 0u) | (rx ? I2C_IC_DMA_CR_RDMAE_BITS : 0u);
    }
    /// The DMA watermarks (IC_DMA_TDLR / IC_DMA_RDLR), 0 at reset and
    /// left there: the block requests single transfers (4.3.15.3).
    static void dma_levels(uint8_t tx, uint8_t rx) {
        regs().IC_DMA_TDLR = tx & I2C_IC_DMA_TDLR_DMATDL_BITS;
        regs().IC_DMA_RDLR = rx & I2C_IC_DMA_RDLR_DMARDL_BITS;
    }

    /// IC_ENABLE.TX_CMD_BLOCK: entries queue and nothing executes until
    /// it is cleared (set only with the FIFO empty and the host idle).
    static void command_block(bool on) {
        if (on) { hw_set(regs().IC_ENABLE, I2C_IC_ENABLE_TX_CMD_BLOCK_BITS); }
        else { hw_clear(regs().IC_ENABLE, I2C_IC_ENABLE_TX_CMD_BLOCK_BITS); }
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish: the engine's own code, in
/// the range util/bus_master.hpp leaves to engines and above the four
/// i2c_bus.hpp holds.
inline constexpr uint8_t i2c_dma_fault = bus_engine_status + 4;

/**
 * I2cHost<n, pins, TxEngine, RxEngine>
 *
 * The engine I2cBus (util/i2c_bus.hpp = BusMaster) drives. One
 * Request is one bus tenure: START, the address, tx_len bytes written,
 * then - if rx_len is not zero - a repeated START, the address again
 * with the read bit and rx_len bytes taken with the last one NACKed,
 * then STOP. Both spans empty is the PROBE, served as a one-byte read
 * (the file header: this controller has no address-only entry) - the
 * ACK on the address is still the answer. Every outcome the wire can
 * give comes back as i2c_bus.hpp's codes through TransferDone{status()}.
 *
 * THE PUMP writes IC_DATA_CMD entries while the transmit FIFO takes
 * them and refills on TX_EMPTY (TX_TL at the FIFO's half), takes the
 * bytes read on RX_FULL (RX_TL at 0: every byte), ends a write on
 * STOP_DET and a read when the last byte is in, and ends anything on
 * TX_ABRT with the abort source decoded. TX_EMPTY is a level: it is
 * enabled only while entries remain to be written.
 *
 * `speed` names a row of the timing table init() solves for clk_sys;
 * a speed the clock cannot produce is answered i2c_rejected inside
 * start(), no byte moved - the one synchronous completion of an I2C
 * engine, and the arbiter replies with status() for it. The address
 * and the speed go into the block under a disable/enable pair at every
 * start() (IC_TAR and IC_CON take a write only with ENABLE clear); a
 * block that will not disable - a command under way with no STOP,
 * which a finished tenure never leaves - is answered i2c_bus_error the
 * same way.
 *
 * THE ENGINE SLOTS: a DmaTxEngine of uint16_t and a DmaRxEngine of
 * uint8_t on any two channels, both or neither, serving a READ phase of
 * three bytes or more (the file header): the transmit engine pours the
 * plain read command from a fixed cell for the middle entries, the
 * receive engine collects every byte, the pump writes the first entry
 * (RESTART) and, once the block of commands is in, the last (STOP).
 */
template <uint8_t n, I2cPins pins, typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
class I2cHost {
    using S = DwApbI2c<n>;
    static_assert(i2c_pins_valid(n, pins),
                  "brio I2cHost: these pins cannot carry this I2C (2.19.2, table 279: under function 3 "
                  "I2C0 has SDA on GPIO 0, 4, 8, 12, 16, 20, 24, 28 and SCL on 1, 5, 9, 13, 17, 21, 25, 29; "
                  "I2C1 has SDA on 2, 6, 10, 14, 18, 22, 26 and SCL on 3, 7, 11, 15, 19, 23, 27)");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0, "the engine slots must name a complete type");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio I2cHost: name both DMA engines or neither - a read phase needs the commands poured "
                  "and the bytes collected");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "brio I2cHost: the two engines must ride two different DMA channels");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint16_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio I2cHost: the transmit engine carries uint16_t entries (IC_DATA_CMD is a command word and "
         "a narrow write is replicated across it, 2.1.4), the receive engine uint8_t bytes");

    using SclPin = Pin<pins.scl>;
    using SdaPin = Pin<pins.sda>;

public:
    I2cHost() = delete;
    using Resource = S;
    static constexpr I2cPins pin_set = pins;
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        uint8_t addr;          ///< 7-bit client address (unshifted)
        /// Bytes written after START, LENT until the reply lands (may be
        /// null if tx_len == 0).
        Borrowed<const uint8_t, Lease::reply> tx;
        uint8_t tx_len;
        /// Where the bytes read after the (repeated) START+R land; LENT
        /// until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        I2cSpeed speed = I2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a bus host. `clock` is the app's Clock
    /// tag, the one truth of ic_clk (clk_sys). The three speeds are
    /// solved here and at rebase(); one the clock cannot produce is
    /// marked unreachable (speed_ok()) and a Request naming it is
    /// answered i2c_rejected. False when not even standard_100k is
    /// legal (a clock under 2.7 MHz) or the block did not come up.
    template <typename Clock>
    static bool init(Clock clock) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not list it among its "
                      "Users: its timing table would go stale on a clock change");
        Nvic::disable(S::irq());
        solve(clock_hz(clock));
        if (!valid_[0]) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        applied_ = I2cSpeed::standard_100k;
        config_ = I2cConfig{};
        config_.role = I2cRole::host;
        config_.speed = applied_;
        if (!S::configure(config_) || !S::timing(table_[0])) {
            return false;
        }
        S::rx_threshold(0);
        S::tx_threshold(S::fifo_depth / 2u);
        S::interrupts_only(0);
        S::clear_all();
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address(), S::dreq_tx);
            RxEngine::arm(S::data_address(), S::dreq_rx);
        }
        // The pads go to the peripheral with the block still disabled:
        // its outputs idle released, the pull-ups own the level.
        SclPin::function(PinFunction::i2c, i2c_pad_config);
        SdaPin::function(PinFunction::i2c, i2c_pad_config);
        status_ = i2c_ok;
        phase_ = Phase::idle;
        S::enable();
        Nvic::enable(S::irq());
        return true;
    }

    /// clk_sys changed (DynamicClock fan-out): the three rows solved
    /// again and the applied one rewritten. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        solve(hz);
        if (S::enabled() && valid_[static_cast<uint8_t>(applied_)]) {
            (void)S::disable();
            (void)S::timing(table_[static_cast<uint8_t>(applied_)]);
            S::enable();
        }
    }

    static bool speed_ok(I2cSpeed s) { return valid_[static_cast<uint8_t>(s)]; }
    static I2cTiming timing_of(I2cSpeed s) { return table_[static_cast<uint8_t>(s)]; }
    static uint32_t scl_hz(I2cSpeed s) { return i2c_scl_hz(hz_, table_[static_cast<uint8_t>(s)]); }
    static uint32_t reference_hz() { return hz_; }
    /// The ENGINE's phase, not the wire's (the SAM's idle()).
    static bool idle() { return phase_ == Phase::idle; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a tenure (I2cBus calls it from main context). False
    /// whenever the wire moves: the tenure completes on the vector and
    /// a TransferDone{status()} follows. True for the refusals that
    /// move nothing - a speed the clock cannot produce (i2c_rejected),
    /// a block that would not disable to take the address
    /// (i2c_bus_error) - answered through status(), the I2cHost
    /// contract (docs/design/i2c-bus.md).
    static bool start(const Request& r) {
        req_ = r;
        issued_ = 0;
        received_ = 0;
        stop_seen_ = false;
        dma_active_ = false;
        if (!speed_ok(r.speed)) {
            status_ = i2c_rejected;
            phase_ = Phase::idle;
            return true;
        }
        // The address and the speed take a write only with ENABLE
        // clear; a finished tenure leaves the block able to disable at
        // once (its last entry carried STOP).
        if (!S::disable()) {
            status_ = i2c_bus_error;
            phase_ = Phase::idle;
            return true;
        }
        apply(r.speed);
        (void)S::target(r.addr);
        S::clear_all();
        S::enable();
        S::flush_rx();
        status_ = i2c_ok;
        phase_ = r.tx_len != 0u ? Phase::write : Phase::read;
        S::interrupts_only(I2cInterrupt::tx_abrt | I2cInterrupt::stop_det | I2cInterrupt::rx_full);
        fill();
        return false;
    }

    /// The engine's completion status, for the TransferDone payload.
    static uint8_t status() { return status_; }

    /// The instance's interrupt body - call from its vector. True when
    /// the tenure just completed: the edge the app's glue posts
    /// TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t up = S::isr();
        if (phase_ == Phase::idle) {
            // Nothing in flight: whatever stands is stale (a STOP_DET
            // from a finished tenure, an abort already reported).
            S::clear_all();
            S::interrupts_only(0);
            return false;
        }
        if ((up & I2cInterrupt::tx_abrt) != 0u) {
            const uint32_t src = S::clear_abort();
            if constexpr (has_engines) {
                put_engines_away();
            }
            return finish(i2c_status_of_abort(src));
        }
        if ((up & I2cInterrupt::rx_full) != 0u) {
            take();
        }
        if ((up & I2cInterrupt::stop_det) != 0u) {
            S::clear_pending(I2cInterrupt::stop_det);
            stop_seen_ = true;
        }
        if ((up & I2cInterrupt::tx_empty) != 0u) {
            fill();
        }
        // A write ends on its STOP; a read when the last byte is in.
        const uint8_t want = read_len();
        if (want == 0u) {
            if (stop_seen_ && issued_ >= total_entries()) {
                return finish(i2c_ok);
            }
            return false;
        }
        if (received_ >= want) {
            return finish(i2c_ok);
        }
        return false;
    }

    /// The DMA line's interrupt body - call it from the line the engines
    /// report on. The transmit block ending hands the pump the STOP
    /// entry; the receive block ending is the tenure's end; a bus error
    /// on either channel ends it with i2c_dma_fault. Compiles away on
    /// an engineless host. True when the tenure just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if ((tx & TxEngine::flag_error) != 0u) {
                put_engines_away();
                S::abort();
                return finish(i2c_dma_fault);
            }
            if ((tx & TxEngine::flag_complete) != 0u && dma_active_) {
                (void)TxEngine::complete();
                S::dma_requests(false, true);
                // The block of plain read commands is in the FIFO - up to
                // its whole depth at once, since the request is a level
                // the DMA banks credits on while the FIFO is empty and
                // spends in a burst (measured: a STOP entry pushed here
                // by hand was dropped with TX_OVER). The last entry goes
                // through the pump, which writes when there is room and
                // waits on TX_EMPTY otherwise.
                issued_ = static_cast<uint16_t>(total_entries() - 1u);
                fill();
            }
            const uint8_t rx = RxEngine::service();
            if (rx != 0u && dma_active_) {
                if ((rx & RxEngine::flag_error) != 0u) {
                    put_engines_away();
                    S::abort();
                    return finish(i2c_dma_fault);
                }
                if ((rx & RxEngine::flag_complete) != 0u) {
                    S::dma_requests(false, false);
                    dma_active_ = false;
                    received_ = read_len();
                    return finish(i2c_ok);
                }
            }
        }
        return false;
    }

    /// The classic bus unstick: nine SCL pulses and a STOP, by hand,
    /// open-drain through the SIO (the pad driven low or released to the
    /// pull-up), with the pads reclaimed from the peripheral for the
    /// duration. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE WIRE.
    /// Returns the pulses it took a stuck client to release SDA (0 =
    /// the wire was never stuck), or 0xFF when nine pulses and a STOP
    /// left SDA low - a short, not a client. A healthy wire is left
    /// alone.
    static uint8_t unstick() {
        Nvic::disable(S::irq());
        SclPin::input(PinPull::up);
        SdaPin::input(PinPull::up);
        // Open drain by hand: OUT held low, OE is the drive.
        SclPin::clear();
        SdaPin::clear();

        uint8_t released_at = 0xFF;
        bool free_now = SdaPin::read();
        if (!free_now) {
            uint8_t pulses = 0;
            for (uint8_t i = 0; i < 9u && released_at == 0xFF; ++i) {
                Gpio::oe_set(1u << pins.scl);     // SCL low
                spin_half_bit();
                Gpio::oe_clear(1u << pins.scl);   // SCL released
                spin_half_bit();
                ++pulses;
                if (SdaPin::read()) {
                    released_at = pulses;
                }
            }
            // A STOP: SDA low while SCL is high, then SDA released.
            Gpio::oe_set(1u << pins.sda);
            spin_half_bit();
            Gpio::oe_clear(1u << pins.sda);
            spin_half_bit();
            free_now = SdaPin::read();
        } else {
            released_at = 0;
        }

        SclPin::function(PinFunction::i2c, i2c_pad_config);
        SdaPin::function(PinFunction::i2c, i2c_pad_config);
        Nvic::enable(S::irq());
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    /// Put the ENGINE back where start() is legal - the verb a timed
    /// I2cBus calls on a tenure that never answered: the engines put
    /// away, the block reset (a command under way with no STOP holds
    /// the block enabled forever, and the reset controller is the one
    /// way out), the configuration and the timing rewritten. The WIRE
    /// stays the application's (unstick()).
    static bool recover() {
        Nvic::disable(S::irq());
        if constexpr (has_engines) {
            put_engines_away();
        }
        phase_ = Phase::idle;
        dma_active_ = false;
        const bool ok = S::reset();
        const bool cfg = S::configure(config_) && S::timing(table_[static_cast<uint8_t>(applied_)]);
        S::rx_threshold(0);
        S::tx_threshold(S::fifo_depth / 2u);
        S::interrupts_only(0);
        S::clear_all();
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address(), S::dreq_tx);
            RxEngine::arm(S::data_address(), S::dreq_rx);
        }
        S::enable();
        Nvic::enable(S::irq());
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Nvic::disable(S::irq());
        S::interrupts_only(0);
        (void)S::disable();
        SclPin::release();
        SdaPin::release();
        S::hold();
    }

private:
    enum class Phase : uint8_t { idle, write, read };

    /// How many bytes the read phase takes: the request's, or ONE for
    /// the probe (the file header).
    static uint8_t read_len() {
        if (req_.tx_len == 0u && req_.rx_len == 0u) {
            return 1;
        }
        return req_.rx_len;
    }
    static uint16_t total_entries() { return static_cast<uint16_t>(req_.tx_len) + read_len(); }

    /// The engines serve a read phase of three entries or more: the
    /// first and the last are the pump's, the middle block the engine's.
    static bool dma_serves() {
        if constexpr (has_engines) {
            return read_len() >= 3u;
        } else {
            return false;
        }
    }

    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        S::interrupts_only(0);
        return true;
    }

    /// Write entries while the transmit FIFO takes them: the write
    /// phase's bytes, then the read commands (the first with RESTART
    /// when a write preceded it, the last with STOP). TX_EMPTY stays
    /// enabled only while entries remain.
    static void fill() {
        const uint16_t total = total_entries();
        const uint8_t want = read_len();
        while (S::tx_not_full() && issued_ < total) {
            if (issued_ < req_.tx_len) {
                const bool last = (issued_ + 1u == total);
                S::push(i2c_write_entry(req_.tx.get()[issued_], {.stop = last}));
                ++issued_;
                continue;
            }
            const uint8_t k = static_cast<uint8_t>(issued_ - req_.tx_len);   // the k-th read entry
            if constexpr (has_engines) {
                if (dma_serves() && k == 1u) {
                    // The first read entry is in: the engines take the
                    // middle block, dma_isr() the last entry.
                    S::interrupts(I2cInterrupt::tx_empty | I2cInterrupt::rx_full, false);
                    launch_dma();
                    return;
                }
            }
            const bool first = (k == 0u && req_.tx_len != 0u);
            const bool last = (k + 1u == want);
            S::push(i2c_read_entry({.restart = first, .stop = last}));
            ++issued_;
        }
        S::interrupts(I2cInterrupt::tx_empty, issued_ < total);
    }

    /// Take what came back into the request's buffer (the probe's byte
    /// into the sink).
    static void take() {
        const uint8_t want = read_len();
        while (S::rx_not_empty() && received_ < want) {
            const uint8_t b = S::pop();
            if (req_.rx.get() != nullptr) {
                req_.rx.get()[received_] = b;
            }
            ++received_;
        }
        if (S::rx_not_empty()) {
            S::flush_rx();   // more than asked: never, but never a stall
        }
    }

    static void launch_dma() {
        if constexpr (has_engines) {
            dma_active_ = true;
            const uint8_t want = read_len();
            // The receive channel first: every byte of the phase,
            // including the one the pump's first entry brings back.
            if (req_.rx.get() != nullptr) {
                (void)RxEngine::start(req_.rx.get(), want);
            } else {
                (void)RxEngine::start_discard(&rx_sink_, want);
            }
            S::dma_requests(false, true);
            (void)TxEngine::start_fixed(&read_cmd_, static_cast<uint32_t>(want) - 2u);
            S::dma_requests(true, true);
        }
    }

    static void put_engines_away() {
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dreq_rx);
            dma_active_ = false;
        }
    }

    /// The block is disabled here (start() did it).
    static void apply(I2cSpeed s) {
        if (s == applied_) {
            return;
        }
        applied_ = s;
        config_.speed = s;
        (void)S::configure(config_);
        (void)S::timing(table_[static_cast<uint8_t>(s)]);
    }

    static void solve(uint32_t hz) {
        hz_ = hz;
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const auto t = i2c_timing_for(hz, static_cast<I2cSpeed>(i));
            valid_[i] = t.has_value();
            table_[i] = t.value_or(I2cTiming{});
        }
        delay_rate_ = delay_rate(hz);
    }

    /// Half a standard-mode bit, for the unstick.
    static void spin_half_bit() { (void)delay_us(delay_rate_, 5); }

    static inline Request req_{};
    static inline uint16_t issued_ = 0;
    static inline uint8_t received_ = 0;
    static inline volatile Phase phase_ = Phase::idle;
    static inline bool stop_seen_ = false;
    static inline uint8_t status_ = i2c_ok;
    static inline volatile bool dma_active_ = false;
    static inline uint8_t rx_sink_ = 0;
    static constexpr uint16_t read_cmd_ = i2c_read_entry({});
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cConfig config_{};
    static inline uint32_t hz_ = 0;
    static inline I2cTiming table_[i2c_speed_count]{};
    static inline bool valid_[i2c_speed_count]{};
    static inline DelayRate delay_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/// What the client's ISR body reports: one thing per call, the app's
/// glue acts on it.
enum class I2cClientEvent : uint8_t {
    none,
    byte_received,   ///< RX_FULL: take() it
    byte_wanted,     ///< RD_REQ: give() the next, SCL held meanwhile
    stop,            ///< STOP_DET: the tenure ended
    restart,         ///< RESTART_DET: a repeated START while addressed
    nacked,          ///< RX_DONE: the host refused a byte this client gave - the end of a read
    general_call,    ///< GEN_CALL: the general call answered
    flushed,         ///< TX_ABRT with SLVFLUSH: a read arrived with stale bytes queued, thrown away
    overrun,         ///< RX_OVER: the receive FIFO overflowed
    error,           ///< TX_ABRT for any other reason
};

/**
 * I2cClient<n, pins>
 *
 * The target side. The peripheral matches IC_SAR, acknowledges, and
 * then: a byte written to it lands in the receive FIFO (RX_FULL), a
 * read of it raises RD_REQ and HOLDS SCL until a byte is given (4.3.5.1
 * - the clock stretch is the block's own); the host's NACK at the end
 * of a read is RX_DONE; a repeated START while addressed is
 * RESTART_DET; the STOP is STOP_DET, raised for this client's tenures
 * alone (STOP_DET_IFADDRESSED). A read arriving with bytes still queued
 * from an earlier one flushes them and says so (ABRT_SLVFLUSH_TXFIFO);
 * bytes queued beyond what the host takes are flushed at its NACK the
 * same way. There is no address-match event: the tenure is known by
 * its first byte or its first read request.
 *
 *   extern "C" void isr_i2c1() { act(Peer::service()); }
 */
template <uint8_t n, I2cPins pins>
class I2cClient {
    using S = DwApbI2c<n>;
    static_assert(i2c_pins_valid(n, pins), "brio I2cClient: these pins cannot carry this I2C (table 279)");
    using SclPin = Pin<pins.scl>;
    using SdaPin = Pin<pins.sda>;

public:
    I2cClient() = delete;
    using Resource = S;
    static constexpr I2cPins pin_set = pins;

    struct Config {
        uint16_t address = 0x55;        ///< 7 bits, or 10 with ten_bit
        bool ten_bit = false;
        bool general_call = false;      ///< answer 0x00 too
        /// The fastest bus this client will see: the spike filter and
        /// the setup time follow it.
        I2cSpeed speed = I2cSpeed::fast_400k;
    };

    /// Bring the instance up as a bus target. `clock` is the app's
    /// Clock tag (ic_clk = clk_sys). False when the block did not come
    /// up or the speed's timing cannot be met at this clock.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg) {
        Nvic::disable(S::irq());
        if (!S::reset()) {
            return false;
        }
        const auto t = i2c_timing_for(clock_hz(clock), cfg.speed);
        if (!t) {
            return false;
        }
        I2cConfig c{};
        c.role = I2cRole::client;
        c.speed = cfg.speed;
        c.ten_bit_client = cfg.ten_bit;
        if (!S::configure(c) || !S::timing(*t) || !S::own_address(cfg.address)) {
            return false;
        }
        S::ack_general_call(cfg.general_call);
        S::rx_threshold(0);
        S::interrupts_only(0);
        S::clear_all();
        SclPin::function(PinFunction::i2c, i2c_pad_config);
        SdaPin::function(PinFunction::i2c, i2c_pad_config);
        host_reads_ = false;
        S::enable();
        Nvic::enable(S::irq());
        return true;
    }

    /// The sources service() reports on, enabled in one word.
    static constexpr uint32_t events = I2cInterrupt::rx_full | I2cInterrupt::rd_req | I2cInterrupt::stop_det |
                                       I2cInterrupt::restart_det | I2cInterrupt::rx_done | I2cInterrupt::gen_call |
                                       I2cInterrupt::tx_abrt | I2cInterrupt::rx_over;
    static void interrupts(uint32_t mask, bool on) { S::interrupts(mask, on); }

    // ---- the protocol surface, polled -----------------------------------------

    /// Host write: a byte has arrived.
    static bool data_ready() { return S::rx_not_empty(); }
    static uint8_t take() { return S::pop(); }
    /// Host read: the shifter wants a byte and SCL is held until it gets
    /// one; give() up to the FIFO's depth ahead.
    static bool data_wanted() { return (S::raw_pending() & I2cInterrupt::rd_req) != 0u; }
    static void give(uint8_t v) { S::push(v); }
    static bool writable() { return S::tx_not_full(); }
    /// The read request answered (the flag cleared): after give().
    static void clear_read_request() { S::clear_pending(I2cInterrupt::rd_req); }
    /// A STOP arrived - the end of a tenure of this client's.
    static bool stop_seen() { return (S::raw_pending() & I2cInterrupt::stop_det) != 0u; }
    static void clear_stop() { S::clear_pending(I2cInterrupt::stop_det); }
    /// The host refused a byte this client gave: the end of a read.
    static bool host_nacked() { return (S::raw_pending() & I2cInterrupt::rx_done) != 0u; }
    static void clear_nack() { S::clear_pending(I2cInterrupt::rx_done); }
    static bool overrun() { return (S::raw_pending() & I2cInterrupt::rx_over) != 0u; }
    static void clear_overrun() { S::clear_pending(I2cInterrupt::rx_over); }
    static bool general_call_seen() { return (S::raw_pending() & I2cInterrupt::gen_call) != 0u; }
    static uint32_t flags() { return S::flags(); }
    static uint32_t raw_pending() { return S::raw_pending(); }
    /// Whether to acknowledge the data bytes that arrive from here on
    /// (IC_SLV_DATA_NACK_ONLY, which takes a write with the block
    /// disabled and idle: a disable/enable pair, refused mid-tenure).
    static bool acknowledge(bool on) {
        if (!S::disable()) {
            return false;
        }
        const bool ok = S::nack_data(!on);
        S::enable();
        return ok;
    }
    /// Throw away what is queued to give.
    static void flush_tx() {
        (void)S::disable();
        S::enable();
    }

    // ---- the ISR body -----------------------------------------------------------

    /// One event per call, the flag consumed where the chapter consumes
    /// it (STOP_DET, RESTART_DET, RX_DONE, GEN_CALL, TX_ABRT, RX_OVER
    /// here; RX_FULL by take(), RD_REQ by give() + clear_read_request(),
    /// which the glue owes).
    [[gnu::always_inline]] static I2cClientEvent service() {
        const uint32_t up = S::isr();
        if ((up & I2cInterrupt::tx_abrt) != 0u) {
            const uint32_t src = S::clear_abort();
            return (src & I2cAbort::client_flush) != 0u ? I2cClientEvent::flushed : I2cClientEvent::error;
        }
        if ((up & I2cInterrupt::rx_over) != 0u) {
            S::clear_pending(I2cInterrupt::rx_over);
            return I2cClientEvent::overrun;
        }
        if ((up & I2cInterrupt::gen_call) != 0u) {
            S::clear_pending(I2cInterrupt::gen_call);
            return I2cClientEvent::general_call;
        }
        if ((up & I2cInterrupt::rd_req) != 0u) {
            host_reads_ = true;
            return I2cClientEvent::byte_wanted;
        }
        if ((up & I2cInterrupt::rx_full) != 0u) {
            host_reads_ = false;
            return I2cClientEvent::byte_received;
        }
        if ((up & I2cInterrupt::rx_done) != 0u) {
            S::clear_pending(I2cInterrupt::rx_done);
            return I2cClientEvent::nacked;
        }
        if ((up & I2cInterrupt::restart_det) != 0u) {
            S::clear_pending(I2cInterrupt::restart_det);
            return I2cClientEvent::restart;
        }
        if ((up & I2cInterrupt::stop_det) != 0u) {
            S::clear_pending(I2cInterrupt::stop_det);
            return I2cClientEvent::stop;
        }
        return I2cClientEvent::none;
    }

    /// The direction of the last event: true = the host reads, this
    /// side transmits.
    static bool host_reads() { return host_reads_; }

    static void release() {
        Nvic::disable(S::irq());
        S::interrupts_only(0);
        (void)S::disable();
        SclPin::release();
        SdaPin::release();
        S::hold();
    }

private:
    static inline bool host_reads_ = false;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// At 125 MHz: the spike filter is 7 cycles (56 ns); standard mode's
// period is 1250 cycles, split 500 high (HCNT 486 + 14) and 750 low
// (LCNT 749 + 1); fast mode's 313 (125 + 188); fast-mode-plus's 125
// (50 + 75). Each produces its asked rate on an ideal wire.
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::standard_100k)->spklen == 7u);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::standard_100k)->hcnt == 486u);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::standard_100k)->lcnt == 749u);
static_assert(i2c_scl_hz(125'000'000UL, *i2c_timing_for(125'000'000UL, I2cSpeed::standard_100k)) == 100'000UL);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::fast_400k)->hcnt == 111u);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::fast_400k)->lcnt == 187u);
static_assert(i2c_scl_hz(125'000'000UL, *i2c_timing_for(125'000'000UL, I2cSpeed::fast_400k)) == 399'361UL);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::fast_plus_1m)->hcnt == 36u);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::fast_plus_1m)->lcnt == 74u);
static_assert(i2c_scl_hz(125'000'000UL, *i2c_timing_for(125'000'000UL, I2cSpeed::fast_plus_1m)) == 1'000'000UL);
// The hold: 300 ns + 1 = 39 cycles, 120 ns + 1 = 16 in fast-mode-plus.
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::standard_100k)->sda_tx_hold == 39u);
static_assert(i2c_timing_for(125'000'000UL, I2cSpeed::fast_plus_1m)->sda_tx_hold == 16u);
// Table 450's floors reproduced: at 12 MHz fast mode is the block's
// own minimum (low 16, high 14 cycles); at 2.7 MHz standard mode is
// (13, 14); a hertz below either is refused, and fast-mode-plus needs
// 32 MHz.
static_assert(i2c_timing_for(12'000'000UL, I2cSpeed::fast_400k)->lcnt == 15u);
static_assert(i2c_timing_for(12'000'000UL, I2cSpeed::fast_400k)->hcnt == 6u);
static_assert(i2c_timing_for(2'700'000UL, I2cSpeed::standard_100k)->lcnt == 12u);
static_assert(i2c_timing_for(2'700'000UL, I2cSpeed::standard_100k)->hcnt == 6u);
static_assert(!i2c_timing_for(11'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(31'000'000UL, I2cSpeed::fast_plus_1m).has_value());
static_assert(i2c_timing_for(32'000'000UL, I2cSpeed::fast_plus_1m).has_value());
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_plus_1m).has_value());

static_assert(i2c_pins_valid(0, {.scl = 13, .sda = 12}) && i2c_pins_valid(1, {.scl = 15, .sda = 14}));
static_assert(i2c_sda_pin(0, 0) && i2c_sda_pin(0, 28) && i2c_scl_pin(0, 29) && i2c_sda_pin(1, 26) && i2c_scl_pin(1, 27));
static_assert(!i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_pins_valid(0, {.scl = 12, .sda = 13}));
static_assert(i2c_con_of({}) == (I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
                                 I2C_IC_CON_IC_RESTART_EN_BITS | I2C_IC_CON_STOP_DET_IFADDRESSED_BITS |
                                 I2C_IC_CON_TX_EMPTY_CTRL_BITS | I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_BITS |
                                 (I2C_IC_CON_SPEED_VALUE_STANDARD << I2C_IC_CON_SPEED_LSB)));
static_assert(i2c_read_entry({.restart = true, .stop = true}) == 0x0700u && i2c_write_entry(0xA5, {.stop = true}) == 0x02A5u);
static_assert(i2c_status_of_abort(I2cAbort::addr_noack) == i2c_nack_addr &&
              i2c_status_of_abort(I2cAbort::data_noack) == i2c_nack_data &&
              i2c_status_of_abort(I2cAbort::arb_lost | I2cAbort::data_noack) == i2c_arb_lost &&
              i2c_status_of_abort(I2cAbort::user_abort) == i2c_bus_error);

} // namespace brio
