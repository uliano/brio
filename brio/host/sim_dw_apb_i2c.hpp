/*
 * sim_dw_apb_i2c.hpp
 *
 * A DW_apb_i2c MADE OF RAM, and the chip traits over it: what
 * brio/dw_apb_i2c/i2c.hpp asks of a family (`DwApbI2cChip`), answered by
 * no family at all.
 *
 * WHY IT EXISTS. The IP stratum's claim is that the driver knows no
 * chip. A claim of that shape is proved by a SECOND realization, and the
 * cheapest second realization is one with no silicon under it: a
 * register block that is an ordinary array, a reset that fills it with
 * the block's reset values, an interrupt controller that counts, pads
 * that remember what they were handed to, and a microsecond ruler that
 * counts instead of waiting. The same header is compiled by the host
 * suite (test_dw_apb_i2c) and by a family's compile check
 * (test/family_rp2040/dw_apb_i2c_ip.cpp), so the proof is made twice,
 * once per compiler.
 *
 * WHAT IT MODELS, and what it does not. The register map is the block's,
 * at the block's offsets, with the RESET VALUES a data sheet gives them
 * (IC_CON comes up 0x65, IC_STATUS with both transmit-FIFO bits set and
 * the receive FIFO empty, which is what lets a tenure's flush_rx()
 * terminate). Three things behave rather than merely store: IC_ENABLE's
 * enable bit is mirrored into IC_ENABLE_STATUS, so a driver's bounded
 * disable() completes - and `stuck_enabled` stages the block that will
 * NOT disable, which is the one refusal a bench cannot produce; THE
 * PADS AND THE WIRE ARE MODELLED, an undriven line reading high through
 * its pull-up and SDA held low for as many hand-driven SCL pulses as a
 * test asks, which is what makes the unstick judgeable with no wire; and
 * the ruler counts the microseconds a caller spends.
 *
 * The FIFOs are NOT modelled: nothing moves an entry out of
 * IC_DATA_CMD and nothing ever fills the receive side, so the transmit
 * path runs (IC_STATUS says the transmit FIFO is never full and the
 * register holds the LAST entry the pump wrote) and the receive path has
 * nothing to read. A test that wants a received byte wants silicon: a
 * receive FIFO that never empties would hang the drain that every tenure
 * begins with, which is itself a fact about the driver worth knowing.
 *
 * WHAT IT RECORDS, beside the registers: the order of the observable
 * acts of a bring-up - which pad was claimed when, and when the block's
 * enable went up - so a test can assert the ORDER the IP file's init()
 * promises (both pads with the block still disabled), which is a thing
 * no bench can see.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "dw_apb_i2c/i2c.hpp"

namespace brio {

/// The DW_apb_i2c's register block as plain memory, at the offsets every
/// vendor's device description gives them.
struct SimDwApbI2cRegs {
    volatile uint32_t IC_CON;                ///< 0x00
    volatile uint32_t IC_TAR;                ///< 0x04
    volatile uint32_t IC_SAR;                ///< 0x08
    volatile uint32_t reserved_0;            ///< 0x0c
    volatile uint32_t IC_DATA_CMD;           ///< 0x10
    volatile uint32_t IC_SS_SCL_HCNT;        ///< 0x14
    volatile uint32_t IC_SS_SCL_LCNT;        ///< 0x18
    volatile uint32_t IC_FS_SCL_HCNT;        ///< 0x1c
    volatile uint32_t IC_FS_SCL_LCNT;        ///< 0x20
    volatile uint32_t reserved_1[2];         ///< 0x24
    volatile uint32_t IC_INTR_STAT;          ///< 0x2c
    volatile uint32_t IC_INTR_MASK;          ///< 0x30
    volatile uint32_t IC_RAW_INTR_STAT;      ///< 0x34
    volatile uint32_t IC_RX_TL;              ///< 0x38
    volatile uint32_t IC_TX_TL;              ///< 0x3c
    volatile uint32_t IC_CLR_INTR;           ///< 0x40
    volatile uint32_t IC_CLR_RX_UNDER;       ///< 0x44
    volatile uint32_t IC_CLR_RX_OVER;        ///< 0x48
    volatile uint32_t IC_CLR_TX_OVER;        ///< 0x4c
    volatile uint32_t IC_CLR_RD_REQ;         ///< 0x50
    volatile uint32_t IC_CLR_TX_ABRT;        ///< 0x54
    volatile uint32_t IC_CLR_RX_DONE;        ///< 0x58
    volatile uint32_t IC_CLR_ACTIVITY;       ///< 0x5c
    volatile uint32_t IC_CLR_STOP_DET;       ///< 0x60
    volatile uint32_t IC_CLR_START_DET;      ///< 0x64
    volatile uint32_t IC_CLR_GEN_CALL;       ///< 0x68
    volatile uint32_t IC_ENABLE;             ///< 0x6c
    volatile uint32_t IC_STATUS;             ///< 0x70
    volatile uint32_t IC_TXFLR;              ///< 0x74
    volatile uint32_t IC_RXFLR;              ///< 0x78
    volatile uint32_t IC_SDA_HOLD;           ///< 0x7c
    volatile uint32_t IC_TX_ABRT_SOURCE;     ///< 0x80
    volatile uint32_t IC_SLV_DATA_NACK_ONLY; ///< 0x84
    volatile uint32_t IC_DMA_CR;             ///< 0x88
    volatile uint32_t IC_DMA_TDLR;           ///< 0x8c
    volatile uint32_t IC_DMA_RDLR;           ///< 0x90
    volatile uint32_t IC_SDA_SETUP;          ///< 0x94
    volatile uint32_t IC_ACK_GENERAL_CALL;   ///< 0x98
    volatile uint32_t IC_ENABLE_STATUS;      ///< 0x9c
    volatile uint32_t IC_FS_SPKLEN;          ///< 0xa0
    volatile uint32_t reserved_2;            ///< 0xa4
    volatile uint32_t IC_CLR_RESTART_DET;    ///< 0xa8
};

static_assert(offsetof(SimDwApbI2cRegs, IC_DATA_CMD) == 0x10);
static_assert(offsetof(SimDwApbI2cRegs, IC_ENABLE) == 0x6c);
static_assert(offsetof(SimDwApbI2cRegs, IC_FS_SPKLEN) == 0xa0);
static_assert(offsetof(SimDwApbI2cRegs, IC_CLR_RESTART_DET) == 0xa8);

/// The interrupt line of one instance - a number, since there is no
/// controller here to give it a meaning.
enum class SimDwApbI2cLine : uint8_t { i2c0 = 0, i2c1 = 1 };

/// The interrupt controller: two verbs and their counters.
struct SimDwApbI2cInterrupts {
    SimDwApbI2cInterrupts() = delete;

    static inline bool line_enabled[2]{};
    static inline uint32_t enables[2]{};
    static inline uint32_t disables[2]{};

    static void enable(SimDwApbI2cLine line) {
        line_enabled[slot(line)] = true;
        enables[slot(line)] = enables[slot(line)] + 1u;
    }
    static void disable(SimDwApbI2cLine line) {
        line_enabled[slot(line)] = false;
        disables[slot(line)] = disables[slot(line)] + 1u;
    }

    static void reset() {
        for (uint8_t i = 0; i < 2; ++i) {
            line_enabled[i] = false;
            enables[i] = 0;
            disables[i] = 0;
        }
    }

    static constexpr uint8_t slot(SimDwApbI2cLine line) { return static_cast<uint8_t>(line); }
};

/// A pad's pulls, as much of one as the IP file ever asks a family to
/// distinguish.
enum class SimDwApbI2cPull : uint8_t { none, up };

/// A pad's electrical setup: what an I2C line wants, in this imaginary
/// chip's own two bits.
struct SimDwApbI2cPadConfig {
    bool pull_up = false;
    bool schmitt = false;
};

/**
 * THE PADS AND THE WIRE. Each pad remembers what it was handed to and
 * whether this side is pulling it down; the LEVEL of a line is the wire's
 * answer - low while somebody drives it, high through the pull-up
 * otherwise - with one staged fault: SDA is held low by an imaginary
 * client until `release_after` hand-driven SCL pulses have gone by
 * (0 = a healthy wire, 0xFF = one nothing will free).
 */
struct SimDwApbI2cBench {
    SimDwApbI2cBench() = delete;

    static constexpr uint8_t pad_count = 8;

    static inline uint8_t pad_function[pad_count]{};   ///< 0 = nobody
    static inline bool pad_pulled_up[pad_count]{};
    static inline bool pad_software[pad_count]{};      ///< handed to the software path
    static inline bool pad_out_low[pad_count]{};
    static inline bool pad_driven[pad_count]{};
    static inline uint32_t pad_claimed_at[pad_count]{};
    static inline uint32_t pad_released_at[pad_count]{};

    static inline uint8_t scl_pad = 0xFFu;        ///< which pad the wire counts pulses on
    static inline uint8_t sda_pad = 0xFFu;
    static inline uint8_t release_after = 0;      ///< SCL pulses a stuck client needs
    static inline uint8_t pulses = 0;             ///< SCL low periods driven by hand

    static inline uint32_t stamp = 0;             ///< ticks on every recorded act
    static inline uint32_t enabled_at = 0;        ///< when IC_ENABLE's enable last went up
    static inline uint32_t resets = 0;
    static inline bool held[2]{};

    static inline uint32_t spins = 0;             ///< how often the ruler was asked
    static inline uint32_t micros = 0;            ///< and for how long in all

    static uint32_t tick() { return ++stamp; }

    /// The wire's answer at one pad: low while this side drives it, low
    /// while the staged client holds SDA, high otherwise.
    static bool level(uint8_t pin) {
        if (pad_driven[pin]) {
            return false;
        }
        if (pin == sda_pad && pulses < release_after) {
            return false;
        }
        return true;
    }

    static void reset() {
        for (uint8_t i = 0; i < pad_count; ++i) {
            pad_function[i] = 0;
            pad_pulled_up[i] = false;
            pad_software[i] = false;
            pad_out_low[i] = false;
            pad_driven[i] = false;
            pad_claimed_at[i] = 0;
            pad_released_at[i] = 0;
        }
        scl_pad = 0xFFu;
        sda_pad = 0xFFu;
        release_after = 0;
        pulses = 0;
        stamp = 0;
        enabled_at = 0;
        resets = 0;
        held[0] = held[1] = false;
        spins = 0;
        micros = 0;
    }
};

/// One pad as a type, with the seven verbs the IP file calls on one.
template <uint8_t pin>
struct SimDwApbI2cPad {
    static_assert(pin < SimDwApbI2cBench::pad_count, "this imaginary chip has eight pads");

    static void function(uint8_t code, const SimDwApbI2cPadConfig& cfg) {
        SimDwApbI2cBench::pad_function[pin] = code;
        SimDwApbI2cBench::pad_pulled_up[pin] = cfg.pull_up;
        SimDwApbI2cBench::pad_software[pin] = false;
        SimDwApbI2cBench::pad_driven[pin] = false;
        SimDwApbI2cBench::pad_claimed_at[pin] = SimDwApbI2cBench::tick();
    }
    static void release() {
        SimDwApbI2cBench::pad_function[pin] = 0;
        SimDwApbI2cBench::pad_pulled_up[pin] = false;
        SimDwApbI2cBench::pad_software[pin] = false;
        SimDwApbI2cBench::pad_driven[pin] = false;
        SimDwApbI2cBench::pad_released_at[pin] = SimDwApbI2cBench::tick();
    }
    /// The software path takes the pad back, as an input with its pull.
    static void input(SimDwApbI2cPull pull) {
        SimDwApbI2cBench::pad_function[pin] = 0;
        SimDwApbI2cBench::pad_software[pin] = true;
        SimDwApbI2cBench::pad_pulled_up[pin] = pull == SimDwApbI2cPull::up;
        SimDwApbI2cBench::pad_driven[pin] = false;
    }
    /// The output register low - what the drive shows when it is on.
    static void clear() { SimDwApbI2cBench::pad_out_low[pin] = true; }
    static bool read() { return SimDwApbI2cBench::level(pin); }
    static void drive_low() {
        SimDwApbI2cBench::pad_driven[pin] = true;
        if (pin == SimDwApbI2cBench::scl_pad) {
            SimDwApbI2cBench::pulses = static_cast<uint8_t>(SimDwApbI2cBench::pulses + 1u);
        }
    }
    static void release_drive() { SimDwApbI2cBench::pad_driven[pin] = false; }
};

/// The pin set: a pad per line and nothing else to check.
struct SimDwApbI2cPins {
    uint8_t scl;
    uint8_t sda;
};

/// A DMA request, as a number that names nothing.
enum class SimDwApbI2cRequest : uint8_t { i2c0_tx = 0, i2c0_rx = 1, i2c1_tx = 2, i2c1_rx = 3 };

/// The microsecond ruler: a rate, and a wait that counts instead of
/// waiting.
struct SimDwApbI2cSpin {
    uint32_t cycles_per_us = 0;
};

/**
 * THE CHIP THAT IS NOT A CHIP: every member `DwApbI2cChip` asks for, over
 * two register blocks in RAM.
 */
struct SimDwApbI2c {
    SimDwApbI2c() = delete;

    using Regs = SimDwApbI2cRegs;
    using Irq = SimDwApbI2cLine;
    using Interrupts = SimDwApbI2cInterrupts;
    using Pins = SimDwApbI2cPins;
    using DmaRequest = SimDwApbI2cRequest;
    using SpinRate = SimDwApbI2cSpin;

    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 16;

    /// The two blocks, and the reset values the block comes up with.
    static inline Regs block[instances]{};
    /// A staged block that will not disable: IC_ENABLE_STATUS keeps
    /// saying the hardware is running, which is the refusal start()
    /// answers i2c_bus_error.
    static inline bool stuck_enabled = false;

    template <uint8_t i>
    static Regs& regs() { return block[i]; }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? SimDwApbI2cLine::i2c0 : SimDwApbI2cLine::i2c1; }

    template <uint8_t i>
    static bool reset() {
        fill(block[i]);
        SimDwApbI2cBench::held[i] = false;
        SimDwApbI2cBench::resets = SimDwApbI2cBench::resets + 1u;
        return true;
    }
    template <uint8_t i>
    static bool released() { return !SimDwApbI2cBench::held[i]; }
    template <uint8_t i>
    static void hold() { SimDwApbI2cBench::held[i] = true; }

    template <uint8_t i>
    static constexpr DmaRequest tx_request() {
        return i == 0 ? SimDwApbI2cRequest::i2c0_tx : SimDwApbI2cRequest::i2c1_tx;
    }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() {
        return i == 0 ? SimDwApbI2cRequest::i2c0_rx : SimDwApbI2cRequest::i2c1_rx;
    }

    /// No atomic aliases here: a read-modify-write is what a chip without
    /// them would do. IC_ENABLE is the one register that answers back -
    /// the hardware's own enable status follows it, and the stamp records
    /// the act a test wants to place in time.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) {
        reg = reg | bits;
        follow_enable(reg, bits);
    }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) {
        reg = reg & ~bits;
        follow_enable(reg, bits);
    }

    /// Any pad may carry either line, as long as they are two.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) {
        return n < instances && p.scl < SimDwApbI2cBench::pad_count &&
               p.sda < SimDwApbI2cBench::pad_count && p.scl != p.sda;
    }
    static constexpr uint8_t scl_pad(const Pins& p) { return p.scl; }
    static constexpr uint8_t sda_pad(const Pins& p) { return p.sda; }

    template <uint8_t pin>
    using Pad = SimDwApbI2cPad<pin>;
    static constexpr uint8_t pad_function = 3;
    static constexpr SimDwApbI2cPadConfig pad_config{.pull_up = true, .schmitt = true};
    static constexpr SimDwApbI2cPull open_drain_pull = SimDwApbI2cPull::up;

    /// The channel is the identity here too: two present engines of one
    /// host must name two of them.
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() {
        if constexpr (Tx::present && Rx::present) {
            return Tx::channel != Rx::channel;
        } else {
            return true;
        }
    }

    /// The ruler counts instead of waiting.
    static constexpr SpinRate spin_rate(uint32_t hz) {
        return {static_cast<uint32_t>((hz + 999'999UL) / 1'000'000UL)};
    }
    static void spin_us(SpinRate rate, uint32_t us) {
        (void)rate;
        SimDwApbI2cBench::spins = SimDwApbI2cBench::spins + 1u;
        SimDwApbI2cBench::micros = SimDwApbI2cBench::micros + us;
    }

    /// Any clock type whose `hz` this imaginary tree would hand the block
    /// undivided.
    template <typename Clock>
    static constexpr uint32_t ic_clk_hz(Clock clock) { return clock_hz(clock); }

    /// Back to power-on, registers, wire and bench alike.
    static void reset_all() {
        for (uint8_t i = 0; i < instances; ++i) {
            fill(block[i]);
        }
        stuck_enabled = false;
        SimDwApbI2cBench::reset();
        SimDwApbI2cInterrupts::reset();
    }

    /// The reset values of the registers this file models.
    static void fill(Regs& r) {
        r = Regs{};
        r.IC_CON = 0x65u;
        r.IC_TAR = 0x55u;
        r.IC_SAR = 0x55u;
        r.IC_SS_SCL_HCNT = 0x28u;
        r.IC_SS_SCL_LCNT = 0x2Fu;
        r.IC_FS_SCL_HCNT = 0x06u;
        r.IC_FS_SCL_LCNT = 0x0Du;
        r.IC_INTR_MASK = 0x8FFu;
        r.IC_STATUS = I2cFlag::tx_not_full | I2cFlag::tx_empty;
        r.IC_SDA_HOLD = 0x01u;
        r.IC_SDA_SETUP = 0x64u;
        r.IC_ACK_GENERAL_CALL = 0x01u;
        r.IC_FS_SPKLEN = 0x07u;
    }

    static bool is_enable(const volatile uint32_t& reg) {
        return &reg == &block[0].IC_ENABLE || &reg == &block[1].IC_ENABLE;
    }

private:
    /// IC_ENABLE_STATUS.IC_EN is the hardware's own answer: it follows
    /// the enable bit, and a staged stuck block never lets it fall.
    static void follow_enable(volatile uint32_t& reg, uint32_t bits) {
        if (!is_enable(reg) || (bits & I2cEnable::enable) == 0u) {
            return;
        }
        const bool on = (reg & I2cEnable::enable) != 0u;
        volatile uint32_t& status = (&reg == &block[0].IC_ENABLE) ? block[0].IC_ENABLE_STATUS
                                                                 : block[1].IC_ENABLE_STATUS;
        if (on) {
            status = status | I2cEnableStatus::running;
            SimDwApbI2cBench::enabled_at = SimDwApbI2cBench::tick();
        } else if (!stuck_enabled) {
            status = status & ~I2cEnableStatus::running;
        }
    }
};

static_assert(DwApbI2cChip<SimDwApbI2c>);

/// A static clock for the tests over SimDwApbI2c: the rate a family's own
/// Clock type would carry.
template <uint32_t rate>
struct SimDwApbI2cClock {
    static constexpr bool is_static = true;
    static constexpr uint32_t hz = rate;
};

/// The empty engine slot, this imaginary chip's spelling.
struct SimI2cNoEngine {
    SimI2cNoEngine() = delete;
    static constexpr bool present = false;
};

/**
 * A DMA engine of no controller: the slot's whole vocabulary over a few
 * counters, so that a host's engine BRANCHES - the ones a family with no
 * DMA driver never compiles - are exercised here too.
 */
template <uint8_t ch, typename Elem>
struct SimDwApbI2cEngine {
    SimDwApbI2cEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_error = 1u << 1;
    using element = Elem;

    static inline uint8_t next_flags = 0;   ///< what the next service() reports
    static inline uint32_t armed = 0;
    static inline uint32_t blocks = 0;
    static inline uint32_t faults = 0;
    static inline uint32_t length = 0;
    static inline bool fixed = false;       ///< the last block poured one cell
    static inline bool discarded = false;   ///< the last block was thrown away
    static inline bool running = false;

    static uint8_t service() {
        const uint8_t f = next_flags;
        next_flags = 0;
        return f;
    }
    static void arm(volatile void* data, SimDwApbI2cRequest request) {
        (void)data;
        (void)request;
        armed = armed + 1u;
    }
    static bool start(Elem* buffer, uint32_t len) {
        (void)buffer;
        fixed = false;
        discarded = false;
        return begin(len);
    }
    static bool start_fixed(const Elem* cell, uint32_t len) {
        (void)cell;
        fixed = true;
        discarded = false;
        return begin(len);
    }
    static bool start_discard(Elem* cell, uint32_t len) {
        (void)cell;
        fixed = false;
        discarded = true;
        return begin(len);
    }
    static uint32_t complete() {
        running = false;
        return length;
    }
    static bool busy() { return running; }
    static bool abandon() {
        running = false;
        faults = faults + 1u;
        return true;
    }
    static void stop() { running = false; }

    static void reset() {
        next_flags = 0;
        armed = 0;
        blocks = 0;
        faults = 0;
        length = 0;
        fixed = false;
        discarded = false;
        running = false;
    }

private:
    static bool begin(uint32_t len) {
        length = len;
        running = true;
        blocks = blocks + 1u;
        return true;
    }
};

} // namespace brio
