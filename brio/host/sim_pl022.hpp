/*
 * sim_pl022.hpp
 *
 * A PL022 MADE OF RAM, and the chip traits over it: what
 * brio/pl022/spi.hpp asks of a family (`Pl022Chip`), answered by no
 * family at all.
 *
 * WHY IT EXISTS. The IP stratum's claim is that the driver knows no
 * chip. A claim of that shape is proved by a SECOND realization, and the
 * cheapest second realization is one with no silicon under it: a
 * register block that is an ordinary array, a reset that memsets it to
 * the block's reset values, an interrupt controller that counts, pads
 * that remember what they were handed to, a run-time pin that remembers
 * which way it was driven and when, and a busy-wait that counts
 * microseconds instead of spending them. The same header is compiled by
 * the host suite (test_pl022) and by a family's compile check
 * (test/family_rp2040/pl022_ip.cpp), so the proof is made twice, once
 * per compiler.
 *
 * WHAT IT MODELS, and what it does not. The register map is the PL022's,
 * at the PL022's offsets, and the RESET VALUES are the block's (SSPSR
 * comes up with the transmit FIFO empty and not full, which is what lets
 * a bring-up's flush of the receive FIFO stop and a pump write its first
 * batch). The FIFOs themselves are NOT modelled - a register block that
 * is plain memory cannot: a store to SSPDR is a store and nothing
 * watches it - so nothing ever comes BACK, and the receive half of a
 * transfer stays the bench's to judge. What is judged here is what a
 * bench cannot see cheaply: the exact words a configuration leaves in
 * the two control registers, the frames a pump wrote, and the ORDER of
 * the acts of a bring-up.
 *
 * WHAT IT RECORDS, beside the registers: which pad was claimed or
 * released when, when SSPCR1's enable went up, and which way a Request's
 * select line was driven and when - so a test can assert the ORDER the
 * IP file's init() and start() promise (the pads after the registers
 * hold the idle polarity, the select asserted before the first frame),
 * which is a thing no bench can see.
 */

#pragma once

#include <stdint.h>

#include "pl022/spi.hpp"

namespace brio {

/// The PL022's register block as plain memory, the names and the offsets
/// of the block every vendor's device description spells the same way.
struct SimPl022Regs {
    volatile uint32_t SSPCR0;     ///< 0x00
    volatile uint32_t SSPCR1;     ///< 0x04
    volatile uint32_t SSPDR;      ///< 0x08
    volatile uint32_t SSPSR;      ///< 0x0c
    volatile uint32_t SSPCPSR;    ///< 0x10
    volatile uint32_t SSPIMSC;    ///< 0x14
    volatile uint32_t SSPRIS;     ///< 0x18
    volatile uint32_t SSPMIS;     ///< 0x1c
    volatile uint32_t SSPICR;     ///< 0x20
    volatile uint32_t SSPDMACR;   ///< 0x24
};

/// The interrupt line of one instance - a number, since there is no
/// controller here to give it a meaning.
enum class SimPl022Line : uint8_t { spi0 = 0, spi1 = 1 };

/// The interrupt controller: two verbs and one flag per line.
struct SimPl022Interrupts {
    SimPl022Interrupts() = delete;

    static inline bool line_enabled[2]{};
    static inline uint32_t enables[2]{};
    static inline uint32_t disables[2]{};

    static void enable(SimPl022Line line) {
        line_enabled[slot(line)] = true;
        enables[slot(line)] = enables[slot(line)] + 1u;
    }
    static void disable(SimPl022Line line) {
        line_enabled[slot(line)] = false;
        disables[slot(line)] = disables[slot(line)] + 1u;
    }

    static void reset() {
        line_enabled[0] = line_enabled[1] = false;
        enables[0] = enables[1] = 0;
        disables[0] = disables[1] = 0;
    }

    static constexpr uint8_t slot(SimPl022Line line) { return static_cast<uint8_t>(line); }
};

/// A pad's electrical setup: one bit, because one bit is all the IP file
/// ever asks a family to distinguish.
struct SimPl022PadConfig {
    bool pull_up = false;
};

/// What the pads of this imaginary chip remember, and the running stamp
/// that puts the acts of a bring-up in order.
struct SimPl022Bench {
    SimPl022Bench() = delete;

    static constexpr uint8_t pad_count = 12;

    static inline uint8_t pad_function[pad_count]{};   ///< 0 = nobody
    static inline bool pad_pulled_up[pad_count]{};
    static inline uint32_t pad_claimed_at[pad_count]{};
    static inline uint32_t pad_released_at[pad_count]{};
    static inline bool pad_level[pad_count]{};         ///< what a pad READS, the test's to set

    static inline bool pin_level[pad_count]{};         ///< a run-time pin, as last driven
    static inline uint32_t pin_set_at[pad_count]{};
    static inline uint32_t pin_clear_at[pad_count]{};

    static inline uint32_t stamp = 0;        ///< ticks on every recorded act
    static inline uint32_t enabled_at = 0;   ///< when SSPCR1's enable last went up
    static inline uint32_t resets = 0;
    static inline bool held[2]{};

    static inline uint32_t waited_us = 0;       ///< what the busy-wait was asked for
    static inline uint32_t waited_cycles = 0;   ///< the factor it was asked with

    static uint32_t tick() { return ++stamp; }

    static void reset() {
        for (uint8_t i = 0; i < pad_count; ++i) {
            pad_function[i] = 0;
            pad_pulled_up[i] = false;
            pad_claimed_at[i] = 0;
            pad_released_at[i] = 0;
            pad_level[i] = false;
            pin_level[i] = false;
            pin_set_at[i] = 0;
            pin_clear_at[i] = 0;
        }
        stamp = 0;
        enabled_at = 0;
        resets = 0;
        held[0] = held[1] = false;
        waited_us = 0;
        waited_cycles = 0;
    }
};

/// One pad as a type, with the three verbs the IP file calls on one.
template <uint8_t pin>
struct SimPl022Pad {
    static_assert(pin < SimPl022Bench::pad_count, "this imaginary chip has twelve pads");

    static void function(uint8_t code, SimPl022PadConfig cfg) {
        SimPl022Bench::pad_function[pin] = code;
        SimPl022Bench::pad_pulled_up[pin] = cfg.pull_up;
        SimPl022Bench::pad_claimed_at[pin] = SimPl022Bench::tick();
    }
    static void release() {
        SimPl022Bench::pad_function[pin] = 0;
        SimPl022Bench::pad_pulled_up[pin] = false;
        SimPl022Bench::pad_released_at[pin] = SimPl022Bench::tick();
    }
    static bool read() { return SimPl022Bench::pad_level[pin]; }
};

/// A pin named at RUN TIME - a Request's chip select and its D/C line.
/// It remembers which way it was driven and when, which is the only way
/// a test can see a select window from the outside.
struct SimPl022PinRef {
    uint8_t pin = 0xFFu;

    constexpr bool valid() const { return pin < SimPl022Bench::pad_count; }
    void set() const {
        if (valid()) {
            SimPl022Bench::pin_level[pin] = true;
            SimPl022Bench::pin_set_at[pin] = SimPl022Bench::tick();
        }
    }
    void clear() const {
        if (valid()) {
            SimPl022Bench::pin_level[pin] = false;
            SimPl022Bench::pin_clear_at[pin] = SimPl022Bench::tick();
        }
    }
};

/// The pin set: the four signals, and the same sentinel a real family's
/// table uses for a signal it does not route.
struct SimPl022Pins {
    uint8_t sck = 0xFFu;
    uint8_t tx = 0xFFu;
    uint8_t rx = 0xFFu;
    uint8_t cs = 0xFFu;
};

/// A DMA request, as a number that names nothing.
enum class SimPl022Request : uint8_t { spi0_tx = 0, spi0_rx = 1, spi1_tx = 2, spi1_rx = 3 };

/// The busy-wait's precomputed factor, and the verb over it: this
/// imaginary machine COUNTS the microseconds it is asked for instead of
/// spending them, which is what makes `cs_setup_us` testable at all.
/// The driver calls it unqualified on a type of the family's, so this
/// overload is the one that is found.
struct SimPl022DelayRate {
    uint32_t cycles_per_us = 0;
};

inline bool delay_us(SimPl022DelayRate rate, uint32_t us) {
    SimPl022Bench::waited_us = SimPl022Bench::waited_us + us;
    SimPl022Bench::waited_cycles = rate.cycles_per_us;
    return rate.cycles_per_us != 0u;
}

/**
 * THE CHIP THAT IS NOT A CHIP: every member `Pl022Chip` asks for, over
 * two register blocks in RAM.
 */
struct SimPl022 {
    SimPl022() = delete;

    using Regs = SimPl022Regs;
    using Irq = SimPl022Line;
    using Interrupts = SimPl022Interrupts;
    using Pins = SimPl022Pins;
    using PinRef = SimPl022PinRef;
    using DmaRequest = SimPl022Request;
    using DelayRate = SimPl022DelayRate;

    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 8;
    static constexpr uint8_t no_pad = 0xFFu;

    /// The two blocks, and the reset value the block comes up with:
    /// everything zero but SSPSR, whose transmit-FIFO bits are set.
    static inline Regs block[instances]{};
    static constexpr uint32_t status_reset = SpiFlag::tx_empty | SpiFlag::tx_not_full;

    template <uint8_t i>
    static Regs& regs() { return block[i]; }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? SimPl022Line::spi0 : SimPl022Line::spi1; }

    template <uint8_t i>
    static bool reset() {
        block[i] = Regs{};
        block[i].SSPSR = status_reset;
        SimPl022Bench::held[i] = false;
        SimPl022Bench::resets = SimPl022Bench::resets + 1u;
        return true;
    }
    template <uint8_t i>
    static bool released() { return !SimPl022Bench::held[i]; }
    template <uint8_t i>
    static void hold() { SimPl022Bench::held[i] = true; }

    template <uint8_t i>
    static constexpr DmaRequest tx_request() {
        return i == 0 ? SimPl022Request::spi0_tx : SimPl022Request::spi1_tx;
    }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() {
        return i == 0 ? SimPl022Request::spi0_rx : SimPl022Request::spi1_rx;
    }

    /// No atomic aliases here: a read-modify-write is what a chip without
    /// them would do, and the stamp records the one act a test wants to
    /// place in time.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) {
        reg = reg | bits;
        if (is_control(reg) && (bits & SpiControl1::enable) != 0u) {
            SimPl022Bench::enabled_at = SimPl022Bench::tick();
        }
    }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { reg = reg & ~bits; }

    /// Any pad may carry any signal, as long as a link has a clock and
    /// no two signals share a pad.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) {
        if (n >= instances || p.sck >= SimPl022Bench::pad_count) {
            return false;
        }
        const uint8_t named[4] = {p.sck, p.tx, p.rx, p.cs};
        for (uint8_t i = 0; i < 4u; ++i) {
            if (named[i] == no_pad) {
                continue;
            }
            if (named[i] >= SimPl022Bench::pad_count) {
                return false;
            }
            for (uint8_t k = 0; k < i; ++k) {
                if (named[k] == named[i]) {
                    return false;
                }
            }
        }
        return true;
    }
    static constexpr uint8_t sck_pad(const Pins& p) { return p.sck; }
    static constexpr uint8_t tx_pad(const Pins& p) { return p.tx; }
    static constexpr uint8_t rx_pad(const Pins& p) { return p.rx; }
    static constexpr uint8_t cs_pad(const Pins& p) { return p.cs; }

    template <uint8_t pin>
    using Pad = SimPl022Pad<pin>;
    static constexpr uint8_t pad_function = 1;
    static constexpr SimPl022PadConfig sck_pad_config() { return {}; }
    static constexpr SimPl022PadConfig tx_pad_config() { return {}; }
    static constexpr SimPl022PadConfig host_rx_pad_config() { return {.pull_up = true}; }
    static constexpr SimPl022PadConfig client_rx_pad_config() { return {}; }
    static constexpr SimPl022PadConfig cs_pad_config() { return {.pull_up = true}; }

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

    /// One microsecond is one count on this imaginary machine.
    static constexpr DelayRate delay_rate(uint32_t hz) {
        return {static_cast<uint32_t>((hz + 999'999UL) / 1'000'000UL)};
    }

    /// Any clock type whose `hz` this imaginary tree would divide.
    template <typename Clock>
    static constexpr uint32_t sspclk_hz(Clock clock) { return clock_hz(clock); }

    /// Back to power-on, registers and bench alike.
    static void reset_all() {
        for (uint8_t i = 0; i < instances; ++i) {
            block[i] = Regs{};
            block[i].SSPSR = status_reset;
        }
        SimPl022Bench::reset();
        SimPl022Interrupts::reset();
    }

    static bool is_control(const volatile uint32_t& reg) {
        return &reg == &block[0].SSPCR1 || &reg == &block[1].SSPCR1;
    }
};

static_assert(Pl022Chip<SimPl022>);

/// A static clock for the tests over SimPl022: the rate a family's own
/// Clock type would carry.
template <uint32_t rate>
struct SimPl022Clock {
    static constexpr bool is_static = true;
    static constexpr uint32_t hz = rate;
};

/// The empty engine slot, this imaginary chip's spelling.
struct SimPl022NoEngine {
    SimPl022NoEngine() = delete;
    static constexpr bool present = false;
};

/**
 * A DMA engine of no controller: the slot's whole vocabulary over a few
 * counters, so that a host's engine BRANCHES - the ones a family with no
 * DMA driver never compiles - are exercised here too.
 */
template <uint8_t ch>
struct SimPl022Engine {
    SimPl022Engine() = delete;

    using element = uint8_t;
    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_error = 1u << 1;

    static inline uint8_t next_flags = 0;   ///< what the next service() reports
    static inline uint32_t armed = 0;
    static inline uint32_t blocks = 0;
    static inline uint32_t faults = 0;
    static inline uint32_t stops = 0;
    static inline uint32_t length = 0;
    static inline bool fixed = false;       ///< the last block was a fixed cell
    static inline bool discarded = false;   ///< the last block went to a sink
    static inline bool running = false;

    static uint8_t service() {
        const uint8_t f = next_flags;
        next_flags = 0;
        return f;
    }
    static void arm(volatile void* data, SimPl022Request request) {
        (void)data;
        (void)request;
        armed = armed + 1u;
    }
    static bool start(const uint8_t* buffer, uint32_t len) {
        (void)buffer;
        length = len;
        fixed = false;
        discarded = false;
        running = true;
        blocks = blocks + 1u;
        return true;
    }
    static bool start(uint8_t* buffer, uint32_t len) {
        return start(static_cast<const uint8_t*>(buffer), len);
    }
    static bool start_fixed(const uint8_t* cell, uint32_t len) {
        const bool ok = start(cell, len);
        fixed = true;
        return ok;
    }
    static bool start_discard(uint8_t* sink, uint32_t len) {
        const bool ok = start(static_cast<const uint8_t*>(sink), len);
        discarded = true;
        return ok;
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
    static void stop() {
        running = false;
        stops = stops + 1u;
    }

    static void reset() {
        next_flags = 0;
        armed = 0;
        blocks = 0;
        faults = 0;
        stops = 0;
        length = 0;
        fixed = false;
        discarded = false;
        running = false;
    }
};

} // namespace brio
