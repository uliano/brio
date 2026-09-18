/*
 * sim_pl011.hpp
 *
 * A PL011 MADE OF RAM, and the chip traits over it: what
 * brio/pl011/uart.hpp asks of a family (`Pl011Chip`), answered by no
 * family at all.
 *
 * WHY IT EXISTS. The IP stratum's claim is that the driver knows no
 * chip. A claim of that shape is proved by a SECOND realization, and
 * the cheapest second realization is one with no silicon under it: a
 * register block that is an ordinary array, a reset that memsets it to
 * the block's reset values, an interrupt controller that counts, pads
 * that remember what they were handed to. The same header is compiled
 * by the host suite (test_pl011) and by a family's compile check
 * (test/family_rp2040/pl011_ip.cpp), so the proof is made twice, once
 * per compiler.
 *
 * WHAT IT MODELS, and what it does not. The register map is the PL011's,
 * at the PL011's offsets, and the RESET VALUES are the block's (UARTFR
 * comes up with both FIFOs empty, which is what lets a transport's
 * init() drain the receive FIFO and stop). The FIFOs themselves are NOT
 * modelled: nothing moves a byte out of UARTDR and nothing ever fills
 * the receive side, so the transmit path runs (the flag register says
 * the transmit FIFO is never full) and the receive path has nothing to
 * read. A test that wants a received byte wants silicon.
 *
 * WHAT IT RECORDS, beside the registers: the order of the observable
 * acts of a bring-up - which pad was claimed when, and when UARTCR's
 * enable went up - so a test can assert the ORDER the IP file's init()
 * promises (the receive pad before the enable, the transmit pad after
 * it), which is a thing no bench can see.
 */

#pragma once

#include <stdint.h>

#include "host/platform.hpp"
#include "pl011/uart.hpp"

namespace brio {

/// The PL011's register block as plain memory, the names and the offsets
/// of the block every vendor's device description spells the same way.
struct SimPl011Regs {
    volatile uint32_t UARTDR;        ///< 0x00
    volatile uint32_t UARTRSR;       ///< 0x04
    volatile uint32_t reserved_0[4]; ///< 0x08
    volatile uint32_t UARTFR;        ///< 0x18
    volatile uint32_t reserved_1;    ///< 0x1c
    volatile uint32_t UARTILPR;      ///< 0x20
    volatile uint32_t UARTIBRD;      ///< 0x24
    volatile uint32_t UARTFBRD;      ///< 0x28
    volatile uint32_t UARTLCR_H;     ///< 0x2c
    volatile uint32_t UARTCR;        ///< 0x30
    volatile uint32_t UARTIFLS;      ///< 0x34
    volatile uint32_t UARTIMSC;      ///< 0x38
    volatile uint32_t UARTRIS;       ///< 0x3c
    volatile uint32_t UARTMIS;       ///< 0x40
    volatile uint32_t UARTICR;       ///< 0x44
    volatile uint32_t UARTDMACR;     ///< 0x48
};

/// The interrupt line of one instance - a number, since there is no
/// controller here to give it a meaning.
enum class SimPl011Line : uint8_t { uart0 = 0, uart1 = 1 };

/// The interrupt controller: three verbs and three counters.
struct SimPl011Interrupts {
    SimPl011Interrupts() = delete;

    static inline bool line_enabled[2]{};
    static inline uint32_t pends[2]{};

    static void enable(SimPl011Line line) { line_enabled[slot(line)] = true; }
    static void disable(SimPl011Line line) { line_enabled[slot(line)] = false; }
    static void set_pending(SimPl011Line line) { pends[slot(line)] = pends[slot(line)] + 1u; }

    static void reset() {
        line_enabled[0] = line_enabled[1] = false;
        pends[0] = pends[1] = 0;
    }

    static constexpr uint8_t slot(SimPl011Line line) { return static_cast<uint8_t>(line); }
};

/// A pad's electrical setup: one bit, because one bit is all the IP file
/// ever asks a family to distinguish.
struct SimPl011PadConfig {
    bool pull_up = false;
};

/// What the pads of this imaginary chip remember, and the running stamp
/// that puts the acts of a bring-up in order.
struct SimPl011Bench {
    SimPl011Bench() = delete;

    static constexpr uint8_t pad_count = 8;

    static inline uint8_t pad_function[pad_count]{};   ///< 0 = nobody
    static inline bool pad_pulled_up[pad_count]{};
    static inline uint32_t pad_claimed_at[pad_count]{};
    static inline uint32_t pad_released_at[pad_count]{};

    static inline uint32_t stamp = 0;        ///< ticks on every recorded act
    static inline uint32_t enabled_at = 0;   ///< when UARTCR's enable last went up
    static inline uint32_t resets = 0;
    static inline bool held[2]{};

    static uint32_t tick() { return ++stamp; }

    static void reset() {
        for (uint8_t i = 0; i < pad_count; ++i) {
            pad_function[i] = 0;
            pad_pulled_up[i] = false;
            pad_claimed_at[i] = 0;
            pad_released_at[i] = 0;
        }
        stamp = 0;
        enabled_at = 0;
        resets = 0;
        held[0] = held[1] = false;
    }
};

/// One pad as a type, with the two verbs the IP file calls on one.
template <uint8_t pin>
struct SimPl011Pad {
    static_assert(pin < SimPl011Bench::pad_count, "this imaginary chip has eight pads");

    static void function(uint8_t code, SimPl011PadConfig cfg) {
        SimPl011Bench::pad_function[pin] = code;
        SimPl011Bench::pad_pulled_up[pin] = cfg.pull_up;
        SimPl011Bench::pad_claimed_at[pin] = SimPl011Bench::tick();
    }
    static void release() {
        SimPl011Bench::pad_function[pin] = 0;
        SimPl011Bench::pad_pulled_up[pin] = false;
        SimPl011Bench::pad_released_at[pin] = SimPl011Bench::tick();
    }
};

/// The pin set: a pad per direction and nothing else to check.
struct SimPl011Pins {
    uint8_t tx;
    uint8_t rx;
};

/// A DMA request, as a number that names nothing.
enum class SimPl011Request : uint8_t { uart0_tx = 0, uart0_rx = 1, uart1_tx = 2, uart1_rx = 3 };

/**
 * THE CHIP THAT IS NOT A CHIP: every member `Pl011Chip` asks for, over
 * two register blocks in RAM.
 */
struct SimPl011 {
    SimPl011() = delete;

    using Regs = SimPl011Regs;
    using Irq = SimPl011Line;
    using Interrupts = SimPl011Interrupts;
    using Guard = HostPlatform::CriticalSection;
    using Platform = HostPlatform;
    using Pins = SimPl011Pins;
    using DmaRequest = SimPl011Request;

    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 32;

    /// The two blocks, and the reset values the block comes up with:
    /// everything zero but UARTFR, whose two FIFO-empty bits are set.
    static inline Regs block[instances]{};
    static constexpr uint32_t flag_reset = UartFlag::rx_empty | UartFlag::tx_empty;

    template <uint8_t i>
    static Regs& regs() { return block[i]; }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? SimPl011Line::uart0 : SimPl011Line::uart1; }

    template <uint8_t i>
    static bool reset() {
        block[i] = Regs{};
        block[i].UARTFR = flag_reset;
        SimPl011Bench::held[i] = false;
        SimPl011Bench::resets = SimPl011Bench::resets + 1u;
        return true;
    }
    template <uint8_t i>
    static bool released() { return !SimPl011Bench::held[i]; }
    template <uint8_t i>
    static void hold() { SimPl011Bench::held[i] = true; }

    template <uint8_t i>
    static constexpr DmaRequest tx_request() {
        return i == 0 ? SimPl011Request::uart0_tx : SimPl011Request::uart1_tx;
    }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() {
        return i == 0 ? SimPl011Request::uart0_rx : SimPl011Request::uart1_rx;
    }

    /// No atomic aliases here: a read-modify-write under the guard is
    /// what a chip without them would do, and the stamp records the one
    /// act a test wants to place in time.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) {
        reg = reg | bits;
        if (is_control(reg) && (bits & UartControl::enable) != 0u) {
            SimPl011Bench::enabled_at = SimPl011Bench::tick();
        }
    }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { reg = reg & ~bits; }

    /// Any pad may carry either signal, as long as they are two.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) {
        return n < instances && p.tx < SimPl011Bench::pad_count &&
               p.rx < SimPl011Bench::pad_count && p.tx != p.rx;
    }
    static constexpr uint8_t tx_pad(const Pins& p) { return p.tx; }
    static constexpr uint8_t rx_pad(const Pins& p) { return p.rx; }

    template <uint8_t pin>
    using Pad = SimPl011Pad<pin>;
    static constexpr uint8_t pad_function = 2;
    static constexpr SimPl011PadConfig tx_pad_config() { return {}; }
    static constexpr SimPl011PadConfig rx_pad_config() { return {.pull_up = true}; }

    /// The channel is the identity here too: two present engines of one
    /// transport must name two of them.
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() {
        if constexpr (Tx::present && Rx::present) {
            return Tx::channel != Rx::channel;
        } else {
            return true;
        }
    }

    /// Any clock type whose `hz` this imaginary tree would divide.
    template <typename Clock>
    static constexpr uint32_t uartclk_hz(Clock clock) { return clock_hz(clock); }

    /// Back to power-on, registers and bench alike.
    static void reset_all() {
        for (uint8_t i = 0; i < instances; ++i) {
            block[i] = Regs{};
            block[i].UARTFR = flag_reset;
        }
        SimPl011Bench::reset();
        SimPl011Interrupts::reset();
    }

    static bool is_control(const volatile uint32_t& reg) {
        return &reg == &block[0].UARTCR || &reg == &block[1].UARTCR;
    }
};

static_assert(Pl011Chip<SimPl011>);

/// A static clock for the tests over SimPl011: the rate a family's own
/// Clock type would carry.
template <uint32_t rate>
struct SimPl011Clock {
    static constexpr bool is_static = true;
    static constexpr uint32_t hz = rate;
};

/// The empty engine slot, this imaginary chip's spelling.
struct SimNoEngine {
    SimNoEngine() = delete;
    static constexpr bool present = false;
};

/**
 * A DMA engine of no controller: the slot's whole vocabulary over two
 * counters, so that a transport's engine BRANCHES - the ones a family
 * with no DMA driver never compiles - are exercised here too.
 */
template <uint8_t ch>
struct SimPl011Engine {
    SimPl011Engine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr uint8_t flag_complete = 1u << 0;
    static constexpr uint8_t flag_error = 1u << 1;

    static inline uint8_t next_flags = 0;   ///< what the next service() reports
    static inline uint32_t armed = 0;
    static inline uint32_t blocks = 0;
    static inline uint32_t faults = 0;
    static inline uint32_t length = 0;
    static inline bool running = false;

    static uint8_t service() {
        const uint8_t f = next_flags;
        next_flags = 0;
        return f;
    }
    static void arm(volatile void* data, SimPl011Request request) {
        (void)data;
        (void)request;
        armed = armed + 1u;
    }
    static bool start(const uint8_t* buffer, uint32_t len) {
        (void)buffer;
        length = len;
        running = true;
        blocks = blocks + 1u;
        return true;
    }
    static bool start(uint8_t* buffer, uint32_t len) {
        return start(static_cast<const uint8_t*>(buffer), len);
    }
    static uint32_t complete() {
        running = false;
        return length;
    }
    static uint32_t take() {
        const uint32_t n = running ? length : 0u;
        running = false;
        length = 0;
        return n;
    }
    static bool busy() { return running; }
    static bool idle() { return !running; }
    static bool full() { return false; }
    static uint32_t capacity() { return length; }
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
        running = false;
    }
};

} // namespace brio
