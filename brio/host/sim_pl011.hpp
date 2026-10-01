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
 * the block's reset values, an interrupt controller that keeps its
 * enables, pads that remember what they were handed to. The same header
 * is compiled by the host suite (test_pl011) and by a family's compile
 * check (test/family_rp2040/pl011_ip.cpp), so the proof is made twice,
 * once per compiler.
 *
 * WHAT IT MODELS, and what it does not. The register map is the PL011's,
 * at the PL011's offsets, and the RESET VALUES are the block's (UARTFR
 * comes up with both FIFOs empty, which is what lets a transport's
 * init() drain the receive FIFO and stop). THE TRANSMIT FIFO IS
 * MODELLED, because the transport's transmit policy is a contract with
 * it: a write of UARTDR is an entry (one into a full FIFO is counted as
 * the byte the silicon would lose), the WIRE is a verb the test calls
 * (`shift<i>()` takes the oldest entry and hands it back, so the order
 * on the wire is judged), UARTFR's TXFF, TXFE and BUSY follow the
 * entries, and TXRIS follows the rule measured on the RP2350
 * (docs/rp2350/uart.md) - set when the FIFO falls through the level
 * UARTIFLS selects, cleared by a write that takes it above the level or
 * by UARTICR, never set by the level alone. UARTICR clears what it is
 * written, and UARTMIS is kept equal to UARTRIS under UARTIMSC. The
 * shifter is not modelled: a byte leaves the FIFO when the wire takes
 * it. The RECEIVE side is not modelled at all - nothing ever fills it,
 * so the receive path has nothing to read; a test raises a receive
 * cause by hand when it wants an entry for the receiver alone. A test
 * that wants a received byte wants silicon.
 *
 * WHAT IT RECORDS, beside the registers: the order of the observable
 * acts of a bring-up - which pad was claimed when, and when UARTCR's
 * enable went up - so a test can assert the ORDER the IP file's init()
 * promises (the receive pad before the enable, the transmit pad after
 * it), which is a thing no bench can see.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <optional>

#include "host/platform.hpp"
#include "pl011/uart.hpp"

namespace brio {

/// UARTDR: a word in memory whose WRITE is an entry into the transmit
/// FIFO (defined below the chip, which owns the FIFO). A read gives back
/// the last word written - there is no receive side to read from.
struct SimPl011Data {
    volatile uint32_t last;
    SimPl011Data& operator=(uint32_t v);
    operator uint32_t() const { return last; }
};

/// UARTICR: a write clears the UARTRIS bits it names.
struct SimPl011Clear {
    volatile uint32_t last;
    SimPl011Clear& operator=(uint32_t mask);
    operator uint32_t() const { return last; }
};

static_assert(sizeof(SimPl011Data) == 4u && sizeof(SimPl011Clear) == 4u,
              "a register of the block is one word at its offset");

/// The PL011's register block as plain memory, the names and the offsets
/// of the block every vendor's device description spells the same way -
/// UARTDR and UARTICR the two words whose writes act.
struct SimPl011Regs {
    SimPl011Data UARTDR;             ///< 0x00
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
    SimPl011Clear UARTICR;           ///< 0x44
    volatile uint32_t UARTDMACR;     ///< 0x48
};

static_assert(offsetof(SimPl011Regs, UARTFR) == 0x18u && offsetof(SimPl011Regs, UARTICR) == 0x44u &&
              offsetof(SimPl011Regs, UARTDMACR) == 0x48u);

/// The interrupt line of one instance - a number, since there is no
/// controller here to give it a meaning.
enum class SimPl011Line : uint8_t { uart0 = 0, uart1 = 1 };

/// The interrupt controller: the two verbs the IP file calls, and the
/// state they leave. Whether the line is RAISED is the block's own
/// UARTMIS; a test that plays the core runs the handler while it is.
struct SimPl011Interrupts {
    SimPl011Interrupts() = delete;

    static inline bool line_enabled[2]{};

    static void enable(SimPl011Line line) { line_enabled[slot(line)] = true; }
    static void disable(SimPl011Line line) { line_enabled[slot(line)] = false; }

    static void reset() { line_enabled[0] = line_enabled[1] = false; }

    static constexpr uint8_t slot(SimPl011Line line) { return static_cast<uint8_t>(line); }
};

/// The transmit FIFO of each instance: its entries, and what crossed it.
struct SimPl011TxFifo {
    SimPl011TxFifo() = delete;

    static constexpr uint8_t depth = 32;

    static inline uint8_t entry[2][depth]{};
    static inline uint8_t head[2]{};       ///< the oldest entry
    static inline uint8_t count[2]{};      ///< entries waiting for the wire
    static inline uint32_t written[2]{};   ///< writes the FIFO took
    static inline uint32_t lost[2]{};      ///< writes into a FULL FIFO: lost on silicon
    static inline uint32_t sent[2]{};      ///< entries the wire took

    static void reset(uint8_t i) {
        head[i] = 0;
        count[i] = 0;
        written[i] = 0;
        lost[i] = 0;
        sent[i] = 0;
    }
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

    static_assert(fifo_depth == SimPl011TxFifo::depth);

    template <uint8_t i>
    static bool reset() {
        block[i] = Regs{};
        block[i].UARTFR = flag_reset;
        SimPl011TxFifo::reset(i);
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
    /// act a test wants to place in time. A write of UARTIMSC moves
    /// UARTMIS with it.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) {
        reg = reg | bits;
        if (is_control(reg) && (bits & UartControl::enable) != 0u) {
            SimPl011Bench::enabled_at = SimPl011Bench::tick();
        }
        refresh_all();
    }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) {
        reg = reg & ~bits;
        refresh_all();
    }

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
            SimPl011TxFifo::reset(i);
        }
        SimPl011Bench::reset();
        SimPl011Interrupts::reset();
    }

    static bool is_control(const volatile uint32_t& reg) {
        return &reg == &block[0].UARTCR || &reg == &block[1].UARTCR;
    }

    // ---- the transmit FIFO and the line, for a test that plays the wire
    // ---- and the core

    /// THE WIRE takes instance i's oldest entry and hands it back; nothing
    /// when the FIFO is empty. The fall from one entry above the level to
    /// the level is the edge that sets TXRIS.
    template <uint8_t i>
    static std::optional<uint8_t> shift() {
        if (SimPl011TxFifo::count[i] == 0u) {
            return std::nullopt;
        }
        const uint8_t b = SimPl011TxFifo::entry[i][SimPl011TxFifo::head[i]];
        SimPl011TxFifo::head[i] = static_cast<uint8_t>((SimPl011TxFifo::head[i] + 1u) % fifo_depth);
        SimPl011TxFifo::count[i] = static_cast<uint8_t>(SimPl011TxFifo::count[i] - 1u);
        SimPl011TxFifo::sent[i] = SimPl011TxFifo::sent[i] + 1u;
        if (SimPl011TxFifo::count[i] == tx_level(i)) {
            block[i].UARTRIS = block[i].UARTRIS | UartInterrupt::tx;
        }
        settle(i);
        return b;
    }

    /// A receive cause raised by hand (the receive side is not modelled):
    /// what the handler sees on an entry for the receiver alone.
    template <uint8_t i>
    static void raise(uint32_t bits) {
        block[i].UARTRIS = block[i].UARTRIS | bits;
        settle(i);
    }

    /// The line as the interrupt controller sees it: enabled, and some
    /// masked source standing (UARTINTR is the OR of UARTMIS).
    template <uint8_t i>
    static bool line_raised() {
        return SimPl011Interrupts::line_enabled[i] && block[i].UARTMIS != 0u;
    }

    /// Where TXIFLSEL puts the transmit level, in entries: at or below it
    /// the FIFO is "<= 1/8 full" and so on.
    static uint8_t tx_level(uint8_t i) {
        constexpr uint8_t eighths[] = {1, 2, 4, 6, 7};
        const uint32_t sel = (block[i].UARTIFLS & UartTriggerField::tx_bits) >> UartTriggerField::tx_lsb;
        return static_cast<uint8_t>(fifo_depth * eighths[sel < 5u ? sel : 2u] / 8u);
    }

    /// UARTFR's three transmit flags from the entries, UARTMIS from UARTRIS
    /// under UARTIMSC.
    static void settle(uint8_t i) {
        constexpr uint32_t tx_flags = UartFlag::tx_full | UartFlag::tx_empty | UartFlag::busy;
        const uint8_t n = SimPl011TxFifo::count[i];
        uint32_t fr = block[i].UARTFR & ~tx_flags;
        if (n == fifo_depth) { fr |= UartFlag::tx_full; }
        if (n == 0u) { fr |= UartFlag::tx_empty; } else { fr |= UartFlag::busy; }
        block[i].UARTFR = fr;
        block[i].UARTMIS = block[i].UARTRIS & block[i].UARTIMSC;
    }
    static void refresh_all() {
        for (uint8_t i = 0; i < instances; ++i) {
            block[i].UARTMIS = block[i].UARTRIS & block[i].UARTIMSC;
        }
    }

    /// Which block a register word belongs to.
    static uint8_t instance_of(const SimPl011Data* reg) { return reg == &block[1].UARTDR ? 1u : 0u; }
    static uint8_t instance_of(const SimPl011Clear* reg) { return reg == &block[1].UARTICR ? 1u : 0u; }

    /// A write of UARTDR: an entry, or a byte lost to a full FIFO. A write
    /// that takes the FIFO above its level clears TXRIS.
    static void transmit(uint8_t i, uint8_t b) {
        if (SimPl011TxFifo::count[i] == fifo_depth) {
            SimPl011TxFifo::lost[i] = SimPl011TxFifo::lost[i] + 1u;
            return;
        }
        const uint8_t tail = static_cast<uint8_t>((SimPl011TxFifo::head[i] + SimPl011TxFifo::count[i]) % fifo_depth);
        SimPl011TxFifo::entry[i][tail] = b;
        SimPl011TxFifo::count[i] = static_cast<uint8_t>(SimPl011TxFifo::count[i] + 1u);
        SimPl011TxFifo::written[i] = SimPl011TxFifo::written[i] + 1u;
        if (SimPl011TxFifo::count[i] > tx_level(i)) {
            block[i].UARTRIS = block[i].UARTRIS & ~UartInterrupt::tx;
        }
        settle(i);
    }

    /// A write of UARTICR.
    static void clear_raw(uint8_t i, uint32_t mask) {
        block[i].UARTRIS = block[i].UARTRIS & ~mask;
        settle(i);
    }
};

static_assert(Pl011Chip<SimPl011>);

inline SimPl011Data& SimPl011Data::operator=(uint32_t v) {
    last = v;
    SimPl011::transmit(SimPl011::instance_of(this), static_cast<uint8_t>(v));
    return *this;
}

inline SimPl011Clear& SimPl011Clear::operator=(uint32_t mask) {
    last = mask;
    SimPl011::clear_raw(SimPl011::instance_of(this), mask);
    return *this;
}

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
    /// Stage the race a real controller offers: the run's last element
    /// lands right AFTER take() has read the count, so the next idle()
    /// answers true over a run one element short of counted.
    static inline bool ends_under_take = false;
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
        if (ends_under_take && running && length != 0u) {
            ends_under_take = false;
            running = false;
            return length - 1u;
        }
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
        ends_under_take = false;
        armed = 0;
        blocks = 0;
        faults = 0;
        length = 0;
        running = false;
    }
};

} // namespace brio
