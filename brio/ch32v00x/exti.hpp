/*
 * exti.hpp
 *
 * The external interrupt/event lines of the CH32V00x (RM 6.4, 6.5):
 * ten lines, eight of them pads, two internal - and the wake sources
 * the power modes count on.
 *
 * THE LINES. EXTI0..EXTI7 are pins: line n is pin n of ONE port, the
 * port chosen per line in AFIO_EXTICR (two bits each: A, B, C, D), so
 * PA3 and PB3 cannot both interrupt at once - the line is the pin
 * number. EXTI8 is the PVD's output and EXTI9 the auto-wakeup unit's
 * (RM table 6-2): the two internal events that end a Standby, each
 * with a vector of its own (Irq::pvd, Irq::awu), while the eight pin
 * lines share one (Irq::exti7_0) and a handler asks INTFR which fired.
 *
 * INTERRUPT OR EVENT. A line enabled in INTENR raises its flag, and
 * through the PFIC its vector; a line enabled in EVENR raises a WAKE
 * EVENT instead - what ends a WFE with no handler run and no flag to
 * clear (6.4.2). The flags are write-one-clear. SWIEVR raises a line
 * by software, which is how a program tests its own wiring without a
 * pad.
 *
 * `ExtInt<Pin>` is the pad-facing task: the port select, the edges,
 * the enables, the flag - the shape of the SAM's ExtInt over its EIC.
 * The bare `Exti` resource is what the internal lines are driven with
 * (sleep.hpp's AWU uses line 9 through it).
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pin.hpp"

namespace brio {

struct ExtiRegs {
    volatile uint32_t INTENR;   ///< 0x00 interrupt enable, one bit per line
    volatile uint32_t EVENR;    ///< 0x04 event enable
    volatile uint32_t RTENR;    ///< 0x08 rising-edge trigger
    volatile uint32_t FTENR;    ///< 0x0c falling-edge trigger
    volatile uint32_t SWIEVR;   ///< 0x10 software trigger, write 1
    volatile uint32_t INTFR;    ///< 0x14 the flags, write 1 to clear
};

inline ExtiRegs* exti() { return reinterpret_cast<ExtiRegs*>(pb2_base + 0x0400); }

/// AFIO_EXTICR: two bits per line 0..7 selecting the port.
inline volatile uint32_t& afio_exticr() {
    return *reinterpret_cast<volatile uint32_t*>(pb2_base + 0x0008);
}

/// The two internal lines.
inline constexpr uint8_t exti_line_pvd = 8;
inline constexpr uint8_t exti_line_awu = 9;
inline constexpr uint8_t exti_line_count = 10;

/// The block, line by line. Monostate.
struct Exti {
    Exti() = delete;

    static void interrupt(uint8_t line, bool on) {
        if (on) { exti()->INTENR |= bit(line); } else { exti()->INTENR &= ~bit(line); }
    }
    static void event(uint8_t line, bool on) {
        if (on) { exti()->EVENR |= bit(line); } else { exti()->EVENR &= ~bit(line); }
    }
    static void rising(uint8_t line, bool on) {
        if (on) { exti()->RTENR |= bit(line); } else { exti()->RTENR &= ~bit(line); }
    }
    static void falling(uint8_t line, bool on) {
        if (on) { exti()->FTENR |= bit(line); } else { exti()->FTENR &= ~bit(line); }
    }
    /// Raise the line by software: the flag comes up as if the edge had.
    static void soft(uint8_t line) { exti()->SWIEVR = bit(line); }
    static bool flag(uint8_t line) { return (exti()->INTFR & bit(line)) != 0u; }
    static void clear(uint8_t line) { exti()->INTFR = bit(line); }
    static uint32_t flags() { return exti()->INTFR; }

    /// Which port line n (0..7) listens to: 0 = A, 1 = B, 2 = C, 3 = D.
    static void port(uint8_t line, uint8_t code) {
        rcc()->PB2PCENR |= rcc_pb2_afio;
        const uint32_t shift = static_cast<uint32_t>(line) * 2u;
        afio_exticr() = (afio_exticr() & ~(0x3UL << shift)) | (static_cast<uint32_t>(code & 0x3u) << shift);
    }

    static constexpr uint32_t bit(uint8_t line) { return 1UL << line; }
};

/// One pad as an external interrupt: its line is its pin number, its
/// port is written into AFIO_EXTICR, and the edges are the caller's.
template <char L, uint8_t N>
struct ExtInt {
    static_assert(gpio_base_for(L) != 0 && N < 8u, "ExtInt: a pin of ports A..D, 0..7");

    ExtInt() = delete;

    using Input = Pin<L, N>;
    static constexpr uint8_t line = N;
    static constexpr uint8_t port_code = L == 'A' ? 0 : L == 'B' ? 1 : L == 'C' ? 2 : 3;

    /// Route the line to this pad and arm the edges asked for. The pad
    /// itself is configured by the caller (input, with or without a
    /// pull) - the line only listens.
    static void init(bool on_rising, bool on_falling) {
        Exti::port(line, port_code);
        Exti::rising(line, on_rising);
        Exti::falling(line, on_falling);
        Exti::clear(line);
    }

    static void interrupt(bool on) { Exti::interrupt(line, on); }
    static void event(bool on) { Exti::event(line, on); }
    static bool flag() { return Exti::flag(line); }
    static void clear() { Exti::clear(line); }
    static void soft() { Exti::soft(line); }
};

} // namespace brio
