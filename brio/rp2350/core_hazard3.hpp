/*
 * core_hazard3.hpp
 *
 * The RISC-V half of this target's core: a pair of Hazard3 harts
 * (RV32IMA with Zicsr/Zifencei, the Zba/Zbb/Zbs/Zbkb bit manipulation
 * sets and the Zca/Zcb/Zcmp compressed ones - datasheet 3.8.1), with
 * three custom extensions of which one matters here: Xh3irq, the
 * interrupt controller that multiplexes the chip's 52 system interrupt
 * lines onto the standard external interrupt (3.8.6.1).
 *
 * NOBODY INCLUDES THIS FILE DIRECTLY: rp2350/core.hpp does, when the
 * processor in the socket is a hart, and that is the ONE place in the
 * stratum where the architecture is asked. The names below are the same
 * names core_m33.hpp exports, so a driver that includes core.hpp cannot
 * tell which half answered.
 *
 * THE INTERRUPT NUMBERS ARE THE ARM ONES (3.8.4.2): the system IRQ
 * numbering is shared between the two architectures, so this controller
 * takes the device header's `IRQn_Type` enumerators - `UART0_IRQ_IRQn`
 * means line 33 on a hart as it does on an M33 - and the vector an app
 * binds has the same name on both (the crts: rp2350/src/glue/).
 *
 * THE ARRAY CSR IDIOM (3.8.6.1.1). Xh3irq holds one bit per line in
 * arrays too wide for a CSR, so each array answers at ONE CSR address
 * as a 16-bit WINDOW: the low half of the value written selects which
 * window, the high half is the data. A set is `csrs`, a clear is
 * `csrc`, and a read is a `csrrs` of the index alone (whose high half is
 * zero, so it sets nothing). Three arrays are used here - the enables
 * MEIEA, the pending bits MEIPA and the FORCE bits MEIFA, which are how
 * a line is made pending in software.
 *
 * THE PRIORITY ARRAY MEIPRA IS LEFT AT ITS RESET VALUE, as every other
 * target of this project leaves its own: the kernel's promise is that no
 * interrupt nests over another (design/kernel.md sections 1 and 11), and
 * here that is kept structurally - the crt's dispatch never sets
 * mstatus.MIE inside a handler, so nothing can preempt whatever the
 * numbers say. Note for a reader who knows the SDK: in the hardware,
 * numerically HIGHER means more urgent, the opposite of the Arm
 * convention and of the SDK's own irq_set_priority(). That is exactly
 * why `Irq` carries no priority verb.
 *
 * MSLEEP IS NEVER WRITTEN. Hazard3's Xh3power extension can make a WFI
 * gate the core's clock (MSLEEP.DEEPSLEEP) or drop its power
 * (POWERDOWN); erratum RP2350-E4 makes the first of those a trap on core
 * 1 - with DEEPSLEEP set, a debugger's system-bus reads stall until that
 * core wakes - and the datasheet's own note is that the saving over a
 * plain WFI is minimal. So `wait_for_interrupt()` is a bare wfi, and the
 * power chapter is where anything deeper gets argued.
 *
 * WFI HERE IS NOT THE QingKe's WFI (the trap the ch32v00x stratum
 * carries): 3.8.5 states that wfi IGNORES mstatus.MIE and respects every
 * other interrupt control, so a wfi executed with interrupts masked and
 * a pending enabled line falls through immediately. That is what makes
 * the kernel's idle path - mask, look at the queues, sleep, unmask -
 * free of a lost-wakeup window on this half, exactly as on the Arm one.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

namespace brio {

/// mstatus.MIE, the one global interrupt enable of this architecture.
inline constexpr uint32_t mstatus_mie = 1UL << 3;

/// Read a CSR named by number (the assembler knows none of Hazard3's
/// names, and the pico-sdk's rvcsr.h is where the numbers come from).
template <uint32_t csr>
[[gnu::always_inline]] inline uint32_t csr_read() {
    uint32_t value;
    __asm__ volatile("csrr %0, %1" : "=r"(value) : "i"(csr));
    return value;
}

/// Set bits in a CSR, atomically with respect to interrupts.
template <uint32_t csr>
[[gnu::always_inline]] inline void csr_set(uint32_t bits) {
    __asm__ volatile("csrs %0, %1" :: "i"(csr), "r"(bits) : "memory");
}

/// Clear bits in a CSR, atomically with respect to interrupts.
template <uint32_t csr>
[[gnu::always_inline]] inline void csr_clear(uint32_t bits) {
    __asm__ volatile("csrc %0, %1" :: "i"(csr), "r"(bits) : "memory");
}

/// Set bits and return what was there: one instruction, which is what
/// makes it safe against an interrupt between the halves - and the
/// documented way to READ an array CSR's window (the bits written are
/// the window index alone, and setting them sets nothing in the data).
template <uint32_t csr>
[[gnu::always_inline]] inline uint32_t csr_read_set(uint32_t bits) {
    uint32_t old;
    __asm__ volatile("csrrs %0, %1, %2" : "=r"(old) : "i"(csr), "r"(bits) : "memory");
    return old;
}

inline bool interrupts_enabled() {
    return (csr_read<RVCSR_MSTATUS_OFFSET>() & mstatus_mie) != 0u;
}

/// Unmask interrupts globally. What an app calls once, at the end of
/// main()'s setup, before handing control to the kernel.
inline void enable_interrupts() { __asm__ volatile("csrsi mstatus, 8" ::: "memory"); }

/// Mask interrupts globally. Prefer InterruptGuard: a bare disable has
/// no matching restore and does not nest.
inline void disable_interrupts() { __asm__ volatile("csrci mstatus, 8" ::: "memory"); }

/**
 * The kernel's CriticalSection on this half.
 *
 * The constructor clears mstatus.MIE and keeps what it found; the
 * destructor sets the bit again only if it WAS set, so an inner guard
 * inside an outer one leaves interrupts masked on the way out. `csrrci`
 * does the read and the clear in ONE instruction, which is what makes
 * the guard correct against an interrupt arriving between the two
 * halves. Both halves carry a "memory" clobber, the barrier the Platform
 * concept asks for: data shared with a handler needs no volatile of its
 * own inside a guarded region.
 *
 * AND IT IS PER CORE, as PRIMASK is on the other half: mstatus belongs
 * to the hart that writes it, so a critical section excludes this core's
 * handlers and nothing that runs on the other core.
 */
class InterruptGuard {
public:
    InterruptGuard() {
        __asm__ volatile("csrrci %0, mstatus, 8" : "=r"(saved_) :: "memory");
    }

    ~InterruptGuard() {
        if ((saved_ & mstatus_mie) != 0u) {
            __asm__ volatile("csrsi mstatus, 8" ::: "memory");
        }
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t saved_;
};

/// The three interrupt sources of the architecture itself, in mie: the
/// external one (every system line, through Xh3irq), the machine timer
/// (the SIO's MTIME against this core's MTIMECMP) and the machine
/// software interrupt (the SIO's RISCV_SOFTIRQ bit for this core). Each
/// is armed by whoever owns it - the interrupt controller below with its
/// first line, the ticker once a deadline is in MTIMECMP - and NOT by
/// the crt, which leaves mie empty: MTIME runs out of reset, so a
/// blanket arm would take a timer trap on the spot.
inline void machine_external_interrupt(bool on) {
    if (on) { csr_set<RVCSR_MIE_OFFSET>(RVCSR_MIE_MEIE_BITS); }
    else { csr_clear<RVCSR_MIE_OFFSET>(RVCSR_MIE_MEIE_BITS); }
}
inline void machine_timer_interrupt(bool on) {
    if (on) { csr_set<RVCSR_MIE_OFFSET>(RVCSR_MIE_MTIE_BITS); }
    else { csr_clear<RVCSR_MIE_OFFSET>(RVCSR_MIE_MTIE_BITS); }
}
inline void machine_software_interrupt(bool on) {
    if (on) { csr_set<RVCSR_MIE_OFFSET>(RVCSR_MIE_MSIE_BITS); }
    else { csr_clear<RVCSR_MIE_OFFSET>(RVCSR_MIE_MSIE_BITS); }
}

/// Number of distinct interrupt priority levels: sixteen, as on the M33
/// half - and unused on both, since nothing nests.
inline constexpr uint8_t irq_priority_levels = 16;

/**
 * The per-line interrupt controller, under the name both architectures
 * answer to: Xh3irq's enable, pending and force arrays, addressed by the
 * device header's IRQn_Type. There is one controller per hart, each with
 * its own arrays, and every system line reaches both - so a line is
 * enabled by exactly one core, the rule this stratum keeps.
 */
struct Irq {
    Irq() = delete;

    /// Enable a line, and with the first one the architecture's external
    /// interrupt itself. A number outside the chip's 52 lines (a core
    /// exception of the other half, say) does nothing.
    static void enable(IRQn_Type irq) {
        if (!valid(irq)) {
            return;
        }
        csr_set<RVCSR_MEIEA_OFFSET>(window(irq));
        machine_external_interrupt(true);
    }

    /// Disable a line. mie.MEIE is left as it is: the per-line enable is
    /// the gate, and the other lines are not this one's business.
    static void disable(IRQn_Type irq) {
        if (!valid(irq)) {
            return;
        }
        csr_clear<RVCSR_MEIEA_OFFSET>(window(irq));
    }

    static bool enabled(IRQn_Type irq) {
        return valid(irq) && (read_array<RVCSR_MEIEA_OFFSET>(irq) != 0u);
    }

    /// Is the line asserted? MEIPA shows a line whether or not it is
    /// enabled, and whether or not its priority lets it in.
    static bool pending(IRQn_Type irq) {
        return valid(irq) && (read_array<RVCSR_MEIPA_OFFSET>(irq) != 0u);
    }

    /// Raise a line in software: the FORCE array, which makes MEIPA read
    /// as asserted although the peripheral is silent. The six spare
    /// lines (46..51) exist for exactly this and reach no hardware.
    static void set_pending(IRQn_Type irq) {
        if (!valid(irq)) {
            return;
        }
        csr_set<RVCSR_MEIFA_OFFSET>(window(irq));
    }

    /// Withdraw a FORCED line. What a peripheral asserts is not
    /// withdrawable here - it clears when the peripheral stops asserting
    /// it - and a forced bit also clears by itself when the dispatch
    /// samples it, so this verb is for a force that has not been taken
    /// yet.
    static void clear_pending(IRQn_Type irq) {
        if (!valid(irq)) {
            return;
        }
        csr_clear<RVCSR_MEIFA_OFFSET>(window(irq));
    }

private:
    static constexpr bool valid(IRQn_Type irq) {
        return static_cast<int32_t>(irq) >= 0 && static_cast<uint32_t>(irq) < NUM_IRQS;
    }

    /// The window index in the low half, the line's bit in the high one.
    static constexpr uint32_t window(IRQn_Type irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (n >> 4) | (1UL << ((n & 15u) + 16u));
    }

    template <uint32_t csr>
    static uint32_t read_array(IRQn_Type irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        const uint32_t value = csr_read_set<csr>(n >> 4);
        return (value >> 16) & (1UL << (n & 15u));
    }
};

/// Sleep until an interrupt is pending. mstatus.MIE is ignored by this
/// instruction and every other interrupt control is respected (3.8.5),
/// so a wake that is already pending does not sleep at all - the
/// lost-wakeup window the kernel's idle path would otherwise have.
[[gnu::always_inline]] inline void wait_for_interrupt() {
    __asm__ volatile("wfi" ::: "memory");
}

/// Halt in the debugger. With no debugger attached this raises a
/// breakpoint exception, which lands in the crt's exception trap - the
/// legible wreck a panic wants.
[[gnu::always_inline]] inline void debug_break() { __asm__ volatile("ebreak"); }

} // namespace brio
