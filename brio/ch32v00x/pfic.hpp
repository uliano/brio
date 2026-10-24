/*
 * pfic.hpp
 *
 * Interrupt control on the CH32V00x: the global mask that is a CSR bit,
 * the controller that holds the per-line enables, and the one attribute
 * every interrupt handler of this target is declared with. The smallest
 * use is a vector binding and its line's enable:
 *
 *     extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
 *     brio::Pfic::enable(brio::Irq::exti7_0);       // a line a driver does not open itself
 *     brio::enable_interrupts();                     // in main, before Tenuto's run()
 *
 * Who calls what. InterruptGuard (the kernel's CriticalSection) may be
 * opened from any context and nests. enable_interrupts() is called once
 * by main() before the kernel's run(); disable_interrupts() and the
 * guard mask, nothing else here does. The Pfic verbs may be called from
 * any context: each is one store to a write-one register or one load,
 * so none needs a guard and none waits. A handler body is called from
 * the vector the program binds and nowhere else.
 *
 * Two layers. `mstatus.MIE` is the RISC-V global mask - one bit, all or
 * nothing, the equivalent of the AVR's I flag and of ARMv6-M's PRIMASK -
 * and `InterruptGuard` is the kernel's CriticalSection over it:
 * read-and-clear on entry, restore the previous value on exit, so guards
 * nest. `csrrci` does the read and the clear in one instruction, which
 * is what makes the guard correct against an interrupt arriving between
 * the two halves.
 *
 * The per-line enables belong to the PFIC, WCH's own controller (WCH's
 * register definitions call it NVIC as well; this stratum uses the name
 * the chapter uses). Its enable and disable registers are write-one: a
 * write of 1 to a bit acts, a 0 does nothing, so there is no
 * read-modify-write and no guard needed around it.
 *
 * The handler attribute is one spelling for the whole image.
 * BRIO_CH32_INTERRUPT is what an app puts on a vector binding, and it
 * follows the project's CH32V00X_HPE option (ch32v00x/CMakeLists.txt, a
 * CMake option chosen at configure, on by default, which defines
 * BRIO_CH32_HPE): with the core's hardware prologue/epilogue on, the
 * crt (the startup code that runs before main(),
 * ch32v00x/src/glue/startup_ch32v00x.S) has set INTSYSCR.HWSTKEN and
 * the attribute is WCH's "WCH-Interrupt-fast" - the ten caller-saved
 * registers are pushed to the stack by the hardware on entry and popped
 * on MRET (QingKe V2 manual 3.4), and gcc emits no prologue at all; with
 * it off, the attribute is the plain `interrupt` and the handler saves
 * what it clobbers itself. The two must agree: a fast handler under an
 * HPE that is off returns into a program whose registers it clobbered.
 * One macro, one option, no way to spell them apart. Interrupt nesting
 * is off in both cases: the kernel's rule (docs/design/kernel.md,
 * section 1, "The model in one page") that an ISR body runs to
 * completion holds on this target by INTSYSCR.INESTEN never being set.
 *
 * Not here: the block's priorities and its two free vectored entries
 * (VTFIDR/VTFADDR) are real features of this core with no user yet -
 * they arrive with one.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"

/// Both spellings carry `no_icf`: gcc's identical code folding would
/// otherwise turn the second of two handlers with one body - a transport's
/// two DMA channels, each vector calling the same dma_isr() - into a handler
/// that calls the first, and the first ends in MRET, so the call never
/// returns and whatever the caller pushed stays on the stack. Under the
/// hardware prologue the caller pushes nothing and the merge was harmless
/// by luck; under a plain `interrupt` it pushes the caller-saved registers
/// (measured on the CH32V303VCT6, whose FPU made the frame eighty bytes).
/// The attribute of every interrupt handler of this target (see the
/// file header): what an app writes on a vector binding, and the one
/// place the choice between the core's hardware prologue and gcc's is
/// spelled. Where only the preprocessor can ask (a build option, an
/// attribute), a macro is the honest tool.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
#define BRIO_CH32_INTERRUPT [[gnu::interrupt("WCH-Interrupt-fast"), gnu::no_icf]]
#else
#define BRIO_CH32_INTERRUPT [[gnu::interrupt, gnu::no_icf]]
#endif

namespace brio {

/// mstatus.MIE, the one global interrupt enable.
inline constexpr uint32_t mstatus_mie = 1UL << 3;

/// Whether mstatus.MIE is set now; any context.
inline bool interrupts_enabled() {
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    return (mstatus & mstatus_mie) != 0u;
}

/// Set mstatus.MIE: interrupts on. Called by main() once the drivers
/// are initialized and before the kernel's run().
inline void enable_interrupts() {
    __asm__ volatile("csrsi mstatus, 8" ::: "memory");
}

/// Clear mstatus.MIE. A scope that masks and unmasks uses
/// InterruptGuard instead, which restores what it found.
inline void disable_interrupts() {
    __asm__ volatile("csrci mstatus, 8" ::: "memory");
}

/**
 * The kernel's CriticalSection on this target.
 *
 * The constructor clears mstatus.MIE and keeps what it found; the
 * destructor sets the bit again only if it was set, so an inner guard
 * inside an outer one leaves interrupts masked on the way out. Both
 * halves carry a "memory" clobber, which is the barrier the Platform
 * concept asks for: data shared with a handler needs no volatile of
 * its own inside a guarded region.
 */
class InterruptGuard {
public:
    [[gnu::always_inline]] InterruptGuard() {
        __asm__ volatile("csrrci %0, mstatus, 8" : "=r"(saved_) :: "memory");
    }

    [[gnu::always_inline]] ~InterruptGuard() {
        if ((saved_ & mstatus_mie) != 0u) {
            __asm__ volatile("csrsi mstatus, 8" ::: "memory");
        }
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t saved_;
};

/// PFIC_GISR.GACTSTA: an interrupt handler is executing (Pfic::in_handler()).
inline constexpr uint32_t pfic_gisr_gactsta = 1UL << 8;

/// The interrupt controller's per-line enables. Monostate (a type used
/// only through its static members, never as an object): there is one
/// PFIC, and its two registers are write-one, so no verb here needs a
/// guard around it.
struct Pfic {
    Pfic() = delete;

    /// Enable a line at the controller (IENR, write-one). Ticker::init(),
    /// Uart::init(), the I2C tasks, the SPI host and client, a DMA
    /// engine and the timed sleep site's arm() open their own vector's
    /// line; a line no driver opens, such as the EXTI pin group of
    /// docs/ch32v00x/pin.md's example or the ADC's (Adc::init() leaves
    /// it closed), is the program's to enable.
    static void enable(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IENR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Disable a line at the controller (IRER, write-one).
    static void disable(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IRER[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Is the line enabled? (The manual's ISR bank is the enable status.)
    static bool enabled(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (pfic()->ISR[n >> 5] & (1UL << (n & 0x1fu))) != 0u;
    }

    /// Is the line asserted and waiting? (The IPR bank.)
    static bool pending(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (pfic()->IPR[n >> 5] & (1UL << (n & 0x1fu))) != 0u;
    }

    /// Raise a line by hand - the software interrupt's own way in, and
    /// a test's way of firing any vector on demand. Write-one.
    static void set_pending(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IPSR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Withdraw a pending line. Write-one.
    static void clear_pending(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IPRR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Is the line being serviced right now? (The IACTR bank.)
    static bool active(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (pfic()->IACTR[n >> 5] & (1UL << (n & 0x1fu))) != 0u;
    }

    /// Whether the caller runs inside an interrupt handler: PFIC_GISR's
    /// GACTSTA, bit 8 of the register at offset 0x4C, which the chapter
    /// describes as an interrupt being executed, whatever the nesting level
    /// (the CH32V00X reference manual 6.5.2.7 for the CH32V006, the
    /// CH32V003's own 6.5.2.7 and the QingKe V2 manual 3.1: the same
    /// register and the same bit on both parts). Not mstatus.MIE, which
    /// says nothing here: interrupt entry leaves it set on this core
    /// (QingKe V2 manual 2.2, docs/ch32v00x/platform.md), and a critical
    /// section in thread code clears it. The platform's in_handler()
    /// (kernel/platform.hpp's optional member) is this read. Any context;
    /// one load and a test.
    static bool in_handler() { return (pfic()->GISR & pfic_gisr_gactsta) != 0u; }
};

} // namespace brio
