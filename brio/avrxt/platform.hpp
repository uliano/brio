/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on an AVR DA, DB or EA:
 * this target's realization of the kernel's Platform concept, the
 * compiler-checked set of requirements kernel/platform.hpp states. A
 * program names it once and passes it to every kernel template - its
 * queues, its time events and its Tenuto - as the maintained example
 * `blink` (avrxt/src/apps/blink.cpp) spells it:
 *
 *     using P = brio::AvrPlatform;
 *     ...
 *     ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init();
 *         sei();
 *         brio::Tenuto<P, Blinker, Supervisor>::run();
 *     }
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time
 * (avrxt/ticker.hpp's Ticker, 1024 Hz on the RTC's periodic interrupt),
 * a breakpoint for a panic and the storage a panic record survives a
 * reset in. Who calls what: the kernel loop calls idle() in main-loop
 * context, with interrupts already masked, and nothing else should;
 * now() may be read from any context (Ticker::ticks() masks for the
 * four loads that copy the count); CriticalSection may be opened from
 * any context and nests; break_here() is the panic path's and
 * panic_record() the panic path's and the next boot's. A program binds
 * the timebase's vector itself (RTC_PIT_vect to Ticker::pit(), above);
 * the kernel binds no vector, and a program whose handler does not call
 * pit() never advances time.
 *
 * The one promise the kernel relies on and no member states - no
 * interrupt handler interrupts another (docs/design/kernel.md, "The
 * model in one page") - this core keeps in its interrupt controller,
 * not in SREG: the I bit is not cleared on entry to a handler, nor set
 * by RETI (DS40002247B and DS40002443A 7.6.3). Every vector but the NMI
 * is at priority level 0 out of reset (15.3.2.4.3), a level-0 handler is
 * interrupted only by the one vector CPUINT.LVL1VEC raises to level 1 or
 * by an NMI (15.3.2.4.2), and CPUINT.STATUS.LVL0EX holds the others off
 * until the handler's RETI (15.5.2). Nothing in brio writes LVL1VEC.
 *
 * CriticalSection mirrors ATOMIC_BLOCK(ATOMIC_RESTORESTATE): save SREG,
 * cli, restore on scope exit, with compiler memory barriers on both ends
 * (cli() carries one; the explicit asm barrier before the SREG restore
 * keeps protected writes inside the section).
 *
 * The other half of the panic breadcrumb - which reset actually
 * happened, and how to cause one - is avrxt/reset.hpp: this header
 * only provides the storage the record lives in. The sleep modes deeper
 * than IDLE, which idle() takes when something above the kernel armed
 * one, are avrxt/sleep.hpp's.
 */

#pragma once

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "kernel/platform.hpp"
#include "avrxt/ticker.hpp"

namespace brio {

struct AvrPlatform {
    class CriticalSection {
    public:
        [[gnu::always_inline]] CriticalSection() : saved_sreg_(SREG) { cli(); }
        [[gnu::always_inline]] ~CriticalSection() {
            __asm__ __volatile__("" ::: "memory");
            SREG = saved_sreg_;
        }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;

    private:
        uint8_t saved_sreg_;
    };

    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt and returns with interrupts enabled. Not for a program
    /// to call. sei takes effect after the following instruction,
    /// so no wakeup can slip between the caller's queue check and the
    /// sleep - the lost-wakeup race is closed by the silicon.
    ///
    /// IDLE by default: the CPU stops, every peripheral and every
    /// interrupt source stays alive (13.3.3.1), and waking costs six
    /// CLK_PER cycles (13.3.3.2). The deeper modes gate clocks and
    /// shrink the wake-up list, which is a power-management decision an
    /// AO system must take deliberately - not something the kernel's
    /// "nothing to do" hook may do behind the app's back.
    ///
    /// But an already-armed mode stands. If SEN is set when this runs,
    /// something above the kernel armed a mode on purpose - a power
    /// manager (util/power.hpp) through avrxt/sleep.hpp's AvrSleepSite,
    /// which arms and returns, leaving the sleeping to this hook. That
    /// split is the whole reason the power model needs no new kernel
    /// hook, and re-arming IDLE here would silently undo the decision
    /// every stakeholder just voted on. The arming is then also the
    /// manager's to clear: it disarms on the first event it dispatches
    /// after the wake, so a mode survives a wake that had nothing to say
    /// to it and the program stops again on the next empty turn.
    static void idle() {
        // One write arms mode + enable, one write disarms. Errata
        // DS80000915F 2.2.4 (all silicon revisions): a store to an
        // address >= 64 immediately followed by a write to SLPCTRL.CTRLA
        // loses that write - a NOP before each CTRLA write is the
        // documented workaround (avr-libc's set_sleep_mode/sleep_enable
        // pair would be two back-to-back read-modify-writes of CTRLA).
        // The AVR EA's twin is DS80001048D 2.2.3 (revisions B1 and B2).
        // The DA errata of record (DS80000882C) predates the item and
        // has no twin; the NOP costs one cycle and is emitted on every
        // family rather than trusting that silence.
        // SLPCTRL.CTRLA is not under CCP (13.3.5): only VREGCTRL is, and
        // the AVR EA's SLPCTRL is CTRLA alone (DS40002443A 13.4).
        if ((SLPCTRL.CTRLA & SLPCTRL_SEN_bm) != 0) {
            sei();
            sleep_cpu();
            return;                              // the arming is not ours
        }
        __asm__ __volatile__("nop");
        SLPCTRL.CTRLA = SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm;
        sei();
        sleep_cpu();
        __asm__ __volatile__("nop");
        SLPCTRL.CTRLA = 0;                       // sleep disabled again
    }

    /// SLPCTRL.CTRLA.SEN: a SLEEP instruction would suspend the CPU.
    /// idle() leaves it false - the readback that proves the disarm.
    static bool sleep_armed() { return (SLPCTRL.CTRLA & SLPCTRL_SEN_bm) != 0; }

    /// SREG.I: the one bit CriticalSection saves and restores.
    static bool interrupts_enabled() { return (SREG & (1u << SREG_I)) != 0; }

    /// Halt in the debugger when the OCD is active, plain NOP otherwise:
    /// the panic path's breakpoint (kernel/panic.hpp), callable from any
    /// context. With no debugger the panic continues into its reporter.
    static void break_here() { __asm__ __volatile__("break"); }

    /// The kernel's clock: the Ticker's count, readable from any context
    /// (what kernel/time_event.hpp arms deadlines against).
    static uint32_t now() { return Ticker::ticks(); }

    /// Tick rate of the timebase: 1024 Hz, a truth of the 32k PIT dividers.
    static constexpr uint32_t ticks_per_second = Ticker::ticks_per_second;

    /// 8-bit core: only a single byte moves in one uninterruptible access.
    static constexpr unsigned atomic_width = 1;

    /// Panic breadcrumb in .noinit: startup clears only .bss, so this
    /// survives a (watchdog) reset and can be reported at the next boot.
    /// The C runtime is the only guarantee here - the data sheet lists
    /// what each reset clears (table 14-1) and SRAM is in no list, for
    /// any source including power-on. Hence take_panic_record()'s magic
    /// word: cold RAM is what makes the check necessary, not optional.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // NOTE: gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name, and gas
    // warns "group name for SHF_GROUP not specified". Harmless (the symbol
    // lands in .noinit, weak dedup works); candidate upstream bug report.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<AvrPlatform>);

} // namespace brio
