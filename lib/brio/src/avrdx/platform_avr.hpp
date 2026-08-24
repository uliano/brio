/*
 * platform_avr.hpp
 *
 * AVR implementation of the brio Platform concept - the one header of
 * the kernel family that is allowed to touch AVR headers. Apps select it
 * by including it and passing AvrPlatform to the kernel templates.
 *
 * CriticalSection mirrors ATOMIC_BLOCK(ATOMIC_RESTORESTATE): save SREG,
 * cli, restore on scope exit, with compiler memory barriers on both ends
 * (cli() carries one; the explicit asm barrier before the SREG restore
 * keeps protected writes inside the section).
 *
 * The other half of the panic breadcrumb - which reset actually
 * happened, and how to cause one - is avrdx/reset.hpp: this header
 * only provides the storage the record lives in.
 */

#pragma once

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "kernel/platform.hpp"
#include "avrdx/ticker.hpp"

namespace brio {

struct AvrPlatform {
    class CriticalSection {
    public:
        CriticalSection() : saved_sreg_(SREG) { cli(); }
        ~CriticalSection() {
            __asm__ __volatile__("" ::: "memory");
            SREG = saved_sreg_;
        }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;

    private:
        uint8_t saved_sreg_;
    };

    /// Entered with interrupts masked and nothing to do: sleep until the
    /// next interrupt. sei takes effect after the following instruction,
    /// so no wakeup can slip between the caller's queue check and the
    /// sleep - the lost-wakeup race is closed by the silicon.
    ///
    /// IDLE only: the CPU stops, every peripheral and every interrupt
    /// source stays alive (13.3.3.1), and waking costs six CLK_PER
    /// cycles (13.3.3.2). The deeper modes gate clocks and shrink the
    /// wake-up list, which is a power-management decision an AO system
    /// must take deliberately - not something the kernel's "nothing to
    /// do" hook may do behind the app's back.
    static void idle() {
        // One write arms mode + enable, one write disarms. Errata
        // DS80000915F 2.2.4 (all silicon revisions): a store to an
        // address >= 64 immediately followed by a write to SLPCTRL.CTRLA
        // loses that write - a NOP before each CTRLA write is the
        // documented workaround (avr-libc's set_sleep_mode/sleep_enable
        // pair would be two back-to-back read-modify-writes of CTRLA).
        // The DA errata of record (DS80000882C) predates the item and
        // has no twin; the NOP costs one cycle and is emitted on both
        // families rather than trusting that silence.
        // SLPCTRL.CTRLA is NOT under CCP (13.3.5): only VREGCTRL is.
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

    /// Halt in the debugger when the OCD is active, plain NOP otherwise.
    static void break_here() { __asm__ __volatile__("break"); }

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
    // custom section attribute as `"awG"` WITHOUT the group name, and gas
    // warns "group name for SHF_GROUP not specified". Harmless (the symbol
    // lands in .noinit, weak dedup works); candidate upstream bug report.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<AvrPlatform>);

} // namespace brio
