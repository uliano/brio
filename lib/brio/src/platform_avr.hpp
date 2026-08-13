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
 */

#pragma once

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "platform.hpp"
#include "ticker.hpp"

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
    static void idle() {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    }

    /// Halt in the debugger when the OCD is active, plain NOP otherwise.
    static void break_here() { __asm__ __volatile__("break"); }

    static uint32_t now() { return Ticker::ticks(); }
};

static_assert(Platform<AvrPlatform>);

} // namespace brio
