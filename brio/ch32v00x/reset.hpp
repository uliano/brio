/*
 * reset.hpp
 *
 * The failing half of the CH32V00x platform: WHICH reset happened, how
 * to cause one, and the fault body that turns a crash into a note the
 * next boot can read.
 *
 * THE FLAGS ACCUMULATE. RCC_RSTSCKR (RM 3.4.9) carries one bit per
 * reset source, and they stay set until the software writes RMVF, so a
 * read is a HISTORY since the last clear, not "the cause". PORRSTF is
 * set at power-on and reads 1 on a board that never cleared it. And
 * PINRSTF NAMES THE PIN ALONE: on the STM32 this register descends
 * from, the pin flag is raised beside every system reset and can only
 * be trusted when it stands by itself; here a software reset leaves
 * SFTRSTF standing WITHOUT PINRSTF (measured: 0x18000000 at the boot
 * after one, SFTRSTF and the power-on flag and nothing else), so each
 * flag means what it says.
 *
 * THE RESET REQUEST IS THE CORE'S. There is no reset controller on
 * this family: the request is PFIC_CFGR.SYSRESET written together with
 * its key (QingKe V2 manual 3.1, KEY3 = 0xBEEF in the high half), and
 * it shows up as SFTRSTF at the next boot.
 *
 * THE FAULT BODY DOES NOT GO THROUGH panic(). panic() ends in
 * break_here(), which on this core is `ebreak`, and `ebreak` with no
 * debugger attached lands in the fault vector - so a fault body that
 * called panic() would re-enter itself. It writes the same record
 * panic() would, by hand, and resets; and it never overwrites a record
 * that already stands, because with no debugger panic()'s own ebreak
 * arrives HERE, and clobbering the record would turn every diagnosed
 * panic into a kernel_fault.
 *
 * Not here: the two watchdogs (RM ch. 4 and 5), which are the timers
 * phase's, and the reset-related option bytes (the PD7/RST choice,
 * the standby reset), which are the flash chapter's.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "kernel/panic.hpp"
#include "kernel/platform.hpp"

namespace brio {

/// One bit per reset source, exactly as RCC_RSTSCKR carries them.
struct ResetFlag {
    static constexpr uint32_t window_watchdog      = 1UL << 30;  ///< WWDGRSTF
    static constexpr uint32_t independent_watchdog = 1UL << 29;  ///< IWDGRSTF
    static constexpr uint32_t software             = 1UL << 28;  ///< SFTRSTF
    static constexpr uint32_t power                = 1UL << 27;  ///< PORRSTF
    static constexpr uint32_t pin                  = 1UL << 26;  ///< PINRSTF, the NRST pin alone
    static constexpr uint32_t opcm                 = 1UL << 25;  ///< OPCMRSTF
    static constexpr uint32_t adc                  = 1UL << 23;  ///< ADCRSTF

    static constexpr uint32_t all = window_watchdog | independent_watchdog | software |
                                    power | pin | opcm | adc;
    static constexpr uint32_t watchdog = window_watchdog | independent_watchdog;
};

/// RCC_RSTSCKR's own bits beside the flags.
inline constexpr uint32_t rstsckr_rmvf   = 1UL << 24;
inline constexpr uint32_t rstsckr_lsirdy = 1UL << 1;
inline constexpr uint32_t rstsckr_lsion  = 1UL << 0;

/// PFIC_CFGR: the key the reset bit must be written with.
inline constexpr uint32_t pfic_key3      = 0xBEEF0000UL;
inline constexpr uint32_t pfic_sysreset  = 1UL << 7;

/**
 * The reset flags, and the core's way of causing a reset.
 */
struct Reset {
    Reset() = delete;

    /// The flags as they stand. Non-destructive.
    static uint32_t flags() { return rcc()->RSTSCKR & ResetFlag::all; }

    /// Clear every flag through RMVF, leaving the LSI bits the register
    /// shares with the clock tree alone. RMVF is a plain bit: set, the
    /// flags go, put back to zero so the next boot's read is clean.
    static void clear_flags() {
        rcc()->RSTSCKR = rcc()->RSTSCKR | rstsckr_rmvf;
        rcc()->RSTSCKR = rcc()->RSTSCKR & ~rstsckr_rmvf;
    }

    /// Read-and-clear: the boot verb.
    static uint32_t take_flags() {
        const uint32_t f = flags();
        clear_flags();
        return f;
    }

    /// Reset the device now. Shows up as SFTRSTF - alone - at the next
    /// boot. The spin after the store is for the compiler and for the
    /// few cycles the request takes to land.
    [[noreturn]] static void software() {
        pfic()->CFGR = pfic_key3 | pfic_sysreset;
        for (;;) {
        }
    }
};

/// The panic Reporter that ends in a reset instead of a halt: the
/// breadcrumb is written by panic() before this runs, and the next boot
/// takes it.
struct ResetReporter {
    static void report(PanicCode, uint8_t) { Reset::software(); }
};

/**
 * The fault vector's BODY: record the wreck and reset. An app binds it:
 *
 *     extern "C" BRIO_CH32_INTERRUPT void fault_handler() {
 *         brio::fault_reset<brio::Ch32v00xPlatform<>>();
 *     }
 *
 * An existing record is not overwritten (see the file header).
 */
template <Platform P>
[[noreturn]] void fault_reset(uint8_t context = 0) {
    PanicRecord& r = P::panic_record();
    if (r.magic != panic_magic) {
        r = PanicRecord{panic_magic,
                        static_cast<uint8_t>(PanicCode::kernel_fault), context};
    }
    Reset::software();
}

} // namespace brio
