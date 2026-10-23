/*
 * reset.hpp
 *
 * The failing half of the platform of the CH32V203 and the CH32V303:
 * which reset happened, how to cause one, and the fault body that turns
 * a crash into a note the next boot can read. A program reads the flags
 * first thing at boot and hands both trap entries the fault body, as
 * docs/ch32vx03/platform.md's "How to use it" writes it:
 *
 *     BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }
 *     BRIO_CH32_VECTOR(breakpoint_handler) { brio::fault_reset<P>(); }
 *
 *     int main() {
 *         const uint32_t flags = brio::Reset::take_flags();    // first, and once
 *         const auto record = brio::take_panic_record<P>();    // fetch-and-clear
 *         ...
 *     }
 *
 * `P` is the program's platform type (`using P =
 * brio::Ch32vx03Platform<>;`, platform.hpp), whose panic_record() is the
 * breadcrumb in `.noinit` that survives the reset; fault_handler and
 * breakpoint_handler are the crt's names for the exception entry (index
 * 3) and the breakpoint entry (index 9) of the vector table, and both
 * are bound because an `ebreak` with no debugger arrives on the second
 * (below). take_panic_record() is kernel/panic.hpp's, the reader of the
 * record a fault_reset() or a panic() left.
 *
 * Who calls what: take_flags() is the boot verb, in main() before
 * anything else touches RCC_RSTSCKR - clear_flags() is two unguarded
 * read-modify-writes of a register it shares with the clock tree's LSI
 * bits; flags() is one load, any context; Reset::software(), from any
 * context, is one keyed store and then a spin until the reset lands,
 * never returning; the trap readers machine_cause(), machine_epc() and
 * machine_tval() are one CSR read each, meant for the top of a handler;
 * fault_reset<P>() is the body of a bound trap vector and runs in the
 * trap, writing the record by hand and resetting, never returning;
 * ResetReporter is the reporter panic() calls. Nothing here masks
 * interrupts.
 *
 * The flags accumulate. RCC_RSTSCKR (RM 3.4.10) carries one bit per
 * reset source and they stay set until the software writes RMVF, so a
 * read is a history since the last clear, not "the cause". The
 * register's own reset value states the first entry of that history:
 * 0x0C000000, PORRSTF and PINRSTF together, so at the boot after a
 * power-on the pin flag stands without a pin having been pulled low. A
 * software reset, measured, raises SFTRSTF alone and leaves the pin
 * flag where it was. Each flag therefore means what it says only for a
 * program that clears the register at every boot, which is what
 * take_flags() is for.
 *
 * The family's own sixth flag is LPWRRSTF, the low-power reset: with
 * nRST_STDBY or nRST_STOP cleared in the option bytes, the sequence
 * that would enter Standby or Stop resets the chip instead (RM 3.2.2).
 * The option bytes are the flash chapter's; the flag is read here.
 * Nothing on the bench has raised it (docs/ch32vx03/platform.md, "Not
 * covered yet").
 *
 * The reset request is the core's. There is no reset controller on this
 * family either: the request is PFIC_CFGR.RSTSYS (RM 9.5.2.10, the
 * QingKe V4 manual's PFIC_CFGR.SYSRESET), and that register takes a
 * write only with the right one of its three keys in the high half -
 * KEY3, 0xBEEF, for this bit. It shows up as SFTRSTF at the next boot.
 * PFIC_SCTLR bit 31 is the same reset with no key and has no verb here.
 *
 * The fault body does not go through panic(). panic() ends in
 * break_here(), which on this core is `ebreak`, and `ebreak` with no
 * debugger attached is taken to a vector of this same table - so a
 * fault body that called panic() would re-enter itself. It writes the
 * same record panic() would, by hand, and resets; and it never
 * overwrites a record that already stands, because with no debugger
 * panic()'s own ebreak arrives here, and clobbering the record would
 * turn every diagnosed panic into a kernel_fault.
 *
 * Which vector a trap arrives on is not the exception code. This
 * table has two entries a trap can take - the exception vector at
 * index 3 and the breakpoint one at index 9 - and an `ebreak` with no
 * debugger attached lands on the second (measured), while mcause
 * reports exception code 3, this core's number for a breakpoint. A
 * program that wants a crash recorded binds both entries to this body.
 *
 * What killed it is in mcause. The core updates mcause, mepc and mtval
 * on the way in (QingKe V4 manual 2.2): the cause register's top bit
 * says interrupt or exception and its low bits carry the code of table
 * 2-1, mepc the instruction that trapped and mtval the address or the
 * opcode behind it. The breadcrumb has one byte for the detail, so the
 * cause byte is what crosses the reset (fault_context()); a program
 * that wants the address reads mepc in its own handler, before it calls
 * anything.
 *
 * Not here: the two watchdogs (RM ch. 7 and 8), which reset the chip
 * too and are watchdog.hpp's - this file reads their flags and knows
 * nothing else about them - and the reset-related option bytes, which
 * are the flash chapter's.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"
#include "kernel/panic.hpp"
#include "kernel/platform.hpp"

namespace brio {

/// One bit per reset source, exactly as RCC_RSTSCKR carries them.
struct ResetFlag {
    static constexpr uint32_t low_power            = 1UL << 31;  ///< LPWRRSTF
    static constexpr uint32_t window_watchdog      = 1UL << 30;  ///< WWDGRSTF
    static constexpr uint32_t independent_watchdog = 1UL << 29;  ///< IWDGRSTF
    static constexpr uint32_t software             = 1UL << 28;  ///< SFTRSTF
    static constexpr uint32_t power                = 1UL << 27;  ///< PORRSTF
    static constexpr uint32_t pin                  = 1UL << 26;  ///< PINRSTF, and see the file header

    static constexpr uint32_t all = low_power | window_watchdog | independent_watchdog |
                                    software | power | pin;
    static constexpr uint32_t watchdog = window_watchdog | independent_watchdog;
};

/// RCC_RSTSCKR's own bit beside the flags. The two the register shares
/// with the clock tree (LSION, LSIRDY) are device.hpp's.
inline constexpr uint32_t rstsckr_rmvf = 1UL << 24;

/// PFIC_CFGR: the bit that resets the chip, and the key it must be
/// written with. The register accepts three keys for three groups of
/// bits (KEY1 = 0xFA05, KEY2 = 0xBCAF, KEY3 = 0xBEEF); this one is
/// KEY3's, and no other bit of the register has a user here.
inline constexpr uint32_t pfic_key3   = 0xBEEF0000UL;
inline constexpr uint32_t pfic_rstsys = 1UL << 7;

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

    /// Read-and-clear: the boot verb, called once in main() before
    /// anything else - the flags as they stood, and the register left
    /// clean for the next boot's read.
    static uint32_t take_flags() {
        const uint32_t f = flags();
        clear_flags();
        return f;
    }

    /// Reset the device now. Shows up as SFTRSTF at the next boot,
    /// beside whatever the flags already carried. The spin after the
    /// store is for the compiler and for the few cycles the request
    /// takes to land.
    [[noreturn]] static void software() {
        pfic()->CFGR = pfic_key3 | pfic_rstsys;
        for (;;) {
        }
    }
};

// =============================================================================
// The trap registers (QingKe V4 manual 2.2 and 8.2)
// =============================================================================

/// mcause[31]: set for an interrupt, clear for an exception.
inline constexpr uint32_t mcause_interrupt = 1UL << 31;

/// The exception codes of this core (manual table 2-1), which are what
/// mcause[30:0] carries when mcause[31] is clear. With mcause[31] set
/// the same field is the interrupt number instead - an `Irq`.
enum class FaultCause : uint8_t {
    instruction_misaligned = 0,
    instruction_access     = 1,
    illegal_instruction    = 2,
    breakpoint             = 3,
    load_misaligned        = 4,
    load_access            = 5,
    store_misaligned       = 6,
    store_access           = 7,
    ecall_user             = 8,
    ecall_machine          = 11,
};

/// Why the core trapped, where it was, and the value behind it. Read
/// them at the top of a handler: what they hold is the last trap's, so
/// a second one on the way to reading them is what loses the first.
inline uint32_t machine_cause() {
    uint32_t v;
    __asm__ volatile("csrr %0, mcause" : "=r"(v));
    return v;
}
inline uint32_t machine_epc() {
    uint32_t v;
    __asm__ volatile("csrr %0, mepc" : "=r"(v));
    return v;
}
inline uint32_t machine_tval() {
    uint32_t v;
    __asm__ volatile("csrr %0, mtval" : "=r"(v));
    return v;
}

/// The breadcrumb's one detail byte, packed: bit 7 is mcause[31] and
/// the seven below it are the code. Every exception code of this core
/// fits, and so does every interrupt number of this family's vector
/// tables (103 at most, the CH32V303's); a number above 127 would not,
/// and this silicon has none.
inline constexpr uint8_t fault_interrupt_bit = 0x80;

inline uint8_t fault_context() {
    const uint32_t cause = machine_cause();
    return static_cast<uint8_t>(((cause >> 24) & fault_interrupt_bit) |
                                static_cast<uint8_t>(cause & 0x7FUL));
}

constexpr bool fault_was_interrupt(uint8_t context) {
    return (context & fault_interrupt_bit) != 0u;
}
constexpr uint8_t fault_code(uint8_t context) {
    return static_cast<uint8_t>(context & 0x7Fu);
}

/// The cause byte as a word for a console line. An interrupt number and
/// a code this core does not define both answer "unknown": the number
/// is in the byte either way.
inline const char* fault_cause_name(uint8_t context) {
    if (fault_was_interrupt(context)) {
        return "interrupt";
    }
    switch (static_cast<FaultCause>(fault_code(context))) {
        case FaultCause::instruction_misaligned: return "instruction misaligned";
        case FaultCause::instruction_access:     return "instruction access";
        case FaultCause::illegal_instruction:    return "illegal instruction";
        case FaultCause::breakpoint:             return "breakpoint";
        case FaultCause::load_misaligned:        return "load misaligned";
        case FaultCause::load_access:            return "load access";
        case FaultCause::store_misaligned:       return "store misaligned";
        case FaultCause::store_access:           return "store access";
        case FaultCause::ecall_user:             return "ecall from user mode";
        case FaultCause::ecall_machine:          return "ecall from machine mode";
    }
    return "unknown";
}

/// The panic Reporter that ends in a reset instead of a halt: the
/// breadcrumb is written by panic() before this runs, and the next boot
/// takes it.
struct ResetReporter {
    static void report(PanicCode, uint8_t) { Reset::software(); }
};

/**
 * The fault vector's body: record the wreck and reset. An app binds it
 * to the vectors an exception can arrive on -
 *
 *     BRIO_CH32_VECTOR(fault_handler) {
 *         brio::fault_reset<brio::Ch32vx03Platform<>>();
 *     }
 *
 * - and the record carries the core's own account of the cause
 * (fault_context()). The overload taking a byte is for a handler that
 * has something better to say than mcause: the vector it was bound to,
 * an application state, a subsystem id.
 *
 * An existing record is not overwritten (see the file header). It runs
 * in the trap, in the vector's own context, and never returns: the
 * record is written in place in P::panic_record() and
 * Reset::software() follows.
 */
template <Platform P>
[[noreturn]] void fault_reset(uint8_t context) {
    PanicRecord& r = P::panic_record();
    if (r.magic != panic_magic) {
        r = PanicRecord{panic_magic,
                        static_cast<uint8_t>(PanicCode::kernel_fault), context};
    }
    Reset::software();
}

template <Platform P>
[[noreturn]] void fault_reset() {
    fault_reset<P>(fault_context());
}

} // namespace brio
