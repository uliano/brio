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
 * THE TWO WATCHDOGS (RM ch. 4 and 5) live here as well, because what
 * they do is a reset: the IWDG, a 12-bit down-counter on the LSI
 * behind a prescaler, started by a key and never stopped again but by
 * a reset, its two setting registers written only after the unlock
 * key and read only once their update flag drops; and the WWDG, a
 * 7-bit down-counter on HCLK/4096 behind a second divider that resets
 * when T6 falls AND when it is refreshed above its window, with an
 * early-warning interrupt one step before the reset. 5.2.1 says its
 * counter runs "regardless of whether the watchdog function is turned
 * on or not"; ON THIS SILICON IT DOES NOT (measured: 0x7F for 100 ms
 * with WDGA clear, EWIF never raised), so nothing about the WWDG can
 * be timed before it is armed, and armed it stays until the RCC
 * pulse. Both are the STM32F1's: the G0 stratum's driver minus the
 * IWDG window register, and a debug freeze that is a CORE CSR here
 * (DBGMCU_CR at 0x7C0, RM ch. 21) rather than a peripheral register.
 *
 * Not here: the reset-related option bytes (the RST pin choice, the
 * standby reset, IWDG_SW), which are the flash chapter's.
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

// =============================================================================
// IWDG (RM ch. 4)
// =============================================================================

struct IwdgRegs {
    volatile uint16_t CTLR;   ///< 0x00 the key register, write-only
    uint16_t RESERVED0;
    volatile uint16_t PSCR;   ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t RLDR;   ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t STATR;  ///< 0x0c
    uint16_t RESERVED3;
};

inline IwdgRegs* iwdg() { return reinterpret_cast<IwdgRegs*>(pb1_base + 0x3000); }

inline constexpr uint16_t iwdg_pvu = 1u << 0;
inline constexpr uint16_t iwdg_rvu = 1u << 1;

/// IWDG_PSCR.PR: the LSI divider ahead of the 12-bit counter. Codes 6
/// and 7 BOTH mean /256 (4.3.2), so `div256` is the one spelling and 7
/// is never written.
enum class IwdgPrescaler : uint8_t {
    div4 = 0, div8 = 1, div16 = 2, div32 = 3, div64 = 4, div128 = 5, div256 = 6,
};

constexpr uint32_t iwdg_divider(IwdgPrescaler p) { return 4UL << static_cast<uint8_t>(p); }

/// The time-out of a (prescaler, reload) pair in milliseconds at a
/// STATED LSI rate - the caller's argument, never a constant here: the
/// LSI is "about 128 kHz" and the bench measured 124 (sleep.md).
/// 4.2.1: the time-out is (reload + 1) counts.
constexpr uint32_t iwdg_timeout_ms(IwdgPrescaler p, uint16_t reload, uint32_t lsi_hz = 128'000UL) {
    return static_cast<uint32_t>((iwdg_divider(p) * (static_cast<uint32_t>(reload) + 1UL) * 1000UL) /
                                 lsi_hz);
}

struct IwdgConfig {
    IwdgPrescaler prescaler = IwdgPrescaler::div4;
    uint16_t reload = 0x0FFF;   ///< RL[11:0], loaded on every refresh
};

constexpr bool iwdg_config_valid(const IwdgConfig& c) { return c.reload <= 0x0FFFu; }

/**
 * The independent watchdog: started by a key and never stopped again
 * (4.2.1: "it can no longer be turned off unless a reset occurs"), so
 * what a caller commits to is this run of the program. `configure()`
 * unlocks and writes both registers and waits, bounded, for the two
 * update flags to drop - which they do only once the block has its
 * clock, i.e. after `start()`: on a stopped watchdog the wait runs out
 * and the verb answers false. `arm()` is the chapter's order: start,
 * configure, refresh.
 */
struct Iwdg {
    Iwdg() = delete;

    static constexpr uint16_t key_refresh = 0xAAAAu;
    static constexpr uint16_t key_unlock = 0x5555u;
    static constexpr uint16_t key_start = 0xCCCCu;

    /// Reload the counter from RLDR. The kick.
    [[gnu::always_inline]] static void refresh() { iwdg()->CTLR = key_refresh; }
    /// Open the write window on PSCR and RLDR.
    static void unlock() { iwdg()->CTLR = key_unlock; }
    /// Start the watchdog. ONE WAY IN SOFTWARE.
    static void start() { iwdg()->CTLR = key_start; }

    /// Ask for the reset on purpose: the smallest reload, then wait.
    /// Not [[noreturn]] - a few prescaled LSI counts pass before it
    /// lands, and the caller may want to say so first.
    static void force_reset() {
        unlock();
        iwdg()->RLDR = 0;
        refresh();
    }

    static constexpr uint16_t update_mask = iwdg_pvu | iwdg_rvu;
    static uint16_t status() { return iwdg()->STATR; }
    static bool busy(uint16_t mask = update_mask) { return (status() & mask) != 0u; }

    /// Bounded wait for an update to cross into the LSI domain: five
    /// LSI cycles at most (4.3.4), some 40 us; the bound is a safety
    /// net, and on a STOPPED watchdog it is what turns a hang into a
    /// false.
    static constexpr uint32_t sync_spin_limit = 400'000UL;
    static bool sync(uint16_t mask = update_mask) {
        for (uint32_t i = 0; i < sync_spin_limit; ++i) {
            if (!busy(mask)) {
                return true;
            }
        }
        return false;
    }

    /// The fields as they stand - valid only with their update flag
    /// down (4.3.2, 4.3.3), which sync() guarantees.
    static IwdgPrescaler prescaler() {
        const uint8_t code = static_cast<uint8_t>(iwdg()->PSCR & 7u);
        return static_cast<IwdgPrescaler>(code > 6u ? 6u : code);
    }
    static uint16_t reload() { return static_cast<uint16_t>(iwdg()->RLDR & 0x0FFFu); }

    /// Unlock, write both fields, wait for both updates. False for an
    /// invalid config or when the updates never land (the block is not
    /// started).
    static bool configure(const IwdgConfig& cfg) {
        if (!iwdg_config_valid(cfg)) {
            return false;
        }
        unlock();
        iwdg()->PSCR = static_cast<uint16_t>(cfg.prescaler);
        unlock();
        iwdg()->RLDR = cfg.reload;
        return sync();
    }

    /// The chapter's order (4.2.1): start, configure, refresh.
    static bool arm(const IwdgConfig& cfg) {
        start();
        const bool ok = configure(cfg);
        refresh();
        return ok;
    }

    /// DBGMCU_CR.IWDG_STOP (RM 21.2.1): whether the counter freezes
    /// while the core is halted by a debugger. A core CSR at 0x7C0.
    static bool debug_freeze() { return (dbgmcu_cr() & (1UL << 8)) != 0u; }
    static void debug_freeze(bool on) {
        if (on) {
            asm volatile("csrs 0x7C0, %0" ::"r"(1UL << 8));
        } else {
            asm volatile("csrc 0x7C0, %0" ::"r"(1UL << 8));
        }
    }

    static uint32_t dbgmcu_cr() {
        uint32_t v;
        asm volatile("csrr %0, 0x7C0" : "=r"(v));
        return v;
    }
};

// =============================================================================
// WWDG (RM ch. 5)
// =============================================================================

struct WwdgRegs {
    volatile uint16_t CTLR;   ///< 0x00 WDGA + T[6:0]
    uint16_t RESERVED0;
    volatile uint16_t CFGR;   ///< 0x04 EWI, WDGTB, W[6:0]
    uint16_t RESERVED1;
    volatile uint16_t STATR;  ///< 0x08 EWIF
    uint16_t RESERVED2;
};

inline WwdgRegs* wwdg() { return reinterpret_cast<WwdgRegs*>(pb1_base + 0x2C00); }

inline constexpr uint16_t wwdg_wdga = 1u << 7;
inline constexpr uint16_t wwdg_t6   = 1u << 6;
inline constexpr uint16_t wwdg_ewi  = 1u << 9;
inline constexpr uint16_t wwdg_ewif = 1u << 0;

/// WWDG_CFGR.WDGTB: the second divider after the fixed HCLK/4096.
enum class WwdgPrescaler : uint8_t { div1 = 0, div2 = 1, div4 = 2, div8 = 3 };

/// HCLK cycles per decrement of the 7-bit counter: 4096 x 2^WDGTB.
constexpr uint32_t wwdg_step_cycles(WwdgPrescaler p) { return 4096UL << static_cast<uint8_t>(p); }

/// Microseconds from a refresh to `t` until the reset (the counter
/// falls from t to 0x3F): (t - 0x3F) steps.
constexpr uint32_t wwdg_timeout_us(uint32_t hclk_hz, WwdgPrescaler p, uint8_t t) {
    if (t <= 0x3Fu || hclk_hz == 0u) {
        return 0;
    }
    const uint32_t mhz = hclk_hz / 1'000'000UL;
    return wwdg_step_cycles(p) * (static_cast<uint32_t>(t) - 0x3FUL) / mhz;
}

struct WwdgConfig {
    WwdgPrescaler prescaler = WwdgPrescaler::div1;
    /// W[6:0]: the high limit of the refresh window. 0x7F (the reset
    /// value) means "refresh whenever you like".
    uint8_t window = 0x7F;
    /// EWI: interrupt when the counter reaches 0x40, one step before
    /// the reset. One-way like WDGA: cleared by hardware after a reset.
    bool early_wakeup = false;
};

/// A window below 0x40 leaves no legal instant at all (5.2.1: a refresh
/// is legal while the counter is below W and above 0x3F).
constexpr bool wwdg_config_valid(const WwdgConfig& c) {
    return c.window <= 0x7Fu && c.window >= 0x40u;
}

/**
 * The window watchdog: a counter that runs only once armed (the file
 * header), an enable that only a reset clears - the RCC pulse on
 * PB1PRSTR is one, and the one way to put the block back without
 * rebooting -, and two ways to reset: the counter reaching 0x3F, or a
 * refresh made above the window.
 */
struct Wwdg {
    Wwdg() = delete;

    /// RCC_PB1PCENR.WWDGEN, CLEAR AT RESET: until it is on the
    /// registers read as nothing. 5.2.1's own note: closing it again
    /// pauses the count, "stopping the watchdog indirectly".
    static void bus_clock(bool on) {
        if (on) { rcc()->PB1PCENR |= rcc_pb1_wwdg; } else { rcc()->PB1PCENR &= ~rcc_pb1_wwdg; }
    }
    static bool bus_clock() { return (rcc()->PB1PCENR & rcc_pb1_wwdg) != 0u; }
    static constexpr Irq irq() { return Irq::wwdg; }

    static bool enabled() { return (wwdg()->CTLR & wwdg_wdga) != 0u; }
    /// T[6:0] as it stands - a live read of a free-running counter.
    static uint8_t counter() { return static_cast<uint8_t>(wwdg()->CTLR & 0x7Fu); }
    static WwdgPrescaler prescaler() { return static_cast<WwdgPrescaler>((wwdg()->CFGR >> 7) & 3u); }
    static uint8_t window() { return static_cast<uint8_t>(wwdg()->CFGR & 0x7Fu); }
    static bool early_wakeup_enabled() { return (wwdg()->CFGR & wwdg_ewi) != 0u; }

    /// The CFGR fields. EWI is one-way, so a configuration that turns
    /// it off after one that turned it on is written and does not take.
    static bool configure(const WwdgConfig& cfg) {
        if (!wwdg_config_valid(cfg)) {
            return false;
        }
        bus_clock(true);
        wwdg()->CFGR = static_cast<uint16_t>((static_cast<uint16_t>(cfg.prescaler) << 7) | cfg.window |
                                             (cfg.early_wakeup ? wwdg_ewi : 0u));
        return true;
    }

    /// Reload T[6:0]. T6 IS WRITTEN SET whatever the value asked for:
    /// a value below 0x40 written into the register is the "software
    /// reset" 5.2.1 describes, and force_reset() spells that on purpose.
    [[gnu::always_inline]] static void refresh(uint8_t counter_value = 0x7Fu) {
        wwdg()->CTLR = static_cast<uint16_t>((counter_value & 0x7Fu) | wwdg_t6);
    }
    /// WDGA up, with the first count. One way: only a reset clears it.
    static void start(uint8_t counter_value = 0x7Fu) {
        bus_clock(true);
        wwdg()->CTLR = static_cast<uint16_t>(wwdg_wdga | (counter_value & 0x7Fu) | wwdg_t6);
    }
    /// T6 written zero with WDGA set: the reset, now.
    static void force_reset() { wwdg()->CTLR = wwdg_wdga; }

    /// EWIF: raised at 0x40 whether or not EWI is set (5.3.3), cleared
    /// by writing zero, and a software set attempt lands nowhere
    /// (measured, as 5.3.3 says).
    static bool flag() { return (wwdg()->STATR & wwdg_ewif) != 0u; }
    static void clear_flag() { wwdg()->STATR = 0u; }

    /// The ISR body: true when the early warning fired (and clears it).
    [[gnu::always_inline]] static bool isr() {
        if (!flag()) {
            return false;
        }
        clear_flag();
        return true;
    }

    /// DBGMCU_CR.WWDG_STOP (RM 21.2.1).
    static bool debug_freeze() { return (Iwdg::dbgmcu_cr() & (1UL << 9)) != 0u; }
    static void debug_freeze(bool on) {
        if (on) {
            asm volatile("csrs 0x7C0, %0" ::"r"(1UL << 9));
        } else {
            asm volatile("csrc 0x7C0, %0" ::"r"(1UL << 9));
        }
    }
};

// The chapters' arithmetic, pinned.
static_assert(iwdg_timeout_ms(IwdgPrescaler::div4, 0x0FFF) == 128u);      // 4 x 4096 / 128 kHz
static_assert(iwdg_timeout_ms(IwdgPrescaler::div256, 0x0FFF) == 8192u);   // the longest: 8.2 s
static_assert(iwdg_timeout_ms(IwdgPrescaler::div32, 999, 124'000UL) == 258u);
static_assert(!iwdg_config_valid(IwdgConfig{.reload = 0x1000}));
static_assert(wwdg_step_cycles(WwdgPrescaler::div8) == 32768u);
static_assert(wwdg_timeout_us(48'000'000UL, WwdgPrescaler::div1, 0x7F) == 5461u);   // 64 x 4096 / 48
static_assert(wwdg_timeout_us(48'000'000UL, WwdgPrescaler::div8, 0x7F) == 43690u);
static_assert(!wwdg_config_valid(WwdgConfig{.window = 0x3F}));
static_assert(wwdg_config_valid(WwdgConfig{.window = 0x40}));

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
