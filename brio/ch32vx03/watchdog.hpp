/*
 * watchdog.hpp
 *
 * The CH32V203's two watchdogs: the INDEPENDENT one (RM ch. 7), a
 * twelve-bit down-counter on the LSI that nothing but a reset stops,
 * and the WINDOW one (ch. 8), a seven-bit down-counter on PCLK1 that
 * resets the chip both when it is refreshed too LATE and when it is
 * refreshed too EARLY.
 *
 * WHY THIS IS ITS OWN FILE AND NOT reset.hpp's. The two chapters are
 * reset SOURCES, and reset.hpp reads their flags (ResetFlag::
 * independent_watchdog and ::window_watchdog) - but they are two
 * peripherals with their own registers, their own clocks, their own
 * arithmetic and, for the window one, its own vector; reset.hpp is the
 * PLATFORM's failing half (the reset causes, the software reset, the
 * fault body) and includes kernel/panic.hpp for it. Keeping the
 * watchdogs here leaves that file about the core and this one about
 * two blocks, and a program that wants neither pays for neither.
 *
 * WHAT MAKES THEM DIFFERENT, IN ONE PARAGRAPH EACH.
 *
 * THE INDEPENDENT WATCHDOG has no enable bit and no clock gate: its
 * registers answer out of reset, a key value written into IWDG_CTLR is
 * the only way in (0x5555 opens PSCR and RLDR, 0xCCCC starts it,
 * 0xAAAA refreshes it), and once started NOTHING IN SOFTWARE STOPS IT
 * - only a reset does. Its clock is the LSI, which the silicon FORCES
 * ON when the watchdog starts (3.3.5.4) whatever the program did with
 * LSION: that is also how a program can tell the watchdog is running,
 * there being no status bit that says so. And the LSI is an
 * uncalibrated RC - device::lsi_min_hz..lsi_max_hz, a spread of better
 * than two to one on this part - so a time-out computed from its
 * nominal rate is a nominal time-out. Every arithmetic helper here
 * therefore takes the LSI rate as an ARGUMENT: the caller decides
 * whether it is asking about the fast corner, the slow one or a
 * measured rate.
 *
 * ITS REGISTERS NEED THAT CLOCK TO BE RUNNING, which the chapter never
 * says and the bench does: with the LSI stopped, a prescaler or reload
 * written behind the unlock key NEVER ARRIVES - PVU and RVU stand set
 * for ever, because the update they report is a crossing into the LSI's
 * own domain, and a read of either register is invalid while they do
 * (7.3.2, 7.3.3). Measured: the registers keep their reset values and
 * the flags never clear. So the order a program writes them in is not
 * free - `arm()` STARTS THE WATCHDOG FIRST, which forces the LSI on,
 * and only then writes the setting, which is why the reset value (0x0FFF
 * at /4, some four hundred milliseconds at the nominal rate) is the
 * budget it has to get there. A program that wants to prepare the
 * registers without committing starts the LSI itself first
 * (`Rcc::lsi_start()`).
 *
 * THE WINDOW WATCHDOG is a PB1 peripheral with a clock gate, an
 * early-wake-up interrupt on its own vector one tick before the reset,
 * and TWO ways to be wrong - a refresh after the counter has fallen out
 * of its window's bottom (0x3F, the reset) and a refresh before it has
 * fallen INTO the window (also the reset). Its tick is PCLK1 / 4096 /
 * 2^WDGTB, so its time-outs move with the clock tree and are
 * milliseconds at most, where the independent one reaches seconds.
 *
 * TWO OF THIS CHAPTER'S SENTENCES ARE NOT WHAT THE SILICON DOES, and
 * both are measured. ITS COUNTER DOES NOT FREE-RUN: 8.2.1 says it
 * counts down "no matter whether the watchdog function is enabled or
 * not", and with WDGA clear it does not move at all - it holds what the
 * last write put in it, and starts falling when WDGA is set. (The
 * CH32V00x's block behaves the same way against the same sentence, so
 * this is WCH's design and not this part's accident.) AND ITS CLOCK
 * GATE DOES NOT SILENCE THE REGISTERS: with RCC's WWDGEN clear, CTLR
 * and CFGR still read their reset values and still take writes - what
 * the gate stops is the COUNTER, which is what 8.2.1 offers it for.
 *
 * WHAT IS NOT HERE. The option byte that starts the independent
 * watchdog at every boot without software (IWDG_SW): the option bytes
 * are the flash chapter's. The debug module's freeze bits (ch. 34),
 * which stop either counter while a probe has the core halted: the
 * debug chapter's. Neither is reachable through this file, and a
 * suite that measures a time-out states which of them it did not
 * touch.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"

namespace brio {

// =============================================================================
// The independent watchdog (RM ch. 7)
// =============================================================================

/// Table 7-1: four sixteen-bit registers, on a four-byte stride.
struct IwdgRegs {
    volatile uint16_t CTLR;    uint16_t RESERVED0;   ///< 0x00 the key, write-only
    volatile uint16_t PSCR;    uint16_t RESERVED1;   ///< 0x04 the prescaler
    volatile uint16_t RLDR;    uint16_t RESERVED2;   ///< 0x08 the reload value
    volatile uint16_t STATR;   uint16_t RESERVED3;   ///< 0x0c PVU and RVU, read-only
};

inline IwdgRegs* iwdg_regs() { return reinterpret_cast<IwdgRegs*>(pb1_base + 0x3000); }

/// IWDG_PSCR's three bits (7.3.2). Both 110 and 111 divide by 256,
/// which is why the enumerators stop at one of them.
enum class IwdgPrescaler : uint8_t {
    div4 = 0, div8 = 1, div16 = 2, div32 = 3, div64 = 4, div128 = 5, div256 = 6,
};

/// 4 << code - the divider a prescaler field means.
constexpr uint32_t iwdg_prescaler_divider(IwdgPrescaler p) {
    const uint8_t code = static_cast<uint8_t>(p) > 6u ? 6u : static_cast<uint8_t>(p);
    return 4UL << code;
}

/// IWDG_STATR's two bits (7.3.4): an update of the prescaler or of the
/// reload value crossing into the LSI's own domain.
inline constexpr uint16_t iwdg_pvu = 1u << 0;
inline constexpr uint16_t iwdg_rvu = 1u << 1;

/**
 * The nominal time-out of a (prescaler, reload) pair in MICROSECONDS,
 * at a STATED LSI rate: the counter is reloaded with `reload` and
 * counts down to zero, so the period is `reload + 1` prescaled LSI
 * ticks (7.2.1's own sentence).
 *
 * The rate is an argument and not a constant of this file, because the
 * LSI is an uncalibrated RC: device::lsi_min_hz gives the LONGEST
 * time-out this part can produce for a setting and device::lsi_max_hz
 * the shortest, and a program that must not be reset early computes
 * its refresh interval from the SHORTEST.
 */
constexpr uint32_t iwdg_timeout_us(IwdgPrescaler p, uint16_t reload, uint32_t lsi_hz) {
    if (lsi_hz == 0u) {
        return 0;
    }
    const uint64_t ticks = static_cast<uint64_t>(reload & 0x0FFFu) + 1u;
    const uint64_t us = ticks * iwdg_prescaler_divider(p) * 1000000ULL / lsi_hz;
    return us > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : static_cast<uint32_t>(us);
}

/// The same in milliseconds, for the range where microseconds overflow
/// nothing but the reader's patience.
constexpr uint32_t iwdg_timeout_ms(IwdgPrescaler p, uint16_t reload, uint32_t lsi_hz) {
    if (lsi_hz == 0u) {
        return 0;
    }
    const uint64_t ticks = static_cast<uint64_t>(reload & 0x0FFFu) + 1u;
    return static_cast<uint32_t>(ticks * iwdg_prescaler_divider(p) * 1000ULL / lsi_hz);
}

/// The smallest reload value whose time-out REACHES `us` at that LSI
/// rate - "at least", never early, the direction every waiting verb in
/// brio takes - or 0xFFFF when this prescaler cannot reach it at all
/// (the field is twelve bits).
constexpr uint16_t iwdg_reload_for(IwdgPrescaler p, uint32_t us, uint32_t lsi_hz) {
    if (lsi_hz == 0u) {
        return 0xFFFFu;
    }
    const uint64_t tick_us_num = static_cast<uint64_t>(iwdg_prescaler_divider(p)) * 1000000ULL;
    // ticks = ceil(us * lsi_hz / (divider * 1e6))
    const uint64_t ticks =
        (static_cast<uint64_t>(us) * lsi_hz + tick_us_num - 1u) / tick_us_num;
    if (ticks == 0u) {
        return 0;
    }
    if (ticks > 4096u) {
        return 0xFFFFu;
    }
    return static_cast<uint16_t>(ticks - 1u);
}

/// A whole IWDG setting. The reset values are the silicon's own
/// (0x0FFF at /4), which at the nominal LSI of this part is a little
/// over four hundred milliseconds. There is no window register in this
/// chapter, so this is the whole of it.
struct IwdgConfig {
    IwdgPrescaler prescaler = IwdgPrescaler::div4;
    /// IWDG_RLDR.RL, twelve bits: the value loaded on every refresh. A
    /// reload of zero is legal and is the shortest time-out the part
    /// has - what force_reset() programs on purpose.
    uint16_t reload = 0x0FFFu;
};

constexpr bool iwdg_config_valid(const IwdgConfig& c) {
    return static_cast<uint8_t>(c.prescaler) <= 6u && c.reload <= 0x0FFFu;
}

/**
 * The independent watchdog, monostate.
 *
 *   brio::Iwdg::configure({.prescaler = brio::IwdgPrescaler::div32,
 *                          .reload = 1250});     // about a second, nominal
 *   brio::Iwdg::start();                         // no way back but a reset
 *   ...
 *   brio::Iwdg::refresh();                       // in the program's loop
 *
 * ONE WAY IN SOFTWARE. start() commits the board to being reset unless
 * refresh() keeps arriving; nothing here stops it again, because the
 * silicon has nothing that does.
 */
struct Iwdg {
    Iwdg() = delete;

    /// The three key values (7.3.1). Any other write to IWDG_CTLR does
    /// nothing, and writing one of these RE-LOCKS the two protected
    /// registers unless it is the unlock key itself.
    static constexpr uint16_t key_refresh = 0xAAAAu;
    static constexpr uint16_t key_unlock = 0x5555u;
    static constexpr uint16_t key_start = 0xCCCCu;

    /// This peripheral has NO clock gate: it answers out of reset, and
    /// the LSI it counts is forced on when it starts (3.3.5.4).
    static IwdgRegs& regs() { return *iwdg_regs(); }

    /// Reload the counter from RLDR - the verb a program calls in its
    /// loop. It also RE-LOCKS PSCR and RLDR, which is why configure()
    /// unlocks each time.
    [[gnu::always_inline]] static void refresh() { regs().CTLR = key_refresh; }

    /// Open the write window on PSCR and RLDR.
    static void unlock() { regs().CTLR = key_unlock; }

    /// Start the watchdog. See the type's own comment: there is no way
    /// back that is not a reset.
    static void start() { regs().CTLR = key_start; }

    /**
     * The prescaler and the reload value, with the unlock key before
     * them. False - and nothing written - for a code the fields cannot
     * hold.
     *
     * The two writes cross into the LSI's clock domain and STATR's PVU
     * and RVU stand while they do; 7.3.4's own closing note says a
     * program need not wait for them before carrying on, and only a
     * READ of either register needs the flag clear, which is what
     * prescaler() and reload() do.
     *
     * THE LSI MUST BE RUNNING or neither write arrives at all and both
     * flags stand for ever (the file header): either the watchdog is
     * already started, or the program started the oscillator itself.
     * This verb writes what it was given and does not start anything -
     * arm() is the verb that gets the order right.
     *
     * AND IT LEAVES THE WRITE WINDOW OPEN. The unlock stands until
     * ANOTHER key value is written (7.3.1), so after this verb a plain
     * store into PSCR or RLDR still lands - measured. refresh() is what
     * closes it again, which is why arm() ends with one.
     */
    static bool configure(const IwdgConfig& c) {
        if (!iwdg_config_valid(c)) {
            return false;
        }
        IwdgRegs& r = regs();
        r.CTLR = key_unlock;
        r.PSCR = static_cast<uint16_t>(c.prescaler);
        r.RLDR = static_cast<uint16_t>(c.reload & 0x0FFFu);
        return true;
    }

    /// The compile-time twin: a setting written as a constant is
    /// REFUSED AT COMPILE TIME rather than at run time, which is where
    /// a watchdog's configuration usually comes from.
    template <IwdgConfig c>
    static bool configure() {
        static_assert(iwdg_config_valid(c),
                      "brio Iwdg: the prescaler field takes codes 0..6 (divide by 4 up to "
                      "256) and the reload is twelve bits (RM 7.3.2, 7.3.3)");
        return configure(c);
    }

    /**
     * Configure and start in one verb, the commonest use - and THE
     * ORDER IS THE POINT: the start key goes first, because it is what
     * forces the LSI on, and PSCR and RLDR take a write only while that
     * oscillator runs (the file header). The watchdog therefore runs on
     * its RESET setting (0x0FFF at /4) for as long as the two writes
     * take, which is four hundred milliseconds of budget for a handful
     * of stores.
     *
     * A caller that wants the registers set BEFORE committing starts
     * the LSI itself (`Rcc::lsi_start()`), calls configure(), and then
     * start().
     */
    static bool arm(const IwdgConfig& c) {
        if (!iwdg_config_valid(c)) {
            return false;
        }
        start();
        if (!configure(c)) {
            return false;
        }
        refresh();
        return true;
    }

    /// STATR, and the two waits over it. A read of PSCR or RLDR while
    /// its update flag stands is invalid (7.3.2, 7.3.3), so the
    /// readers below wait first - a bounded wait, five prescaled LSI
    /// periods being the manual's own worst case.
    static uint16_t status() { return regs().STATR; }
    static bool busy(uint16_t mask = iwdg_pvu | iwdg_rvu) { return (status() & mask) != 0u; }
    static bool wait_idle(uint16_t mask = iwdg_pvu | iwdg_rvu,
                          uint32_t turns = clock_timeout_turns) {
        for (uint32_t i = 0; i < turns; ++i) {
            if (!busy(mask)) {
                return true;
            }
        }
        return !busy(mask);
    }

    static IwdgPrescaler prescaler() {
        (void)wait_idle(iwdg_pvu);
        return static_cast<IwdgPrescaler>(regs().PSCR & 0x7u);
    }
    static uint16_t reload() {
        (void)wait_idle(iwdg_rvu);
        return static_cast<uint16_t>(regs().RLDR & 0x0FFFu);
    }

    /**
     * Is it running? There is no bit that says so - but the silicon
     * forces the LSI on when the watchdog starts and the LSI cannot
     * then be switched off (3.3.5.4), so an LSI that is READY while
     * nothing in the program asked for it is the watchdog's own doing.
     * A program that runs the LSI for its own reasons (an RTC on it, a
     * measurement) cannot ask this question, and the answer says so by
     * being false.
     */
    static bool running() { return Rcc::lsi_ready() && !Rcc::lsi_enabled(); }

    /// The time-out of what is IN the registers now, at a stated LSI
    /// rate - the witness for a configure() the program did not make
    /// itself.
    static uint32_t timeout_us(uint32_t lsi_hz) {
        return iwdg_timeout_us(prescaler(), reload(), lsi_hz);
    }

    /// Reset the board through this watchdog, as fast as it can: the
    /// shortest setting there is (one prescaled tick at /4), started
    /// and not refreshed. At the nominal LSI of this part that is a
    /// hundred microseconds and never returns. The start comes first
    /// for arm()'s reason.
    [[noreturn]] static void force_reset() {
        start();
        (void)configure({.prescaler = IwdgPrescaler::div4, .reload = 0});
        refresh();
        for (;;) {
        }
    }
};

// =============================================================================
// The window watchdog (RM ch. 8)
// =============================================================================

/// Table 8-1: three sixteen-bit registers on a four-byte stride.
struct WwdgRegs {
    volatile uint16_t CTLR;   uint16_t RESERVED0;   ///< 0x00 WDGA and the counter
    volatile uint16_t CFGR;   uint16_t RESERVED1;   ///< 0x04 EWI, WDGTB, the window
    volatile uint16_t STATR;  uint16_t RESERVED2;   ///< 0x08 EWIF, write zero to clear
};

inline WwdgRegs* wwdg_regs() { return reinterpret_cast<WwdgRegs*>(pb1_base + 0x2c00); }

inline constexpr uint16_t wwdg_t_mask   = 0x7Fu;      ///< CTLR's counter
inline constexpr uint16_t wwdg_wdga     = 1u << 7;    ///< CTLR's activation bit
inline constexpr uint16_t wwdg_w_mask   = 0x7Fu;      ///< CFGR's window
inline constexpr uint16_t wwdg_wdgtb_mask  = 3u << 7;
inline constexpr uint8_t wwdg_wdgtb_shift  = 7;
inline constexpr uint16_t wwdg_ewi      = 1u << 9;    ///< CFGR's early-wake-up enable
inline constexpr uint16_t wwdg_ewif     = 1u << 0;    ///< STATR's flag, rc_w0

/// The counter value at which the watchdog resets: the reset happens
/// when T6 falls, that is on the step from 0x40 to 0x3F (8.2.1).
inline constexpr uint8_t wwdg_floor = 0x3Fu;

/// CFGR's WDGTB (8.3.2): the tick is PCLK1 / 4096 divided again by
/// one, two, four or eight.
enum class WwdgPrescaler : uint8_t { div1 = 0, div2 = 1, div4 = 2, div8 = 3 };

/// PCLK1 cycles per decrement of the seven-bit counter.
constexpr uint32_t wwdg_cycles_per_tick(WwdgPrescaler p) {
    return 4096UL << static_cast<uint8_t>(p);
}

/// The tick rate itself, for a program that wants to state the window
/// in its own units.
constexpr uint32_t wwdg_tick_hz(uint32_t pclk1_hz, WwdgPrescaler p) {
    const uint32_t cycles = wwdg_cycles_per_tick(p);
    return cycles == 0u ? 0u : pclk1_hz / cycles;
}

/**
 * How long, in microseconds, from a refresh with counter value
 * `counter` until the reset - figure 8-2's own formula: the counter
 * falls from T to 0x3F, which is `T[5:0] + 1` ticks because the sixth
 * bit is what the reset watches.
 */
constexpr uint32_t wwdg_timeout_us(uint32_t pclk1_hz, WwdgPrescaler p, uint8_t counter) {
    if (pclk1_hz == 0u) {
        return 0;
    }
    const uint64_t ticks = static_cast<uint64_t>(counter & 0x3Fu) + 1u;
    const uint64_t us = ticks * wwdg_cycles_per_tick(p) * 1000000ULL / pclk1_hz;
    return us > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : static_cast<uint32_t>(us);
}

/**
 * How long a refresh must WAIT after one with counter value `counter`
 * before the window opens: the counter has to fall from T to W, so
 * `counter - window` ticks. Zero when the window is at or above the
 * counter - the setting in which a refresh is legal at once.
 */
constexpr uint32_t wwdg_window_wait_us(uint32_t pclk1_hz, WwdgPrescaler p, uint8_t counter,
                                       uint8_t window) {
    if (pclk1_hz == 0u || (window & wwdg_w_mask) >= (counter & wwdg_t_mask)) {
        return 0;
    }
    const uint64_t ticks = static_cast<uint64_t>((counter & wwdg_t_mask) - (window & wwdg_w_mask));
    const uint64_t us = ticks * wwdg_cycles_per_tick(p) * 1000000ULL / pclk1_hz;
    return us > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : static_cast<uint32_t>(us);
}

/// A whole WWDG setting: CFGR's fields. The counter itself is not here
/// - it is written by start() and refresh(), being the register the
/// application touches at run time.
struct WwdgConfig {
    WwdgPrescaler prescaler = WwdgPrescaler::div1;
    /// CFGR's W: the high edge of the refresh window. 0x7F (the reset
    /// value) means "refresh whenever you like".
    uint8_t window = 0x7Fu;
    /// CFGR's EWI: an interrupt when the counter reaches 0x40, one tick
    /// before the reset. ONE-WAY like WDGA - 8.3.2 says it is cleared
    /// only by a reset.
    bool early_wakeup = false;
};

/// Seven-bit fields, and a window below 0x40 can never be served: a
/// refresh is legal only while the counter is at or below W AND above
/// 0x3F (8.2.1), so W < 0x40 leaves no legal instant at all.
constexpr bool wwdg_config_valid(const WwdgConfig& c) {
    return c.window <= wwdg_t_mask && c.window > wwdg_floor;
}

/**
 * The window watchdog, monostate.
 *
 *   brio::Wwdg::bus_clock(true);
 *   brio::Wwdg::configure({.prescaler = brio::WwdgPrescaler::div8,
 *                          .window = 0x50});
 *   brio::Wwdg::start(0x7F);
 *   ...
 *   if (brio::Wwdg::in_window()) { brio::Wwdg::refresh(); }
 *
 * Like the independent watchdog, WDGA is one-way in software (8.3.1) -
 * but unlike it, this block has a CLOCK GATE, and closing that gate
 * stops the counter, which 8.2.1 names as the way to suspend it.
 */
struct Wwdg {
    Wwdg() = delete;

    /// RCC_PB1PCENR's WWDGEN, clear at reset. What it gates is the
    /// COUNTER and not the register file: with it closed CTLR and CFGR
    /// still read and still take writes (measured), which is why
    /// closing it is 8.2.1's own way to suspend a watchdog that cannot
    /// otherwise be stopped.
    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb1, rcc_pb1_wwdg);
        } else {
            Rcc::disable(Bus::pb1, rcc_pb1_wwdg);
        }
    }
    static bool bus_clock() { return Rcc::enabled(Bus::pb1, rcc_pb1_wwdg); }

    /// Pulse the block's reset line - the other half of what 8.2.1
    /// offers a program that wants the watchdog to stop: WDGA goes back
    /// to zero with every other bit.
    static void reset() { Rcc::reset(Bus::pb1, rcc_pb1_wwdg); }

    /// Clock on, then reset: the whole of "bring this block up".
    static void init() {
        bus_clock(true);
        reset();
    }

    /// The early-wake-up interrupt's line - this block's own vector.
    static constexpr Irq irq() { return Irq::wwdg; }

    static WwdgRegs& regs() { return *wwdg_regs(); }

    static uint16_t ctlr() { return regs().CTLR; }
    static uint16_t cfgr() { return regs().CFGR; }

    /// WDGA. Set by software, cleared only by a reset (8.3.1).
    static bool enabled() { return (regs().CTLR & wwdg_wdga) != 0u; }
    /// T[6:0] as it stands. It only MOVES while WDGA is set: this
    /// block's counter does not free-run against 8.2.1's own sentence
    /// (the file header), so a program timing it arms the watchdog and
    /// refreshes it.
    static uint8_t counter() { return static_cast<uint8_t>(regs().CTLR & wwdg_t_mask); }
    static WwdgPrescaler prescaler() {
        return static_cast<WwdgPrescaler>((regs().CFGR & wwdg_wdgtb_mask) >> wwdg_wdgtb_shift);
    }
    static uint8_t window() { return static_cast<uint8_t>(regs().CFGR & wwdg_w_mask); }
    static bool early_wakeup_enabled() { return (regs().CFGR & wwdg_ewi) != 0u; }

    /// Whether a refresh would be legal RIGHT NOW: the counter at or
    /// below the window and above 0x3F (8.2.1). A snapshot of a moving
    /// counter and therefore a hint, not a lock - a caller racing the
    /// window arranges its timing and does not poll this.
    static bool in_window() {
        const uint8_t t = counter();
        return t > wwdg_floor && t <= window();
    }

    /**
     * CFGR: the prescaler, the window and the early-wake-up enable.
     * False - and nothing written - for a window no counter value can
     * ever be inside.
     *
     * EWI is one-way (8.3.2), so a configure() with it false does not
     * clear one that is already set; what this verb writes is the
     * union, and the readers say what stands.
     */
    static bool configure(const WwdgConfig& c) {
        if (!wwdg_config_valid(c)) {
            return false;
        }
        uint16_t v = static_cast<uint16_t>(c.window & wwdg_w_mask);
        v = static_cast<uint16_t>(
            v | ((static_cast<uint16_t>(c.prescaler) << wwdg_wdgtb_shift) & wwdg_wdgtb_mask));
        if (c.early_wakeup) {
            v = static_cast<uint16_t>(v | wwdg_ewi);
        }
        regs().CFGR = v;
        return true;
    }

    /// The compile-time twin, for a window written as a constant.
    template <WwdgConfig c>
    static bool configure() {
        static_assert(wwdg_config_valid(c),
                      "brio Wwdg: the window is seven bits and must be ABOVE 0x3F - the "
                      "counter resets at that value, so a lower window has no legal "
                      "refresh instant in it at all (RM 8.2.1)");
        return configure(c);
    }

    /**
     * Refresh: write the counter, keeping WDGA as it stands. The
     * value's low six bits are what the time-out is counted from; bit 6
     * must be set, and a value with it clear IS the reset (8.2.1's own
     * note about a software reset through T6).
     */
    [[gnu::always_inline]] static void refresh(uint8_t counter_value = wwdg_t_mask) {
        WwdgRegs& r = regs();
        r.CTLR = static_cast<uint16_t>((r.CTLR & wwdg_wdga) | (counter_value & wwdg_t_mask));
    }

    /// Start the watchdog with that counter value: WDGA and T in one
    /// store, which is the only way to start it - there being no
    /// sequence in this chapter.
    static void start(uint8_t counter_value = wwdg_t_mask) {
        regs().CTLR = static_cast<uint16_t>(wwdg_wdga | (counter_value & wwdg_t_mask));
    }

    /// Reset the board through this watchdog: WDGA with a counter of
    /// zero, which is T6 clear and therefore the reset itself. Never
    /// returns.
    [[noreturn]] static void force_reset() {
        regs().CTLR = wwdg_wdga;
        for (;;) {
        }
    }

    /// EWIF. Raised when the counter reaches 0x40 whether or not the
    /// interrupt is enabled (8.3.3), which is what lets a disarmed
    /// block be timed. rc_w0: cleared by writing zero into it.
    static bool flag() { return (regs().STATR & wwdg_ewif) != 0u; }
    static void clear_flag() { regs().STATR = static_cast<uint16_t>(~wwdg_ewif); }

    /**
     * The vector's BODY: true when the early wake-up was this
     * peripheral's doing, the flag cleared. An application binds it -
     *
     *     extern "C" BRIO_CH32_INTERRUPT void wwdg_handler() {
     *         if (brio::Wwdg::isr()) { brio::Wwdg::refresh(); }
     *     }
     *
     * - and has ONE TICK to act: 8.2.1 gives the interrupt a single
     * counter clock cycle before the reset.
     */
    [[gnu::always_inline]] static bool isr() {
        if (!flag()) {
            return false;
        }
        clear_flag();
        return true;
    }

    /// The time-out of what is IN the registers now, given PCLK1 - the
    /// witness for a configuration the program did not make itself.
    static uint32_t timeout_us(uint32_t pclk1_hz, uint8_t counter_value = wwdg_t_mask) {
        return wwdg_timeout_us(pclk1_hz, prescaler(), counter_value);
    }
};

} // namespace brio
