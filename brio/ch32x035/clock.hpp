/*
 * clock.hpp
 *
 * The main clock of the CH32X035: where HCLK comes from and what its rate
 * is, as ONE compile-time truth every driver of this stratum derives from
 * (docs/design/clock.md; no F_CPU here either).
 *
 * THE TREE, IN ONE PARAGRAPH (RM 3.3). There is ONE root: the 48 MHz
 * internal RC (HSI), on out of reset, factory-calibrated through HSICAL
 * and nudged by a five-bit HSITRIM. There is no PLL, no crystal
 * oscillator, no low-speed RC and no clock switch: SYSCLK IS the HSI, and
 * HCLK is SYSCLK divided by HPRE - whose reset value is /6, which is why
 * the chip wakes up at 8 MHz. There is no peripheral-bus prescaler
 * either: every block, the four USARTs included, runs at HCLK (14.3), so
 * `pclk_hz` is `hz`. The auto-wakeup's and the independent watchdog's
 * clock is the HSI divided by 1024, some 47 kHz, and the USB blocks take
 * their 48 MHz straight from the HSI - neither is a rate this file sets.
 *
 * WHAT init() ORDERS. The HSI is asked for (it is on out of reset, and a
 * program that stopped it wants it back), then the flash's wait states
 * are raised to what the faster of the two rates needs, then HPRE, then
 * the wait states are set to the new rate's (RM 20.3.1: 0 waits to 12
 * MHz, 1 to 24, 2 to 48). WCH's own clock setup does the same with two
 * waits as the ceiling (system_ch32x035.c in the EVT, read as the
 * vendor's sequence and not copied).
 *
 * ONE NOTE THIS FILE DOES NOT ACT ON. RM 3.4.2 closes HPRE's description
 * with "when the prescaler factor of the HB clock source is greater than
 * 1, the prefetch buffer must be turned on" - and no register of this
 * manual has a prefetch-buffer bit: FLASH_ACTLR holds LATENCY alone
 * (20.3.1), and the vendor's clock setup divides HCLK without touching
 * anything else. The sentence is recorded (docs/ch32x035/clock.md), and
 * what a divided HCLK does is the clock suite's measurement.
 *
 * THE RESOURCE UNDER THE TASKS. `Rcc` is the block itself: the HSI and
 * its trim, the divider as it stands, the MCO output, the reset flags
 * RCC_RSTSCKR holds, and the per-peripheral clock enables and resets on
 * the three buses. Every configuring driver of the stratum opens its own
 * enable through it.
 *
 * THE RUNTIME REGIME. `DynamicClock<Boot, Users...>` is the CH32V00x's
 * shape, and the simplest there is: one root, one divider, and
 * set<hz>() / set(hz) that name the NEW RATE, fan it out to the users
 * synchronously and in list order, and then move the divider - with the
 * flash wait states raised before a rise and lowered after a fall. The
 * discrete-rate surface (rate_count, rate_hz, rate_index) is what a delay
 * table indexes by.
 */

#pragma once

#include <stdint.h>

#include <concepts>

#include "ch32x035/device.hpp"
#include "ch32x035/pin.hpp"
#include "util/clock.hpp"

namespace brio {

/// Where SYSCLK comes from: the HSI, and nothing else on this series.
enum class ClockSource : uint8_t {
    internal,   ///< HSI, the 48 MHz internal RC
};

inline constexpr uint32_t sysclk_max_hz = 48'000'000UL;

/// RM 3.4.2, HPRE[3:0]: codes 0..7 divide by 1..8, codes 8..15 divide by
/// 2, 4, 8, 16, 32, 64, 128, 256. The two halves overlap on 2, 4 and 8;
/// hpre_for() below picks the low code there, so a divider has one
/// spelling.
constexpr uint32_t hpre_divider(uint8_t code) {
    return code < 8u ? static_cast<uint32_t>(code) + 1u
                     : 2UL << (code - 8u);
}

/// The HPRE code that divides `src_hz` to exactly `hz`, or 0xFF when no
/// divider does.
constexpr uint8_t hpre_for(uint32_t src_hz, uint32_t hz) {
    for (uint8_t code = 0; code < 16u; ++code) {
        const uint32_t div = hpre_divider(code);
        if (hz != 0u && src_hz / div == hz && src_hz % div == 0u) {
            return code;
        }
    }
    return 0xFF;
}

/// How long the HSI may take before a verb gives up. Bounded spins, not
/// while(1): a boot that cannot reach its rate must return and let the
/// program say so (the datasheet's table 3-9 gives the HSI 3.5 us).
inline constexpr uint32_t clock_timeout_turns = 100'000UL;

/// Which bus a peripheral's enable and reset bits sit on (RM 3.4.3..3.4.9).
enum class Bus : uint8_t { hb, pb2, pb1 };

/// What the clock output carries (RM 3.4.2, MCO[2:0]): SYSCLK or the HSI,
/// and every other code no clock at all.
enum class McoSource : uint8_t { none = 0, sysclk = 4, hsi = 5 };

/**
 * The RCC block, monostate: the root, the divider, the output, the reset
 * flags and the peripheral gates. Verbs read and write the registers as
 * they stand; the policy - which rate - is the task's below.
 */
struct Rcc {
    Rcc() = delete;

    // ---- the peripheral gates --------------------------------------------
    static void enable(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  rcc()->AHBPCENR |= mask; break;
            case Bus::pb2: rcc()->APB2PCENR |= mask; break;
            case Bus::pb1: rcc()->APB1PCENR |= mask; break;
        }
    }

    static void disable(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  rcc()->AHBPCENR &= ~mask; break;
            case Bus::pb2: rcc()->APB2PCENR &= ~mask; break;
            case Bus::pb1: rcc()->APB1PCENR &= ~mask; break;
        }
    }

    static bool enabled(Bus bus, uint32_t mask) {
        switch (bus) {
            case Bus::hb:  return (rcc()->AHBPCENR & mask) == mask;
            case Bus::pb2: return (rcc()->APB2PCENR & mask) == mask;
            case Bus::pb1: return (rcc()->APB1PCENR & mask) == mask;
        }
        return false;
    }

    /// Pulse a peripheral's reset line: held, then released, which is how
    /// a driver puts its block back to its reset state without touching
    /// anyone else's. On the HB bus the mask is RCC_AHBRSTR's, whose bits
    /// are the USBFS's, the PIOC's and the USB PD's alone (3.4.9). THE
    /// POWER CONTROLLER'S LINE IS REFUSED (false, nothing written): a
    /// pulse on PWRRST left a CH32X035F8U6 unreachable by its debug port
    /// until its supply was cycled - observed once, with the pulse the
    /// last thing the program did before its console went silent, and not
    /// measured again on purpose, because the way back is a hand on the
    /// board (docs/ch32x035/clock.md).
    static bool reset(Bus bus, uint32_t mask) {
        if (bus == Bus::pb1 && (mask & rcc_pb1_pwr) != 0u) {
            return false;
        }
        switch (bus) {
            case Bus::hb:  rcc()->AHBRSTR |= mask;   rcc()->AHBRSTR &= ~mask;   break;
            case Bus::pb2: rcc()->APB2PRSTR |= mask; rcc()->APB2PRSTR &= ~mask; break;
            case Bus::pb1: rcc()->APB1PRSTR |= mask; rcc()->APB1PRSTR &= ~mask; break;
        }
        return true;
    }

    // ---- the root ----------------------------------------------------------
    static bool hsi_on() { return (rcc()->CTLR & rcc_hsion) != 0u; }
    static bool hsi_ready() { return (rcc()->CTLR & rcc_hsirdy) != 0u; }

    /// Start the HSI and wait a bounded time for HSIRDY. It is on out of
    /// reset, so this is usually a readback - but a program that stopped
    /// it must be able to get it back: it is the only root there is.
    static bool hsi_start() {
        rcc()->CTLR |= rcc_hsion;
        for (uint32_t i = 0; i < clock_timeout_turns; ++i) {
            if ((rcc()->CTLR & rcc_hsirdy) != 0u) {
                return true;
            }
        }
        return false;
    }

    /// The HSI's user trim, five bits around a centre of 16, ADDED to the
    /// factory calibration (3.4.1: "about 110 kHz per step").
    static void hsi_trim(uint8_t trim) {
        rcc()->CTLR = (rcc()->CTLR & ~rcc_hsitrim_mask) |
                      ((static_cast<uint32_t>(trim) & 0x1Fu) << 3);
    }
    static uint8_t hsi_trim() {
        return static_cast<uint8_t>((rcc()->CTLR & rcc_hsitrim_mask) >> 3);
    }
    /// The factory calibration the hardware loaded at reset (read-only).
    static uint8_t hsi_calibration() {
        return static_cast<uint8_t>((rcc()->CTLR & rcc_hsical_mask) >> 8);
    }

    // ---- the divider as it stands -----------------------------------------
    static uint8_t hpre_code() {
        return static_cast<uint8_t>((rcc()->CFGR0 & rcc_hpre_mask) >> rcc_hpre_shift);
    }
    static void hpre(uint8_t code) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_hpre_mask) |
                       (static_cast<uint32_t>(code & 0xFu) << rcc_hpre_shift);
    }
    /// HCLK as the registers say it is: the HSI through the current HPRE.
    static uint32_t hclk_hz() { return hsi_hz / hpre_divider(hpre_code()); }

    // ---- the clock output --------------------------------------------------
    /// The clock output's SOURCE. The pad is the caller's (Mco below
    /// claims it): this verb only says what the multiplexer carries.
    static void mco(McoSource src) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_mco_mask) |
                       (static_cast<uint32_t>(src) << rcc_mco_shift);
    }
    static McoSource mco() {
        const uint32_t code = (rcc()->CFGR0 & rcc_mco_mask) >> rcc_mco_shift;
        return code == 4u ? McoSource::sysclk : code == 5u ? McoSource::hsi : McoSource::none;
    }

    // ---- the reset flags (RCC_RSTSCKR, 3.4.8) -------------------------------
    /// The reset causes as they ACCUMULATE: every flag a reset raised since
    /// the last clear - PORRSTF among them after a power-on, and PINRSTF
    /// too if table 3-1's reset value and not 3.4.8's bit table is the
    /// silicon's (device.hpp). The bits are the rcc_*rstf constants of
    /// device.hpp.
    static uint32_t reset_flags() { return rcc()->RSTSCKR & rcc_reset_flags; }

    /// RMVF: clear the flags, then clear RMVF itself. On this silicon the
    /// bit is a LEVEL and not a pulse (measured on a CH32X035F8U6: written
    /// 1 it reads back 1, where the CH32V203's clears itself), and a
    /// standing RMVF would clear the next reset's flags before the boot
    /// read them. What the clear leaves is what the next reset raises.
    static void clear_reset_flags() {
        rcc()->RSTSCKR = rcc()->RSTSCKR | rcc_rmvf;
        rcc()->RSTSCKR = rcc()->RSTSCKR & ~rcc_rmvf;
    }
};

/**
 * The clock output on PB9 as a task: the pad claimed and the multiplexer
 * set in one verb, so a program cannot half-do it.
 *
 *   brio::Mco::init(brio::McoSource::hsi);      // the RC on the pad
 *   brio::Mco::off();                           // the pad released
 *
 * THE PAD IS A PART FACT: PB9 is bonded on the LQFP64M, the LQFP48 and the
 * two 28-pin packages and on none of the 20-pin ones, so `has_pad` says
 * which and init() ANSWERS FALSE on a package without it rather than
 * refusing to compile - the multiplexer still exists there and
 * `Rcc::mco()` still writes it. The pad is named through its PORT and not
 * as Pin<'B', 9>, because a Pin naming a pad is formed where it is
 * written, and this header compiles on every part.
 */
struct Mco {
    Mco() = delete;

    /// Whether this package brings the output pad out at all.
    static constexpr bool has_pad = (device::port_pins('B') & (1UL << 9)) != 0u;

    /// Claim the pad and put `src` on it. False, and nothing done, on a
    /// package with no such pad.
    static bool init(McoSource src) {
        if constexpr (!has_pad) {
            (void)src;
            return false;
        } else {
            Port<'B'>::configure(9, pin_nibble_alternate);
            Rcc::mco(src);
            return true;
        }
    }

    /// Stop driving: the multiplexer to no clock, the pad back to a
    /// floating input.
    static void off() {
        Rcc::mco(McoSource::none);
        if constexpr (has_pad) {
            Port<'B'>::configure(9, pin_nibble_floating);
        }
    }

    static McoSource source() { return Rcc::mco(); }
};

/**
 * The static main clock: `hz` is the ONE compile-time truth about HCLK.
 *
 *   using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
 *   constexpr SysClock clock;
 *   SysClock::init();                // first thing in main()
 *   Serial::init(clock, 115200);     // drivers ask the tag
 *
 * `Clock<ClockSource::internal, 8'000'000>` is the reset clock named out
 * loud; any rate that is 48 MHz divided by an HPRE divider exactly is
 * legal, and one that is not is a COMPILE error, never a rounded one.
 */
template <ClockSource src, uint32_t target_hz>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = target_hz;        ///< HCLK
    static constexpr uint32_t pclk_hz = target_hz;   ///< every bus runs at HCLK here
    static constexpr bool is_static = true;

    /// SYSCLK before HPRE: the HSI, always.
    static constexpr uint32_t sysclk_hz = hsi_hz;
    static constexpr uint8_t hpre_code = hpre_for(sysclk_hz, target_hz);

    static_assert(src == ClockSource::internal,
                  "brio Clock: the CH32X035 has one root, the 48 MHz HSI");
    static_assert(hpre_code != 0xFF,
                  "brio Clock: this rate is not the 48 MHz HSI divided by an HPRE divider "
                  "(1..8, then 16, 32, 64, 128, 256) - the CH32X035 has no PLL and no crystal");
    static_assert(target_hz <= sysclk_max_hz, "HCLK must not exceed 48 MHz");

    /**
     * Take the clock to `hz`. Returns false if the HSI never reports
     * ready - the caller decides what to say about a boot that stayed on
     * the reset clock.
     */
    static bool init() {
        if (!Rcc::hsi_start()) {
            return false;
        }
        // The wait states the faster of the two rates needs, BEFORE the
        // divider moves - then the new rate's own, after it. At boot the
        // core runs at the reset rate (8 MHz, zero waits), and init() is
        // also restore(), which runs from whatever rate the program was
        // left at.
        const uint32_t now_hz = Rcc::hclk_hz();
        const uint32_t high = now_hz > target_hz ? now_hz : target_hz;
        flash_ctl()->ACTLR = flash_latency_for(high);
        Rcc::hpre(hpre_code);
        flash_ctl()->ACTLR = flash_latency_for(target_hz);
        return true;
    }

    /// After a Stop or a Standby the HSI is SYSCLK again (RM 2.3.3, 2.3.4)
    /// and the tree is put back by running init() again, which is what a
    /// sleep site would call on its way out.
    static bool restore() { return init(); }
};

/**
 * The runtime regime. Boot is a static Clock<ClockSource::internal,
 * 48'000'000> naming the root at its undivided rate; set<hz>() / set(hz)
 * name the NEW RATE (the app speaks Hz - the HPRE code that produces it
 * is this silicon's detail, resolved by hpre_for; a rate no divider
 * reaches is a compile error / a false), fan it out to Users (each a
 * ClockUser, checked where the list is written) in list order,
 * synchronously, and THEN move the divider - so a user can drain what it
 * has in flight at the old rate before adopting the new one. The flash
 * wait states are raised before a rise and lowered after a fall, so the
 * array is never read with fewer waits than its rate needs. Call set()
 * only when nothing that depends on the rate is mid-transfer.
 *
 *   using Boot = brio::Clock<brio::ClockSource::internal, 48'000'000>;
 *   using SysClock = brio::DynamicClock<Boot, brio::Ticker, Serial>;
 *   constexpr SysClock clock;
 *   SysClock::init();                 // Boot's init: 48 MHz
 *   SysClock::set<6'000'000>();       // the users rebased, then HPRE /8
 */
template <typename Boot, ClockUser... Users>
struct DynamicClock {
    static constexpr ClockSource source = Boot::source;
    static constexpr uint32_t source_hz = Boot::sysclk_hz;
    static constexpr bool is_static = false;
    static_assert(Boot::hpre_code == 0,
                  "DynamicClock: give Boot the root's undivided rate (HPRE 1); the "
                  "divider is what set() changes");

    static uint32_t hz() { return hz_; }
    /// Every bus runs at HCLK on this series.
    static uint32_t pclk_hz() { return hz_; }

    /// The DISCRETE-RATE surface: this clock only ever runs at source_hz
    /// divided by one of the sixteen HPRE codes, so per-rate arithmetic can
    /// be expanded per index at compile time, and the one runtime fact is
    /// WHICH code is current. (Three rates have two codes; the low one is
    /// the spelling hpre_for() picks.)
    static constexpr uint8_t rate_count = 16;
    static constexpr uint32_t rate_hz(uint8_t i) { return source_hz / hpre_divider(i); }
    static uint8_t rate_index() { return idx_; }

    template <typename U>
    static constexpr bool rebases = (std::same_as<U, Users> || ...);

    static bool init() {
        const bool ok = Boot::init();
        hz_ = Boot::hz;
        idx_ = 0;
        return ok;
    }

    /// After a Stop or a Standby: the root back up (Boot's init), then the
    /// CURRENT rate's divider and wait states again. The users are told
    /// nothing: their rate never changed, only the silicon forgot it.
    static bool restore() {
        const bool ok = Boot::init();
        Rcc::hpre(idx_);
        flash_ctl()->ACTLR = flash_latency_for(hz_);
        return ok;
    }

    static constexpr bool can_run_at(uint32_t hz) { return hpre_for(source_hz, hz) != 0xFF; }

    template <uint32_t hz>
    static void set() {
        static_assert(can_run_at(hz),
                      "DynamicClock: this rate is not the root divided by an HPRE "
                      "divider (1..8, then 16, 32, 64, 128, 256)");
        apply(hpre_for(source_hz, hz), hz);
    }

    static bool set(uint32_t hz) {
        const uint8_t code = hpre_for(source_hz, hz);
        if (code == 0xFF) {
            return false;
        }
        apply(code, hz);
        return true;
    }

private:
    static void apply(uint8_t code, uint32_t next) {
        (Users::rebase(next), ...);
        const bool rising = next > hz_;
        if (rising) {
            flash_ctl()->ACTLR = flash_latency_for(next);
        }
        Rcc::hpre(code);
        if (!rising) {
            flash_ctl()->ACTLR = flash_latency_for(next);
        }
        hz_ = next;
        idx_ = code;
    }

    static inline uint32_t hz_ = Boot::hz;
    static inline uint8_t idx_ = 0;
};

} // namespace brio
