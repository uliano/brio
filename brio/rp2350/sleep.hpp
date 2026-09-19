/*
 * sleep.hpp
 *
 * The ways this chip stops (datasheet 6.5: 6.5.1 the top-level clock
 * gates, 6.5.2 the SLEEP state, 6.5.3 DORMANT and 6.5.3.1 what ends it,
 * 6.5.4 the memory periphery, 6.5.5 the memory power domains; 8.2.6 and
 * 8.3.10 the two oscillators' DORMANT registers; 9.5 the IO bank's
 * dormant-wake destination), and the sleep sites of util/power.hpp over
 * them.
 *
 * FOUR DEPTHS OF STOPPING EXIST ON THIS SILICON, and only three of them
 * are on the ladder:
 *
 *   light    a plain sleep instruction. The core stops, the chip does
 *            not: every peripheral, the other core and the DMA run on.
 *            The kernel's own idle, named.
 *   standby  THE SLEEP STATE (6.5.2). The chip enters it when both
 *            processors are asleep and the DMA has no transfer in
 *            flight, and then the top-level clock gates switch from the
 *            WAKE_ENx masks to the SLEEP_ENx ones: what a program leaves
 *            out of SLEEP_ENx is unclocked until an interrupt of a block
 *            still clocked wakes a core. So this rung is the program's
 *            pruning set, applied at arm() and taken back at disarm() -
 *            AND THE OTHER HALF OF THE CONDITION, which is the one thing
 *            the two architectures do not share: "a processor is asleep"
 *            is a signal the core releases, and a Cortex-M33 releases it
 *            on any WFI while a Hazard3 releases it only with
 *            MSLEEP.POWERDOWN set. `sleep_releases_power_request()` in
 *            each half's core header is that one bit, and this rung arms
 *            it. What is NOT here is SLEEPDEEP: the M33's bit plays no
 *            part in 6.5.2's condition, and the Hazard3's equivalent is
 *            the one erratum RP2350-E4 forbids.
 *   deep     DORMANT (6.5.3). The oscillator the program runs on is
 *            stopped by a keyword written into its DORMANT register;
 *            every clock derived from it stands still, the core
 *            included, and it restarts on a GPIO event the IO bank's
 *            dormant-wake destination detects, or on the ALWAYS-ON
 *            TIMER'S ALARM - which is this chip's replacement for the
 *            RP2040's calendar, and a better one: it runs in every power
 *            state. NOT A SLEEP INSTRUCTION, so the platform's
 *            `sleep_hook` takes the idle path instead. REFUSED with no
 *            way back.
 *
 * And off the ladder, deliberately: THE P1.m STATES (6.2.2), where the
 * switched core has no supply at all. They are not a deeper rung because
 * leaving one is a BOOT and not a wake - nothing of the program's state
 * survives but the always-on block's scratch words and whichever SRAM
 * domain was left powered - and a rung of a ladder that never returns to
 * its caller would make `arm()` a lie. `Powman::power_down()`
 * (rp2350/powman.hpp) is that door, named for what it is.
 *
 * THE ALWAYS-ON TIMER IS BOTH THE ALARM AND THE WITNESS, on every rung,
 * which is what makes this target's timed site simpler than the
 * RP2040's: one counter answers "wake me in n milliseconds" and "how
 * long was I gone" in a standby, in a dormant AND across a power-down,
 * because it is the one counter that never stops. Its unit is the
 * millisecond and the kernel's tick is a millisecond on both halves, so
 * the two rulers need no scaling at all - but the conversions are
 * written out anyway, rounding the wake UP and the resync DOWN, because
 * a program may choose another tick rate and the direction of each
 * rounding is the contract (late, never early).
 *
 * WHAT STOPS THE KERNEL'S OWN TIMEBASE. The two halves differ here and
 * the suite measures both: on the Cortex-M33 the timebase is SysTick,
 * core-private and clocked by the core, which has no gate in SLEEP_ENx
 * at all; on Hazard3 it is the platform timer in the SIO, counting a
 * tick generator's output off clk_ref, and both the SIO and the tick
 * generator DO have gates. Either way a DORMANT stops both, because
 * every clock stops - and that is what the witness is for.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "rp2350/device.hpp"

#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/powman.hpp"
#include "util/power.hpp"

namespace brio {

// ---- the gate sets a sleeping program composes ------------------------------
//
// SleepClocks is clock.hpp's type (one bit per clock endpoint, both
// registers, all open at reset). The SETS live here because they are
// about SLEEPING and nothing else reads them: a program names the ones
// it needs and ORs them together.

/// What a program must keep clocked to go on being a program: the bus
/// fabric and its arbiter, every SRAM bank, the ROM and the boot RAM,
/// the execute-in-place interface the code is fetched through, the SIO
/// (the cores' own registers, and the RISC-V half's timebase), the clock
/// and reset infrastructure, the pads and the IO controller, the access
/// controller the fabric checks, and both oscillators.
inline constexpr SleepClocks sleep_clocks_core{
    CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_BUSCTRL_BITS |
        CLOCKS_SLEEP_EN0_CLK_SYS_ROM_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_BOOTRAM_BITS |
        CLOCKS_SLEEP_EN0_CLK_SYS_SIO_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_CLOCKS_BITS |
        CLOCKS_SLEEP_EN0_CLK_SYS_PSM_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_PADS_BITS |
        CLOCKS_SLEEP_EN0_CLK_SYS_IO_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_ACCESSCTRL_BITS |
        CLOCKS_SLEEP_EN0_CLK_SYS_ROSC_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_PLL_SYS_BITS,
    CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_XIP_BITS |
        CLOCKS_SLEEP_EN1_CLK_SYS_SRAM0_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM1_BITS |
        CLOCKS_SLEEP_EN1_CLK_SYS_SRAM2_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM3_BITS |
        CLOCKS_SLEEP_EN1_CLK_SYS_SRAM4_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM5_BITS |
        CLOCKS_SLEEP_EN1_CLK_SYS_SRAM6_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM7_BITS |
        CLOCKS_SLEEP_EN1_CLK_SYS_SRAM8_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM9_BITS |
        CLOCKS_SLEEP_EN1_CLK_SYS_SYSCFG_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_SYSINFO_BITS};

/// The always-on block's two endpoints - its own reference clock and its
/// bus interface. Without the first the alarm cannot be reached by a
/// sleeping program at all; without the second its registers do not
/// answer the handler that reads them.
inline constexpr SleepClocks sleep_clocks_powman{
    CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS | CLOCKS_SLEEP_EN0_CLK_SYS_POWMAN_BITS, 0u};

/// The microsecond ruler and the tick generators behind it - what a
/// program keeps when it means to measure its own sleep with TIMER0 as
/// well.
inline constexpr SleepClocks sleep_clocks_timer{
    0u, CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_TICKS_BITS |
            CLOCKS_SLEEP_EN1_CLK_REF_TICKS_BITS};

/// The countdown that reboots the chip, which a sleep must not starve if
/// it is running.
inline constexpr SleepClocks sleep_clocks_watchdog{0u, CLOCKS_SLEEP_EN1_CLK_SYS_WATCHDOG_BITS};

/// A byte transport's two endpoints. A UART whose gates are pruned loses
/// the byte in flight, so the console's set stays in.
inline constexpr SleepClocks sleep_clocks_uart0{
    0u, CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS | CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS};
inline constexpr SleepClocks sleep_clocks_uart1{
    0u, CLOCKS_SLEEP_EN1_CLK_SYS_UART1_BITS | CLOCKS_SLEEP_EN1_CLK_PERI_UART1_BITS};
/// The DMA, whose idleness is half of 6.5.2's condition and whose clock
/// a transfer still in flight needs.
inline constexpr SleepClocks sleep_clocks_dma{CLOCKS_SLEEP_EN0_CLK_SYS_DMA_BITS, 0u};
/// The PWM block: a slice left counting is the cheapest instrument there
/// is for telling whether the SLEEP state was reached.
inline constexpr SleepClocks sleep_clocks_pwm{CLOCKS_SLEEP_EN0_CLK_SYS_PWM_BITS, 0u};

/// The oscillator a dormant stops.
enum class DormantSource : uint8_t {
    xosc,   ///< the crystal: everything stops, and its restart is the datasheet's ">1 ms"
    rosc,   ///< the ring oscillator: the crystal goes on running, and its restart is a microsecond
};

/// The events the IO bank's dormant-wake destination detects. They are
/// the pin events of rp2350/pin.hpp - the same four bits in the same
/// four registers - reached through `PinIrqTarget::dormant_wake`, which
/// is why this file adds no type of its own for them.
struct DormantWake {
    DormantWake() = delete;

    static constexpr PinIrqTarget target = PinIrqTarget::dormant_wake;

    /// Arm `events` of `pin` as a wake, with whatever is latched cleared
    /// first. False when this package has not got the pad.
    static bool enable(uint8_t pin, PinEvents events) {
        if (pin >= gpio_count) {
            return false;
        }
        PinIrq::clear(pin, pin_edges);
        PinIrq::enable(pin, events, target);
        return true;
    }
    static void disable(uint8_t pin, PinEvents events) { PinIrq::disable(pin, events, target); }
    static PinEvents enabled(uint8_t pin) { return PinIrq::enabled(pin, target); }
    /// What this pad is raising towards the wake logic right now.
    static PinEvents pending(uint8_t pin) { return PinIrq::status(pin, target); }
    /// The raw events, shared with the cores' own destinations.
    static PinEvents raw(uint8_t pin) { return PinIrq::raw(pin); }
    static void acknowledge(uint8_t pin, PinEvents events) { PinIrq::clear(pin, events); }

    /// Is there any GPIO wake at all - is there a way out of a dormant?
    /// The summary registers of 9.5 answer it in two reads.
    static bool any_enabled() {
        return PinIrq::summary(target, 0) != 0u || PinIrq::summary(target, 1) != 0u ||
               any_enable_bit();
    }
    static void disable_all() { PinIrq::disable_all(target); }

private:
    /// A level that is not standing and an edge that has not happened
    /// raise nothing in the summary, so the ENABLES are asked too.
    static bool any_enable_bit() {
        for (uint8_t w = 0; w < 6u; ++w) {
            if (reg_at(IO_BANK0_BASE, IO_BANK0_DORMANT_WAKE_INTE0_OFFSET + 4u * w) != 0u) {
                return true;
            }
        }
        return false;
    }
};

/**
 * The plain site: the ladder onto a sleep instruction, the SLEEP state
 * and DORMANT.
 *
 * `Clock` is the program's static clock type, whose init() restores the
 * tree after a dormant; `source` the oscillator a dormant stops. Core
 * 0's: a dormant is chip-wide, and the hook it installs is core 0's
 * platform's.
 *
 * THE PRUNING SET IS THE PROGRAM'S, handed over once with
 * `standby_clocks()`. The site writes it into SLEEP_ENx when it arms a
 * standby and writes every gate open again when it disarms, so a light
 * sleep prunes nothing and a standby prunes exactly what the program
 * named. The default prunes nothing either, which makes an unconfigured
 * standby a light sleep and not a wedge.
 */
template <typename Clock, DormantSource source = DormantSource::xosc>
struct Rp2350SleepSite {
    Rp2350SleepSite() = delete;

    static_assert(Clock::is_static, "brio Rp2350SleepSite: the tree is restored after a dormant by "
                                    "the program's static Clock");

    using Platform0 = Rp2350Platform<0>;

    /// The gates a standby keeps open. Composed from the sets above;
    /// `sleep_clocks_all` (the default) prunes nothing.
    static void standby_clocks(SleepClocks c) { standby_ = c; }
    static SleepClocks standby_clocks() { return standby_; }

    static bool arm(SleepDepth d) {
        switch (d) {
        case SleepDepth::none:
            disarm();
            return true;
        case SleepDepth::light:
            Platform0::sleep_hook = nullptr;
            Clocks::sleep_enables(sleep_clocks_all);
            sleep_releases_power_request(false);
            armed_ = SleepDepth::light;
            return true;
        case SleepDepth::standby:
            Platform0::sleep_hook = nullptr;
            Clocks::sleep_enables(standby_);
            // THE OTHER HALF OF 6.5.2'S CONDITION, and the one place
            // this file asks the PROCESSOR for anything. A Cortex-M33
            // releases its power request on any WFI and this call is
            // empty there; a Hazard3 does it only with MSLEEP.POWERDOWN
            // set, and without it the chip never leaves its wake masks.
            sleep_releases_power_request(true);
            armed_ = SleepDepth::standby;
            return true;
        case SleepDepth::deep:
            if (!dormant_wake_ready()) {
                return false;
            }
            Clocks::sleep_enables(sleep_clocks_all);
            Platform0::sleep_hook = &go_dormant;
            armed_ = SleepDepth::deep;
            return true;
        }
        return false;
    }

    static void disarm() {
        Platform0::sleep_hook = nullptr;
        Clocks::sleep_enables(sleep_clocks_all);
        sleep_releases_power_request(false);
        armed_ = SleepDepth::none;
    }

    static SleepDepth armed() { return armed_; }

    /**
     * Is there a way back from a dormant?
     *
     * A GPIO dormant-wake event always is one - it needs no clock at
     * all. The always-on timer's alarm is one too, and with ONE
     * CONDITION that is this chip's own: the timer must not be counting
     * the crystal when the crystal is what the dormant stops (12.10.5.3
     * gives the power-down sequencer that fallback and gives a DORMANT
     * none). On the ring-oscillator site the crystal keeps running and
     * either source will do.
     */
    static bool dormant_wake_ready() {
        if (DormantWake::any_enabled()) {
            return true;
        }
        if (!AonTimer::alarm_enabled()) {
            return false;
        }
        if constexpr (source == DormantSource::xosc) {
            return AonTimer::source() != AonSource::xosc;
        } else {
            return true;
        }
    }

    /// How many dormants this site has run and returned from.
    static uint32_t dormants() { return dormants_; }

    /**
     * The idle path's substitute for the sleep instruction when deep is
     * armed: called with interrupts masked.
     *
     * clk_sys onto clk_ref and clk_ref onto the oscillator that stays
     * (the crystal already is clk_ref; the ring oscillator is started
     * and selected), then the PLLs - which the silicon does NOT stop and
     * whose VCO misbehaves when its reference goes away (8.2.6's note) -
     * then the keyword. Once a wake has restarted the oscillator, the
     * program's tree is put back exactly as its Clock states it, before
     * the caller unmasks and the wake's handler runs at the rate every
     * driver was told.
     */
    static void go_dormant() {
        (void)Clocks::sys_from_ref();
        Clocks::sys_divider(1);
        if constexpr (source == DormantSource::rosc) {
            // THE REFERENCE CLOCK IS LEFT ON THE CRYSTAL and clk_sys is
            // taken to the ring oscillator through its OWN aux mux, not
            // through clk_ref. That is the whole point of this site: the
            // core stops and clk_ref does not, so the always-on block -
            // which runs on clk_ref while the switched core is powered
            // (SEQ_CFG.USE_FAST_POWCK) - goes on counting and its alarm
            // can still end the stop.
            (void)Rosc::start();
            (void)Clocks::sys_from_aux(SysAux::rosc);
            PllSys::stop();
            Rosc::dormant();
        } else {
            // AND THE SAME TRICK THE OTHER WAY ROUND. clk_ref is taken
            // to the LOW-POWER OSCILLATOR, which no dormant stops, and
            // clk_sys to the crystal through its own aux mux - so
            // stopping the crystal stops the core and leaves clk_ref
            // running. The always-on block's alarm needs clk_ref alive
            // to end a dormant (measured; docs/rp2350/sleep.md), and
            // this is what gives it to a program that means to stop the
            // crystal.
            (void)Clocks::ref_select(RefSource::lposc);
            Clocks::ref_divider(1);
            (void)Clocks::sys_from_aux(SysAux::xosc);
            PllSys::stop();
            PllUsb::stop();
            Xosc::dormant();
        }
        (void)Clock::init();
        ++dormants_;
    }

private:
    static inline SleepDepth armed_ = SleepDepth::none;
    static inline uint32_t dormants_ = 0;
    static inline SleepClocks standby_ = sleep_clocks_all;
};

/**
 * The timed site: the plain site plus THE ALWAYS-ON TIMER as alarm and
 * witness on every rung it covers.
 *
 * `init()` enables the alarm's line at the interrupt controller; the
 * timer itself is the application's (`AonTimer::init(clock)`, which
 * measures the low-power oscillator and tells the divider what it
 * found). The app binds
 *
 *   extern "C" void isr_powman_timer() { Site::isr(); }
 *
 * and that name is bound on both architectures.
 *
 * THE ALARM IS PLACED AT ARM AND THE SPAN IS READ AT WAKE. A deadline is
 * rounded UP into milliseconds (late, never early) and the span slept is
 * rounded DOWN into ticks (the kernel's time never runs ahead of the
 * program's). What the resync hands `Ticker::advance()` is the span less
 * the ticks the timebase counted for itself - zero when the timebase
 * never stopped, which is the honest answer for a standby whose SysTick
 * ran through it.
 */
template <Platform P, typename Clock, DormantSource source = DormantSource::xosc>
struct Rp2350TimedSleepSite {
    Rp2350TimedSleepSite() = delete;

    static_assert(P::core == 0u, "brio Rp2350TimedSleepSite: core 0's site - a dormant is chip-wide "
                                 "and the tree is core 0's to restore");

    using Plain = Rp2350SleepSite<Clock, source>;
    using Timebase = typename P::Timebase;

    /// The gates a standby keeps for the ALARM to be reached: the
    /// always-on block's two endpoints. A program ORs its own set with
    /// this one; `standby_clocks()` below does it for the caller.
    static constexpr SleepClocks alarm_gates = sleep_clocks_powman;

    /// The program's pruning set, with the alarm's gates forced in.
    static void standby_clocks(SleepClocks c) { Plain::standby_clocks(c | alarm_gates); }
    static SleepClocks standby_clocks() { return Plain::standby_clocks(); }

    /// The alarm's line enabled at the interrupt controller. Call once,
    /// before the manager's first round. False when the always-on timer
    /// is not running - there being no witness then, and no alarm.
    static bool init() {
        if (!AonTimer::running()) {
            return false;
        }
        AonTimer::alarm_enable(false);
        AonTimer::clear_alarm();
        AonTimer::interrupt(true);
        Irq::enable(AonTimer::irq());
        ready_ = true;
        return true;
    }

    static bool arm(SleepDepth d) {
        if (!Plain::arm(d)) {
            return false;
        }
        if (!ready_ || !is_deep_mode(Plain::armed())) {
            return true;   // a light sleep ticks on its own
        }
        const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
        tick_at_arm_ = Timebase::ticks();
        aon_at_arm_ = AonTimer::now_low();
        witnessed_ = true;
        if (next.has_value()) {
            AonTimer::alarm_in(ticks_to_ms(*next));
            alarm_armed_ = true;
        }
        // A dormant with no way back must not be armed, and the alarm
        // just placed may be exactly the way back the plain site was
        // missing: the refusal is re-asked now that it exists.
        if (Plain::armed() == SleepDepth::deep && !Plain::dormant_wake_ready()) {
            disarm();
            return false;
        }
        return true;
    }

    static void disarm() {
        if (alarm_armed_) {
            AonTimer::alarm_enable(false);
            AonTimer::clear_alarm();
            alarm_armed_ = false;
        }
        resync();
        Plain::disarm();
    }

    static SleepDepth armed() { return Plain::armed(); }

    /// Catch kernel time up by the FROZEN span: the always-on timer's
    /// elapsed milliseconds converted DOWN to ticks, less what the
    /// timebase counted for itself since arm(). Consumed once - isr()
    /// and disarm() both call it - and a round that never slept advances
    /// by nothing, because the two rulers then agree. The critical
    /// section makes the once-ness hold against an alarm landing inside
    /// a disarm().
    static void resync() {
        typename P::CriticalSection cs;
        if (!witnessed_) {
            return;
        }
        witnessed_ = false;
        const uint32_t ms = AonTimer::now_low() - aon_at_arm_;   // wrap-safe
        const uint32_t span = ms_to_ticks(ms);
        const uint32_t awake = Timebase::ticks() - tick_at_arm_;   // wrap-safe
        if (span > awake) {
            last_advance_ = span - awake;
            Timebase::advance(last_advance_);
        } else {
            last_advance_ = 0;
        }
    }

    /**
     * THE ISR BODY the app binds to `isr_powman_timer`: the timer's own
     * body first (the alarm acknowledged AND disarmed - see
     * AonTimer::isr on why the second is not optional), then the RESYNC,
     * then THE MACHINE HANDED BACK TO A TICKING SLEEP.
     *
     * The third act is the SAM's lesson, and it holds here for the same
     * reason: the never-early rounding can leave kernel time a fraction
     * of a tick short of the deadline when the alarm lands, and an alarm
     * posts nothing to any queue, so with the site still armed the loop
     * would re-enter a sleep that nothing ends. Disarming here makes the
     * next idle a plain sleep, which the tick ends.
     */
    [[gnu::always_inline]] static bool isr() {
        if (!AonTimer::isr()) {
            return false;
        }
        alarm_armed_ = false;
        resync();
        Plain::disarm();
        return true;
    }

    // ---- readbacks the suites and a status console want ---------------------
    static bool ready() { return ready_; }
    static bool alarm_armed() { return alarm_armed_; }
    /// Ticks the last resync handed to the timebase (0 when the round
    /// never slept, or slept less than it stayed awake).
    static uint32_t last_advance() { return last_advance_; }

    /// Ticks to milliseconds, rounded UP: a wake a shade late is "at
    /// least", a wake a shade early is a wasted vote round.
    static constexpr uint32_t ticks_to_ms(uint32_t ticks) {
        return static_cast<uint32_t>(
            (static_cast<uint64_t>(ticks) * 1000u + P::ticks_per_second - 1u) /
            P::ticks_per_second);
    }
    /// Milliseconds to ticks, rounded DOWN: kernel time never runs ahead
    /// of the program.
    static constexpr uint32_t ms_to_ticks(uint32_t ms) {
        return static_cast<uint32_t>(static_cast<uint64_t>(ms) * P::ticks_per_second / 1000u);
    }

private:
    static inline bool ready_ = false;
    static inline bool alarm_armed_ = false;
    static inline bool witnessed_ = false;
    static inline uint32_t aon_at_arm_ = 0;
    static inline uint32_t tick_at_arm_ = 0;
    static inline uint32_t last_advance_ = 0;
};

} // namespace brio
