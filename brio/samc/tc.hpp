/*
 * tc.hpp
 *
 * The SAM C21 Timer/Counters (DS60001479M ch. 35): five instances of a
 * counter with a prescaler, two compare/capture channels, four waveform
 * modes, one event input and three event outputs.
 *
 *  Tc<n>            the RESOURCE: the whole chapter's register surface -
 *                   the three counter resolutions, the prescaler and its
 *                   synchronization, the waveform modes, capture with
 *                   its event actions, the commands, the buffered
 *                   registers, status, flags and the ISR body.
 *
 *  TcWo<Pin>        one waveform output reached through the pad that
 *                   carries it: which TC and which WO a pad is comes
 *                   from the device header, never from a table this file
 *                   keeps.
 *
 *  TcPwm<Tc, ch>    tasks over the resource: a 16-bit PWM channel and an
 *  TcPwm8<...>      8-bit one, both satisfying util/pwm_channel.hpp's
 *                   PwmChannel; TcPeriodMeter and TcPulseWidthMeter,
 *  TcPeriodMeter    the capture shapes util/meter_sampler.hpp's
 *  TcPulseWidthMeter MeterSource is fed from.
 *
 * SIX FACTS THAT SHAPE THE FILE.
 *
 * 1. THE GEOMETRY IS THE DEVICE HEADER'S, ALL OF IT. How many
 *    instances, how many channels each, which generic clock channel
 *    each uses, which instances can be PAIRED into a 32-bit counter -
 *    every one of those is a `TCn_*` constant, and none of them is
 *    spelled out here. Two of them are worth reading before writing
 *    any code:
 *      - TC0 and TC1 SHARE generic clock channel 30, TC2 and TC3 share
 *        31, and TC4 has 32 to itself. 35.5.3 warns that two instances
 *        sharing a channel "cannot be set to different clock
 *        frequencies", and `gclk_id` is what says which do.
 *      - `TCn_MASTER_SLAVE_MODE` is 1 for a master, 2 for a client and
 *        0 for an instance that cannot pair at all - TC0+TC1 and
 *        TC2+TC3 pair, TC4 does not. That is exactly 35.6.2.4's
 *        sentence, and it is what `config_valid()` refuses a 32-bit
 *        mode on.
 *
 * 2. THE PAD-TO-(TC, WO) MAP IS NOT A FORMULA either, and it is
 *    per-package: 26 pads on the J, 18 on the G, 8 on the E, with
 *    PA22 = TC0/WO0, PB12 = TC0/WO0 again, PB23 = TC3/WO1 (the bench
 *    board's LED). `PIN_P<pad>E_TC<n>_WO<k>` is the authority and the
 *    table at the bottom of this file is those symbols, each guarded by
 *    its own `#ifdef`, so `TcWo<>` on an unbonded pad simply does not
 *    compile.
 *
 * 3. READING COUNT IS A COMMAND, not a load. COUNT is
 *    READ-synchronized "on demand through the READSYNC command"
 *    (35.6.8), so a correct read is: write CTRLBSET.CMD = READSYNC,
 *    wait out SYNCBUSY.CTRLB and SYNCBUSY.COUNT, then read. That is
 *    what `count16()` and its siblings do - the raw accessors are
 *    spelled `*_raw()` and say what they are.
 *
 * 4. THE REGISTER MAP IS THREE OVERLAID VIEWS. The device header gives
 *    `tc_registers_t` as a union of COUNT8/COUNT16/COUNT32 structs;
 *    everything up to SYNCBUSY sits at the same offset in all three and
 *    only COUNT, PER and CC/CCBUF differ. So the control surface is
 *    written once against the 16-bit view, and the width-carrying verbs
 *    come in explicit flavours (`count8`/`count16`/`count32`,
 *    `cc8`/`cc16`/`cc32`). A caller that configured COUNT8 uses the
 *    8-bit ones; nothing dispatches at run time on a mode it could
 *    have known at compile time.
 *
 * 5. ENABLE-PROTECTION AND WRITE-SYNCHRONIZATION ARE DIFFERENT THINGS
 *    here and both matter. CTRLA (bar ENABLE/SWRST), DRVCTRL, WAVE and
 *    EVCTRL are ENABLE-protected: `configure()` and `event_config()`
 *    refuse while the TC is enabled. CTRLB, COUNT, PER/PERBUF and
 *    CC/CCBUF are WRITE-synchronized: every verb that touches one
 *    waits its SYNCBUSY bit out, bounded, and reports.
 *
 * 6. THE PERIOD REGISTER EXISTS ONLY IN 8-BIT MODE (35.6.2.4). In
 *    COUNT16 and COUNT32 the TOP of an NPWM or NFRQ waveform is fixed
 *    at MAX and the only way to shorten it is MPWM/MFRQ, which spend
 *    CC0 as the period and leave one channel. `period8()` says so by
 *    existing only in one width.
 *
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F - and once
 * again the row is the whole story:
 *  - 1.20.1 Capture Overflow (a capture overflow within 3 APB + 3 GCLK
 *    periods of the previous one raises no INTFLAG.ERR): revision B
 *    only.
 *  - 1.20.2 I/O Pins ("the input capture on I/O pins does not work",
 *    workaround: capture through the event system with the EIC or CCL
 *    as generator): REVISION B ONLY. This is the item most likely to be
 *    applied by mistake - CTRLA.COPEN is implemented and usable here,
 *    and docs/samc/tc.md carries the measurement.
 *  - 1.20.3 SYNCBUSY Flag: EVERY REVISION OF E/G/J, so LIVE HERE. When
 *    STATUS.PERBUFV or STATUS.CCBUFVx is cleared, SYNCBUSY is released
 *    BEFORE the buffer register has been restored, and the remedy is to
 *    clear the flag twice. `clear_buffer_valid()` writes it twice, and
 *    that is the whole workaround.
 *
 * NOT BUILT (docs/samc/tc.md carries the list): the minimum and maximum
 * capture modes (SAM C20/C21 N variants only - this family's device
 * header does not declare CTRLA.CAPTMODE at all), and sleep, which the
 * power pass owns together with CTRLA.ONDEMAND's clock-request
 * behaviour.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// CTRLA.MODE (35.6.2.4). COUNT32 pairs two instances and is legal only
/// on a master - see `tc_can_pair()`.
enum class TcMode : uint8_t {
    count8 = TC_CTRLA_MODE_COUNT8_Val,
    count16 = TC_CTRLA_MODE_COUNT16_Val,
    count32 = TC_CTRLA_MODE_COUNT32_Val,
};

/// CTRLA.PRESCALER. Bypassed entirely when the counter counts events
/// (35.6.2.3).
enum class TcPrescaler : uint8_t {
    div1 = TC_CTRLA_PRESCALER_DIV1_Val,
    div2 = TC_CTRLA_PRESCALER_DIV2_Val,
    div4 = TC_CTRLA_PRESCALER_DIV4_Val,
    div8 = TC_CTRLA_PRESCALER_DIV8_Val,
    div16 = TC_CTRLA_PRESCALER_DIV16_Val,
    div64 = TC_CTRLA_PRESCALER_DIV64_Val,
    div256 = TC_CTRLA_PRESCALER_DIV256_Val,
    div1024 = TC_CTRLA_PRESCALER_DIV1024_Val,
};

constexpr uint16_t tc_prescaler_divisor(TcPrescaler p) {
    switch (p) {
    case TcPrescaler::div1: return 1;
    case TcPrescaler::div2: return 2;
    case TcPrescaler::div4: return 4;
    case TcPrescaler::div8: return 8;
    case TcPrescaler::div16: return 16;
    case TcPrescaler::div64: return 64;
    case TcPrescaler::div256: return 256;
    case TcPrescaler::div1024: return 1024;
    }
    return 1;
}

/// CTRLA.PRESCSYNC: when a counter update lands relative to the
/// prescaled clock. Only meaningful with a prescaler above 1.
enum class TcPrescalerSync : uint8_t {
    gclk = TC_CTRLA_PRESCSYNC_GCLK_Val,
    prescaler = TC_CTRLA_PRESCSYNC_PRESC_Val,
    resync = TC_CTRLA_PRESCSYNC_RESYNC_Val,
};

/// WAVE.WAVEGEN (35.6.2.6.1). The two NORMAL modes take TOP from the
/// counter's own width (or PER in 8-bit mode) and leave both channels
/// free; the two MATCH modes spend CC0 as the period.
enum class TcWaveform : uint8_t {
    normal_frequency = TC_WAVE_WAVEGEN_NFRQ_Val,
    match_frequency = TC_WAVE_WAVEGEN_MFRQ_Val,
    normal_pwm = TC_WAVE_WAVEGEN_NPWM_Val,
    match_pwm = TC_WAVE_WAVEGEN_MPWM_Val,
};

/// CTRLBSET.CMD - write-only commands, taken on the write.
enum class TcCommand : uint8_t {
    none = TC_CTRLBSET_CMD_NONE_Val,
    retrigger = TC_CTRLBSET_CMD_RETRIGGER_Val,
    stop = TC_CTRLBSET_CMD_STOP_Val,
    update = TC_CTRLBSET_CMD_UPDATE_Val,
    /// The only way to read COUNT correctly (35.6.8) - `count16()` and
    /// friends issue it themselves.
    read_sync = TC_CTRLBSET_CMD_READSYNC_Val,
    dma_one_shot = TC_CTRLBSET_CMD_DMAOS_Val,
};

/// EVCTRL.EVACT: what an incoming event DOES. The last four are the
/// capture actions and they decide what CC0 and CC1 come to mean.
enum class TcEventAction : uint8_t {
    off = TC_EVCTRL_EVACT_OFF_Val,
    /// Reload or clear the counter. NOTE 35.6.2.5.2: with this action
    /// selected, ENABLING the TC does not start it - the first event
    /// does.
    retrigger = TC_EVCTRL_EVACT_RETRIGGER_Val,
    /// Count events instead of clock ticks; the prescaler is bypassed.
    /// 35.6.2.5.3: PWM generation is not supported in this mode.
    count = TC_EVCTRL_EVACT_COUNT_Val,
    /// Start a stopped counter. No effect while it is already counting.
    start = TC_EVCTRL_EVACT_START_Val,
    /// Time-stamp: COUNT copied into CCx on every event. The chapter
    /// requires TOP < MAX for this one (35.6.3.2).
    stamp = TC_EVCTRL_EVACT_STAMP_Val,
    /// Period into CC0, pulse width into CC1.
    period_pulse_width = TC_EVCTRL_EVACT_PPW_Val,
    /// Pulse width into CC0, period into CC1 - the other order, and the
    /// only difference between the two.
    pulse_width_period = TC_EVCTRL_EVACT_PWP_Val,
    /// Pulse width alone into CC0; the counter stops between pulses.
    pulse_width = TC_EVCTRL_EVACT_PW_Val,
};

constexpr bool tc_action_is_capture(TcEventAction a) {
    return a == TcEventAction::stamp || a == TcEventAction::period_pulse_width ||
           a == TcEventAction::pulse_width_period ||
           a == TcEventAction::pulse_width;
}

/**
 * Everything CTRLA, WAVE, DRVCTRL and the two CTRLB mode bits hold -
 * one struct, because they are written together at initialization and
 * the first three are enable-protected as a group (35.6.2.1).
 */
struct TcConfig {
    TcMode mode = TcMode::count16;
    TcPrescaler prescaler = TcPrescaler::div1;
    TcPrescalerSync prescaler_sync = TcPrescalerSync::gclk;
    TcWaveform waveform = TcWaveform::normal_frequency;

    /// CTRLA.CAPTENx, bit x = channel x: that channel captures instead
    /// of comparing.
    uint8_t capture_enable = 0;
    /// CTRLA.COPENx: that channel captures from its WO PAD rather than
    /// from the event input. Mutually exclusive with the event source
    /// per channel, and only legal in the `stamp` and event capture
    /// actions (35.6.2.8 note 1).
    uint8_t capture_on_pin = 0;
    /// DRVCTRL.INVENx: inverts the waveform OUTPUT, and - for a capture
    /// channel fed from its pad - the input edge that captures.
    uint8_t invert = 0;

    bool run_standby = false;
    /// CTRLA.ONDEMAND: stop requesting the generic clock while stopped.
    bool on_demand = false;
    /// CTRLA.ALOCK: hold buffered updates until an UPDATE command.
    bool lock_update = false;

    /// CTRLB.DIR: count down from TOP instead of up from zero.
    bool count_down = false;
    /// CTRLB.ONESHOT: stop at the next overflow/underflow (35.6.3.1).
    bool one_shot = false;
};

/// EVCTRL, which is enable-protected with the rest.
struct TcEventConfig {
    TcEventAction action = TcEventAction::off;
    bool input_enable = false;   ///< TCEI: incoming events reach the TC
    bool invert_input = false;   ///< TCINV: the incoming event is inverted
    bool overflow_out = false;   ///< OVFEO: overflow/underflow as an event
    uint8_t match_out = 0;       ///< MCEOx, bit x = channel x
};

// ---- geometry: samc/device_tables.hpp is the authority -----------------------
//
// `tc_count()`, `tc_pair_role(n)` / `tc_can_pair(n)`, `tc_gclk_id(n)`
// and the DMAC trigger ids `tc_dma_overflow_id(n)` /
// `tc_dma_match0_id(n)` live in the reserve (device_tables.hpp),
// probed from the header's own `TCn_*` constants - see that file for
// why they are probes and not per-variant tables.

/// A configuration's legality for THIS instance.
///
/// Three refusals:
///  - COUNT32 on an instance that cannot be a pair master (35.6.2.4);
///  - a capture-on-pin channel that is not also a capture channel -
///    COPEN without CAPTEN captures nothing (35.6.2.8);
///  - a channel bit past the two this family implements.
constexpr bool tc_config_valid(uint8_t n, const TcConfig& c) {
    if (c.mode == TcMode::count32 && !tc_can_pair(n)) {
        return false;
    }
    if ((c.capture_enable | c.capture_on_pin | c.invert) > 0x3u) {
        return false;
    }
    return (c.capture_on_pin & ~c.capture_enable) == 0u;
}

/**
 * The two rules that live BETWEEN the two configuration structs, so
 * they get a function of their own rather than being lost:
 *  - "If this operation mode is selected, PWM generation is not
 *    supported" for the count-on-event action (35.6.2.5.3);
 *  - a capture EVENT ACTION needs at least one capture channel, and a
 *    capture channel needs a source - its pad or the event input.
 */
constexpr bool tc_event_config_valid(const TcConfig& c, const TcEventConfig& e) {
    if (e.match_out > 0x3u) {
        return false;
    }
    if (e.action == TcEventAction::count &&
        (c.waveform == TcWaveform::normal_pwm ||
         c.waveform == TcWaveform::match_pwm)) {
        return false;
    }
    if (tc_action_is_capture(e.action)) {
        if (c.capture_enable == 0u) {
            return false;
        }
        // Every enabled capture channel needs a source: its own pad, or
        // the shared event input.
        if (!e.input_enable && (c.capture_enable & ~c.capture_on_pin) != 0u) {
            return false;
        }
    }
    return true;
}

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
class Tc {
    static_assert(n < tc_count(), "this device does not have that TC instance");

public:
    Tc() = delete;

    static constexpr uint8_t index = n;
    /// From `TCn_CC_NUM` - two on every member of this family.
    static constexpr uint8_t cc_count = 2;
    static constexpr uint8_t gclk_id = tc_gclk_id(n);
    static constexpr bool can_pair = tc_can_pair(n);
    /// The client instance a 32-bit pair borrows. Only meaningful when
    /// `can_pair`; its APB clock must be enabled too (35.6.2.4).
    static constexpr uint8_t pair_index = static_cast<uint8_t>(n + 1u);

    static constexpr IRQn_Type irq() {
        return static_cast<IRQn_Type>(static_cast<int>(TC0_IRQn) + n);
    }

    // ---- the EVSYS and DMAC vocabularies this peripheral publishes ---------
    //
    // evsys.hpp owns the fabric and not the vocabulary; dmac.hpp owns
    // the channels and not the trigger table. Both codes live here.

    /// Generator: overflow/underflow. TC0 OVF is 0x34 and each instance
    /// takes three consecutive codes.
    static constexpr uint8_t overflow_generator =
        static_cast<uint8_t>(0x34u + 3u * n);
    /// Generator: match or capture on channel `ch`.
    static constexpr uint8_t match_generator(uint8_t ch) {
        return static_cast<uint8_t>(0x35u + 3u * n + ch);
    }
    /// User: TCnEVU, this instance's single event input (users 23..27).
    /// Table 29-3 allows all three paths, but 35.6.6 says "the TC
    /// requires only asynchronous event inputs" and 35.6.2.8's note 2
    /// requires the asynchronous path for CAPTURE specifically -
    /// docs/samc/tc.md carries what the bench found.
    static constexpr uint8_t event_user = static_cast<uint8_t>(23u + n);

    /// DMAC trigger ids, from the device header's own TCn_DMAC_ID_*.
    static constexpr uint8_t dma_trigger_overflow = tc_dma_overflow_id(n);
    static constexpr uint8_t dma_trigger_match(uint8_t ch) {
        return static_cast<uint8_t>(tc_dma_match0_id(n) + ch);
    }

    static tc_count16_registers_t& regs() { return regs_union().COUNT16; }
    static tc_count8_registers_t& regs8() { return regs_union().COUNT8; }
    static tc_count32_registers_t& regs32() { return regs_union().COUNT32; }

    static constexpr bool config_valid(const TcConfig& c) {
        return tc_config_valid(n, c);
    }

    // ---- claim and teardown ------------------------------------------------

    /// The APB clock. A 32-bit pair needs BOTH instances' bus clocks
    /// (35.6.2.4), which is why this takes the pair into account.
    static void bus_clock(bool on) { Mclk::apb_c(apb_mask(), on); }
    static void pair_bus_clock(bool on) {
        if constexpr (can_pair) {
            Mclk::apb_c(pair_apb_mask(), on);
        } else {
            (void)on;
        }
    }

    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().TC_SYNCBUSY, mask, false, spins);
    }

    /// CTRLA.SWRST. 35.6.2.2: the TC "should be disabled before the TC
    /// is reset in order to avoid undefined behavior", so this disables
    /// first rather than trusting the caller.
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().TC_CTRLA = 0u;
        if (!sync_wait(TC_SYNCBUSY_ENABLE_Msk, spins)) {
            return false;
        }
        regs().TC_CTRLA = TC_CTRLA_SWRST_Msk;
        return sync_wait(TC_SYNCBUSY_SWRST_Msk, spins);
    }

    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = regs().TC_CTRLA & ~TC_CTRLA_ENABLE_Msk;
        regs().TC_CTRLA = on ? (v | TC_CTRLA_ENABLE_Msk) : v;
        return sync_wait(TC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() { return (regs().TC_CTRLA & TC_CTRLA_ENABLE_Msk) != 0u; }

    /// APB clock (both halves of a pair), generic clock, software reset;
    /// the TC is left DISABLED because CTRLA, WAVE, DRVCTRL and EVCTRL
    /// are enable-protected and the caller has them to write.
    static bool init(uint8_t generator, uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        bus_clock(true);
        pair_bus_clock(true);
        if (!clock(generator, spins)) {
            return false;
        }
        return reset(spins);
    }

    /// Hand everything back - AND MIND THE SHARED CLOCK CHANNEL. TC0/TC1
    /// share one generic clock channel and TC2/TC3 share another (the
    /// header's TCn_GCLK_ID, stated at the top of this file), so the
    /// disconnect below STOPS THE SIBLING TOO: Tc<2>::release() silently
    /// halts a running TC3, which cost the TSENS campaign half a letter
    /// before it was understood. This driver cannot know whether the
    /// sibling is in use - a caller releasing one half of a shared pair
    /// while the other must keep running should skip release() and tear
    /// down by hand (reset + bus_clock), leaving the channel connected.
    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        pair_bus_clock(false);
        bus_clock(false);
    }

    // ---- configuration (enable-protected) ----------------------------------

    /**
     * CTRLA, WAVE, DRVCTRL and the two CTRLB mode bits, in that order.
     * REFUSED while the TC is enabled: those registers are
     * enable-protected (35.6.2.1) and a write that lands nowhere is
     * worse than a false return.
     */
    static bool configure(const TcConfig& c, uint32_t spins = 0xFFFFu) {
        if (!config_valid(c) || enabled()) {
            return false;
        }
        regs().TC_CTRLA =
            TC_CTRLA_MODE(static_cast<uint32_t>(c.mode)) |
            TC_CTRLA_PRESCALER(static_cast<uint32_t>(c.prescaler)) |
            TC_CTRLA_PRESCSYNC(static_cast<uint32_t>(c.prescaler_sync)) |
            TC_CTRLA_CAPTEN(c.capture_enable) |
            TC_CTRLA_COPEN(c.capture_on_pin) |
            (c.run_standby ? TC_CTRLA_RUNSTDBY_Msk : 0u) |
            (c.on_demand ? TC_CTRLA_ONDEMAND_Msk : 0u) |
            (c.lock_update ? TC_CTRLA_ALOCK_Msk : 0u);
        regs().TC_WAVE = static_cast<uint8_t>(
            TC_WAVE_WAVEGEN(static_cast<uint32_t>(c.waveform)));
        regs().TC_DRVCTRL = static_cast<uint8_t>(TC_DRVCTRL_INVEN(c.invert));

        // CTRLB is write-synchronized, and its two mode bits are set and
        // cleared through different registers - so both are stated
        // explicitly rather than left at whatever a reset happened to
        // leave.
        const uint8_t set = static_cast<uint8_t>(
            (c.count_down ? TC_CTRLBSET_DIR_Msk : 0u) |
            (c.one_shot ? TC_CTRLBSET_ONESHOT_Msk : 0u));
        const uint8_t clear = static_cast<uint8_t>(
            (c.count_down ? 0u : TC_CTRLBCLR_DIR_Msk) |
            (c.one_shot ? 0u : TC_CTRLBCLR_ONESHOT_Msk));
        if (clear != 0u) {
            regs().TC_CTRLBCLR = clear;
            if (!sync_wait(TC_SYNCBUSY_CTRLB_Msk, spins)) {
                return false;
            }
        }
        if (set != 0u) {
            regs().TC_CTRLBSET = set;
            if (!sync_wait(TC_SYNCBUSY_CTRLB_Msk, spins)) {
                return false;
            }
        }
        return true;
    }

    /// EVCTRL, enable-protected like the rest. Takes the TcConfig too,
    /// because the two rules that matter live between them - see
    /// `tc_event_config_valid()`.
    static bool event_config(const TcConfig& c, const TcEventConfig& e) {
        if (!tc_event_config_valid(c, e) || enabled()) {
            return false;
        }
        regs().TC_EVCTRL = static_cast<uint16_t>(
            TC_EVCTRL_EVACT(static_cast<uint32_t>(e.action)) |
            (e.input_enable ? TC_EVCTRL_TCEI_Msk : 0u) |
            (e.invert_input ? TC_EVCTRL_TCINV_Msk : 0u) |
            (e.overflow_out ? TC_EVCTRL_OVFEO_Msk : 0u) |
            TC_EVCTRL_MCEO(e.match_out));
        return true;
    }

    static uint32_t ctrla() { return regs().TC_CTRLA; }
    static uint16_t evctrl() { return regs().TC_EVCTRL; }
    static TcMode mode() {
        return static_cast<TcMode>((regs().TC_CTRLA & TC_CTRLA_MODE_Msk) >>
                                   TC_CTRLA_MODE_Pos);
    }

    // ---- commands ----------------------------------------------------------

    /// CTRLBSET.CMD - write-synchronized like the rest of CTRLB.
    static bool command(TcCommand c, uint32_t spins = 0xFFFFu) {
        regs().TC_CTRLBSET =
            static_cast<uint8_t>(TC_CTRLBSET_CMD(static_cast<uint32_t>(c)));
        return sync_wait(TC_SYNCBUSY_CTRLB_Msk, spins);
    }
    static bool retrigger(uint32_t spins = 0xFFFFu) {
        return command(TcCommand::retrigger, spins);
    }
    static bool stop(uint32_t spins = 0xFFFFu) {
        return command(TcCommand::stop, spins);
    }

    /// CTRLB.DIR, live: 35.6.2.5 allows the direction to change while
    /// the counter runs.
    static bool count_down(bool down, uint32_t spins = 0xFFFFu) {
        if (down) {
            regs().TC_CTRLBSET = TC_CTRLBSET_DIR_Msk;
        } else {
            regs().TC_CTRLBCLR = TC_CTRLBCLR_DIR_Msk;
        }
        return sync_wait(TC_SYNCBUSY_CTRLB_Msk, spins);
    }
    static bool counting_down() {
        return (regs().TC_CTRLBSET & TC_CTRLBSET_DIR_Msk) != 0u;
    }

    // ---- the counter -------------------------------------------------------
    //
    // READING COUNT IS A COMMAND (35.6.8): READSYNC, then the two waits,
    // then the load. `*_raw()` skips it and says so.
    //
    // AND THE COMMAND IS ISSUED TWICE, because the silicon gives no
    // other honest way. Measured (tc_readsync_probe, designed from
    // test_samc_sleep's one-behind finding): after a READSYNC,
    // SYNCBUSY.CTRLB stands for the command's own crossing and falls -
    // and the COUNT shadow lands about HALF A COUNTER-CLOCK PERIOD
    // LATER, with NO SYNCBUSY bit advertising it (SYNCBUSY.COUNT never
    // rises for a READSYNC; it is the WRITE's bit). A single-command
    // read therefore returns the PREVIOUS command's snapshot: four
    // consecutive reads of a 32 kHz pair that had run six milliseconds
    // gave 0, 216, 221, 225. The second command's own crossing covers
    // the first's landing gap, so the double-command read below returns
    // the count AT THIS CALL'S ENTRY - measured 224, 233, 241, 249 on
    // the same setup, the first value finally current. THE PRICE is two
    // crossings per read: ~242 us at a 32.768 kHz counter clock (vs
    // ~117 single), unmeasurably small at 48 MHz. A caller measuring an
    // interval between two of its own reads never needed the fix (the
    // lag cancelled) and now simply pays the double crossing; `*_raw()`
    // still skips everything and says so. AND THE PIPELINE IS NEVER
    // PRIMED - measured, because it was worth refuting: after a double
    // read, a later SINGLE command returns the PREVIOUS read's second
    // snapshot, stale by the whole inter-read gap (233 against a true
    // 464 after a 6 ms gap; identical on the TCC). Two commands per
    // call, always - no first-read-double-then-singles shortcut
    // exists. docs/samc/tc.md carries the numbers.

    static bool read_sync(uint32_t spins = 0xFFFFu) {
        return command(TcCommand::read_sync, spins) &&
               sync_wait(TC_SYNCBUSY_COUNT_Msk, spins) &&
               command(TcCommand::read_sync, spins) &&
               sync_wait(TC_SYNCBUSY_COUNT_Msk, spins);
    }

    static uint8_t count8_raw() { return regs8().TC_COUNT; }
    static uint16_t count16_raw() { return regs().TC_COUNT; }
    static uint32_t count32_raw() { return regs32().TC_COUNT; }

    static uint8_t count8(uint32_t spins = 0xFFFFu) {
        (void)read_sync(spins);
        return count8_raw();
    }
    static uint16_t count16(uint32_t spins = 0xFFFFu) {
        (void)read_sync(spins);
        return count16_raw();
    }
    static uint32_t count32(uint32_t spins = 0xFFFFu) {
        (void)read_sync(spins);
        return count32_raw();
    }

    /// Writing COUNT is write-synchronized and takes priority over
    /// count/clear/reload (35.6.2.5).
    static bool set_count8(uint8_t v, uint32_t spins = 0xFFFFu) {
        regs8().TC_COUNT = v;
        return sync_wait(TC_SYNCBUSY_COUNT_Msk, spins);
    }
    static bool set_count16(uint16_t v, uint32_t spins = 0xFFFFu) {
        regs().TC_COUNT = v;
        return sync_wait(TC_SYNCBUSY_COUNT_Msk, spins);
    }
    static bool set_count32(uint32_t v, uint32_t spins = 0xFFFFu) {
        regs32().TC_COUNT = v;
        return sync_wait(TC_SYNCBUSY_COUNT_Msk, spins);
    }

    // ---- period: 8-BIT MODE ONLY (35.6.2.4) --------------------------------

    static uint8_t period8() { return regs8().TC_PER; }
    static bool set_period8(uint8_t v, uint32_t spins = 0xFFFFu) {
        regs8().TC_PER = v;
        return sync_wait(TC_SYNCBUSY_PER_Msk, spins);
    }
    /// The buffered period: taken at the next UPDATE condition.
    static bool set_period_buffer8(uint8_t v, uint32_t spins = 0xFFFFu) {
        regs8().TC_PERBUF = v;
        return sync_wait(TC_SYNCBUSY_PER_Msk, spins);
    }

    // ---- compare / capture -------------------------------------------------

    static uint8_t cc8(uint8_t ch) { return regs8().TC_CC[ch]; }
    static uint16_t cc16(uint8_t ch) { return regs().TC_CC[ch]; }
    static uint32_t cc32(uint8_t ch) { return regs32().TC_CC[ch]; }

    static bool set_cc8(uint8_t ch, uint8_t v, uint32_t spins = 0xFFFFu) {
        regs8().TC_CC[ch] = v;
        return sync_wait(TC_SYNCBUSY_CC0_Msk << ch, spins);
    }
    static bool set_cc16(uint8_t ch, uint16_t v, uint32_t spins = 0xFFFFu) {
        regs().TC_CC[ch] = v;
        return sync_wait(TC_SYNCBUSY_CC0_Msk << ch, spins);
    }
    static bool set_cc32(uint8_t ch, uint32_t v, uint32_t spins = 0xFFFFu) {
        regs32().TC_CC[ch] = v;
        return sync_wait(TC_SYNCBUSY_CC0_Msk << ch, spins);
    }

    /// The double-buffered write: taken at the next UPDATE condition,
    /// which is what keeps a live PWM glitch-free (35.6.2.7).
    static bool set_cc_buffer8(uint8_t ch, uint8_t v, uint32_t spins = 0xFFFFu) {
        regs8().TC_CCBUF[ch] = v;
        return sync_wait(TC_SYNCBUSY_CC0_Msk << ch, spins);
    }
    static bool set_cc_buffer16(uint8_t ch, uint16_t v, uint32_t spins = 0xFFFFu) {
        regs().TC_CCBUF[ch] = v;
        return sync_wait(TC_SYNCBUSY_CC0_Msk << ch, spins);
    }

    // ---- status ------------------------------------------------------------

    static uint8_t status() { return regs().TC_STATUS; }
    static bool stopped() { return (status() & TC_STATUS_STOP_Msk) != 0u; }
    /// STATUS.SLAVE: this instance is the client half of a 32-bit pair,
    /// and "normal access to the Client COUNT and CCx registers is not
    /// allowed" (35.6.2.4).
    static bool is_client() { return (status() & TC_STATUS_SLAVE_Msk) != 0u; }
    static bool period_buffer_valid() {
        return (status() & TC_STATUS_PERBUFV_Msk) != 0u;
    }
    static bool cc_buffer_valid(uint8_t ch) {
        return (status() & (TC_STATUS_CCBUFV0_Msk << ch)) != 0u;
    }

    /**
     * Clear a buffer-valid flag - TWICE, and that is ERRATUM 1.20.3,
     * live on every revision of this family: "SYNCBUSY flag is released
     * before the PERBUF/CCBUFx register is restored to its appropriate
     * value", and the documented workaround is to clear the flag
     * successively twice. STATUS.CCBUFVx is write-synchronized, so each
     * clear waits.
     */
    static bool clear_buffer_valid(uint8_t mask, uint32_t spins = 0xFFFFu) {
        regs().TC_STATUS = mask;
        if (!sync_wait(TC_SYNCBUSY_STATUS_Msk, spins)) {
            return false;
        }
        regs().TC_STATUS = mask;
        return sync_wait(TC_SYNCBUSY_STATUS_Msk, spins);
    }

    // ---- interrupts --------------------------------------------------------

    static constexpr uint8_t overflow_flag = TC_INTFLAG_OVF_Msk;
    static constexpr uint8_t error_flag = TC_INTFLAG_ERR_Msk;
    static constexpr uint8_t match_flag(uint8_t ch) {
        return static_cast<uint8_t>(TC_INTFLAG_MC0_Msk << ch);
    }

    static uint8_t flags() { return regs().TC_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().TC_INTFLAG = mask; }
    static uint8_t armed() { return regs().TC_INTENSET; }
    static void arm(uint8_t mask) { regs().TC_INTENSET = mask; }
    static void disarm(uint8_t mask) { regs().TC_INTENCLR = mask; }

    /**
     * The ISR body; the app binds TCn_Handler.
     *
     * Masked with INTENSET like every other one-vector-many-sources
     * block in this stratum. NOTE for a CAPTURE handler: reading CCx is
     * what empties the FIFO stage and lets CCBUFx move up (35.6.2.8), so
     * a handler that clears MCx without reading CCx throws the reading
     * away.
     *
     * AND THE OTHER HALF OF THE SAME FACT, measured
     * (test_samc_timer_dma, docs/samc/tc.md): because CCx has CCBUFx
     * behind it, ONE read taken after the signal under test has changed
     * hands back a value the PREVIOUS arrangement captured. A reader
     * that has just reconfigured something drains both stages and then
     * takes a whole fresh capture; a reader keeping up with a running
     * stream never notices.
     */
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t p = static_cast<uint8_t>(flags() & armed());
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    /// DBGCTRL.DBGRUN: keep counting while the CPU is halted.
    static void debug_run(bool on) {
        regs().TC_DBGCTRL = static_cast<uint8_t>(on ? TC_DBGCTRL_DBGRUN_Msk : 0u);
    }

private:
    static tc_registers_t& regs_union() {
        if constexpr (n == 0) {
            return *TC0_REGS;
        } else if constexpr (n == 1) {
            return *TC1_REGS;
        } else if constexpr (n == 2) {
            return *TC2_REGS;
        } else if constexpr (n == 3) {
            return *TC3_REGS;
#ifdef TC4_REGS
        } else if constexpr (n == 4) {
            return *TC4_REGS;
#endif
        } else {
            return *TC0_REGS;
        }
    }

    static constexpr uint32_t apb_mask() {
        if constexpr (n == 0) {
            return MCLK_APBCMASK_TC0_Msk;
        } else if constexpr (n == 1) {
            return MCLK_APBCMASK_TC1_Msk;
        } else if constexpr (n == 2) {
            return MCLK_APBCMASK_TC2_Msk;
        } else if constexpr (n == 3) {
            return MCLK_APBCMASK_TC3_Msk;
#ifdef MCLK_APBCMASK_TC4_Msk
        } else if constexpr (n == 4) {
            return MCLK_APBCMASK_TC4_Msk;
#endif
        } else {
            return 0u;
        }
    }
    static constexpr uint32_t pair_apb_mask() {
        if constexpr (n == 0) {
            return MCLK_APBCMASK_TC1_Msk;
        } else if constexpr (n == 2) {
            return MCLK_APBCMASK_TC3_Msk;
        } else {
            return 0u;
        }
    }

};

// =============================================================================
// Pad to waveform output: samc/device_tables.hpp is the authority
// =============================================================================
//
// `tc_wo_code(port, pin)` ((tc << 4) | wo, -1 for a pad that carries
// none) and the `tc_wo_exists<L, N>` probe live in the reserve
// (device_tables.hpp), generated symbol by symbol from the device
// header's own `PIN_P<pad>E_TC<n>_WO<k>` constants.

/**
 * One waveform output reached through its pad.
 *
 *   using LedPin = brio::Pin<'B', 23>;
 *   using LedWo = brio::TcWo<LedPin>;    // TC3, WO1 - from the header
 *   LedWo::claim();                      // PMUX function E
 *
 * The pad is the input side of a capture-on-pin channel too, which is
 * why `claim()` turns the input buffer on: the mux does not.
 */
template <class P>
struct TcWo {
    TcWo() = delete;

    static_assert(tc_wo_exists<P::port_letter, P::pin_number>,
                  "this pad carries no TC waveform output on this device (the "
                  "device header defines no PIN_P<pad>E_TC<n>_WO<k> for it)");

    using pin = P;
    static constexpr uint8_t timer =
        static_cast<uint8_t>(tc_wo_code(P::port_letter, P::pin_number) >> 4);
    static constexpr uint8_t channel =
        static_cast<uint8_t>(tc_wo_code(P::port_letter, P::pin_number) & 0xFu);

    /// Hand the pad to the timer (peripheral function E on this family)
    /// with the input buffer on, so the level can be read back and so a
    /// capture-on-pin channel sees the pin.
    static void claim() {
        P::function(PinFunction::e, PinConfig{.input_enable = true});
    }
    static void release() { P::release(); }
};

// =============================================================================
// Tasks
// =============================================================================

/**
 * TcPwm<Tc, ch>: one 16-bit PWM channel, a `PwmChannel` (max 65535).
 *
 * NPWM in COUNT16 fixes TOP at MAX, so the frequency is
 * GCLK_TC / prescaler / 65536 and the only knob is the duty - which is
 * exactly the PwmChannel contract's division of labour: frequency
 * belongs to the timer instance, duty to the channel.
 *
 * `duty()` writes CCBUFx, not CCx: the buffered write is taken at the
 * next UPDATE and is what keeps a live change glitch-free (35.6.2.7).
 */
template <class T, uint8_t ch>
struct TcPwm {
    TcPwm() = delete;
    static_assert(ch < T::cc_count, "this TC has two compare/capture channels");

    static constexpr uint16_t max = 0xFFFFu;

    /// Bring the timer up as a 16-bit NPWM generator. The caller has
    /// already called T::init() and claimed the pad.
    static bool setup(TcPrescaler prescaler = TcPrescaler::div1,
                      uint32_t spins = 0xFFFFu) {
        if (!T::configure(TcConfig{.mode = TcMode::count16,
                                   .prescaler = prescaler,
                                   .waveform = TcWaveform::normal_pwm},
                          spins)) {
            return false;
        }
        return T::set_cc16(ch, 0, spins) && T::enable(true, spins);
    }

    static void duty(uint16_t v) { (void)T::set_cc_buffer16(ch, v); }
};

/**
 * TcPwm8<Tc, ch, top>: the 8-bit PWM channel, a `PwmChannel` whose
 * `max` is the PERIOD the timer is set to - which is the whole reason
 * 8-bit mode has a PER register at all (35.6.2.4).
 *
 * `top` is a template parameter because PwmChannel requires `max` to be
 * a compile-time constant, and because a channel whose full-scale can
 * move under it is not one value a generic actuator can scale against.
 */
template <class T, uint8_t ch, uint8_t top>
struct TcPwm8 {
    TcPwm8() = delete;
    static_assert(ch < T::cc_count, "this TC has two compare/capture channels");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    static bool setup(TcPrescaler prescaler = TcPrescaler::div1,
                      uint32_t spins = 0xFFFFu) {
        if (!T::configure(TcConfig{.mode = TcMode::count8,
                                   .prescaler = prescaler,
                                   .waveform = TcWaveform::normal_pwm},
                          spins)) {
            return false;
        }
        return T::set_period8(top, spins) && T::set_cc8(ch, 0, spins) &&
               T::enable(true, spins);
    }

    static void duty(uint16_t v) {
        (void)T::set_cc_buffer8(ch, static_cast<uint8_t>(v > max ? max : v));
    }
};

/**
 * TcPeriodMeter<Tc>: the period AND the pulse width of an input signal
 * in one capture sequence (EVACT = PPW, 35.6.2.8.2).
 *
 * Both channels have to be enabled - "consequently, both channels must
 * be enabled to fully characterize the input" - and the source is the
 * EVENT input, because that is what the chapter's PPW/PWP action reads.
 * The stimulus therefore comes through EVSYS on an ASYNCHRONOUS channel
 * (35.6.2.8 note 2), typically from an EIC line.
 *
 * `period_ticks()` and `width_ticks()` READ CCx, which is what empties
 * the capture FIFO stage; a handler that clears the flag without
 * reading throws the reading away.
 */
template <class T>
struct TcPeriodMeter {
    TcPeriodMeter() = delete;

    /// Both channels capturing, period into CC0 and width into CC1.
    static bool setup(TcPrescaler prescaler = TcPrescaler::div1,
                      bool wrap_on_falling = false, uint32_t spins = 0xFFFFu) {
        const TcConfig cfg{.mode = TcMode::count16,
                           .prescaler = prescaler,
                           .capture_enable = 0x3};
        if (!T::configure(cfg, spins)) {
            return false;
        }
        if (!T::event_config(cfg, TcEventConfig{
                                      .action = TcEventAction::period_pulse_width,
                                      .input_enable = true,
                                      .invert_input = wrap_on_falling})) {
            return false;
        }
        return T::enable(true, spins);
    }

    static uint16_t period_ticks() { return T::cc16(0); }
    static uint16_t width_ticks() { return T::cc16(1); }

    /// INTFLAG.MC0 is the one that says a whole period landed.
    static constexpr uint8_t period_flag = T::match_flag(0);
    static constexpr uint8_t width_flag = T::match_flag(1);
    /// INTFLAG.ERR: a capture arrived while the previous one was still
    /// unread, and the new value was DROPPED (35.6.2.8.2).
    static constexpr uint8_t overrun_flag = T::error_flag;
};

/**
 * TcPulseWidthMeter<Tc>: the high time of an input signal alone
 * (EVACT = PW, 35.6.2.8.3). The counter is cleared and stopped on the
 * falling edge and restarts on the rising one, so CC0 holds the width
 * and the timer spends the gaps stopped.
 */
template <class T>
struct TcPulseWidthMeter {
    TcPulseWidthMeter() = delete;

    static bool setup(TcPrescaler prescaler = TcPrescaler::div1,
                      bool invert = false, uint32_t spins = 0xFFFFu) {
        const TcConfig cfg{.mode = TcMode::count16,
                           .prescaler = prescaler,
                           .capture_enable = 0x1};
        if (!T::configure(cfg, spins)) {
            return false;
        }
        if (!T::event_config(cfg, TcEventConfig{
                                      .action = TcEventAction::pulse_width,
                                      .input_enable = true,
                                      .invert_input = invert})) {
            return false;
        }
        return T::enable(true, spins);
    }

    static uint16_t width_ticks() { return T::cc16(0); }
    static constexpr uint8_t width_flag = T::match_flag(0);
    static constexpr uint8_t overrun_flag = T::error_flag;
};

} // namespace brio
