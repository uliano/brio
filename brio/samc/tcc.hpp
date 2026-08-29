/*
 * tcc.hpp
 *
 * The SAM C21 Timer/Counters for Control applications (DS60001479M
 * ch. 36): three instances of the family's richest timer - a counter
 * with a real period register in every mode, up to four double-buffered
 * compare/capture channels, seven waveform modes, ramp operation,
 * dithering, and a waveform-extension stage (output matrix, dead-time
 * insertion, swap, pattern generation) behind a two-level fault system.
 *
 *  Tcc<n>            the RESOURCE: the whole chapter's register surface.
 *
 *  TccWo<Pin, fn>    one waveform output reached through the pad that
 *                    carries it - and the FUNCTION is part of the key,
 *                    see fact 2 below.
 *
 *  TccPwm<T,ch,top>  tasks over the resource: a single-slope PWM channel
 *  TccPairPwm<...>   satisfying util/pwm_channel.hpp's PwmChannel, and
 *                    the complementary pair with dead-time insertion.
 *
 * NINE FACTS THAT SHAPE THE FILE - the last two of which are not in the
 * chapter at all, and were measured (docs/samc/tcc.md carries the
 * numbers).
 *
 * 1. THE THREE INSTANCES ARE NOT COPIES OF EACH OTHER, which is the
 *    first thing that separates this chapter from the TC's. TCC0 is a
 *    24-bit counter with four channels, eight outputs and all five
 *    extension units; TCC1 is 24-bit with two channels, four outputs and
 *    only pattern generation and dithering; TCC2 is a 16-bit counter
 *    with two channels, two outputs and no extension at all. Every one
 *    of those numbers is a `TCCn_*` constant of the device header, read
 *    through samc/device_tables.hpp, and a configuration that asks an
 *    instance for a unit it does not have is REFUSED AT COMPILE TIME
 *    rather than written into a reserved bit.
 *
 * 2. THE PAD MAP IS KEYED BY PAD **AND FUNCTION**. A TC waveform output
 *    lives on exactly one peripheral function, so `TcWo<Pin>` needs no
 *    more than the pad. A TCC output does not: PA08 is TCC0/WO0 under
 *    function E and TCC1/WO2 under function F, PA16 is TCC2/WO0 under E
 *    and TCC0/WO6 under F, and the two are different outputs of
 *    different instances. `TccWo<Pin, PinFunction::f>` therefore takes
 *    the function, and the device header's own
 *    `PIN_P<pad><fn>_TCC<n>_WO<k>` says whether that pair exists.
 *
 * 3. ENABLE-PROTECTION AND WRITE-SYNCHRONIZATION SPLIT DIFFERENTLY HERE
 *    THAN IN THE TC, and the difference is useful rather than pedantic.
 *    Enable-protected (36.6.2.1): CTRLA except RUNSTDBY/ENABLE/SWRST,
 *    FCTRLA, FCTRLB, WEXCTRL, DRVCTRL, EVCTRL - `configure()`,
 *    `fault()`, `wave_extension()`, `drive()` and `event_config()`
 *    refuse while the TCC is enabled. Write-synchronized and NOT
 *    enable-protected: CTRLB, STATUS, COUNT, PATT, **WAVE**, PER and
 *    CC/CCBUF. So the waveform mode, the polarity, the swap and the
 *    circular-buffer bits can all be changed UNDER A RUNNING TIMER,
 *    where the TC's WAVE cannot.
 *
 * 4. READING COUNT IS A COMMAND. COUNT is read-synchronized "on demand
 *    through the READSYNC command" (36.6.7), exactly as in the TC:
 *    write CTRLBSET.CMD = READSYNC, wait out SYNCBUSY.CTRLB and
 *    SYNCBUSY.COUNT, then read. `count()` does it; `count_raw()` skips
 *    it and says so in its name.
 *
 * 5. THE FAULT INPUTS ARE THE EVENT INPUTS. Recoverable Fault A and B
 *    are the channel event inputs MCE0 and MCE1 (36.6.3.5); the two
 *    non-recoverable faults are the counter event inputs TCE0 and TCE1,
 *    selected by EVACT0/EVACT1 = FAULT (36.6.3.6). Both must be carried
 *    on ASYNCHRONOUS event channels - the chapter says so for the fault
 *    use, and erratum 1.21.9 says so for every use. That is a
 *    CALLER OBLIGATION this header states and cannot enforce: evsys.hpp
 *    owns the fabric, this file owns only the codes it publishes.
 *
 *    The same obligation, in the other direction: a recoverable fault
 *    needs its channel's incoming event ENABLED (EVCTRL.MCEIx), because
 *    the fault input is that event input. `tcc_fault_input_bit()` names
 *    the bit; docs/samc/tcc.md carries what the bench measured happens
 *    without it.
 *
 * 6. THE PERIOD REGISTER IS ALWAYS THERE, unlike the TC's, and PER is
 *    TOP in every waveform mode but MFRQ - where CC0 is (36.6.2.5.1,
 *    table 36-2). So a TCC PWM channel has an arbitrary period at any
 *    counter width, which is what makes `TccPwm<T, ch, top>` a
 *    PwmChannel with a `max` the caller chose.
 *
 * 7. DITHERING SHARES THE LOW BITS OF COUNT, PER AND CCx. With
 *    CTRLA.RESOLUTION = DITH4/5/6 the low 4, 5 or 6 bits of those
 *    registers stop being part of the value and become the number of
 *    extra clock cycles to spread over a frame of 16, 32 or 64 PWM
 *    cycles (36.6.3.3). `tcc_dither()` packs the pair; a value written
 *    without it while a dithering resolution is selected is off by a
 *    factor of 16, 32 or 64, silently.
 *
 * 8. A BUFFERED WRITE'S SYNCBUSY BIT DOES NOT CLEAR UNTIL THE UPDATE
 *    CONSUMES THE BUFFER - measured, and nowhere in the chapter.
 *    SYNCBUSY.CCx is shared between CCx and CCBUFx, and after a write to
 *    CCBUFx it stands for the rest of the waveform period (up to 1299 us
 *    of a 1333 us period, measured) and FOREVER while CTRLB.LUPD is set.
 *    A second write into that window is DISCARDED by the silicon - also
 *    measured. So every buffered setter here REFUSES instead of waiting:
 *    `set_cc_buffer()`, `set_period_buffer()` and `pattern_buffer()`
 *    return false when the previous value has not been taken yet, and
 *    return at once in every case. Waiting would put a whole PWM period
 *    inside `TccPwm::duty()`, which is not what a PwmChannel promises.
 *    The direct setters (`set_cc()`, `set_period()`) keep their bounded
 *    wait, and are fast when nothing is pending - but they share that
 *    same SYNCBUSY bit, so a direct write issued while a buffered one is
 *    outstanding waits for the update too.
 *
 *    Its twin, from the same measurement: A READ OF CCx WHILE A BUFFERED
 *    WRITE IS PENDING RETURNS THE BUFFERED VALUE, not the one the
 *    waveform generator is using. The pad and the register disagree, and
 *    the pad is right. Read `cc_buffer_valid()` to know which you are
 *    looking at.
 *
 * 9. THE TWO CTRLB REGISTERS ARE NOT A SET/CLEAR PAIR FOR THE COMMAND
 *    FIELDS. DIR, LUPD and ONESHOT behave as one expects - a '1' to
 *    CTRLBSET sets, a '1' to CTRLBCLR clears - but CMD and IDXCMD are
 *    written as a VALUE, and "writing zero to this bit group has no
 *    effect" on BOTH halves (36.8.2, 36.8.3). So a command is issued
 *    through CTRLBSET and CANCELLED through CTRLBCLR, and a driver that
 *    cancels by writing zero to CTRLBSET cancels nothing at all. The
 *    bench found this with a RAMP2 index held forever; `command()` and
 *    `ramp_index_command()` route their `none`/`off` through CTRLBCLR.
 *
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F. This chapter
 * has ELEVEN items and SEVEN of them are live here.
 *
 *  LIVE:
 *  - 1.21.5 Advance Capture Mode: CAPTMIN/CAPTMAX/LOCMIN/LOCMAX/DERIV0
 *    do not work unless every UPPER channel is also in one of those
 *    modes. A caller obligation - "basic capture mode must be set in
 *    lower channel and advance capture mode in upper channel" - stated
 *    at `TccFaultCapture` and in docs/samc/tcc.md, since it is a
 *    property of the whole channel set and not of one register write.
 *  - 1.21.6 SYNCBUSY: clearing a STATUS.xxBUFV flag releases SYNCBUSY
 *    before the register behind it is restored. `clear_buffer_valid()`
 *    clears TWICE, which is the whole workaround (and the exact twin of
 *    the TC's 1.20.3).
 *  - 1.21.7 Dithering Mode: dithering plus external RETRIGGER events
 *    stretches or shrinks pulses. Caller obligation: do not retrigger a
 *    dithering TCC. Stated here and in the doc; not refusable, because
 *    the retrigger may come from an event channel this file cannot see.
 *  - 1.21.8 PERBUF: in DOWN-COUNTING mode CTRLB.LUPD is said not to
 *    protect PER from being updated out of PERBUF, with no workaround.
 *    IT DID NOT REPRODUCE HERE: with the WAVEFORM as the witness - the
 *    overflow rate, since a register read cannot be one, see fact 8 -
 *    the lock held in NPWM up, NPWM down and NFRQ down alike. What DOES
 *    happen in all three is fact 8's register read: PER reads back the
 *    buffered value while the counter is still using the old one, which
 *    is exactly what a reader checking PER would report as 1.21.8.
 *    docs/samc/tcc.md carries the numbers; the item is left standing as
 *    unreproduced, not as disproved.
 *  - 1.21.9 TCC with EVSYS in SYNC/RESYNC: the TCC is not compatible
 *    with a synchronous or resynchronized event channel at all. Caller
 *    obligation (fact 5): every channel feeding a TCC user is
 *    asynchronous.
 *  - 1.21.10 ALOCK: the auto-lock feature is simply not functional, and
 *    there is no workaround. `TccConfig::auto_lock` is therefore
 *    REFUSED by `tcc_config_valid()` - the one erratum in this chapter
 *    that can be turned into a compile error, and it is.
 *  - 1.21.11 Overflow DMA trigger: the OVF DMA trigger in DMAOS mode is
 *    broken FOR RAMP2C AND RAMP2CS ONLY. Those ramp modes are variant-L
 *    (36.8.17) and this family does not have them, so the item cannot
 *    be reached here - see `TccRamp::ramp2_critical`, which this driver
 *    refuses for that independent reason.
 *
 *  NOT THIS SILICON (revision B only - do not code around them):
 *  1.21.1 Circular Buffer in standby, 1.21.2 RAMP2 double restart,
 *  1.21.3 CAPTMARK, 1.21.4 Capture Overflow within 3+3 clocks.
 *
 * NOT BUILT (docs/samc/tcc.md carries the list): DMA-driven operation
 * (the trigger ids are published, nothing wires them), the debug-fault
 * state as an exercised path (FDDBD is exposed, a halted debugger is
 * not something a bench suite can stage), and sleep, which the power
 * pass owns together with RUNSTDBY.
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

/// CTRLA.PRESCALER. Bypassed when the counter counts events (36.6.2.3).
enum class TccPrescaler : uint8_t {
    div1 = TCC_CTRLA_PRESCALER_DIV1_Val,
    div2 = TCC_CTRLA_PRESCALER_DIV2_Val,
    div4 = TCC_CTRLA_PRESCALER_DIV4_Val,
    div8 = TCC_CTRLA_PRESCALER_DIV8_Val,
    div16 = TCC_CTRLA_PRESCALER_DIV16_Val,
    div64 = TCC_CTRLA_PRESCALER_DIV64_Val,
    div256 = TCC_CTRLA_PRESCALER_DIV256_Val,
    div1024 = TCC_CTRLA_PRESCALER_DIV1024_Val,
};

constexpr uint16_t tcc_prescaler_divisor(TccPrescaler p) {
    switch (p) {
    case TccPrescaler::div1: return 1;
    case TccPrescaler::div2: return 2;
    case TccPrescaler::div4: return 4;
    case TccPrescaler::div8: return 8;
    case TccPrescaler::div16: return 16;
    case TccPrescaler::div64: return 64;
    case TccPrescaler::div256: return 256;
    case TccPrescaler::div1024: return 1024;
    }
    return 1;
}

/// CTRLA.PRESCSYNC: where a retrigger puts the counter relative to the
/// prescaled clock, and whether the prescaler itself is reset with it.
enum class TccPrescalerSync : uint8_t {
    gclk = TCC_CTRLA_PRESCSYNC_GCLK_Val,
    prescaler = TCC_CTRLA_PRESCSYNC_PRESC_Val,
    resync = TCC_CTRLA_PRESCSYNC_RESYNC_Val,
};

/// CTRLA.RESOLUTION: the dithering frame (36.6.3.3). See `tcc_dither()`
/// - with anything but `none` selected, the low bits of COUNT, PER and
/// CCx are no longer part of the value.
enum class TccResolution : uint8_t {
    none = TCC_CTRLA_RESOLUTION_NONE_Val,
    dither16 = TCC_CTRLA_RESOLUTION_DITH4_Val,   ///< DITH4: 16-cycle frame
    dither32 = TCC_CTRLA_RESOLUTION_DITH5_Val,   ///< DITH5: 32-cycle frame
    dither64 = TCC_CTRLA_RESOLUTION_DITH6_Val,   ///< DITH6: 64-cycle frame
};

/// How many low bits of COUNT/PER/CCx a resolution takes for itself.
constexpr uint8_t tcc_dither_bits(TccResolution r) {
    switch (r) {
    case TccResolution::none: return 0;
    case TccResolution::dither16: return 4;
    case TccResolution::dither32: return 5;
    case TccResolution::dither64: return 6;
    }
    return 0;
}

/// The frame length in PWM cycles: 1, 16, 32 or 64.
constexpr uint8_t tcc_dither_frame(TccResolution r) {
    return static_cast<uint8_t>(1u << tcc_dither_bits(r));
}

/**
 * Pack a compare (or period) value and its dithering cycle count into
 * the one register they share (36.8.18, 36.8.19).
 *
 * The average period or pulse width becomes `value + extra/frame`
 * clocks, which is the whole point of the feature: a fractional
 * compare value with no fractional hardware.
 */
constexpr uint32_t tcc_dither(uint32_t value, uint8_t extra, TccResolution r) {
    const uint8_t bits = tcc_dither_bits(r);
    const uint32_t mask = static_cast<uint32_t>(tcc_dither_frame(r)) - 1u;
    return (value << bits) | (static_cast<uint32_t>(extra) & mask);
}

/// WAVE.WAVEGEN (36.8.17, table 36-2). PER is TOP in every mode but
/// `match_frequency`, where CC0 is.
enum class TccWaveform : uint8_t {
    normal_frequency = TCC_WAVE_WAVEGEN_NFRQ_Val,
    match_frequency = TCC_WAVE_WAVEGEN_MFRQ_Val,
    normal_pwm = TCC_WAVE_WAVEGEN_NPWM_Val,
    /// Dual-slope, non-aligned centred pulses: CCx sets the up-counting
    /// edge and CC(x + CC_NUM/2) the down-counting one (36.6.2.5.7).
    dual_slope_critical = TCC_WAVE_WAVEGEN_DSCRITICAL_Val,
    /// Dual-slope, interrupt/event at ZERO.
    dual_slope_bottom = TCC_WAVE_WAVEGEN_DSBOTTOM_Val,
    /// Dual-slope, interrupt/event at both ends. The UPDATE on TOP only
    /// happens with a circular buffer enabled (table 36-2, note 1).
    dual_slope_both = TCC_WAVE_WAVEGEN_DSBOTH_Val,
    /// Dual-slope, interrupt/event at TOP.
    dual_slope_top = TCC_WAVE_WAVEGEN_DSTOP_Val,
};

constexpr bool tcc_waveform_is_dual_slope(TccWaveform w) {
    return w == TccWaveform::dual_slope_critical ||
           w == TccWaveform::dual_slope_bottom ||
           w == TccWaveform::dual_slope_both || w == TccWaveform::dual_slope_top;
}

/// WAVE.RAMP (36.6.3.4). All three ramp modes require single-slope PWM.
enum class TccRamp : uint8_t {
    ramp1 = TCC_WAVE_RAMP_RAMP1_Val,
    ramp2_alternate = TCC_WAVE_RAMP_RAMP2A_Val,
    ramp2 = TCC_WAVE_RAMP_RAMP2_Val,
    /// RAMP2C: "only available in variant L devices" (36.8.17). The
    /// device header defines the encoding for the whole silicon family,
    /// this part is not one of those devices, and `tcc_wave_valid()`
    /// refuses it - see docs/samc/tcc.md for what writing it does here.
    ramp2_critical = TCC_WAVE_RAMP_RAMP2C_Val,
};

/// CTRLBSET.CMD - write-only, executed on the next prescaled clock, and
/// read back as zero once taken.
enum class TccCommand : uint8_t {
    none = TCC_CTRLBSET_CMD_NONE_Val,
    retrigger = TCC_CTRLBSET_CMD_RETRIGGER_Val,
    stop = TCC_CTRLBSET_CMD_STOP_Val,
    /// Force the buffered registers into their live counterparts. Acts
    /// independently of CTRLB.LUPD (36.6.2.6).
    update = TCC_CTRLBSET_CMD_UPDATE_Val,
    read_sync = TCC_CTRLBSET_CMD_READSYNC_Val,
    dma_one_shot = TCC_CTRLBSET_CMD_DMAOS_Val,
};

/// CTRLBSET.IDXCMD: force the next RAMP2/RAMP2A cycle to be A or B.
enum class TccRampIndexCommand : uint8_t {
    off = TCC_CTRLBSET_IDXCMD_DISABLE_Val,   ///< IDX toggles by itself
    force_b = TCC_CTRLBSET_IDXCMD_SET_Val,
    force_a = TCC_CTRLBSET_IDXCMD_CLEAR_Val,
    hold = TCC_CTRLBSET_IDXCMD_HOLD_Val,
};

/// EVCTRL.EVACT0: what the TCE0 counter event input DOES (36.8.10).
enum class TccEvent0Action : uint8_t {
    off = TCC_EVCTRL_EVACT0_OFF_Val,
    /// NOTE 36.6.2.4: with a retrigger action selected, ENABLING the TCC
    /// does not start it - the first event does.
    retrigger = TCC_EVCTRL_EVACT0_RETRIGGER_Val,
    count = TCC_EVCTRL_EVACT0_COUNTEV_Val,
    /// Start a stopped counter. Further events do not restart it.
    start = TCC_EVCTRL_EVACT0_START_Val,
    /// Increment regardless of CTRLB.DIR.
    increment = TCC_EVCTRL_EVACT0_INC_Val,
    /// Count on the ACTIVE STATE of an asynchronous event, one tick per
    /// prescaled clock for as long as it is asserted.
    count_while_active = TCC_EVCTRL_EVACT0_COUNT_Val,
    /// Capture the overflow times into a capture channel.
    stamp = TCC_EVCTRL_EVACT0_STAMP_Val,
    /// Non-recoverable fault 0 (36.6.3.6). Asynchronous events only.
    fault = TCC_EVCTRL_EVACT0_FAULT_Val,
};

/// EVCTRL.EVACT1: what the TCE1 counter event input does.
enum class TccEvent1Action : uint8_t {
    off = TCC_EVCTRL_EVACT1_OFF_Val,
    retrigger = TCC_EVCTRL_EVACT1_RETRIGGER_Val,
    /// The event LEVEL overrides CTRLB.DIR. Asynchronous events only.
    direction = TCC_EVCTRL_EVACT1_DIR_Val,
    stop = TCC_EVCTRL_EVACT1_STOP_Val,
    decrement = TCC_EVCTRL_EVACT1_DEC_Val,
    /// Period into CC0, pulse width into CC1.
    period_pulse_width = TCC_EVCTRL_EVACT1_PPW_Val,
    /// Period into CC1, pulse width into CC0 - the other order, and the
    /// only difference between the two.
    pulse_width_period = TCC_EVCTRL_EVACT1_PWP_Val,
    /// Non-recoverable fault 1. Asynchronous events only.
    fault = TCC_EVCTRL_EVACT1_FAULT_Val,
};

constexpr bool tcc_event1_is_capture(TccEvent1Action a) {
    return a == TccEvent1Action::period_pulse_width ||
           a == TccEvent1Action::pulse_width_period;
}

/// EVCTRL.CNTSEL: where in the counter cycle the CNT event and
/// interrupt land.
/// (36.8.10 spells value 0 "BEGIN"; the device header spells the same
/// encoding "START" and the header wins in code.)
enum class TccCountEvent : uint8_t {
    begin = TCC_EVCTRL_CNTSEL_START_Val,
    end = TCC_EVCTRL_CNTSEL_END_Val,
    between = TCC_EVCTRL_CNTSEL_BETWEEN_Val,
    boundary = TCC_EVCTRL_CNTSEL_BOUNDARY_Val,
};

/// WEXCTRL.OTMX: how the compare channels are spread over the outputs
/// (table 36-4). Only an instance with the output-matrix unit has it.
enum class TccOutputMatrix : uint8_t {
    /// 0x0: channels modulo the channel count - CC0 to WO0 and WO[N],
    /// CC1 to WO1 and WO[N+1], and so on.
    per_channel = 0,
    /// 0x1: channels modulo HALF the channel count - twice as many
    /// outputs for the lower channels (a full bridge on two channels).
    half = 1,
    /// 0x2: CC0 to every output (a stepper, with pattern generation).
    broadcast_cc0 = 2,
    /// 0x3: CC0 to the first output, CC1 to all the others.
    cc0_then_cc1 = 3,
};

// ---- the recoverable fault vocabulary ---------------------------------------

/// FCTRLn.SRC: which event input drives this fault (36.8.5).
enum class TccFaultSource : uint8_t {
    off = TCC_FCTRLA_SRC_DISABLE_Val,
    /// Fault A takes MCE0, fault B takes MCE1.
    event = TCC_FCTRLA_SRC_ENABLE_Val,
    inverted_event = TCC_FCTRLA_SRC_INVERT_Val,
    /// The OTHER fault's state at the end of the previous period.
    alternate = TCC_FCTRLA_SRC_ALTFAULT_Val,
};

/// FCTRLn.BLANK: which waveform edge starts the blanking window.
enum class TccFaultBlank : uint8_t {
    period_start = TCC_FCTRLA_BLANK_START_Val,
    rising_edge = TCC_FCTRLA_BLANK_RISE_Val,
    falling_edge = TCC_FCTRLA_BLANK_FALL_Val,
    both_edges = TCC_FCTRLA_BLANK_BOTH_Val,
};

/// FCTRLn.HALT: what a valid fault does to the counter.
enum class TccFaultHalt : uint8_t {
    off = TCC_FCTRLA_HALT_DISABLE_Val,
    /// Halt while the fault is present; resume by itself when it goes.
    hardware = TCC_FCTRLA_HALT_HW_Val,
    /// Halt until the fault goes AND software clears STATUS.FAULTn.
    software = TCC_FCTRLA_HALT_SW_Val,
    /// Escalate: treat this recoverable fault as a non-recoverable one.
    non_recoverable = TCC_FCTRLA_HALT_NR_Val,
};

/**
 * FCTRLn.CAPTURE: what the fault does to the selected capture channel.
 *
 * ERRATUM 1.21.5, LIVE ON EVERY REVISION: the five "advance" actions -
 * everything but `off` and `capture` - do not work unless every channel
 * ABOVE the selected one is also in one of them. The documented
 * arrangement is basic capture on the low channels and advance capture
 * on the high ones. This driver cannot check it (it is a property of
 * the whole channel set, written one register at a time), so it is
 * stated here and in docs/samc/tcc.md.
 */
enum class TccFaultCapture : uint8_t {
    off = TCC_FCTRLA_CAPTURE_DISABLE_Val,
    /// Timestamp on every valid fault edge.
    capture = TCC_FCTRLA_CAPTURE_CAPT_Val,
    capture_min = TCC_FCTRLA_CAPTURE_CAPTMIN_Val,
    capture_max = TCC_FCTRLA_CAPTURE_CAPTMAX_Val,
    local_min = TCC_FCTRLA_CAPTURE_LOCMIN_Val,
    local_max = TCC_FCTRLA_CAPTURE_LOCMAX_Val,
    /// Either extreme - the OR of local_min and local_max.
    extremum = TCC_FCTRLA_CAPTURE_DERIV0_Val,
    /// Capture with the ramp index as the MSB. Broken on revision B
    /// (erratum 1.21.3), fine here.
    capture_marked = TCC_FCTRLA_CAPTURE_CAPTMARK_Val,
};

constexpr bool tcc_capture_is_advanced(TccFaultCapture c) {
    return c != TccFaultCapture::off && c != TccFaultCapture::capture &&
           c != TccFaultCapture::capture_marked;
}

/// Which of the two recoverable faults a verb addresses.
enum class TccFault : uint8_t { a = 0, b = 1 };

/// The EVCTRL.MCEIx bit a recoverable fault's input needs enabled: the
/// fault input IS that channel's event input (36.6.3.5).
constexpr uint8_t tcc_fault_input_bit(TccFault f) {
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(f));
}

// =============================================================================
// The configuration structs
// =============================================================================

/**
 * CTRLA and the two CTRLB mode bits: everything written once, before
 * the TCC is enabled, and enable-protected as a group (36.6.2.1).
 */
struct TccConfig {
    TccPrescaler prescaler = TccPrescaler::div1;
    TccPrescalerSync prescaler_sync = TccPrescalerSync::gclk;
    TccResolution resolution = TccResolution::none;

    /// CTRLA.CPTENx, bit x = channel x captures instead of comparing.
    uint8_t capture_enable = 0;

    bool run_standby = false;
    /// CTRLA.DMAOS: one DMA trigger per DMAOS command instead of one per
    /// cycle (36.6.5.1).
    bool dma_one_shot = false;
    /// CTRLA.MSYNC: this CLIENT instance's counter is driven by its host
    /// (36.6.4). Legal only where `TCCn_MASTER_SLAVE_MODE` is 2.
    bool host_sync = false;

    /**
     * CTRLA.ALOCK - and it is REFUSED. Erratum 1.21.10, live on every
     * revision of this family: "ALOCK feature is not functional",
     * workaround "None". The field exists so that a caller asking for it
     * gets a compile error naming the erratum instead of a bit that does
     * nothing.
     */
    bool auto_lock = false;

    /// CTRLB.DIR: count down from TOP rather than up from zero.
    bool count_down = false;
    /// CTRLB.ONESHOT: stop at the next overflow/underflow (36.6.3.1).
    bool one_shot = false;
    /// CTRLB.LUPD: hold the buffered registers back until an UPDATE
    /// command. NOTE erratum 1.21.8: this does NOT protect PER while
    /// counting down.
    bool lock_update = false;
};

/**
 * WAVE, which is write-synchronized but NOT enable-protected - so all
 * of it can change under a running counter (fact 3).
 */
struct TccWaveConfig {
    TccWaveform waveform = TccWaveform::normal_frequency;
    TccRamp ramp = TccRamp::ramp1;

    /// WAVE.POLx, bit x = channel x. In PWM modes it flips which edge of
    /// the cycle the output is set on (table 36-3); in the two frequency
    /// modes it is the output's initial state.
    uint8_t polarity = 0;
    /// WAVE.SWAPx, bit x = DTI slice x: exchange the low-side and
    /// high-side pins of that pair. Needs the swap unit.
    uint8_t swap = 0;
    /// WAVE.CICCENx: channel x's CC value is copied BACK into CCBUFx at
    /// every update - the circular buffer of 36.6.3.2.
    uint8_t circular_cc = 0;
    /// WAVE.CIPEREN: the same for PER and PERBUF.
    bool circular_period = false;
};

/**
 * WEXCTRL: the output matrix and the dead-time insertion unit. Both are
 * optional per instance, and enable-protected.
 */
struct TccWaveExtConfig {
    TccOutputMatrix output_matrix = TccOutputMatrix::per_channel;
    /// WEXCTRL.DTIENx, bit x = slice x: override outputs WO[x] and
    /// WO[x + WO_NUM/2] with the complementary pair built from matrix
    /// output x.
    uint8_t dead_time_enable = 0;
    /// WEXCTRL.DTLS / DTHS, in GCLK_TCC cycles - NOT prescaled cycles
    /// (36.8.7 says GCLK_TCC, and the bench measured which is true).
    uint8_t dead_time_low = 0;
    uint8_t dead_time_high = 0;
};

/**
 * DRVCTRL: the per-output inversion, the non-recoverable fault output
 * state, and the two non-recoverable fault input filters.
 */
struct TccDriveConfig {
    /// DRVCTRL.INVENx, bit x = output x inverted.
    uint8_t invert = 0;
    /// DRVCTRL.NREx: output x is driven to NRVx under a non-recoverable
    /// fault (a clear bit tri-states it instead).
    uint8_t fault_output_enable = 0;
    /// DRVCTRL.NRVx: the level output x is driven to.
    uint8_t fault_output_value = 0;
    /// DRVCTRL.FILTERVAL0/1, in prescaled clocks. MUST be zero when the
    /// matching TCE input is carried on a synchronous event channel -
    /// which erratum 1.21.9 forbids anyway.
    uint8_t filter0 = 0;
    uint8_t filter1 = 0;
};

/// EVCTRL, enable-protected with the rest.
struct TccEventConfig {
    TccEvent0Action action0 = TccEvent0Action::off;
    TccEvent1Action action1 = TccEvent1Action::off;

    bool input0_enable = false;   ///< TCEI0
    bool input1_enable = false;   ///< TCEI1
    bool invert0 = false;         ///< TCINV0
    bool invert1 = false;         ///< TCINV1

    /// EVCTRL.MCEIx, bit x: channel x's incoming event is enabled. This
    /// is the bit a recoverable fault's input needs (fact 5).
    uint8_t match_in = 0;
    /// EVCTRL.MCEOx, bit x: channel x's match/capture leaves as an event.
    uint8_t match_out = 0;

    bool overflow_out = false;    ///< OVFEO
    bool retrigger_out = false;   ///< TRGEO
    bool count_out = false;       ///< CNTEO
    TccCountEvent count_select = TccCountEvent::begin;
};

/// FCTRLA or FCTRLB - one recoverable fault, whole (36.8.5).
struct TccFaultConfig {
    TccFaultSource source = TccFaultSource::off;
    TccFaultHalt halt = TccFaultHalt::off;
    TccFaultCapture capture = TccFaultCapture::off;
    /// FCTRLn.CHSEL: which channel the capture action writes.
    uint8_t capture_channel = 0;

    /// FCTRLn.KEEP: hold the fault state to the end of the TCC cycle
    /// instead of releasing it with the input.
    bool keep = false;
    /// FCTRLn.QUAL: ignore the input while the channel's output is at
    /// its inactive level.
    bool qualify = false;
    /// FCTRLn.RESTART: restart the counter as soon as the fault is
    /// valid.
    bool restart = false;

    /// FCTRLn.BLANK / BLANKVAL: ignore the input for BLANKVAL prescaled
    /// clocks after the selected waveform edge. `blank_value` 0 means no
    /// blanking window at all.
    TccFaultBlank blank = TccFaultBlank::period_start;
    uint8_t blank_value = 0;

    /// FCTRLn.FILTERVAL, in prescaled clocks; a pulse shorter than this
    /// is discarded and a valid one is delayed by it. MUST be zero for a
    /// synchronous event source - which erratum 1.21.9 forbids anyway.
    uint8_t filter_value = 0;
};

// =============================================================================
// Geometry and legality - samc/device_tables.hpp is the authority
// =============================================================================
//
// `tcc_count()`, `tcc_cc_count(n)`, `tcc_wo_count(n)`, `tcc_size(n)`,
// `tcc_gclk_id(n)`, `tcc_pair_role(n)`, the five extension probes and
// the EVSYS/DMAC codes all live in the reserve, read out of the device
// header's own `TCCn_*` constants.

/// How many dead-time/swap slices an instance has: one per compare
/// channel, but never more than half its outputs, since slice x drives
/// the pair (WO[x], WO[x + WO_NUM/2]) - 36.6.3.7.
constexpr uint8_t tcc_slice_count(uint8_t n) {
    if (!tcc_has_dead_time(n)) {
        return 0;
    }
    const uint8_t half = static_cast<uint8_t>(tcc_wo_count(n) / 2u);
    const uint8_t channels = tcc_cc_count(n);
    return half < channels ? half : channels;
}

/// The counter's MAX for this instance: 0xFFFFFF at 24 bits, 0xFFFF at
/// 16. 36.8.15 - "the excess bits are read zero" on a 16-bit instance.
constexpr uint32_t tcc_max_count(uint8_t n) {
    const uint8_t bits = tcc_size(n);
    return bits >= 32u ? 0xFFFFFFFFul
                       : static_cast<uint32_t>((1ul << bits) - 1ul);
}

constexpr bool tcc_channel_mask_ok(uint8_t n, uint8_t mask) {
    const uint8_t channels = tcc_cc_count(n);
    return channels >= 8u || (mask >> channels) == 0u;
}

constexpr bool tcc_output_mask_ok(uint8_t n, uint8_t mask) {
    const uint8_t outputs = tcc_wo_count(n);
    return outputs >= 8u || (mask >> outputs) == 0u;
}

/**
 * A CTRLA/CTRLB configuration's legality for THIS instance.
 *
 * Four refusals:
 *  - ERRATUM 1.21.10: `auto_lock` is not functional and has no
 *    workaround, so it is never written;
 *  - a dithering resolution on an instance without the dithering unit;
 *  - CTRLA.MSYNC on an instance that is not somebody's client (36.6.4);
 *  - a capture-enable bit past the channels this instance has.
 */
constexpr bool tcc_config_valid(uint8_t n, const TccConfig& c) {
    if (c.auto_lock) {
        return false;   // erratum 1.21.10
    }
    if (c.resolution != TccResolution::none && !tcc_has_dithering(n)) {
        return false;
    }
    if (c.host_sync && tcc_pair_role(n) != 2u) {
        return false;
    }
    return tcc_channel_mask_ok(n, c.capture_enable);
}

/**
 * A WAVE configuration's legality.
 *
 *  - RAMP2C is a variant-L encoding (36.8.17) and is refused;
 *  - a swap bit needs the swap unit AND a slice to swap;
 *  - polarity and circular-buffer bits must name channels that exist.
 */
constexpr bool tcc_wave_valid(uint8_t n, const TccWaveConfig& w) {
    if (w.ramp == TccRamp::ramp2_critical) {
        return false;
    }
    if (w.swap != 0u) {
        if (!tcc_has_swap(n)) {
            return false;
        }
        // WAVE.SWAPx addresses the same pairs the dead-time unit does,
        // whether or not DTIENx is set (36.8.17).
        const uint8_t slices = tcc_slice_count(n);
        if (slices == 0u || (w.swap >> slices) != 0u) {
            return false;
        }
    }
    return tcc_channel_mask_ok(n, w.polarity) &&
           tcc_channel_mask_ok(n, w.circular_cc);
}

/**
 * A WEXCTRL configuration's legality: the output matrix and the
 * dead-time unit are each optional per instance, and a dead-time slice
 * must exist.
 */
constexpr bool tcc_wave_ext_valid(uint8_t n, const TccWaveExtConfig& w) {
    if (w.output_matrix != TccOutputMatrix::per_channel &&
        !tcc_has_output_matrix(n)) {
        return false;
    }
    if (w.dead_time_enable != 0u) {
        const uint8_t slices = tcc_slice_count(n);
        if (slices == 0u || (w.dead_time_enable >> slices) != 0u) {
            return false;
        }
    }
    return true;
}

/// A DRVCTRL configuration's legality: every mask names outputs this
/// instance has, and the two filter values fit their 4-bit fields.
constexpr bool tcc_drive_valid(uint8_t n, const TccDriveConfig& d) {
    return tcc_output_mask_ok(n, d.invert) &&
           tcc_output_mask_ok(n, d.fault_output_enable) &&
           tcc_output_mask_ok(n, d.fault_output_value) && d.filter0 <= 0xFu &&
           d.filter1 <= 0xFu;
}

/// A recoverable fault's legality: the capture channel must exist and
/// the filter value must fit its 4-bit field.
constexpr bool tcc_fault_valid(uint8_t n, const TccFaultConfig& f) {
    return f.capture_channel < tcc_cc_count(n) && f.filter_value <= 0xFu;
}

/// Pattern generation: the unit must exist and every bit must name an
/// output this instance has.
constexpr bool tcc_pattern_valid(uint8_t n, uint8_t enable, uint8_t value) {
    return tcc_has_pattern(n) && tcc_output_mask_ok(n, enable) &&
           tcc_output_mask_ok(n, value);
}

/**
 * The rules that live BETWEEN the CTRLA and EVCTRL structs, so they get
 * a function of their own rather than being lost:
 *  - a PPW/PWP capture action needs a capture channel to put the
 *    readings in - "the corresponding capture is done only if the
 *    channel is enabled in capture mode" (36.6.2.7);
 *  - so does the STAMP action on event input 0;
 *  - MCEIx/MCEOx must name channels this instance has.
 */
constexpr bool tcc_event_config_valid(uint8_t n, const TccConfig& c,
                                      const TccEventConfig& e) {
    if (!tcc_channel_mask_ok(n, e.match_in) ||
        !tcc_channel_mask_ok(n, e.match_out)) {
        return false;
    }
    if (tcc_event1_is_capture(e.action1) && (c.capture_enable & 0x3u) == 0u) {
        return false;
    }
    if (e.action0 == TccEvent0Action::stamp && c.capture_enable == 0u) {
        return false;
    }
    return true;
}

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
class Tcc {
    static_assert(n < tcc_count(), "this device does not have that TCC instance");

public:
    Tcc() = delete;

    static constexpr uint8_t index = n;

    // ---- the geometry, all of it the device header's -----------------------

    static constexpr uint8_t cc_count = tcc_cc_count(n);
    static constexpr uint8_t wo_count = tcc_wo_count(n);
    static constexpr uint8_t counter_bits = tcc_size(n);
    static constexpr uint32_t max_count = tcc_max_count(n);
    static constexpr uint8_t gclk_id = tcc_gclk_id(n);
    static constexpr uint8_t slice_count = tcc_slice_count(n);

    static constexpr bool has_dead_time = tcc_has_dead_time(n);
    static constexpr bool has_output_matrix = tcc_has_output_matrix(n);
    static constexpr bool has_swap = tcc_has_swap(n);
    static constexpr bool has_pattern = tcc_has_pattern(n);
    static constexpr bool has_dithering = tcc_has_dithering(n);
    /// The header's own one-word summary of the five above.
    static constexpr uint8_t extension_code = tcc_ext_code(n);

    /// 36.6.4: two instances sharing a generic clock can be linked, and
    /// `TCCn_MASTER_SLAVE_MODE` says which end each is.
    static constexpr bool is_pair_host = tcc_pair_role(n) == 1u;
    static constexpr bool is_pair_client = tcc_pair_role(n) == 2u;

    static constexpr IRQn_Type irq() {
        return static_cast<IRQn_Type>(static_cast<int>(TCC0_IRQn) + n);
    }

    // ---- the EVSYS and DMAC vocabularies this peripheral publishes ---------
    //
    // evsys.hpp owns the fabric and not the vocabulary; dmac.hpp owns
    // the channels and not the trigger table. Both come out of the
    // device header, through the reserve - the TCC's generator codes are
    // NOT evenly spaced across instances (TCC0 spends seven of them,
    // TCC1 and TCC2 five each), so they are read and not computed.
    //
    // EVERY CHANNEL FEEDING ONE OF THESE USERS MUST BE ASYNCHRONOUS:
    // 36.6.3.5 for the fault inputs, and erratum 1.21.9 for all of them.

    static constexpr uint8_t overflow_generator = tcc_overflow_generator(n);
    static constexpr uint8_t retrigger_generator =
        static_cast<uint8_t>(tcc_overflow_generator(n) + 1u);
    static constexpr uint8_t count_generator =
        static_cast<uint8_t>(tcc_overflow_generator(n) + 2u);
    static constexpr uint8_t match_generator(uint8_t ch) {
        return static_cast<uint8_t>(tcc_match0_generator(n) + ch);
    }

    /// The two counter event inputs, TCE0 and TCE1 - the ones that carry
    /// the NON-recoverable faults.
    static constexpr uint8_t event_user(uint8_t which) {
        return static_cast<uint8_t>(tcc_event0_user(n) + which);
    }
    /// Channel x's event input MCEx - a capture source, and for x = 0
    /// and 1 the RECOVERABLE fault inputs A and B.
    static constexpr uint8_t match_user(uint8_t ch) {
        return static_cast<uint8_t>(tcc_match0_user(n) + ch);
    }
    /// The user index a recoverable fault listens on.
    static constexpr uint8_t fault_user(TccFault f) {
        return match_user(static_cast<uint8_t>(f));
    }

    static constexpr uint8_t dma_trigger_overflow = tcc_dma_overflow_id(n);
    static constexpr uint8_t dma_trigger_match(uint8_t ch) {
        return static_cast<uint8_t>(tcc_dma_match0_id(n) + ch);
    }

    static tcc_registers_t& regs() {
        if constexpr (n == 0) {
            return *TCC0_REGS;
#ifdef TCC1_REGS
        } else if constexpr (n == 1) {
            return *TCC1_REGS;
#endif
#ifdef TCC2_REGS
        } else if constexpr (n == 2) {
            return *TCC2_REGS;
#endif
        } else {
            return *TCC0_REGS;
        }
    }

    static constexpr bool config_valid(const TccConfig& c) {
        return tcc_config_valid(n, c);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_c(apb_mask(), on); }

    /// TCC0 and TCC1 SHARE generic clock channel 28 (36.5.3), so a
    /// connect() for one of them moves the other too.
    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().TCC_SYNCBUSY, mask, false, spins);
    }
    static bool sync_busy(uint32_t mask) {
        return (regs().TCC_SYNCBUSY & mask) != 0u;
    }
    /// SYNCBUSY.CCx stands from a BUFFERED write until the update
    /// consumes it, not merely for the clock-domain crossing - fact 8.
    static bool cc_sync_busy(uint8_t ch) {
        return sync_busy(TCC_SYNCBUSY_CC0_Msk << ch);
    }
    static bool period_sync_busy() { return sync_busy(TCC_SYNCBUSY_PER_Msk); }
    static bool pattern_sync_busy() { return sync_busy(TCC_SYNCBUSY_PATT_Msk); }

    /// CTRLA.SWRST. 36.6.2.2: "the TCC should be disabled before the TCC
    /// is reset to avoid undefined behavior", so this disables first
    /// rather than trusting the caller. DBGCTRL survives a reset.
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().TCC_CTRLA = 0u;
        if (!sync_wait(TCC_SYNCBUSY_ENABLE_Msk, spins)) {
            return false;
        }
        regs().TCC_CTRLA = TCC_CTRLA_SWRST_Msk;
        return sync_wait(TCC_SYNCBUSY_SWRST_Msk | TCC_SYNCBUSY_ENABLE_Msk, spins);
    }

    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = regs().TCC_CTRLA & ~TCC_CTRLA_ENABLE_Msk;
        regs().TCC_CTRLA = on ? (v | TCC_CTRLA_ENABLE_Msk) : v;
        return sync_wait(TCC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() {
        return (regs().TCC_CTRLA & TCC_CTRLA_ENABLE_Msk) != 0u;
    }

    /// APB clock, generic clock, software reset; left DISABLED, because
    /// six of this chapter's registers are enable-protected and the
    /// caller has them all to write.
    static bool init(uint8_t generator, uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        bus_clock(true);
        if (!clock(generator, spins)) {
            return false;
        }
        return reset(spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        bus_clock(false);
    }

    // ---- configuration: the enable-protected group -------------------------

    /**
     * CTRLA and the two CTRLB mode bits.
     *
     * REFUSED while the TCC is enabled: CTRLA is enable-protected
     * (36.6.2.1) and a write that lands nowhere is worse than a false
     * return. The CTRLB half is not enable-protected but is written
     * here anyway, because DIR/LUPD/ONESHOT belong to the same one-shot
     * description of how the counter behaves - and because each is set
     * and cleared through a DIFFERENT register, so leaving them at
     * "whatever a reset happened to leave" is not a state a caller can
     * reason about.
     */
    static bool configure(const TccConfig& c, uint32_t spins = 0xFFFFu) {
        if (!config_valid(c) || enabled()) {
            return false;
        }
        regs().TCC_CTRLA =
            TCC_CTRLA_PRESCALER(static_cast<uint32_t>(c.prescaler)) |
            TCC_CTRLA_PRESCSYNC(static_cast<uint32_t>(c.prescaler_sync)) |
            TCC_CTRLA_RESOLUTION(static_cast<uint32_t>(c.resolution)) |
            TCC_CTRLA_CPTEN(c.capture_enable) |
            (c.run_standby ? TCC_CTRLA_RUNSTDBY_Msk : 0u) |
            (c.dma_one_shot ? TCC_CTRLA_DMAOS_Msk : 0u) |
            (c.host_sync ? TCC_CTRLA_MSYNC_Msk : 0u);

        const uint8_t set = static_cast<uint8_t>(
            (c.count_down ? TCC_CTRLBSET_DIR_Msk : 0u) |
            (c.one_shot ? TCC_CTRLBSET_ONESHOT_Msk : 0u) |
            (c.lock_update ? TCC_CTRLBSET_LUPD_Msk : 0u));
        const uint8_t clear = static_cast<uint8_t>(
            (c.count_down ? 0u : TCC_CTRLBCLR_DIR_Msk) |
            (c.one_shot ? 0u : TCC_CTRLBCLR_ONESHOT_Msk) |
            (c.lock_update ? 0u : TCC_CTRLBCLR_LUPD_Msk));
        if (clear != 0u) {
            regs().TCC_CTRLBCLR = clear;
            if (!sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins)) {
                return false;
            }
        }
        if (set != 0u) {
            regs().TCC_CTRLBSET = set;
            if (!sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins)) {
                return false;
            }
        }
        return true;
    }

    /// WEXCTRL: the output matrix and the dead-time generators.
    static bool wave_extension(const TccWaveExtConfig& w) {
        if (!tcc_wave_ext_valid(n, w) || enabled()) {
            return false;
        }
        regs().TCC_WEXCTRL =
            TCC_WEXCTRL_OTMX(static_cast<uint32_t>(w.output_matrix)) |
            TCC_WEXCTRL_DTIEN(w.dead_time_enable) |
            TCC_WEXCTRL_DTLS(w.dead_time_low) | TCC_WEXCTRL_DTHS(w.dead_time_high);
        return true;
    }

    /// DRVCTRL: inversion, the non-recoverable fault output state, and
    /// the two non-recoverable fault filters.
    static bool drive(const TccDriveConfig& d) {
        if (!tcc_drive_valid(n, d) || enabled()) {
            return false;
        }
        regs().TCC_DRVCTRL =
            TCC_DRVCTRL_INVEN(d.invert) | TCC_DRVCTRL_NRE(d.fault_output_enable) |
            TCC_DRVCTRL_NRV(d.fault_output_value) |
            TCC_DRVCTRL_FILTERVAL0(d.filter0) | TCC_DRVCTRL_FILTERVAL1(d.filter1);
        return true;
    }

    /// FCTRLA or FCTRLB - one recoverable fault, whole.
    static bool fault(TccFault which, const TccFaultConfig& f) {
        if (!tcc_fault_valid(n, f) || enabled()) {
            return false;
        }
        const uint32_t word =
            TCC_FCTRLA_SRC(static_cast<uint32_t>(f.source)) |
            (f.keep ? TCC_FCTRLA_KEEP_Msk : 0u) |
            (f.qualify ? TCC_FCTRLA_QUAL_Msk : 0u) |
            TCC_FCTRLA_BLANK(static_cast<uint32_t>(f.blank)) |
            (f.restart ? TCC_FCTRLA_RESTART_Msk : 0u) |
            TCC_FCTRLA_HALT(static_cast<uint32_t>(f.halt)) |
            TCC_FCTRLA_CHSEL(f.capture_channel) |
            TCC_FCTRLA_CAPTURE(static_cast<uint32_t>(f.capture)) |
            TCC_FCTRLA_BLANKVAL(f.blank_value) |
            TCC_FCTRLA_FILTERVAL(f.filter_value);
        if (which == TccFault::a) {
            regs().TCC_FCTRLA = word;
        } else {
            regs().TCC_FCTRLB = word;
        }
        return true;
    }

    /// EVCTRL. Takes the TccConfig too, because the rules that decide a
    /// capture action's legality live between the two structs.
    static bool event_config(const TccConfig& c, const TccEventConfig& e) {
        if (!tcc_event_config_valid(n, c, e) || enabled()) {
            return false;
        }
        regs().TCC_EVCTRL =
            TCC_EVCTRL_EVACT0(static_cast<uint32_t>(e.action0)) |
            TCC_EVCTRL_EVACT1(static_cast<uint32_t>(e.action1)) |
            TCC_EVCTRL_CNTSEL(static_cast<uint32_t>(e.count_select)) |
            (e.overflow_out ? TCC_EVCTRL_OVFEO_Msk : 0u) |
            (e.retrigger_out ? TCC_EVCTRL_TRGEO_Msk : 0u) |
            (e.count_out ? TCC_EVCTRL_CNTEO_Msk : 0u) |
            (e.invert0 ? TCC_EVCTRL_TCINV0_Msk : 0u) |
            (e.invert1 ? TCC_EVCTRL_TCINV1_Msk : 0u) |
            (e.input0_enable ? TCC_EVCTRL_TCEI0_Msk : 0u) |
            (e.input1_enable ? TCC_EVCTRL_TCEI1_Msk : 0u) |
            TCC_EVCTRL_MCEI(e.match_in) | TCC_EVCTRL_MCEO(e.match_out);
        return true;
    }

    // ---- configuration: WAVE, which is NOT enable-protected -----------------

    /**
     * WAVE: the waveform mode, the ramp, the polarities, the swaps and
     * the circular buffers.
     *
     * Write-synchronized and NOT enable-protected (fact 3), so this is
     * the one configuration verb that works on a RUNNING TCC - which is
     * what makes a live polarity flip or a live swap possible.
     */
    static bool wave(const TccWaveConfig& w, uint32_t spins = 0xFFFFu) {
        if (!tcc_wave_valid(n, w)) {
            return false;
        }
        regs().TCC_WAVE = TCC_WAVE_WAVEGEN(static_cast<uint32_t>(w.waveform)) |
                          TCC_WAVE_RAMP(static_cast<uint32_t>(w.ramp)) |
                          TCC_WAVE_POL(w.polarity) | TCC_WAVE_SWAP(w.swap) |
                          TCC_WAVE_CICCEN(w.circular_cc) |
                          (w.circular_period ? TCC_WAVE_CIPEREN_Msk : 0u);
        return sync_wait(TCC_SYNCBUSY_WAVE_Msk, spins);
    }

    static uint32_t ctrla() { return regs().TCC_CTRLA; }
    static uint32_t evctrl() { return regs().TCC_EVCTRL; }
    static uint32_t wave_reg() { return regs().TCC_WAVE; }
    static uint32_t wave_ext_reg() { return regs().TCC_WEXCTRL; }
    static uint32_t drive_reg() { return regs().TCC_DRVCTRL; }
    static uint32_t fault_reg(TccFault which) {
        return which == TccFault::a ? regs().TCC_FCTRLA : regs().TCC_FCTRLB;
    }
    static uint32_t syncbusy() { return regs().TCC_SYNCBUSY; }

    static TccWaveform waveform() {
        return static_cast<TccWaveform>(
            (regs().TCC_WAVE & TCC_WAVE_WAVEGEN_Msk) >> TCC_WAVE_WAVEGEN_Pos);
    }
    static TccRamp ramp() {
        return static_cast<TccRamp>((regs().TCC_WAVE & TCC_WAVE_RAMP_Msk) >>
                                    TCC_WAVE_RAMP_Pos);
    }

    // ---- commands ----------------------------------------------------------

    /**
     * CTRLBSET.CMD, and `TccCommand::none` CANCELS a pending one.
     *
     * THE TWO CTRLB REGISTERS ARE NOT A SET/CLEAR PAIR FOR THE COMMAND
     * FIELDS: "writing zero to this bit group has no effect" on BOTH of
     * them (36.8.2, 36.8.3), so a command is issued by writing its code
     * to CTRLBSET and cancelled by writing ONES to CTRLBCLR - which is
     * the opposite of how DIR, LUPD and ONESHOT work in the same
     * register. The bench found this the hard way with IDXCMD.
     */
    static bool command(TccCommand c, uint32_t spins = 0xFFFFu) {
        if (c == TccCommand::none) {
            regs().TCC_CTRLBCLR = static_cast<uint8_t>(TCC_CTRLBCLR_CMD_Msk);
        } else {
            regs().TCC_CTRLBSET =
                static_cast<uint8_t>(TCC_CTRLBSET_CMD(static_cast<uint32_t>(c)));
        }
        return sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins);
    }
    static bool retrigger(uint32_t spins = 0xFFFFu) {
        return command(TccCommand::retrigger, spins);
    }
    static bool stop(uint32_t spins = 0xFFFFu) {
        return command(TccCommand::stop, spins);
    }
    /// Force every valid buffer into its live register, whatever LUPD
    /// says (36.6.2.6's note).
    static bool update(uint32_t spins = 0xFFFFu) {
        return command(TccCommand::update, spins);
    }

    /// CTRLBSET.IDXCMD: force the next RAMP2/RAMP2A cycle. Taken at the
    /// next update condition, and cleared by the hardware then. `off`
    /// CANCELS a command still pending, through CTRLBCLR - see
    /// `command()` for why it cannot be a zero written to CTRLBSET.
    static bool ramp_index_command(TccRampIndexCommand c,
                                   uint32_t spins = 0xFFFFu) {
        if (c == TccRampIndexCommand::off) {
            regs().TCC_CTRLBCLR = static_cast<uint8_t>(TCC_CTRLBCLR_IDXCMD_Msk);
        } else {
            regs().TCC_CTRLBSET = static_cast<uint8_t>(
                TCC_CTRLBSET_IDXCMD(static_cast<uint32_t>(c)));
        }
        return sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins);
    }

    /// CTRLB.DIR, live - the direction may change while the counter runs
    /// (36.6.2.4).
    static bool count_down(bool down, uint32_t spins = 0xFFFFu) {
        if (down) {
            regs().TCC_CTRLBSET = TCC_CTRLBSET_DIR_Msk;
        } else {
            regs().TCC_CTRLBCLR = TCC_CTRLBCLR_DIR_Msk;
        }
        return sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins);
    }
    static bool counting_down() {
        return (regs().TCC_CTRLBSET & TCC_CTRLBSET_DIR_Msk) != 0u;
    }

    /**
     * CTRLB.LUPD, live: hold the buffered registers back (true) or let
     * the hardware take them at every update condition (false).
     *
     * ERRATUM 1.21.8, live on every revision: while COUNTING DOWN this
     * does NOT protect PER - PERBUF is copied into it regardless. There
     * is no workaround; the bench measured it and docs/samc/tcc.md
     * carries the numbers.
     */
    static bool lock_update(bool locked, uint32_t spins = 0xFFFFu) {
        if (locked) {
            regs().TCC_CTRLBSET = TCC_CTRLBSET_LUPD_Msk;
        } else {
            regs().TCC_CTRLBCLR = TCC_CTRLBCLR_LUPD_Msk;
        }
        return sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins);
    }
    static bool update_locked() {
        return (regs().TCC_CTRLBSET & TCC_CTRLBSET_LUPD_Msk) != 0u;
    }

    static bool one_shot(bool on, uint32_t spins = 0xFFFFu) {
        if (on) {
            regs().TCC_CTRLBSET = TCC_CTRLBSET_ONESHOT_Msk;
        } else {
            regs().TCC_CTRLBCLR = TCC_CTRLBCLR_ONESHOT_Msk;
        }
        return sync_wait(TCC_SYNCBUSY_CTRLB_Msk, spins);
    }

    // ---- the counter -------------------------------------------------------
    //
    // READING COUNT IS A COMMAND (36.6.7): READSYNC, then the two waits,
    // then the load. `count_raw()` skips it and says so.

    /// READSYNC ISSUED TWICE, deliberately: the COUNT shadow lands about
    /// half a counter-clock period AFTER SYNCBUSY clears, with no bit
    /// advertising it (measured on the TCC exactly as on the TC -
    /// tc.hpp's counter comment carries the whole mechanism and the
    /// numbers). The second command's crossing covers the first's
    /// landing gap, so count() returns the value at THIS call's entry
    /// instead of the previous call's.
    static bool read_sync(uint32_t spins = 0xFFFFu) {
        return command(TccCommand::read_sync, spins) &&
               sync_wait(TCC_SYNCBUSY_COUNT_Msk, spins) &&
               command(TccCommand::read_sync, spins) &&
               sync_wait(TCC_SYNCBUSY_COUNT_Msk, spins);
    }

    static uint32_t count_raw() { return regs().TCC_COUNT; }
    static uint32_t count(uint32_t spins = 0xFFFFu) {
        (void)read_sync(spins);
        return count_raw();
    }

    /// Writing COUNT takes priority over count, clear and reload
    /// (36.6.2.4).
    static bool set_count(uint32_t v, uint32_t spins = 0xFFFFu) {
        regs().TCC_COUNT = v;
        return sync_wait(TCC_SYNCBUSY_COUNT_Msk, spins);
    }

    // ---- period and compare ------------------------------------------------

    static uint32_t period() { return regs().TCC_PER; }
    static bool set_period(uint32_t v, uint32_t spins = 0xFFFFu) {
        regs().TCC_PER = v;
        return sync_wait(TCC_SYNCBUSY_PER_Msk, spins);
    }
    /// The buffered period, taken at the next UPDATE condition. Like
    /// every buffered write here it does not wait and refuses while the
    /// previous one is pending - see `set_cc_buffer()` and fact 8.
    static uint32_t period_buffer() { return regs().TCC_PERBUF; }
    static bool set_period_buffer(uint32_t v) {
        if (period_sync_busy()) {
            return false;
        }
        regs().TCC_PERBUF = v;
        return true;
    }

    static uint32_t cc(uint8_t ch) { return regs().TCC_CC[ch]; }
    static bool set_cc(uint8_t ch, uint32_t v, uint32_t spins = 0xFFFFu) {
        regs().TCC_CC[ch] = v;
        return sync_wait(TCC_SYNCBUSY_CC0_Msk << ch, spins);
    }

    /**
     * The double-buffered write - taken at the next UPDATE, which is
     * what keeps a live duty change glitch-free (36.6.2.6).
     *
     * IT DOES NOT WAIT, and that is fact 8: SYNCBUSY.CCx stands from
     * this write until the update CONSUMES the buffer, so waiting it out
     * would cost a whole PWM period (and never end while CTRLB.LUPD is
     * set). A write attempted while the bit stands is DISCARDED by the
     * silicon, so this refuses instead - false means "the previous
     * buffered value has not been taken yet", which is the honest answer
     * for a duty set faster than the waveform can accept one.
     */
    static uint32_t cc_buffer(uint8_t ch) { return regs().TCC_CCBUF[ch]; }
    static bool set_cc_buffer(uint8_t ch, uint32_t v) {
        if (cc_sync_busy(ch)) {
            return false;
        }
        regs().TCC_CCBUF[ch] = v;
        return true;
    }

    // ---- pattern generation ------------------------------------------------

    /**
     * PATT: which outputs the pattern unit overrides and with what
     * level. It sits AFTER the swap stage, so it beats everything the
     * waveform generator, the matrix, the dead-time unit and the swap
     * produced (36.6.3.7).
     */
    static bool pattern(uint8_t enable, uint8_t value, uint32_t spins = 0xFFFFu) {
        if (!tcc_pattern_valid(n, enable, value)) {
            return false;
        }
        regs().TCC_PATT =
            static_cast<uint16_t>(TCC_PATT_PGE(enable) | TCC_PATT_PGV(value));
        return sync_wait(TCC_SYNCBUSY_PATT_Msk, spins);
    }
    /// The buffered pattern, taken at the next UPDATE condition - which
    /// is what "synchronized bit pattern" means (36.6.3.7). Does not
    /// wait, and refuses while the previous one is pending (fact 8).
    static bool pattern_buffer(uint8_t enable, uint8_t value) {
        if (!tcc_pattern_valid(n, enable, value) || pattern_sync_busy()) {
            return false;
        }
        regs().TCC_PATTBUF =
            static_cast<uint16_t>(TCC_PATTBUF_PGEB(enable) | TCC_PATTBUF_PGVB(value));
        return true;
    }
    static uint16_t pattern_reg() { return regs().TCC_PATT; }

    // ---- status ------------------------------------------------------------
    //
    // STATUS is BOTH read- and write-synchronized (36.8.14), so every
    // read waits its SYNCBUSY bit out. The wait is bounded and costs
    // nothing when the register is quiet.

    static uint32_t status(uint32_t spins = 0xFFFFu) {
        (void)sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins);
        return regs().TCC_STATUS;
    }

    static bool stopped() { return (status() & TCC_STATUS_STOP_Msk) != 0u; }
    /// STATUS.IDX: the current RAMP2/RAMP2A cycle - 0 is cycle A, 1 is
    /// cycle B, and it always reads zero in RAMP1 (36.8.14).
    static bool ramp_index() { return (status() & TCC_STATUS_IDX_Msk) != 0u; }
    /// STATUS.SLAVE, which follows CTRLA.MSYNC.
    static bool is_client() { return (status() & TCC_STATUS_SLAVE_Msk) != 0u; }

    static bool period_buffer_valid() {
        return (status() & TCC_STATUS_PERBUFV_Msk) != 0u;
    }
    static bool cc_buffer_valid(uint8_t ch) {
        return (status() & (TCC_STATUS_CCBUFV0_Msk << ch)) != 0u;
    }
    static bool pattern_buffer_valid() {
        return (status() & TCC_STATUS_PATTBUFV_Msk) != 0u;
    }
    /// STATUS.CMPx: what the waveform generator's channel-x comparator
    /// currently outputs - BEFORE the extension stage, so it is not
    /// necessarily what a pad shows.
    static bool compare_output(uint8_t ch) {
        return (status() & (TCC_STATUS_CMP0_Msk << ch)) != 0u;
    }

    /**
     * Clear a buffer-valid flag - TWICE, and that is ERRATUM 1.21.6,
     * live on every revision of this family: "when clearing STATUS.xxBUFV
     * flag, SYNCBUSY is released before the register is restored to its
     * appropriate value", and the documented workaround is to clear it
     * successively two times. The exact twin of the TC's 1.20.3.
     */
    static bool clear_buffer_valid(uint32_t mask, uint32_t spins = 0xFFFFu) {
        regs().TCC_STATUS = mask;
        if (!sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins)) {
            return false;
        }
        regs().TCC_STATUS = mask;
        return sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins);
    }

    // ---- the fault status --------------------------------------------------

    /// STATUS.FAULTAIN/FAULTBIN: the recoverable fault INPUT is asserted
    /// right now (a level, not a latch).
    static bool fault_input(TccFault f) {
        const uint32_t m = f == TccFault::a ? TCC_STATUS_FAULTAIN_Msk
                                            : TCC_STATUS_FAULTBIN_Msk;
        return (status() & m) != 0u;
    }
    /// STATUS.FAULTA/FAULTB: the recoverable fault STATE is latched.
    static bool fault_state(TccFault f) {
        const uint32_t m =
            f == TccFault::a ? TCC_STATUS_FAULTA_Msk : TCC_STATUS_FAULTB_Msk;
        return (status() & m) != 0u;
    }
    /**
     * Clear a recoverable fault state. It only clears while the matching
     * input is low, and with `TccFaultHalt::software` selected THIS is
     * what releases the counter (36.8.14).
     */
    static bool clear_fault_state(TccFault f, uint32_t spins = 0xFFFFu) {
        const uint32_t m =
            f == TccFault::a ? TCC_STATUS_FAULTA_Msk : TCC_STATUS_FAULTB_Msk;
        regs().TCC_STATUS = m;
        return sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins);
    }

    /// STATUS.FAULT0IN/FAULT1IN: a non-recoverable fault input is
    /// asserted right now.
    static bool non_recoverable_input(uint8_t which) {
        const uint32_t m = which == 0u ? TCC_STATUS_FAULT0IN_Msk
                                       : TCC_STATUS_FAULT1IN_Msk;
        return (status() & m) != 0u;
    }
    /// STATUS.FAULT0/FAULT1: the latched non-recoverable fault state.
    /// While it stands, the outputs follow DRVCTRL.NRE/NRV and the
    /// counter is stopped.
    static bool non_recoverable_state(uint8_t which) {
        const uint32_t m =
            which == 0u ? TCC_STATUS_FAULT0_Msk : TCC_STATUS_FAULT1_Msk;
        return (status() & m) != 0u;
    }
    /**
     * Clear a non-recoverable fault state - only effective while the
     * matching input is low. 36.8.14: once cleared "the timer/counter
     * will restart from the last COUNT value", so a caller that wants to
     * restart from ZERO issues RETRIGGER **before** clearing.
     */
    static bool clear_non_recoverable(uint8_t which, uint32_t spins = 0xFFFFu) {
        const uint32_t m =
            which == 0u ? TCC_STATUS_FAULT0_Msk : TCC_STATUS_FAULT1_Msk;
        regs().TCC_STATUS = m;
        return sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins);
    }

    /// STATUS.DFS: the debug fault state, raised when the CPU halts with
    /// DBGCTRL.FDDBD set.
    static bool debug_fault_state() { return (status() & TCC_STATUS_DFS_Msk) != 0u; }
    static bool clear_debug_fault(uint32_t spins = 0xFFFFu) {
        regs().TCC_STATUS = TCC_STATUS_DFS_Msk;
        return sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins);
    }
    /// STATUS.UFS: the non-recoverable UPDATE fault - raised when the
    /// ramp index or the direction changes while CTRLB.LUPD is set
    /// (36.6.3.6).
    static bool update_fault_state() { return (status() & TCC_STATUS_UFS_Msk) != 0u; }
    static bool clear_update_fault(uint32_t spins = 0xFFFFu) {
        regs().TCC_STATUS = TCC_STATUS_UFS_Msk;
        return sync_wait(TCC_SYNCBUSY_STATUS_Msk, spins);
    }

    // ---- interrupts --------------------------------------------------------

    static constexpr uint32_t overflow_flag = TCC_INTFLAG_OVF_Msk;
    static constexpr uint32_t retrigger_flag = TCC_INTFLAG_TRG_Msk;
    static constexpr uint32_t count_flag = TCC_INTFLAG_CNT_Msk;
    static constexpr uint32_t error_flag = TCC_INTFLAG_ERR_Msk;
    static constexpr uint32_t update_fault_flag = TCC_INTFLAG_UFS_Msk;
    static constexpr uint32_t debug_fault_flag = TCC_INTFLAG_DFS_Msk;
    static constexpr uint32_t fault_a_flag = TCC_INTFLAG_FAULTA_Msk;
    static constexpr uint32_t fault_b_flag = TCC_INTFLAG_FAULTB_Msk;
    static constexpr uint32_t non_recoverable_flag(uint8_t which) {
        return TCC_INTFLAG_FAULT0_Msk << which;
    }
    static constexpr uint32_t match_flag(uint8_t ch) {
        return TCC_INTFLAG_MC0_Msk << ch;
    }
    static constexpr uint32_t recoverable_flag(TccFault f) {
        return TCC_INTFLAG_FAULTA_Msk << static_cast<uint8_t>(f);
    }

    static uint32_t flags() { return regs().TCC_INTFLAG; }
    static void clear_flags(uint32_t mask) { regs().TCC_INTFLAG = mask; }
    static uint32_t armed() { return regs().TCC_INTENSET; }
    static void arm(uint32_t mask) { regs().TCC_INTENSET = mask; }
    static void disarm(uint32_t mask) { regs().TCC_INTENCLR = mask; }

    /**
     * The ISR body; the app binds TCCn_Handler.
     *
     * Masked with INTENSET like every other one-vector-many-sources
     * block in this stratum. NOTE for a CAPTURE handler: INTFLAG.MCx is
     * "automatically cleared when CCx register is read" (36.8.13), and
     * reading CCx is what lets CCBUFx move up - so a handler that clears
     * MCx without reading CCx throws the reading away.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t p = flags() & armed();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    // ---- debug -------------------------------------------------------------

    /// DBGCTRL.DBGRUN: keep counting while the CPU is halted. Survives a
    /// software reset, and 36.8.9 asks that it not be changed while the
    /// TCC is enabled.
    static void debug_run(bool on) {
        const uint8_t v = static_cast<uint8_t>(regs().TCC_DBGCTRL &
                                               ~TCC_DBGCTRL_DBGRUN_Msk);
        regs().TCC_DBGCTRL =
            static_cast<uint8_t>(on ? (v | TCC_DBGCTRL_DBGRUN_Msk) : v);
    }
    /// DBGCTRL.FDDBD: an OCD break request becomes a NON-RECOVERABLE
    /// fault, so a debugger halt shuts the drivers down (36.8.9).
    static void fault_on_debug(bool on) {
        const uint8_t v =
            static_cast<uint8_t>(regs().TCC_DBGCTRL & ~TCC_DBGCTRL_FDDBD_Msk);
        regs().TCC_DBGCTRL =
            static_cast<uint8_t>(on ? (v | TCC_DBGCTRL_FDDBD_Msk) : v);
    }
    static uint8_t debug_reg() { return regs().TCC_DBGCTRL; }

private:
    static constexpr uint32_t apb_mask() {
        if constexpr (n == 0) {
            return MCLK_APBCMASK_TCC0_Msk;
#ifdef MCLK_APBCMASK_TCC1_Msk
        } else if constexpr (n == 1) {
            return MCLK_APBCMASK_TCC1_Msk;
#endif
#ifdef MCLK_APBCMASK_TCC2_Msk
        } else if constexpr (n == 2) {
            return MCLK_APBCMASK_TCC2_Msk;
#endif
        } else {
            return 0u;
        }
    }
};

// =============================================================================
// Pad to waveform output: samc/device_tables.hpp is the authority
// =============================================================================

/**
 * One TCC waveform output reached through its pad AND the peripheral
 * function that carries it.
 *
 *   using Low = brio::TccWo<brio::Pin<'A', 8>, brio::PinFunction::e>;
 *   //  -> TCC0 / WO0, from PIN_PA08E_TCC0_WO0
 *   using High = brio::TccWo<brio::Pin<'A', 22>, brio::PinFunction::f>;
 *   //  -> TCC0 / WO4, from PIN_PA22F_TCC0_WO4
 *
 * The function is part of the key because the same pad carries a
 * different output of a different instance under each of the two: PA08
 * is TCC0/WO0 under E and TCC1/WO2 under F (fact 2 at the top of this
 * file).
 *
 * `claim()` turns the input buffer on, so the driven level can be read
 * back through PORT.IN - the only instrument a wireless bench has for a
 * waveform.
 */
template <class P, PinFunction F>
struct TccWo {
    TccWo() = delete;

    static_assert(F == PinFunction::e || F == PinFunction::f,
                  "TCC waveform outputs live on peripheral functions E and F "
                  "on this family");
    static constexpr char function_letter = F == PinFunction::e ? 'e' : 'f';

    static_assert(tcc_wo_exists<P::port_letter, P::pin_number, function_letter>,
                  "this pad carries no TCC waveform output on that peripheral "
                  "function on this device (the device header defines no "
                  "PIN_P<pad><fn>_TCC<n>_WO<k> for it)");

    using pin = P;
    static constexpr uint8_t timer = static_cast<uint8_t>(
        tcc_wo_code(P::port_letter, P::pin_number, function_letter) >> 4);
    static constexpr uint8_t output = static_cast<uint8_t>(
        tcc_wo_code(P::port_letter, P::pin_number, function_letter) & 0xFu);

    static void claim() {
        P::function(F, PinConfig{.input_enable = true});
    }
    static void release() { P::release(); }
};

// =============================================================================
// Tasks
// =============================================================================

/**
 * TccPwm<Tcc, ch, top>: one single-slope PWM channel, a `PwmChannel`
 * whose `max` is the PERIOD the timer is set to.
 *
 * Unlike the TC's 16-bit channel, this one gets to choose its own full
 * scale at any counter width, because the TCC has a real PER register in
 * every mode (fact 6). `top` is a template parameter because PwmChannel
 * requires `max` to be a compile-time constant - a channel whose full
 * scale can move under it is not something a generic actuator can scale
 * against.
 *
 * `duty()` writes CCBUFx, not CCx: the buffered write is taken at the
 * next UPDATE and is what keeps a live change glitch-free.
 */
template <class T, uint8_t ch, uint32_t top>
struct TccPwm {
    TccPwm() = delete;
    static_assert(ch < T::cc_count, "this TCC does not have that channel");
    static_assert(top > 0, "a PWM period of zero has no duty to set");
    static_assert(top <= T::max_count, "that period does not fit this counter");
    static_assert(top <= 0xFFFFu,
                  "PwmChannel's scale is 16-bit; a wider period needs the "
                  "resource's own set_cc_buffer()");

    static constexpr uint16_t max = static_cast<uint16_t>(top);

    /// Bring the timer up as a single-slope PWM generator. The caller
    /// has already called T::init() and claimed the pad.
    static bool setup(TccPrescaler prescaler = TccPrescaler::div1,
                      uint32_t spins = 0xFFFFu) {
        if (!T::configure(TccConfig{.prescaler = prescaler}, spins)) {
            return false;
        }
        if (!T::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}, spins)) {
            return false;
        }
        return T::set_period(top, spins) && T::set_cc(ch, 0, spins) &&
               T::enable(true, spins);
    }

    static void duty(uint16_t v) {
        (void)T::set_cc_buffer(ch, v > max ? max : v);
    }
};

/**
 * TccPairPwm<Tcc, slice, top>: the complementary output pair with
 * dead-time insertion - this chapter's reason to exist, and the SAM
 * counterpart of the AVR's TcdPwm.
 *
 * One duty value drives two pads: the dead-time unit splits matrix
 * output `slice` into a non-inverted LOW SIDE on WO[slice] and an
 * inverted HIGH SIDE on WO[slice + WO_NUM/2], with a programmable OFF
 * time between the two switchings so they can never conduct together
 * (36.6.3.7).
 *
 * The two dead times are counted in GCLK_TCC cycles and are NOT
 * prescaled (36.8.7), so a prescaled PWM and its dead time are set in
 * different units - which is exactly the sort of thing the bench had to
 * measure rather than assume.
 */
template <class T, uint8_t slice, uint32_t top>
struct TccPairPwm {
    TccPairPwm() = delete;
    static_assert(T::has_dead_time,
                  "this TCC instance has no dead-time insertion unit "
                  "(TCCn_DTI is 0 in the device header)");
    static_assert(slice < T::slice_count,
                  "this TCC does not have that dead-time slice");
    static_assert(top > 0, "a PWM period of zero has no duty to set");
    static_assert(top <= T::max_count, "that period does not fit this counter");
    static_assert(top <= 0xFFFFu, "PwmChannel's scale is 16-bit");

    static constexpr uint16_t max = static_cast<uint16_t>(top);
    /// The two outputs this pair drives, for the caller that has to
    /// claim their pads.
    static constexpr uint8_t low_output = slice;
    static constexpr uint8_t high_output =
        static_cast<uint8_t>(slice + T::wo_count / 2u);

    static bool setup(TccPrescaler prescaler, uint8_t dead_time_low,
                      uint8_t dead_time_high, uint32_t spins = 0xFFFFu) {
        if (!T::configure(TccConfig{.prescaler = prescaler}, spins)) {
            return false;
        }
        if (!T::wave_extension(TccWaveExtConfig{
                .dead_time_enable = static_cast<uint8_t>(1u << slice),
                .dead_time_low = dead_time_low,
                .dead_time_high = dead_time_high})) {
            return false;
        }
        if (!T::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}, spins)) {
            return false;
        }
        return T::set_period(top, spins) && T::set_cc(slice, 0, spins) &&
               T::enable(true, spins);
    }

    static void duty(uint16_t v) {
        (void)T::set_cc_buffer(slice, v > max ? max : v);
    }
};

} // namespace brio
