/*
 * rtc.hpp
 *
 * The AVR DA/DB real-time counter and periodic interrupt timer (RTC,
 * DS40002247B ch. 26) as three resources - see docs/avrdx/rtc.md:
 *
 *   RtcClock - the CLK_RTC selector (CLKSEL), shared by both functions;
 *   Rtc      - the 16-bit counter: prescaler, CNT/PER/CMP, OVF and CMP
 *              flags/interrupts/events, crystal error correction;
 *   Pit      - the periodic interrupt timer: PERIOD, PITEN, PI.
 *
 * ONE BLOCK, ONE CLOCK. The RTC peripheral has a single clock select
 * and a single prescaler chain serving both functions (26.4.1.1). The
 * OWNER of CLKSEL in a brio program is the timebase - `BasicTicker`
 * (ticker.hpp) selects it in init(); `Rtc` never writes it. An
 * application that runs the counter without a Ticker calls
 * RtcClock::select() itself, once, before either function is enabled.
 * `Rtc` and `Pit` are otherwise independent: their control, status,
 * interrupt and debug registers are disjoint.
 *
 * Facts that shape the code (26.3 - 26.11, both errata documents list
 * NO RTC items - DB DS80000915F, DA DS80000882C):
 *  - the two functions share the prescaler's internal counter and it
 *    RUNS as soon as EITHER is enabled: the first PIT interrupt and the
 *    first RTC count tick fall anywhere inside one full period
 *    (26.5.2.2). Nothing can pin that phase down - a period boundary is
 *    the only instant an application may rely on;
 *  - the PIT PERIOD counts CLK_RTC cycles, not prescaled ones: it taps
 *    the shared chain directly and CTRLA.PRESCALER does not move it
 *    (bench-verified, see rtc.md);
 *  - every write crosses into CLK_RTC: CTRLA, CNT, PER and CMP each
 *    have a busy flag in STATUS and PITCTRLA one in PITSTATUS (26.10).
 *    The verbs below wait for the flag of the register they are about
 *    to write; the wait is BOUNDED, so a CLK_RTC that no peripheral
 *    requests (no oscillator running) cannot hang the caller - the
 *    write simply lands when the clock returns;
 *  - CLK_PER must be at least four times CLK_RTC to read CNT (26.3);
 *  - the 16-bit registers go through the peripheral's one TEMP
 *    register: a CNT/PER/CMP access from the main context while an ISR
 *    touches them too wants a critical section;
 *  - crystal error correction is a TRIM, not a cure: CALIB carries a
 *    sign and 7 bits of ppm, so +-127 ppm, spread over a million
 *    CLK_RTC cycles (30.5 s at 32.768 kHz - a window shorter than that
 *    sees the correction quantized). A NEGATIVE correction requires
 *    the prescaler to be DIV2 or slower (26.6): the config and
 *    calibrate() refuse the illegal combination instead of programming
 *    a silently wrong trim;
 *  - the RTC keeps counting in idle sleep, and in standby with
 *    RUNSTDBY; the PIT runs in every sleep mode including power-down
 *    (26.9). DBGCTRL/PITDBGCTRL keep them running under a halted CPU;
 *    with PITDBGCTRL.DBGRUN = 0 the PIT output is forced low while
 *    halted, so a break taken with the output high costs one extra
 *    interrupt on resume (26.11);
 *  - the event generators (EvRtcOvf, EvRtcCmp, EvPitDiv<n>, evsys.hpp)
 *    fire on exactly the conditions that raise the flags.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"

namespace brio {

// ---- the shared clock (26.13.8) ----------------------------------------------

/// CLKSEL: the source of CLK_RTC for BOTH functions.
enum class RtcSource : uint8_t {
    osc32k = RTC_CLKSEL_OSC32K_gc,    ///< 32.768 kHz from the internal OSC32K
    osc1k = RTC_CLKSEL_OSC1K_gc,      ///< 1.024 kHz: the same OSC32K divided by 32
    xosc32k = RTC_CLKSEL_XOSC32K_gc,  ///< 32.768 kHz crystal (or external clock) on XTAL32K
    extclk = RTC_CLKSEL_EXTCLK_gc,    ///< external clock on the EXTCLK / XTALHF1 pin
};

/// Nominal rate of a source in Hz; 0 for `extclk`, whose rate only the
/// application knows.
constexpr uint32_t rtc_source_hz(RtcSource s) {
    return s == RtcSource::osc1k ? 1024u : s == RtcSource::extclk ? 0u : 32768u;
}

/// The CLK_RTC selector. One block, one clock: whoever owns the
/// timebase owns this register (see the file header).
struct RtcClock {
    RtcClock() = delete;

    /// Select the source. Do it once, before enabling either function:
    /// switching under a running counter/PIT costs an unknown fraction
    /// of a period, exactly like the first tick after an enable.
    static void select(RtcSource s) { RTC.CLKSEL = static_cast<uint8_t>(s); }

    static RtcSource selected() {
        return static_cast<RtcSource>(RTC.CLKSEL & RTC_CLKSEL_gm);
    }

    /// Nominal rate of the selected source (0 = external clock).
    static uint32_t hz() { return rtc_source_hz(selected()); }

    /// What a timebase picks when the application does not say: the
    /// 32.768 kHz crystal if the clock init started one and it is
    /// running, the internal OSC32K otherwise.
    static RtcSource preferred() {
        return Xosc32k::stable() ? RtcSource::xosc32k : RtcSource::osc32k;
    }
};

// ---- synchronization (26.10) --------------------------------------------------

/// Wait for the named STATUS bits to clear, at most `spins` times.
/// False = still busy (CLK_RTC is not running); the pending write lands
/// when the clock returns.
inline bool rtc_wait(uint8_t mask, uint16_t spins = 0xFFFFu) {
    for (uint16_t i = 0; i < spins; ++i) {
        if ((RTC.STATUS & mask) == 0) return true;
    }
    return (RTC.STATUS & mask) == 0;
}

/// The same for PITSTATUS.CTRLBUSY.
inline bool rtc_pit_wait(uint16_t spins = 0xFFFFu) {
    for (uint16_t i = 0; i < spins; ++i) {
        if ((RTC.PITSTATUS & RTC_CTRLBUSY_bm) == 0) return true;
    }
    return (RTC.PITSTATUS & RTC_CTRLBUSY_bm) == 0;
}

// ---- the counter (26.4, 26.13.1 - 26.13.11) -----------------------------------

/// PRESCALER: CLK_RTC divided before the counter. The PIT does not see
/// it (its PERIOD counts CLK_RTC cycles).
enum class RtcPrescaler : uint8_t {
    div1 = RTC_PRESCALER_DIV1_gc,
    div2 = RTC_PRESCALER_DIV2_gc,
    div4 = RTC_PRESCALER_DIV4_gc,
    div8 = RTC_PRESCALER_DIV8_gc,
    div16 = RTC_PRESCALER_DIV16_gc,
    div32 = RTC_PRESCALER_DIV32_gc,
    div64 = RTC_PRESCALER_DIV64_gc,
    div128 = RTC_PRESCALER_DIV128_gc,
    div256 = RTC_PRESCALER_DIV256_gc,
    div512 = RTC_PRESCALER_DIV512_gc,
    div1024 = RTC_PRESCALER_DIV1024_gc,
    div2048 = RTC_PRESCALER_DIV2048_gc,
    div4096 = RTC_PRESCALER_DIV4096_gc,
    div8192 = RTC_PRESCALER_DIV8192_gc,
    div16384 = RTC_PRESCALER_DIV16384_gc,
    div32768 = RTC_PRESCALER_DIV32768_gc,
};

/// The divisor a prescaler setting stands for (1 .. 32768).
constexpr uint16_t rtc_prescaler_div(RtcPrescaler p) {
    return static_cast<uint16_t>(
        1u << ((static_cast<uint8_t>(p) & RTC_PRESCALER_gm) >> RTC_PRESCALER_gp));
}

/// A correction is legal when it is inside CALIB's range (+-127 ppm)
/// and, if negative, the prescaler is DIV2 or slower (26.6).
constexpr bool rtc_correction_valid(int8_t ppm, RtcPrescaler p) {
    return ppm >= 0 ? true
                    : (ppm != -128 && p != RtcPrescaler::div1);
}

struct RtcConfig {
    RtcPrescaler prescaler = RtcPrescaler::div1;
    uint16_t period = 0xFFFF;      ///< PER: the counter wraps to 0 after it (OVF)
    uint16_t compare = 0;          ///< CMP: the value that raises CMP
    int8_t correction_ppm = 0;     ///< CALIB + CORREN; 0 = correction off
    bool run_standby = false;      ///< keep counting in standby sleep
    bool debug_run = false;        ///< keep counting while the CPU is halted
};

/// The two flags of the RTC counter's one vector (RTC_CNT_vect).
struct RtcFlags {
    bool ovf;
    bool cmp;
};

/// The real-time counter. A single instance on every package of the
/// family; its clock is RtcClock's, never touched here.
class Rtc {
public:
    Rtc() = delete;

    /// The event vocabulary of this counter (evsys.hpp).
    using OvfEvent = EvRtcOvf;      ///< generator: counter wrapped at PER
    using CmpEvent = EvRtcCmp;      ///< generator: counter reached CMP

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: an illegal correction is a compile error.
    template <RtcConfig cfg>
    static void init() {
        static_assert(rtc_correction_valid(cfg.correction_ppm, cfg.prescaler),
                      "RTC crystal error correction: +-127 ppm, and a NEGATIVE "
                      "correction needs the prescaler at DIV2 or slower (26.6)");
        (void)init(cfg);
    }

    /// Run-time form. Stops the counter, programs CALIB, PER, CMP, a
    /// zeroed CNT and the debug bit, clears the flags, then enables with
    /// the prescaler. Interrupts stay off: the caller enables them.
    /// False (and nothing programmed) when the correction asked for is
    /// out of range or negative with the prescaler at DIV1.
    static bool init(const RtcConfig& cfg) {
        if (!rtc_correction_valid(cfg.correction_ppm, cfg.prescaler)) return false;
        rtc_wait(RTC_CTRLABUSY_bm);
        RTC.CTRLA = 0;                       // stop; CORREN off while CALIB moves
        RTC.INTCTRL = 0;
        RTC.CALIB = calib_byte(cfg.correction_ppm);
        rtc_wait(RTC_PERBUSY_bm);
        RTC.PER = cfg.period;
        rtc_wait(RTC_CMPBUSY_bm);
        RTC.CMP = cfg.compare;
        rtc_wait(RTC_CNTBUSY_bm);
        RTC.CNT = 0;
        RTC.DBGCTRL = cfg.debug_run ? RTC_DBGRUN_bm : 0;
        RTC.INTFLAGS = RTC_OVF_bm | RTC_CMP_bm;
        rtc_wait(RTC_CTRLABUSY_bm);
        RTC.CTRLA = static_cast<uint8_t>(
            static_cast<uint8_t>(cfg.prescaler) |
            (cfg.correction_ppm != 0 ? RTC_CORREN_bm : 0) |
            (cfg.run_standby ? RTC_RUNSTDBY_bm : 0) |
            RTC_RTCEN_bm);
        return true;
    }

    static void enable() { ctrla_bits(RTC_RTCEN_bm, true); }
    static void disable() { ctrla_bits(RTC_RTCEN_bm, false); }
    static bool enabled() { return (RTC.CTRLA & RTC_RTCEN_bm) != 0; }

    /// Keep counting in standby sleep (26.9).
    static void run_standby(bool on) { ctrla_bits(RTC_RUNSTDBY_bm, on); }
    static bool run_standby() { return (RTC.CTRLA & RTC_RUNSTDBY_bm) != 0; }
    /// Keep counting while the debugger holds the CPU (26.11).
    static void debug_run(bool on) { RTC.DBGCTRL = on ? RTC_DBGRUN_bm : 0; }
    static bool debug_run() { return (RTC.DBGCTRL & RTC_DBGRUN_bm) != 0; }

    // ---- the counter ------------------------------------------------------

    static uint16_t count() { return RTC.CNT; }
    static void count(uint16_t v) { rtc_wait(RTC_CNTBUSY_bm); RTC.CNT = v; }
    static uint16_t period() { return RTC.PER; }
    static void period(uint16_t v) { rtc_wait(RTC_PERBUSY_bm); RTC.PER = v; }
    static uint16_t compare() { return RTC.CMP; }
    static void compare(uint16_t v) { rtc_wait(RTC_CMPBUSY_bm); RTC.CMP = v; }

    static RtcPrescaler prescaler() {
        return static_cast<RtcPrescaler>(RTC.CTRLA & RTC_PRESCALER_gm);
    }
    static void prescaler(RtcPrescaler p) {
        rtc_wait(RTC_CTRLABUSY_bm);
        RTC.CTRLA = static_cast<uint8_t>((RTC.CTRLA & ~RTC_PRESCALER_gm) |
                                         static_cast<uint8_t>(p));
    }

    /// Ticks per second of CNT: the selected source divided by the
    /// prescaler (0 when the source rate is the application's secret,
    /// i.e. an external clock).
    static uint32_t tick_hz() { return RtcClock::hz() / rtc_prescaler_div(prescaler()); }

    // ---- crystal error correction (26.6) ----------------------------------

    /// Trim CLK_RTC by `ppm`: positive slows the prescaler down (the
    /// crystal runs fast), negative speeds it up. 0 disables CORREN.
    /// False - and NOTHING written - outside +-127 ppm, or for a
    /// negative correction while the prescaler is at DIV1.
    static bool calibrate(int8_t ppm) {
        if (!rtc_correction_valid(ppm, prescaler())) return false;
        RTC.CALIB = calib_byte(ppm);
        rtc_wait(RTC_CTRLABUSY_bm);
        if (ppm != 0) RTC.CTRLA |= RTC_CORREN_bm;
        else RTC.CTRLA &= static_cast<uint8_t>(~RTC_CORREN_bm);
        return true;
    }
    /// The trim in force, signed (0 when CORREN is off).
    static int8_t calibration_ppm() {
        if (!correcting()) return 0;
        const uint8_t c = RTC.CALIB;
        const int8_t mag = static_cast<int8_t>(c & RTC_ERROR_gm);
        return (c & RTC_SIGN_bm) ? static_cast<int8_t>(-mag) : mag;
    }
    static bool correcting() { return (RTC.CTRLA & RTC_CORREN_bm) != 0; }

    // ---- flags and interrupts (26.13.3, 26.13.4) --------------------------

    static bool ovf_flag() { return (RTC.INTFLAGS & RTC_OVF_bm) != 0; }
    static bool cmp_flag() { return (RTC.INTFLAGS & RTC_CMP_bm) != 0; }
    static void clear_ovf() { RTC.INTFLAGS = RTC_OVF_bm; }
    static void clear_cmp() { RTC.INTFLAGS = RTC_CMP_bm; }

    static void enable_ovf_interrupt(bool on) { irq(RTC_OVF_bm, on); }
    static void enable_cmp_interrupt(bool on) { irq(RTC_CMP_bm, on); }

    /// ISR body for RTC_CNT_vect: which flags were up, both cleared.
    /// OVF and CMP share the vector.
    [[gnu::always_inline]] static RtcFlags take_flags() {
        const uint8_t f = RTC.INTFLAGS;
        RTC.INTFLAGS = f;
        return {(f & RTC_OVF_bm) != 0, (f & RTC_CMP_bm) != 0};
    }

    // ---- synchronization state (26.13.2) ----------------------------------

    static bool ctrla_busy() { return (RTC.STATUS & RTC_CTRLABUSY_bm) != 0; }
    static bool count_busy() { return (RTC.STATUS & RTC_CNTBUSY_bm) != 0; }
    static bool period_busy() { return (RTC.STATUS & RTC_PERBUSY_bm) != 0; }
    static bool compare_busy() { return (RTC.STATUS & RTC_CMPBUSY_bm) != 0; }
    /// Wait until no write is in flight (bounded, see rtc_wait).
    static bool sync() {
        return rtc_wait(RTC_CTRLABUSY_bm | RTC_CNTBUSY_bm | RTC_PERBUSY_bm | RTC_CMPBUSY_bm);
    }

private:
    static constexpr uint8_t calib_byte(int8_t ppm) {
        return ppm >= 0 ? static_cast<uint8_t>(ppm)
                        : static_cast<uint8_t>(RTC_SIGN_bm | static_cast<uint8_t>(-ppm));
    }
    static void irq(uint8_t bit, bool on) {
        if (on) RTC.INTCTRL |= bit; else RTC.INTCTRL &= static_cast<uint8_t>(~bit);
    }
    static void ctrla_bits(uint8_t bits, bool on) {
        rtc_wait(RTC_CTRLABUSY_bm);
        if (on) RTC.CTRLA |= bits; else RTC.CTRLA &= static_cast<uint8_t>(~bits);
    }
};

// ---- the periodic interrupt timer (26.5, 26.13.12 - 26.13.16) -----------------

/// PERIOD: how many CLK_RTC cycles between two periodic interrupts.
/// Independent of the counter's PRESCALER - the PIT taps the shared
/// prescaler chain directly.
enum class PitPeriod : uint8_t {
    off = RTC_PERIOD_OFF_gc,
    cyc4 = RTC_PERIOD_CYC4_gc,
    cyc8 = RTC_PERIOD_CYC8_gc,
    cyc16 = RTC_PERIOD_CYC16_gc,
    cyc32 = RTC_PERIOD_CYC32_gc,
    cyc64 = RTC_PERIOD_CYC64_gc,
    cyc128 = RTC_PERIOD_CYC128_gc,
    cyc256 = RTC_PERIOD_CYC256_gc,
    cyc512 = RTC_PERIOD_CYC512_gc,
    cyc1024 = RTC_PERIOD_CYC1024_gc,
    cyc2048 = RTC_PERIOD_CYC2048_gc,
    cyc4096 = RTC_PERIOD_CYC4096_gc,
    cyc8192 = RTC_PERIOD_CYC8192_gc,
    cyc16384 = RTC_PERIOD_CYC16384_gc,
    cyc32768 = RTC_PERIOD_CYC32768_gc,
};

/// CLK_RTC cycles a PitPeriod stands for (0 for `off`).
constexpr uint16_t pit_cycles(PitPeriod p) {
    const uint8_t code = static_cast<uint8_t>(
        (static_cast<uint8_t>(p) & RTC_PERIOD_gm) >> RTC_PERIOD_gp);
    return code == 0 ? 0 : static_cast<uint16_t>(4u << (code - 1));
}

/// The PIT: one interrupt every PERIOD cycles of CLK_RTC, alive in
/// every sleep mode. Its own control, status, interrupt and debug
/// registers - enabling it does not disturb the counter, and vice
/// versa; the prescaler chain they share runs while either is enabled.
class Pit {
public:
    Pit() = delete;

    /// Program the period and enable, with the periodic interrupt on by
    /// default. The FIRST interrupt lands anywhere inside one period
    /// (26.5.2.2): the phase is not knowable, only the rate is.
    static void init(PitPeriod p, bool interrupt = true) {
        rtc_pit_wait();
        RTC.PITCTRLA = static_cast<uint8_t>(static_cast<uint8_t>(p) | RTC_PITEN_bm);
        RTC.PITINTFLAGS = RTC_PI_bm;
        RTC.PITINTCTRL = interrupt ? RTC_PI_bm : 0;
    }

    static PitPeriod period() {
        return static_cast<PitPeriod>(RTC.PITCTRLA & RTC_PERIOD_gm);
    }
    /// Change the period; the phase of the next interrupt is as unknown
    /// as after an enable.
    static void period(PitPeriod p) {
        rtc_pit_wait();
        RTC.PITCTRLA = static_cast<uint8_t>((RTC.PITCTRLA & ~RTC_PERIOD_gm) |
                                            static_cast<uint8_t>(p));
    }

    static void enable() { pitctrla_bits(RTC_PITEN_bm, true); }
    static void disable() { pitctrla_bits(RTC_PITEN_bm, false); }
    static bool enabled() { return (RTC.PITCTRLA & RTC_PITEN_bm) != 0; }

    /// Interrupts per second (0 when the period is `off` or the source
    /// rate is the application's secret).
    static uint32_t tick_hz() {
        const uint16_t cyc = pit_cycles(period());
        return cyc ? RtcClock::hz() / cyc : 0;
    }

    static void enable_interrupt(bool on) { RTC.PITINTCTRL = on ? RTC_PI_bm : 0; }
    static bool interrupt_enabled() { return (RTC.PITINTCTRL & RTC_PI_bm) != 0; }

    static bool flag() { return (RTC.PITINTFLAGS & RTC_PI_bm) != 0; }
    static void clear_flag() { RTC.PITINTFLAGS = RTC_PI_bm; }

    /// ISR body for RTC_PIT_vect: clears the flag, tells whether it was
    /// set (it always is in the vector; the answer is for pollers).
    [[gnu::always_inline]] static bool take_flag() {
        const bool f = flag();
        RTC.PITINTFLAGS = RTC_PI_bm;
        return f;
    }

    static bool ctrl_busy() { return (RTC.PITSTATUS & RTC_CTRLBUSY_bm) != 0; }

    /// Keep the PIT running while the debugger holds the CPU. With this
    /// off, a break taken while the PIT output is high costs one extra
    /// interrupt when the CPU resumes (26.11).
    static void debug_run(bool on) { RTC.PITDBGCTRL = on ? RTC_DBGRUN_bm : 0; }
    static bool debug_run() { return (RTC.PITDBGCTRL & RTC_DBGRUN_bm) != 0; }

private:
    static void pitctrla_bits(uint8_t bits, bool on) {
        rtc_pit_wait();
        if (on) RTC.PITCTRLA |= bits; else RTC.PITCTRLA &= static_cast<uint8_t>(~bits);
    }
};

} // namespace brio
