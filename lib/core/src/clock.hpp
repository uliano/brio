/*
 * clock.hpp
 *
 * Main-clock initialization for the AVR DA/DB families.
 * Ported from uliano/AVR-Multislope (src/clocks.h) and merged with the
 * deterministic 24 MHz crystal path used by this repo's boards.
 *
 * Two entry points are provided:
 *
 *  - init_clocks()       generic, probing: brings CLK_PER to 24 MHz starting
 *                        from the internal OSCHF, then probes (in order) a
 *                        DB HF crystal on PA0/PA1, an external clock on PA0,
 *                        and a 32.768 kHz crystal on PF0/PF1 (enabling OSCHF
 *                        autotune when only the 32k crystal is present).
 *                        Returns a ClockInitCode describing what was found.
 *                        Use it on boards whose clock fixture is unknown.
 *
 *  - init_clock_24mhz()  deterministic, DB only: the 24 MHz crystal on
 *                        PA0/PA1 is a known fixture; start it and switch
 *                        CLK_PER to it, falling back to OSCHF @ 24 MHz if it
 *                        does not start. Returns true when running from the
 *                        crystal. No 32k probing: the RTC/ticker falls back
 *                        to the internal OSC32K on its own.
 *
 * Either way CLK_PER ends up at 24 MHz, so F_CPU=24000000 stays correct.
 * CLKCTRL registers are CCP-protected, hence _PROTECTED_WRITE.
 */

#pragma once

#include <avr/io.h>
#include <stdint.h>
#include <utility>  // std::to_underlying

namespace dx {

enum class ClockInitCode : uint8_t {
    MainMask = 0x0F,

    MainOschf24M = 0x00,       // Internal OSCHF @ 24 MHz
    MainExtclkPa0 = 0x01,      // External HF clock on PA0
    MainDbXtalhfPa0Pa1 = 0x02, // DB only: HF crystal on PA0/PA1

    HasXosc32k = 0x10,         // 32.768 kHz crystal detected on PF0/PF1
    OschfAutotuned = 0x20,     // OSCHF autotune enabled from XOSC32K
    DeviceDb = 0x40,           // Built for DB family
    DeviceDa = 0x80            // Built for DA family
};

static inline constexpr uint8_t clock_code_u8(ClockInitCode code)
{
    return std::to_underlying(code);
}

static inline constexpr ClockInitCode clock_code_from_u8(uint8_t code)
{
    return static_cast<ClockInitCode>(code);
}

static inline ClockInitCode clock_main_source(ClockInitCode code)
{
    return clock_code_from_u8(
        static_cast<uint8_t>(clock_code_u8(code) & clock_code_u8(ClockInitCode::MainMask)));
}

static inline bool clock_has_flag(ClockInitCode code, ClockInitCode flag)
{
    return (clock_code_u8(code) & clock_code_u8(flag)) != 0u;
}

static inline const char *clock_main_source_str(ClockInitCode code)
{
    switch (clock_main_source(code)) {
    case ClockInitCode::MainExtclkPa0:
        return "EXTCLK PA0";
    case ClockInitCode::MainDbXtalhfPa0Pa1:
        return "DB XOSCHF crystal PA0/PA1";
    case ClockInitCode::MainOschf24M:
    default:
        return "OSCHF 24MHz";
    }
}

static inline const char *clock_device_family_str(ClockInitCode code)
{
    if (clock_has_flag(code, ClockInitCode::DeviceDb)) {
        return "DB";
    }
    if (clock_has_flag(code, ClockInitCode::DeviceDa)) {
        return "DA";
    }
    return "unknown";
}

static inline uint8_t clock_wait_status(uint8_t mask, uint32_t timeout)
{
    while (timeout--) {
        if (CLKCTRL.MCLKSTATUS & mask) {
            return 1;
        }
    }
    return 0;
}

static inline void set_oschf_24mhz(uint8_t autotune)
{
    uint8_t value = CLKCTRL_FRQSEL_24M_gc | CLKCTRL_RUNSTDBY_bm;
    if (autotune) {
        value |= CLKCTRL_AUTOTUNE_bm;
    }
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, value);
}

static inline uint8_t probe_extclk_pa0(void)
{
#if defined(CLKCTRL_XOSCHFCTRLA)
    // DB: probe PA0 as XOSCHF external clock input.
    _PROTECTED_WRITE(
        CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_ENABLE_bm |
            CLKCTRL_RUNSTDBY_bm |
            CLKCTRL_SELHF_EXTCLOCK_gc |
            CLKCTRL_FRQRANGE_24M_gc |
            CLKCTRL_CSUTHF_256_gc);
    return clock_wait_status(CLKCTRL_EXTS_bm, 0x0FFFu);
#else
    // DA: request EXTCLK through PLL (without switching MCLK).
    _PROTECTED_WRITE(
        CLKCTRL.PLLCTRLA,
        CLKCTRL_SOURCE_bm | CLKCTRL_MULFAC_2x_gc);
    const uint8_t found = clock_wait_status(CLKCTRL_EXTS_bm, 0x0FFFu);
    _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, CLKCTRL_MULFAC_DISABLE_gc);
    return found;
#endif
}

#if defined(CLKCTRL_XOSCHFCTRLA)
static inline uint8_t start_db_hf_crystal_pa0_pa1(void)
{
    _PROTECTED_WRITE(
        CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_ENABLE_bm |
            CLKCTRL_RUNSTDBY_bm |
            CLKCTRL_SELHF_XTAL_gc |
            CLKCTRL_FRQRANGE_24M_gc |
            CLKCTRL_CSUTHF_4K_gc);
    return clock_wait_status(CLKCTRL_EXTS_bm, 0xFFFFu);
}
#endif

static inline uint8_t probe_xosc32k_crystal_pf0_pf1(void)
{
    _PROTECTED_WRITE(
        CLKCTRL.XOSC32KCTRLA,
        CLKCTRL_ENABLE_bm |
            CLKCTRL_RUNSTDBY_bm |
            CLKCTRL_CSUT_64K_gc);
    return clock_wait_status(CLKCTRL_XOSC32KS_bm, 0x0FFFFFu);
}

// Generic probing init for boards whose clock fixture is unknown.
// See the header comment for the probe order and the returned code.
static inline ClockInitCode init_clocks(void)
{
    uint8_t result = clock_code_u8(ClockInitCode::MainOschf24M);

#if defined(CLKCTRL_XOSCHFCTRLA)
    result |= clock_code_u8(ClockInitCode::DeviceDb);
#else
    result |= clock_code_u8(ClockInitCode::DeviceDa);
#endif

    // 1) Baseline: internal high-frequency oscillator at 24 MHz.
    set_oschf_24mhz(0);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
    (void)clock_wait_status(CLKCTRL_OSCHFS_bm, 0xFFFFu);

    // 2) Probe high-frequency external source.
    //
    // On DB we prioritize PA0/PA1 crystal detection first. In practice, probing
    // EXTCLK first can report a false-positive classification on crystal-based
    // boards.
    uint8_t has_hf_external = 0;
#if defined(CLKCTRL_XOSCHFCTRLA)
    if (start_db_hf_crystal_pa0_pa1()) {
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
        (void)clock_wait_status(CLKCTRL_EXTS_bm, 0xFFFFu);
        result = static_cast<uint8_t>(
            (result & ~clock_code_u8(ClockInitCode::MainMask)) |
            clock_code_u8(ClockInitCode::MainDbXtalhfPa0Pa1));
        has_hf_external = 1;
    }
#endif
    if (!has_hf_external && probe_extclk_pa0()) {
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
        (void)clock_wait_status(CLKCTRL_EXTS_bm, 0x0FFFu);
        result = static_cast<uint8_t>(
            (result & ~clock_code_u8(ClockInitCode::MainMask)) |
            clock_code_u8(ClockInitCode::MainExtclkPa0));
        has_hf_external = 1;
    }

    // 3) Probe 32.768 kHz crystal on PF0/PF1 with long timeout.
    if (probe_xosc32k_crystal_pf0_pf1()) {
        result |= clock_code_u8(ClockInitCode::HasXosc32k);

        // If no external HF source was found, run OSCHF in autotune.
        if (!has_hf_external) {
            set_oschf_24mhz(1);
            result |= clock_code_u8(ClockInitCode::OschfAutotuned);
        }
    } else {
        // If the 32k crystal is absent, release the pins/peripheral.
        _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, 0);
    }

    return clock_code_from_u8(result);
}

#if defined(CLKCTRL_XOSCHFCTRLA)
// Deterministic init for DB boards where the 24 MHz crystal on PA0/PA1 is a
// known fixture (no probing, no XOSC32K touch). Returns true if running from
// the crystal, false on OSCHF fallback; CLK_PER is 24 MHz either way.
static inline bool init_clock_24mhz(void)
{
    // Keep a sane 24 MHz OSCHF as the starting (and fallback) main clock.
    set_oschf_24mhz(0);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
    (void)clock_wait_status(CLKCTRL_OSCHFS_bm, 0xFFFFu);

    // Start the 24 MHz crystal on PA0/PA1 (4k-cycle start-up time).
    if (!start_db_hf_crystal_pa0_pa1()) {
        // Crystal did not start: release it and stay on OSCHF.
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, 0);
        return false;
    }

    // Switch the main clock to the crystal.
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    (void)clock_wait_status(CLKCTRL_EXTS_bm, 0xFFFFu);
    return true;
}
#endif

} // namespace dx
