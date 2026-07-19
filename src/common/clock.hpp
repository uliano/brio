#pragma once
#include <avr/io.h>

// Set the main clock to 24 MHz from the external crystal on PA0/PA1 (XOSCHF,
// DB family only), falling back to the internal OSCHF @ 24 MHz if the crystal
// fails to start. Either way CLK_PER ends up at 24 MHz, so F_CPU and all
// timing stay correct; the return value tells which source won.
//
// This board HAS a 24 MHz crystal on PA0/PA1 (those pins are therefore not
// available as GPIO). Register sequence and constants follow the proven
// crystal path of uliano/AVR-Multislope (src/clocks.h), without the probing:
// the crystal is a known fixture here, not something to discover.
//
// No 32.768 kHz crystal is assumed: the RTC/ticker falls back to the internal
// OSC32K automatically (see init_ticker()); XOSC32K is left untouched so
// PF0/PF1 stay free for USART2.
//
// CLKCTRL registers are CCP-protected, hence _PROTECTED_WRITE.

static inline uint8_t clock_wait_status_(uint8_t mask, uint32_t timeout) {
    while (timeout--) {
        if (CLKCTRL.MCLKSTATUS & mask) return 1;
    }
    return 0;
}

// Returns true if running from the PA0/PA1 crystal, false on OSCHF fallback.
static inline bool init_clock_24mhz() {
    // Keep a sane 24 MHz OSCHF as the starting (and fallback) main clock.
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc | CLKCTRL_RUNSTDBY_bm);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
    (void)clock_wait_status_(CLKCTRL_OSCHFS_bm, 0xFFFFu);

    // Start the 24 MHz crystal on PA0/PA1 (4k-cycle start-up time).
    _PROTECTED_WRITE(
        CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_ENABLE_bm |
            CLKCTRL_RUNSTDBY_bm |
            CLKCTRL_SELHF_XTAL_gc |
            CLKCTRL_FRQRANGE_24M_gc |
            CLKCTRL_CSUTHF_4K_gc);
    if (!clock_wait_status_(CLKCTRL_EXTS_bm, 0xFFFFu)) {
        // Crystal did not start: release it and stay on OSCHF.
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, 0);
        return false;
    }

    // Switch the main clock to the crystal.
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    (void)clock_wait_status_(CLKCTRL_EXTS_bm, 0xFFFFu);
    return true;
}
