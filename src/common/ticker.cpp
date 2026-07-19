// Ported from uliano/AVR-Multislope (lib/core/src/ticker.cpp).
/**
 * @file ticker.cpp
 * @author uliano
 * @brief Implementation of RTC initialization for the Ticker system
 * @date Created: 01/9/2026
 * @revised 01/9/2026
 *
 * This file provides the init_ticker() function which configures the AVR's
 * Real-Time Clock (RTC) peripheral to use the 32.768 kHz oscillator and
 * initializes the Ticker singleton.
 */

#include <avr/io.h>
#include "ticker.hpp"

/**
 * @brief Initialize RTC peripheral and Ticker singleton with automatic clock source selection
 *
 * This function performs the complete initialization sequence:
 *
 * 1. **Wait for RTC synchronization**
 *    - Ensures any pending RTC register writes have completed
 *    - Critical for reliable RTC operation after power-up or reset
 *
 * 2. **Configure clock source (automatic selection)**
 *    - Checks if external 32.768 kHz crystal (XOSC32K) is stable
 *    - If XOSC32K is available: uses RTC_CLKSEL_XOSC32K_gc (high precision)
 *    - If XOSC32K is not available: falls back to RTC_CLKSEL_OSC32K_gc (internal 32kHz)
 *    - The auto-detection relies on init_clocks() being called first
 *
 * 3. **Initialize Ticker singleton**
 *    - Creates the Ticker instance (via Ticker::instance())
 *    - Calls ticker.init() which:
 *      - Sets up Ticker::ptr for ISR access
 *      - Resets all time counters
 *      - Configures PIT period based on ticks_per_second
 *      - Enables the PIT interrupt
 *
 * ## Commented-out RTC Counter Configuration
 * The commented lines show how to enable the RTC counter mode if needed:
 * - RTC.CTRLA: Enables RTC counter with prescaler and standby operation
 * - RTC.PER: Sets the period for RTC overflow interrupts
 *
 * These are not needed for PIT-only operation (current implementation).
 *
 * @note Must be called AFTER init_clocks() and before enabling global interrupts
 * @note After this call, you must define the ISR: ISR(RTC_PIT_vect) { Ticker::ptr->pit(); }
 *
 * @see Ticker::init() for details on Ticker initialization
 * @see ticker.hpp for complete system documentation
 */
void init_ticker(void){

    // Wait for RTC peripheral synchronization
    // STATUS register bits indicate pending synchronization operations
    while (RTC.STATUS > 0);

    // Optional: Enable RTC counter mode (not needed for PIT-only operation)
    // RTC.CTRLA = RTC_PRESCALER_DIV1_gc   /* 1:1 prescaler */
    //           | 1 << RTC_RTCEN_bp       /* Enable RTC counter */
    //           | 1 << RTC_RUNSTDBY_bp;   /* Run during standby sleep */
    //
    // RTC.PER = 0x0;  /* RTC overflow period (0 = maximum period) */

    // Auto-select RTC clock source based on availability
    // Check if external 32.768 kHz crystal was successfully started by init_clocks()
    if (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm) {
        // External crystal is stable - use it for high precision
        RTC.CLKSEL = RTC_CLKSEL_XOSC32K_gc;
    } else {
        // External crystal not available - fall back to internal 32kHz oscillator
        RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;
    }

    // Create and initialize the Ticker singleton
    Ticker& ticker = Ticker::instance();
    ticker.init();  // Configures PIT and enables interrupt
}
