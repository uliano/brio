/*
 * pwr.hpp
 *
 * The power controller (RM0090 ch. 5, RM0390 ch. 5, RM0383 ch. 5) - the
 * part of it the CLOCK TASK needs: the block's bus clock, the main
 * regulator's VOLTAGE SCALE and the OVER-DRIVE pair. The rest of the
 * chapter - the Sleep/Stop/Standby ladder and the util/power.hpp sites
 * over it, the PVD, the backup domain's protection, the wake-up pin -
 * is the power chapter's, and it will grow this file rather than open
 * another: one chapter, one owner.
 *
 * WHY THE SCALE IS A CLOCK QUESTION. RM0090 5.1.4: the regulator's
 * output "can be scaled by software to different voltage values (scale
 * 1, scale 2, and scale 3 ...). The scale can be modified only when
 * the PLL is OFF and the HSI or HSE clock source is selected as system
 * clock source. The new value programmed is active only when the PLL is
 * ON." Each scale carries a HCLK ceiling (stm32f4/device_tables.hpp's
 * ladder), and the top of the ladder - 168 or 180 MHz on the parts
 * that reach it - is reached only in OVER-DRIVE, entered by a sequence
 * that belongs between the PLL's start and the switch to it (5.1.4,
 * "Entering Over-drive mode"): ODEN then wait ODRDY, ODSWEN then wait
 * ODSWRDY - the system is stalled during the switch. Entering Stop
 * disables the over-drive mode and the PLL; whoever brings the part
 * back does both again.
 *
 * ONE VOS BIT OR TWO. The F405/F407 class has a single VOS bit (scale 2
 * and scale 1, PWR_CR bit 14); every later part has two (scales 3, 2
 * and 1). The reserve says which (pwr_vos_two_bits()), and the code
 * for the scale is written once against that fact.
 *
 * THE OVER-DRIVE VERBS ARE PREPROCESSOR-GUARDED, not if-constexpr'd:
 * `Pwr` is a plain struct and a discarded branch of a non-template is
 * still checked, so a header without ODEN would not compile the name.
 * The one fact the guards ask - is PWR_CR_ODEN declared? - is the
 * reserve's own probe, so the two can never disagree.
 *
 * THE BLOCK IS BEHIND A GATE. PWR sits on APB1 behind RCC_APB1ENR.PWREN,
 * clear at reset; a register read through the closed gate answers zero
 * and a write is dropped in silence. Every verb here that touches a
 * register opens the gate first (bus_clock(true), idempotent), the
 * STM32G0 lesson.
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"

namespace brio {

/// The main regulator's scale in Run mode. The numbers are the
/// chapter's: scale 1 is the highest voltage (and the highest HCLK),
/// scale 3 the lowest.
enum class VoltageScale : uint8_t { scale3 = 3, scale2 = 2, scale1 = 1 };

/// PWR as a monostate resource.
struct Pwr {
    Pwr() = delete;

    static constexpr uint32_t ready_spins = 1'000'000u;

    /// Open (or close) the block's APB1 gate, with the readback that
    /// covers the enable's propagation (ES0206 2.2.7 and its twins: a
    /// peripheral register written right after its clock enable may not
    /// take the store; the readback is the recommended dummy access).
    static void bus_clock(bool on) {
        if (on) {
            RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        } else {
            RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
        }
        (void)RCC->APB1ENR;
    }
    static bool bus_clock() { return (RCC->APB1ENR & RCC_APB1ENR_PWREN) != 0u; }

    /// Whether `s` exists on this part: scale 3 does not on the one-bit
    /// VOS class.
    static constexpr bool scale_exists(VoltageScale s) {
        return pwr_vos_two_bits() || s != VoltageScale::scale3;
    }

    /// Program the regulator scale. The chapter's precondition - PLL off,
    /// HSI or HSE as SYSCLK - is the caller's (the clock task honours
    /// it); the value takes effect when the PLL turns on. False when the
    /// scale does not exist on this part; nothing written then.
    static bool scale(VoltageScale s) {
        if (!scale_exists(s)) {
            return false;
        }
        bus_clock(true);
        if constexpr (pwr_vos_two_bits()) {
            PWR->CR = (PWR->CR & ~PWR_CR_VOS_Msk) |
                      (static_cast<uint32_t>(vos_code(s)) << PWR_CR_VOS_Pos);
        } else {
            // One bit: set = scale 1, clear = scale 2 (RM0090 5.4.1).
            if (s == VoltageScale::scale1) {
                PWR->CR |= PWR_CR_VOS;
            } else {
                PWR->CR &= ~PWR_CR_VOS;
            }
        }
        return true;
    }

    /// The scale in force (or programmed, PLL still off).
    static VoltageScale scale() {
        bus_clock(true);
        if constexpr (pwr_vos_two_bits()) {
            const uint8_t code = static_cast<uint8_t>((PWR->CR & PWR_CR_VOS_Msk) >> PWR_CR_VOS_Pos);
            return scale_of(code);
        } else {
            return (PWR->CR & PWR_CR_VOS) != 0u ? VoltageScale::scale1 : VoltageScale::scale2;
        }
    }

    /// PWR_CSR.VOSRDY: the regulator has settled at the programmed
    /// scale - after the PLL is on and the value is in force.
    static bool scale_ready() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_VOSRDY) != 0u;
    }

    /// Whether this part has the over-drive mode at all (the reserve's
    /// PWR_CR_ODEN probe).
    static constexpr bool has_over_drive() { return pwr_has_over_drive(); }

    /// Enter over-drive: ODEN and wait for ODRDY, ODSWEN and wait for
    /// ODSWRDY (RM0090 5.1.4 steps 3..5). To be called with the PLL
    /// configured and ON but not yet the system clock, and BEFORE the
    /// peripheral clocks are enabled ("during the Over-drive switch
    /// activation, no peripheral clocks should be enabled"). Bounded
    /// waits: false when either flag does not rise. Always false, and
    /// nothing written, on a part without the mode.
    static bool over_drive_enter() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        PWR->CR |= PWR_CR_ODEN;
        if (!wait_csr(PWR_CSR_ODRDY)) {
            return false;
        }
        PWR->CR |= PWR_CR_ODSWEN;
        return wait_csr(PWR_CSR_ODSWRDY);
#else
        return false;
#endif
    }

    /// Leave over-drive, both bits at once (5.1.4, sequence 1). The
    /// caller has SYSCLK on HSI or HSE, per the chapter.
    static void over_drive_exit() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        PWR->CR &= ~(PWR_CR_ODEN | PWR_CR_ODSWEN);
#endif
    }

    /// Whether the regulator is in over-drive right now (ODSWRDY).
    static bool over_drive_active() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_ODSWRDY) != 0u;
#else
        return false;
#endif
    }

private:
    /// The two-bit VOS encoding (RM0090 5.4.1 for the F42x/F43x, RM0390,
    /// RM0383): 01 scale 3, 10 scale 2, 11 scale 1; 00 is Reserved and
    /// reads as scale 3.
    static constexpr uint8_t vos_code(VoltageScale s) {
        switch (s) {
            case VoltageScale::scale3: return 1;
            case VoltageScale::scale2: return 2;
            case VoltageScale::scale1: return 3;
        }
        return 3;
    }
    static constexpr VoltageScale scale_of(uint8_t code) {
        switch (code) {
            case 2: return VoltageScale::scale2;
            case 3: return VoltageScale::scale1;
            default: return VoltageScale::scale3;
        }
    }
    static bool wait_csr(uint32_t mask) {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if ((PWR->CSR & mask) != 0u) {
                return true;
            }
        }
        return false;
    }
};

} // namespace brio
