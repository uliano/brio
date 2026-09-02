/*
 * flash.hpp
 *
 * The FLASH interface (RM0444 ch. 3), bring-up surface only: the read
 * access latency the clock task must set before it raises HCLK, and the
 * two CPU-side accelerators the same register carries. Programming,
 * erasing, the option bytes, ECC and the read/write protections are the
 * FLASH campaign's (this file grows into the FlashMedia backend that
 * util/nv_journal.hpp was host-validated against: 2048-byte pages,
 * 8-byte double-word writes).
 *
 * WHY THE WAIT STATES LIVE HERE AND NOT IN clock.hpp: they are the flash
 * interface's register, and the samc stratum paid for the other choice
 * (clock.hpp squatted on NVMCTRL's RWS until the NVM campaign took it
 * back). The clock task calls in; this file owns the register.
 *
 * Facts that shape the code (RM0444 3.3.4, 3.7.1):
 *  - table 13: at VCORE Range 1 (the reset range, and the only one this
 *    stratum runs in) HCLK <= 24 MHz needs 0 wait states, <= 48 needs 1,
 *    <= 64 needs 2; Range 2 halves the ceilings (8 / 16 MHz) and forbids
 *    2 WS. Out of reset HCLK is 16 MHz at 0 WS.
 *  - the ORDER is the same rule as on every target: wait states go UP
 *    before a frequency rise and DOWN after a fall, and a new LATENCY
 *    value is in force only when it READS BACK - 3.7.1 says so in one
 *    sentence, and the samc side proved the cost of not waiting.
 *  - ICEN (instruction cache) is set at reset, PRFTEN (prefetch) is
 *    clear. This stratum leaves both at their reset values: erratum
 *    ES0548 2.2.10 (prefetch failure branching across the two banks of
 *    a dual-bank part, no workaround) makes PRFTEN a decision to take
 *    knowingly, with a measurement, not a default.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

namespace brio {

/// FLASH_ACR.LATENCY - the read wait states, and the rule that picks
/// them. Monostate: one flash interface per device.
struct FlashWaitStates {
    FlashWaitStates() = delete;

    static constexpr uint8_t max_latency = 2;

    static uint8_t get() {
        return static_cast<uint8_t>(FLASH->ACR & FLASH_ACR_LATENCY_Msk);
    }

    /// Program `ws` and wait until it reads back (3.7.1: the write
    /// becomes effective when it returns the same value upon read).
    /// Bounded: a value the field cannot take (> 2) would never read
    /// back, so it is refused instead, and false says so.
    static bool set(uint8_t ws) {
        if (ws > max_latency) {
            return false;
        }
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) |
                     (static_cast<uint32_t>(ws) << FLASH_ACR_LATENCY_Pos);
        for (uint32_t spins = 0; spins < 1000u; ++spins) {
            if (get() == ws) {
                return true;
            }
        }
        return false;
    }

    /// Table 13, VCORE Range 1 column: the wait states HCLK needs.
    static constexpr uint8_t for_hz(uint32_t hz) {
        if (hz <= 24'000'000UL) return 0;
        if (hz <= 48'000'000UL) return 1;
        return 2;
    }

    /// Table 13, Range 2 column - declared for the day a low-power
    /// clock task runs the core in Range 2; nothing calls it yet.
    static constexpr uint8_t for_hz_range2(uint32_t hz) {
        return hz <= 8'000'000UL ? 0 : 1;
    }
};

/// The two CPU-side accelerators in FLASH_ACR. Readback verbs only need
/// a plain load; the setters are plain stores (no synchronization).
struct FlashAccel {
    FlashAccel() = delete;

    static bool prefetch() { return (FLASH->ACR & FLASH_ACR_PRFTEN) != 0u; }
    static void prefetch(bool on) {
        FLASH->ACR = on ? (FLASH->ACR | FLASH_ACR_PRFTEN) : (FLASH->ACR & ~FLASH_ACR_PRFTEN);
    }

    static bool instruction_cache() { return (FLASH->ACR & FLASH_ACR_ICEN) != 0u; }
    static void instruction_cache(bool on) {
        FLASH->ACR = on ? (FLASH->ACR | FLASH_ACR_ICEN) : (FLASH->ACR & ~FLASH_ACR_ICEN);
    }
};

/// The flash size the part reports, in kilobytes (RM0444 41.2: the
/// register holds it in KB; the header's FLASH_SIZE macro is the same
/// read in bytes).
inline uint32_t flash_size_kb() {
    return *reinterpret_cast<const volatile uint32_t*>(FLASHSIZE_BASE) & 0x03FFu;
}

} // namespace brio
