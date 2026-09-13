/*
 * flash.hpp
 *
 * The embedded flash memory interface (RM0090 ch. 3, RM0390 ch. 3,
 * RM0383 ch. 3) - the part the CLOCK TASK needs: the read access
 * LATENCY the task must set before it raises HCLK, and the ART
 * accelerator's three switches. The program/erase engine, the option
 * bytes and the OTP are the NVM chapter's, and they will grow this
 * file rather than open another: one chapter, one owner.
 *
 * WHY THE WAIT STATES LIVE HERE AND NOT IN clock.hpp: they are the
 * flash interface's register. The clock task calls in; this file owns
 * the register.
 *
 * Facts that shape the code (RM0090 3.5.1 and table 12, RM0383 3.4.1):
 *  - the wait states are a function of HCLK AND of the supply voltage;
 *    stm32f4/device_tables.hpp carries each part class's 2.7..3.6 V
 *    column (0 WS up to 30 MHz, one more per 30 MHz band, 5 WS at
 *    180 MHz on the F42x/F43x and F446; the F411's bands are 30, 64,
 *    90 and 100 MHz). Out of reset HCLK is 16 MHz at 0 WS;
 *  - the ORDER: to increase the frequency, program the new latency,
 *    "check that the new number of wait states is taken into account
 *    by reading the FLASH_ACR register", then change the clock; to
 *    decrease it, change the clock first and the latency after (3.5.1's
 *    two sequences). set() below does the readback;
 *  - the ART ACCELERATOR (3.5.2): an instruction prefetch buffer
 *    (PRFTEN), a 64-line instruction cache (ICEN) and an 8-line data
 *    cache (DCEN), all three CLEAR at reset. With them off a 5-wait-
 *    state core loses most of its speed to the flash; the clock task
 *    turns all three on when it raises the rate. A cache may be RESET
 *    (ICRST, DCRST) only while it is disabled (3.9.1);
 *  - the LATENCY field is four bits on the parts whose ladder reaches
 *    180 MHz and three on the others (FLASH_ACR_LATENCY_Msk says which;
 *    max_latency below reads it).
 *
 * The errata sheets of the three bench parts (ES0206 Rev 24, ES0298
 * Rev 8, ES0287 Rev 6) carry no item against the accelerator or the
 * latency; the prefetch is therefore ON by default here where the
 * STM32G0's stratum leaves it off (that family's erratum has no twin
 * on this one).
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"

namespace brio {

/// FLASH_ACR.LATENCY - the read wait states, and the rule that picks
/// them. Monostate: one flash interface per device.
struct FlashWaitStates {
    FlashWaitStates() = delete;

    /// The field's own ceiling: 15 where LATENCY is four bits, 7 where
    /// it is three.
    static constexpr uint8_t max_latency = static_cast<uint8_t>(FLASH_ACR_LATENCY_Msk >> FLASH_ACR_LATENCY_Pos);

    static uint8_t get() {
        return static_cast<uint8_t>((FLASH->ACR & FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);
    }

    /// Program `ws` and wait until it reads back (3.5.1). Bounded: a
    /// value the field cannot take is refused instead, and false says so.
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

    /// The wait states `hclk` needs on this part at 2.7..3.6 V, from
    /// the reserve's ladder; 0xFF when the ladder is not known or the
    /// rate is beyond it (the clock task refuses at compile time first).
    static constexpr uint8_t needed_for(uint32_t hclk) { return flash_wait_states_for(hclk); }
};

/// The ART accelerator's switches (3.5.2, 3.9.1).
struct FlashAccel {
    FlashAccel() = delete;

    static void prefetch(bool on) { bit(FLASH_ACR_PRFTEN, on); }
    static bool prefetch() { return (FLASH->ACR & FLASH_ACR_PRFTEN) != 0u; }

    static void icache(bool on) { bit(FLASH_ACR_ICEN, on); }
    static bool icache() { return (FLASH->ACR & FLASH_ACR_ICEN) != 0u; }

    static void dcache(bool on) { bit(FLASH_ACR_DCEN, on); }
    static bool dcache() { return (FLASH->ACR & FLASH_ACR_DCEN) != 0u; }

    /// Reset the instruction cache: legal only while it is disabled
    /// (3.9.1), so a request on an enabled cache is refused.
    static bool icache_reset() {
        if (icache()) {
            return false;
        }
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
        return true;
    }
    static bool dcache_reset() {
        if (dcache()) {
            return false;
        }
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        return true;
    }

    /// All three on: what the clock task does when it raises the rate.
    static void enable_all() {
        FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    }

private:
    static void bit(uint32_t mask, bool on) {
        if (on) {
            FLASH->ACR |= mask;
        } else {
            FLASH->ACR &= ~mask;
        }
    }
};

/// The device electronic signature (RM0090 ch. 39, RM0390 ch. 34,
/// RM0383 ch. 24): the 96-bit unique ID, the flash size in Kbytes and
/// the package code, in system memory at the addresses the header
/// states. Read-only by nature.
struct DeviceUid {
    uint32_t word[3];

    static DeviceUid read() {
        const volatile uint32_t* p = reinterpret_cast<const volatile uint32_t*>(device_uid_base);
        return DeviceUid{{p[0], p[1], p[2]}};
    }
};

/// FLASHSIZE: the device's flash in Kbytes (0x200 = 512 K, 0x800 = 2 M).
inline uint16_t flash_size_kbytes() {
    return *reinterpret_cast<const volatile uint16_t*>(flash_size_register);
}

/// DBGMCU_IDCODE (RM0090 38.6.1): DEV_ID names the part class (0x419 the
/// F42x/F43x, 0x421 the F446, 0x431 the F411), REV_ID the silicon
/// revision the errata sheets key on.
struct DeviceIdcode {
    uint16_t dev_id;
    uint16_t rev_id;

    static DeviceIdcode read() {
        const uint32_t v = DBGMCU->IDCODE;
        return DeviceIdcode{static_cast<uint16_t>(v & DBGMCU_IDCODE_DEV_ID_Msk),
                            static_cast<uint16_t>((v & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos)};
    }
};

} // namespace brio
