/*
 * syscfg.hpp
 *
 * The STM32F4's system configuration controller (RM0090 ch. 9, RM0390
 * ch. 8, RM0383 ch. 7) - a handful of unrelated switches that share one
 * register block and one clock gate:
 *
 *  Syscfg   the memory seen at address 0, the GPIO multiplexer that feeds
 *           the EXTI's sixteen pin lines, the I/O compensation cell, and
 *           the Ethernet PHY interface select.
 *
 * WHY IT IS ITS OWN FILE AND NOT PART OF exti.hpp. On the STM32G0 the pin
 * multiplexer is a register of the EXTI itself; here it is four registers
 * of ANOTHER peripheral, in another chapter, behind another clock gate -
 * and the three switches beside it have nothing to do with interrupts.
 * stm32f4/exti.hpp includes this file and calls exti_source() through it;
 * a program that only wants the compensation cell never mentions the EXTI.
 *
 * THE GATE IS THE ONE THING TO KNOW. SYSCFG sits on APB2 behind
 * RCC_APB2ENR.SYSCFGEN, which is CLEAR out of reset (RM0090 7.3.14): with
 * it clear every register of the block reads back zero and every write is
 * dropped in silence - the same trap as an unclocked GPIO port. So EVERY
 * verb here opens the gate first (an idempotent read-modify-write of one
 * RCC bit, plus the readback that is the dummy access ES0206 2.2.7 asks
 * for) rather than only the configuring ones: a read of MEMRMP is just as
 * likely to be a program's first touch of the block as a write of EXTICR.
 * Nothing here ever closes it - a second user of SYSCFG would lose its
 * block; that is a program-wide decision and clock(false) exists for it.
 *
 * WHAT THIS FILE DOES NOT CARRY. SYSCFG_CFGR (the FMPI2C1 Fm+ drive on the
 * F446 and F410) and SYSCFG_PMC's ADCxDC2 bits are struct members and bit
 * masks that exist on some headers and not others, and each belongs to a
 * peripheral chapter that has no driver in this stratum yet; they will be
 * born with their first user. MEMRMP is READ ONLY here on purpose: moving
 * what lives at address 0 under a running program is not a verb, it is a
 * boot decision the BOOT pins already made.
 *
 * CONCURRENCY. Every verb is a read-modify-write of one field of one
 * register with no set/clear twin, so two contexts configuring SYSCFG at
 * once can lose a field - the same contract as stm32f4/pin.hpp's
 * configuring verbs, and the same intended use: setup and kernel time.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"

namespace brio {

/// What SYSCFG_MEMRMP's MEM_MODE field says is mapped at address
/// 0x0000 0000 (RM0090 9.2.1). The reset value is whatever the BOOT pins
/// chose, which is why this is read and not written.
enum class MemoryMap : uint8_t {
    main_flash = 0,
    system_flash = 1,
    external_memory = 2,   ///< FSMC/FMC bank 1, on the parts that have one
    sram = 3,
};

class Syscfg {
public:
    Syscfg() = delete;

    static SYSCFG_TypeDef& regs() { return *SYSCFG; }

    // ---- the clock gate (RM0090 7.3.14) ------------------------------------

    /// Open or close RCC_APB2ENR.SYSCFGEN. Every other verb of this class
    /// opens it first, so an application only calls this to CLOSE the gate.
    static void clock(bool on) { Rcc::apb2_clock(syscfg_clock_mask, on); }
    static bool clock() { return Rcc::apb2_clock(syscfg_clock_mask); }

    // ---- the memory map (9.2.1) --------------------------------------------

    /// What is visible at address 0 right now: the BOOT pins' choice
    /// unless something has overridden it.
    static MemoryMap memory_map() {
        clock(true);
        return static_cast<MemoryMap>(regs().MEMRMP & 0x3u);
    }

    // ---- the EXTI's GPIO multiplexer (9.2.3 .. 9.2.6) ----------------------

    /// Point EXTI line `line` (0..15) at GPIO port `port`: the ONE choice
    /// the sixteen pin lines offer, and the reason two pads with the same
    /// pin number cannot both raise interrupts. Refuses a line that is not
    /// a GPIO line and a port this device does not bond.
    ///
    /// There is no arbitration in the silicon: the last write owns the
    /// line and the previous pad goes quiet with no flag anywhere. The
    /// refusal that makes that visible is stm32f4/exti.hpp's
    /// Exti::select(), which asks whether the line is in use first; this
    /// verb is the register store under it.
    static bool exti_source(uint8_t line, char port) {
        const uint8_t code = exti_port_code(port);
        if (line >= exti_gpio_lines || code == 0xFFu) {
            return false;
        }
        clock(true);
        const uint32_t reg = line / 4u;
        const uint32_t shift = static_cast<uint32_t>(line % 4u) * 4u;
        regs().EXTICR[reg] = (regs().EXTICR[reg] & ~(0xFUL << shift)) |
                             (static_cast<uint32_t>(code) << shift);
        return true;
    }

    /// Which port feeds line `line` right now, as a letter; 0 for a line
    /// that is not a GPIO line or whose field holds a code this device has
    /// no port for. EXTICR's reset value is zero, so an untouched line
    /// reads 'A'.
    static char exti_source(uint8_t line) {
        if (line >= exti_gpio_lines) {
            return 0;
        }
        clock(true);
        const uint32_t reg = line / 4u;
        const uint32_t shift = static_cast<uint32_t>(line % 4u) * 4u;
        const uint8_t code = static_cast<uint8_t>((regs().EXTICR[reg] >> shift) & 0xFu);
        const char letter = static_cast<char>('A' + code);
        return exti_port_code(letter) == code ? letter : static_cast<char>(0);
    }

    // ---- the I/O compensation cell (9.1, 9.2.7) ----------------------------
    //
    // 9.1: the cell is off by default and is RECOMMENDED once an output
    // buffer is driven at the 50 or 100 MHz speed class, to hold the rise
    // and fall times steady and keep the switching noise off the supply. It
    // is legal only from 2.4 V up. It is not a correctness switch - nothing
    // stops working without it - which is why enabling it is the
    // application's call and not a side effect of PinSpeed::very_high.

    /// Power the compensation cell up or down (CMPCR.CMP_PD). Coming back
    /// from a power-down, the cell is usable when ready() says so.
    static void compensation_cell(bool on) {
        clock(true);
        if (on) {
            regs().CMPCR |= SYSCFG_CMPCR_CMP_PD;
        } else {
            regs().CMPCR &= ~SYSCFG_CMPCR_CMP_PD;
        }
    }
    static bool compensation_cell() {
        clock(true);
        return (regs().CMPCR & SYSCFG_CMPCR_CMP_PD) != 0u;
    }

    /// CMPCR.READY: the cell has settled and can be relied on. Read-only,
    /// and meaningless while the cell is powered down.
    static bool compensation_ready() {
        clock(true);
        return (regs().CMPCR & SYSCFG_CMPCR_READY) != 0u;
    }

    // ---- the Ethernet PHY interface (9.2.2) --------------------------------

    /// Whether this device has SYSCFG_PMC's PHY selector at all - the
    /// parts with an Ethernet MAC, and the F405/F415, whose header declares
    /// the selector without the MAC.
    static constexpr bool has_phy_select() { return syscfg_phy_select_mask() != 0u; }

    /// MII (false) or RMII (true) for the Ethernet MAC. 9.2.2: the choice
    /// must be made while the MAC is held in reset and before its clocks
    /// are enabled, which is the Ethernet driver's sequence and not this
    /// file's - here it is the one bit. Answers false on a part without the
    /// selector and changes nothing there.
    static bool phy_rmii(bool on) {
        if constexpr (has_phy_select()) {
            clock(true);
            if (on) {
                regs().PMC |= syscfg_phy_select_mask();
            } else {
                regs().PMC &= ~syscfg_phy_select_mask();
            }
            return true;
        } else {
            (void)on;
            return false;
        }
    }
    static bool phy_rmii() {
        if constexpr (has_phy_select()) {
            clock(true);
            return (regs().PMC & syscfg_phy_select_mask()) != 0u;
        } else {
            return false;
        }
    }
};

} // namespace brio
