/*
 * divas.hpp
 *
 * The SAM C21 Divide and Square Root Accelerator (DS60001479M ch. 14):
 * a 32-bit signed/unsigned integer divider and a 32-bit unsigned square
 * root engine, sitting on the bus matrix because the Cortex-M0+ core has
 * neither.
 *
 * WHY IT IS INTERESTING AND WHY IT IS ONLY A DRIVER. Every `a / b` in
 * C++ on this core is a call to `__aeabi_uidiv` or `__aeabi_idiv` - a
 * software routine in libgcc - and this block does the same work in 2 to
 * 16 cycles with no code. The tempting next step is to make the
 * TOOLCHAIN use it (gcc lets `__aeabi_uidiv` be overridden), and THAT
 * STEP IS DELIBERATELY NOT TAKEN HERE. It is a whole-image decision with
 * consequences this header cannot own: the routines would have to be
 * re-entrant against interrupts (the block has ONE set of operand
 * registers), the block would have to be clocked and idle at every point
 * a division can happen including before main(), and the win has to be
 * real. So this driver EXPOSES and MEASURES; docs/samc/divas.md carries
 * the measured cycle counts as the input to that decision and names it
 * as open.
 *
 * WHAT THE SILICON DOES.
 *
 * THE OPERATION STARTS ON THE OPERAND WRITE. Writing DIVIDEND does
 * nothing; writing DIVISOR starts a division, writing SQRNUM starts a
 * square root (14.6.2.2). There is no start bit and no command register
 * - which is also why the order of the two operand writes is not a
 * convention but a requirement.
 *
 * TWO BUSES, AND THE DIFFERENCE MATTERS. The block answers at
 * 0x48000000 on the high-speed bus AND at 0x60000200 on the CPU's local
 * single-cycle IOBUS. On the AHB path a read of RESULT while the engine
 * is busy INSERTS WAIT STATES and returns the right answer, so no
 * polling is needed; the IOBUS cannot wait, so a caller there MUST poll
 * STATUS.BUSY first or read a stale result (14.5.10). Both are built,
 * `DivasBus` selects, and the AHB is the default because it is the one
 * that cannot be got wrong.
 *
 * THE IOBUS ADDRESS COMES FROM THE DATA SHEET, NOT THE HEADER. The
 * device header defines DIVAS at 0x48000000 and says nothing about the
 * IOBUS alias; the memory map (9.2, table 9-1) and the product mapping
 * (figure 8-3) both put the IOBUS region at 0x60000000 with PORT at its
 * base and DIVAS at +0x200. This is the CCL's `LutInput::tcc` situation
 * again and it takes the same ruling: THE HEADER WINS WHERE BOTH
 * DOCUMENTS SPEAK, and where the header is SILENT the datasheet is
 * spelled out and the bench arbitrates. It does - test_samc_debug letter
 * h computes the same quotients through both paths.
 *
 * WRITING AN OPERAND WHILE BUSY IS AN ERROR. 14.5.8 write-protects
 * CTRLA, DIVIDEND, DIVISOR and SQRNUM for the duration of an operation
 * and says an access "will result in an error" without saying where the
 * error appears; the PAC's INTFLAGAHB has a DIVAS bit (11.7.5) and that
 * is the only candidate. The verbs here DO NOT poll before writing,
 * because the chapter's own flow does not and because a poll would cost
 * more than the division: a CPU cannot issue two stores faster than the
 * engine finishes. `wait_idle()` exists for a caller who wants the
 * guarantee, and the bench measures whether a back-to-back sequence
 * raises the flag.
 *
 * DIVIDE BY ZERO IS NOT A FAULT. 14.6.2.5: the quotient comes back zero,
 * the remainder comes back equal to the dividend, and STATUS.DBZ is set
 * and stays set until written back. Nothing traps. `divide()` therefore
 * returns a result like any other and the caller reads `divide_by_zero()`
 * - the same shape the silicon has, rather than an optional that would
 * hide the remainder the chapter promises.
 *
 * THE SIGNED OVERFLOW HAS NO INDICATION EITHER. 14.6.2.4: the most
 * negative number divided by minus one overflows the signed range and
 * "will return the maximum negative number with no indication of the
 * overflow". Stated here because it cannot be enforced.
 *
 * LEADING-ZERO OPTIMIZATION IS ON BY DEFAULT AND COSTS DETERMINISM.
 * CTRLA.DLZ = 0 lets the engine skip leading zeros of the dividend, so a
 * division takes 2 to 16 cycles depending on the operands; DLZ = 1
 * forces every 32-bit division to 16. Which one a program wants is a
 * real choice and both are exposed.
 *
 * NO INTERRUPT, NO EVENTS, NO DMA, AND NO SLEEP. 14.5.4/5/6 are all
 * "Not applicable", and 14.5.2 is blunter than any other chapter in this
 * family: "The DIVAS will not operate in any sleep mode."
 *
 * NO ERRATA. DS80000740S has no DIVAS section.
 *
 * NOT BUILT (docs/samc/divas.md carries the list): adopting the block as
 * the toolchain's division, for the reasons above; and 16-bit operation,
 * which the chapter mentions in its cycle counts (14.6.2.6) without ever
 * giving it a register - the operands are one 32-bit width and the
 * "16-bit division" of that sentence is just a 32-bit division whose
 * dividend has sixteen leading zeros.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"

namespace brio {

/// Which of the block's two addresses a verb goes through.
enum class DivasBus : uint8_t {
    /// 0x48000000, the high-speed bus: a RESULT read while busy inserts
    /// wait states and returns the finished answer.
    ahb,
    /// 0x60000200, the CPU's single-cycle local bus: no wait states, so
    /// STATUS.BUSY must be polled before the result is read.
    iobus,
};

/// A division's or a square root's two answers.
struct DivasResult {
    uint32_t result;
    uint32_t remainder;
};

/// The same in two's complement, for `divide_signed()`.
struct DivasSignedResult {
    int32_t result;
    int32_t remainder;
};

/**
 * The divider as a monostate resource.
 *
 * There is no init() beyond the bus clock and no enable bit: the block
 * is live whenever it is clocked, and its clock is ON out of reset
 * (table 12-3, AHB index 12).
 */
struct Divas {
    Divas() = delete;

    /// The high-speed bus address, from the device header.
    static constexpr uint32_t ahb_base = DIVAS_BASE_ADDRESS;

    /**
     * The IOBUS alias, SPELLED FROM THE DATA SHEET because no device
     * header in this pack defines it: table 9-1 puts the IOBUS region
     * at 0x60000000 and figure 8-3 places PORT at its base with DIVAS
     * at offset 0x200 (and nothing else in the region - 0x60000220
     * onwards is Reserved).
     */
    static constexpr uint32_t iobus_base = 0x60000200UL;

    /// DIVAS is an AHB CLIENT, not a peripheral on an APB bridge: it has
    /// no `ID_` macro, no PAC write protection and no entry in any
    /// STATUSn register. What it does have is a bit of its own in
    /// PAC.INTFLAGAHB - `PacAhbFlag::divas` - which is where 14.5.8's
    /// unnamed "error" has to appear if it appears anywhere.

    static divas_registers_t& regs() { return *DIVAS_REGS; }
    static divas_registers_t& io_regs() {
        return *reinterpret_cast<divas_registers_t*>(iobus_base);
    }

    template <DivasBus bus>
    static divas_registers_t& at() {
        if constexpr (bus == DivasBus::iobus) {
            return io_regs();
        } else {
            return regs();
        }
    }

    // ---- clocks --------------------------------------------------------------

    /// The AHB clock, on out of reset. There is no APB clock: the block
    /// is not on a bridge.
    static void bus_clock(bool on) { Mclk::ahb(MCLK_AHBMASK_DIVAS_Msk, on); }
    static bool bus_clock() {
        return (MCLK_REGS->MCLK_AHBMASK & MCLK_AHBMASK_DIVAS_Msk) != 0u;
    }

    // ---- configuration -------------------------------------------------------

    /**
     * CTRLA: signedness and the leading-zero optimization.
     *
     * ENABLE-PROTECTED IN THE ONLY SENSE THIS BLOCK HAS ONE: 14.6.2.1
     * says both bits "must be written prior to starting a division", and
     * 14.5.8 makes a write while busy an error. Callers set this once
     * and then divide.
     */
    static void configure(bool signed_division, bool disable_leading_zero = false) {
        DIVAS_REGS->DIVAS_CTRLA =
            static_cast<uint8_t>((signed_division ? DIVAS_CTRLA_SIGNED_Msk : 0u) |
                                 (disable_leading_zero ? DIVAS_CTRLA_DLZ_Msk : 0u));
    }
    static uint8_t ctrla() { return DIVAS_REGS->DIVAS_CTRLA; }
    static bool signed_division() {
        return (ctrla() & DIVAS_CTRLA_SIGNED_Msk) != 0u;
    }
    static bool leading_zero_disabled() {
        return (ctrla() & DIVAS_CTRLA_DLZ_Msk) != 0u;
    }

    // ---- status --------------------------------------------------------------

    static uint8_t status() { return DIVAS_REGS->DIVAS_STATUS; }
    static bool busy() { return (status() & DIVAS_STATUS_BUSY_Msk) != 0u; }

    /// STATUS.DBZ, which LATCHES: a single divide-by-zero anywhere in a
    /// program leaves this standing until it is written back, so a
    /// caller that cares must clear it before the division it is asking
    /// about.
    static bool divide_by_zero() { return (status() & DIVAS_STATUS_DBZ_Msk) != 0u; }
    static void clear_divide_by_zero() {
        DIVAS_REGS->DIVAS_STATUS = DIVAS_STATUS_DBZ_Msk;
    }

    /// Spin until the engine is idle. Not called by the operation verbs
    /// - see this file's header - and here for a caller who wants the
    /// guarantee before writing an operand from an interrupt.
    static bool wait_idle(uint32_t spins = 0xFFFFu) {
        while (busy() && spins-- != 0u) {
        }
        return !busy();
    }

    // ---- the operations ------------------------------------------------------

    /**
     * Unsigned division. The DIVISOR write starts it; the RESULT read
     * on the AHB path stalls until it is finished, and on the IOBUS path
     * STATUS.BUSY is polled first because that bus cannot stall.
     *
     * CTRLA.SIGNED is NOT written here: configuring is a separate verb
     * because writing it costs a store the caller often does not need,
     * and because a write while busy is an error. `divide_unsigned()`
     * on a block left in signed mode divides signed - the register is
     * the truth and this header does not paper over it.
     */
    template <DivasBus bus = DivasBus::ahb>
    static DivasResult divide_unsigned(uint32_t dividend, uint32_t divisor) {
        auto& r = at<bus>();
        r.DIVAS_DIVIDEND = dividend;
        r.DIVAS_DIVISOR = divisor;
        if constexpr (bus == DivasBus::iobus) {
            // THE POLL GOES THROUGH THE SAME BUS. Reaching for the AHB
            // status register here would put an AHB access - the very
            // thing the IOBUS path exists to avoid - in the middle of
            // every IOBUS division, and the bench measured it costing
            // more than the wait states it was meant to save.
            while ((r.DIVAS_STATUS & DIVAS_STATUS_BUSY_Msk) != 0u) {
            }
        }
        return DivasResult{r.DIVAS_RESULT, r.DIVAS_REM};
    }

    /// Signed division in two's complement (14.6.2.4: the remainder
    /// takes the dividend's sign, the quotient is negative when the
    /// operands' signs differ).
    template <DivasBus bus = DivasBus::ahb>
    static DivasSignedResult divide_signed(int32_t dividend, int32_t divisor) {
        const DivasResult raw = divide_unsigned<bus>(
            static_cast<uint32_t>(dividend), static_cast<uint32_t>(divisor));
        return DivasSignedResult{static_cast<int32_t>(raw.result),
                                 static_cast<int32_t>(raw.remainder)};
    }

    /**
     * Unsigned square root: RESULT is the integer square root and
     * REMAINDER is SQRNUM - RESULT^2 (14.6.2.3).
     *
     * The square root is unsigned WHATEVER CTRLA.SIGNED says - 14.6.2.7
     * gives it no signed mode - so this verb neither reads nor writes
     * that bit.
     */
    template <DivasBus bus = DivasBus::ahb>
    static DivasResult square_root(uint32_t value) {
        auto& r = at<bus>();
        r.DIVAS_SQRNUM = value;
        if constexpr (bus == DivasBus::iobus) {
            while ((r.DIVAS_STATUS & DIVAS_STATUS_BUSY_Msk) != 0u) {
            }
        }
        return DivasResult{r.DIVAS_RESULT, r.DIVAS_REM};
    }
};

} // namespace brio
