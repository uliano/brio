/*
 * vref.hpp
 *
 * The STM32G0's ANALOG REFERENCE: the VREF+ rail every converter of this
 * family measures against, and the on-chip buffer that can drive it
 * (VREFBUF, RM0444 ch. 17).
 *
 * WHY THIS FILE EXISTS AT ALL, and why `brio::Ref` lives HERE rather than
 * in stm32g0/adc.hpp: on this family there IS one shared reference block.
 * The ADC (15.3.1), the DAC (16.4.6) and the comparators' VREFINT scaler
 * all work against the SAME VREF+ pin, and the buffer of chapter 17 is
 * the one thing that can change what that pin is worth. On the SAM C21
 * each converter carried its own REFSEL vocabulary and each header
 * therefore carried its own reference enum (samc/adc.hpp's comment says
 * so); here one enum serves all three, so it sits in the chapter that
 * owns the rail. util/analog.hpp's contract is unchanged: `Ref` and
 * `ref_mv()` are each target's, under the same name.
 *
 * THE RULE THAT SHAPES THE DRIVER. 17.1: "When the VREF+ pin is
 * double-bonded with VDDA pin in a package, the voltage reference buffer
 * is not available and must be kept disabled." Even where the pin IS
 * bonded on its own - and it is on the LQFP64 this stratum's bench chip
 * wears (DS13560 table 12: VREF+ is pin 7) - a BOARD may still tie it to
 * VDDA, and then enabling the buffer means an internal 2.048 V source
 * driving into a 3.3 V regulator. NOTHING INSIDE THE CHIP CAN TELL THE
 * TWO APART: a VREF+ at VDDA reads the same either way. So `enable()`
 * REFUSES unless the caller states, in the config and by name, that the
 * board leaves the pin free - the one thing only the schematic knows.
 * The reset state (ENVR = 0, HIZ = 1, "external voltage reference mode",
 * table 91) is the safe one and this file never leaves it on its own.
 *
 * The one mode this driver does NOT offer is table 91's ENVR = 0 +
 * HIZ = 0, "VREF+ pin pulled-down to VSSA": on a board that ties VREF+ to
 * VDDA that is a short across the supply, and it buys nothing a disabled
 * buffer does not already give. `disable()` therefore always leaves
 * HIZ set. Hold mode (ENVR = 1, HIZ = 1) is reachable, because it drives
 * nothing.
 *
 * THE BLOCK IS BEHIND SYSCFG'S CLOCK GATE, and the bench is how that was
 * learned. Chapter 17 names no clock at all, and the device header's own
 * address map is what says it: VREFBUF_BASE is SYSCFG_BASE + 0x30, the
 * same block COMP1..COMP3 live in - so RCC_APBENR2.SYSCFGEN gates it,
 * and 5.2.17's rule applies. WITH THE GATE CLOSED VREFBUF_CSR READS
 * ZERO, and zero is not the reset value: it is table 91's OTHER off mode,
 * the one that pulls VREF+ down to VSSA. An application that read this
 * register before opening the gate would conclude the pin was being
 * grounded. `init()` opens the gate and every verb below assumes it has
 * been called.
 *
 * ERRATA: ES0548 Rev 3 has NO item touching VREFBUF on either silicon
 * revision.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"

namespace brio {

// =============================================================================
// The reference vocabulary (util/analog.hpp's `Ref` for this target)
// =============================================================================

/// What VREF+ is worth, as far as software can know it.
///
/// `external` is the reset arrangement and the one every board that ties
/// VREF+ to VDDA is stuck with: the chip does not know the number, so
/// `ref_mv()` wants it - and stm32g0/adc.hpp's `vdda_mv()` is how it is
/// MEASURED rather than assumed, through VREFINT and its factory value.
enum class Ref : uint8_t {
    external = 0,      ///< VREF+ as the board supplies it (VDDA, or a source)
    buffer_2v048 = 1,  ///< VREFBUF driving VREF+, VRS = 0 (17.2)
    buffer_2v5 = 2,    ///< VREFBUF driving VREF+, VRS = 1
};

constexpr bool ref_valid(Ref r) {
    return r == Ref::external || r == Ref::buffer_2v048 || r == Ref::buffer_2v5;
}

/// The reference in millivolts. For the two buffer levels these are
/// 17.2's nominal values (DS13560 table 67 gives their tolerance); for
/// `external` the answer is `known_mv`, which a caller gets from
/// `Adc::vdda_mv()` or from its own board knowledge. A zero answer means
/// "not known", and every arithmetic verb above treats it as such rather
/// than dividing by it.
constexpr uint16_t ref_mv(Ref r, uint16_t known_mv = 0) {
    switch (r) {
        case Ref::buffer_2v048: return 2048;
        case Ref::buffer_2v5: return 2500;
        case Ref::external:
        default: return known_mv;
    }
}

// =============================================================================
// VREFBUF (RM0444 ch. 17)
// =============================================================================

/// VREFBUF_CSR.VRS - which of the two levels the buffer produces.
enum class VrefScale : uint8_t { v2_048 = 0, v2_5 = 1 };

/// What `Vref::enable()` is asked for.
///
/// `board_vref_pin_is_free` is not a convenience flag: it is the caller
/// asserting the one fact 17.1 needs and no register carries - that
/// nothing on the board drives VREF+. Leaving it false is what makes
/// every enable a refusal, which is the safe default for a driver that
/// cannot see the schematic.
struct VrefBufConfig {
    VrefScale scale = VrefScale::v2_048;
    bool board_vref_pin_is_free = false;
};

/// The block. One instance on every part of the family, so a monostate
/// (the samc Dac/Sdadc/Tsens precedent).
struct Vref {
    static_assert(vrefbuf_present(),
                  "brio Vref: this device declares no VREFBUF_BASE");

    Vref() = delete;

    static VREFBUF_TypeDef& regs() {
        return *reinterpret_cast<VREFBUF_TypeDef*>(vrefbuf_base());
    }

    /// Open the SYSCFG clock gate the block's registers live behind (see
    /// this file's header). Idempotent, and nothing here ever closes it -
    /// SYSCFG is shared with the comparators and the EXTI multiplexer.
    static void init() { Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN, true); }

    static bool bus_clock() { return Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN); }

    // ---- state (17.3.1's table 91, read back rather than remembered) --------

    /// ENVR: the buffer's mode enable.
    static bool enabled() { return (regs().CSR & VREFBUF_CSR_ENVR) != 0u; }

    /// HIZ: 1 = VREF+ is an input pin, 0 = it is connected to the buffer.
    /// This reads 1 out of reset (reset value 0x02), which is table 91's
    /// "external voltage reference mode".
    static bool high_impedance() { return (regs().CSR & VREFBUF_CSR_HIZ) != 0u; }

    /// VRR: the output has reached its level. It means nothing in hold
    /// mode - 17.2 says the detection is disabled there and the bit keeps
    /// its last state - so `ready()` is only evidence when the buffer is
    /// actually driving.
    static bool ready() { return (regs().CSR & VREFBUF_CSR_VRR) != 0u; }

    static VrefScale scale() {
        return (regs().CSR & VREFBUF_CSR_VRS) != 0u ? VrefScale::v2_5 : VrefScale::v2_048;
    }

    /// The `Ref` this block is currently producing, or `Ref::external`
    /// whenever the buffer is not driving the pin.
    static Ref reference() {
        if (!enabled() || high_impedance()) {
            return Ref::external;
        }
        return scale() == VrefScale::v2_5 ? Ref::buffer_2v5 : Ref::buffer_2v048;
    }

    /// VREFBUF_CCR.TRIM, loaded from the production value at reset
    /// (17.3.2). Read-only here: 17.3.2's own note makes a user trim an
    /// ascending sweep from zero, which is a calibration procedure and
    /// not a setter, and nothing in this stratum has a reason to want it.
    static uint8_t trim() {
        return static_cast<uint8_t>(regs().CCR & VREFBUF_CCR_TRIM);
    }

    // ---- the two verbs -------------------------------------------------------

    /**
     * Drive VREF+ from the internal buffer at `cfg.scale` (table 91's
     * "internal voltage reference mode": ENVR = 1, HIZ = 0), then wait
     * for VRR.
     *
     * REFUSES, writing nothing, when `cfg.board_vref_pin_is_free` is
     * false - see this file's header. Also returns false if VRR never
     * rises within `spins`, leaving the block in the state it reached, so
     * a caller that gets false can call `disable()` and go on.
     */
    static bool enable(const VrefBufConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!cfg.board_vref_pin_is_free) {
            return false;
        }
        uint32_t csr = regs().CSR & ~(VREFBUF_CSR_VRS | VREFBUF_CSR_HIZ);
        if (cfg.scale == VrefScale::v2_5) {
            csr |= VREFBUF_CSR_VRS;
        }
        regs().CSR = csr | VREFBUF_CSR_ENVR;
        for (uint32_t i = 0; i < spins; ++i) {
            if (ready()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Hold mode (17.2's ENVR = 1 + HIZ = 1): the reference runs, the
     * output buffer does not, and VREF+ keeps whatever an external
     * capacitor holds. It drives NOTHING, so unlike `enable()` it needs
     * no acknowledgement - and unlike `enable()` it makes `ready()`
     * meaningless.
     */
    static void hold() {
        regs().CSR = (regs().CSR | VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ);
    }

    /// Back to the reset arrangement: buffer off, VREF+ an input pin.
    /// HIZ is left SET on purpose - table 91's other off mode pulls the
    /// pin down to VSSA, which on a board that ties VREF+ to VDDA is a
    /// short (this file's header).
    static void disable() {
        regs().CSR = (regs().CSR & ~VREFBUF_CSR_ENVR) | VREFBUF_CSR_HIZ;
    }

    static void release() { disable(); }
};

} // namespace brio
