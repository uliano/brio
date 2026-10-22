/*
 * irtim.hpp
 *
 * The infrared interface (RM0444 ch. 27), for a program that drives an
 * infrared LED with a modulated carrier - the physical layer of a remote
 * control - with no CPU in the loop. Two pages of manual and one
 * register field, but a real peripheral: it ANDs a carrier with an
 * envelope in hardware and puts the product on one pad. `Irtim` is the
 * block, `IrtimPad<PinSel>` the pad.
 *
 *   TIM17_CH1  ---> carrier   \
 *                              AND --(IR_POL)--> IR_OUT pad
 *   TIM16_CH1 / USART1 / USARTx --> envelope
 *
 * The smallest use, a carrier under a timer's envelope: the two timers
 * brought up and configured first (stm32g0/tim.hpp's TimPwm on Tim<17>
 * channel 1 for the carrier and on Tim<16> channel 1 for the envelope),
 * then Irtim::init(), Irtim::envelope(IrtimEnvelope::tim16),
 * Irtim::polarity(false), Irtim::pb9_high_sink(true) where the pad is
 * PB9, and last IrtimPad<ir_out>::claim() - the pad joins the gate once
 * the gate is configured (docs/stm32g0/irtim.md writes it out).
 *
 * Who calls what: every verb here is a setup verb - main(), or the
 * handler of an active object (a brio object with an event queue the
 * kernel serves one event at a time). Each is a read-modify-write of
 * SYSCFG_CFGR1 with no guard, and that register is shared with the I2C
 * fast-mode-plus drive bits, so two of these writes from two contexts
 * (a handler and the loop) could lose a field: not from an interrupt
 * handler. Nothing waits and nothing masks. The state is the register's
 * own: the block has no registers of its own, everything below is
 * SYSCFG's, which is why `Irtim` is a monostate - a class with static
 * state and no instances, since the part has one block.
 *
 * What the chapter says and this file encodes:
 *  - TIM17 channel 1 is always the carrier; the envelope is chosen by
 *    SYSCFG_CFGR1.IR_MOD - 00 TIM16_CH1, 01 USART1, 10 the family's
 *    second USART, 11 Reserved. Which instance code 10 means is a
 *    per-part fact (USART4 on the G071/G0B1 class, USART2 on the G031
 *    class), so it lives in the reserve (stm32g0/device_tables.hpp, the
 *    table of what each part has, read off the device header) as
 *    irtim_second_usart() and this driver publishes it rather than
 *    restating it;
 *  - SYSCFG_CFGR1.IR_POL inverts the output, which is what an active-low
 *    LED driver wants;
 *  - the high-sink LED driver is available on PB9 alone and is turned on
 *    through SYSCFG_CFGR1.I2C_PB9_FMP - a bit whose name belongs to I2C
 *    and whose second job is this. It is the last paragraph of ch. 27 and
 *    it is the only reason this driver has a verb with "I2C" behind it;
 *  - the function reaches a pad through the GPIO alternate function, and
 *    the pads are the datasheet's: PB9 AF0 and PA13 AF1 (DS13560 tables
 *    15 and 13). PA13 is SWDIO on every Nucleo, so `IrtimPad` will claim
 *    it if asked and this file says, in as many words, never to.
 *
 * Why init() opens RCC_APBENR2.SYSCFGEN and release() does not close it:
 * the gate is the same one stm32g0/comp.hpp and stm32g0/vref.hpp open,
 * and shared (the comparators live inside the SYSCFG block and the
 * EXTI's port multiplexer is there too).
 *
 * What the timers do is the timers' business: the carrier and the
 * envelope are TimPwm's, and neither needs its own pad - figure 278's
 * connections are internal. That is the fact that makes an infrared
 * output cost exactly one pin.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/pin.hpp"

namespace brio {

/// SYSCFG_CFGR1.IR_MOD: which signal is the modulation envelope. Code 11
/// is Reserved and is not spelled; `second_usart` is code 10, whose
/// instance differs per part (irtim_second_usart()).
enum class IrtimEnvelope : uint8_t {
    tim16 = 0,
    usart1 = 1,
    second_usart = 2,
};

constexpr bool irtim_envelope_valid(IrtimEnvelope e) {
    return static_cast<uint8_t>(e) <= 2u;
}

/// The pad an IR_OUT claim needs: the port, the pin and the AF the
/// DATASHEET gives IR_OUT there - PB9 is AF0 and PA13 is AF1, and
/// nothing in the device header can check either.
template <PinSel sel>
struct IrtimPad {
    static_assert(sel.valid(), "brio IrtimPad: not a pin of a port this device has");
    using pin = Pin<sel.port, sel.pin>;

    IrtimPad() = delete;

    /// Hand the pad to the interface. `open_drain` is what an LED pulled
    /// to the supply wants; push-pull is the default. A setup verb, called
    /// once the gate is configured so the pad never carries a half-set
    /// output; release() gives the pad back to analog.
    static void claim(PinSpeed speed = PinSpeed::low, bool open_drain = false) {
        pin::function(sel.function, {.open_drain = open_drain, .speed = speed});
    }
    static void release() { pin::release(); }
};

/**
 * Irtim: the infrared interface, a monostate over three bits of
 * SYSCFG_CFGR1. Setup verbs, one context at a time (the opening
 * comment); init() first, since the block's registers are dead without
 * the SYSCFG clock.
 */
struct Irtim {
    Irtim() = delete;

    /// Which USART instance IR_MOD code 10 selects on this part (ch. 27:
    /// USART4 on the G071/G081/G0B1/G0C1, USART2 on the G031/G041/G051/
    /// G061). Published here so an application names a timer and a USART
    /// and never a device.
    static constexpr uint8_t second_usart_index = irtim_second_usart();

    /// The registers live in SYSCFG, whose bus clock gates them - the
    /// comp.hpp and vref.hpp precedent. Idempotent.
    static void init() { Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN, true); }
    static bool bus_clock() { return Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN); }

    /// SYSCFG_CFGR1.IR_MOD. A Reserved code is refused (false, nothing
    /// written) rather than left to the silicon to interpret.
    static bool envelope(IrtimEnvelope e) {
        if (!irtim_envelope_valid(e)) {
            return false;
        }
        SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_IR_MOD) |
                        (static_cast<uint32_t>(e) << SYSCFG_CFGR1_IR_MOD_Pos);
        return true;
    }
    static IrtimEnvelope envelope() {
        return static_cast<IrtimEnvelope>(
            (SYSCFG->CFGR1 & SYSCFG_CFGR1_IR_MOD) >> SYSCFG_CFGR1_IR_MOD_Pos);
    }

    /// SYSCFG_CFGR1.IR_POL: invert the output. With the carrier and the
    /// envelope both idle the pad rests LOW; inverted it rests HIGH,
    /// which is what an LED wired to the supply wants.
    static void polarity(bool inverted) {
        SYSCFG->CFGR1 = inverted ? (SYSCFG->CFGR1 | SYSCFG_CFGR1_IR_POL)
                                 : (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_IR_POL);
    }
    static bool polarity() { return (SYSCFG->CFGR1 & SYSCFG_CFGR1_IR_POL) != 0u; }

    /**
     * SYSCFG_CFGR1.I2C_PB9_FMP - the high-sink LED driver, available on
     * PB9 and nowhere else (ch. 27's last paragraph). The bit is named
     * for its other job (I2C fast-mode-plus drive on that pad), and that
     * is not a brio spelling but the silicon's: one bit, two uses, and a
     * program using PB9 for I2C and for an infrared LED cannot have it
     * both ways.
     */
    static void pb9_high_sink(bool on) {
        SYSCFG->CFGR1 = on ? (SYSCFG->CFGR1 | SYSCFG_CFGR1_I2C_PB9_FMP)
                           : (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_I2C_PB9_FMP);
    }
    static bool pb9_high_sink() {
        return (SYSCFG->CFGR1 & SYSCFG_CFGR1_I2C_PB9_FMP) != 0u;
    }

    /// Everything this block owns, back to reset. The SYSCFG clock stays
    /// on: it is shared with the comparators, the voltage reference and
    /// the EXTI's port multiplexer.
    static void release() {
        SYSCFG->CFGR1 = SYSCFG->CFGR1 &
                        ~(SYSCFG_CFGR1_IR_MOD | SYSCFG_CFGR1_IR_POL |
                          SYSCFG_CFGR1_I2C_PB9_FMP);
    }
};

} // namespace brio
