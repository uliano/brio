/*
 * irtim.hpp
 *
 * The INFRARED INTERFACE (RM0444 ch. 27) - two pages of manual and one
 * register field, but a real peripheral: it ANDs a carrier with an
 * envelope in hardware and puts the product on one pad, which is the
 * whole of an infrared remote's physical layer with no CPU in it.
 *
 *   TIM17_CH1  ---> carrier   \
 *                              AND --(IR_POL)--> IR_OUT pad
 *   TIM16_CH1 / USART1 / USARTx --> envelope
 *
 * WHAT THE CHAPTER SAYS AND THIS FILE ENCODES:
 *  - TIM17 channel 1 is ALWAYS the carrier; the envelope is chosen by
 *    SYSCFG_CFGR1.IR_MOD - 00 TIM16_CH1, 01 USART1, 10 the family's
 *    SECOND USART, 11 Reserved. Which instance code 10 means is a
 *    PER-PART fact (USART4 on the G071/G0B1 class, USART2 on the G031
 *    class), so it lives in the reserve as irtim_second_usart() and this
 *    driver publishes it rather than restating it;
 *  - SYSCFG_CFGR1.IR_POL inverts the output, which is what an active-low
 *    LED driver wants;
 *  - the high-sink LED driver is available ON PB9 ALONE and is turned on
 *    through SYSCFG_CFGR1.I2C_PB9_FMP - a bit whose NAME belongs to I2C
 *    and whose second job is this. It is the last paragraph of ch. 27 and
 *    it is the only reason this driver has a verb with "I2C" behind it;
 *  - the function reaches a pad through the GPIO alternate function, and
 *    the pads are the DATASHEET's: PB9 AF0 and PA13 AF1 (DS13560 tables
 *    15 and 13). PA13 IS SWDIO on every Nucleo, so `IrtimPad` will claim
 *    it if asked and this file says, in as many words, never to.
 *
 * THE BLOCK HAS NO REGISTERS OF ITS OWN. Everything above is SYSCFG's,
 * which is why `Irtim` is a monostate and why init() opens
 * RCC_APBENR2.SYSCFGEN - the same gate stm32g0/comp.hpp and
 * stm32g0/vref.hpp open, and share (the comparators live inside the
 * SYSCFG block and the EXTI's port multiplexer is there too), so
 * release() does NOT close it.
 *
 * WHAT THE TIMERS DO IS THE TIMERS' BUSINESS: stm32g0/tim.hpp's TimPwm
 * on Tim<17> channel 1 makes the carrier and on Tim<16> channel 1 the
 * envelope, and NEITHER NEEDS ITS OWN PAD - figure 278's connections are
 * internal. That is the fact that makes an infrared output cost exactly
 * one pin.
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
    /// to the supply wants; push-pull is the default.
    static void claim(PinSpeed speed = PinSpeed::low, bool open_drain = false) {
        pin::function(sel.function, {.open_drain = open_drain, .speed = speed});
    }
    static void release() { pin::release(); }
};

/**
 * Irtim: the infrared interface, a monostate over three bits of
 * SYSCFG_CFGR1.
 */
struct Irtim {
    Irtim() = delete;

    /// Which USART instance IR_MOD code 10 selects on THIS part (ch. 27:
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
     * SYSCFG_CFGR1.I2C_PB9_FMP - the HIGH SINK LED DRIVER, available on
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
