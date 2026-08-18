/*
 * pwm.hpp
 *
 * TcaPwm<n, port>: a TCA timer in SPLIT mode used as six independent
 * 8-bit PWM channels, WO0..WO5 on pins 0..5 of one port. Static
 * (monostate) like every brio driver; the channel is a compile-time
 * argument, so duty<ch>(v) compiles to one 8-bit store.
 *
 * Why split mode: one 16-bit TCA becomes two 8-bit counters (low: WO0-2,
 * high: WO3-5), each in single-slope PWM with PER = 255 - exactly the
 * "six dimmable channels" shape an RGB pair or a six-LED bar wants, at
 * the price of 8-bit resolution, which is all a LED needs. Both halves
 * run from the same prescaled clock, so all six channels share the PWM
 * frequency: F_CPU / prescaler / 256 (24 MHz, div16 -> ~5.9 kHz: no
 * visible flicker, no audible whine).
 *
 * Endpoints: the compare hardware cannot express a clean 0 % or 100 %
 * (in split mode CMP = 0 still produces one clock of output and CMP =
 * PER leaves one clock off), so duty 0 and 255 disable the channel's
 * waveform output and drive the pin low/high from PORT.OUT instead -
 * same policy as the DxCore analogWrite. Everything in between writes
 * CMP = value: brightness = value / 256, output active-high (common-
 * cathode LED, resistor, pin -> LED anode). For active-low loads use
 * Pin<>::invert(true) on the pin: INVEN inverts the waveform too.
 *
 * Routing: PORTMUX.TCAROUTEA. Which ports a TCA can reach is a fact of
 * the family and the package - the table below is checked at compile
 * time and an impossible route is a clear error (C++23 static_assert
 * (false) in the instantiated branch), same style as pin.hpp/uart.hpp.
 * TCA1 exists only on 48/64-pin DA/DB parts (#ifdef TCA1).
 *
 *   using Bar = brio::TcaPwm<0, 'C'>;   // TCA0 -> PC0..PC5
 *   Bar::init();                       // split mode, div16, all six on
 *   Bar::duty<2>(64);                  // PC2 at 25 %
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

namespace brio {

/// TCA clock prescaler (split-mode CLKSEL values).
enum class TcaClock : uint8_t {
    div1 = TCA_SPLIT_CLKSEL_DIV1_gc,
    div2 = TCA_SPLIT_CLKSEL_DIV2_gc,
    div4 = TCA_SPLIT_CLKSEL_DIV4_gc,
    div8 = TCA_SPLIT_CLKSEL_DIV8_gc,
    div16 = TCA_SPLIT_CLKSEL_DIV16_gc,
    div64 = TCA_SPLIT_CLKSEL_DIV64_gc,
    div256 = TCA_SPLIT_CLKSEL_DIV256_gc,
    div1024 = TCA_SPLIT_CLKSEL_DIV1024_gc,
};

template <uint8_t tca_num, char PortLetter>
class TcaPwm {
    static_assert(tca_num <= 1, "AVR Dx has TCA0 and (48/64-pin parts) TCA1");

public:
    TcaPwm() = delete;

    static constexpr uint8_t channels = 6;

    /// Route the timer to the port, enter split mode with PER = 255 on
    /// both halves, all six compare outputs enabled at duty 0, pins as
    /// outputs driven low, counter running from the chosen prescaler.
    static void init(TcaClock clock = TcaClock::div16) {
        auto& t = regs();
        t.CTRLA = 0;                                   // stop while configuring
        t.CTRLESET = TCA_SPLIT_CMD_RESET_gc;           // known state (also clears CTRLD)
        PORTMUX.TCAROUTEA = static_cast<uint8_t>(
            (PORTMUX.TCAROUTEA & ~route_mask()) | route_bits());
        t.CTRLD = TCA_SPLIT_SPLITM_bm;
        t.LPER = 255;
        t.HPER = 255;
        t.LCMP0 = 0; t.LCMP1 = 0; t.LCMP2 = 0;
        t.HCMP0 = 0; t.HCMP1 = 0; t.HCMP2 = 0;
        port().OUTCLR = pin_mask;                      // duty 0 = pin low
        port().DIRSET = pin_mask;                      // WO drives only output pins
        t.CTRLB = 0;                                   // endpoints policy: all off = PORT.OUT
        t.CTRLA = static_cast<uint8_t>(
            static_cast<uint8_t>(clock) | TCA_SPLIT_ENABLE_bm);
    }

    /// Set channel ch (WO0..WO5 = pin 0..5) to value/256 duty.
    /// 0 and 255 leave the waveform and drive the pin from PORT.OUT.
    template <uint8_t ch>
    static void duty(uint8_t value) {
        static_assert(ch < channels, "TCA split mode has six channels, WO0..WO5");
        auto& t = regs();
        if (value == 0) {
            t.CTRLB &= static_cast<uint8_t>(~cmp_enable_bit<ch>());
            port().OUTCLR = static_cast<uint8_t>(1u << ch);
        } else if (value == 255) {
            t.CTRLB &= static_cast<uint8_t>(~cmp_enable_bit<ch>());
            port().OUTSET = static_cast<uint8_t>(1u << ch);
        } else {
            cmp<ch>() = value;
            t.CTRLB |= cmp_enable_bit<ch>();
        }
    }

private:
    static constexpr uint8_t pin_mask = 0x3F;   // pins 0..5

    static constexpr TCA_SPLIT_t& regs() {
        if constexpr (tca_num == 0) {
            return TCA0.SPLIT;
        } else {
#ifdef TCA1
            return TCA1.SPLIT;
#else
            static_assert(false, "TCA1 is not present on this device (28/32-pin parts)");
#endif
        }
    }

    static constexpr volatile PORT_t& port() {
        if constexpr (PortLetter == 'A') return PORTA;
#ifdef PORTB
        else if constexpr (PortLetter == 'B') return PORTB;
#endif
        else if constexpr (PortLetter == 'C') return PORTC;
        else if constexpr (PortLetter == 'D') return PORTD;
#ifdef PORTE
        else if constexpr (PortLetter == 'E') return PORTE;
#endif
        else if constexpr (PortLetter == 'F') return PORTF;
        else static_assert(false, "TCA cannot be routed to this port on this device");
    }

    static constexpr uint8_t route_mask() {
        return tca_num == 0 ? PORTMUX_TCA0_gm : PORTMUX_TCA1_gm;
    }

    // The routing table: TCA0 reaches PORTA..PORTF (PORTE only WO0-3 on
    // 48-pin parts, PORTG on 64-pin only); TCA1 reaches PORTB (WO0-5)
    // and, with WO0-2 only, PORTC/PORTE/PORTG - the partial routes are
    // deliberately not offered here (six channels or nothing).
    static constexpr uint8_t route_bits() {
        if constexpr (tca_num == 0) {
            if constexpr (PortLetter == 'A') return PORTMUX_TCA0_PORTA_gc;
            else if constexpr (PortLetter == 'B') return PORTMUX_TCA0_PORTB_gc;
            else if constexpr (PortLetter == 'C') return PORTMUX_TCA0_PORTC_gc;
            else if constexpr (PortLetter == 'D') return PORTMUX_TCA0_PORTD_gc;
            else if constexpr (PortLetter == 'F') return PORTMUX_TCA0_PORTF_gc;
            else static_assert(false, "TCA0 six-channel routes: PORTA/B/C/D/F");
        } else {
#ifdef TCA1
            if constexpr (PortLetter == 'B') return PORTMUX_TCA1_PORTB_gc;
            else static_assert(false, "TCA1 six-channel route: PORTB only");
#else
            return 0;
#endif
        }
    }

    // Split-mode compare registers interleave: LCMP0, HCMP0, LCMP1, HCMP1,
    // LCMP2, HCMP2. WO0-2 -> LCMPn, WO3-5 -> HCMPn.
    template <uint8_t ch>
    static volatile uint8_t& cmp() {
        if constexpr (ch < 3) {
            return (&regs().LCMP0)[2 * ch];
        } else {
            return (&regs().HCMP0)[2 * (ch - 3)];
        }
    }

    // CTRLB: LCMP0EN..LCMP2EN = bits 0-2, HCMP0EN..HCMP2EN = bits 4-6.
    template <uint8_t ch>
    static constexpr uint8_t cmp_enable_bit() {
        return ch < 3 ? static_cast<uint8_t>(1u << ch)
                      : static_cast<uint8_t>(1u << (ch + 1));
    }
};

} // namespace brio
