/*
 * rgb_lamp.hpp
 *
 * RgbLamp<R, G, B>: three PwmChannels, one colour. A plain static
 * actuator (not an AO): no state, no events, it turns an 8-bit RGB
 * level triple into three duties, scaling each level to its channel's
 * own max - so the same lamp type drives a PWM triple (max 255) or an
 * on/off pin triple (max 1: any non-zero level lights the die). Apps
 * keep their colour vocabulary above it (an enum and a palette table)
 * and call show(palette[c]).
 */

#pragma once

#include <stdint.h>

#include "util/pwm_channel.hpp"

namespace brio {

/// 8-bit level per die: 0 = off, 255 = full.
struct Rgb {
    uint8_t r, g, b;
};

template <PwmChannel R, PwmChannel G, PwmChannel B>
struct RgbLamp {
    RgbLamp() = delete;

    static void show(Rgb c) {
        R::duty(scale<R>(c.r));
        G::duty(scale<G>(c.g));
        B::duty(scale<B>(c.b));
    }

    static void off() { show(Rgb{0, 0, 0}); }

private:
    /// level/255 of the channel's max, rounded UP: a non-zero level is
    /// never lost (matters for max = 1), 255 always maps to max.
    template <PwmChannel C>
    static constexpr uint16_t scale(uint8_t level) {
        if constexpr (C::max == 255) {
            return level;
        } else if constexpr (C::max == 1) {
            return level != 0 ? 1 : 0;           // on/off channel: no arithmetic
        } else {
            return static_cast<uint16_t>(
                (static_cast<uint32_t>(level) * C::max + 254u) / 255u);
        }
    }
};

} // namespace brio
