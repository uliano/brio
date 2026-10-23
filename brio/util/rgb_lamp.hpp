/*
 * rgb_lamp.hpp
 *
 * RgbLamp<R, G, B>: three PwmChannels (util/pwm_channel.hpp), one
 * colour. A program names its three channels once and then shows a
 * colour with one call; the same lamp type drives a PWM triple (max
 * 255) or an on/off pin triple (max 1: any non-zero level lights the
 * die), because it scales each 8-bit level to its channel's own max.
 * The smallest use (Red, Green and Blue being three PwmChannel types,
 * pwm_channel.hpp shows one; the host suite test/test_rgb_lamp uses
 * fakes):
 *
 *     using Lamp = brio::RgbLamp<Red, Green, Blue>;
 *     Lamp::show(brio::Rgb{255, 90, 60});     // r, g, b levels, 0 = off
 *     Lamp::off();
 *
 * A plain static actuator, not an active object (an object with its own
 * event queue): no state, no events, no queue. show() and off() write
 * three duties in a row and return, waiting for nothing, so they run
 * wherever the channels' duty() may run (pwm_channel.hpp says where
 * that is the implementation's question); the channels' drivers are
 * initialized first, by the program. Apps keep their colour vocabulary
 * above it (an enum and a palette table) and call show(palette[c]).
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

    /// Light the three dies at the colour's levels, each scaled to its
    /// channel's max (rounded up, below): three duty() calls, red first.
    static void show(Rgb c) {
        R::duty(scale<R>(c.r));
        G::duty(scale<G>(c.g));
        B::duty(scale<B>(c.b));
    }

    /// All three channels at duty 0.
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
