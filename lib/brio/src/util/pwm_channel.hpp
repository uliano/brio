/*
 * pwm_channel.hpp
 *
 * PwmChannel: the role-level contract of "one dimmable output". This is
 * what generic actuators (util/rgb_lamp.hpp, a future dimmer or servo)
 * depend on; how the duty is produced - a TCA in split mode, a 16-bit
 * TIM channel, or a plain GPIO pin that only knows on/off - is the
 * target driver's business and never leaks above this concept.
 *
 * Contract:
 *  - `max`: a positive compile-time constant, the value of full duty
 *    (255 for an 8-bit compare, 65535 for a 16-bit one, 1 for a pin);
 *  - `duty(v)`: set the output to v/max, v in [0, max]. Synchronous,
 *    cheap, callable from any handler; no completion, no event.
 * Deliberately NOT here: frequency (a property of the timer instance,
 * shared by its channels on every architecture), polarity (a fact of
 * the pin/target), and any notion of time (fades are AOs above).
 * Values are raw counts, not percentages: a caller that wants "level
 * out of 255" scales with the channel's own max (see RgbLamp).
 */

#pragma once

#include <stdint.h>

namespace brio {

template <typename C>
concept PwmChannel = requires(uint16_t v) {
    requires C::max > 0;
    C::duty(v);
};

} // namespace brio
