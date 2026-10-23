/*
 * pwm_channel.hpp
 *
 * PwmChannel: the role-level contract of "one dimmable output", a
 * concept (a compiler-checked set of requirements on a type) that a
 * target's timer channel or pin meets. It is what generic actuators
 * (util/rgb_lamp.hpp, a future dimmer or servo) depend on, so one
 * actuator type drives a timer's compare channel and a bare pin alike;
 * how the duty is produced - a split-mode 8-bit timer, a 16-bit compare
 * channel, or a plain GPIO pin that only knows on/off - is the target
 * driver's business and never leaks above this concept. The smallest
 * use is a type that names a channel and the call that sets it, as the
 * AVR's drivers spell them (avrxt/tca.hpp, avrxt/pin.hpp):
 *
 *     using PwmB = brio::TcaPwm<1, 'B'>;      // TCA1 on PB0..PB5, split mode
 *     using Red = PwmB::Channel<0>;           // a PwmChannel, max 255
 *     PwmB::init();                           // the driver's own setup, once
 *     Red::duty(128);                         // about half duty
 *
 * and a pin is the degenerate channel (max 1), checked in avrxt/pin.hpp
 * by `static_assert(PwmChannel<Pin<'A', 0>>)`. The concept has no setup
 * verb: the driver behind it is configured first, its own way.
 *
 * Contract:
 *  - `max`: a positive compile-time constant, the value of full duty
 *    (255 for an 8-bit compare, 65535 for a 16-bit one, 1 for a pin);
 *  - `duty(v)`: set the output to v/max, v in [0, max]. Synchronous and
 *    cheap: the value is handed to the hardware before it returns (a
 *    buffered compare takes it at the next period), it waits for
 *    nothing, posts no completion and no event, so it may be called
 *    from any event handler. Whether it may also run in an interrupt
 *    handler beside main-context calls is the implementation's to say:
 *    the AVR's TCA split mode, for one, read-modify-writes a control
 *    register its six channels share (avrxt/tca.hpp).
 * Deliberately not here: frequency (a property of the timer instance,
 * shared by its channels on every architecture), polarity (a fact of
 * the pin/target), and any notion of time (fades are active objects
 * above). Values are raw counts, not percentages: a caller that wants
 * "level out of 255" scales with the channel's own max (see RgbLamp).
 * The concept checks that `max` is positive and that `duty(v)` takes a
 * uint16_t; keeping v within [0, max] is the caller's duty (some
 * implementations clamp, rp2040/pwm.hpp's for one; the contract does
 * not promise it).
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
