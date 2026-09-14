/*
 * sim_input.hpp (host)
 *
 * The world's side of a button and a knob, for a program running on the
 * host. Each is an ordinary ScannedInput - a level somebody else
 * decides - so nothing above learns that the somebody is a test script
 * or a viewer's mouse instead of a finger.
 *
 * THE SEAM IS set(), AND THAT IS THE WHOLE INTERFACE. A test calls it
 * directly; an interactive simulator calls it from the kernel's idle
 * path when a snapshot arrives from the viewer. One policy, two
 * sources, and the automated half needs no socket, no viewer and no
 * second process - the same split the pixel store has.
 *
 * A KNOB IS DRIVEN BY ITS SHAFT, not by its contacts: step() turns it
 * one quadrature count and the two pads follow the Gray sequence a real
 * one walks. force() is the other door, for putting the contacts in a
 * state a turn would never produce - a bounce between two of them, or a
 * jump that skips one - because INJECTING THE DIRT IS THE POINT. A real
 * contact cannot be asked to bounce between two named states on demand;
 * here it is three lines, which is why the host is the better place to
 * test this and not merely the cheaper one.
 */

#pragma once

#include <stdint.h>

#include "util/input_scanner.hpp"

namespace brio {

/**
 * A contact a test or a viewer holds down. `id` only tells two of them
 * apart - each is its own type, with its own state.
 *
 * True means ACTIVE, as the ScannedInput contract asks; whether the real
 * pin would be high or low is the board file's business, and there is
 * nothing here to invert.
 */
template <uint8_t id>
struct SimButton {
    static bool read() { return active_; }

    /// The world's side: press or release.
    static void set(bool active) { active_ = active; }

    static void reset() { active_ = false; }

private:
    static inline bool active_ = false;
};

/**
 * The two contacts of an incremental encoder, driven by the shaft.
 *
 * The pads are `A` and `B`, each a ScannedInput of its own, so they pass
 * to a decoder exactly as two pins would.
 */
template <uint8_t id>
class SimEncoder {
public:
    /// The pads, as a decoder takes them.
    struct A {
        static bool read() { return (state_ & 2u) != 0; }
    };
    struct B {
        static bool read() { return (state_ & 1u) != 0; }
    };

    /// Turn the shaft by one quadrature count. The contacts follow the
    /// Gray sequence 00, 01, 11, 10 - one changing at a time, which is
    /// what a real part does and what makes a missed state detectable.
    static void step(int8_t direction) {
        // forward from state s, and backward, as tables rather than
        // arithmetic: the order is a property of the part.
        constexpr uint8_t forward[4] = {1, 3, 0, 2};  // 00->01 01->11 10->00 11->10
        constexpr uint8_t back[4] = {2, 0, 3, 1};     // 00->10 01->00 10->11 11->01
        state_ = direction >= 0 ? forward[state_ & 3u] : back[state_ & 3u];
    }

    /// Turn by several counts at once. What a shaft does between two
    /// samples of a decoder that cannot keep up.
    static void spin(int8_t counts) {
        const int8_t dir = counts >= 0 ? 1 : -1;
        for (int8_t i = 0; i < (counts >= 0 ? counts : -counts); ++i) {
            step(dir);
        }
    }

    /// Put the contacts somewhere directly, bypassing the sequence: the
    /// door for bounce, for a skipped state, for anything a clean model
    /// would not produce.
    static void force(uint8_t state) { state_ = static_cast<uint8_t>(state & 3u); }

    /// Where the contacts are, for a test that wants to say so.
    static uint8_t state() { return state_; }

    static void reset() { state_ = 0; }

private:
    static inline uint8_t state_ = 0;
};

static_assert(ScannedInput<SimButton<0>>);
static_assert(ScannedInput<SimEncoder<0>::A>);
static_assert(ScannedInput<SimEncoder<0>::B>);

} // namespace brio
