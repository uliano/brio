/*
 * quadrature.hpp (util)
 *
 * Quadrature: the active object that watches the two contacts of an
 * incremental rotary encoder and publishes when the shaft has turned by
 * whole detents - the knob half of what InputScanner is for a button.
 *
 * AN INCREMENTAL ENCODER HAS NO POSITION. It says add or subtract, and
 * a count is an accumulator somebody added; here that accumulator is
 * private and never leaves, because what an application wants is "the
 * user turned it by two" and not a number to subtract from last time's.
 * A concept over a POSITION would only be needed to abstract over the
 * hardware counters some families have, and is born with the first user
 * that needs one - which will be a motor and not a front panel.
 *
 * WHY SOFTWARE, ON EVERY TARGET. Hardware quadrature decoding exists on
 * some of these families and not others (a timer mode on three, a
 * programmable block on one, an application note's worth of glue on the
 * AVR, nothing at all on the SAM C21), and a panel knob does not justify
 * spending a timer anywhere. The arithmetic says so: a common 20-detent
 * part at four counts a detent gives 80 states a revolution, so a hand
 * at one turn a second changes state every 12 ms and a poll on the
 * kernel's own millisecond tick oversamples it twelvefold. Hardware
 * decoding earns its keep at motor speeds and nowhere else.
 *
 * WHAT A MISSED SAMPLE COSTS, which is the whole of what can go wrong.
 * One missed state is DETECTABLE: two states away is not a legal step,
 * and is the same distance in both directions, so nothing can say which
 * way it went. Two missed states are worse than undetectable - three
 * states forward is one state backward, and the decoder reports a turn
 * in the wrong direction. That is a property of the encoding and not of
 * this implementation; the defence is sampling fast enough, and the
 * measured degradation is in the test suite.
 *
 * THE BUTTON IS NOT HERE. The shaft of these parts carries a push
 * switch, and it is a contact like any other: it belongs to
 * InputScanner, which already debounces contacts, and putting it here
 * would be a second place for the same mechanism.
 *
 * COUNTS PER DETENT IS A FACT OF THE PART - four on most, two on some -
 * so it is configuration and not a constant, and it is to be measured
 * with the part in hand rather than assumed.
 */

#pragma once

#include <stdint.h>

#include <concepts>
#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "util/input_scanner.hpp"

namespace brio {

/**
 * The step from one pair of contacts to the next, or NOTHING when at
 * least one state was missed.
 *
 * A state is the two contacts read as (a << 1) | b, and a turn walks
 * them in Gray order - 00, 01, 11, 10 - so that exactly one contact
 * changes at a time. That is what makes a jump of two detectable at all:
 * it is the only transition where both changed.
 */
constexpr std::optional<int8_t> quadrature_step(uint8_t prev, uint8_t curr) {
    // Indexed by (prev << 2) | curr. 2 marks the jump that cannot be
    // read: two states away is equally far in both directions.
    constexpr int8_t lost = 2;
    constexpr int8_t table[16] = {
        //          to 00    to 01    to 10    to 11
        /* 00 */       0,      +1,      -1,    lost,
        /* 01 */      -1,       0,    lost,      +1,
        /* 10 */      +1,    lost,       0,      -1,
        /* 11 */    lost,      -1,      +1,       0,
    };
    const int8_t s = table[((prev & 3u) << 2) | (curr & 3u)];
    if (s == lost) {
        return {};
    }
    return s;
}

/// Published when the shaft has turned by whole detents: positive one
/// way, negative the other, and never zero. Counts short of a whole
/// detent are kept and go towards the next one.
struct Turned {
    int8_t detents;
};

/// The decoder's own periodic tick.
struct QuadratureTick {};

/// The knobs, as an NTTP so a wrong value cannot reach run time.
struct QuadratureConfig {
    /// Quadrature counts the part gives per detent. A FACT OF THE PART -
    /// four on most, two on some - to be measured, never assumed.
    uint8_t counts_per_detent = 4;

    /// What to do when a reading says a state was missed. False keeps
    /// still: under a spin faster than the poll the knob feels sticky
    /// but never runs backwards, which is the better failure for a
    /// front panel. True assumes the shaft kept going and counts two in
    /// the last known direction, which keeps up better and is wrong if
    /// the user really did reverse at speed.
    bool count_lost_steps = false;
};

/**
 * The AO: one periodic tick, one reading of the two contacts, one
 * Turned per whole detent. The period is set at init(); start_every()
 * re-paces a running decoder and stop() silences it.
 *
 * NO TURN AT STARTUP. The first reading ESTABLISHES where the shaft
 * sits without publishing it - a knob left somewhere is a fact about the
 * world and not something the user just did.
 */
template <Platform P, typename Subs, ScannedInput A, ScannedInput B,
          QuadratureConfig config = QuadratureConfig{}>
class Quadrature
    : public Fsm<Quadrature<P, Subs, A, B, config>, QuadratureTick> {
    using Self = Quadrature<P, Subs, A, B, config>;
    using Base = Fsm<Self, QuadratureTick>;

    static_assert(config.counts_per_detent > 0,
                  "Quadrature: a detent of no counts never arrives");

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static constexpr uint8_t counts_per_detent = config.counts_per_detent;

    /// Depth two: the tick, and one more in case a long dispatch
    /// elsewhere let a second mature.
    static inline EventQueue<Event, 2, P> queue;

    /**
     * Start decoding. The shaft's position is unknown until the first
     * tick reads it.
     *
     * The period is DEFAULTED because the kernel's AO contract calls
     * init() with no arguments: Tenuto::init_all() leaves the decoder
     * quiet and the application arms it right after.
     */
    static void init(uint32_t period_ticks = 0) {
        state_ = 0;
        settled_ = false;
        counts_ = 0;
        last_dir_ = 0;
        lost_ = 0;
        detents_ = 0;
        tick_.disarm();
        Base::start(&running);
        if (period_ticks != 0) {
            tick_.arm_every(period_ticks);
        }
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    static void start_every(uint32_t period_ticks) {
        tick_.arm_every(period_ticks);
    }
    static void stop() { tick_.disarm(); }
    static bool running_every() { return tick_.armed(); }

    /// Has the shaft's position been read at all yet?
    static bool settled() { return settled_; }

    /// Readings that said a state had been missed. Saturating. THE
    /// ACCOUNTING IS THE API: a decoder that cannot keep up cannot be
    /// made correct by cleverness, so it says how often it could not.
    static uint16_t lost() { return lost_; }

    /// Detents published since init(). Saturating.
    static uint16_t detents() { return detents_; }

private:
    static Status running(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](QuadratureTick) {
                sample();
                return Base::handled();
            });
    }

    static void sample() {
        const uint8_t now =
            static_cast<uint8_t>((A::read() ? 2u : 0u) | (B::read() ? 1u : 0u));
        if (!settled_) {
            state_ = now;
            settled_ = true;
            return;
        }
        const std::optional<int8_t> step = quadrature_step(state_, now);
        state_ = now;

        if (!step) {
            if (lost_ != UINT16_MAX) {
                ++lost_;
            }
            if constexpr (config.count_lost_steps) {
                counts_ = static_cast<int8_t>(counts_ + 2 * last_dir_);
            } else {
                return;
            }
        } else {
            if (*step == 0) {
                return;
            }
            counts_ = static_cast<int8_t>(counts_ + *step);
            last_dir_ = *step;
        }

        int8_t whole = 0;
        while (counts_ >= static_cast<int8_t>(counts_per_detent)) {
            counts_ = static_cast<int8_t>(counts_ - counts_per_detent);
            ++whole;
        }
        while (counts_ <= -static_cast<int8_t>(counts_per_detent)) {
            counts_ = static_cast<int8_t>(counts_ + counts_per_detent);
            --whole;
        }
        if (whole != 0) {
            publish(Subs{}, Turned{whole});
            if (detents_ != UINT16_MAX) {
                ++detents_;
            }
        }
    }

    static inline TimeEvent<P, Self, QuadratureTick> tick_{QuadratureTick{}};
    static inline uint8_t state_ = 0;
    static inline bool settled_ = false;
    static inline int8_t counts_ = 0;
    static inline int8_t last_dir_ = 0;
    static inline uint16_t lost_ = 0;
    static inline uint16_t detents_ = 0;
};

} // namespace brio
