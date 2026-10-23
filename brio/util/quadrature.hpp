/*
 * quadrature.hpp (util)
 *
 * Quadrature: the active object (an object with its own event queue,
 * dispatched by the kernel loop) that watches the two contacts of an
 * incremental rotary encoder and publishes when the shaft has turned
 * by whole detents - the knob half of what InputScanner is for a
 * button. A program names the two contacts as ScannedInput types (each
 * a struct whose read() returns the contact's level,
 * util/input_scanner.hpp), lists the decoder among its active objects
 * and gives it a poll period; each subscriber handles Turned{detents}.
 * The smallest use, two pads of port A read as the contacts:
 *
 *     struct PadA { static bool read() { return brio::Pin<'A', 0>::read(); } };
 *     struct PadB { static bool read() { return brio::Pin<'A', 1>::read(); } };
 *     using Knob = brio::Quadrature<P, brio::Subscribers<Panel>, PadA, PadB>;
 *
 *     // once Tenuto's init_all() has run (it calls init() with no
 *     // period, which leaves the decoder quiet): from an event handler, say
 *     Knob::init(brio::ticks_from_ms<P>(1));   // poll every millisecond
 *
 * Who calls what, and from where: everything runs in the kernel loop,
 * in main context. The decoder's TimeEvent posts a QuadratureTick every
 * period; its dispatch reads the two contacts once (two read() calls,
 * nothing masked, nothing waited for) and publishes a Turned by value
 * when whole detents have accumulated. init(), start_every(), stop()
 * and running_every() drive that TimeEvent, so they are main context
 * only (kernel/time_event.hpp), never an interrupt handler. No
 * interrupt is involved at all: the contacts are polled.
 *
 * An incremental encoder has no position. It says add or subtract, and
 * a count is an accumulator somebody added; here that accumulator is
 * private and never leaves, because what an application wants is "the
 * user turned it by two" and not a number to subtract from last time's.
 * A concept over a position would only be needed to abstract over the
 * hardware counters some families have, and is born with the first user
 * that needs one - which will be a motor and not a front panel.
 *
 * Why software, on every target. Hardware quadrature decoding exists on
 * some of these families and not others (a timer mode on three, a
 * programmable block on one, an application note's worth of glue on the
 * AVR, nothing at all on the SAM C21), and a panel knob does not justify
 * spending a timer anywhere. The arithmetic says so: a common 20-detent
 * part at four counts a detent gives 80 states a revolution, so a hand
 * at one turn a second changes state every 12 ms and a poll on the
 * kernel's own millisecond tick oversamples it twelvefold. Hardware
 * decoding earns its keep at motor speeds and nowhere else.
 *
 * What a missed sample costs, which is the whole of what can go wrong.
 * One missed state is detectable: two states away is not a legal step,
 * and is the same distance in both directions, so nothing can say which
 * way it went. Two missed states are worse than undetectable - three
 * states forward is one state backward, and the decoder reports a turn
 * in the wrong direction. That is a property of the encoding and not of
 * this implementation; the defence is sampling fast enough, and the
 * measured degradation is in the host suite (test/test_quadrature, "a
 * shaft outrunning the poll degrades in a known way").
 *
 * The button is not here. The shaft of these parts carries a push
 * switch, and it is a contact like any other: it belongs to
 * InputScanner, which already debounces contacts, and putting it here
 * would be a second place for the same mechanism.
 *
 * Counts per detent is a fact of the part - four on most, two on some -
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
 * The step from one pair of contacts to the next (+1, -1, or 0 for no
 * change), or nothing when at least one state was missed. Pure and
 * constexpr; only the low two bits of each argument are read.
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

/// The knobs, as a non-type template parameter (a value given in the
/// angle brackets) so a wrong value cannot reach run time.
struct QuadratureConfig {
    /// Quadrature counts the part gives per detent. A fact of the part -
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
 * The active object: one periodic tick, one reading of the two contacts,
 * one Turned per tick on which whole detents accumulated (its count may
 * be more than one). The period is set at init(); start_every() re-paces
 * a running decoder and stop() silences it.
 *
 * No turn at startup. The first reading establishes where the shaft
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
     * Start decoding: every count and counter back to zero, any pace in
     * force disarmed, and the new one armed when `period_ticks` is not
     * zero. Main context. The shaft's position is unknown until the
     * first tick reads it.
     *
     * The period is defaulted because the kernel's active-object
     * contract calls init() with no arguments: Tenuto::init_all() leaves
     * the decoder quiet and the application arms it right after. A pace
     * armed before init_all() is undone by it: this init() disarms
     * first, and Tenuto::run() calls init_all() before its first event.
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

    /// Re-pace a running decoder, stop it (its counts are kept), and
    /// whether a pace is armed. Main context only, like every TimeEvent.
    static void start_every(uint32_t period_ticks) {
        tick_.arm_every(period_ticks);
    }
    static void stop() { tick_.disarm(); }
    static bool running_every() { return tick_.armed(); }

    /// Has the shaft's position been read at all yet?
    static bool settled() { return settled_; }

    /// Readings that said a state had been missed. Saturating. The
    /// accounting is the API: a decoder that cannot keep up cannot be
    /// made correct by cleverness, so it says how often it could not.
    static uint16_t lost() { return lost_; }

    /// Turned events published since init() (one per tick that
    /// published, whatever its count). Saturating.
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
