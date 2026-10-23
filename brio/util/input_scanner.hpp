/*
 * input_scanner.hpp (util)
 *
 * InputScanner: the active object (an object with its own event queue,
 * dispatched by the kernel loop) that watches a fixed list of binary
 * inputs and publishes an event when one of them changes - the button
 * half of what AnalogSampler is for a converter. A program wraps each
 * contact in a type whose read() returns true when it is active, lists
 * the scanner among its active objects with a poll period, and each
 * subscriber handles InputEdge{index, active}, debounced. The smallest
 * use, as the family fixture test/family_avrxt/util_services.cpp
 * spells it:
 *
 *     struct ButtonA2 { static bool read() { return !brio::Pin<'A', 2>::read(); } };
 *     struct ButtonA3 { static bool read() { return !brio::Pin<'A', 3>::read(); } };
 *     using Buttons = brio::InputScanner<P, brio::Subscribers<Watcher>, brio::ScanConfig{},
 *                                        ButtonA2, ButtonA3>;
 *
 *     // once Tenuto's init_all() has run (it calls init() with no
 *     // period, which leaves the scanner quiet): from an event handler, say
 *     Buttons::init(brio::ticks_from_ms<P>(4));   // a reading every 4 ms
 *
 * Who calls what, and from where: everything runs in the kernel loop,
 * in main context. The scanner's TimeEvent posts a ScanTick every
 * period; its dispatch calls each input's read() once (nothing masked,
 * nothing waited for) and publishes an InputEdge by value for each
 * believed flip. init(), start_every(), stop() and running_every()
 * drive that TimeEvent, so they are main context only
 * (kernel/time_event.hpp), never an interrupt handler; state(),
 * settled(), all_settled() and edges() are plain reads of what the last
 * dispatch left. No interrupt is involved: the inputs are polled.
 *
 * Why polling and not an edge interrupt. A mechanical contact does not
 * change state: it changes it a few dozen times over a few
 * milliseconds, and every one of those changes is a real edge the pin's
 * interrupt would report. Debouncing in the interrupt handler means
 * counting time there; debouncing after it means the interrupt bought
 * nothing. Sampling on the kernel's own timebase costs one dispatch per
 * period for the whole list, needs no pin hardware beyond a readable
 * level, and makes the debounce what it actually is - a decision about
 * how long a level must hold before it is believed.
 *
 * The rule. `stable_samples` consecutive readings of the same level
 * flip the published state; anything shorter is absorbed. So the worst
 * case a real press takes to be seen is period x stable_samples, and
 * any bounce shorter than that is invisible. The default, three samples,
 * is a working figure for switches on a 1 ms-class tick; an input with
 * a nastier contact or a slower tick sets its own.
 *
 * No edge at startup. The first `stable_samples` readings establish the
 * state without publishing it: a switch already held down when the
 * program boots is a fact about the world, not something that just
 * happened, and an app that treats every InputEdge as a user action
 * would otherwise act on a press nobody made. Whoever needs the initial
 * levels reads state(i) once settled(i) (or all_settled()) is true.
 *
 * Polarity is the input's. `read()` returns true for active, and what
 * makes an input active is the input's own business: a button to ground
 * with a pull-up wraps its pin and inverts there, as ButtonA2 above
 * does - because that inversion is a fact about the wiring, which
 * belongs where the wiring is described. The scanner has no polarity
 * knob for the same reason it has no pin type: it would be a second
 * place for the same truth. A type without a read() returning bool is
 * refused where it is listed, at compile time (the negative fixture
 * test/family_avrxt/neg/scanner_input_without_read.cpp).
 *
 * The mechanism asks for nothing target-specific: a periodic read of a
 * level, which every target has. It has not yet met a real contact -
 * the debounce count is a chosen number, not a measured one; the host
 * suite (test/test_input_scanner) drives it with simulated inputs.
 * (docs/design/overview.md, "Authority of util/".)
 */

#pragma once

#include <stdint.h>
#include <concepts>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"

namespace brio {

/// What the scanner needs of an input: a level, true when active.
template <typename I>
concept ScannedInput = requires {
    { I::read() } -> std::same_as<bool>;
};

/// Published on every flip: input `index` (its position in the list) is
/// now active or no longer active. Nothing is published while a level
/// merely holds.
struct InputEdge {
    uint8_t index;
    bool active;
};

/// The scanner's own periodic tick.
struct ScanTick {};

/// The debounce knob, as a non-type template parameter (a value given
/// in the angle brackets) so a wrong value cannot reach run time.
struct ScanConfig {
    /// Consecutive equal readings that make a level believed. One
    /// degenerates to raw sampling: every tick's reading is the state.
    uint8_t stable_samples = 3;
};

/**
 * The active object: one periodic tick, one read of each input, one
 * InputEdge per flip. The period is set at init(); start_every()
 * re-paces a running scanner and stop() silences it.
 */
template <Platform P, typename Subs, ScanConfig config = ScanConfig{},
          ScannedInput... Inputs>
    requires (sizeof...(Inputs) > 0)
class InputScanner : public Fsm<InputScanner<P, Subs, config, Inputs...>, ScanTick> {
    using Self = InputScanner<P, Subs, config, Inputs...>;
    using Base = Fsm<Self, ScanTick>;

    static_assert(config.stable_samples > 0,
                  "InputScanner: zero samples believe nothing");

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static constexpr uint8_t input_count = sizeof...(Inputs);
    static constexpr uint8_t stable_samples = config.stable_samples;

    /// Depth two: the tick, and one more in case a long dispatch
    /// elsewhere let a second mature.
    static inline EventQueue<Event, 2, P> queue;

    /**
     * Start scanning: edges() back to zero, any pace in force disarmed,
     * and the new one armed when `period_ticks` is not zero. Main
     * context. Every input begins unsettled: the first stable_samples
     * readings establish its state silently.
     *
     * The period is defaulted because the kernel's active-object
     * contract calls init() with no arguments: Tenuto::init_all() leaves
     * the scanner quiet and the application arms it right after with
     * init(period) or start_every(period). A pace armed before
     * init_all() is undone by it: this init() disarms first, and
     * Tenuto::run() calls init_all() before its first event.
     */
    static void init(uint32_t period_ticks = 0) {
        for (uint8_t i = 0; i < input_count; ++i) {
            state_[i] = false;
            candidate_[i] = false;
            run_[i] = 0;
            settled_[i] = false;
        }
        edges_ = 0;
        tick_.disarm();
        Base::start(&running);
        if (period_ticks != 0) {
            tick_.arm_every(period_ticks);
        }
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Re-pace a running scanner, stop it (the believed states are
    /// kept), and whether a pace is armed. Main context only, like every
    /// TimeEvent.
    static void start_every(uint32_t period_ticks) { tick_.arm_every(period_ticks); }
    static void stop() { tick_.disarm(); }
    static bool running_every() { return tick_.armed(); }

    /// The believed level of input `index`, false for an out-of-range one.
    static bool state(uint8_t index) {
        return index < input_count ? state_[index] : false;
    }

    /// Has input `index` seen enough samples to have a state at all?
    static bool settled(uint8_t index) {
        return index < input_count ? settled_[index] : false;
    }

    /// Every input has established its level: the moment from which an
    /// InputEdge can arrive.
    static bool all_settled() {
        for (uint8_t i = 0; i < input_count; ++i) {
            if (!settled_[i]) {
                return false;
            }
        }
        return true;
    }

    /// Edges published since init(). Saturating.
    static uint16_t edges() { return edges_; }

private:
    static Status running(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](ScanTick) {
                uint8_t i = 0;
                (scan_one<Inputs>(i), ...);
                return Base::handled();
            });
    }

    /// One input's debounce: count the run of equal readings, and act
    /// when it reaches the threshold.
    template <typename I>
    static void scan_one(uint8_t& i) {
        const uint8_t k = i++;
        const bool now = I::read();
        if (now != candidate_[k]) {
            candidate_[k] = now;
            run_[k] = 1;
        } else if (run_[k] < stable_samples) {
            ++run_[k];
        }
        if (run_[k] < stable_samples) {
            return;
        }
        if (!settled_[k]) {
            settled_[k] = true;      // power-on is not a press
            state_[k] = now;
            return;
        }
        if (now != state_[k]) {
            state_[k] = now;
            publish(Subs{}, InputEdge{k, now});
            if (edges_ != UINT16_MAX) {
                ++edges_;
            }
        }
    }

    static inline TimeEvent<P, Self, ScanTick> tick_{ScanTick{}};
    static inline bool state_[input_count]{};
    static inline bool candidate_[input_count]{};
    static inline bool settled_[input_count]{};
    static inline uint8_t run_[input_count]{};
    static inline uint16_t edges_ = 0;
};

} // namespace brio
