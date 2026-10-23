/*
 * meter_sampler.hpp (util)
 *
 * MeterSampler: the active object (an object with its own event queue,
 * dispatched by the kernel loop) that turns a set of hardware meters
 * into a paced stream of value events, and MeterLatch, the one-cell
 * bridge that carries a measurement out of the capture interrupt. A
 * program that measures a frequency or a pulse width with a timer's
 * capture names one latch per meter, stores each reading into its latch
 * from the capture handler, and lets the sampler publish the latest
 * readings at the program's own pace. The smallest use, as
 * docs/design/meters.md and the AVR's meter suite (test_avr_meter) spell
 * it:
 *
 *     using Period = brio::MeterLatch<uint16_t, P, 0>;
 *     using Width  = brio::MeterLatch<uint16_t, P, 1>;
 *     using Meters = brio::MeterSampler<P, brio::Subscribers<Display>, Period, Width>;
 *
 *     ISR(TCB0_INT_vect) { Period::store(brio::FrequencyMeter<brio::Tcb<0>>::period_ticks()); }
 *     ISR(TCB1_INT_vect) { Width::store(brio::PulseWidthMeter<brio::Tcb<1>>::width_ticks()); }
 *
 *     // after the meters' own init() (avrxt/tcb.hpp), and once Tenuto's
 *     // init_all() has run (it calls init() with no period, which leaves
 *     // the sampler quiet): from an event handler, say
 *     Meters::init(brio::ticks_from_ms<P>(100));   // publish ten times a second
 *
 * Each subscriber handles MeterSample{index, value}: index is the
 * source's position in the list, value the reading widened to 32 bits.
 * The page this service belongs to is docs/design/meters.md, "Meters in
 * the kernel".
 *
 * Who calls what, and from where:
 *  - the capture interrupt handler (the driver's body returns the
 *    reading and re-arms the capture) calls the latch's store(), which
 *    masks interrupts for the few instructions of the copy and never
 *    waits;
 *  - the sampler runs in the kernel loop, in main context: on each
 *    MeterTick of its period it calls every source's take() (the same
 *    short critical section) and publishes one MeterSample per source
 *    that had a fresh value. Subscribers receive it by value, copied
 *    into their own queues: nothing is lent;
 *  - init(period), start_every(), stop() and running_every() drive a
 *    TimeEvent, so they run in main context only
 *    (kernel/time_event.hpp), never in an interrupt handler. Each latch
 *    has one producer, the handler that stores into it.
 *
 * The problem this solves is not measuring - the silicon already does
 * that. A timer in a capture mode produces one reading per edge of the
 * signal it watches, and the edges come at the signal's rate: a 10 kHz
 * input is ten thousand readings a second, each one an interrupt. A
 * program that wants "how fast is the fan turning" does not want ten
 * thousand events a second, and a queue that is offered them either
 * overflows or starves everything else in the loop. So the pace of the
 * measurement and the pace of the publication are two different
 * things, and this file is where they are separated:
 *
 *   edge -> capture ISR -> MeterLatch::store()      (as fast as the wire)
 *   tick -> MeterSampler -> publish(MeterSample)    (as fast as the app)
 *
 * Why a latch and not a queue (docs/design/meters.md, "Why a latch and
 * not a queue"). What a periodic reading is worth is its last value;
 * the ones before it are history nobody asked for. A single cell is
 * therefore the right container - it costs one value and one flag, it
 * cannot overflow, and the reading it holds is always the most recent
 * one. What a queue would give instead is a backlog of stale
 * measurements and a hard question about what to do when it fills. The
 * overwrites are not silent, though: missed() counts every store that
 * landed on an untaken value, which is exactly "the signal is faster
 * than my pace" - a diagnostic, not an error.
 *
 * Why the sampler publishes only fresh values (docs/design/meters.md,
 * "Why the sampler publishes only fresh values"). A subscriber that
 * receives an event learns a fact: this meter read this. A repeat of
 * the previous reading is not a fact about the signal, it is a fact
 * about the sampler having nothing to say - and a subscriber cannot
 * tell the two apart once they look identical. So a stale source
 * publishes nothing and silence carries the information: no edges
 * arrived. A subscriber that needs to act on that silence (a stall
 * detector, a watchdog) times it with its own time event, which is the
 * only place where "how long is too long" is known.
 *
 * The drivers stay untouched. A target's capture tasks (the driver
 * classes a program uses for one job of a timer) - a frequency, a
 * pulse-width or a duty meter - expose interrupt handler bodies that
 * return a reading and re-arm the capture; the application's vector
 * binding is what joins them to a latch, exactly as it joins a
 * converter's result to AnalogSampler. `Meter` below is a placeholder
 * for the family's meter type, a driver verb and not a register read
 * (avrxt/tcb.hpp's FrequencyMeter and ch32v00x/tim.hpp's TimPeriodMeter
 * both offer period_ticks()), and the vector binding is the family's
 * own spelling:
 *
 *   using Period = brio::MeterLatch<uint16_t, P, 0>;
 *   ISR(<the capture vector>) { Period::store(Meter::period_ticks()); }
 *
 * The sampler never names a timer, an interrupt or a unit: a source is
 * anything with a take(), the value is an unsigned number, and what it
 * means (ticks, edges, a period to be turned into Hz) is the
 * subscriber's business - the same division of labour AnalogSampler
 * makes between raw counts and millivolts.
 *
 * The contract carries nothing target-specific: a capture unit that
 * reports through an interrupt exists on most machines brio targets
 * (the RP2040 and the RP2350 have none, and offer no MeterSource; the
 * ch32x035 stratum offers none either, its timers not written there:
 * docs/design/meters.md's realizations table), and a peripheral that
 * instead keeps a readable register needs no latch at all - it
 * satisfies MeterSource by reading it (an adapter of three lines).
 * (docs/design/meters.md, docs/design/overview.md "Authority of
 * util/".)
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

namespace brio {

/**
 * The bridge from a capture interrupt to the loop: one value, one
 * freshness flag, and a count of what the freshness flag cost.
 *
 * The type is the object - a latch is addressed the way an active
 * object is, so the interrupt glue names it with no pointer to plumb.
 * Two latches of the same width therefore need different `id`s: the id
 * is what makes them two (one type with one id is one cell, whatever
 * the code that names it).
 *
 * store() is the interrupt half and takes the platform's critical
 * section, so the main-context take() can never read a half-written
 * wide value on a machine whose store is not atomic (every 8-bit one).
 * Every verb takes that critical section for a few loads and stores
 * and none waits, so each may be called from either context; the
 * contract is one producer (the capture handler's store()) and one
 * consumer (the sampler's take()).
 */
template <std::unsigned_integral T, Platform P, uint8_t id = 0>
class MeterLatch {
public:
    using Value = T;

    /// Called from the capture interrupt handler with the reading it just
    /// took. Overwrites whatever was there; an overwrite of an untaken
    /// value is counted in missed().
    static void store(T v) {
        typename P::CriticalSection cs;
        if (fresh_ && missed_ != UINT16_MAX) {
            ++missed_;
        }
        value_ = v;
        fresh_ = true;
    }

    /// Read and clear: the value if one arrived since the last take(),
    /// std::nullopt otherwise. Main context (the sampler's dispatch).
    static std::optional<T> take() {
        typename P::CriticalSection cs;
        if (!fresh_) {
            return std::nullopt;
        }
        fresh_ = false;
        return value_;
    }

    /// Stores that landed on an untaken value: the signal is arriving
    /// faster than it is being consumed. Saturating.
    static uint16_t missed() {
        typename P::CriticalSection cs;
        return missed_;
    }

    /// Is a value waiting? (Diagnostics: take() is the verb.)
    static bool fresh() {
        typename P::CriticalSection cs;
        return fresh_;
    }

    /// Back to the state a cold start leaves: nothing waiting, nothing
    /// missed.
    static void clear() {
        typename P::CriticalSection cs;
        fresh_ = false;
        missed_ = 0;
    }

private:
    static inline T value_{};
    static inline uint16_t missed_ = 0;
    static inline bool fresh_ = false;
};

/// What MeterSampler needs of a source: a read-and-clear that says
/// whether anything arrived. Any unsigned width does - the sampler
/// widens to 32 bits, which is what the event carries.
template <typename S>
concept MeterSource = requires {
    typename decltype(S::take())::value_type;
    requires std::unsigned_integral<typename decltype(S::take())::value_type>;
    requires sizeof(typename decltype(S::take())::value_type) <= sizeof(uint32_t);
};

/// The sampler's own periodic tick.
struct MeterTick {};

/// Published to the subscribers: source `index` (its position in the
/// sampler's list) read `value`. Only ever sent for a fresh reading -
/// see the file header.
struct MeterSample {
    uint8_t index;
    uint32_t value;
};

/**
 * The active object: one periodic tick, one pass over the sources in
 * list order, one MeterSample published per source that had something
 * to say.
 *
 * The period is set at init() because a sampler with no pace has no
 * reason to exist; start_every() re-paces a running one and stop()
 * silences it without forgetting the sources.
 */
template <Platform P, typename Subs, MeterSource... Sources>
    requires (sizeof...(Sources) > 0)
class MeterSampler : public Fsm<MeterSampler<P, Subs, Sources...>, MeterTick> {
    using Self = MeterSampler<P, Subs, Sources...>;
    using Base = Fsm<Self, MeterTick>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static constexpr uint8_t source_count = sizeof...(Sources);

    /// Depth two: the tick, and one more in case a long dispatch
    /// elsewhere let a second mature. A sampler that falls further
    /// behind than that is mis-paced, and the queue's overflow counter
    /// is where it says so.
    static inline EventQueue<Event, 2, P> queue;

    /**
     * Arm the pace. The sources must already be fed by their interrupt
     * glue (the sampler configures no hardware - it does not know any).
     * Main context. Resets published() to zero and disarms any pace in
     * force before arming the new one; the latches are not cleared.
     *
     * The period is an argument because a sampler without a pace does
     * nothing, and it is defaulted because the kernel's active-object
     * contract calls init() with none: Tenuto::init_all() therefore
     * leaves the sampler quiet, and the application arms it right after
     * with init(period) or start_every(period) - the same order in which
     * it configures the hardware the sources read. A pace armed before
     * init_all() is undone by it: this init() disarms first, and
     * Tenuto::run() calls init_all() before its first event.
     */
    static void init(uint32_t period_ticks = 0) {
        published_ = 0;
        tick_.disarm();
        Base::start(&running);
        if (period_ticks != 0) {
            tick_.arm_every(period_ticks);
        }
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Re-pace a running sampler, stop it (the sources and their
    /// latches are kept, and keep filling), and whether a pace is armed.
    /// Main context only, like every TimeEvent.
    static void start_every(uint32_t period_ticks) { tick_.arm_every(period_ticks); }
    static void stop() { tick_.disarm(); }
    static bool running_every() { return tick_.armed(); }

    /// Samples published since init(), all sources together. Saturating.
    static uint16_t published() { return published_; }

    /// A source's own overwrite count, passed through for diagnostics.
    /// Zero for a source that does not keep one (the concept asks only
    /// for take()).
    static uint16_t missed(uint8_t index) {
        uint16_t n = 0;
        uint8_t k = 0;
        (pick_missed<Sources>(k, index, n), ...);
        return n;
    }

private:
    static Status running(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](MeterTick) {
                uint8_t i = 0;
                (sweep_one<Sources>(i), ...);
                return Base::handled();
            });
    }

    /// One source: take it, and publish only if it had something.
    template <typename S>
    static void sweep_one(uint8_t& i) {
        const auto v = S::take();
        if (v.has_value()) {
            publish(Subs{}, MeterSample{i, static_cast<uint32_t>(*v)});
            if (published_ != UINT16_MAX) {
                ++published_;
            }
        }
        ++i;
    }

    /// The pack unrolled into a chain of compares - the same shape
    /// AnalogSampler uses to select the i-th input of its list.
    template <typename S>
    static void pick_missed(uint8_t& k, uint8_t index, uint16_t& out) {
        if (k++ == index) {
            out = missed_of<S>();
        }
    }

    template <typename S>
    static uint16_t missed_of() {
        if constexpr (requires { { S::missed() } -> std::convertible_to<uint16_t>; }) {
            return static_cast<uint16_t>(S::missed());
        } else {
            return 0;
        }
    }

    static inline TimeEvent<P, Self, MeterTick> tick_{MeterTick{}};
    static inline uint16_t published_ = 0;
};

} // namespace brio
