/*
 * meter_sampler.hpp (util)
 *
 * MeterSampler: the active object that turns a set of HARDWARE METERS
 * into a paced stream of value events, and MeterLatch, the one-cell
 * bridge that carries a measurement out of the capture interrupt.
 *
 * The problem this solves is not measuring - the silicon already does
 * that. A timer in a capture mode produces one reading per EDGE of the
 * signal it watches, and the edges come at the signal's rate: a 10 kHz
 * input is ten thousand readings a second, each one an interrupt. A
 * program that wants "how fast is the fan turning" does not want ten
 * thousand events a second, and a queue that is offered them either
 * overflows or starves everything else in the loop. So the pace of the
 * MEASUREMENT and the pace of the PUBLICATION are two different things,
 * and this file is where they are separated:
 *
 *   edge -> capture ISR -> MeterLatch::store()      (as fast as the wire)
 *   tick -> MeterSampler -> publish(MeterSample)    (as fast as the app)
 *
 * WHY A LATCH AND NOT A QUEUE. What a periodic reading is worth is its
 * LAST value; the ones before it are history nobody asked for. A single
 * cell is therefore the right container - it costs one value and one
 * flag, it cannot overflow, and the reading it holds is always the most
 * recent one. What a queue would give instead is a backlog of stale
 * measurements and a hard question about what to do when it fills. The
 * overwrites are not silent, though: missed() counts every store that
 * landed on an untaken value, which is exactly "the signal is faster
 * than my pace" - a diagnostic, not an error.
 *
 * WHY THE SAMPLER PUBLISHES ONLY FRESH VALUES. A subscriber that
 * receives an event learns a FACT: this meter read this. A repeat of the
 * previous reading is not a fact about the signal, it is a fact about
 * the sampler having nothing to say - and a subscriber cannot tell the
 * two apart once they look identical. So a stale source publishes
 * NOTHING and silence carries the information: no edges arrived. A
 * subscriber that needs to act on that silence (a stall detector, a
 * watchdog) times it with its own time event, which is the only place
 * where "how long is too long" is known.
 *
 * THE DRIVERS STAY UNTOUCHED. avrdx/tcb.hpp's FrequencyMeter,
 * PulseWidthMeter and DutyMeter expose ISR handler BODIES that return a
 * reading and re-arm the capture; the application's vector binding is
 * what joins them to a latch, exactly as it joins a converter's result
 * to AnalogSampler:
 *
 *   using Period = brio::MeterLatch<uint16_t, P, 0>;
 *   ISR(TCB0_INT_vect) { Period::store(brio::FrequencyMeter<Tcb<0>>::period_ticks()); }
 *
 * The sampler never names a timer, an interrupt or a unit: a source is
 * anything with a take(), the value is an unsigned number, and what it
 * MEANS (ticks, edges, a period to be turned into Hz) is the
 * subscriber's business - the same division of labour AnalogSampler
 * makes between raw counts and millivolts.
 *
 * Validated on: AVR DA/DB (the three TCB meters) and the host fake. The
 * contract carries nothing target-specific: a capture unit that reports
 * through an interrupt exists on every machine brio targets, and a
 * peripheral that instead keeps a readable register needs no latch at
 * all - it satisfies MeterSource by reading it (an adapter of three
 * lines). (docs/design/meters.md, docs/design/overview.md "Authority of
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
 * The TYPE is the object - a latch is addressed the way an AO is, so
 * the ISR glue names it with no pointer to plumb. Two latches of the
 * same width therefore need different `id`s: the id is what makes them
 * two.
 *
 * store() is the ISR half and takes the platform's critical section, so
 * the main-context take() can never read a half-written wide value on a
 * machine whose store is not atomic (every 8-bit one).
 */
template <std::unsigned_integral T, Platform P, uint8_t id = 0>
class MeterLatch {
public:
    using Value = T;

    /// Called from the capture ISR with the reading it just took.
    /// Overwrites whatever was there; an overwrite of an UNTAKEN value
    /// is counted.
    static void store(T v) {
        typename P::CriticalSection cs;
        if (fresh_ && missed_ != UINT16_MAX) {
            ++missed_;
        }
        value_ = v;
        fresh_ = true;
    }

    /// Read and clear: the value if one arrived since the last take(),
    /// nothing otherwise. Main context.
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
/// sampler's list) read `value`. Only ever sent for a FRESH reading -
/// see the file header.
struct MeterSample {
    uint8_t index;
    uint32_t value;
};

/**
 * The AO: one periodic tick, one pass over the sources in pack order,
 * one MeterSample published per source that had something to say.
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
     * Arm the pace. The sources must already be fed by their ISR glue
     * (the sampler configures no hardware - it does not know any).
     *
     * The period is an argument because a sampler without a pace does
     * nothing, and it is DEFAULTED because the kernel's AO contract
     * calls init() with none: Kernel::init_all() therefore leaves the
     * sampler quiet, and the application arms it right after with
     * init(period) or start_every(period) - the same order in which it
     * configures the hardware the sources read.
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

    /// Re-pace, stop, and the readback.
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
