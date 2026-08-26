/*
 * trace.hpp (util)
 *
 * Trace: a ring of timestamped marks, written from anywhere and read
 * back afterwards. The bench technique every campaign reinvented,
 * promoted to a service.
 *
 * WHAT IT IS FOR. On a machine with one console and no debugger
 * attached, the two things a program cannot do while something
 * interesting happens are PRINT and STOP. Printing from inside an
 * interrupt changes the timing it was meant to observe (a 460800-baud
 * byte is thousands of cycles); a breakpoint destroys the timing
 * altogether. What survives contact with a live system is a mark: a
 * timestamp, a small label, one number - stored in a few cycles into
 * RAM, and printed later at leisure. That is the whole of this file.
 *
 * WHAT IT IS NOT. Not a log: the ring keeps the LAST N marks and
 * overwrites the oldest, because the interesting end of a run is the
 * end. count() says how many were ever stamped, so a dump that shows N
 * records out of thousands says so instead of pretending it is
 * complete. Not a profiler either: what a mark costs is a critical
 * section and a handful of stores, which is small but not nothing, and
 * a mark inside the hot path of a measurement is part of the
 * measurement.
 *
 * THE DISABLED SPECIALIZATION IS THE POINT. `Trace<N, P, false>` has no
 * storage and no code: every verb is an empty inline and the object is
 * EMPTY (std::is_empty_v), so instrumentation can be left in the source
 * and compiled out of the image by one bool - which is what makes it
 * worth writing marks in the first place. A tracing build and a
 * shipping build then differ in one template argument, not in a patch.
 *
 * THE OBJECT IS THE TRACE. Unlike a driver resource, a trace has state
 * worth several hundred bytes and a program may want more than one
 * (one per subsystem, at different sizes). So it is an ordinary object,
 * usually a file-scope one, and the ISR that stamps into it names it:
 *
 *     brio::Trace<64, P> trace;
 *     ISR(TCB0_INT_vect) { trace.stamp(tag_capture, Meter::period_ticks()); }
 *     ...
 *     trace.dump(serial);
 *
 * TAGS ARE THE APP'S. A tag is a byte, and what it means is written
 * where the marks are - an enum in the app, printed as a number here.
 * Nothing in a service can name an application's events, and a table of
 * strings would cost more flash than the ring costs RAM.
 *
 * Validated on: AVR DA/DB and the host. Nothing here is target-specific
 * beyond the platform's clock and critical section.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"
#include "util/print.hpp"
#include "util/stream.hpp"

namespace brio {

/// One mark: when, what, and one number of the app's choosing.
struct TraceRecord {
    uint32_t t;      ///< P::now() at the stamp
    uint16_t arg;    ///< the app's number (0 when not given)
    uint8_t tag;     ///< the app's label
};

/**
 * The ring. `N` is how many marks it keeps; `enabled` false compiles the
 * whole thing away (see the file header).
 *
 * stamp() is safe from an interrupt and from the loop: the index update
 * runs under the platform's critical section, so two writers cannot
 * claim the same slot. Reading (dump, at()) is main-context: a dump
 * taken while stamps are still arriving is a snapshot of a moving
 * target and says so by its own timestamps.
 */
template <uint8_t N, Platform P, bool enabled = true>
class Trace {
    static_assert(N > 0, "Trace: a ring of nothing keeps nothing");

public:
    static constexpr uint8_t capacity = N;

    /// A mark with no number.
    void stamp(uint8_t tag) { stamp(tag, 0); }

    /// A mark with the app's number.
    void stamp(uint8_t tag, uint16_t arg) {
        const uint32_t t = P::now();
        typename P::CriticalSection cs;
        records_[next_] = TraceRecord{t, arg, tag};
        if (++next_ == N) {
            next_ = 0;
        }
        if (held_ < N) {
            ++held_;
        }
        if (count_ != UINT32_MAX) {
            ++count_;
        }
    }

    /// Forget the marks - and the count, which is a count of what this
    /// ring has seen since it was last cleared.
    void clear() {
        typename P::CriticalSection cs;
        next_ = 0;
        held_ = 0;
        count_ = 0;
    }

    /// How many marks were EVER stamped (saturating): the ring holds at
    /// most N of them, and this is how one knows the difference.
    uint32_t count() const { return count_; }

    /// How many the ring is holding right now: N once it has wrapped.
    uint8_t held() const { return held_; }

    /// The i-th mark held, oldest first. i >= held() is a zero record.
    TraceRecord at(uint8_t i) const {
        if (i >= held_) {
            return TraceRecord{};
        }
        // Widened on purpose: the sum reaches 2N - 2, which a uint8_t
        // would wrap for a ring bigger than 128.
        uint16_t slot = static_cast<uint16_t>(next_ + N - held_ + i);
        if (slot >= N) {
            slot = static_cast<uint16_t>(slot - N);
        }
        return records_[slot];
    }

    /// One line per mark, OLDEST FIRST, closing with what was dropped.
    /// The sink is any ByteSink, so a host test reads the same bytes a
    /// console would.
    template <ByteSink S>
    void dump(S sink) const {
        const uint8_t n = held_;
        for (uint8_t i = 0; i < n; ++i) {
            const TraceRecord r = at(i);
            print(sink, "t ", r.t, " tag ", r.tag, " arg ", r.arg, crlf);
        }
        print(sink, "-- ", n, " of ", count_, " stamps", crlf);
    }

private:
    TraceRecord records_[N]{};
    uint32_t count_ = 0;
    uint8_t next_ = 0;
    uint8_t held_ = 0;
};

/**
 * The compiled-out trace: same surface, no storage, no code. Every verb
 * is an empty inline the optimizer removes together with whatever the
 * caller computed only to pass in - so `trace.stamp(tag, expensive())`
 * costs nothing but the call to expensive(), which is the app's to
 * guard if it is not free.
 */
template <uint8_t N, Platform P>
class Trace<N, P, false> {
public:
    static constexpr uint8_t capacity = N;

    void stamp(uint8_t) {}
    void stamp(uint8_t, uint16_t) {}
    void clear() {}
    uint32_t count() const { return 0; }
    uint8_t held() const { return 0; }
    TraceRecord at(uint8_t) const { return TraceRecord{}; }

    template <ByteSink S>
    void dump(S) const {}
};

} // namespace brio
