/*
 * serial_port.hpp
 *
 * SerialPort: the active object that turns a byte transport into LINE
 * events - bytes at high rate below (ISR + ring, lock-free), events at
 * low rate above. Pure logic over the transport template parameter:
 * host-testable with a fake transport, target-agnostic (layering rule).
 *
 * Reception pipeline:
 *   ISR:      bytes -> RX ring (an interrupt receiver's, lock-free, or a
 *             receive engine's); when the ring holds bytes the consumer
 *             has not been told of, the transport's vector returns true
 *             and the app glue posts RxActivity{} (docs/design/serial.md,
 *             "The burst edge comes from a vector")
 *   SerialPort: drains the ring, feeds a LineAssembler; each completed
 *             line is posted to LineSink as LineReceived{line} - a
 *             REFERENCE, the 80-byte payload never travels in a queue
 *   LineSink: parses/uses the line within its own dispatch
 *
 * Line buffer ownership (ping-pong): two LineAssemblers alternate; a
 * completed line stays untouched in its assembler's buffer while the
 * OTHER one assembles the next line. With both lines in flight SerialPort
 * stops draining (the ring absorbs, that is its job) and posts
 * RxActivity to ITSELF: "leftover work, reschedule me".
 *
 * SCHEDULING CONTRACT - consumer above producer: LineSink MUST precede
 * SerialPort in the Tenuto pack. The kernel then serves every posted
 * LineReceived before SerialPort runs again, so when a SerialPort dispatch
 * starts, all its previously posted lines have been consumed and both
 * buffers are free (in_flight resets). The line is a Lease::dispatch loan
 * (kernel/borrowed.hpp): the sink may read AND mutate it (in-place
 * tokenization) during its dispatch only; keeping the pointer across
 * dispatches is a bug. SerialPort declares `LendsTo = Subscribers<
 * LineSink>` and Tenuto refuses a pack that violates the order.
 *
 * TX has no AO: print() goes straight to the transport's blocking
 * push path - bounded by the wire rate (~2 ms worst case at 460800),
 * naturally atomic between AOs (run-to-completion), revisited only for
 * slow links or hard latency budgets (docs/design/serial.md).
 *
 * THE DRAIN TAKES A RUN WHERE THE TRANSPORT LENDS ONE. A transport that
 * offers its receive ring's consumer half in place (util/stream.hpp's
 * SpanSource: read_span() and consume(n), the ring's own two verbs) is
 * drained a run at a time - the bytes fed to the assembler where the
 * ring holds them and released with one index store per run; any other
 * ByteSource is drained a byte at a time through read_byte(). Either way
 * the drain stops at the byte that completes the second line in flight
 * and leaves every byte after it queued, so the two paths deliver the
 * same lines in the same dispatches.
 *
 * A LINE IS POSTED ONLY FROM A RUN RELEASED CLEAN. The lines completed
 * from a run are posted after its consume(), never before: where the
 * ring's producer is the hardware (util/ring.hpp's HardwareRing, behind
 * a DMA receive engine) the run can be written over while it is read,
 * and consume() answers false. Then the lines completed from that run
 * and the line begun when it came are dropped and counted in
 * torn_lines(), and the stream resumes after its next end of line - the
 * bytes before it end a line whose beginning the ring skipped. Over a
 * Ring, whose consume() answers nothing, the release cannot refuse and
 * none of that is compiled.
 *
 * A SKIP BETWEEN TWO RUNS IS SEEN. The stream a ring hands out can jump:
 * a HardwareRing skips to its producer whenever a look finds a lap unread
 * - in the drain's own read_span(), or in a count() asked between two
 * drains - and a receive interrupt drops a byte that finds its ring full,
 * or that the receiver flagged as corrupt. The run handed out after such
 * a gap is the stream after it, which the line begun in an assembler
 * would otherwise swallow as its continuation. So a transport reports the
 * gaps in its stream (util/stream.hpp's SkippingSource, rx_skips(), never
 * cleared), the drain compares the count at every run - over EVERY
 * transport that has it, whether its release can refuse or not - and a
 * change ends the line begun as torn, counted in torn_lines(), and skips
 * the stream to its next end of line. One load and one compare a run,
 * nothing a byte. The count moves exactly at the gap where the ring
 * knows where each one falls - a HardwareRing's skip, a GapRing's mark
 * (util/ring.hpp), which hands out no run across a gap - so the line torn
 * is the one the gap cut. A transport whose release can refuse and that
 * does not report its skips is refused at compile time; over a transport
 * without the verb (a test capture, a simulated port) nothing of it is
 * compiled.
 *
 * A SILENCE ENDS THE SKIP. A gap can take the cut line's own end of line,
 * and then the next end of line is a later, whole line's. So where the
 * drain found the ring empty after the gap and the next run comes
 * quiet_ticks (100 ms) or more after that look, the line is taken to have
 * ended in the silence, and the run is drained as a line's beginning.
 * Undecidable still: the next line close behind a gap that took a line's
 * end (skipped as the cut line's tail), a sender pausing mid-line longer
 * than the silence after a gap took its end, and a gap landing on a
 * line's start (that whole line dropped with the fragment expected).
 *
 * The contract assumes a byte stream whose edge is told once until the
 * ring is found empty - the transport's, whichever event of its silicon
 * makes it (docs/design/overview.md, "Authority of util/").
 */

#pragma once

#include <stdint.h>
#include <span>
#include <type_traits>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "util/proto/line_parser.hpp"
#include "util/stream.hpp"

namespace brio {

/// Posted (by ISR glue or by SerialPort itself) when RX bytes are pending.
struct RxActivity {};

/// A completed line, NUL-terminated, lent for the receiving dispatch
/// only (mutable: in-place tokenization is the point of the loan).
struct LineReceived {
    Borrowed<char, Lease::dispatch> line;
};

template <typename Transport, Platform P, typename LineSink,
          uint8_t max_line = 80>
class SerialPort
    : public Fsm<SerialPort<Transport, P, LineSink, max_line>, RxActivity> {
    using Base = Fsm<SerialPort<Transport, P, LineSink, max_line>, RxActivity>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // Depth 2: one external edge + one self-post is the steady-state
    // worst case; a dropped extra RxActivity is harmless (the queued
    // ones already guarantee the drain will happen).
    static inline EventQueue<Event, 2, P> queue;

    /// LineReceived is a Lease::dispatch loan: Tenuto checks LineSink
    /// precedes this AO in the pack.
    using LendsTo = Subscribers<LineSink>;

    static void init() {
        if constexpr (SkippingSource<Transport>) {
            // The stream starts here: a skip before it tears nothing.
            seen_skips_ = static_cast<uint32_t>(Transport::rx_skips());
            quiet_ = false;
        }
        Base::start(&running);
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Line-assembly overflow count (lines longer than max_line).
    static uint8_t line_overflows() {
        return static_cast<uint8_t>(assembler_[0].overflow_count() +
                                    assembler_[1].overflow_count());
    }

    /// Lines dropped because the ring behind the transport lost bytes
    /// under them: the lines completed from a run written over while it
    /// was read, which the transport refused at its release (a
    /// HardwareRing), and the line begun in the assemblers when such a run
    /// came or when the stream had a gap between two runs (rx_skips()
    /// moved). Always zero over a transport that reports no gaps.
    static uint32_t torn_lines() { return torn_lines_; }

private:
    static Status running(const Event& e) {
        if (std::holds_alternative<RxActivity>(e)) {
            // Scheduling contract: every line posted before this dispatch
            // has been consumed by the (higher-priority) sink.
            in_flight_ = 0;
            drain();
            return Base::handled();
        }
        return Base::unhandled();
    }

    static void drain() {
        if constexpr (SpanSource<Transport>) {
            static_assert(!release_can_refuse || SkippingSource<Transport>,
                          "a transport whose release can refuse (a HardwareRing "
                          "behind it) reports its ring's skips: rx_skips()");
            // A run at a time: the bytes read where the ring holds them,
            // then ONE release for every byte the assemblers took, and only
            // THEN the lines completed from them posted. A line points into
            // an assembler, never into the ring, so the release costs it
            // nothing - and the release is the run's verdict: over a ring
            // whose producer is the hardware the run can be written over
            // while it is read, and consume() says so (util/ring.hpp's
            // HardwareRing), so a line is posted only from a run released
            // clean. Over Ring the run cannot move - only this side's
            // consume() frees its slots - and the release never refuses.
            while (in_flight_ < 2) {
                const std::span<const uint8_t> run = Transport::read_span();
                if (run.empty()) {
                    if constexpr (SkippingSource<Transport>) {
                        if (resync_ || static_cast<uint32_t>(Transport::rx_skips()) != seen_skips_)
                            [[unlikely]] {
                            found_quiet();
                        }
                    }
                    break;
                }
                const uint8_t* first = run.data();
                const uint8_t* const end = first + run.size();
                if constexpr (SkippingSource<Transport>) {
                    // A gap since the last run, or a resync under way: the
                    // bytes up to the next end of line are skipped and
                    // released as a run of their own, and this run is
                    // done - the drain goes on from a fresh one, so the
                    // byte loop below starts at its run's first byte or
                    // not at all (a second back edge to the loop's head
                    // costs the Cortex-M0+ three instructions a byte).
                    const uint32_t skips = static_cast<uint32_t>(Transport::rx_skips());
                    if (skips != seen_skips_ || resync_) [[unlikely]] {
                        if (after_gap(skips, first, end)) {
                            first = end;
                        }
                    }
                }
                const uint8_t* next = first;
                const uint8_t active_at_run = active_;
                char* lines[2];
                uint8_t held = 0;
                while (next != end && in_flight_ + held < 2) {
                    // The active assembler stands until a line completes,
                    // so the inner loop holds the byte, the assembler and
                    // the end of the run, and nothing else.
                    LineAssembler<max_line>& assembler = assembler_[active_];
                    char* line;
                    do {
                        line = assembler.push(*next++);
                    } while (line == nullptr && next != end);
                    if (line != nullptr) {
                        lines[held++] = line;
                        switch_assembler();
                    }
                }
                if (release(static_cast<uint32_t>(next - first))) {
                    for (uint8_t i = 0; i < held; ++i) {
                        deliver(lines[i]);
                    }
                } else if constexpr (release_can_refuse) {
                    tear(held, active_at_run);
                }
            }
        } else {
            uint8_t byte;
            while (in_flight_ < 2 && Transport::read_byte(byte)) {
                if (char* line = assembler_[active_].push(byte)) {
                    deliver(line);
                    switch_assembler();
                }
            }
        }
        if (in_flight_ >= 2) {
            // Both buffers in flight and possibly more bytes in the ring:
            // reschedule ourselves AFTER the sink has consumed.
            post<SerialPort>(RxActivity{});
        }
    }

    /// A completed line to the sink, lent until the sink's dispatch ends.
    [[gnu::always_inline]] static void deliver(char* line) {
        post<LineSink>(LineReceived{Borrowed<char, Lease::dispatch>{line}});
        ++in_flight_;
    }

    /// The other buffer takes over the assembly, from the byte after a
    /// completed line.
    [[gnu::always_inline]] static void switch_assembler() {
        active_ = static_cast<uint8_t>(active_ ^ 1);
    }

    // ---- the run's release, and a run refused at it -------------------------

    /// Whether the transport's consume() can refuse a run: it answers a
    /// bool (a HardwareRing behind it) rather than nothing (a Ring).
    static constexpr bool release_can_refuse =
        !std::is_void_v<decltype(Transport::consume(uint32_t{}))>;

    /// Release `n` bytes of the run; true when the run was intact.
    [[gnu::always_inline]] static bool release(uint32_t n) {
        if constexpr (release_can_refuse) {
            return Transport::consume(n);
        } else {
            Transport::consume(n);
            return true;
        }
    }

    /// The stream had a gap since the last run, or the drain is still
    /// skipping to a line's end: `skips` is the epoch read for the run in
    /// hand, [first, end) the run. An epoch moved since the drain saw it
    /// last put a gap between the bytes the assemblers hold and the run:
    /// the line begun - always in the active assembler, which lends
    /// nothing - is ended and counted (seen()), and the stream is skipped
    /// to its next end of line, the bytes before it being the end of a
    /// line whose beginning the gap took. The bytes skipped - up to and
    /// with the run's first end of line, or all of it - are released here
    /// as a run of their own (a release refused tears as the drain's
    /// does), and true says this run is done: the drain goes on from a
    /// fresh one.
    ///
    /// A SILENCE ENDS THE SKIP. The gap may have taken the cut line's own
    /// end of line, and then the next one is a later, whole line's. So
    /// where the drain found the ring empty after the gap (found_quiet())
    /// and this run arrived `quiet_ticks` or more after that look, the line
    /// is taken to have ended in the silence: the skip ends, false is
    /// returned, and the run is drained as the beginning of a line.
    ///
    /// OUT OF LINE, AND THE WHOLE OF IT: the rare path. Inline - or with
    /// its release beside the drain's - its registers and its call crowd
    /// the byte loop's on the Cortex-M0+ (two to three instructions a byte,
    /// counted in the release listing); the per-run test is all a run
    /// without a gap pays.
    [[gnu::noinline]] static bool after_gap(uint32_t skips, const uint8_t* const first,
                                            const uint8_t* const end) {
        if (skips != seen_skips_) {
            seen(skips);
        } else if (quiet_ && static_cast<uint32_t>(P::now() - quiet_at_) >= quiet_ticks) {
            resync_ = false;
            quiet_ = false;
            return false;
        }
        quiet_ = false;   // bytes came: only a later empty look is a silence
        const uint8_t* next = first;
        while (next != end) {
            if (*next++ == '\n') {
                resync_ = false;
                break;
            }
        }
        if (!release(static_cast<uint32_t>(next - first))) {
            if constexpr (release_can_refuse) {
                tear(0u, active_);
            }
        }
        return true;
    }

    /// A gap seen: the epoch recorded, the line begun ended and counted,
    /// the skip to the next end of line armed.
    static void seen(uint32_t skips) {
        seen_skips_ = skips;
        torn_lines_ = torn_lines_ + end_begun_line(active_);
        resync_ = true;
    }

    /// The drain found the ring empty with a skip under way, or with a gap
    /// crossed by that very look: the gap seen now, and the time of the
    /// look kept - the start of a silence, if the next run is long coming.
    [[gnu::noinline]] static void found_quiet() {
        const uint32_t skips = static_cast<uint32_t>(Transport::rx_skips());
        if (skips != seen_skips_) {
            seen(skips);
        }
        quiet_at_ = P::now();
        quiet_ = true;
    }

    /// The silence that ends a skip: 100 ms, longer than a USB serial
    /// adapter holds a short packet back (an FTDI's default latency timer
    /// is 16 ms) or splits a burst over its frames, and shorter than a
    /// person or a script waits before the next command. One tick at
    /// least.
    static constexpr uint32_t quiet_ticks =
        P::ticks_per_second / 10u != 0u ? P::ticks_per_second / 10u : 1u;

    /// A run refused at its release: the bytes the assemblers took from it
    /// are not the stream, and the ring has skipped to its producer. So
    /// the `held` lines completed from the run are dropped, and so is the
    /// line the assemblers had begun - ended in every assembler that lends
    /// nothing, a '\n' emptying an assembler and ending an overflow's drop
    /// alike - and the stream is skipped to its next end of line, the bytes
    /// before it being the end of a line whose beginning the ring skipped.
    /// The run started on `active_at_run`, which lends nothing; the other
    /// one is lent only when a line went out before this run in the same
    /// dispatch. Counted: the lines completed and the one begun, if any.
    static void tear(uint8_t held, uint8_t active_at_run) {
        uint32_t dropped = held;
        active_ = active_at_run;
        dropped += end_begun_line(active_at_run);
        if (in_flight_ == 0u) {
            dropped += end_begun_line(static_cast<uint8_t>(active_at_run ^ 1));
        }
        torn_lines_ = torn_lines_ + dropped;
        resync_ = true;
    }

    /// Ends what an assembler holds; 1 when that was a line begun - a
    /// character at least, and not an overflow's drop, already counted.
    static uint32_t end_begun_line(uint8_t which) {
        const char* const line = assembler_[which].push('\n');
        return line != nullptr && line[0] != '\0' ? 1u : 0u;
    }

    static inline LineAssembler<max_line> assembler_[2]{};
    static inline uint8_t active_ = 0;
    static inline uint8_t in_flight_ = 0;
    static inline bool resync_ = false;        // skip to the next end of line
    static inline uint32_t torn_lines_ = 0;
    static inline uint32_t seen_skips_ = 0;    // the ring's skip epoch, as last seen
    static inline bool quiet_ = false;         // the ring found empty during a skip ...
    static inline uint32_t quiet_at_ = 0;      // ... at this tick
};

} // namespace brio
