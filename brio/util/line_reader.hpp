/*
 * line_reader.hpp
 *
 * LineReader: the active object that turns the bytes a serial transport
 * receives into complete lines, each delivered as one event to another
 * active object of the program, the line sink. Below it bytes arrive at
 * the wire's rate, into a ring an interrupt handler or a receive engine
 * fills; above it events arrive at a line's rate. LineReader is pure
 * logic over its Transport template parameter: it compiles against any
 * type that meets util/stream.hpp's ByteSource, so it runs on every
 * target unchanged and the host suite drives it with a scripted
 * transport (test/test_line_reader). The stack it belongs to is
 * docs/design/serial.md, "LineReader".
 *
 * The smallest use is a console. The program names a transport of its
 * family, a LineReader over it with the active object that handles the
 * lines, lists the sink before the reader, and posts RxActivity from the
 * transport's interrupt handler:
 *
 *     using Lines = brio::LineReader<Serial, P, Shell>;   // Shell handles LineReceived
 *     brio::Tenuto<P, Shell, Lines>::run();              // the sink before the reader
 *
 *     // in the program's handler for the transport's vector:
 *     if (Serial::isr()) { brio::post<Lines>(brio::RxActivity{}); }
 *
 * Who calls what, and from where:
 *  - the transport's interrupt handler moves each byte into the receive
 *    ring (an interrupt receiver's ring, or a receive engine's) and
 *    returns true when the ring holds bytes the consumer
 *    has not been told of; the program's glue posts one RxActivity on
 *    that true (docs/design/serial.md, "The burst edge comes from a
 *    vector"). RxActivity is the one event LineReader handles;
 *  - LineReader runs in the kernel loop, in main context like every
 *    active object: its dispatch drains the ring into a LineAssembler
 *    (util/proto/line_parser.hpp) and posts each completed line to
 *    LineSink as LineReceived{line}. The event carries a pointer to the
 *    line, never the line itself: the text, up to max_line - 1
 *    characters (max_line is 80 by default) and a NUL, stays in the
 *    assembler's buffer and never travels through a queue;
 *  - LineSink reads or parses the line inside its own dispatch, the one
 *    that handles LineReceived. LineReader itself waits for nothing and
 *    masks nothing (a transport's ring may take the platform's critical
 *    section on its guarded path, util/ring.hpp); the console's one wait
 *    is print, below.
 *
 * Who owns a line, and for how long. Two LineAssemblers alternate: a
 * completed line stays untouched in its assembler's buffer while the
 * other one assembles the next line, and the line after that is
 * assembled into the first buffer again. The line is lent to the sink
 * for one dispatch, a Lease::dispatch loan (kernel/borrowed.hpp): the
 * sink may read it and may write it (tokenizing it in place is the
 * point of the loan) while handling LineReceived, and must not keep the
 * pointer for a later event, because LineReader's next dispatch may
 * already be assembling into that buffer again. With both lines in
 * flight LineReader stops draining (the ring absorbs the bytes; that is
 * its job) and posts RxActivity to itself: there is leftover work,
 * serve me again. Two lines can be in flight, so the sink's queue needs
 * room for two LineReceived beside its own events; a LineReceived
 * dropped by a full sink queue is a line lost, counted in the sink's
 * queue.overflows() (kernel/event_queue.hpp).
 *
 * The order the loan needs: LineSink must come before LineReader in the
 * list of active objects given to Tenuto. The kernel serves the queues
 * in list order (the position in the list is the priority, the first
 * the highest) and a dispatch runs to its end, so every LineReceived
 * posted is handled before LineReader runs again; when a LineReader
 * dispatch starts, every line it posted has been consumed and both
 * buffers are free (in_flight_ goes back to zero). LineReader declares
 * `using LendsTo = Subscribers<LineSink>` and Tenuto refuses, when the
 * program is compiled, a list in which the sink comes after the reader.
 *
 * Transmit has no active object. util/line_writer.hpp's LineWriter is
 * the one writer of a port where printers preempt one another: a print()
 * to it formats into the caller's own line and the line goes to the
 * transport whole at its end, the lines of objects that preempt one
 * another never mixed. Without one, print() (util/print.hpp) goes
 * straight to the transport, from main context only (print.hpp says
 * why), and waits until the transmit ring has taken every byte: the
 * wait is zero while the message fits the
 * ring's room, and otherwise one byte time for each byte that does not
 * yet fit. A byte is ten bit times at 8N1, 21.7 us at 460800 baud and
 * 87 us at 115200, so a message that finds the transmit ring full waits
 * the ring's room over again: about 5.5 ms at 460800 where the default
 * ring is 256 bytes (255 of room, util/ring.hpp: the AVR, the STM32 and
 * the RP families), about 1.4 ms where it is 64 (the three WCH
 * families) (docs/design/serial.md, "Output policy: whole or refused,
 * counted"). Such a print is atomic between active objects under
 * Tenuto, where a dispatch runs to completion, and not under Rubato,
 * whose preempting dispatch can print between two of its runs: the
 * reason the writer exists.
 *
 * The drain takes a run where the transport lends one. A run is a
 * contiguous stretch of bytes the ring already holds. A transport that
 * offers its receive ring's consumer half in place (util/stream.hpp's
 * SpanSource: read_span() and consume(n), the ring's own two verbs) is
 * drained a run at a time, the bytes fed to the assembler where the ring
 * holds them and released with one index store per run; any other
 * ByteSource is drained a byte at a time through read_byte(). Either way
 * the drain stops at the byte that completes the second line in flight
 * and leaves every byte after it queued, so the two paths deliver the
 * same lines in the same dispatches.
 *
 * A line is posted only from a run released clean. The lines completed
 * from a run are posted after its consume(), never before: where the
 * ring's producer is the hardware (util/ring.hpp's HardwareRing, behind
 * a DMA receive engine) the run can be written over while it is read,
 * and consume() answers false. Then the lines completed from that run
 * and the line begun when it came are dropped and counted in
 * torn_lines(), and the stream resumes after its next end of line (the
 * bytes before it end a line whose beginning the ring skipped). Over a
 * Ring, whose consume() answers nothing, the release cannot refuse and
 * none of that is compiled.
 *
 * A skip between two runs is seen. The stream a ring hands out can jump:
 * a HardwareRing skips to its producer whenever a look finds a lap
 * unread (in the drain's own read_span(), or in a count() asked between
 * two drains), and a SkipRing skips to its producer at the look after a
 * receive interrupt dropped a byte that found the ring full, or that the
 * receiver flagged as corrupt. The run handed out after such a gap is
 * the stream after it, which the line begun in an assembler would
 * otherwise swallow as its continuation. So a transport reports the gaps
 * in its stream (util/stream.hpp's SkippingSource: rx_skips(), an epoch
 * that is never cleared), the drain compares the epoch at every run,
 * over every transport that has the verb whether its release can refuse
 * or not, and a change ends the line begun as torn, counted in
 * torn_lines(), and skips the stream to its next end of line. One load
 * and one compare a run, nothing a byte. Where the ring skips at the
 * look that finds the gap (util/ring.hpp's HardwareRing and SkipRing),
 * no run handed out holds one and the epoch moves between two runs, so
 * no line is ever delivered across a gap: the line begun is torn, and
 * what the ring held when the gap was found is skipped with it. A
 * transport whose release can refuse and that does not report its skips
 * is refused at compile time; over a transport without the verb (a test
 * capture, a simulated port) nothing of it is compiled, and a byte such
 * a transport drops joins the bytes around it into one line that
 * nothing here counts (the transport's own counters say it happened).
 *
 * A silence ends the skip. A gap can take the cut line's own end of
 * line, and then the next end of line is a later, whole line's. So where
 * the drain found the ring empty after the gap and the next run comes
 * quiet_ticks (100 ms, or one tick where the tick is longer) or more
 * after that look, the line is taken to
 * have ended in the silence, and the run is drained as a line's
 * beginning. Still undecidable: the next line close behind a gap that
 * took a line's end (skipped as the cut line's tail), a sender pausing
 * mid-line longer than the silence after a gap took its end, and a gap
 * landing on a line's start (that whole line dropped with the fragment
 * expected).
 *
 * The contract assumes a byte stream whose edge is told once until the
 * ring is found empty: the transport's edge, whichever event of its
 * silicon makes it (docs/design/overview.md, "Authority of util/").
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

/// Bytes are waiting in the receive ring. Posted by the program's
/// interrupt glue when the transport's handler returns true, and by
/// LineReader to itself when it stopped a drain with bytes still queued.
struct RxActivity {};

/// A completed line, NUL-terminated and without its end of line, lent to
/// the sink for the dispatch that handles this event only. The sink may
/// write it (tokenizing it in place is the point of the loan) and must
/// not keep the pointer past that dispatch: LineReader's next dispatch
/// may assemble a line into that buffer again (the two buffers
/// alternate, the header says how).
struct LineReceived {
    Borrowed<char, Lease::dispatch> line;
};

template <typename Transport, Platform P, typename LineSink,
          uint8_t max_line = 80>
class LineReader
    : public Fsm<LineReader<Transport, P, LineSink, max_line>, RxActivity> {
    using Base = Fsm<LineReader<Transport, P, LineSink, max_line>, RxActivity>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    // Depth 2: one edge from the interrupt glue plus one self-post is the
    // steady-state worst case; a further RxActivity dropped by a full
    // queue is harmless, since the queued ones already guarantee that
    // the drain will happen.
    static inline EventQueue<Event, 2, P> queue;

    /// The line is a Lease::dispatch loan to LineSink: Tenuto refuses at
    /// compile time a list in which LineSink comes after this object.
    using LendsTo = Subscribers<LineSink>;

    /// Called by the kernel before the first event is served. Over a
    /// transport that reports its skips, the stream starts here: a gap
    /// recorded before init() tears nothing.
    static void init() {
        if constexpr (SkippingSource<Transport>) {
            // The stream starts here: a skip before it tears nothing.
            seen_skips_ = static_cast<uint32_t>(Transport::rx_skips());
            quiet_ = false;
        }
        Base::start(&running);
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Lines that did not fit an assembler's buffer (max_line - 1
    /// characters), each dropped up to its end of line and counted; a
    /// diagnostic, readable from anywhere (one byte).
    static uint8_t line_overflows() {
        return static_cast<uint8_t>(assembler_[0].overflow_count() +
                                    assembler_[1].overflow_count());
    }

    /// Lines dropped because the ring behind the transport lost bytes
    /// under them: the lines completed from a run written over while it
    /// was read, which the transport refused at its release (a
    /// HardwareRing), and the line begun in the assemblers when such a run
    /// came or when the stream had a gap between two runs (rx_skips()
    /// moved). Always zero over a transport that reports no gaps. A
    /// diagnostic: read it from main context, where it is written.
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
            post<LineReader>(RxActivity{});
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
