/*
 * serial_port.hpp
 *
 * SerialPort: the active object that turns a byte transport into LINE
 * events - bytes at high rate below (ISR + ring, lock-free), events at
 * low rate above. Pure logic over the transport template parameter:
 * host-testable with a fake transport, target-agnostic (layering rule).
 *
 * Reception pipeline:
 *   ISR:      byte -> RX ring (always, lock-free); on the ring's
 *             empty -> non-empty EDGE the app glue posts RxActivity{}
 *             (see Uart::rxc()'s return value)
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
 * The contract assumes a byte stream with an "RX went non-empty" edge
 * from the ISR; a DMA/FIFO transport may change it (docs/design/
 * overview.md, "Authority of util/").
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

    static void init() { Base::start(&running); }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Line-assembly overflow count (lines longer than max_line).
    static uint8_t line_overflows() {
        return static_cast<uint8_t>(assembler_[0].overflow_count() +
                                    assembler_[1].overflow_count());
    }

    /// Lines dropped because a run they were read from was written over
    /// while it was read - the transport refused it at its release (a
    /// HardwareRing behind it): the lines completed from that run, and the
    /// line begun in the assemblers when it came. Always zero over a
    /// transport whose release cannot refuse.
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
                    break;
                }
                const uint8_t* const first = run.data();
                const uint8_t* const end = first + run.size();
                const uint8_t* next = first;
                if constexpr (release_can_refuse) {
                    if (resync_) {
                        next = skip_to_line_end(next, end);
                    }
                }
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

    /// The bytes before the next end of line, and the end of line itself,
    /// skipped; the line after it is the first whole one.
    static const uint8_t* skip_to_line_end(const uint8_t* next, const uint8_t* end) {
        while (next != end) {
            if (*next++ == '\n') {
                resync_ = false;
                break;
            }
        }
        return next;
    }

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
};

} // namespace brio
