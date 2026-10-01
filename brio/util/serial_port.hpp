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
 * offers its receive ring's consumer half in place (SpanSource below:
 * read_span() and consume(n), the ring's own two verbs) is drained a run
 * at a time - the bytes fed to the assembler where the ring holds them
 * and released with one index store per run; any other ByteSource is
 * drained a byte at a time through read_byte(). Either way the drain
 * stops at the byte that completes the second line in flight and leaves
 * every byte after it queued, so the two paths deliver the same lines in
 * the same dispatches.
 *
 * The contract assumes a byte stream with an "RX went non-empty" edge
 * from the ISR; a DMA/FIFO transport may change it (docs/design/
 * overview.md, "Authority of util/").
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <span>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "util/proto/line_parser.hpp"

namespace brio {

/// Posted (by ISR glue or by SerialPort itself) when RX bytes are pending.
struct RxActivity {};

/// A completed line, NUL-terminated, lent for the receiving dispatch
/// only (mutable: in-place tokenization is the point of the loan).
struct LineReceived {
    Borrowed<char, Lease::dispatch> line;
};

/// A source that lends its received bytes IN PLACE: read_span() is the
/// contiguous run ready to be read - it never wraps, the rest of a
/// wrapped ring coming on the next call - and consume(n) releases the
/// first n of it, oldest first, clamped to what is queued. These are the
/// consumer half of util/ring.hpp under the ring's own names, and the
/// ring's rules hold: consumer side only, a run valid until the
/// consumer's next operation on the source.
template <typename S>
concept SpanSource = requires(uint32_t n) {
    { S::read_span() } -> std::convertible_to<std::span<const uint8_t>>;
    S::consume(n);
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
            // then ONE release for every byte the assemblers took. The
            // run cannot move under the loop - only this side's consume()
            // frees its slots - and a line posted from it points into an
            // assembler, never into the ring.
            while (in_flight_ < 2) {
                const std::span<const uint8_t> run = Transport::read_span();
                if (run.empty()) {
                    break;
                }
                const uint8_t* const first = run.data();
                const uint8_t* const end = first + run.size();
                const uint8_t* next = first;
                while (next != end && in_flight_ < 2) {
                    // The active assembler stands until a line completes,
                    // so the inner loop holds the byte, the assembler and
                    // the end of the run, and nothing else.
                    LineAssembler<max_line>& assembler = assembler_[active_];
                    char* line;
                    do {
                        line = assembler.push(*next++);
                    } while (line == nullptr && next != end);
                    if (line != nullptr) {
                        deliver(line);
                    }
                }
                Transport::consume(static_cast<uint32_t>(next - first));
            }
        } else {
            uint8_t byte;
            while (in_flight_ < 2 && Transport::read_byte(byte)) {
                if (char* line = assembler_[active_].push(byte)) {
                    deliver(line);
                }
            }
        }
        if (in_flight_ >= 2) {
            // Both buffers in flight and possibly more bytes in the ring:
            // reschedule ourselves AFTER the sink has consumed.
            post<SerialPort>(RxActivity{});
        }
    }

    /// A completed line to the sink; the other buffer takes over.
    [[gnu::always_inline]] static void deliver(char* line) {
        post<LineSink>(LineReceived{Borrowed<char, Lease::dispatch>{line}});
        ++in_flight_;
        active_ = static_cast<uint8_t>(active_ ^ 1);
    }

    static inline LineAssembler<max_line> assembler_[2]{};
    static inline uint8_t active_ = 0;
    static inline uint8_t in_flight_ = 0;
};

} // namespace brio
