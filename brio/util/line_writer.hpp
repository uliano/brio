/*
 * line_writer.hpp
 *
 * LineWriter: the active object that is the one writer of a transport.
 * Every other part of the program prints through it: a print() call names
 * the writer instead of the transport, its argument list travels to the
 * writer as one event, unformatted, and the writer formats it in its own
 * dispatch, straight into the transport's transmit ring, with the same
 * print_one() verbs print() uses on a transport (util/print.hpp). The
 * bytes that reach the wire for a print() are the bytes print() writes
 * to the transport itself. What the writer adds is that the lines of
 * different callers never mix, whichever preempts which, and that a
 * caller never waits on the wire. LineWriter is pure logic over its
 * Transport template parameter, the twin of util/line_reader.hpp's
 * LineReader, and the host suite drives it over two simulated transports
 * (test/test_line_writer). The stack it belongs to is
 * docs/design/serial.md, "LineWriter".
 *
 * The smallest use names the writer over a transport of the family,
 * lists it LAST among the program's active objects, posts TxActivity from
 * the transport's interrupt glue, and prints to it:
 *
 *     using Log = brio::LineWriter<Serial, P>;           // depth 8, 32-byte messages
 *     brio::Tenuto<P, App, Log>::run();                  // the writer last
 *
 *     // in the program's handler for the transport's vector:
 *     if (Serial::tx_drained()) { brio::post<Log>(brio::TxActivity{}); }
 *
 *     // anywhere:
 *     brio::print(Log{}, "count = ", count, brio::crlf);
 *
 * THE MESSAGE. A print() to a writer packs its arguments as they are into
 * one Message, the writer's event: a text (a `const char*` or a
 * std::string_view) as its pointer and its length, measured at the call;
 * every other value by value; `crlf` as the last argument as a flag, the
 * line's end. With them go a pointer to the format function the compiler
 * instantiates for that list of types, the caller's SOURCE (the index of
 * the active object being dispatched, kernel/active_object.hpp's
 * dispatching<P>, or `handler_source` from an interrupt handler) and the
 * message's BOUND: the most bytes it can produce, the measured length of
 * its texts plus each other argument's worst case (print_bound below: a
 * uint32_t is ten bytes, a hex() ten, a fixed() 59). A list whose packed
 * bytes do not fit the message (`slot` bytes, the header included), or
 * whose bound the transport's ring could never hold, does not compile; a
 * text that makes the bound pass 255 or the ring is refused at the call
 * and counted. The texts are the caller's: the pointer is kept, never the
 * characters, so a text must still hold what it held when the writer
 * dispatches - a literal, a constant, a buffer that outlives the line. It
 * is the caller's duty, and in a debug build a text on the stack panics
 * at the call where the platform tells its stack (`P::stack_span()`, an
 * optional member).
 *
 * LINES IN PIECES. A line may be printed in several calls, the last one
 * ending with crlf. Each call is a message and a PIECE of the line; the
 * writer holds the pieces of each source in their queue slots, chained
 * by slot number (kernel/event_queue.hpp's hold()), and writes the line
 * only when its last piece arrives, the pieces in order. So a line from
 * an active object that a higher one preempts, or that an interrupt
 * handler's line lands in the middle of, still reaches the wire whole,
 * after the line that overtook it. The end of a line is the TYPE crlf_t
 * as the last argument, never a byte: a "\n" inside a text is text, and a
 * crlf that is not the last argument is written as the two bytes it is.
 * `flush_line` as the last argument ends a line too, closed by nothing:
 * the shape of a prompt, which must leave before its line ends.
 *
 * Who calls what, and from where:
 *  - print(Writer{}, ...) from any context. It never waits but in the
 *    serving context (below), and masks interrupts only for the queue's
 *    own push and for the counters below; it returns false when the
 *    message was refused. From an interrupt
 *    handler a message must end its line (a handler's piece has no
 *    source that will reliably end it): a piece from a handler is
 *    refused. ONE context may also SERVE the writer: from it (by default
 *    main context outside every dispatch, `dispatching<P> == no_dispatch`;
 *    the `inline_from` option names another) print() posts and then runs
 *    the writer's dispatches itself until its queue is empty and every
 *    complete line is in the ring, polling the ring's room when a line
 *    does not fit yet - print() to a transport, waiting as it waits, in
 *    the writer's dress. Only one context may, because two would run one
 *    dispatch inside the other; on a preemptive platform the serving
 *    context holds the ceiling at the writer's own index meanwhile, so
 *    the kernel does not start the writer over it;
 *  - the program's interrupt glue posts TxActivity when the transport's
 *    tx_drained() answers true (util/stream.hpp's EdgeSink), as it posts
 *    RxActivity for a LineReader;
 *  - the writer's dispatch, in main context: it chains the message's
 *    slot to its source's open line, moves a line whose end arrived to
 *    the queue of complete lines, then DRAINS: each complete line's
 *    messages are formatted into the ring in order, a message only when
 *    the ring's room holds its bound (and the line's closing, for its
 *    last), each slot released as its message is written. A message that
 *    does not fit stops the drain and counts a wait (once per edge waited
 *    for); the transmit edge's TxActivity brings the next dispatch. The
 *    dispatch formats and copies and never waits on the wire.
 *
 * Refusals and tears. A message is refused when the queue has one free
 * slot or none (the last is kept for a TxActivity, so that the edge, and
 * the wake a tear posts, always find a place), when its bound is too
 * large, or when it is a piece from a handler; nobody waits for a slot,
 * so the queue cannot lock whatever the callers do. A whole line refused
 * is a line LOST. A piece refused TEARS its line: the caller's further
 * pieces of that line are refused until its end passes (not posted
 * either), and the pieces the writer holds are released unwritten at its
 * next dispatch. Both are counted in lines, never retried: lost() counts
 * every line lost or torn, torn_lines() the torn among them, waits() the
 * transmit edges the writer waited for. And the wire shows where lines are
 * missing: after a loss the writer writes a line of its own, "lost N",
 * the losses since the last such mark, once the lines before it are out.
 * The queue must be deeper than the longest line in pieces, and the ring
 * longer than the largest message bound; a program sizes both by these
 * counters (docs/design/serial.md has the procedure).
 *
 * The one-writer rule. init() claims the transport (util/stream.hpp's
 * WriterClaim): in a debug build any write to it from anywhere but the
 * writer panics, at the first such write; a release build compiles none
 * of it. What stays outside the writer is what runs outside the kernel:
 * a banner main() prints before the writer's init(), and the panic path,
 * which drains the writer into the transport itself (drain_to_port() and
 * LineWriterReporter below).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <atomic>
#include <bit>
#include <concepts>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#include "kernel/active_object.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "util/print.hpp"
#include "util/stream.hpp"

namespace brio {

/// The transport ran its transmit ring dry: there is room again. Posted
/// by the program's interrupt glue when the transport's tx_drained()
/// answers true.
struct TxActivity {};

/// The source a message from an interrupt handler carries, beside the
/// pack indices and no_dispatch (kernel/active_object.hpp).
inline constexpr uint8_t handler_source = 254;

/// The interpretation codes: how the writer closes a line. A text line
/// (console text, the bench grammar) is closed with CRLF; a bare one
/// (a line ended by flush_line) with nothing.
inline constexpr uint8_t code_text = 0;
inline constexpr uint8_t code_bare = 1;

/// The bytes the closing of a line of `code` writes.
constexpr uint8_t closing_bytes(uint8_t code) { return code == code_text ? 2u : 0u; }

/// What the writer's options name beyond its depth and its slot.
///  - max_sources: the entries of the per-source table - the pack's
///    objects, main context and the handlers - and so the pack size it
///    serves: indices from 0 to max_sources - 3, main context and the
///    handlers in the last two. A message from an index past them is
///    refused and counted;
///  - inline_from: the one context that serves the writer inline (the
///    header): no_dispatch, main context outside every dispatch, or the
///    pack index of an object whose dispatch runs main-context work (a
///    bench console);
///  - measured: count the bytes written and the bound they were written
///    under (bytes_written(), bound_written()), at one add a run.
struct LineWriterOptions {
    uint8_t max_sources = 16;
    uint8_t inline_from = no_dispatch;
    bool measured = false;
};

// ---- the bound -----------------------------------------------------------------

/// The extension point of the bound: a program that prints a type of its
/// own (its print_one overload, util/print.hpp) states the most bytes one
/// value prints, as `static constexpr uint16_t value`.
template <typename T>
struct PrintBound;

/// The most bytes one value of T prints, for every type print.hpp
/// prints: a char and a bool one, an integer the digits of its type's
/// extreme with the sign, hex() 10 and its 64-bit form 18, fixed() the
/// widest field it is kept to (print_fixed_text less the NUL), a float
/// or sci() fifteen, a TimeStamp fifteen (ten digits of seconds, the
/// point, three of millis, the 's'). A text's bound is its length, known
/// at the call only, so 0 here.
template <typename T>
constexpr uint16_t print_bound() {
    if constexpr (std::is_same_v<T, char> || std::is_same_v<T, bool>) {
        return 1;
    } else if constexpr (std::is_integral_v<T>) {
        constexpr uint16_t unsigned_digits[] = {0, 3, 5, 0, 10, 0, 0, 0, 20};
        constexpr uint16_t signed_digits[] = {0, 4, 6, 0, 11, 0, 0, 0, 20};
        static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8);
        return std::is_signed_v<T> ? signed_digits[sizeof(T)] : unsigned_digits[sizeof(T)];
    } else if constexpr (std::is_same_v<T, Hex>) {
        return 10;
    } else if constexpr (std::is_same_v<T, Hex64>) {
        return 18;
    } else if constexpr (std::is_same_v<T, Fixed<float>> || std::is_same_v<T, Fixed<double>>) {
        return print_fixed_text - 1u;
    } else if constexpr (std::is_same_v<T, Sci<float>> || std::is_same_v<T, Sci<double>> ||
                         std::is_floating_point_v<T>) {
        return 15;
    } else if constexpr (std::is_same_v<T, TimeStamp>) {
        return 15;
    } else if constexpr (std::is_same_v<T, crlf_t> || std::is_same_v<T, flush_line_t>) {
        return 0;
    } else if constexpr (std::is_same_v<T, const char*> || std::is_same_v<T, char*> ||
                         std::is_same_v<T, std::string_view>) {
        return 0;
    } else {
        static_assert(requires { PrintBound<T>::value; },
                      "print to a LineWriter: this type has no bound - state the most bytes "
                      "one value prints in brio::PrintBound<T>::value");
        return PrintBound<T>::value;
    }
}

namespace line_writer_detail {

template <typename T>
inline constexpr bool is_text =
    std::is_same_v<T, const char*> || std::is_same_v<T, char*> || std::is_same_v<T, std::string_view>;

template <typename T>
inline constexpr bool is_end = std::is_same_v<T, crlf_t> || std::is_same_v<T, flush_line_t>;

/// A text argument's length, measured at the call (a literal's folded by
/// the compiler); 0 for every other argument.
template <typename A>
[[gnu::always_inline]] inline size_t text_length(const A& arg) {
    using T = std::decay_t<A>;
    if constexpr (std::is_same_v<T, std::string_view>) {
        return arg.size();
    } else if constexpr (std::is_same_v<T, const char*> || std::is_same_v<T, char*>) {
        return __builtin_strlen(arg);
    } else {
        return 0;
    }
}

/// The bytes one argument packs into: a text its pointer and a length
/// byte, an end of line nothing, every other value itself.
template <typename T>
constexpr uint16_t packed_size() {
    if constexpr (is_end<T>) {
        return 0;
    } else if constexpr (is_text<T>) {
        return sizeof(const char*) + 1u;
    } else {
        static_assert(std::is_trivially_copyable_v<T>,
                      "print to a LineWriter: a value travels by value in the message, so its "
                      "type must be trivially copyable");
        return sizeof(T);
    }
}

/// Where each argument starts in the packed bytes, and where they end.
template <size_t n>
struct Offsets {
    uint16_t at[n + 1];
};

/// The facts of one argument list, all known when it is compiled.
template <typename... Ts>
struct Layout {
    static constexpr size_t count = sizeof...(Ts);
    using Last = typename decltype((std::type_identity<Ts>{}, ...))::type;
    /// The list ends its line: its last argument is crlf or flush_line.
    static constexpr bool ends = is_end<Last>;
    static constexpr uint8_t code = std::is_same_v<Last, flush_line_t> ? code_bare : code_text;

    static constexpr Offsets<count> offsets = [] {
        Offsets<count> o{};
        uint16_t at = 0;
        size_t i = 0;
        ((o.at[i++] = at, at = static_cast<uint16_t>(at + packed_size<Ts>())), ...);
        o.at[count] = at;
        return o;
    }();
    static constexpr uint16_t size = offsets.at[count];

    /// The bound of everything but the texts: a crlf that is not the last
    /// argument is two bytes written, the last one the line's closing.
    static constexpr uint16_t static_bound = [] {
        uint16_t b = 0;
        size_t i = 0;
        ((b = static_cast<uint16_t>(b + (std::is_same_v<Ts, crlf_t> ? (i + 1 == count ? 0u : 2u)
                                                                    : print_bound<Ts>())),
          ++i),
         ...);
        return b;
    }();
};

/// A value back out of the packed bytes, which are not aligned.
template <typename T>
[[gnu::always_inline]] inline T unpack(const uint8_t* p) {
    struct Raw {
        uint8_t b[sizeof(T)];
    } raw;
    __builtin_memcpy(raw.b, p, sizeof(T));
    return std::bit_cast<T>(raw);
}

/// One argument out of the packed bytes and onto the sink, through the
/// print_one() overload print() would call for it. A text goes as one
/// run, its length known; a crlf that is not the last argument as its
/// two bytes; the last one, the line's end, is the writer's to close.
template <typename Sink, typename T, bool last>
[[gnu::always_inline]] inline void emit(const uint8_t* p) {
    if constexpr (std::is_same_v<T, crlf_t>) {
        if constexpr (!last) {
            print_one(Sink{}, crlf);
        }
    } else if constexpr (std::is_same_v<T, flush_line_t>) {
    } else if constexpr (is_text<T>) {
        const char* const text = unpack<const char*>(p);
        print_one(Sink{}, std::string_view(text, p[sizeof(const char*)]));
    } else {
        const T value = unpack<T>(p);
        print_one(Sink{}, value);
    }
}

template <typename Sink, typename... Ts, size_t... I>
[[gnu::always_inline]] inline void format_in_order(const uint8_t* packed, std::index_sequence<I...>) {
    using L = Layout<Ts...>;
    (emit<Sink, Ts, I + 1 == sizeof...(Ts)>(packed + L::offsets.at[I]), ...);
}

/// The format function of one argument list: what a Message points at.
template <typename Sink, typename... Ts>
void format(const uint8_t* packed) {
    format_in_order<Sink, Ts...>(packed, std::index_sequence_for<Ts...>{});
}

/// The writer's sink when it is measured: the transport, with the bytes
/// it takes counted.
template <typename Transport>
struct CountingSink {
    static inline uint32_t bytes = 0;

    static bool write_byte(uint8_t b) {
        if (!Transport::write_byte(b)) {
            return false;
        }
        ++bytes;
        return true;
    }
    static uint32_t write_bulk(std::span<const uint8_t> run)
        requires BulkSink<Transport>
    {
        const uint32_t n = Transport::write_bulk(run);
        bytes += n;
        return n;
    }
};

/// A byte another context writes: read fresh.
[[gnu::always_inline]] inline uint8_t load_shared(const uint8_t& b) {
    return *const_cast<const volatile uint8_t*>(&b);
}

} // namespace line_writer_detail

/// The writer's event: one print() call's arguments, packed (the header,
/// "The message"). `slot` bytes, the header's four bytes and the format
/// function's pointer included.
template <uint8_t slot>
struct LineMessage {
    using Format = void (*)(const uint8_t*);
    static constexpr uint8_t header =
        static_cast<uint8_t>((4u + alignof(Format) - 1u) / alignof(Format) * alignof(Format) + sizeof(Format));
    static_assert(slot > header, "a LineWriter's slot holds its message's header and some arguments");
    /// The bytes of arguments a message holds.
    static constexpr uint8_t room = static_cast<uint8_t>(slot - header);

    uint8_t source;   ///< dispatching<P>, no_dispatch, or handler_source
    uint8_t code;     ///< how the line is closed: code_text, code_bare
    uint8_t bound;    ///< the most bytes format() writes
    uint8_t flags;    ///< bit 0: the line ends here; bits 1..7: the source's tear epoch
    Format format;
    uint8_t packed[room];
};

template <typename Transport, Platform P, uint8_t depth = 8, uint8_t slot = 32,
          LineWriterOptions opts = LineWriterOptions{}>
class LineWriter {
    static_assert(EdgeSink<Transport>,
                  "a LineWriter's transport tells its room and its transmit edge: tx_room() "
                  "and tx_drained() (util/stream.hpp)");
    static_assert(requires { { Transport::tx_capacity() } -> std::convertible_to<uint32_t>; },
                  "a LineWriter's transport tells its transmit ring's capacity: tx_capacity()");
    static_assert(HandlerAware<P>,
                  "a LineWriter tells a handler's message from a dispatch's: P::in_handler() "
                  "(kernel/platform.hpp)");
    static_assert(depth >= 2, "a LineWriter's queue keeps one slot for the transmit edge");
    static_assert(opts.max_sources >= 3 && opts.max_sources <= 128,
                  "the source table holds main context, the handlers and at least one object");

    using Sink = std::conditional_t<opts.measured, line_writer_detail::CountingSink<Transport>, Transport>;

public:
    using Message = LineMessage<slot>;
    using Event = std::variant<Message, TxActivity>;
    static inline EventQueue<Event, depth, P> queue;

    /// print()'s mark: arguments for this type travel as a message.
    static constexpr bool takes_messages = true;

    /// The why of a debug panic (PanicCode::assert_failed's context).
    static constexpr uint8_t misuse_stack_text = 0xA1;   ///< a text on the stack
    static constexpr uint8_t misuse_direct_write = 0xA2; ///< a write to the claimed transport

    /// The default of on_misuse below: panic<P>(), the why its context.
    static void panic_on_misuse(uint8_t why) { panic<P>(PanicCode::assert_failed, why); }

    /// What a debug check calls on a misuse: panic_on_misuse() by default;
    /// a host test may point it at a recorder. Debug builds only.
    static inline void (*on_misuse)(uint8_t why) = &panic_on_misuse;

    /// Called by the kernel before the first event is served: claims the
    /// transport (a debug build checks the claim) and starts with no line
    /// open and nothing complete. A full reset: the slots a previous life
    /// holds go back to the queue unwritten; messages still waiting in the
    /// queue are kept and served.
    static void init() {
        for (uint8_t s = 0; s < opts.max_sources; ++s) {
            release_chain(head_[s]);
            head_[s] = 0;
            epoch_[s] = line_writer_detail::load_shared(tears_[s]);
        }
        while (completed_count_ != 0u) {
            release_chain(completed_[completed_head_]);
            pop_completed();
        }
        seen_tears_ = line_writer_detail::load_shared(tear_events_);
        if constexpr (debug_checks) {
            WriterClaim<Transport>::violated = &direct_write;
            WriterClaim<Transport>::claimed = true;
        }
    }

    /// The writer's dispatch, the kernel's or the inline service's call:
    /// the tears seen, the message chained, the complete lines drained.
    static void dispatch(const Event& e) {
        sweep_tears();
        match(e, [](const Message& m) { on_message(m); }, [](TxActivity) {});
        drain();
    }

    /// The client's verb behind print(Writer{}, ...): pack, bound, post,
    /// and, from the serving context, serve. True when the message was
    /// taken; false when it was refused (counted in lost(), the header's
    /// "Refusals and tears"). Any context; never waits but in the serving
    /// context, where it waits as print() on a transport waits.
    template <typename... Args>
    [[gnu::always_inline]] static bool post_message(const Args&... args) {
        static_assert(sizeof...(Args) > 0, "print to a LineWriter: nothing to print");
        using L = line_writer_detail::Layout<std::decay_t<const Args>...>;
        static_assert(L::size <= Message::room,
                      "print to a LineWriter: these arguments do not fit one message - split the "
                      "line into several prints (the pieces of one line), or raise the writer's slot");
        static_assert(L::static_bound <= limit(L::ends, L::code),
                      "print to a LineWriter: this line's bound exceeds what the transport's ring "
                      "holds (or 255): it would never fit - split it, or lengthen the ring");
        // The texts' lengths at the call, where the compiler folds a
        // literal's; the rest out of line, so that a dispatch that prints
        // carries no message in its own frame.
        const size_t lengths[] = {line_writer_detail::text_length(args)...};
        return pack_and_post<std::decay_t<const Args>...>(lengths, static_cast<std::decay_t<const Args>>(args)...);
    }

    /// Lines lost - refused whole, or torn - since the program started;
    /// saturating. Any context.
    static uint16_t lost() {
        typename P::CriticalSection cs;
        return lost_;
    }
    /// The torn lines among them: a piece refused, the line's held
    /// pieces released unwritten. Main context, where it is written.
    static uint16_t torn_lines() { return torn_lines_; }
    /// Transmit edges waited for: each time a drain stopped because the
    /// ring could not hold the next message (or the mark), counted once
    /// until something is written again. Main context.
    static uint32_t waits() { return waits_; }
    /// With `measured`: the bytes written into the ring and the sum of
    /// the bounds they were written under (the gap marks excluded), and
    /// the lines. Zero otherwise. Main context.
    static uint32_t bytes_written() {
        if constexpr (opts.measured) {
            return Sink::bytes;
        } else {
            return 0;
        }
    }
    static uint32_t bound_written() { return bound_written_; }
    static uint32_t lines_written() { return lines_written_; }

    /// Whether the writer has nothing left: no complete line waiting for
    /// the ring, no mark owed, nothing in its queue. Lines still open
    /// (pieces held, waiting for their end) do not count. Main context.
    static bool idle() {
        return completed_count_ == 0u && !mark_owed() && queue.empty();
    }

    /// The panic path's drain: every line the writer still has, written
    /// into the transport in order - the complete lines, then the open
    /// lines as they are (closed by their code), then the messages still
    /// waiting in the queue - waiting for the ring's room before each
    /// message by calling `pump` while it is short. With interrupts
    /// enabled the transmitter drains the ring by itself and the pump may
    /// do nothing; with them masked (a panic) the pump must move the
    /// ring itself, by running the transport's interrupt body. Any
    /// context that may write the transport; the writer's state is
    /// consumed.
    template <typename Pump>
    static void drain_to_port(Pump pump) {
        writing(true);
        while (completed_count_ != 0u) {
            write_chain_waiting(completed_[completed_head_], true, pump);
            pop_completed();
        }
        for (uint8_t s = 0; s < opts.max_sources; ++s) {
            if (head_[s] != 0u) {
                write_chain_waiting(head_[s], true, pump);
                head_[s] = 0;
            }
        }
        for (;;) {
            const uint8_t h = queue.take();
            if (h == decltype(queue)::none) {
                break;
            }
            // The alternative asked by its index, not by get_if()'s null
            // test: avr-gcc 16.2 fails on that test of a slot's address
            // (an unrecognizable insn), the index the same question.
            if (queue.at(h).index() == 0u) {
                const Message& m = message(h);
                const bool last = (m.flags & ends_line) != 0u;
                wait_room(need(m, last), pump);
                m.format(m.packed);
                if (last) {
                    close_line(m.code);
                }
            }
            queue.release(h);
        }
        writing(false);
    }
    static void drain_to_port() {
        drain_to_port([] {});
    }

private:
    static constexpr uint8_t ends_line = 0x01;
    static constexpr uint8_t no_entry = 0xFF;
    static constexpr uint8_t epoch_mask = 0x7F;
    static constexpr uint8_t mark_bound = 5u + 5u + 2u;   // "lost ", a uint16_t, CRLF

    // ---- the client's side ----------------------------------------------------

    /// The largest bound a message may carry: what one byte holds, and
    /// what the ring holds with the line's closing beside it.
    static constexpr uint16_t limit(bool ends, uint8_t code) {
        const uint32_t ring = static_cast<uint32_t>(Transport::tx_capacity());
        const uint32_t closing = ends ? closing_bytes(code) : 0u;
        const uint32_t most = ring > closing ? ring - closing : 0u;
        return static_cast<uint16_t>(most < 255u ? most : 255u);
    }

    /// The source table's entry for a source: the pack indices first,
    /// then main context, then the handlers; no_entry past the table.
    static uint8_t entry_of(uint8_t source) {
        if (source == handler_source) {
            return static_cast<uint8_t>(opts.max_sources - 1u);
        }
        if (source == no_dispatch) {
            return static_cast<uint8_t>(opts.max_sources - 2u);
        }
        return source < opts.max_sources - 2u ? source : no_entry;
    }

    /// The message of one argument list, built and posted: one function
    /// per list of types, its frame holding the message.
    template <typename... Ts>
    [[gnu::noinline]] static bool pack_and_post(const size_t* lengths, const Ts&... args) {
        using L = line_writer_detail::Layout<Ts...>;
        Message m;
        m.code = L::code;
        m.format = &line_writer_detail::format<Sink, Ts...>;
        uint16_t bound = L::static_bound;
        bool fits = true;
        pack_all<Ts...>(m.packed, lengths, bound, fits, std::index_sequence_for<Ts...>{}, args...);
        fits = fits && bound <= limit(L::ends, L::code);
        m.bound = static_cast<uint8_t>(bound);
        return post_built(m, L::ends, fits);
    }

    /// The part every list shares: the source, the decision, the service.
    [[gnu::noinline]] static bool post_built(Message& m, bool ends, bool fits) {
        const bool from_handler = P::in_handler();
        const uint8_t source = from_handler ? handler_source : dispatching<P>;
        m.source = source;
        if (!from_handler && source == opts.inline_from) {
            const uint8_t outer = hold_ceiling();
            const bool taken = admit(source, false, ends, fits, m);
            serve_inline();
            restore_ceiling(outer);
            return taken;
        }
        return admit(source, from_handler, ends, fits, m);
    }

    template <typename... Ts, size_t... I>
    [[gnu::always_inline]] static void pack_all(uint8_t* packed, const size_t* lengths, uint16_t& bound,
                                                bool& fits, std::index_sequence<I...>, const Ts&... args) {
        using L = line_writer_detail::Layout<Ts...>;
        (pack_one<Ts>(packed + L::offsets.at[I], args, lengths[I], bound, fits), ...);
    }

    template <typename T>
    [[gnu::always_inline]] static void pack_one(uint8_t* at, const T& arg, size_t length, uint16_t& bound,
                                                bool& fits) {
        if constexpr (line_writer_detail::is_end<T>) {
        } else if constexpr (line_writer_detail::is_text<T>) {
            const char* text;
            if constexpr (std::is_same_v<T, std::string_view>) {
                text = arg.data();
            } else {
                text = arg;
            }
            if constexpr (debug_checks) {
                if constexpr (requires { P::stack_span(); }) {
                    const std::span<const uint8_t> stack = P::stack_span();
                    const uintptr_t a = reinterpret_cast<uintptr_t>(text);
                    const uintptr_t lo = reinterpret_cast<uintptr_t>(stack.data());
                    if (a >= lo && a < lo + stack.size()) {
                        on_misuse(misuse_stack_text);
                    }
                }
            }
            if (length > 255u) {
                fits = false;
                length = 0;
            }
            bound = static_cast<uint16_t>(bound + length);
            __builtin_memcpy(at, &text, sizeof text);
            at[sizeof text] = static_cast<uint8_t>(length);
        } else {
            __builtin_memcpy(at, &arg, sizeof arg);
        }
    }

    /// The decision: post, refuse, or tear (the header's "Refusals and
    /// tears"). The per-source flags are written by their own source's
    /// context alone - an object is never preempted by itself, a handler
    /// by another handler - so only the shared counters take the guard.
    static bool admit(uint8_t source, bool from_handler, bool ends, bool fits, Message& m) {
        const uint8_t s = entry_of(source);
        if (s == no_entry) {
            count_loss(false, 0);
            return false;
        }
        if (from_handler) {
            m.flags = static_cast<uint8_t>(ends_line | (tears_[s] << 1u));
            if (!ends || !fits || !push(m)) {
                count_loss(false, s);
                return false;
            }
            return true;
        }
        if (torn_[s]) {
            // A piece of this line was refused: the rest of it is refused
            // unposted, its end included, which ends the tear.
            if (ends) {
                torn_[s] = false;
            }
            return false;
        }
        m.flags = static_cast<uint8_t>((ends ? ends_line : 0u) | (tears_[s] << 1u));
        if (fits && push(m)) {
            composing_[s] = !ends;
            return true;
        }
        const bool tear = !ends || composing_[s];
        torn_[s] = !ends;
        composing_[s] = false;
        count_loss(tear, s);
        if (tear) {
            wake();
        }
        return false;
    }

    /// A tear wakes the writer in the slot the clients keep: the pieces it
    /// holds for the torn line may be all the clients' slots, and only a
    /// dispatch releases them. A wake that finds even that slot taken is
    /// not needed - whatever holds it brings the dispatch.
    static void wake() {
        if (queue.push_keeping(TxActivity{}, 0u)) {
            if constexpr (Preemptive<P>) {
                if (queue.priority() < P::ceiling) {
                    P::pend_schedule();
                }
            }
        }
    }

    /// The push that keeps one slot for TxActivity, and on a preemptive
    /// platform the scheduling question post() asks (kernel/post.hpp).
    static bool push(const Message& m) {
        if (!queue.push_keeping(m, 1u)) {
            return false;
        }
        if constexpr (Preemptive<P>) {
            if (queue.priority() < P::ceiling) {
                P::pend_schedule();
            }
        }
        return true;
    }

    /// A line lost, and a tear of source entry `s` when it was torn: the
    /// tear moves the source's epoch, which tells the writer to release
    /// the held pieces and to drop the line's pieces still queued.
    static void count_loss(bool tear, uint8_t s) {
        typename P::CriticalSection cs;
        if (lost_ != UINT16_MAX) {
            ++lost_;
        }
        if (tear) {
            tears_[s] = static_cast<uint8_t>((tears_[s] + 1u) & epoch_mask);
            tear_events_ = static_cast<uint8_t>(tear_events_ + 1u);
        }
    }

    /// The serving context holds the ceiling at the writer's index on a
    /// preemptive platform, so that the kernel does not start the writer
    /// over the dispatches the caller runs; the previous ceiling returned.
    static uint8_t hold_ceiling() {
        if constexpr (Preemptive<P>) {
            const uint8_t outer = P::ceiling;
            const uint8_t own = queue.priority();
            if (own < outer) {
                P::ceiling = own;
            }
            std::atomic_signal_fence(std::memory_order_seq_cst);
            return outer;
        } else {
            return 0;
        }
    }

    /// The ceiling back, and a scheduling request when something arrived
    /// for the writer meanwhile that the restored ceiling now lets run.
    static void restore_ceiling([[maybe_unused]] uint8_t outer) {
        if constexpr (Preemptive<P>) {
            std::atomic_signal_fence(std::memory_order_seq_cst);
            P::ceiling = outer;
            if (!queue.empty() && queue.priority() < outer) {
                P::pend_schedule();
            }
        }
    }

    /// The inline service: the writer's dispatches run here until its
    /// queue is empty and every complete line is in the ring, the ring's
    /// room polled while the next message does not fit.
    static void serve_inline() {
        for (;;) {
            while (serve_one<LineWriter>()) {
            }
            if (completed_count_ == 0u && !mark_owed()) {
                return;
            }
            const uint32_t wanted = completed_count_ != 0u ? head_need() : mark_bound;
            while (Transport::tx_room() < wanted) {
            }
            drain();
        }
    }

    // ---- the writer's side ----------------------------------------------------

    static const Message& message(uint8_t h) {
        const Message* m = std::get_if<Message>(&queue.at(h));
        if (m == nullptr) {
            __builtin_unreachable();   // every slot the writer keeps holds a Message
        }
        return *m;
    }

    /// The bytes a message needs in the ring: its bound, and the closing
    /// when it is the last of its line.
    static uint32_t need(const Message& m, bool last) {
        return static_cast<uint32_t>(m.bound) + (last ? closing_bytes(m.code) : 0u);
    }

    static uint32_t head_need() {
        const uint8_t h = completed_[completed_head_];
        return need(message(h), next_[h] == 0u);
    }

    static bool mark_owed() {
        typename P::CriticalSection cs;
        return lost_ != marked_;
    }

    /// A tear moved some source's epoch since the last look: every source
    /// is looked at, and the held pieces of a torn line released.
    static void sweep_tears() {
        const uint8_t t = line_writer_detail::load_shared(tear_events_);
        if (t == seen_tears_) {
            return;
        }
        seen_tears_ = t;
        for (uint8_t s = 0; s < opts.max_sources; ++s) {
            sync(s);
        }
    }

    /// Source entry `s`'s epoch read: when it moved, the line open there
    /// was torn - its held pieces released unwritten, counted.
    static void sync(uint8_t s) {
        const uint8_t t = line_writer_detail::load_shared(tears_[s]);
        if (t != epoch_[s]) {
            release_chain(head_[s]);
            head_[s] = 0;
            torn_lines_ = static_cast<uint16_t>(
                saturate(static_cast<uint32_t>(torn_lines_) + ((t - epoch_[s]) & epoch_mask)));
            epoch_[s] = t;
        }
    }

    static void on_message(const Message& m) {
        const uint8_t s = entry_of(m.source);
        sync(s);
        if ((m.flags >> 1u) != epoch_[s]) {
            return;   // a piece of a line torn since: dropped, the kernel releases its slot
        }
        const uint8_t h = queue.hold().handle();
        next_[h] = 0;
        if (head_[s] == 0u) {
            head_[s] = h;
        } else {
            next_[tail_[s]] = h;
        }
        tail_[s] = h;
        if ((m.flags & ends_line) != 0u) {
            uint8_t at = static_cast<uint8_t>(completed_head_ + completed_count_);
            if (at >= depth) {
                at = static_cast<uint8_t>(at - depth);
            }
            completed_[at] = head_[s];
            ++completed_count_;
            head_[s] = 0;
        }
    }

    /// The complete lines into the ring, in order, a message at a time
    /// while the room holds it; then the gap mark, when one is owed and
    /// the lines are out. The room is read once and spent by the bounds,
    /// read again only when it runs short: what the ring really took is
    /// at most the bound, and the room only grows behind the writer.
    static void drain() {
        writing(true);
        uint32_t room = Transport::tx_room();
        while (completed_count_ != 0u) {
            uint8_t h = completed_[completed_head_];
            for (;;) {
                const Message& m = message(h);
                const uint8_t after = next_[h];
                const uint32_t wanted = need(m, after == 0u);
                if (wanted > room) {
                    room = Transport::tx_room();
                    if (wanted > room) {
                        completed_[completed_head_] = h;   // the line resumes here
                        count_wait();
                        writing(false);
                        return;
                    }
                }
                room -= wanted;
                waiting_ = false;
                write(m, after == 0u);
                queue.release(h);
                if (after == 0u) {
                    break;
                }
                h = after;
            }
            pop_completed();
        }
        write_mark(room);
        writing(false);
    }

    static void write(const Message& m, bool last) {
        m.format(m.packed);
        if constexpr (opts.measured) {
            bound_written_ += m.bound;
        }
        if (last) {
            close_line(m.code);
            if constexpr (opts.measured) {
                ++lines_written_;
            }
        }
    }

    /// The closing of a line, by its code.
    static void close_line(uint8_t code) {
        switch (code) {
        case code_text:
            print_one(Transport{}, crlf);
            break;
        default:
            break;   // code_bare: nothing
        }
    }

    /// "lost N", the losses since the last mark, when one is owed, no
    /// message waits in the queue (it may have been posted before the
    /// loss) and the mark fits; otherwise the next drain tries again.
    static void write_mark(uint32_t room) {
        uint16_t now;
        {
            typename P::CriticalSection cs;
            now = lost_;
        }
        if (now == marked_ || !queue.empty()) {
            return;   // none owed, or lines posted before the loss may still be waiting
        }
        if (room < mark_bound) {
            room = Transport::tx_room();
            if (room < mark_bound) {
                count_wait();
                return;
            }
        }
        waiting_ = false;
        print_one(Transport{}, std::string_view("lost ", 5));
        print_one(Transport{}, static_cast<uint16_t>(now - marked_));
        close_line(code_text);
        marked_ = now;
    }

    /// A drain stopped for the ring: counted once per edge waited for,
    /// however many dispatches find the ring still short before it comes.
    static void count_wait() {
        if (!waiting_) {
            waiting_ = true;
            if (waits_ != UINT32_MAX) {
                ++waits_;
            }
        }
    }

    template <typename Pump>
    static void wait_room(uint32_t wanted, Pump& pump) {
        while (Transport::tx_room() < wanted) {
            pump();
        }
    }

    /// A chain written whole, closed by its last message's code, its
    /// slots released: the panic drain's.
    template <typename Pump>
    static void write_chain_waiting(uint8_t h, bool close, Pump& pump) {
        while (h != 0u) {
            const Message& m = message(h);
            const uint8_t after = next_[h];
            wait_room(need(m, close && after == 0u), pump);
            m.format(m.packed);
            if (close && after == 0u) {
                close_line(m.code);
            }
            queue.release(h);
            h = after;
        }
    }

    static void release_chain(uint8_t h) {
        while (h != 0u) {
            const uint8_t after = next_[h];
            queue.release(h);
            h = after;
        }
    }

    static void pop_completed() {
        completed_head_ = static_cast<uint8_t>(completed_head_ + 1u == depth ? 0u : completed_head_ + 1u);
        --completed_count_;
    }

    static constexpr uint32_t saturate(uint32_t v) { return v > UINT16_MAX ? UINT16_MAX : v; }

    /// The writer's own writes, told to the transport's debug check.
    static void writing(bool on) {
        if constexpr (debug_checks) {
            WriterClaim<Transport>::writing = on;
        }
    }

    /// A write to the claimed transport from anyone but the writer.
    static void direct_write() { on_misuse(misuse_direct_write); }

    // ---- the writer's state: main context only, written by its dispatch -------
    static inline uint8_t next_[depth + 2u]{};               ///< slot links, 0 the end
    static inline uint8_t head_[opts.max_sources]{};          ///< each source's open line, 0 none
    static inline uint8_t tail_[opts.max_sources]{};
    static inline uint8_t epoch_[opts.max_sources]{};         ///< the tear epoch its open line belongs to
    static inline uint8_t completed_[depth]{};                ///< heads of complete lines, oldest first
    static inline uint8_t completed_head_ = 0;
    static inline uint8_t completed_count_ = 0;
    static inline uint8_t seen_tears_ = 0;
    static inline uint16_t torn_lines_ = 0;
    static inline uint16_t marked_ = 0;                       ///< lost_ when the last mark was written
    static inline uint32_t waits_ = 0;
    static inline bool waiting_ = false;                     ///< a drain stopped, no write since
    static inline uint32_t bound_written_ = 0;
    static inline uint32_t lines_written_ = 0;

    // ---- the clients' side: each source's flags written by its own context ----
    static inline bool torn_[opts.max_sources]{};             ///< a piece refused, the line not ended
    static inline bool composing_[opts.max_sources]{};        ///< a piece taken, the line not ended
    static inline uint8_t tears_[opts.max_sources]{};         ///< the tear epoch, 7 bits
    static inline uint8_t tear_events_ = 0;                   ///< every tear, under the guard
    static inline uint16_t lost_ = 0;                         ///< under the guard
};

/// Whether a type is a LineWriter: print() to it posts a message.
template <typename W>
concept LineWriterType = MessageSink<W> && requires { typename W::Message; };

/// print() to a LineWriter: the writer's client verb, post_message() (the
/// header). True when the message was taken. Always inlined, as the front
/// of post_message() is: the texts' lengths are measured where the call
/// is written, so a literal's is a constant.
template <LineWriterType W, typename... Args>
[[gnu::always_inline]] inline bool print(W, const Args&... args) {
    return W::post_message(args...);
}

/// A panic reporter that drains a writer into its transport first
/// (drain_to_port()), calling `pump` while the ring is short - with the
/// interrupts masked for good, the transport's interrupt body run by
/// hand - then hands over to `Then`. It runs where panic()'s reporter
/// runs, after break_here() (kernel/panic.hpp says where that returns).
template <typename Writer, void (*pump)() = nullptr, typename Then = HaltReporter>
struct LineWriterReporter {
    static void report(PanicCode code, uint8_t context) {
        Writer::drain_to_port([] {
            if constexpr (pump != nullptr) {
                pump();
            }
        });
        Then::report(code, context);
    }
};

} // namespace brio
