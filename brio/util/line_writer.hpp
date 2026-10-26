/*
 * line_writer.hpp
 *
 * LineWriter: the one writer of a transport, for a program in which one
 * printer can run in the middle of another's line - the objects of a
 * Rubato kernel (kernel/rubato.hpp), where a higher object's dispatch
 * preempts a lower one's, and an interrupt handler that prints. Every
 * line printed to it reaches the wire whole, the lines in the order they
 * were completed, or is refused whole and counted. It is not an active
 * object and has no queue: it is a sink type, like a transport, whose
 * verbs format each print() at the call, with the print_one() verbs of
 * util/print.hpp, into a buffer the caller's line owns (its STAGE), and
 * hand the whole line to the transport in one run when the line ends.
 * The transport's own transmit ring is the only queue. The stack it
 * belongs to is docs/design/serial.md, "LineWriter"; the host suite
 * drives it under the preempting kernel (test/test_line_writer).
 *
 * The smallest use names the writer over a transport of the family,
 * claims the transport once from main() before the kernel runs, and
 * prints to it wherever the program prints:
 *
 *     using Log = brio::LineWriter<Serial, P>;      // one stage of 192, two of 96
 *     ...
 *     Log::init();                                   // in main(), before run()
 *     brio::Rubato<P, App, Console>::run();
 *     ...
 *     brio::print(Log{}, "count = ", count, brio::crlf);   // anywhere
 *
 * Under Tenuto (kernel/tenuto.hpp) a program needs none of this: one
 * dispatch runs at a time, so a line printed straight to the transport
 * is whole by construction, and the program writes `using Log = Serial;`
 * and keeps its print(Log{}, ...) calls. A header's template that no
 * program instantiates compiles to nothing, so that image carries no
 * byte of the writer. (A handler does not print there: a print to a
 * transport is main context's alone, util/print.hpp says why.)
 *
 * Sources and stages. A line belongs to its source: the active object
 * being dispatched (dispatching<P>, kernel/active_object.hpp: the
 * object's index in the list the program gives the kernel, the first
 * object 0 and the highest priority), main context (the code that runs
 * outside every dispatch and every handler: main() before the kernel
 * runs, and the kernel's idle turn after; no_dispatch), or
 * `handler_source` for every interrupt handler alike (P::in_handler(),
 * kernel/platform.hpp). A table of a byte per source (`max_sources`
 * entries: the objects' indices from 0, main context and the handlers in
 * the last two) says which stage holds the source's open line. The
 * stages are a fixed pool: one long stage (`long_size` bytes) for the
 * sources allowed to wait - main context, and the one object `wait_from`
 * names - which they take before a small one and which no other source
 * ever gets, and `small_stages` small ones (`small_size` bytes) for every
 * source; so a line of any other source is bounded by `small_size`, the
 * CRLF included. A line's first print takes a free stage, the one
 * decision made with interrupts masked (a preemptor landing between the
 * look and the claim would take the same stage); a line that finds none
 * is refused whole, and its later bytes are dropped up to its end. A
 * line never ended keeps its stage: a forgotten crlf takes a stage out
 * of the pool for good.
 *
 * A line and its end. print() formats each argument into the caller's
 * stage at the call: a C string scanned and copied in one pass, stopping
 * where the stage ends; a std::string_view copied as one run; a number
 * converted by print.hpp's verbs into a short buffer and copied as a C
 * string. Every `crlf` ends a line, wherever it stands in the argument
 * list - a verb cannot know its position - so `print(Log{}, "a", crlf,
 * "b", crlf)` is two lines, each published as its crlf comes and each
 * taking a stage of its own, and a program that wants a line break
 * inside one line writes it as text ("\r\n"). `flush_line` ends a line
 * with no bytes, the shape of a prompt that must leave before an end of
 * line comes. A line may be printed in several calls, across dispatches
 * too: the stage stays its source's until the end. The text is consumed
 * at the call, so a buffer on the caller's stack is as good as a
 * literal.
 *
 * The line's end publishes it: the stage's bytes into the transport as
 * one write_bulk(), under a lock, a flag meaning "a publisher is
 * copying". Taking the lock and finding it held are decided with
 * interrupts masked; the copy runs with them enabled. A line completed
 * while another publisher holds the lock - by a higher object that
 * preempted it, or by a handler - is marked pending with a sequence
 * number and its caller returns: under the kernel's nesting (a preemptor
 * finishes before the preempted resumes, a handler runs to completion)
 * the holder resumes after it, writes its own line, then the pending
 * ones oldest first, and releases the lock. So lines reach the wire in
 * the order they were completed. The source's slot is freed at its
 * line's end, so its next line takes a fresh stage while the last one
 * pends. The sections that run with interrupts masked: the stage's take
 * (a scan of the pool for a free one), the lock's take (a flag tested
 * and set), the release-and-relay step (a written stage freed, the pool
 * scanned for the oldest pending one), and the counting of a refused
 * line; each a few loads and stores, none copies a byte.
 *
 * Who waits. A line is written when the transport's room holds it
 * whole. The sources allowed to wait - main context and `wait_from` -
 * spin on tx_room() for it while they hold the lock, as print() to a
 * transport spins; the transmit interrupt drains the ring under the
 * spin, so a waiting source that prints with interrupts masked spins for
 * ever. The holder of the lock waits the same way for a pending line of
 * such a source, and, when it may wait itself, for every pending line it
 * relays. A line of a waiting source pends only while a holder it
 * preempted copies, so an object that may not wait can be made to spin
 * only by a `wait_from` above it in the list: a program names as
 * `wait_from` its lowest printing object (a console that prints the
 * long reports), and nothing below it ever relays its lines. Every other
 * line that finds no room is refused whole and counted. A handler never
 * waits.
 *
 * The counters and the mark. lost() counts every line refused - no
 * stage free, no room, a line that outgrew its stage, a source past the
 * table - saturating, in lines, never retried; one counter, so a loss is
 * diagnosed one knob at a time (the pool, the stage sizes, the ring).
 * The wire shows where lines are missing: the next holder of the lock
 * writes "lost N" and CRLF before its own line, the losses since the last
 * mark, when the room holds the mark and the line (a source that may
 * wait waits for the mark's room; for any other the mark stays owed to
 * the next line). With the `peak` option the writer also counts the most
 * stages held at once (peak_stages()) and the lines and bytes it
 * published; a program sizes the pool and the ring by these.
 *
 * Interrupt handlers. A handler's print() formats in the handler, so
 * every other interrupt waits for the whole print: a C string's bytes
 * scanned and copied, each number converted, the run handed to the
 * transport. The cost is inferred from the same line printed from a
 * dispatch with interrupts masked - for a line of 60 bytes on the
 * STM32F446RE at 180 MHz, 1785 to 1937 cycles, some 10 us
 * (docs/design/serial.md, "LineWriter", "The cost") - not measured
 * from a handler. The handlers share one source, so one line open at a
 * time among them all: a handler ends the line it starts (the caller's
 * duty: two handlers composing pieces would mix them). A handler that
 * must stay short posts a value to an object instead, and the object
 * prints it.
 *
 * Who calls what. print(Log{}, ...) from any context; it masks
 * interrupts only for the sections above and waits only where the
 * caller may wait or relays a line whose source may. init() once, from
 * main() before the kernel runs: it claims the transport (util/stream.hpp's
 * WriterClaim), so that in a debug build a write to the transport from
 * anywhere but the writer panics; a release build compiles none of it.
 * It sets up nothing else: a print to the writer before init() works,
 * and a print to the transport itself before init() (a banner) is legal
 * where after it is a misuse. drain_to_port() and LineWriterReporter are
 * the panic path, plain code with the kernel dead; the drain releases the
 * claim, so a reporter that prints to the transport after it may.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <concepts>
#include <span>

#include "kernel/active_object.hpp"
#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "util/print.hpp"
#include "util/stream.hpp"

namespace brio {

/// The source an interrupt handler's line belongs to, beside the pack
/// indices and no_dispatch (kernel/active_object.hpp).
inline constexpr uint8_t handler_source = 254;

/// What a LineWriter is sized and allowed by.
///  - max_sources: the entries of the source table - the pack's objects,
///    main context and the handlers - and so the pack it serves: indices
///    0 to max_sources - 3, main context and the handlers in the last
///    two. A line from an index past them is refused and counted;
///  - wait_from: the one object, by its index in the list the program
///    gives the kernel (the first object 0), whose lines may wait for the
///    transport's room: the lowest object that prints, a console whose
///    dispatch prints the long reports (the header, "Who waits");
///    no_dispatch, the default, names none, main context waiting anyway;
///  - long_size: the long stage, main context's and wait_from's alone;
///  - small_stages, small_size: the pool every source takes from, the
///    bound of every other source's line;
///  - peak: count the most stages held at once and the lines and bytes
///    published (peak_stages(), lines_written(), bytes_written()); off,
///    nothing of it is compiled.
/// The stage sizes are bytes, each at most 255 and at most the
/// transport's tx_capacity(): a source that waits spins until its whole
/// line fits, which must be possible. So a transport with a 64-byte ring
/// (the three WCH families' default) takes stages sized under it, 63 and
/// 48 say, or a larger ring. The two sizes are 16-bit fields so that a
/// size over 255 is refused by the check below, with its reason, and not
/// by a narrowing error.
struct LineWriterOptions {
    uint8_t max_sources = 16;
    uint8_t wait_from = no_dispatch;
    uint16_t long_size = 192;
    uint8_t small_stages = 2;
    uint16_t small_size = 96;
    bool peak = false;
};

template <typename Transport, Platform P, LineWriterOptions opts = LineWriterOptions{}>
class LineWriter {
    static_assert(BulkSink<Transport> && RoomSink<Transport>,
                  "a LineWriter's transport takes a run and tells its room: write_bulk() and "
                  "tx_room() (util/stream.hpp)");
    static_assert(requires { { Transport::tx_capacity() } -> std::convertible_to<uint32_t>; },
                  "a LineWriter's transport tells its transmit ring's capacity: tx_capacity()");
    static_assert(HandlerAware<P>,
                  "a LineWriter tells a handler's line from a dispatch's: P::in_handler() "
                  "(kernel/platform.hpp)");
    static_assert(opts.max_sources >= 3 && opts.max_sources <= 128,
                  "the source table holds main context, the handlers and at least one object");
    static_assert(opts.wait_from == no_dispatch || opts.wait_from < opts.max_sources - 2u,
                  "wait_from names an object of the source table, or no_dispatch for none");
    static_assert(opts.small_stages >= 1 && opts.small_stages <= 7, "a pool of one to seven small stages");
    static_assert(opts.long_size >= 2 && opts.small_size >= 2,
                  "a stage holds at least a line's CRLF");
    static_assert(opts.long_size <= 255 && opts.small_size <= 255,
                  "a stage's length is a byte: at most 255");
    static_assert(opts.long_size <= Transport::tx_capacity() && opts.small_size <= Transport::tx_capacity(),
                  "a stage larger than the transport's ring could never be written whole");

public:
    /// The why of a debug panic (PanicCode::assert_failed's context).
    static constexpr uint8_t misuse_direct_write = 0xA2;   ///< a write to the claimed transport

    /// The default of on_misuse below: panic<P>(), the why its context.
    static void panic_on_misuse(uint8_t why) { panic<P>(PanicCode::assert_failed, why); }

    /// What a debug check calls on a misuse: panic_on_misuse() by default;
    /// a host test may point it at a recorder. Debug builds only.
    static inline void (*on_misuse)(uint8_t why) = &panic_on_misuse;

    /// Claims the transport: from here on, in a debug build, a write to
    /// it from anywhere but the writer is a misuse (the header). Main
    /// context, once, before the kernel runs; nothing else is set up here.
    static void init() {
        if constexpr (debug_checks) {
            WriterClaim<Transport>::violated = &direct_write;
            WriterClaim<Transport>::claimed = true;
        }
    }

    // ---- print()'s verbs: a program calls print(), these are its sink --------

    /// One byte onto the caller's line. True always: a refused line drops
    /// its bytes, and print() must never spin on it. Any context.
    [[gnu::noinline]] static bool write_byte(uint8_t b) {
        if (Stage* const s = stage_of_caller()) {
            if (s->len < s->cap) {
                s->data[s->len] = b;
                s->len = static_cast<uint8_t>(s->len + 1u);
            } else {
                s->overflow = true;
            }
        }
        return true;
    }

    /// A run onto the caller's line, copied whole or not at all (a run
    /// that does not fit what is left of the stage refuses the line at its
    /// end). Returns the run's length always, for print()'s loop. Any
    /// context.
    [[gnu::noinline]] static uint32_t write_bulk(std::span<const uint8_t> run) {
        const uint32_t n = static_cast<uint32_t>(run.size());
        if (Stage* const s = stage_of_caller()) {
            if (static_cast<uint32_t>(s->cap - s->len) >= n) {
                __builtin_memcpy(s->data + s->len, run.data(), n);
                s->len = static_cast<uint8_t>(s->len + n);
            } else {
                s->overflow = true;
            }
        }
        return n;
    }

    /// A C string onto the caller's line: one resolution of the stage and
    /// one pass that scans and copies together, stopping where the stage
    /// ends (the line is then refused whole at its end). Any context.
    [[gnu::noinline]] static void text(const char* p) {
        Stage* const s = stage_of_caller();
        if (s == nullptr) {
            return;
        }
        uint8_t* const start = s->data;
        uint8_t* at = start + s->len;
        uint8_t* const end = start + s->cap;
        uint8_t c;
        while ((c = static_cast<uint8_t>(*p++)) != 0u) {
            if (at == end) {
                s->overflow = true;   // the text goes on past the stage's end
                break;
            }
            *at++ = c;
        }
        s->len = static_cast<uint8_t>(at - start);
    }

    /// The caller's line ends: `crlf` appends its two bytes, flush_line
    /// (false) nothing. A line that fit its stage is published (the
    /// header); one that outgrew it is counted lost and dropped whole; a
    /// crlf with nothing open is an empty line. Any context; waits only
    /// where the caller may wait, or relays a line whose source may.
    [[gnu::noinline]] static void end_line(bool crlf) {
        const uint8_t src = source();
        const uint8_t e = entry_of(src);
        if (e == no_entry) {
            count_loss();   // a source past the table: its line refused here
            return;
        }
        const uint8_t slot = slots_[e];
        Stage* s;
        if (static_cast<uint8_t>(slot - 1u) < count) {
            s = &stages_[slot - 1u];
        } else if (slot == slot_refused) {
            slots_[e] = slot_none;   // the refused line ends; counted when it was refused
            return;
        } else {
            if (!crlf) {
                return;   // flush_line with nothing open
            }
            s = take(src, e);
            if (s == nullptr) {
                slots_[e] = slot_none;
                return;
            }
        }
        if (crlf) {
            if (static_cast<uint8_t>(s->cap - s->len) >= 2u) {
                s->data[s->len] = '\r';
                s->data[s->len + 1u] = '\n';
                s->len = static_cast<uint8_t>(s->len + 2u);
            } else {
                s->overflow = true;
            }
        }
        slots_[e] = slot_none;   // the source's next line takes a stage of its own
        if (s->overflow) {
            typename P::CriticalSection cs;
            free_stage(*s);
            note_loss();
            return;
        }
        publish(*s, src);
    }

    // ---- the counters ---------------------------------------------------------

    /// Lines refused since the program started, saturating at 65535 (the
    /// header). Any context.
    static uint16_t lost() {
        typename P::CriticalSection cs;
        return lost_;
    }

    /// With `peak`: the most stages held at once since the program
    /// started, the pool's high-water mark. Any context.
    static uint8_t peak_stages()
        requires(opts.peak)
    {
        return load_shared(peak_);
    }

    /// With `peak`: the lines and the bytes published into the transport
    /// since the program started (the "lost N" marks not counted). Written
    /// by the lock's holder alone; read as a snapshot.
    static uint32_t lines_written()
        requires(opts.peak)
    {
        return lines_;
    }
    static uint32_t bytes_written()
        requires(opts.peak)
    {
        return bytes_;
    }

    // ---- the panic path -------------------------------------------------------

    /// Everything the writer holds, written into the transport by plain
    /// code: the line being published (from its start, so a line a panic
    /// cut in its copy shows its first bytes twice), the pending lines in
    /// the order they were completed, then the open lines as they stand.
    /// `pump` is called while the ring's room is short: with interrupts
    /// enabled the transmitter drains by itself and it may do nothing;
    /// with them masked for good (a panic) it must move the ring itself,
    /// by running the transport's interrupt body. Any context that may
    /// write the transport; the writer's state is consumed, and the
    /// transport's claim released: what prints after the drain (a panic
    /// reporter) prints to the transport itself.
    template <typename Pump>
    static void drain_to_port(Pump pump) {
        writing(true);
        drain_stages<st_publishing>(pump);
        drain_stages<st_pending>(pump);
        drain_stages<st_open>(pump);
        for (uint8_t& slot : slots_) {
            slot = slot_none;
        }
        busy_ = false;
        writing(false);
        if constexpr (debug_checks) {
            WriterClaim<Transport>::claimed = false;
        }
    }
    static void drain_to_port() {
        drain_to_port([] {});
    }

private:
    /// The panic drain's pass over the stages in one state.
    template <uint8_t state, typename Pump>
    static void drain_stages(Pump& pump) {
        while (Stage* const s = oldest_in<state>()) {
            while (Transport::tx_room() < s->len) {
                pump();
            }
            put(s->data, s->len);
            free_stage(*s);
        }
    }

    static constexpr uint8_t count = static_cast<uint8_t>(opts.small_stages + 1u);   ///< stage 0 the long one
    // A source's slot: slot_none, its open line's stage index plus one,
    // or slot_refused - zero at reset, so the table needs no initializer.
    static constexpr uint8_t slot_none = 0;         ///< no line open
    static constexpr uint8_t slot_refused = 0xFF;   ///< its line refused: dropped through its end
    static constexpr uint8_t no_entry = 0xFF;
    static constexpr uint8_t mark_size = 12;        ///< "lost 65535" and CRLF
    static_assert(Transport::tx_capacity() >= mark_size, "the transport's ring holds the \"lost N\" mark");

    enum : uint8_t { st_free = 0, st_open = 1, st_pending = 2, st_publishing = 3 };

    /// A stage: its buffer, its line's length, and who holds it. `owner`
    /// and `state` change under the guard; `len`, `overflow` and the bytes
    /// are written by the owner's context alone while the line is open,
    /// read by the publisher after the line's end.
    struct Stage {
        uint8_t* data;
        uint8_t cap;
        uint8_t len;
        uint8_t owner;      ///< the source composing or holding it
        uint8_t state;      ///< st_free, st_open, st_pending, st_publishing
        uint8_t seq;        ///< the order a pending line was completed in
        bool overflow;      ///< the line outgrew the stage: refused whole at its end
    };

    /// A byte another context writes: read fresh.
    [[gnu::always_inline]] static uint8_t load_shared(const uint8_t& b) {
        return *const_cast<const volatile uint8_t*>(&b);
    }

    [[gnu::always_inline]] static uint8_t source() {
        return P::in_handler() ? handler_source : dispatching<P>;
    }

    static constexpr bool may_wait(uint8_t src) { return src == no_dispatch || src == opts.wait_from; }

    /// The source table's entry: the pack indices first, then main
    /// context, then the handlers; no_entry past the table.
    [[gnu::always_inline]] static uint8_t entry_of(uint8_t src) {
        if (src == handler_source) {
            return static_cast<uint8_t>(opts.max_sources - 1u);
        }
        if (src == no_dispatch) {
            return static_cast<uint8_t>(opts.max_sources - 2u);
        }
        return src < opts.max_sources - 2u ? src : no_entry;
    }

    /// The caller's open line's stage, or a fresh one for it; null while
    /// its line is refused (nothing is formatted into nowhere, the bytes
    /// drop) and for a source past the table (counted at its line's end).
    [[gnu::noinline]] static Stage* stage_of_caller() {
        const uint8_t src = source();
        const uint8_t e = entry_of(src);
        if (e == no_entry) {
            return nullptr;
        }
        const uint8_t slot = slots_[e];
        if (static_cast<uint8_t>(slot - 1u) < count) {
            return &stages_[slot - 1u];
        }
        if (slot == slot_refused) {
            return nullptr;
        }
        return take(src, e);
    }

    /// A free stage for a new line of `src`, entry `e`: the long one first
    /// for a source that may wait, the small ones for every source. The
    /// claim is the decision under the guard; none free refuses the line.
    [[gnu::noinline]] static Stage* take(uint8_t src, uint8_t e) {
        uint8_t i = may_wait(src) ? 0u : 1u;
        {
            typename P::CriticalSection cs;
            for (; i < count; ++i) {
                if (stages_[i].state == st_free) {
                    stages_[i].state = st_open;
                    break;
                }
            }
            if (i == count) {
                note_loss();
            } else if constexpr (opts.peak) {
                held_ = static_cast<uint8_t>(held_ + 1u);
                if (held_ > peak_) {
                    peak_ = held_;
                }
            }
        }
        if (i == count) {
            slots_[e] = slot_refused;
            return nullptr;
        }
        Stage& got = stages_[i];
        got.owner = src;
        got.len = 0;
        got.overflow = false;
        slots_[e] = static_cast<uint8_t>(i + 1u);
        return &got;
    }

    /// A stage given back. Under the guard, or in the panic path.
    [[gnu::always_inline]] static void free_stage(Stage& s) {
        s.state = st_free;
        if constexpr (opts.peak) {
            held_ = static_cast<uint8_t>(held_ - 1u);
        }
    }

    /// A line refused, counted: under the guard.
    [[gnu::always_inline]] static void note_loss() {
        if (lost_ != UINT16_MAX) {
            lost_ = static_cast<uint16_t>(lost_ + 1u);
        }
    }

    static void count_loss() {
        typename P::CriticalSection cs;
        note_loss();
    }

    /// A stage in `state`: for st_pending the one completed first (the
    /// lowest sequence number, read across its wrap), for the others the
    /// first in the pool. Under the guard, or in the panic path.
    template <uint8_t state>
    static Stage* oldest_in() {
        Stage* s = nullptr;
        for (Stage& t : stages_) {
            if (t.state != state) {
                continue;
            }
            if constexpr (state == st_pending) {
                if (s == nullptr || static_cast<int8_t>(t.seq - s->seq) < 0) {
                    s = &t;
                }
            } else {
                return &t;
            }
        }
        return s;
    }

    /// A run into the transport. The caller has seen the room for it, and
    /// the room only grows behind the lock's holder, so one call takes it.
    static void put(const uint8_t* data, uint32_t n) {
        std::span<const uint8_t> run(data, n);
        while (!run.empty()) {
            run = run.subspan(Transport::write_bulk(run));
        }
    }

    /// "lost N" and CRLF, N the losses since the last mark, before the
    /// holder's own line of `line` bytes: written when the room holds
    /// both, waited for by a holder that may wait, owed otherwise.
    static void write_mark(uint16_t owed, uint16_t now, bool waits, uint32_t line) {
        char mark[mark_size + 1] = "lost ";   // the digits' NUL beside the CRLF's room
        (void)ultoa(owed, mark + 5, 10);
        uint8_t n = 5;
        while (mark[n] != '\0') {
            ++n;
        }
        mark[n++] = '\r';
        mark[n++] = '\n';
        if (waits) {
            while (Transport::tx_room() < n) {
            }
        } else if (Transport::tx_room() < n + line) {
            return;
        }
        put(reinterpret_cast<const uint8_t*>(mark), n);
        marked_ = now;
    }

    /// The line of stage `first`, completed by `src`, into the transport
    /// (the header, "The line's end publishes"): the lock taken, or the
    /// line left pending for its holder; then the mark when one is owed,
    /// the line, and every line that pended meanwhile, oldest first, each
    /// written whole when the room holds it - waited for when the holder
    /// or the line's own source may wait - or refused and counted.
    [[gnu::noinline]] static void publish(Stage& first, uint8_t src) {
        uint16_t now;
        {
            typename P::CriticalSection cs;
            if (busy_) {
                first.state = st_pending;
                first.seq = seq_;
                seq_ = static_cast<uint8_t>(seq_ + 1u);
                return;
            }
            busy_ = true;
            first.state = st_publishing;
            now = lost_;
        }
        writing(true);
        const bool holder_waits = may_wait(src);
        if (now != marked_) {
            write_mark(static_cast<uint16_t>(now - marked_), now, holder_waits, first.len);
        }
        Stage* s = &first;
        for (;;) {
            const uint32_t n = s->len;
            // A handler holding the lock never waits, and nothing pends
            // while it holds it (no handler nests over another).
            const bool waits = holder_waits || (src != handler_source && may_wait(s->owner));
            if (waits) {
                while (Transport::tx_room() < n) {
                }
            }
            const bool fits = waits || Transport::tx_room() >= n;
            if (fits) {
                put(s->data, n);
                if constexpr (opts.peak) {
                    lines_ = lines_ + 1u;
                    bytes_ = bytes_ + n;
                }
            }
            typename P::CriticalSection cs;
            if (!fits) {
                note_loss();
            }
            free_stage(*s);
            s = oldest_in<st_pending>();
            if (s == nullptr) {
                writing(false);
                busy_ = false;
                return;
            }
            s->state = st_publishing;
        }
    }

    /// The writer's own writes, told to the transport's debug check.
    static void writing(bool on) {
        if constexpr (debug_checks) {
            WriterClaim<Transport>::writing = on;
        }
    }

    /// A write to the claimed transport from anyone but the writer.
    static void direct_write() { on_misuse(misuse_direct_write); }

    // ---- the state ------------------------------------------------------------
    alignas(4) static inline uint8_t long_store_[opts.long_size];
    alignas(4) static inline uint8_t small_store_[opts.small_stages][opts.small_size];

    static constexpr std::array<Stage, count> initial_stages() {
        std::array<Stage, count> a{};
        a[0] = Stage{long_store_, static_cast<uint8_t>(opts.long_size), 0, no_dispatch, st_free, 0, false};
        for (uint8_t i = 0; i < opts.small_stages; ++i) {
            a[i + 1u] = Stage{small_store_[i], static_cast<uint8_t>(opts.small_size), 0, no_dispatch, st_free, 0,
                              false};
        }
        return a;
    }
    static inline std::array<Stage, count> stages_ = initial_stages();
    /// A byte per source: its open line's stage, slot_none or
    /// slot_refused. Each entry written by its own source's context alone
    /// (an object is never preempted by itself, a handler by another
    /// handler), and by the panic path.
    static inline uint8_t slots_[opts.max_sources] = {};
    static inline bool busy_ = false;     ///< the lock: a publisher is copying
    static inline uint8_t seq_ = 0;       ///< the next pending line's number (at most `count` lines pend, so its wrap is never ambiguous)
    static inline uint16_t lost_ = 0;     ///< under the guard
    static inline uint16_t marked_ = 0;   ///< lost_ at the last mark; the lock's holder's
    static inline uint8_t held_ = 0;      ///< with `peak`: stages not free, under the guard
    static inline uint8_t peak_ = 0;
    static inline uint32_t lines_ = 0;    ///< with `peak`: the lock's holder's
    static inline uint32_t bytes_ = 0;
};

template <typename T, Platform P, LineWriterOptions o>
inline void print_one(LineWriter<T, P, o>, const char* text) {
    LineWriter<T, P, o>::text(text);
}

template <typename T, Platform P, LineWriterOptions o>
inline void print_one(LineWriter<T, P, o>, crlf_t) {
    LineWriter<T, P, o>::end_line(true);
}

template <typename T, Platform P, LineWriterOptions o>
inline void print_one(LineWriter<T, P, o>, flush_line_t) {
    LineWriter<T, P, o>::end_line(false);
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
