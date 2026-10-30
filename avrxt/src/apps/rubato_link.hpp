// rubato_link.hpp - THE TARGET-FREE HALF OF THE LINK SUITES UNDER RUBATO:
// what every family's test_<target>_rubato_link compiles unchanged around
// its own engines - the pack of seven objects, the numbered lines and
// their borrower, the spi_link.hpp window client with its command, query
// and report helpers, letter w's bookkeeping and the UART legs; and,
// through rubato_passes.hpp beside it (the waits and the passes every
// suite under Rubato shares), the base and wait turns, the passes and the
// pressure's verdict. A
// link suite runs a family's real SPI and UART stacks (util/spi_bus.hpp's
// arbiter over the family's SpiHost, util/line_reader.hpp's LineReader
// over its Uart) inside Rubato, the preemptive kernel, against a desk
// peer's unchanged spi_peer and usart_peer, while the pack's highest
// object preempts them at prime periods.
//
// APP-LEVEL BENCH TOOLING beside the protocol headers it speaks
// (spi_link.hpp, usart_link.hpp), included by relative path as they are;
// nothing in brio knows it exists, and nothing here names a register. The
// letters themselves stay in each suite: what a family's chapter changes
// in a verdict (the polled loop's limit, the matrix a peer can serve, the
// rates a pair of divisors allows) is that suite's.
//
// ONE BODY, MANY FAMILIES. What is a family's the including suite states in
// three places, all inside its own anonymous namespace with `using
// namespace brio` in force, having included kernel/rubato.hpp,
// kernel/borrowed.hpp, kernel/time_event.hpp, util/spi_bus.hpp,
// util/line_reader.hpp, util/print.hpp, util/testbench.hpp, spi_link.hpp,
// usart_link.hpp and <variant> and <span> at file scope.
//
// 1. BEFORE THE INCLUDE, these names, every one required:
//
//   core_hz, cycles_per_us   the core clock, in which the ruler counts
//   P                        the Rubato platform
//   Ruler                    Ruler::now(): core cycles, MONOTONE MODULO
//                            2^32 - the difference of two reads exact
//                            across any span a pass takes (seconds). A
//                            counter that restarts at a tick (the QingKe
//                            STK every millisecond) is not one; the
//                            ticker's composed count is.
//   spin(cycles)             a busy wait of at least `cycles` core cycles,
//                            for any span the suites ask (up to a few
//                            hundred thousand cycles: a borrower's spin, a
//                            timer's quieting); a preemption only lengthens
//                            it. wait_us() below is spin() in microseconds.
//   Serial, serial, bench    the console's Uart and its TwoKeyTestBench
//   Host, CsPin              the link's SpiHost with its engines, and the
//                            select, a GPIO. The body writes a Request's
//                            cs, tx, rx, len, polled, cs_setup_us and
//                            reply, and calls Host::prime(mode, rate) where
//                            it frames a select itself (cs_hold_us below);
//                            it asks the bit order through Host::lsb_first()
//                            and Host::bit_order(lsb) only where the host
//                            has them - a host without them clocks MSb
//                            first, and its suite posts no LSb window.
//   SpiClock, SpiDataSize    the host's rate and frame-size vocabulary:
//                            brio's where the family's driver names them,
//                            the suite's own otherwise (bits8 and bits16)
//   link_clock               the command channel's SpiClock
//   bus_clock_name           what the prints call the clock a SpiClock
//                            divides ("PCLK1")
//   LinkPort                 the link's Uart, the one the LineReader reads
//                            and the legs write through (write_bulk(),
//                            tx_idle())
//   PortCounters             the transport's counters over one leg, with
//                            clean() - what a family's Uart counts
//   pressure_name, peer_name what the prints call the pressure's timer
//                            ("TIM2") and the peer board ("the STM32G0B1RE")
//
// 2. BEFORE THE INCLUDE, OPTIONALLY, a `struct LinkTraits` whose static
// members change this body's defaults; a member it leaves out, or a
// LinkTraits it does not declare at all, keeps the default:
//
//   w_client_len, w_minor_len, w_press_len   (uint8_t; 6, 8 and 3) the
//                            frames of each of letter w's requests, so that
//                            a host whose engines take short runs can still
//                            put Pressure's on the pump and Minor's on the
//                            polled loop (at most 8 each)
//   w_rate                   (SpiClock; link_clock) letter w's rate, so
//                            that a host whose command rate keeps two
//                            frames in flight can run w where one is, and
//                            keep w about arbitration
//   cs_hold_us               (uint8_t; 0) the select held this long after a
//                            window's last frame before it rises: 0 leaves
//                            the select to the host's Request (its fall,
//                            then the protocol's 30 us, the frames, its rise
//                            at the end); any other value has the Client
//                            frame the select itself - the mode and rate
//                            primed with it high, its fall, 30 us, the
//                            request posted with no select, and at the
//                            reply the hold and the rise - for a peer whose
//                            client needs time after the last edge
//   bus_settings(r, rate, mode, bits)   (static void) a request's rate,
//                            clock mode and frame size written into a
//                            Host::Request `r` whose fields spell them
//                            otherwise (default: r.clock, r.mode, r.bits)
//
// 3. AFTER THE INCLUDE, the functions declared below under "What the
// including suite defines": the pressure's timer, the receive stream's
// position, the divisor's print, the polled command channel of the UART
// key, the Uart task's two edges and the reply hook.
//
// The vectors stay the suite's: its pressure vector clears its flag and
// calls pressure_landed() (or, where another mechanism makes the post,
// note_landing() alone), and its SPI, DMA and UART vectors post the
// arbiter's TransferDone and the port's RxActivity as the family's driver
// headers say.
//
// THE PACK, from the highest priority down:
//
//   Pressure  the preempting object: the family's timer posts to it every
//             97, 193 or 389 us (primes, against every period of the
//             traffic), and its dispatch spins 300 or 3000 cycles, so it
//             lands inside whatever the objects below were doing. It
//             counts its dispatches; the interrupt records where it
//             landed (the ceiling it found: which object's dispatch, the
//             walk of the time events with the ceiling held at 0 - a tick
//             there waits for the walk's end -, and whether the arbiter
//             was dispatching a completion at any depth).
//   LineSink  the borrower of the LineReader's lines (Lease::dispatch),
//             above the reader as the loan requires: each line parsed
//             against the sequence this file numbers and checks, gaps,
//             duplicates and corrupt lines counted, then written over in
//             place (the borrower may write what it is lent; a lender that
//             read it after the post would find the scribble).
//   Client    the SPI client: it takes one select window from the Console
//             (the Window event), posts the host's Request to the arbiter
//             with the select in it and the protocol's 30 us between the
//             select's fall and the first clock (or frames the select
//             itself, LinkTraits::cs_hold_us above), and on the reply
//             decodes the peer's checksummed answer frame (spi_link.hpp) out
//             of the bytes that came back. The answer windows are clocked
//             in the mode the peer answers in - mode 0, or mode 3 for a peer
//             that serves the protocol's alternate answer mode, found by the
//             rendezvous of sl::need_peer(). Every command window comes
//             settle_ms after the window before it, an answer window's
//             included (spi_link.hpp: "between one chip-select window and
//             the next"); the peer spends that time going dark again, and a
//             command frame clocked sooner after an answer window is not
//             heard.
//   SpiArb    util/spi_bus.hpp's SpiBus over the family's Host, pending
//             depth 4, a per-bus timeout of 100 ticks; a type derived from
//             the SpiBus whose dispatch notes the event kind around the
//             service's own dispatch - the one thing the landing trace
//             needs that the service does not say.
//   Minor     a second SPI client BELOW the arbiter (letter w).
//   LinePort  LineReader<LinkPort, P, LineSink, 80> over the family's
//             receiver.
//   Console   the lowest: it owns the console, runs the letters in its own
//             dispatch, and is the only object that prints. It is also the
//             link Uart's ONE TRANSMIT WRITER - the peer's command frames
//             on the polled resource between legs and the numbered lines
//             through the Uart's write_bulk() inside them: the object that
//             sequences the letters is the one that knows when the instance
//             is the command channel and when it is the Uart task, so no
//             hand-over of the transmitter exists to get wrong, and a
//             writer at the bottom of the pack never holds the port or the
//             sink off its receive side.
//
// A letter's waits are the Console's: each turn of a wait is Rubato's base
// turn written out (the matured time events posted with the ceiling held at
// 0, then the scheduler through the platform's entry and exit,
// rubato_passes.hpp's base_turn()), because the kernel loop is below the
// Console and does not turn while a letter runs - so the arbiter's per-bus
// timeout is a live clock here. The objects' tallies are the one memory the Console
// reads outside events: each written by one object, read by the Console
// after the traffic that wrote it has been answered.
//
// THE PASSES. Every letter runs its traffic seven times - a control with
// the pressure's timer silent, then six pressures - and every pressure
// pass judges the pressure itself; the passes, their arithmetic and the
// waits are rubato_passes.hpp's, whose header states them. This pack's
// marked state, the landing trace's `landed_marked`, is LineSink's
// dispatch on the stack.

#pragma once

// =============================================================================
// What the including suite defines (after the include)
// =============================================================================

/// The pressure's timer: armed for `period_us` with its update interrupt
/// on and the counter stopped; started; stopped (the counter alone, the
/// pass's wall read right after); and quietened (its interrupt off, its
/// flag and its line's pending bit cleared, the last tick's dispatch let
/// finish).
void pressure_arm(uint16_t period_us);
void pressure_go();
void pressure_halt();
void pressure_quiet();
/// The receive stream's position: any reading that changes when the
/// receiver writes and does not change while LineSink runs otherwise - a
/// receive channel's remaining count, an interrupt receiver's ring count
/// (its one consumer, the LineReader, is below LineSink), the link
/// vector's entry count. LineSink reads it at its entry and exit to say
/// whether the stream moved inside a dispatch.
uint32_t rx_stream_position();
/// A SpiClock's division of the bus clock, for the prints.
uint32_t division(SpiClock c);
/// Called in the Client's dispatch at every reply of a window (letter w's
/// excepted), before anything else runs on the bus: where the family's
/// host keeps a state worth reading there (a flag a later transaction
/// clears), the suite latches it here; otherwise it does nothing.
void client_replied();
/// The Uart task up at `rate` on the link's pads, its vectors live, and
/// down again (the instance given back).
bool task_up(uint32_t rate);
void task_down();
/// The transport's counters since the last task_up(), and their print.
PortCounters read_port_counters();
void print_port(const PortCounters& c);
void console_banner();

namespace ul {
/// The UART key's command channel, polled on the link's own instance
/// between legs (usart_link.hpp): an action commanded and acked, the peer's
/// window open settle_ms + 2 ms after its ack; and the peer's report.
bool peer_act(link::Op op, const link::Params& a);
bool peer_report(link::Report& r, uint16_t after_ms);
}  // namespace ul

// =============================================================================
// What the including suite may change: LinkTraits (the file header, 2)
// =============================================================================

/// The family's LinkTraits when it declared one before the include; when
/// it did not, this declaration names an incomplete type, which has none
/// of the members. Every member is read only where the type has it.
struct LinkTraits;

namespace link_traits {

template <typename T>
constexpr uint8_t w_client_len() {
    if constexpr (requires { T::w_client_len; }) {
        return T::w_client_len;
    } else {
        return 6;
    }
}
template <typename T>
constexpr uint8_t w_minor_len() {
    if constexpr (requires { T::w_minor_len; }) {
        return T::w_minor_len;
    } else {
        return 8;
    }
}
template <typename T>
constexpr uint8_t w_press_len() {
    if constexpr (requires { T::w_press_len; }) {
        return T::w_press_len;
    } else {
        return 3;
    }
}
template <typename T>
constexpr SpiClock w_rate() {
    if constexpr (requires { T::w_rate; }) {
        return T::w_rate;
    } else {
        return link_clock;
    }
}
template <typename T>
constexpr uint8_t cs_hold_us() {
    if constexpr (requires { T::cs_hold_us; }) {
        return T::cs_hold_us;
    } else {
        return 0;
    }
}

}  // namespace link_traits

/// The select's hold after a window (the file header, 2).
constexpr uint8_t cs_hold_us = link_traits::cs_hold_us<LinkTraits>();

/// A request's rate, clock mode and frame size, through the family's
/// LinkTraits::bus_settings() where it has one, else into the fields the
/// STM32 hosts name. A template so that the branch a host's Request cannot
/// compile is never instantiated.
template <typename R, typename T = LinkTraits>
void bus_settings(R& r, SpiClock rate, SpiMode mode, SpiDataSize bits) {
    if constexpr (requires { T::bus_settings(r, rate, mode, bits); }) {
        T::bus_settings(r, rate, mode, bits);
    } else {
        r.bits = bits;
        r.clock = rate;
        r.mode = mode;
    }
}

/// The bus's bit order set for a window, where the host has the verb (the
/// file header, 1); a host without it clocks MSb first.
template <typename H>
void window_bit_order(bool lsb) {
    if constexpr (requires { H::lsb_first(); H::bit_order(lsb); }) {
        if (H::lsb_first() != lsb) {
            (void)H::bit_order(lsb);
        }
    }
}

// =============================================================================
// The pack's shared vocabulary and the landing trace
// =============================================================================

struct Tick {};
struct Kick {};

/// The pack's indices, from the top (the Rubato list below).
constexpr uint8_t pressure_index = 0;
constexpr uint8_t sink_index = 1;
constexpr uint8_t client_index = 2;
constexpr uint8_t arb_index = 3;
constexpr uint8_t minor_index = 4;
constexpr uint8_t port_index = 5;
constexpr uint8_t console_index = 6;
constexpr uint8_t landing_slots = 8;   ///< the seven indices, and the loop's base (255)

/// Where the pressure's interrupt found the kernel: the ceiling it read,
/// by slot.
volatile uint32_t landed[landing_slots];
/// ... with the arbiter dispatching a TransferDone at any depth, and of those
/// one a DMA completion posted; and with LineSink's dispatch on the stack
/// (`landed_marked`: this pack's marked state, rubato_passes.hpp's name).
volatile uint32_t landed_completion = 0;
volatile uint32_t landed_dma_completion = 0;
volatile uint32_t landed_marked = 0;
volatile uint32_t pressure_irqs = 0;

/// The arbiter's event kind while its dispatch runs (SpiArb below).
constexpr uint8_t arb_none = 0;
constexpr uint8_t arb_request = 1;
constexpr uint8_t arb_done = 2;
constexpr uint8_t arb_other = 3;
volatile uint8_t arb_event = arb_none;
/// The source of the TransferDone the arbiter dispatches: true the DMA
/// engines' completion, false the SPI vector's (the pump's). The suite's
/// vectors write it before their post.
volatile bool done_from_dma = false;
/// LineSink's dispatch is running (it never nests in itself).
volatile uint8_t sink_depth = 0;

// =============================================================================
// The SPI requests of letter w, shared by its three requesters
// =============================================================================

namespace w {

constexpr uint8_t seed_a = 0x5A;   ///< what every window sends (pattern_fixed)
constexpr uint8_t seed_b = 0xC3;   ///< what the peer answers every frame
constexpr uint8_t untouched = 0xEE;

/// The three requesters' counts, and their lengths and the letter's rate
/// (the family's LinkTraits, the file header, 2).
constexpr uint8_t client_n = 20;
constexpr uint8_t client_len = link_traits::w_client_len<LinkTraits>();
constexpr uint8_t minor_n = 10;
constexpr uint8_t minor_len = link_traits::w_minor_len<LinkTraits>();
constexpr uint8_t press_n = 8;
constexpr uint8_t press_len = link_traits::w_press_len<LinkTraits>();
constexpr SpiClock rate = link_traits::w_rate<LinkTraits>();
static_assert(client_len >= 1u && client_len <= 8u && minor_len >= 1u && minor_len <= 8u && press_len >= 1u &&
                  press_len <= 8u,
              "letter w's requests are 1 to 8 frames, the out buffer below");

alignas(4) uint8_t tx[8] = {seed_a, seed_a, seed_a, seed_a, seed_a, seed_a, seed_a, seed_a};
alignas(4) uint8_t client_rx[client_n][client_len];
alignas(4) uint8_t minor_rx[minor_n][minor_len];
alignas(4) uint8_t press_rx[press_n][press_len];

volatile bool live = false;
volatile bool press_live = false;   ///< Pressure posts its requests (a pressure pass)

struct Tally {
    uint8_t posted = 0;
    uint8_t replies = 0;
    uint8_t ok = 0;
    uint8_t out_of_order = 0;
    uint8_t timeouts = 0;
    uint8_t rejected = 0;
};
volatile Tally client{};
volatile Tally minor{};
volatile Tally press{};

void clear(volatile Tally& t) {
    t.posted = 0;
    t.replies = 0;
    t.ok = 0;
    t.out_of_order = 0;
    t.timeouts = 0;
    t.rejected = 0;
}

bool complete(const uint8_t* slot, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
        if (slot[i] != seed_b) {
            return false;
        }
    }
    return true;
}
bool fresh(const uint8_t* slot, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
        if (slot[i] != untouched) {
            return false;
        }
    }
    return true;
}

/// One of letter w's transactions. THE SELECT IS THE LETTER'S, held low
/// across the whole exchange: the peer's exchange is one burst, so every
/// requester's transaction is a stretch of one window, and every one runs
/// at the window's mode and rate, w::rate, at which the suite primes the
/// host before it lowers the select - a rate change is a disable and an
/// enable of the block, which a selected client would see.
Host::Request request(uint8_t* rx, uint8_t len, bool polled, ReplyTo<SpiDone> reply) {
    Host::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.polled = polled;
    r.cs_setup_us = 0;
    bus_settings(r, rate, SpiMode::mode0, SpiDataSize::bits8);
    r.reply = reply;
    return r;
}

/// One reply's bookkeeping: the status, and whether the requester's window
/// k is complete (and, `next_fresh`, its window k + 1 untouched).
void note(volatile Tally& t, const SpiDone& d, bool in_order) {
    if (d.status == spi_ok) {
        t.ok = static_cast<uint8_t>(t.ok + 1u);
    } else if (d.status == spi_timeout) {
        t.timeouts = static_cast<uint8_t>(t.timeouts + 1u);
    } else if (d.status == spi_rejected) {
        t.rejected = static_cast<uint8_t>(t.rejected + 1u);
    }
    if (!in_order) {
        t.out_of_order = static_cast<uint8_t>(t.out_of_order + 1u);
    }
    t.replies = static_cast<uint8_t>(t.replies + 1u);
}

}  // namespace w

// =============================================================================
// SpiArb: the arbiter, its event kind noted for the trace
// =============================================================================

/// The bus: pending depth 4 (the three requesters of letter w keep at most
/// four outstanding: one in flight, three waiting), and a per-bus timeout of
/// 100 ticks - 100 ms, two orders of magnitude above the longest
/// transaction a letter posts (a few milliseconds at the command rate).
using ArbBase = SpiBus<Host, P, 4, BusPassThrough, 100>;

/// The service as it is; the pack names this type so that its dispatch can
/// say, for the landing trace, which kind of event the service is
/// dispatching. The queue, init() and every verb are the SpiBus's.
struct SpiArb : ArbBase {
    static void dispatch(const Event& e) {
        arb_event = std::holds_alternative<TransferDone>(e)    ? arb_done
                    : std::holds_alternative<Host::Request>(e) ? arb_request
                                                               : arb_other;
        ArbBase::dispatch(e);
        arb_event = arb_none;
    }
};

// =============================================================================
// Pressure
// =============================================================================

volatile uint32_t pressure_spin = 0;

struct Pressure {
    using Event = std::variant<Tick, SpiDone>;
    static inline EventQueue<Event, 4, P> queue;
    static inline volatile uint32_t runs = 0;
    static inline volatile bool outstanding = false;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e,
            [](const Tick&) {
                runs = runs + 1u;
                if (w::press_live && !outstanding && w::press.posted < w::press_n) {
                    outstanding = true;
                    const uint8_t k = w::press.posted;
                    w::press.posted = static_cast<uint8_t>(k + 1u);
                    post<SpiArb>(w::request(w::press_rx[k], w::press_len, false,
                                            reply_to<Pressure, SpiDone>()));
                }
                spin(pressure_spin);
            },
            [](const SpiDone& d) {
                const uint8_t k = w::press.replies;
                w::note(w::press, d, k < w::press_n && w::complete(w::press_rx[k], w::press_len));
                outstanding = false;
            });
    }
};

/// Where one of the pressure's ticks found the kernel, recorded, without
/// the post: for a source whose post another mechanism makes (a
/// cross-core inbox's drain). Interrupt context.
[[gnu::always_inline]] inline void note_landing() {
    const uint8_t c = P::ceiling;
    const uint8_t slot = c < landing_slots - 1u ? c : static_cast<uint8_t>(landing_slots - 1u);
    landed[slot] = landed[slot] + 1u;
    if (arb_event == arb_done) {
        landed_completion = landed_completion + 1u;
        if (done_from_dma) {
            landed_dma_completion = landed_dma_completion + 1u;
        }
    }
    if (sink_depth != 0u) {
        landed_marked = landed_marked + 1u;
    }
    pressure_irqs = pressure_irqs + 1u;
}

/// The pressure vector's body after the timer's flag is cleared: where it
/// landed, and the post. Interrupt context.
[[gnu::always_inline]] inline void pressure_landed() {
    note_landing();
    post<Pressure>(Tick{});
}

// =============================================================================
// LineSink: the borrower of the port's lines
// =============================================================================

/// Line `seq`, without its end of line: 'L', five digits, ':', a payload of
/// 8 to 60 characters from 'A' to 'z' (never ':', '\r' or '\n'), ':', and
/// the 8-bit sum of everything before it in two hex digits - 70 characters
/// at most, inside the assembler's 79.
constexpr uint8_t line_most = 72;
constexpr uint8_t payload_len(uint16_t seq) { return static_cast<uint8_t>(8u + (static_cast<uint32_t>(seq) * 13u) % 53u); }
constexpr char payload_char(uint16_t seq, uint8_t i) {
    return static_cast<char>('A' + (static_cast<uint32_t>(seq) * 7u + static_cast<uint32_t>(i) * 3u) % 58u);
}
constexpr char hex_digit(uint8_t v) { return static_cast<char>(v < 10u ? '0' + v : 'A' + (v - 10u)); }

uint8_t make_line(uint16_t seq, char* out) {
    uint8_t n = 0;
    out[n++] = 'L';
    uint32_t v = seq;
    for (int i = 4; i >= 0; --i) {
        out[n + static_cast<uint8_t>(i)] = static_cast<char>('0' + v % 10u);
        v /= 10u;
    }
    n = static_cast<uint8_t>(n + 5u);
    out[n++] = ':';
    const uint8_t len = payload_len(seq);
    for (uint8_t i = 0; i < len; ++i) {
        out[n++] = payload_char(seq, i);
    }
    out[n++] = ':';
    uint8_t sum = 0;
    for (uint8_t i = 0; i < n; ++i) {
        sum = static_cast<uint8_t>(sum + static_cast<uint8_t>(out[i]));
    }
    out[n++] = hex_digit(static_cast<uint8_t>(sum >> 4));
    out[n++] = hex_digit(static_cast<uint8_t>(sum & 0x0Fu));
    return n;
}

/// The line parsed: its number, when every character is what make_line()
/// writes for that number.
bool parse_line(const char* s, uint16_t& seq) {
    if (s[0] != 'L') {
        return false;
    }
    uint32_t v = 0;
    for (uint8_t i = 1; i <= 5; ++i) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
        v = v * 10u + static_cast<uint32_t>(s[i] - '0');
    }
    if (s[6] != ':' || v > 0xFFFFu) {
        return false;
    }
    const uint16_t q = static_cast<uint16_t>(v);
    const uint8_t len = payload_len(q);
    for (uint8_t i = 0; i < len; ++i) {
        if (s[7u + i] != payload_char(q, i)) {
            return false;
        }
    }
    const uint8_t at = static_cast<uint8_t>(7u + len);
    if (s[at] != ':') {
        return false;
    }
    uint8_t sum = 0;
    for (uint8_t i = 0; i <= at; ++i) {
        sum = static_cast<uint8_t>(sum + static_cast<uint8_t>(s[i]));
    }
    if (s[at + 1u] != hex_digit(static_cast<uint8_t>(sum >> 4)) ||
        s[at + 2u] != hex_digit(static_cast<uint8_t>(sum & 0x0Fu)) || s[at + 3u] != '\0') {
        return false;
    }
    seq = q;
    return true;
}

struct LineSink {
    using Event = std::variant<LineReceived>;
    /// Two LineReceived in flight at most (line_reader.hpp) and room beside.
    static inline EventQueue<Event, 4, P> queue;

    // The tally, written here, read by the Console between legs.
    static inline volatile uint32_t lines = 0;      ///< every LineReceived
    static inline volatile uint32_t good = 0;
    static inline volatile uint32_t empties = 0;    ///< the leg's leading end of line
    static inline volatile uint32_t corrupt = 0;
    static inline volatile uint32_t gaps = 0;       ///< jumps forward
    static inline volatile uint32_t lost = 0;       ///< the lines the jumps skipped
    static inline volatile uint32_t backwards = 0;  ///< duplicates or out of order
    static inline volatile uint32_t waiting = 0;    ///< another line already queued at entry
    static inline volatile uint32_t same_buffer = 0;   ///< a line in the buffer of the one before
    static inline volatile uint32_t nested = 0;
    static inline volatile uint32_t moved = 0;      ///< the receive stream wrote during the dispatch
    static inline volatile uint32_t busy = 0;       ///< cycles inside the dispatches
    static inline volatile uint32_t spin_cycles = 0;
    /// The leg was STREAMED (echo_leg()'s window): its premise is the
    /// stream moving inside a loan, which print_sink() reports.
    static inline bool streamed = false;
    static inline uint16_t next = 0;
    static inline const char* last = nullptr;
    static inline const char* buffers[2] = {nullptr, nullptr};
    static inline volatile uint8_t distinct = 0;

    static void reset(uint16_t first) {
        lines = 0;
        good = 0;
        empties = 0;
        corrupt = 0;
        gaps = 0;
        lost = 0;
        backwards = 0;
        waiting = 0;
        same_buffer = 0;
        nested = 0;
        moved = 0;
        busy = 0;
        next = first;
        last = nullptr;
    }

    static void init() {}
    static void dispatch(const Event& e) {
        match(e, [](const LineReceived& l) { take(l.line.get()); });
    }

private:
    static void take(char* s) {
        const uint32_t t0 = Ruler::now();
        if (sink_depth != 0u) {
            nested = nested + 1u;
        }
        sink_depth = static_cast<uint8_t>(sink_depth + 1u);
        const uint32_t at = rx_stream_position();
        if (!queue.empty()) {
            waiting = waiting + 1u;
        }
        if (s == last) {
            same_buffer = same_buffer + 1u;
        }
        last = s;
        if (s != buffers[0] && s != buffers[1]) {
            buffers[distinct & 1u] = s;
            distinct = static_cast<uint8_t>(distinct + 1u);
        }
        lines = lines + 1u;
        if (s[0] == '\0') {
            empties = empties + 1u;
        } else {
            uint16_t seq = 0;
            if (!parse_line(s, seq)) {
                corrupt = corrupt + 1u;
            } else if (seq == next) {
                good = good + 1u;
                next = static_cast<uint16_t>(seq + 1u);
            } else if (seq > next) {
                gaps = gaps + 1u;
                lost = lost + (seq - next);
                good = good + 1u;
                next = static_cast<uint16_t>(seq + 1u);
            } else {
                backwards = backwards + 1u;
            }
        }
        spin(spin_cycles);
        // The loan lets the borrower write the line: written over, so that
        // a lender reading it after the post would read this.
        for (char* p = s; *p != '\0'; ++p) {
            *p = 'X';
        }
        if (rx_stream_position() != at) {
            moved = moved + 1u;
        }
        sink_depth = static_cast<uint8_t>(sink_depth - 1u);
        busy = busy + (Ruler::now() - t0);
    }
};

// =============================================================================
// Client: the SPI client of the letters n, o and p (and letter w's first)
// =============================================================================

/// One select window, the Console's: the buffers LENT until the Client says
/// it is done (Client::done moves), the bus's settings for this window.
struct Window {
    Borrowed<const uint8_t, Lease::reply> tx;
    Borrowed<uint8_t, Lease::reply> rx;
    uint16_t len = 0;
    SpiClock clock = link_clock;
    SpiMode mode = SpiMode::mode0;
    SpiDataSize bits = SpiDataSize::bits8;
    bool polled = false;
    bool lsb = false;
    bool decode = false;   ///< look for a spi_link frame in what came back
};

struct Client {
    using Event = std::variant<Window, SpiDone, Kick>;
    static inline EventQueue<Event, 8, P> queue;

    static inline volatile uint32_t done = 0;    ///< windows answered
    static inline volatile uint8_t status = spi_ok;
    static inline volatile uint32_t cycles = 0;  ///< the last window, post to reply
    static inline uint32_t t_post = 0;
    static inline uint8_t* rx = nullptr;
    static inline uint16_t rx_len = 0;
    static inline bool decode = false;
    static inline volatile bool found = false;   ///< the last answer window held a frame
    static inline spilink::Frame frame{};
    static inline uint8_t raw[16];
    static inline uint8_t raw_n = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e,
            [](const Window& win) {
                // The bit order is a bus-level verb (SpiHost::bit_order), and
                // the bus is idle: the Console posts a window only after the
                // last one was answered, and in these letters the Client is the
                // bus's one requester.
                window_bit_order<Host>(win.lsb);
                rx = win.rx.get();
                rx_len = win.len;
                decode = win.decode;
                Host::Request r{};
                if constexpr (cs_hold_us == 0u) {
                    r.cs = CsPin::ref();
                }
                r.tx = win.tx;
                r.rx = win.rx;
                r.len = win.len;
                r.polled = win.polled;
                if constexpr (cs_hold_us == 0u) {
                    // The 30 us the protocol pays between the select's fall
                    // and the first clock.
                    r.cs_setup_us = 30;
                } else {
                    // The Client frames the select (the file header, 2): the
                    // mode and rate primed with it high - a CPOL flip inside
                    // an open window is an edge for the client -, its fall,
                    // the protocol's 30 us; the request carries no select.
                    Host::prime(win.mode, win.clock);
                    CsPin::clear();
                    spin(30u * cycles_per_us);
                }
                bus_settings(r, win.clock, win.mode, win.bits);
                r.reply = reply_to<Client, SpiDone>();
                t_post = Ruler::now();
                post<SpiArb>(r);
            },
            [](const SpiDone& d) {
                if (w::live) {
                    const uint8_t k = w::client.replies;
                    const bool in_order = k < w::client_n && w::complete(w::client_rx[k], w::client_len) &&
                                          (k + 1u >= w::client_n || w::fresh(w::client_rx[k + 1u], w::client_len));
                    w::note(w::client, d, in_order);
                    post_next_w();
                    return;
                }
                cycles = Ruler::now() - t_post;
                client_replied();
                if constexpr (cs_hold_us != 0u) {
                    // The select the Client framed: held, then raised.
                    spin(static_cast<uint32_t>(cs_hold_us) * cycles_per_us);
                    CsPin::set();
                }
                status = d.status;
                found = false;
                if (decode && rx != nullptr) {
                    judge_answer();
                }
                done = done + 1u;
            },
            [](const Kick&) {
                post_next_w();
                post_next_w();
            });
    }

private:
    /// The peer's answer frame out of the bytes that came back (the peer
    /// pads with zeros), and the first sixteen for a failure's print.
    static void judge_answer() {
        spilink::Decoder dec;
        raw_n = 0;
        for (uint16_t i = 0; i < rx_len; ++i) {
            if (raw_n < sizeof raw) {
                raw[raw_n++] = rx[i];
            }
            if (dec.feed(rx[i]) == spilink::Decoder::Result::frame) {
                frame = dec.frame();
                found = true;
                return;
            }
        }
    }

    static void post_next_w() {
        const uint8_t k = w::client.posted;
        if (k >= w::client_n) {
            return;
        }
        w::client.posted = static_cast<uint8_t>(k + 1u);
        post<SpiArb>(w::request(w::client_rx[k], w::client_len, false,
                                reply_to<Client, SpiDone>()));
    }
};

// =============================================================================
// Minor: letter w's requester below the arbiter
// =============================================================================

struct Minor {
    using Event = std::variant<Kick, SpiDone>;
    static inline EventQueue<Event, 4, P> queue;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e, [](const Kick&) { post_next(); },
            [](const SpiDone& d) {
                const uint8_t k = w::minor.replies;
                bool in_order = k < w::minor_n;
                for (uint8_t i = 0; in_order && i <= k; ++i) {
                    in_order = w::complete(w::minor_rx[i], w::minor_len);
                }
                w::note(w::minor, d, in_order);
                post_next();
            });
    }

private:
    /// POLLED at letter w's rate: the request completes inside the
    /// arbiter's dispatch, which runs inside this post (the arbiter is
    /// above Minor).
    static void post_next() {
        const uint8_t k = w::minor.posted;
        if (k >= w::minor_n) {
            return;
        }
        w::minor.posted = static_cast<uint8_t>(k + 1u);
        post<SpiArb>(w::request(w::minor_rx[k], w::minor_len, true,
                                reply_to<Minor, SpiDone>()));
    }
};

// =============================================================================
// LinePort, Console, the kernel
// =============================================================================

using LinePort = LineReader<LinkPort, P, LineSink, 80>;

struct ConsoleEvent {
    uint8_t kind;
};
constexpr uint8_t console_rx = 0;

void console_turn();

struct Console {
    using Event = ConsoleEvent;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {
        console_banner();
        bench.prompt();
    }
    static void dispatch(const Event& e) {
        if (e.kind == console_rx) {
            console_turn();
        }
    }
};

using K = Rubato<P, Pressure, LineSink, Client, SpiArb, Minor, LinePort, Console>;
using TE = TimeEvents<P>;

#include "rubato_passes.hpp"

/// The pass's pressure, printed with where it landed.
void print_pass(const PassSpec& s, const PassStats& st) {
    print(serial, "  [", pass_label(s), "] ", st.wall / (core_hz / 1000u), " ms");
    if (s.period_us == 0u) {
        print(serial, ", ", pressure_name, " silent", crlf);
        return;
    }
    print(serial, ", ", st.irqs, " ticks, ", st.runs, " dispatches, ", st.overflows,
          " dropped; landed in the walk (ceiling 0) ", st.land[pressure_index], ", LineSink ",
          st.land[sink_index], ", Client ", st.land[client_index], ", SpiArb ", st.land[arb_index],
          ", Minor ", st.land[minor_index], ", Port ", st.land[port_index], ", Console ",
          st.land[console_index], ", base ", st.land[landing_slots - 1u], crlf);
    print(serial, "    inside a completion's dispatch (any depth) ", st.completion, ", of a DMA "
          "completion ", st.dma_completion, "; LineSink on the stack ", st.marked, crlf);
}

// =============================================================================
// The SPI link: spi_link.hpp through the Client and the arbiter
// =============================================================================

namespace sl {

using spilink::Op;

uint8_t frame_buf[spilink::max_payload + 8];
alignas(4) uint8_t answer_buf[spilink::answer_bytes];
alignas(4) uint8_t dummy_buf[spilink::answer_bytes];
bool quiet = false;
/// The mode the answer windows are clocked in (spi_link.hpp, "THE
/// ALTERNATE ANSWER MODE"): mode 0 until need_peer()'s rendezvous finds the
/// peer answering in mode 3, then mode 3 for as long as it answers there.
/// The command windows stay mode 0.
SpiMode answer_mode = SpiMode::mode0;

/// One select window through the Client and the arbiter, waited for (at
/// most 500 ms, two orders above the longest window a letter clocks). True
/// when it was answered spi_ok.
bool window(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
            SpiMode mode = SpiMode::mode0, SpiDataSize bits = SpiDataSize::bits8, bool polled = false,
            bool lsb = false, bool decode = false) {
    if (n == 0u) {
        return true;
    }
    const uint32_t before = Client::done;
    Window win{};
    win.tx = lend<Lease::reply>(tx);
    win.rx = lend<Lease::reply>(rx);
    win.len = n;
    win.clock = rate;
    win.mode = mode;
    win.bits = bits;
    win.polled = polled;
    win.lsb = lsb;
    win.decode = decode;
    post<Client>(win);
    if (!wait_until([before] { return Client::done != before; }, 500)) {
        return false;
    }
    return Client::status == spi_ok;
}

void settle() { wait_ms(spilink::settle_ms); }

/// A command window, settle_ms after whatever window came before it:
/// spi_link.hpp's "between one chip-select window and the next", which the
/// peer spends going dark again after an answer window - a command frame
/// clocked sooner is not heard (measured: every first attempt lost).
void send_frame(Op op, const uint8_t* p, uint8_t len) {
    settle();
    uint8_t n = 0;
    spilink::write_frame(
        [&n](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    (void)window(frame_buf, nullptr, n);
}

/// One answer window: answer_bytes dummies in the peer's answer mode, the
/// Client decoding the frame.
bool recv_frame(spilink::Frame& out) {
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        dummy_buf[i] = 0;
        answer_buf[i] = 0xEE;
    }
    if (!window(dummy_buf, answer_buf, spilink::answer_bytes, link_clock, answer_mode, SpiDataSize::bits8, false,
                false, true)) {
        return false;
    }
    if (!Client::found) {
        return false;
    }
    out = Client::frame;
    return true;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, each 400 ms after the last - longer than the peer's own
/// answer window bound (the protocol's recovery guarantee).
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len)) {
            return true;
        }
        if (!quiet) {
            print(serial, "    (op ", hex(spilink::byte_of(op)), " attempt ", k, ": status ",
                  Client::status, ", the window carried");
            for (uint8_t i = 0; i < Client::raw_n; ++i) {
                print(serial, " ", hex(Client::raw[i]));
            }
            print(serial, ")", crlf);
        }
        wait_ms(400);
    }
    return false;
}

bool query(Op op, spilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle();
    return recv_frame(data);
}

bool peer_ident(spilink::Ident& d) {
    spilink::Frame f;
    if (!query(Op::ident, f) || f.op != Op::ident_data || f.len != spilink::ident_size) {
        return false;
    }
    d = spilink::get_ident(f.data);
    return true;
}

bool peer_report(spilink::Report& r) {
    for (uint8_t k = 0; k < 4u; ++k) {
        spilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        wait_ms(60);
    }
    return false;
}

bool peer_act(Op op, const spilink::Params& a) {
    uint8_t p[spilink::params_size];
    spilink::put_params(p, a);
    if (!command(op, p, spilink::params_size)) {
        return false;
    }
    settle();
    return true;
}

/// Is the peer there? The protocol's rendezvous: a ping in the answer mode
/// kept, and after a ping that failed in one mode the next in the other, so
/// a peer that answers in mode 3 alone is found at the second; a named
/// failure, never a hang.
bool need_peer() {
    quiet = true;
    bool ok = false;
    for (uint8_t k = 0; k < 3u && !ok; ++k) {
        ok = command(Op::ping);
        if (!ok) {
            answer_mode = answer_mode == SpiMode::mode0 ? SpiMode::mode3 : SpiMode::mode0;
        }
    }
    quiet = false;
    if (!ok) {
        print(serial, "  THE PEER DID NOT ANSWER: ", peer_name, " must be running `spi_peer` "
                      "(its console '0' forces the dark client back)",
              crlf);
        bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    }
    return ok;
}

// ---- one exchange, commanded, clocked, reported ----------------------------------------

constexpr uint16_t cap = 64;   ///< frames a window carries at most
alignas(4) uint8_t xtx[2u * cap + 4u];
alignas(4) uint8_t xrx[2u * cap + 4u];

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    uint16_t count = 32;     ///< frames
    bool wide = false;
    bool polled = false;
    bool odd = false;        ///< the buffers at an odd address: a 16-bit window off the engines
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
};

struct Outcome {
    bool accepted = false;
    bool clocked = false;
    bool reported = false;
    uint8_t status = spi_ok;
    spilink::Report r{};
    uint16_t mism = 0;
    uint16_t idx = 0xFFFF;
    uint8_t got = 0, exp = 0;
    uint32_t cycles = 0;     ///< the data window, post to reply
};

bool reversed(const Exchange& e) { return e.host_lsb != (e.cfg.dord != 0u); }

bool exact(const Exchange& e, const Outcome& o) {
    return o.accepted && o.clocked && o.reported && o.r.count == e.count && o.r.mism == 0u &&
           o.mism == 0u && (!e.wide || (o.r.flags & spilink::report_extended) != 0u);
}

Outcome run(const Exchange& e) {
    Outcome o{};
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = 400;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = e.wide ? spilink::spare_wide : 0u;
    if (reversed(e)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    const uint16_t n = e.count < cap ? e.count : cap;
    const uint16_t bytes = static_cast<uint16_t>(e.wide ? 2u * n : n);
    uint8_t* const tx = xtx + (e.odd ? 1u : 0u);
    uint8_t* const rx = xrx + (e.odd ? 1u : 0u);
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < bytes; ++i) {
            tx[i] = out.next();
            rx[i] = 0xEE;
        }
    }
    settle();
    o.clocked = window(tx, rx, n, e.rate, e.host_mode, e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                       e.polled, e.host_lsb);
    o.status = Client::status;
    o.cycles = Client::cycles;
    o.reported = peer_report(o.r);
    for (uint16_t i = 0; i < bytes; ++i) {
        const uint8_t want = spilink::wide_byte(a.pattern, a.seed_b, i, e.wide, reversed(e));
        if (rx[i] != want) {
            if (o.mism == 0u) {
                o.idx = i;
                o.got = rx[i];
                o.exp = want;
            }
            ++o.mism;
        }
    }
    return o;
}

void print_outcome(const Exchange& e, const Outcome& o) {
    print(serial, "    mode ", static_cast<uint8_t>(e.host_mode), e.host_lsb ? " LSb" : " MSb",
          e.cfg.dord != 0u ? " (peer LSb)" : "", e.wide ? " 16-bit" : " 8-bit", " ", bus_clock_name, "/",
          division(e.rate), ": status ", o.status, " host mism=", o.mism);
    if (o.mism != 0u) {
        print(serial, " (byte ", o.idx, " got ", hex(o.got), " exp ", hex(o.exp), ")");
    }
    print(serial, " peer count=", o.r.count, " mism=", o.r.mism);
    if (o.r.mism != 0u) {
        print(serial, " (byte ", o.r.idx, " got ", hex(o.r.got), " exp ", hex(o.r.exp), ")");
    }
    print(serial, " flags=", hex(o.r.flags), o.accepted ? "" : " NOT ACCEPTED",
          o.reported ? "" : " NO REPORT", crlf);
}

}  // namespace sl

// =============================================================================
// The UART legs
// =============================================================================

/// What a leg's receive side counted: the transport's counters (the
/// family's) and the line assembler's, as deltas over the leg.
struct LegCounters {
    PortCounters port{};
    uint8_t line_overflows = 0;
    uint32_t torn = 0;
    bool clean() const { return port.clean() && line_overflows == 0u && torn == 0u; }
};

// The functions below serve every UART key a family may run; a family whose
// peer has no report (or no sink) leaves some of them uncalled, so each
// carries [[maybe_unused]] - they are functions of an unnamed namespace,
// which -Wunused-function would otherwise name.

[[maybe_unused]] void print_counters(const LegCounters& c) {
    print(serial, "    port: ");
    print_port(c.port);
    print(serial, ", line overflows ", c.line_overflows, ", torn lines ", c.torn, crlf);
}

char line_buf[line_most];

/// One line out through `Port`'s write_bulk(), the ring topped up as it
/// drains. False when the transmitter stopped taking bytes for 50 ms. A
/// family with two transmitters on the link's instance (an interrupt one
/// and a DMA one, two Uart types) names the leg's.
template <typename Port = LinkPort>
bool write_line(const char* s, uint8_t n) {
    uint32_t sent = 0;
    const uint32_t t0 = Ticker::millis();
    while (sent < n) {
        sent += Port::write_bulk(std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(s) + sent, n - sent));
        if (sent < n && Ticker::millis() - t0 >= 50u) {
            return false;
        }
    }
    return true;
}

struct LegResult {
    bool acted = false, up = false, wrote = false, have = false;
    uint32_t bytes = 0;
    uint16_t sum = 0;
    uint32_t cycles = 0;      ///< the first write to the last line's arrival (t, v) or to tx_idle (u)
    LegCounters c{};
    link::Report r{};
};

[[maybe_unused]] uint8_t line_with_eol(uint16_t seq) {
    const uint8_t n = make_line(seq, line_buf);
    line_buf[n] = '\n';
    return static_cast<uint8_t>(n + 1u);
}

/// How echo_leg()'s writer spaces its characters. WHOLE (echo_leg()
/// without a Pace): each line written whole, for a peer whose echo keeps
/// pace with a line back to back. PACED: `burst` characters, then `burst` x
/// gap_bits bit times before the next burst (a floor: a preemption of the
/// writer only lengthens a gap), the bursts running across the lines' ends
/// - for a peer whose echo turns each character around alone and overruns
/// on a run written back to back. A paced leg's time is set by the two
/// ends' per-character work, not the wire, so the peer's deadline is
/// `char_us` a character and a second; a whole leg's is twice the wire's
/// and half a second.
struct Pace {
    uint8_t gap_bits;       ///< the spacing, in bit times a character
    uint8_t burst = 1;      ///< 1 or 2 characters back to back
    uint16_t char_us;       ///< the peer's deadline, a character
};

/// t and v's leg: `lines` numbered lines from `seq0` through the peer's echo
/// at `rate`, the leg's first byte an end of line that empties the
/// assembler; then the port's counters and the peer's. LINE BY LINE (t,
/// `window` 0, the lines whole): each line written after the one before has
/// left this end's wire and two frame times more. STREAMED (v, `window`
/// lines): line k written as soon as line k - window has come back to the
/// sink, so the characters keep arriving while the port drains and the sink
/// borrows; the window bounds what the peer can hold of a backlog (the
/// including suite's arithmetic). `pace`, in the second form, spaces the
/// characters themselves (Pace above), in either shape. The writer's two
/// loops are one template's two instantiations, so an image that never
/// paces carries only the whole one.
template <bool paced>
LegResult echo_leg_as(uint32_t rate, uint16_t lines, uint16_t seq0, uint8_t window, const Pace& pace) {
    LegResult g{};
    for (uint16_t k = 0; k < lines; ++k) {
        g.bytes += line_with_eol(static_cast<uint16_t>(seq0 + k));
    }
    g.bytes += 1u;   // the leading end of line
    link::Params a{};
    a.cfg.rate = rate;
    a.count = static_cast<uint16_t>(g.bytes);
    if constexpr (paced) {
        a.ms = static_cast<uint16_t>(g.bytes * pace.char_us / 1000u + 1000u);
    } else {
        a.ms = static_cast<uint16_t>(g.bytes * 10u * 1000u * 2u / rate + 500u);
    }
    g.acted = ul::peer_act(link::Op::echo, a);
    if (!g.acted) {
        return g;
    }
    LineSink::reset(seq0);
    LineSink::streamed = window != 0u;
    const uint8_t overflows0 = LinePort::line_overflows();
    const uint32_t torn0 = LinePort::torn_lines();
    g.up = task_up(rate);
    if (!g.up) {
        return g;
    }
    wait_ms(6);   // the peer's echo window opens settle_ms + 2 ms after its ack
    const uint32_t t0 = Ruler::now();
    if constexpr (!paced) {
        const uint32_t gap = 2u * 10u * (core_hz / rate);
        g.wrote = write_line("\n", 1);
        for (uint16_t k = 0; k < lines && g.wrote; ++k) {
            if (window != 0u && k >= window) {
                const uint32_t back = static_cast<uint32_t>(k - window) + 1u;
                (void)wait_until([back] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= back; },
                                 20);
            }
            const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
            g.wrote = write_line(line_buf, n);
            if (window == 0u) {
                const uint32_t m0 = Ticker::millis();
                while (!LinkPort::tx_idle() && Ticker::millis() - m0 < 50u) {
                }
                spin(gap);
            }
        }
    } else {
        const uint8_t burst = pace.burst == 2u ? 2u : 1u;
        const uint32_t gap = static_cast<uint32_t>(pace.gap_bits) * burst * (core_hz / rate);
        // A burst may run across a line's end - the end of line goes out
        // with the next line's first character -, so a character is on its
        // way while the line before it is lent; the last one flushed at the
        // end.
        char held[2] = {};
        uint8_t held_n = 0;
        auto put = [&held, &held_n, gap, burst](const char* s, uint8_t n) {
            for (uint8_t i = 0; i < n; ++i) {
                held[held_n++] = s[i];
                if (held_n == burst) {
                    held_n = 0;
                    if (!write_line(held, burst)) {
                        return false;
                    }
                    spin(gap);
                }
            }
            return true;
        };
        g.wrote = put("\n", 1);
        for (uint16_t k = 0; k < lines && g.wrote; ++k) {
            if (window != 0u && k >= window) {
                const uint32_t back = static_cast<uint32_t>(k - window) + 1u;
                (void)wait_until([back] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= back; },
                                 20);
            }
            const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
            g.wrote = put(line_buf, n);
        }
        if (g.wrote && held_n != 0u) {
            g.wrote = write_line(held, held_n);
        }
    }
    (void)wait_until([lines] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= lines; }, 300);
    g.cycles = Ruler::now() - t0;
    wait_ms(5);   // the last edge's dispatch, and anything a slip would still deliver
    g.c.port = read_port_counters();
    g.c.line_overflows = static_cast<uint8_t>(LinePort::line_overflows() - overflows0);
    g.c.torn = LinePort::torn_lines() - torn0;
    task_down();
    g.have = ul::peer_report(g.r, 0);
    return g;
}

[[maybe_unused]] LegResult echo_leg(uint32_t rate, uint16_t lines, uint16_t seq0, uint8_t window = 0) {
    return echo_leg_as<false>(rate, lines, seq0, window, Pace{});
}
[[maybe_unused]] LegResult echo_leg(uint32_t rate, uint16_t lines, uint16_t seq0, uint8_t window, const Pace& pace) {
    return echo_leg_as<true>(rate, lines, seq0, window, pace);
}

/// u's leg: `lines` numbered lines back to back into the peer's sink,
/// through `Port`'s transmitter (write_line()'s).
template <typename Port = LinkPort>
LegResult sink_leg(uint32_t rate, uint16_t lines, uint16_t seq0) {
    LegResult g{};
    for (uint16_t k = 0; k < lines; ++k) {
        const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
        g.bytes += n;
        for (uint8_t i = 0; i < n; ++i) {
            g.sum = static_cast<uint16_t>(g.sum + static_cast<uint8_t>(line_buf[i]));
        }
    }
    link::Params a{};
    a.cfg.rate = rate;
    a.count = static_cast<uint16_t>(g.bytes);
    a.ms = static_cast<uint16_t>(g.bytes * 10u * 1000u * 2u / rate + 500u);
    g.acted = ul::peer_act(link::Op::sink, a);
    if (!g.acted) {
        return g;
    }
    g.up = task_up(rate);
    if (!g.up) {
        return g;
    }
    wait_ms(6);
    const uint32_t t0 = Ruler::now();
    g.wrote = true;
    for (uint16_t k = 0; k < lines && g.wrote; ++k) {
        const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
        g.wrote = write_line<Port>(line_buf, n);
    }
    const uint32_t m0 = Ticker::millis();
    while (!Port::tx_idle() && Ticker::millis() - m0 < 100u) {
    }
    g.cycles = Ruler::now() - t0;
    task_down();
    g.have = ul::peer_report(g.r, 0);
    return g;
}

/// The wire's time for `bytes` 8N1 characters at `rate`, in core cycles.
[[maybe_unused]] uint32_t wire_cycles(uint32_t bytes, uint32_t rate) {
    return static_cast<uint32_t>((static_cast<uint64_t>(bytes) * 10u * core_hz) / rate);
}

[[maybe_unused]] void print_leg(const char* what, uint32_t rate, const LegResult& g) {
    print(serial, "    ", what, " at ", rate, ": ", g.bytes, " bytes in ", g.cycles / cycles_per_us, " us (the wire's ",
          wire_cycles(g.bytes, rate) / cycles_per_us, " us, ",
          g.cycles != 0u ? static_cast<uint32_t>(static_cast<uint64_t>(wire_cycles(g.bytes, rate)) * 1000u / g.cycles) : 0u,
          " permille of it); peer count ", g.r.count, " sum ", hex(g.r.sum), " ferr ", g.r.ferr, " perr ",
          g.r.perr, " ovf ", g.r.ovf, " flags ", hex(g.r.flags), g.acted ? "" : " NOT ACCEPTED",
          g.have ? "" : " NO REPORT", crlf);
}

/// The sink's tally of the last leg; after a STREAMED leg in which the
/// receive stream never moved inside a loan, a line saying the leg did not
/// stream - its lines lent between bursts, not while the next characters
/// arrived - so that a core too slow to run the writer ahead of the echo
/// says so instead of passing letter v without its premise (printed, not
/// judged: it is the core's pace, not the service's correctness).
[[maybe_unused]] void print_sink() {
    print(serial, "    sink: ", LineSink::lines, " lines (", LineSink::good, " good, ", LineSink::empties,
          " empty, ", LineSink::corrupt, " corrupt, ", LineSink::gaps, " gaps losing ", LineSink::lost, ", ",
          LineSink::backwards, " backwards), lent with another waiting ", LineSink::waiting,
          ", same buffer twice ", LineSink::same_buffer, ", nested ", LineSink::nested,
          ", the stream moved inside ", LineSink::moved, crlf);
    if (LineSink::streamed && LineSink::moved == 0u) {
        print(serial, "    THE LEG DID NOT STREAM: the receive stream never moved inside a loan, so no line was "
                      "lent while the next characters arrived (v's premise not exercised)",
              crlf);
    }
}

/// The verdicts of one received leg, `rate` named in each: every line
/// exact, the port's counters zero, and - `peer_reports`, for a peer that
/// keeps a report - the peer's count of what it echoed.
[[maybe_unused]] void judge_echo(const PassSpec& s, uint32_t rate, uint16_t lines, const LegResult& g,
                                 bool peer_reports = true) {
    char what[64];
    auto say = [&what, rate](const char* tail) {
        uint8_t n = 0;
        char d[8];
        uint8_t k = 0;
        uint32_t v = rate;
        do {
            d[k++] = static_cast<char>('0' + v % 10u);
            v /= 10u;
        } while (v != 0u && k < sizeof d);
        while (k != 0u) {
            what[n++] = d[--k];
        }
        what[n++] = ':';
        what[n++] = ' ';
        for (const char* p = tail; *p != '\0' && n + 1u < sizeof what; ++p) {
            what[n++] = *p;
        }
        what[n] = '\0';
        return static_cast<const char*>(what);
    };
    bench.verdict(pass_label(s), say("every line arrived, in order, exact"),
                  g.acted && g.up && g.wrote && LineSink::good == lines && LineSink::gaps == 0u &&
                      LineSink::backwards == 0u && LineSink::corrupt == 0u && LineSink::empties <= 1u);
    bench.verdict(pass_label(s), say("the port's own counters zero"), g.c.clean());
    if (peer_reports) {
        bench.verdict(pass_label(s), say("the peer echoed every byte, clean"),
                      g.have && g.r.count == g.bytes && g.r.ferr == 0u && g.r.perr == 0u && g.r.ovf == 0u);
    }
}
