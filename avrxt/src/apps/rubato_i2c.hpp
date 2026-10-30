// rubato_i2c.hpp - THE TARGET-FREE HALF OF THE I2C SUITES UNDER RUBATO:
// what every family's test_<target>_rubato_i2c compiles unchanged around
// its own I2C host - the pack of five objects, the twi_link.hpp command
// channel through the arbiter, the serve and report helpers, and the
// bodies of the letters n o p q w h i x; and, through rubato_passes.hpp
// beside it, the waits, the passes and the pressure's verdict. An I2C
// suite runs a family's real I2C stack (util/i2c_bus.hpp's arbiter over
// the family's I2cHost and its engines) inside Rubato, the preemptive
// kernel, against a desk peer's unchanged `twi_peer`, while the pack's
// highest object preempts it at prime periods. Every verdict is the
// traffic judged byte-exact and every status the one the wire produced.
//
// APP-LEVEL BENCH TOOLING beside the protocol headers it speaks
// (twi_link.hpp, twi_link_client.hpp), included by relative path as they
// are; nothing in brio knows it exists, and nothing here names a register.
// THE LETTERS LIVE HERE, unlike the link suites' (rubato_link.hpp keeps
// them in each suite): an I2C Request is the same field for field on every
// family and its status vocabulary word for word (docs/design/i2c-bus.md,
// "Realizations"), so what a family's chapter changes in an I2C letter is a
// number - a crossover, a speed, a probe's byte, the peer's stretch - which
// its I2cTraits states with the chapter's section beside it. A family whose
// chapter has a letter no other has adds it in its own suite.
//
// WHERE PREEMPTION CAN REACH AN I2C TENURE. Every I2cHost's start() is
// asynchronous whenever the wire moves, and every byte decision after the
// START is taken in the host's vector, which no dispatch preempts. So the
// thread-level work a preemption reaches is start() itself (run in the
// arbiter's dispatch), the arbiter's dispatch around it (the reply sent
// before the next waiting request is started, the timeout armed and
// disarmed, recover()), and the requesters' dispatches. On a host whose
// start() writes the START last, a preemption there is a later START: a gap
// between tenures. On a host whose start() fills a command FIFO the block
// is already running (the DW_apb_i2c), it is a clock stretch inside the
// tenure, the master holding SCL until start() resumes. A late vector is a
// stretch on every family (I2C is flow-controlled by the master's own
// clock), never a lost byte. Letter x counts which of the two a family has.
//
// ONE BODY, MANY FAMILIES. What is a family's the including suite states in
// three places, all inside its own anonymous namespace with `using
// namespace brio` in force, having included kernel/rubato.hpp,
// kernel/borrowed.hpp, kernel/time.hpp, kernel/time_event.hpp,
// util/i2c_bus.hpp, util/power.hpp, util/print.hpp, util/testbench.hpp,
// <variant> and <iterator> at file scope, and twi_link_client.hpp at file
// scope before its anonymous namespace (it includes twi_link.hpp).
//
// 1. BEFORE THE INCLUDE, these names, every one required:
//
//   core_hz, cycles_per_us   the core clock, in which the ruler counts
//   P                        the Rubato platform
//   Ruler                    Ruler::now(): core cycles, monotone modulo
//                            2^32 (rubato_link.hpp's header says what a
//                            ruler must be)
//   spin(cycles)             a busy wait of at least `cycles` core cycles
//   Serial, serial, bench    the console's Uart and its TestBench (one set,
//                            the all-key `z`)
//   Host                     the family's I2cHost with its engines and its
//                            pads, initialized by the suite before the
//                            kernel runs and never again (the arbiter owns
//                            its vector for the image's life). The body
//                            writes a Request's addr, tx, tx_len, rx,
//                            rx_len, reply and speed, and reads
//                            Host::status() only through the arbiter
//   peer_kind                the far peer's twi_link kind byte (twi_link.hpp,
//                            Ident::version's high byte), which letter n
//                            judges
//   speeds[]                 the speeds the pair judges, ascending, of the
//                            type of the Request's `speed`: speeds[0]
//                            100 kHz (the command channel's and letter h's),
//                            speeds[1] 400 kHz (letters o, q, w and x)
//   speed_hz(s)              the rate a speed names, for the prints
//   pressure_name, peer_name what the prints call the pressure's timer
//                            ("TIM14") and the peer board ("the
//                            STM32F446RE")
//
// 2. BEFORE THE INCLUDE, OPTIONALLY, a `struct I2cTraits` whose static
// members change this body's defaults; a member it leaves out, or an
// I2cTraits it does not declare at all, keeps the default:
//
//   shapes[]                 letter o's table: an array of a struct of the
//                            suite's with `tx_len`, `rx_len` (uint8_t) and
//                            `path` (const char*, the host's path for it:
//                            the pump or the engines, and why); the last
//                            entry a plain write (the counted serve ends on
//                            its last byte, below). Default: the probe,
//                            reads of 1 2 3 8 32, write-then-reads 1+1 1+2
//                            2+8, writes of 1 2 3 8 32, the path unnamed
//   w_client_len             (uint8_t; 6) the read half of letter w's
//                            Client's 1+n write-then-reads, at most 8
//   w_minor_len              (uint8_t; 2) letter w's Minor's writes, at most 8
//   probe_bytes              (uint8_t; 0) the bytes the host's probe moves:
//                            0 for an address-only probe, 1 for a host whose
//                            probe is a one-byte read (the peer counts it)
//   stretch_us               (uint16_t; 1000) letter p's commanded hold
//   stretch_write_holds, stretch_read_holds   (uint8_t; 7 and 8) how many
//                            holds the peer's chapter puts inside an 8-byte
//                            write's and an 8-byte read's tenure - the
//                            letter's lower bound, the arithmetic the
//                            suite's (the default is what any peer that
//                            holds before each data byte produces: a
//                            write's last hold falls after its tenure)
//   fill_tx                  (uint8_t; 16) letter x's write: the host's
//                            longest single-fill shape
//   bring_up()               (static bool; none) called at the opening of
//                            every letter that drives the wire, before the
//                            first tenure: a family whose wire needs a check
//                            or a rail first (a partner's power read on the
//                            lines, a pull-up rail); false fails the letter
//                            by name
//
// 3. AFTER THE INCLUDE, the functions declared below under "What the
// including suite defines": the pressure's timer, host_on_wire(),
// host_dark(), print_host() and console_banner().
//
// The vectors stay the suite's: its pressure vector clears its flag and
// calls pressure_landed(); its I2C vector posts the arbiter's TransferDone
// (`done_from_dma = false` first), and a DMA vector whose engine ends a
// tenure posts it with `done_from_dma = true`.
//
// THE PACK, from the highest priority down:
//
//   Pressure  the preempting object: the family's timer posts to it every
//             97, 193 or 389 us and its dispatch spins 300 or 3000 cycles
//             (rubato_passes.hpp). In letter w it is also a requester (one
//             probe outstanding, posted from its tick); in letter h it posts
//             a probe at every tick, as many outstanding as the arbiter
//             takes.
//   Client    the I2C requester above the arbiter: it takes one Tenure from
//             the Console (the buffers lent until Client::done moves), posts
//             the host's Request with its reply to itself, and at the reply
//             stores the status and the post-to-reply cycles. Letter w's
//             first requester, two outstanding.
//   I2cArb    I2cBus<Marked<Host>, P, 4, BusPassThrough, 20 ms>: pending
//             depth 4 (letter w keeps at most four outstanding: the Client's
//             two, Minor's one, Pressure's one), a per-bus timeout of 20 ms -
//             every Tenuto I2C suite's - above letter p's longest tenure
//             (some 10 ms at 1 ms a byte) by a factor of two. A type derived
//             from the bus whose dispatch notes the event kind for the
//             landing trace and the arbiter's DRAINING window (a timeout's
//             recover() done, its BusFlushed marker not yet served) at each
//             Request. Marked<Host> is the family's host with start()
//             wrapped: `in_start` set around it and its cycles kept, the
//             instrument letter x reads, outside the engine.
//   Minor     a requester BELOW the arbiter (letter w), and the receiver of
//             the arbiter's sleep vote, the one question that says the
//             arbiter is idle with nothing waiting (letter h).
//   Console   the lowest: it owns the console, runs the letters in its own
//             dispatch, waits by the base turn written out
//             (rubato_passes.hpp), and is the only object that prints.
//
// THE COMMAND CHANNEL is twi_link_client.hpp's over ArbPort: one command is
// two tenures through the Client and the arbiter at speeds[0], and the
// peer acts only after its ack read completes. ArbPort::ready() does
// nothing, because the arbiter's timeout and the host's recover() are this
// pack's recovery; ArbPort::wait_ms() is the Console's wait.
//
// THE COUNTED SERVE. Letters o, p and w (and x) end their serve on its
// count (twi_link.hpp: "count = data bytes after which the action may end
// early, at the end of the tenure that moved them"), so that the peer's
// report says the action ran to its count and not to its deadline. A peer
// that ends on the count can do so at the byte that reaches it, before the
// tenure's STOP (the STM32F446RE's twi_peer re-inits its block there), and
// a READ'S last byte is still on the wire at that moment while a write's
// was acknowledged before the peer saw it: so every counted serve here
// ends on a plain write - letter o's table's last entry, letter p's write
// after its read, letter w's closing write after its traffic. Letter q's
// serves and letter h's hold end on their deadlines.
//
// THE LETTERS (z: n o p q w h i; x outside it). Each of n o p q w h runs
// its traffic as the seven passes and judges each on its own, the pressure
// with it; every pass prints the command channel's tenures that were not
// i2c_ok (a command the peer could not take yet, retried).
//   n  the command channel: a ping, ident (twi_peer's sanity byte, the
//      peer's kind), ten pings; every tenure of the pass i2c_ok.
//   o  the shapes table at speeds[1], one counted serve a pass: every status
//      i2c_ok, every read byte-exact against the pattern the serve's count
//      implies (twi_link.hpp's pattern_value() indexed by the bytes the peer
//      has served in the action), the peer's count, 16-bit sum and address
//      matches (a write-then-read is two) exact and the serve ended on its
//      count; each shape's post-to-reply printed, the control's and the worst
//      under pressure.
//   p  for every speed: a read of 16, a 1+4, a write of 16, byte-exact and
//      counted, the wire's own time printed beside each; then the peer's
//      commanded stretch (stretch_us a byte) on a read of 8 and a write of 8
//      at speeds[0], each tenure at least the peer's holds long
//      (stretch_read_holds, stretch_write_holds) and under the per-bus
//      timeout.
//   q  a deaf client (the probe and a write i2c_nack_addr, the next tenure -
//      a probe of deaf_addr, where the peer parks - i2c_ok), the third byte
//      of six refused (i2c_nack_data, then a probe i2c_ok), the last written
//      byte of a 2+4 refused (i2c_nack_data, then a probe i2c_ok, and the
//      peer counting two address matches and no byte served: the read half
//      never began).
//   w  the Client's 20 write-then-reads of 1+w_client_len (two outstanding),
//      Minor's 10 writes of w_minor_len and, in a pressure pass, Pressure's
//      8 probes (one outstanding), all at speeds[1] against a fixed-pattern
//      serve: every reply i2c_ok, to its requester, in its order, nothing
//      rejected, no timeout and no stale event, then a closing write; the
//      peer's count, sum and address matches exact.
//   h  the peer holds SDA low for 60 ms; the Client's write into it at
//      speeds[0], the speed in force, and in a pressure pass Pressure's
//      probe of the command address at every tick until the release: the
//      held tenure answered in its place by i2c_timeout or i2c_arb_lost at
//      22 ms or sooner, every probe answered once (i2c_rejected, i2c_timeout
//      or i2c_arb_lost while held; i2c_ok, or i2c_nack_addr in the moment
//      the peer's client is not yet back, after the release), the next
//      tenure i2c_ok and the arbiter's own vote idle. The Requests that
//      arrived in the draining window are printed. Once after the passes
//      the hold is met at speeds[1], another speed than the one in force,
//      the requirement judged the same.
//   i  dark: the queues empty, the arbiter's vote for the sleep, no time
//      event armed, host_dark(), the ceiling the Console's.
//   x  400 writes of fill_tx with the pressure silent and 400 under 97 us x
//      3000: start()'s own cycles, the landings inside it and of them those
//      with the host on the wire, against the count independent ticks would
//      give (the tenures times the silent start() over the period) - a count
//      far above it says the tenure cycle has locked to the pressure's
//      period; every write i2c_ok and the peer's tallies exact.

#pragma once

// =============================================================================
// What the including suite defines (after the include)
// =============================================================================

/// The pressure's timer: armed for `period_us` with its interrupt on and
/// the counter stopped; started; stopped (the counter alone, the pass's
/// wall read right after); and quietened (its interrupt off, its flag and
/// its line's pending bit cleared, the last tick's dispatch let finish).
void pressure_arm(uint16_t period_us);
void pressure_go();
void pressure_halt();
void pressure_quiet();
/// The host's tenure is on the wire: the family's own bus-activity bit
/// (the STM32G0's ISR.BUSY, the F1 lineage's SR2.MSL, the SAM's BUSSTATE,
/// the DW block's IC_STATUS.MST_ACTIVITY). Called from the pressure's
/// interrupt when it lands inside start(): a register read, nothing more.
bool host_on_wire();
/// Letter i's family half: the pressure's timer stopped with its interrupt
/// off and its line quiet (and its DMA request enables off), the host idle
/// by the family's own definition (its engine's phase, its enables at rest,
/// its DMA channels idle and, where the family has a release contract, no
/// request held), the bus monitor not busy and both lines reading high on
/// their pads. Null when every one holds; otherwise a printable reason, the
/// first that failed.
const char* host_dark();
/// The host's own counters for a pass's print (a spurious bus error count,
/// an unwedge count, DMA faults), starting with "; ": what the family's
/// driver counts that no status says.
void print_host();
void console_banner();

// =============================================================================
// What the including suite may change: I2cTraits (the file header, 2)
// =============================================================================

/// The family's I2cTraits when it declared one before the include; when it
/// did not, this declaration names an incomplete type, which has none of
/// the members. Every member is read only where the type has it.
struct I2cTraits;

using Speed = decltype(Host::Request::speed);
static_assert(std::size(speeds) >= 2u, "speeds[]: 100 kHz first, 400 kHz second, the pair's others after");

/// Letter o's default table (the file header, 2).
struct DefaultShape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};
constexpr DefaultShape default_shapes[] = {
    {0, 0, "-"},  {0, 1, "-"},  {0, 2, "-"},  {0, 3, "-"},  {0, 8, "-"},  {0, 32, "-"},
    {1, 1, "-"},  {1, 2, "-"},  {2, 8, "-"},  {1, 0, "-"},  {2, 0, "-"},  {3, 0, "-"},
    {8, 0, "-"},  {32, 0, "-"},
};

namespace i2c_traits {

template <typename T>
constexpr const auto& shapes() {
    if constexpr (requires { T::shapes; }) {
        return T::shapes;
    } else {
        return default_shapes;
    }
}
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
        return 2;
    }
}
template <typename T>
constexpr uint8_t probe_bytes() {
    if constexpr (requires { T::probe_bytes; }) {
        return T::probe_bytes;
    } else {
        return 0;
    }
}
template <typename T>
constexpr uint16_t stretch_us() {
    if constexpr (requires { T::stretch_us; }) {
        return T::stretch_us;
    } else {
        return 1000;
    }
}
template <typename T>
constexpr uint8_t stretch_write_holds() {
    if constexpr (requires { T::stretch_write_holds; }) {
        return T::stretch_write_holds;
    } else {
        return 7;
    }
}
template <typename T>
constexpr uint8_t stretch_read_holds() {
    if constexpr (requires { T::stretch_read_holds; }) {
        return T::stretch_read_holds;
    } else {
        return 8;
    }
}
template <typename T>
constexpr uint8_t fill_tx() {
    if constexpr (requires { T::fill_tx; }) {
        return T::fill_tx;
    } else {
        return 16;
    }
}
/// The family's bring_up(), where it has one; true otherwise.
template <typename T = I2cTraits>
bool bring_up() {
    if constexpr (requires { T::bring_up(); }) {
        return T::bring_up();
    } else {
        return true;
    }
}

}  // namespace i2c_traits

constexpr const auto& shapes = i2c_traits::shapes<I2cTraits>();
constexpr uint8_t shape_count = static_cast<uint8_t>(std::size(shapes));
static_assert(shape_count >= 1u && shape_count <= 16u, "letter o's table: 1 to 16 shapes");
static_assert(shapes[shape_count - 1u].tx_len != 0u && shapes[shape_count - 1u].rx_len == 0u,
              "letter o's table ends on a plain write: the counted serve ends on its last byte");

// =============================================================================
// The pack's shared vocabulary and the landing trace
// =============================================================================

struct Tick {};
struct Kick {};

/// The pack's indices, from the top (the Rubato list below).
constexpr uint8_t pressure_index = 0;
constexpr uint8_t client_index = 1;
constexpr uint8_t arb_index = 2;
constexpr uint8_t minor_index = 3;
constexpr uint8_t console_index = 4;
constexpr uint8_t landing_slots = 6;   ///< the five indices, and the loop's base (255)

/// Where the pressure's interrupt found the kernel: the ceiling it read,
/// by slot ...
volatile uint32_t landed[landing_slots];
/// ... with the arbiter dispatching a TransferDone at any depth, and of
/// those one a DMA vector posted; inside start() (`landed_in_start`), and
/// of those with the host's tenure on the wire (`landed_marked`: this
/// pack's marked state, rubato_passes.hpp's name).
volatile uint32_t landed_completion = 0;
volatile uint32_t landed_dma_completion = 0;
volatile uint32_t landed_in_start = 0;
volatile uint32_t landed_marked = 0;
volatile uint32_t pressure_irqs = 0;

/// The arbiter's event kind while its dispatch runs (I2cArb below).
constexpr uint8_t arb_none = 0;
constexpr uint8_t arb_request = 1;
constexpr uint8_t arb_done = 2;
constexpr uint8_t arb_other = 3;
volatile uint8_t arb_event = arb_none;
/// The source of the TransferDone the arbiter dispatches: true a DMA
/// vector's, false the I2C vector's. The suite's vectors write it before
/// their post.
volatile bool done_from_dma = false;

/// Inside Host::start() (Marked below), and that start()'s own cycles - the
/// last one's, preemption inside it included.
volatile bool in_start = false;
volatile uint32_t start_cycles = 0;

/// The family's host with start() wrapped by the instrument: `in_start`
/// set around it, the cycles it took kept - two loads of the ruler and
/// three stores a tenure, outside the engine. Every other verb is the
/// host's own.
template <typename H>
struct Marked : H {
    static bool start(const typename H::Request& r) {
        const uint32_t t0 = Ruler::now();
        in_start = true;
        const bool sync = H::start(r);
        in_start = false;
        start_cycles = Ruler::now() - t0;
        return sync;
    }
};

// =============================================================================
// I2cArb: the arbiter, its event kind and its draining window noted
// =============================================================================

/// The per-bus timeout: 20 ms (the pack, in the file header).
constexpr uint32_t arb_timeout_ms = 20;
using ArbBase = I2cBus<Marked<Host>, P, 4, BusPassThrough, ticks_from_ms<P>(arb_timeout_ms)>;

/// The service as it is; the pack names this type so that its dispatch can
/// say, for the landing trace, which kind of event the service is
/// dispatching, and whether a Request arrived in the DRAINING window - the
/// arbiter's state between a timeout's recover() and the BusFlushed marker
/// it posts after it, which only a requester above the arbiter can post
/// into (under Tenuto nothing but an interrupt posts while the arbiter
/// dispatches, and no interrupt posts a Request). The window is read from
/// the outside: a BusTimeout that does not raise the stale count is the
/// live one, and its dispatch enters the window; the next BusFlushed ends
/// it. (The stale count saturates at 255, past which the reading would no
/// longer be exact; letter h prints the count.)
struct I2cArb : ArbBase {
    static inline volatile bool draining = false;
    static inline volatile uint16_t draining_requests = 0;

    static void dispatch(const Event& e) {
        if (std::holds_alternative<TransferDone>(e)) {
            arb_event = arb_done;
        } else if (std::holds_alternative<Host::Request>(e)) {
            arb_event = arb_request;
            if (draining) {
                draining_requests = static_cast<uint16_t>(draining_requests + 1u);
            }
        } else {
            arb_event = arb_other;
        }
        if (std::holds_alternative<BusTimeout>(e)) {
            const uint8_t stale0 = ArbBase::stale_events();
            ArbBase::dispatch(e);
            if (ArbBase::stale_events() == stale0) {
                draining = true;
            }
        } else {
            ArbBase::dispatch(e);
            if (std::holds_alternative<BusFlushed>(e)) {
                draining = false;
            }
        }
        arb_event = arb_none;
    }
};

/// One Request in the shape every family's host takes.
Host::Request request(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len, Speed speed,
                      ReplyTo<I2cDone> reply) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.reply = reply;
    r.speed = speed;
    return r;
}

// =============================================================================
// Letter w's requests and letter h's flood, shared by the requesters
// =============================================================================

namespace w {

constexpr uint8_t seed = 0xA7;        ///< what the peer answers every read byte (pattern_fixed)
constexpr uint8_t client_byte = 0x3C; ///< the Client's written byte
constexpr uint8_t minor_byte = 0x5A;  ///< every byte Minor writes
constexpr uint8_t untouched = 0xEE;
static_assert(seed != untouched, "a slot the peer filled must read differently from an untouched one");

constexpr uint8_t client_n = 20;
constexpr uint8_t client_len = i2c_traits::w_client_len<I2cTraits>();
constexpr uint8_t minor_n = 10;
constexpr uint8_t minor_len = i2c_traits::w_minor_len<I2cTraits>();
constexpr uint8_t press_n = 8;
static_assert(client_len >= 1u && client_len <= 8u && minor_len >= 1u && minor_len <= 8u,
              "letter w's requests are 1 to 8 bytes a half");

const uint8_t client_tx[1] = {client_byte};
const uint8_t minor_tx[8] = {minor_byte, minor_byte, minor_byte, minor_byte,
                             minor_byte, minor_byte, minor_byte, minor_byte};
alignas(4) uint8_t client_rx[client_n][client_len];

volatile bool live = false;
volatile bool press_live = false;   ///< Pressure posts its probes (a pressure pass)

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

bool complete(const uint8_t* slot) {
    for (uint8_t i = 0; i < client_len; ++i) {
        if (slot[i] != seed) {
            return false;
        }
    }
    return true;
}
bool fresh(const uint8_t* slot) {
    for (uint8_t i = 0; i < client_len; ++i) {
        if (slot[i] != untouched) {
            return false;
        }
    }
    return true;
}

void note(volatile Tally& t, const I2cDone& d, bool in_order) {
    if (d.status == i2c_ok) {
        t.ok = static_cast<uint8_t>(t.ok + 1u);
    } else if (d.status == i2c_timeout) {
        t.timeouts = static_cast<uint8_t>(t.timeouts + 1u);
    } else if (d.status == i2c_rejected) {
        t.rejected = static_cast<uint8_t>(t.rejected + 1u);
    }
    if (!in_order) {
        t.out_of_order = static_cast<uint8_t>(t.out_of_order + 1u);
    }
    t.replies = static_cast<uint8_t>(t.replies + 1u);
}

}  // namespace w

namespace h {

/// Letter h's flood: Pressure posts a probe of the command address at
/// every tick while `live`, and its replies are counted by status, each
/// with the time it came back. While the peer holds SDA a probe can be
/// answered i2c_rejected (the pending FIFO full), i2c_timeout (parked past
/// the arbiter's limit) or i2c_arb_lost (its START out into the held line,
/// its address's first one bit read back low); after the release i2c_ok,
/// or i2c_nack_addr in the moment the peer's client is not yet back at the
/// command address.
volatile bool live = false;
struct Tally {
    uint16_t posted = 0;
    uint16_t replies = 0;
    uint16_t ok = 0;
    uint16_t ok_early = 0;   ///< i2c_ok before `held_until` (the wire still held)
    uint16_t rejected = 0;
    uint16_t timeouts = 0;
    uint16_t arb_lost = 0;
    uint16_t nack_addr = 0;
    uint16_t nack_early = 0; ///< i2c_nack_addr before `held_until`
    uint16_t other = 0;
    uint8_t other_status = 0;
};
volatile Tally press{};
/// The ruler's reading before which the peer still holds SDA (the
/// letter's arithmetic).
volatile uint32_t held_from = 0;
volatile uint32_t held_until = 0;

void clear() {
    press.posted = 0;
    press.replies = 0;
    press.ok = 0;
    press.ok_early = 0;
    press.rejected = 0;
    press.timeouts = 0;
    press.arb_lost = 0;
    press.nack_addr = 0;
    press.nack_early = 0;
    press.other = 0;
    press.other_status = 0;
}

/// Before `held_until`, counted from `held_from` (the difference is exact
/// across the ruler's wrap).
bool still_held(uint32_t t) { return t - held_from < held_until - held_from; }

void note(const I2cDone& d) {
    if (d.status == i2c_ok) {
        press.ok = static_cast<uint16_t>(press.ok + 1u);
        if (still_held(Ruler::now())) {
            press.ok_early = static_cast<uint16_t>(press.ok_early + 1u);
        }
    } else if (d.status == i2c_rejected) {
        press.rejected = static_cast<uint16_t>(press.rejected + 1u);
    } else if (d.status == i2c_timeout) {
        press.timeouts = static_cast<uint16_t>(press.timeouts + 1u);
    } else if (d.status == i2c_arb_lost) {
        press.arb_lost = static_cast<uint16_t>(press.arb_lost + 1u);
    } else if (d.status == i2c_nack_addr) {
        // The peer's return to command mode: the line let go, its client
        // not yet back at the command address (a probe in that window
        // meets nobody, which is the wire's own answer).
        press.nack_addr = static_cast<uint16_t>(press.nack_addr + 1u);
        if (still_held(Ruler::now())) {
            press.nack_early = static_cast<uint16_t>(press.nack_early + 1u);
        }
    } else {
        press.other = static_cast<uint16_t>(press.other + 1u);
        press.other_status = d.status;
    }
    press.replies = static_cast<uint16_t>(press.replies + 1u);
}

}  // namespace h

// =============================================================================
// Pressure
// =============================================================================

volatile uint32_t pressure_spin = 0;

struct Pressure {
    using Event = std::variant<Tick, I2cDone>;
    /// A tick and the replies of letter h's flood: each reply runs this
    /// object inside the arbiter's post (it is above the arbiter), so the
    /// queue holds a tick and a reply or two at most; eight is room beside.
    static inline EventQueue<Event, 8, P> queue;
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
                    w::press.posted = static_cast<uint8_t>(w::press.posted + 1u);
                    post<I2cArb>(request(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1],
                                         reply_to<Pressure, I2cDone>()));
                }
                if (h::live) {
                    h::press.posted = static_cast<uint16_t>(h::press.posted + 1u);
                    post<I2cArb>(request(twilink::command_addr, nullptr, 0, nullptr, 0, speeds[0],
                                         reply_to<Pressure, I2cDone>()));
                }
                spin(pressure_spin);
            },
            [](const I2cDone& d) {
                if (w::live) {
                    w::note(w::press, d, w::press.replies < w::press_n);
                    outstanding = false;
                } else {
                    h::note(d);
                }
            });
    }
};

/// The pressure vector's body after the timer's flag is cleared: where it
/// landed, and the post. Interrupt context.
[[gnu::always_inline]] inline void pressure_landed() {
    const uint8_t c = P::ceiling;
    const uint8_t slot = c < landing_slots - 1u ? c : static_cast<uint8_t>(landing_slots - 1u);
    landed[slot] = landed[slot] + 1u;
    if (arb_event == arb_done) {
        landed_completion = landed_completion + 1u;
        if (done_from_dma) {
            landed_dma_completion = landed_dma_completion + 1u;
        }
    }
    if (in_start) {
        landed_in_start = landed_in_start + 1u;
        if (host_on_wire()) {
            landed_marked = landed_marked + 1u;
        }
    }
    pressure_irqs = pressure_irqs + 1u;
    post<Pressure>(Tick{});
}

// =============================================================================
// Client: the requester of every letter's tenures (and letter w's first)
// =============================================================================

/// One tenure, the Console's: the buffers LENT until the Client says it is
/// done (Client::done moves).
struct Tenure {
    uint8_t addr = 0;
    Borrowed<const uint8_t, Lease::reply> tx;
    uint8_t tx_len = 0;
    Borrowed<uint8_t, Lease::reply> rx;
    uint8_t rx_len = 0;
    Speed speed = speeds[0];
};

struct Client {
    using Event = std::variant<Tenure, I2cDone, Kick>;
    static inline EventQueue<Event, 8, P> queue;

    static inline volatile uint32_t done = 0;    ///< tenures answered
    static inline volatile uint8_t status = i2c_ok;
    static inline volatile uint32_t cycles = 0;  ///< the last tenure, post to reply
    static inline uint32_t t_post = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e,
            [](const Tenure& t) {
                Host::Request r{};
                r.addr = t.addr;
                r.tx = t.tx;
                r.tx_len = t.tx_len;
                r.rx = t.rx;
                r.rx_len = t.rx_len;
                r.reply = reply_to<Client, I2cDone>();
                r.speed = t.speed;
                t_post = Ruler::now();
                post<I2cArb>(r);
            },
            [](const I2cDone& d) {
                if (w::live) {
                    // Under Rubato this runs inside the arbiter's finish(),
                    // BEFORE it starts the next waiting request: slot k is
                    // complete and slot k + 1 untouched, exactly.
                    const uint8_t k = w::client.replies;
                    const bool in_order = k < w::client_n && w::complete(w::client_rx[k]) &&
                                          (k + 1u >= w::client_n || w::fresh(w::client_rx[k + 1u]));
                    w::note(w::client, d, in_order);
                    post_next_w();
                    return;
                }
                cycles = Ruler::now() - t_post;
                status = d.status;
                done = done + 1u;
            },
            [](const Kick&) {
                post_next_w();
                post_next_w();
            });
    }

private:
    static void post_next_w() {
        const uint8_t k = w::client.posted;
        if (k >= w::client_n) {
            return;
        }
        w::client.posted = static_cast<uint8_t>(k + 1u);
        post<I2cArb>(request(twilink::dut_addr, w::client_tx, 1, w::client_rx[k], w::client_len, speeds[1],
                             reply_to<Client, I2cDone>()));
    }
};

// =============================================================================
// Minor: letter w's requester below the arbiter, and the vote's receiver
// =============================================================================

struct Minor {
    using Event = std::variant<Kick, I2cDone, SleepVote>;
    static inline EventQueue<Event, 4, P> queue;
    static inline volatile uint8_t votes = 0;
    static inline volatile bool last_vote = false;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e, [](const Kick&) { post_next(); },
            [](const I2cDone& d) {
                w::note(w::minor, d, w::minor.replies < w::minor_n);
                post_next();
            },
            [](const SleepVote& v) {
                last_vote = v.ok;
                votes = static_cast<uint8_t>(votes + 1u);
            });
    }

private:
    static void post_next() {
        if (w::minor.posted >= w::minor_n) {
            return;
        }
        w::minor.posted = static_cast<uint8_t>(w::minor.posted + 1u);
        post<I2cArb>(request(twilink::dut_addr, w::minor_tx, w::minor_len, nullptr, 0, speeds[1],
                             reply_to<Minor, I2cDone>()));
    }
};

// =============================================================================
// Console, the kernel
// =============================================================================

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

using K = Rubato<P, Pressure, Client, I2cArb, Minor, Console>;
using TE = TimeEvents<P>;

#include "rubato_passes.hpp"

/// The command channel's tenures, counted from each pass's begin(): every
/// one of a pass i2c_ok is letter n's verdict, and every other pass prints
/// its refusals (a command the peer could not take yet, retried by
/// twi_link_client.hpp).
uint32_t cmd_tenures = 0;
uint32_t cmd_failed = 0;
uint8_t cmd_last_bad = i2c_ok;

/// The pass's pressure, printed with where it landed, and the host's own
/// counters.
void print_pass(const PassSpec& s, const PassStats& st) {
    print(serial, "  [", pass_label(s), "] ", st.wall / (core_hz / 1000u), " ms");
    if (s.period_us == 0u) {
        print(serial, ", ", pressure_name, " silent");
    } else {
        print(serial, ", ", st.irqs, " ticks, ", st.runs, " dispatches, ", st.overflows,
              " dropped; landed in the walk (ceiling 0) ", st.land[pressure_index], ", Client ",
              st.land[client_index], ", I2cArb ", st.land[arb_index], ", Minor ", st.land[minor_index],
              ", Console ", st.land[console_index], ", base ", st.land[landing_slots - 1u], crlf);
        print(serial, "    inside a completion's dispatch (any depth) ", st.completion, ", of a DMA completion ",
              st.dma_completion, "; inside start() with the host on the wire ", st.marked);
    }
    print(serial, "; command tenures ", cmd_tenures, ", not i2c_ok ", cmd_failed);
    print_host();
    print(serial, crlf);
}

// =============================================================================
// The command channel: twi_link_client.hpp through the Client and the arbiter
// =============================================================================

/// What tenure() answers when no reply came inside its wait.
constexpr uint8_t no_reply = 200;
static_assert(no_reply != i2c_ok && no_reply != i2c_rejected && no_reply != i2c_timeout &&
                  no_reply > i2c_bus_error + 1u,
              "no_reply is no status of the vocabulary");

/// One tenure through the Client and the arbiter, waited for - at most
/// 500 ms, an order above the arbiter's own timeout, so a tenure is always
/// answered by the arbiter first and no_reply means the pack itself
/// stopped. The buffers are lent for the call.
uint8_t arb_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len, Speed speed) {
    const uint32_t before = Client::done;
    Tenure t{};
    t.addr = addr;
    t.tx = lend<Lease::reply>(tx);
    t.tx_len = tx_len;
    t.rx = lend<Lease::reply>(rx);
    t.rx_len = rx_len;
    t.speed = speed;
    post<Client>(t);
    if (!wait_until([before] { return Client::done != before; }, 500)) {
        return no_reply;
    }
    return Client::status;
}

void console_wait_ms(uint32_t ms) { wait_ms(ms); }

/// A letter's pass begun: rubato_passes.hpp's pass_begin() and the command
/// channel's counters cleared.
void begin(const PassSpec& s) {
    cmd_tenures = 0;
    cmd_failed = 0;
    pass_begin(s);
}

/// twi_link_client.hpp's Port: the command channel's tenures through the
/// arbiter at speeds[0], counted.
struct ArbPort {
    static uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len) {
        const uint8_t st = arb_tenure(addr, tx, tx_len, rx, rx_len, speeds[0]);
        cmd_tenures = cmd_tenures + 1u;
        if (st != i2c_ok) {
            cmd_failed = cmd_failed + 1u;
            cmd_last_bad = st;
        }
        return st;
    }
    static void ready() {}
    static void wait_ms(uint32_t ms) { console_wait_ms(ms); }
};
using Link = TwiLinkClient<ArbPort>;
using twilink::Op;

/// The letter's opening: the family's bring_up() where it has one, then
/// the peer's ping. A named failure, never a hang.
bool need_peer() {
    if (!i2c_traits::bring_up()) {
        bench.verdict("the family's bring-up before the wire (I2cTraits::bring_up)", false);
        return false;
    }
    if (Link::command(Op::ping)) {
        return true;
    }
    print(serial, "  THE PEER DID NOT ANSWER: ", peer_name, " must be running `twi_peer` (its console '0' "
                  "forces the command-mode client back); last status ",
          cmd_last_bad, crlf);
    bench.verdict("the peer answers a ping on the command address (the peer board running twi_peer)", false);
    return false;
}

/// A serve commanded: the peer a client at `a.addr`, acked and armed.
bool serve(const twilink::Params& a) { return Link::act(Op::serve, a); }

/// A wait past a deadline-ended action: `ms` of the action's deadline from
/// `t0` (the ruler's reading when it was acked), and 10 ms more - the
/// peer's own clock against this one's (a few hundred ppm at most, under a
/// millisecond over these deadlines) and its return to command mode.
void wait_past(uint32_t t0, uint16_t ms) {
    const uint32_t until = (static_cast<uint32_t>(ms) + 10u) * (core_hz / 1000u);
    while (Ruler::now() - t0 < until) {
        wait_turn();
    }
}

/// The peer's report of its last action. False when four queries could not
/// fetch it.
bool report(twilink::Report& r) { return Link::report(r); }

void print_report(const twilink::Report& r) {
    print(serial, "    peer: count ", r.count, " (received ", r.aux0, ", served ", r.aux1, "), sum ", hex(r.sum),
          ", addr_hits ", r.addr_hits, ", stops ", r.stops, ", flags ", hex(r.flags), ", ", r.ms, " ms", crlf);
}

/// The report says the action ran and ended on its count, not its
/// deadline.
bool ran_to_count(const twilink::Report& r) {
    return (r.flags & twilink::report_ran) != 0u && (r.flags & twilink::report_timed_out) == 0u;
}

/// The wire's own time for a tenure of `tx` + `rx` data bytes at `hz`:
/// nine bit times for each byte and each address (two when both halves
/// are there), in core cycles; a START, a repeated START and a STOP are
/// left out, so the measured time is above it.
uint32_t wire_cycles(uint8_t tx, uint8_t rx, uint32_t hz) {
    const uint32_t addresses = (tx != 0u && rx != 0u) ? 2u : 1u;
    const uint32_t bits = 9u * (addresses + tx + rx);
    return static_cast<uint32_t>(static_cast<uint64_t>(bits) * core_hz / hz);
}

uint32_t us_of(uint32_t cycles) { return cycles / cycles_per_us; }

// =============================================================================
// n - the command channel through the arbiter
// =============================================================================

void tn_command_channel() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        begin(s);
        const bool there = need_peer();
        twilink::Ident d{};
        const bool got = there && Link::ident(d);
        uint8_t pings = 0;
        for (uint8_t i = 0; there && i < 10u; ++i) {
            if (Link::command(Op::ping)) {
                ++pings;
            }
        }
        const uint32_t tenures = cmd_tenures, failed = cmd_failed;
        const PassStats st = pass_end(s);
        print_pass(s, st);
        if (got) {
            print(serial, "    peer: label '");
            for (uint8_t i = 0; i < 8u && d.label[i] != '\0'; ++i) {
                print(serial, d.label[i]);
            }
            print(serial, "' sanity=", hex(d.sanity), " fw=", hex(d.version), "; ", pings, " of 10 pings; ", tenures,
                  " tenures at ", speed_hz(speeds[0]) / 1000u, " kHz, ", failed, " not i2c_ok", crlf);
        }
        bench.verdict(pass_label(s), "ident is twi_peer's (the sanity byte) and the peer's kind",
                      got && d.sanity == twilink::ident_sanity && twilink::peer_kind(d.version) == peer_kind);
        bench.verdict(pass_label(s), "ten more commands acked, every one two tenures through the arbiter",
                      pings == 10u);
        bench.verdict(pass_label(s), "every tenure of the pass i2c_ok", there && failed == 0u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// o - the tenure shapes, on every path the host has
// =============================================================================

alignas(4) uint8_t tx_buf[64];
alignas(4) uint8_t rx_buf[64];

void to_shapes() {
    uint32_t control_cyc[shape_count] = {};
    uint32_t worst_cyc[shape_count] = {};
    uint32_t pass_cyc[shape_count] = {};
    uint16_t total = 0;
    for (const auto& sh : shapes) {
        total = static_cast<uint16_t>(total + sh.tx_len + sh.rx_len);
    }
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        twilink::Params a{};
        a.count = total;
        a.ms = 2000;
        a.addr = twilink::dut_addr;
        a.seed = static_cast<uint8_t>(0x30u + 7u * p);
        a.pattern = twilink::pattern_counting;
        const bool acted = serve(a);
        uint8_t bad = 0, first_bad = 0xFF, bad_status = i2c_ok;
        uint16_t mism = 0, served = 0, sum = 0, hits = 0;
        uint8_t first_mism = 0xFF;
        for (uint8_t i = 0; acted && i < shape_count; ++i) {
            const auto& sh = shapes[i];
            for (uint8_t k = 0; k < sh.tx_len; ++k) {
                tx_buf[k] = static_cast<uint8_t>(0x40u + 16u * i + k);
                sum = static_cast<uint16_t>(sum + tx_buf[k]);
            }
            for (uint8_t k = 0; k < sh.rx_len; ++k) {
                rx_buf[k] = 0xEE;
            }
            const uint8_t st = arb_tenure(twilink::dut_addr, tx_buf, sh.tx_len, rx_buf, sh.rx_len, speeds[1]);
            const uint32_t c = Client::cycles;
            pass_cyc[i] = c;
            if (p == 0u) {
                control_cyc[i] = c;
            } else if (c > worst_cyc[i]) {
                worst_cyc[i] = c;
            }
            if (st != i2c_ok) {
                ++bad;
                if (first_bad == 0xFF) {
                    first_bad = i;
                    bad_status = st;
                }
            }
            for (uint8_t k = 0; k < sh.rx_len; ++k) {
                if (rx_buf[k] != twilink::pattern_value(a.pattern, a.seed, static_cast<uint16_t>(served + k))) {
                    ++mism;
                    if (first_mism == 0xFF) {
                        first_mism = i;
                    }
                }
            }
            served = static_cast<uint16_t>(served + sh.rx_len);
            hits = static_cast<uint16_t>(hits + ((sh.tx_len != 0u && sh.rx_len != 0u) ? 2u : 1u));
        }
        twilink::Report r{};
        const bool rep = acted && report(r);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    post to reply, us:");
        for (uint8_t i = 0; i < shape_count; ++i) {
            print(serial, " ", shapes[i].tx_len, "+", shapes[i].rx_len, "=", us_of(pass_cyc[i]));
        }
        print(serial, crlf);
        if (bad != 0u || mism != 0u) {
            print(serial, "    ", bad, " shapes not i2c_ok (the first, ", first_bad, ", status ", bad_status, "); ",
                  mism, " read bytes off the pattern (the first in shape ", first_mism, ")", crlf);
        }
        if (rep) {
            print_report(r);
        }
        bench.verdict(pass_label(s), "every shape i2c_ok on every path of the host", acted && bad == 0u);
        bench.verdict(pass_label(s), "every read byte-exact against the pattern the serve's count implies",
                      acted && mism == 0u);
        bench.verdict(pass_label(s), "the peer counted every byte, its sum and every address match exact, the serve "
                                     "ended on its count",
                      rep && r.count == total && r.sum == sum && r.addr_hits == hits && ran_to_count(r));
        judge_pressure(s, st);
    }
    print(serial, "  each shape's post to reply, the control's and the worst under pressure (us), and its path:",
          crlf);
    for (uint8_t i = 0; i < shape_count; ++i) {
        print(serial, "    ", shapes[i].tx_len, "+", shapes[i].rx_len, ": ", us_of(control_cyc[i]), " / ",
              us_of(worst_cyc[i]), "  ", shapes[i].path, crlf);
    }
}

// =============================================================================
// p - the speeds, and the commanded stretch
// =============================================================================

constexpr uint16_t stretch_us = i2c_traits::stretch_us<I2cTraits>();
constexpr uint8_t stretch_write_holds = i2c_traits::stretch_write_holds<I2cTraits>();
constexpr uint8_t stretch_read_holds = i2c_traits::stretch_read_holds<I2cTraits>();
/// The longest stretched tenure, a hold before every byte and the address
/// besides, at most nine holds and the wire: under the arbiter's timeout.
static_assert(9u * stretch_us + 1000u < 1000u * arb_timeout_ms,
              "letter p's stretched tenure stays under the per-bus timeout");

void tp_speeds() {
    constexpr uint8_t n_speeds = static_cast<uint8_t>(std::size(speeds));
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        bool rung_ok[n_speeds] = {};
        uint32_t cyc[n_speeds][3] = {};
        for (uint8_t j = 0; j < n_speeds; ++j) {
            // A read of 16, a 1+4 write-then-read, a write of 16 last: 37
            // bytes, the count the serve ends on.
            twilink::Params a{};
            a.count = 37;
            a.ms = 1000;
            a.addr = twilink::dut_addr;
            a.seed = static_cast<uint8_t>(0x50u + 0x10u * j + p);
            a.pattern = twilink::pattern_counting;
            if (!serve(a)) {
                continue;
            }
            for (uint8_t k = 0; k < 16u; ++k) {
                rx_buf[k] = 0xEE;
                tx_buf[k] = static_cast<uint8_t>(0xC0u + k);
            }
            const uint8_t rs = arb_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, speeds[j]);
            cyc[j][0] = Client::cycles;
            uint8_t mism = 0;
            for (uint8_t k = 0; k < 16u; ++k) {
                mism = static_cast<uint8_t>(mism + (rx_buf[k] != twilink::pattern_value(a.pattern, a.seed, k)));
                rx_buf[k] = 0xEE;
            }
            const uint8_t one = 0x9D;
            const uint8_t cs = arb_tenure(twilink::dut_addr, &one, 1, rx_buf, 4, speeds[j]);
            cyc[j][1] = Client::cycles;
            for (uint8_t k = 0; k < 4u; ++k) {
                mism = static_cast<uint8_t>(mism + (rx_buf[k] != twilink::pattern_value(a.pattern, a.seed,
                                                                                        static_cast<uint16_t>(16u + k))));
            }
            const uint8_t ws = arb_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0, speeds[j]);
            cyc[j][2] = Client::cycles;
            uint16_t sum = one;
            for (uint8_t k = 0; k < 16u; ++k) {
                sum = static_cast<uint16_t>(sum + tx_buf[k]);
            }
            twilink::Report r{};
            const bool rep = report(r);
            rung_ok[j] = rs == i2c_ok && cs == i2c_ok && ws == i2c_ok && mism == 0u && rep && r.count == 37u &&
                         r.sum == sum && ran_to_count(r);
            if (!rung_ok[j]) {
                print(serial, "    ", speed_hz(speeds[j]) / 1000u, " kHz: read ", rs, ", write-then-read ", cs,
                      ", write ", ws, ", ", mism, " bytes off the pattern", crlf);
                if (rep) {
                    print_report(r);
                }
            }
        }
        // The commanded stretch at speeds[0]: a read of 8, then a write of
        // 8, the count 16.
        twilink::Params h{};
        h.count = 16;
        h.ms = 1000;
        h.addr = twilink::dut_addr;
        h.seed = static_cast<uint8_t>(0x70u + p);
        h.hold_us = stretch_us;
        h.pattern = twilink::pattern_counting;
        const bool acted = serve(h);
        uint8_t rs = no_reply, ws = no_reply, mism = 0;
        uint32_t rc = 0, wc = 0;
        if (acted) {
            for (uint8_t k = 0; k < 8u; ++k) {
                rx_buf[k] = 0xEE;
                tx_buf[k] = static_cast<uint8_t>(0x21u + 3u * k);
            }
            rs = arb_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, speeds[0]);
            rc = Client::cycles;
            for (uint8_t k = 0; k < 8u; ++k) {
                mism = static_cast<uint8_t>(mism + (rx_buf[k] != twilink::pattern_value(h.pattern, h.seed, k)));
            }
            ws = arb_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0, speeds[0]);
            wc = Client::cycles;
            // The last byte's hold falls after the write's tenure, and the
            // peer ends its serve after it: the report waits it out (a
            // command sooner meets the client still at dut_addr).
            wait_ms(stretch_us / 1000u + 3u);
        }
        twilink::Report r{};
        const bool rep = acted && report(r);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        for (uint8_t j = 0; j < n_speeds; ++j) {
            const uint32_t hz = speed_hz(speeds[j]);
            print(serial, "    ", hz / 1000u, " kHz: read 16 in ", us_of(cyc[j][0]), " us (the wire's ",
                  us_of(wire_cycles(0, 16, hz)), "), 1+4 in ", us_of(cyc[j][1]), " (", us_of(wire_cycles(1, 4, hz)),
                  "), write 16 in ", us_of(cyc[j][2]), " (", us_of(wire_cycles(16, 0, hz)), ")", crlf);
        }
        const uint32_t hold = static_cast<uint32_t>(stretch_us) * cycles_per_us;
        print(serial, "    the stretch, ", stretch_us, " us a byte at ", speed_hz(speeds[0]) / 1000u, " kHz: read 8 in ",
              us_of(rc), " us (at least ", stretch_read_holds, " holds, ", us_of(stretch_read_holds * hold),
              "), write 8 in ", us_of(wc), " us (at least ", stretch_write_holds, ", ",
              us_of(stretch_write_holds * hold), "); the timeout ", arb_timeout_ms, " ms", crlf);
        if (rep && (r.count != 16u || mism != 0u)) {
            print_report(r);
        }
        for (uint8_t j = 0; j < n_speeds; ++j) {
            char what[80];
            uint8_t n = 0;
            uint32_t v = speed_hz(speeds[j]) / 1000u;
            char d[8];
            uint8_t k = 0;
            do {
                d[k++] = static_cast<char>('0' + v % 10u);
                v /= 10u;
            } while (v != 0u);
            while (k != 0u) {
                what[n++] = d[--k];
            }
            for (const char* q = " kHz: a read of 16, a 1+4, a write of 16, byte-exact, i2c_ok, counted";
                 *q != '\0' && n + 1u < sizeof what; ++q) {
                what[n++] = *q;
            }
            what[n] = '\0';
            bench.verdict(pass_label(s), what, rung_ok[j]);
        }
        bench.verdict(pass_label(s), "the commanded stretch: both tenures i2c_ok, the read byte-exact, the peer "
                                     "counting 16",
                      rs == i2c_ok && ws == i2c_ok && mism == 0u && rep && r.count == 16u && ran_to_count(r));
        bench.verdict(pass_label(s), "... each tenure as long as the peer's holds at least (the controller waits)",
                      rc >= stretch_read_holds * hold && wc >= stretch_write_holds * hold);
        judge_pressure(s, st);
    }
}

// =============================================================================
// q - the vocabulary under pressure
// =============================================================================

void tq_vocabulary() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        constexpr uint16_t deadline = 120;
        // A deaf client: the probe and a write meet nobody at dut_addr; the
        // peer is parked at deaf_addr, which answers - the next tenure.
        twilink::Params deaf{};
        deaf.ms = deadline;
        deaf.addr = twilink::dut_addr;
        deaf.flags = twilink::flag_deaf;
        uint8_t probe = no_reply, wdeaf = no_reply, next1 = no_reply;
        bool a1 = serve(deaf);
        const uint32_t t1 = Ruler::now();
        if (a1) {
            const uint8_t w4[4] = {1, 2, 3, 4};
            probe = arb_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wdeaf = arb_tenure(twilink::dut_addr, w4, 4, nullptr, 0, speeds[1]);
            next1 = arb_tenure(twilink::deaf_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wait_past(t1, deadline);
        }
        // The third byte of six refused.
        twilink::Params n3{};
        n3.ms = deadline;
        n3.addr = twilink::dut_addr;
        n3.nack_at = 3;
        uint8_t nack3 = no_reply, next2 = no_reply;
        const bool a2 = serve(n3);
        const uint32_t t2 = Ruler::now();
        if (a2) {
            const uint8_t w6[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
            nack3 = arb_tenure(twilink::dut_addr, w6, 6, nullptr, 0, speeds[1]);
            next2 = arb_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wait_past(t2, deadline);
        }
        // The LAST written byte of a 2+4 write-then-read refused: the
        // controller lets go, no read half follows (the peer counts the
        // tenure's one address match, and the probe after it).
        twilink::Params n2{};
        n2.ms = deadline;
        n2.addr = twilink::dut_addr;
        n2.nack_at = 2;
        uint8_t nack2 = no_reply, next3 = no_reply;
        const bool a3 = serve(n2);
        const uint32_t t3 = Ruler::now();
        if (a3) {
            const uint8_t w2[2] = {0x77, 0x88};
            for (uint8_t k = 0; k < 4u; ++k) {
                rx_buf[k] = 0xEE;
            }
            nack2 = arb_tenure(twilink::dut_addr, w2, 2, rx_buf, 4, speeds[1]);
            next3 = arb_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wait_past(t3, deadline);
        }
        twilink::Report r{};
        const bool rep = a3 && report(r);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    deaf: probe ", probe, ", write ", wdeaf, ", then deaf_addr's probe ", next1,
              "; the third of six refused: ", nack3, ", then the probe ", next2, "; the last of a 2+4's write "
              "refused: ",
              nack2, ", then the probe ", next3, crlf);
        if (rep) {
            print_report(r);
        }
        bench.verdict(pass_label(s), "a deaf client: the probe and a write answered i2c_nack_addr, the next tenure "
                                     "i2c_ok",
                      a1 && probe == i2c_nack_addr && wdeaf == i2c_nack_addr && next1 == i2c_ok);
        bench.verdict(pass_label(s), "the third byte of six refused: i2c_nack_data, the next tenure i2c_ok",
                      a2 && nack3 == i2c_nack_data && next2 == i2c_ok);
        bench.verdict(pass_label(s), "the last written byte of a 2+4 refused: i2c_nack_data, the controller let go "
                                     "(no read half), the next tenure i2c_ok",
                      a3 && nack2 == i2c_nack_data && next3 == i2c_ok && rep && r.addr_hits == 2u &&
                          (r.flags & twilink::report_nacked) != 0u && r.aux1 == 0u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// w - the arbiter with three requesters of three priorities
// =============================================================================

constexpr uint8_t probe_bytes = i2c_traits::probe_bytes<I2cTraits>();
constexpr uint8_t closing_byte = 0x6E;

void tw_arbiter() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        const uint16_t bytes = static_cast<uint16_t>(w::client_n * (1u + w::client_len) + w::minor_n * w::minor_len +
                                                     (pressing ? w::press_n * probe_bytes : 0u) + 1u);
        const uint16_t sum = static_cast<uint16_t>(w::client_n * w::client_byte +
                                                   w::minor_n * w::minor_len * w::minor_byte + closing_byte);
        const uint16_t hits = static_cast<uint16_t>(2u * w::client_n + w::minor_n + (pressing ? w::press_n : 0u) + 1u);
        twilink::Params a{};
        a.count = bytes;
        a.ms = 3000;
        a.addr = twilink::dut_addr;
        a.seed = w::seed;
        a.pattern = twilink::pattern_fixed;
        const bool acted = serve(a);
        for (auto& row : w::client_rx) {
            for (uint8_t& b : row) {
                b = w::untouched;
            }
        }
        w::clear(w::client);
        w::clear(w::minor);
        w::clear(w::press);
        Pressure::outstanding = false;
        const uint8_t rejected0 = I2cArb::rejected_count();
        const uint8_t stale0 = I2cArb::stale_events();
        const uint32_t t0 = Ruler::now();
        bool done = false;
        uint8_t closing = no_reply;
        if (acted) {
            w::live = true;
            w::press_live = pressing;
            post<Client>(Kick{});
            post<Minor>(Kick{});
            done = wait_until(
                [pressing] {
                    return w::client.replies >= w::client_n && w::minor.replies >= w::minor_n &&
                           (!pressing || w::press.replies >= w::press_n);
                },
                2000);
            w::press_live = false;
            w::live = false;
            closing = arb_tenure(twilink::dut_addr, &closing_byte, 1, nullptr, 0, speeds[1]);
        }
        const uint32_t took = Ruler::now() - t0;
        twilink::Report r{};
        const bool rep = acted && report(r);
        uint16_t host_bad = 0;
        for (const auto& row : w::client_rx) {
            for (const uint8_t b : row) {
                host_bad = static_cast<uint16_t>(host_bad + (b != w::seed));
            }
        }
        const uint8_t rejected = static_cast<uint8_t>(I2cArb::rejected_count() - rejected0);
        const uint8_t stale = static_cast<uint8_t>(I2cArb::stale_events() - stale0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    ", us_of(took), " us; Client ", w::client.replies, "/", w::client.posted, " replies (",
              w::client.ok, " ok, ", w::client.out_of_order, " out of order), Minor ", w::minor.replies, "/",
              w::minor.posted, " (", w::minor.ok, " ok), Pressure ", w::press.replies, "/", w::press.posted, " (",
              w::press.ok, " ok); timeouts ", w::client.timeouts + w::minor.timeouts + w::press.timeouts,
              ", rejected ", rejected, ", stale ", stale, "; read bytes wrong ", host_bad, "; the closing write ",
              closing, crlf);
        if (rep) {
            print_report(r);
        }
        bench.verdict(pass_label(s), "every request answered i2c_ok, to its own requester, once",
                      done && w::client.ok == w::client_n && w::minor.ok == w::minor_n &&
                          w::client.replies == w::client_n && w::minor.replies == w::minor_n &&
                          w::press.replies == (pressing ? w::press_n : 0u) &&
                          w::press.ok == (pressing ? w::press_n : 0u) && closing == i2c_ok);
        bench.verdict(pass_label(s), "every reply in its requester's order (the Client's slot k complete and k + 1 "
                                     "untouched at reply k)",
                      w::client.out_of_order == 0u && w::minor.out_of_order == 0u && w::press.out_of_order == 0u);
        bench.verdict(pass_label(s), "nothing rejected, the per-bus timeout never fired (no i2c_timeout, no stale "
                                     "event)",
                      rejected == 0u && stale == 0u && w::client.timeouts == 0u && w::minor.timeouts == 0u &&
                          w::press.timeouts == 0u && w::client.rejected == 0u && w::minor.rejected == 0u &&
                          w::press.rejected == 0u);
        bench.verdict(pass_label(s), "the peer's count, sum and address matches exact, every read byte the peer's",
                      rep && r.count == bytes && r.sum == sum && r.addr_hits == hits && ran_to_count(r) &&
                          host_bad == 0u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// h - the held wire, with requests arriving in the drain window
// =============================================================================

/// The peer's hold: SDA low from its GPIO for 60 ms, its deadline the one
/// release (aux8 0: no SCL edge releases it).
constexpr uint16_t hold_ms = 60;

/// The held tenure's longest answer: the arbiter's 20 ticks, a tick more
/// for where in a tick the timer was armed, and a millisecond for the walk
/// of the time events that posts it (the Console's wait turn and one
/// Pressure dispatch: some 80 us) - 22 ms.
constexpr uint32_t held_answer_ms = arb_timeout_ms + 2u;

/// One held tenure: the hold commanded, the Client's write into it at
/// `speed`, and in a pressure pass Pressure's flood; then the release
/// waited for, the flood stopped and drained, the next tenure and the
/// arbiter's vote. Returns the held tenure's status and its time.
struct HeldOutcome {
    bool acted = false;
    uint8_t status = no_reply;
    uint32_t cycles = 0;
    bool drained = false;
    uint8_t next = no_reply;
    bool voted = false;
    bool idle = false;
};

HeldOutcome held_tenure(Speed speed, bool flood) {
    HeldOutcome o{};
    twilink::Params a{};
    a.ms = hold_ms;
    a.aux16 = static_cast<uint16_t>(1000u * hold_ms);
    a.aux8 = 0;
    h::clear();
    o.acted = Link::act(Op::hold_sda, a);
    if (!o.acted) {
        return o;
    }
    // The peer holds from the moment its ack read completed, which was
    // before the ack's reply reached this end, and act() waited arm_ms
    // more: the hold has run 3 ms at least now and lasts its 60 by the
    // peer's clock, so it stands at least 55 ms from here (2 ms for the
    // two clocks' difference and the peer's coarse look at its own).
    const uint32_t now = Ruler::now();
    h::held_from = now;
    h::held_until = now + (hold_ms - 5u) * (core_hz / 1000u);
    h::live = flood;
    const uint8_t w4[4] = {0x31, 0x32, 0x33, 0x34};
    o.status = arb_tenure(twilink::dut_addr, w4, 4, nullptr, 0, speed);
    o.cycles = Client::cycles;
    // The release: the hold's 60 ms from its start, the peer's coarse
    // deadline check and its return to command mode - 70 ms from the ack.
    wait_past(now, hold_ms);
    h::live = false;
    o.drained = wait_until([] { return h::press.replies >= h::press.posted; }, 500);
    o.next = Link::command(Op::ping) ? i2c_ok : cmd_last_bad;
    const uint8_t votes0 = Minor::votes;
    post<I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<Minor, SleepVote>()});
    o.voted = wait_until([votes0] { return Minor::votes != votes0; }, 100);
    o.idle = o.voted && Minor::last_vote;
    return o;
}

bool held_signature(uint8_t st) { return st == i2c_timeout || st == i2c_arb_lost; }

void th_held_wire() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        I2cArb::draining_requests = 0;
        const uint8_t stale0 = I2cArb::stale_events();
        // At the command channel's rate: the speed in force stays, so the
        // host's start() touches no timing register before the START.
        const HeldOutcome o = held_tenure(speeds[0], pressing);
        const uint8_t stale = static_cast<uint8_t>(I2cArb::stale_events() - stale0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    the held tenure: status ", o.status,
              o.status == i2c_timeout ? " (i2c_timeout)" : (o.status == i2c_arb_lost ? " (i2c_arb_lost)" : ""), " in ",
              us_of(o.cycles), " us; Pressure posted ", h::press.posted, ", answered ", h::press.replies, " (ok ",
              h::press.ok, ", of them while held ", h::press.ok_early, "; rejected ", h::press.rejected, ", timeout ",
              h::press.timeouts, ", arb_lost ", h::press.arb_lost, ", nack_addr ", h::press.nack_addr, " (", h::press.nack_early, " while held), other ", h::press.other, " last ",
              h::press.other_status, "); Requests in the draining window ", I2cArb::draining_requests, ", stale ",
              stale, "; next tenure ", o.next, ", the vote ", o.idle ? "idle" : "NOT IDLE", crlf);
        bench.verdict(pass_label(s), "the held tenure answered in its place, never i2c_ok - i2c_timeout or "
                                     "i2c_arb_lost, the family's two signatures",
                      o.acted && held_signature(o.status));
        bench.verdict(pass_label(s), "... at the arbiter's limit or sooner (22 ms, the letter's arithmetic)",
                      o.acted && o.cycles <= held_answer_ms * (core_hz / 1000u));
        bench.verdict(pass_label(s), "every request Pressure posted answered exactly once, i2c_rejected, "
                                     "i2c_timeout, i2c_arb_lost, or after the release i2c_ok or i2c_nack_addr",
                      o.drained && h::press.replies == h::press.posted && h::press.other == 0u &&
                          h::press.ok_early == 0u && h::press.nack_early == 0u &&
                          (pressing || h::press.posted == 0u));
        bench.verdict(pass_label(s), "after the release the next tenure i2c_ok, the arbiter idle with nothing "
                                     "pending (its own vote)",
                      o.next == i2c_ok && o.idle);
        judge_pressure(s, st);
    }
    // The same hold met at ANOTHER rate than the one in force: a host whose
    // speed change cycles its block before the START (the STM32G0's PE
    // cycle, 32.4.6) forgets the bus it saw busy - printed, the requirement
    // judged.
    if (!need_peer()) {
        return;
    }
    const HeldOutcome o = held_tenure(speeds[1], false);
    print(serial, "  the hold met at ", speed_hz(speeds[1]) / 1000u, " kHz after commands at ",
          speed_hz(speeds[0]) / 1000u, ": status ", o.status,
          o.status == i2c_timeout ? " (i2c_timeout)" : (o.status == i2c_arb_lost ? " (i2c_arb_lost)" : ""), " in ",
          us_of(o.cycles), " us; next tenure ", o.next, crlf);
    bench.verdict("a rate change into the held wire: answered in its place, never i2c_ok, at the limit or sooner, "
                  "the next tenure i2c_ok",
                  o.acted && held_signature(o.status) && o.cycles <= held_answer_ms * (core_hz / 1000u) &&
                      o.next == i2c_ok && o.idle);
}

// =============================================================================
// i - dark
// =============================================================================

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && Client::queue.empty() && I2cArb::queue.empty() && Minor::queue.empty());
    const uint8_t votes0 = Minor::votes;
    post<I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<Minor, SleepVote>()});
    const bool voted = wait_until([votes0] { return Minor::votes != votes0; }, 100);
    bench.verdict("the arbiter votes for the sleep: nothing in flight, nothing waiting", voted && Minor::last_vote);
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    const char* why = host_dark();
    if (why != nullptr) {
        print(serial, "  not dark: ", why, crlf);
    }
    bench.verdict("the pressure's timer stopped and quiet, the host idle by its own definition, the bus free and "
                  "both lines high",
                  why == nullptr);
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// x - outside z: the landings inside start() counted
// =============================================================================

constexpr uint8_t fill_tx = i2c_traits::fill_tx<I2cTraits>();
static_assert(fill_tx >= 1u && fill_tx <= 64u, "letter x's write fits the buffer");

void tx_fill() {
    constexpr uint16_t tenures = 400;
    uint32_t silent_start = 0;   ///< the control's average start(), in cycles
    for (uint8_t press = 0; press < 2u; ++press) {
        const PassSpec& s = passes[press == 0u ? 0u : 4u];
        if (!need_peer()) {
            return;
        }
        twilink::Params a{};
        a.count = static_cast<uint16_t>(tenures * fill_tx);
        a.ms = 5000;
        a.addr = twilink::dut_addr;
        const bool acted = serve(a);
        uint16_t sum = 0, bad = 0;
        uint32_t worst = 0, start_worst = 0, start_sum = 0;
        landed_in_start = 0;
        begin(s);
        for (uint16_t k = 0; acted && k < tenures; ++k) {
            for (uint8_t i = 0; i < fill_tx; ++i) {
                tx_buf[i] = static_cast<uint8_t>(k * 3u + i * 7u);
                sum = static_cast<uint16_t>(sum + tx_buf[i]);
            }
            if (arb_tenure(twilink::dut_addr, tx_buf, fill_tx, nullptr, 0, speeds[1]) != i2c_ok) {
                ++bad;
            }
            worst = Client::cycles > worst ? Client::cycles : worst;
            const uint32_t sc = start_cycles;
            start_worst = sc > start_worst ? sc : start_worst;
            start_sum += sc;
        }
        const uint32_t in_start_n = landed_in_start;
        const PassStats st = pass_end(s);
        twilink::Report r{};
        const bool rep = acted && report(r);
        print_pass(s, st);
        if (press == 0u) {
            silent_start = acted ? start_sum / tenures : 0u;
        }
        // Were the ticks independent of the tenures, a start() of the
        // control's length would hold one with the chance length / period,
        // so the landings would number tenures x length / period; the
        // measured count against it says whether the tenure cycle and the
        // pressure's period lock.
        const uint32_t period = static_cast<uint32_t>(s.period_us) * cycles_per_us;
        print(serial, "  [", pass_label(s), "] ", tenures, " writes of ", fill_tx, ": start() ",
              acted ? start_sum / tenures : 0u, " cycles on average, preemptions inside included (the longest ",
              start_worst, "); landed in start() ", in_start_n, " (independent ticks would give about ",
              period != 0u ? static_cast<uint32_t>(tenures) * silent_start / period : 0u,
              "), of them with the host on the wire ", st.marked, "; the worst post to reply ", us_of(worst), " us; ",
              bad, " not i2c_ok", crlf);
        if (rep) {
            print_report(r);
        }
        bench.verdict(pass_label(s), "every write i2c_ok, the peer's count, sum and address matches exact",
                      acted && bad == 0u && rep && r.count == a.count && r.sum == sum && r.addr_hits == tenures &&
                          ran_to_count(r));
    }
}
