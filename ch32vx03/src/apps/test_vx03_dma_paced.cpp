// test_vx03_dma_paced - the reference bench suite for the requests the
// timers of the CH32V203 and the CH32V303 raise on their DMA controllers
// (ch32vx03/dma.hpp over RM ch. 11): a channel paced by a timer's update
// with no pad and no core in the path - circular mode, a staircase, the
// burst engine, the receive engine's ring at speed - and on the CH32V303
// the second controller's timer rows and its two block engines, letters k
// and l, registered on the part that has it and compiled out of every
// other image.
//
// One of three suites over the chapter: test_vx03_dma walks every channel
// memory to memory, test_vx03_dma_serial takes the USARTs' requests. A
// channel has one owner for the whole image (docs/design/dma.md, "The
// ownership rule"), and here DMA1's channel 2 is TIM2's update's; on the
// CH32V303 DMA2's channel 2 is TIM5's update's, 3 TIM6's, 4 TIM7's and 1
// TIM8's - the ping-pong engine's pace, apart from the player's.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT IT MEASURES WITH. Nothing outside the chip. A channel that needs a
// request takes it from a TIMER, whose update reaches the controller over
// an internal line and no pad; a second timer free-running is the datum a
// staircase samples; and the core's cycle count (the ticker's) is the ruler.
//
// THE PADS. None. NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14 (the
// debug port), PA11/PA12 (the USB pads), PC14/PC15 and PD0/PD1 (the
// crystals), PA0 (the CH32V203 board's KEY) - and PB2, the CH32V203
// board's LED, left undriven, as in every suite of this target.
//
// What is exercised, letter by letter:
//   b  CIRCULAR MODE: a run of compares poured into a timer by its own
//      update request, the buffer reloaded lap after lap, the half and
//      complete flags alternating and remaining() walking down and back
//   e  THE REQUEST PATH WITH NO WIRE: a timer's update copying another
//      timer's counter into a buffer once per period (the values a
//      staircase whose step is the period), and the BURST ENGINE
//      writing four compares from memory through one address
//   o  THE RECEIVE RING AT SPEED: the receive engine's circular shape on
//      TIM2's update request, copying TIM3's counter into a ring of 256
//      half-words lap after lap, read behind the channel through
//      util/ring.hpp's HardwareRing - every element judged against the one
//      before it at 500 thousand and at 3 million elements a second, the
//      laps counted checked against the pace, the wrap seen from the core,
//      a consumer stalling a lap and a half now and then, and a run held
//      while the producer laps it
// and on the CH32V303 alone, the second controller:
//   k  REQUESTS ON DMA2: TIM5's update copying another timer's counter
//      into a buffer once a period (table 11-3's row), and TIM6's and
//      TIM7's wherever this build's timer driver reaches those timers
//   l  THE BLOCK ENGINES ON DMA2: the loop engine playing a table into
//      TIM5's compare lap after lap on TIM5's update (channel 2), and the
//      ping-pong engine filling two blocks in turn on TIM8's update
//      (channel 1), each serviced from its channel's own handler
//
// build: boards = v203c6,v203c8,v303cb,v303vc
// build: monitor_speed = 115200

#include <stdint.h>
#include <span>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
#include "util/ring.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;

using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

TestBench<Serial> bench;

using Pacer = Tim<2>;    ///< the request source: its update drives channel 2
using Datum = Tim<3>;    ///< free-running, the datum the staircase samples
/// Channel 2 is TIM2's update's for the whole image - letters b and e
/// program it by hand, letter o's receive engine rides it
/// (docs/design/dma.md, "The ownership rule").
static_assert(dma_claim<DmaSlotId<1, DmaRequestOf<DmaRequest::tim2_up>::channel>,
                        DmaOwner<DmaPeriph::tim, 2, DmaSignal::update>>);

constexpr uint32_t tim_hz = Pacer::clock_hz(clock);
constexpr uint32_t ticks_per_us = tim_hz / 1'000'000u;
static_assert(tim_hz == SysClock::hz);

// ---------------------------------------------------------------------------
// The buffers
// ---------------------------------------------------------------------------

/// The half-word landing ground of letters b, e, k and l.
alignas(4) uint16_t work[16];
constexpr uint16_t sample_count = 16;

/// The suite's ruler: the ticker's cycle count, the kernel tick and STK's
/// position composed (util/test_wait.hpp). Right while the tick handler
/// runs at least once a period: no span below is measured with interrupts
/// masked for longer than a tick, or with the tick paused.
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};

/// Microseconds on a stopwatch over the ruler.
uint32_t elapsed_us(const Stopwatch<Ruler>& w) { return w.elapsed() / ticks_per_us; }

/// HOW LATE A POLL SEES ITS EVENT, letter b's budget: one turn of its loop
/// (the channel's flags, a clear, remaining() and Ticker::cycles(), about
/// 40 instructions in the release listing, four of them loads across the
/// peripheral bus: under 150 cycles) and the handlers that can be pending
/// at the event, each served once - the tick and the console's transmit,
/// each under 200 cycles (a tick's entry some 200 cycles,
/// docs/ch32vx03/i2c.md; a console byte about 150, usart.md). 550 cycles,
/// four microseconds at 144 MHz, rounded up.
constexpr uint32_t poll_late_cycles = 150u + 2u * 200u;
constexpr uint32_t poll_late_us = (poll_late_cycles + ticks_per_us - 1u) / ticks_per_us;

/// A staircase's step, judged (letters e and k): the datum and the pacer
/// divide one HCLK by the same prescaler, so the datum moves exactly one
/// count a microsecond between two updates 500 us apart, and the
/// controller reads it a service latency after each update - a few bus
/// cycles of arbitration that move from sample to sample, far under the
/// 144 cycles of a count, but able to straddle a count's edge: 500, give
/// or take one.
constexpr bool staircase_step(uint16_t step) { return step >= 499u && step <= 501u; }

// ---------------------------------------------------------------------------
// The handlers' counters
// ---------------------------------------------------------------------------

volatile uint16_t channel_calls[8] = {};
volatile uint32_t channel_mask[8] = {};

void note(uint8_t index, uint32_t flags) {
    if (flags != 0u) {
        channel_calls[index] = static_cast<uint16_t>(channel_calls[index] + 1u);
        channel_mask[index] = channel_mask[index] | flags;
    }
}

// ---------------------------------------------------------------------------
// Transfers, named once
// ---------------------------------------------------------------------------

/// Poll a channel's completion for at most `budget_us`: a wedge must end
/// the letter, not the session. The default, 200 ms, is twenty-five times
/// the longest transfer it waits for (sixteen updates 500 us apart).
bool wait_complete(uint32_t shift, uint32_t budget_us = 200'000u) {
    return wait_until<Ruler>(
        [shift] { return ((Dma<1>::flags() >> shift) & DmaFlag::complete) != 0u; }, budget_us);
}

/// Every channel, every line and both timers back to reset, at the top
/// of each letter, so no letter inherits another's state.
void all_off() {
    Dma<1>::open();
    Dma<1>::stop_all();
    for (uint8_t ch = 1; ch <= Dma<1>::channel_count; ++ch) {
        Pfic::disable(dma_channel_irq(1, ch));
        Pfic::clear_pending(dma_channel_irq(1, ch));
    }
    Pacer::release();
    Datum::release();
    for (uint8_t i = 0; i < 8u; ++i) {
        channel_calls[i] = 0;
        channel_mask[i] = 0;
    }
}

// ===========================================================================
// b - circular mode, paced by a timer's own update request
// ===========================================================================
void tb_circular() {
    all_off();
    using Ch = DmaChannel<1, DmaRequestOf<DmaRequest::tim2_up>::channel>;
    static_assert(Ch::number == 2, "table 11-5: TIM2's update is channel 2's request");

    // A run of eight compares, poured one per update into the timer's
    // own first channel. Circular, so the eighth update starts the run
    // again from its beginning.
    for (uint16_t i = 0; i < 8u; ++i) {
        work[i] = static_cast<uint16_t>(100u + 10u * i);
    }

    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 199});   // an update every 200 us
    (void)Pacer::output_channel(
        0, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = 1, .preload = false});

    Ch::stop();
    const bool armed = Ch::prepare(DmaTransfer{
        .peripheral = Pacer::chcvr_address(0),
        .memory = work,
        .count = 8,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .circular = true,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    const bool circular_read_back = Ch::configuration().circular;
    Ch::enable(true);

    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);

    // Four laps: the half and complete flags alternate, one of each a
    // lap, and the count reloads without a hand on it.
    uint16_t half_seen = 0;
    uint16_t full_seen = 0;
    uint16_t walked_down = 0;
    uint16_t last_remaining = Ch::remaining();
    Stopwatch<Ruler> w;
    w.start();
    while (full_seen < 4u && w.elapsed() < us_cycles<Ruler>(20'000UL)) {
        const uint32_t f = Ch::flags();
        if ((f & DmaFlag::half) != 0u) {
            ++half_seen;
            Ch::clear(DmaFlag::half | DmaFlag::global);
        }
        if ((f & DmaFlag::complete) != 0u) {
            ++full_seen;
            Ch::clear(DmaFlag::complete | DmaFlag::global);
        }
        const uint16_t now = Ch::remaining();
        if (now < last_remaining) {
            ++walked_down;
        }
        last_remaining = now;
    }
    const uint32_t laps_us = elapsed_us(w);
    const bool still_enabled = Ch::enabled();
    const uint32_t landed = Pacer::compare(0);

    Pacer::enable(false);
    print(serial, "  four laps of eight in ", laps_us, " us: half x", half_seen, ", complete x",
          full_seen, ", ", walked_down, " steps down, CH1CVR=", landed, crlf);
    bench.verdict("a circular channel reloads its count and its addresses by itself: the half "
                  "and complete flags come one per lap and the channel is still enabled",
                  armed && circular_read_back && full_seen >= 4u && half_seen >= 4u &&
                      still_enabled);
    bench.verdict("and the count WALKS: remaining() was seen stepping down through the run "
                  "and the value that landed in the timer is one the run carries",
                  walked_down >= 8u && landed >= 100u && landed <= 170u);
    // THE BOUND. The first update comes 200 us after CEN and the
    // thirty-second ends the fourth lap at 6400 us; the window opens after
    // CEN (a few cycles: the bound's floor, one microsecond under, covers
    // elapsed_us()'s truncation) and closes when the poll sees that flag,
    // poll_late_us at most after it.
    bench.verdict("the laps are paced by the REQUEST and not by the bus: thirty-two updates "
                  "of 200 us are 6400 us, which is what the core's counter says to within "
                  "the poll's lateness",
                  laps_us >= 6399UL && laps_us <= 6400UL + poll_late_us);
    all_off();
}
// ===========================================================================
// e - the request path with no wire: a staircase, and the burst engine
// ===========================================================================
void te_request() {
    all_off();
    using Ch = DmaChannel<1, DmaRequestOf<DmaRequest::tim2_up>::channel>;

    // A free-running datum at one megahertz, and a pacer whose update
    // moves one sample of it into memory every 500 us.
    Datum::init();
    (void)Datum::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 0xFFFF});
    Datum::enable(true);

    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 499});

    for (uint16_t i = 0; i < sample_count; ++i) {
        work[i] = 0;
    }
    Ch::stop();
    const bool armed = Ch::load(DmaTransfer{
        .peripheral = Datum::cnt_address(),
        .memory = work,
        .count = sample_count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);
    const bool sampled = wait_complete(Ch::flag_shift);
    Pacer::enable(false);
    Datum::enable(false);

    // The values are a staircase whose step is the pacer's period.
    uint16_t steps_right = 0;
    for (uint16_t i = 1; i < sample_count; ++i) {
        const uint16_t step = static_cast<uint16_t>(work[i] - work[i - 1u]);
        if (staircase_step(step)) {
            ++steps_right;
        }
    }
    print(serial, "  sixteen samples of a 1 MHz counter 500 us apart: ", work[0], " ", work[1],
          " ", work[2], " ... ", work[sample_count - 1u], crlf);
    bench.verdict("a timer's update request moves one sample per period with no pad and no "
                  "core: sixteen samples of another timer's counter, each a period (500 "
                  "counts) after the last to within one count",
                  armed && sampled && steps_right == sample_count - 1u);

    // THE BURST ENGINE. One request, several registers: each access to
    // the timer's DMAADR lands on CTLR1 + DBA + an index the controller
    // walks, so four compares are written through ONE address.
    all_off();
    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                     .period = 999});
    for (uint8_t ch = 0; ch < 4u; ++ch) {
        (void)Pacer::output_channel(
            ch, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = 0, .preload = false});
    }
    work[0] = 111;
    work[1] = 222;
    work[2] = 333;
    work[3] = 444;
    const bool window = Pacer::dma_burst(TimBurstBase::ch1cvr, 4);
    const bool window_read_back =
        Pacer::burst_base() == TimBurstBase::ch1cvr && Pacer::burst_length() == 4u;

    Ch::stop();
    const bool burst_armed = Ch::load(DmaTransfer{
        .peripheral = Pacer::dmaadr_address(),
        .memory = work,
        .count = 4,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);
    const bool burst_done = wait_complete(Ch::flag_shift);
    Pacer::enable(false);

    const uint32_t got[4] = {Pacer::compare(0), Pacer::compare(1), Pacer::compare(2),
                             Pacer::compare(3)};
    print(serial, "  the burst wrote CH1CVR..CH4CVR = ", got[0], " ", got[1], " ", got[2], " ",
          got[3], crlf);
    bench.verdict("the burst engine turns one request into a walk of four registers through "
                  "the single address TIMx_DMAADR - four compares written from memory, read "
                  "back where the waveform reads them",
                  window && window_read_back && burst_armed && burst_done && got[0] == 111u &&
                      got[1] == 222u && got[2] == 333u && got[3] == 444u);
    all_off();
}

// ===========================================================================
// o - the receive ring at speed: a timer's staircase, lap after lap
// ===========================================================================

/// The receive engine in its circular shape, on TIM2's update request,
/// which table 11-5 puts on channel 2: every update copies TIM3's counter
/// into the next slot of a ring the core reads behind it.
using RingRx = DmaRxEngine<1, DmaRequestOf<DmaRequest::tim2_up>::channel>;
using RingRxChannel = DmaChannel<1, DmaRequestOf<DmaRequest::tim2_up>::channel>;
constexpr uint32_t ring_length = 256;
/// Half-words, the counter's width - aligned to its own size where the
/// channel is held to the 64 KB rule, so the CH32V303's DMA1 takes it.
alignas(RingRx::bounded_to_64k ? 512 : 4) uint16_t ring_storage[ring_length];
using RingView = HardwareRing<ring_storage, RingRx>;

/// Channel 2's vector serves the ring while letter o runs.
volatile bool ring_mode = false;

/// The ring's completion, as a transport's handler does it: a lap counted,
/// an error abandoned. False while the ring is not running.
[[gnu::always_inline]] inline bool ring_vector() {
    if (!ring_mode) {
        return false;
    }
    const uint8_t f = RingRx::service();
    if ((f & RingRx::flag_error) != 0u) {
        (void)RingRx::abandon();
    } else if ((f & RingRx::flag_complete) != 0u) {
        RingRx::lap();
    }
    return true;
}

/// What a consumer read off the ring.
struct Drink {
    uint32_t elements = 0;    ///< handed out in runs
    uint32_t bad_steps = 0;   ///< steps that were not the pace's
    uint32_t torn = 0;        ///< runs consume() refused
    uint32_t skips = 0;       ///< looks that counted a lap missed
    uint32_t stalls = 0;
    uint32_t looks = 0;
    uint32_t cycles = 0;
};

/// Read `target` elements off the ring as fast as the core can, every one
/// judged against the one before it: a counter sampled every `period`
/// cycles steps by `period`, give or take how the controller's service
/// latency moves from one sample to the next. Every `stall_every`
/// elements (0: never) the consumer first stands still for `stall` cycles.
///
/// THE STEP'S JUDGEMENT is the nearest whole number of periods: a step
/// within half a period of one period is the next update's sample. What
/// the judgement exists to catch is a whole number of periods away - a
/// skipped sample two, a repeated one none, a slot still holding the
/// previous lap's sample a lap's worth (256 periods, modulo the counter's
/// sixteen bits 8192 cycles at 288 and 12288 at 48) - while the service
/// latency's wander, which no written figure bounds, would have to reach
/// half a period (24 cycles at the fast pace) to be taken for one.
Drink drink(uint32_t target, uint32_t period, uint32_t stall_every = 0, uint32_t stall = 0) {
    Drink d;
    bool have_last = false;
    uint16_t last = 0;
    uint32_t next_stall = stall_every;
    uint32_t seen = RingView::overruns();
    Stopwatch<Ruler> w;
    w.start();
    while (d.elements < target && d.cycles < 400'000'000UL) {
        if (stall_every != 0u && d.elements >= next_stall) {
            next_stall += stall_every;
            ++d.stalls;
            Stopwatch<Ruler> s;
            s.start();
            while (s.elapsed() < stall) {
            }
        }
        const std::span<const uint16_t> run = RingView::read_span();
        ++d.looks;
        if (RingView::overruns() != seen) {
            seen = RingView::overruns();
            ++d.skips;
            have_last = false;
        }
        for (const uint16_t v : run) {
            if (have_last) {
                const uint32_t step = static_cast<uint16_t>(v - last);
                if (2u * step <= period || 2u * step >= 3u * period) {
                    ++d.bad_steps;
                }
            }
            last = v;
            have_last = true;
        }
        if (!RingView::consume(static_cast<uint32_t>(run.size()))) {
            ++d.torn;
            seen = RingView::overruns();
            have_last = false;
        }
        d.elements += static_cast<uint32_t>(run.size());
        d.cycles = w.elapsed();
    }
    return d;
}

/// Whatever the ring holds, read away, with the pace stopped.
void drain_ring() {
    for (;;) {
        const std::span<const uint16_t> run = RingView::read_span();
        if (run.empty()) {
            return;
        }
        (void)RingView::consume(static_cast<uint32_t>(run.size()));
    }
}

void print_drink(const char* what, const Drink& d) {
    print(serial, "  ", what, ": ", d.elements, " elements in ", d.looks, " looks, ", d.cycles,
          " cycles; bad steps ", d.bad_steps, ", skips ", d.skips, ", torn ", d.torn,
          ", stalls ", d.stalls, crlf);
}

void to_ring() {
    all_off();
    constexpr uint32_t slow = 288;   // an update every 2 us
    constexpr uint32_t fast = 48;    // every third of a microsecond
    constexpr uint32_t lap_cycles = ring_length * slow;
    for (uint16_t& v : ring_storage) {
        v = 0;
    }

    Datum::init();
    (void)Datum::configure(TimConfig{.prescaler = 0, .period = 0xFFFF});
    Datum::enable(true);
    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = 0, .period = static_cast<uint16_t>(slow - 1u)});

    // THE SHAPE: bound once, the storage the ring; started, the count at
    // its length and one bus master counted.
    const uint8_t masters_before = BusActivity::active();
    ring_mode = true;
    const bool armed = RingRx::arm(Datum::cnt_address(), ring_storage, DmaPriority::high);
    // arm() keeps the ring's word in the engine and leaves the channel
    // stopped; start() is the store that puts it in CFGR.
    const uint32_t before_start = RingRxChannel::regs().CFGR;
    RingView::clear();
    RingView::clear_overruns();   // the letter's own stalls, a run before in this session
    const bool started = RingRx::start();
    const DmaChannelConfig cfg = RingRxChannel::configuration();
    const uint32_t irqs = RingRxChannel::armed();
    const uint32_t first_count = RingRx::remaining();
    const uint8_t masters_running = BusActivity::active();
    print(serial, "  CFGR after arm() ", hex(before_start), "; after start(): ",
          cfg.circular ? "circular" : "ONE-SHOT", cfg.memory_increment ? ", memory increment" : "",
          ", interrupts ", hex(irqs), ", count ", first_count, " of ", ring_length,
          ", bus masters ", masters_before, " -> ", masters_running, crlf);
    bench.verdict("the circular shape: arm() binds the ring's whole storage and leaves the "
                  "channel stopped, and start() runs it - circular, the memory side "
                  "incrementing, half-words both sides, the lap and the error armed and not the "
                  "half, the count at the storage's length and one bus master counted",
                  armed && started && before_start == 0u && cfg.circular && cfg.memory_increment &&
                      !cfg.peripheral_increment && cfg.peripheral_width == DmaWidth::half &&
                      cfg.memory_width == DmaWidth::half &&
                      irqs == (DmaFlag::complete | DmaFlag::error) &&
                      first_count == ring_length && masters_running == masters_before + 1u);

    // A CONSUMER THAT KEEPS UP, at two paces: every element in order, the
    // step between neighbours the pace's, nothing skipped and nothing torn.
    // A count that ran ahead of a store would hand out a slot still holding
    // the previous lap's sample - a step off by a whole lap's worth.
    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);
    const Drink a = drink(1UL << 17, slow);
    // Where the producer is by laps() and the count, read the moment the
    // consumer stops - the pace still running - against the updates the
    // pace made in the consumer's time. laps() is the channel handler's
    // count and remaining() the controller's, so a wrap between the two
    // reads is a whole lap (256) against the four judged: laps, the count,
    // and laps again a microsecond later - long past the handler's entry,
    // a lap lasting 512 us - taken again until laps held still.
    uint32_t position = 0;
    for (uint8_t tries = 0; tries < 8u; ++tries) {
        const uint32_t laps0 = RingRx::laps();
        const uint32_t count = RingRx::remaining();
        spin_us<Ruler>(1);
        position = laps0 * ring_length + ((ring_length - count) & (ring_length - 1u));
        if (RingRx::laps() == laps0) {
            break;
        }
    }
    const uint32_t due = a.cycles / slow;
    // The count read in a tight loop across laps: what the wrap looks like
    // from the core - the view takes zero and the length alike.
    uint32_t zeros = 0;
    uint32_t highest = 0;
    const uint32_t laps_from = RingRx::laps();
    for (uint32_t i = 0; i < 100'000UL; ++i) {
        const uint32_t r = RingRx::remaining();
        if (r == 0u) {
            ++zeros;
        }
        if (r > highest) {
            highest = r;
        }
    }
    const uint32_t laps_across = RingRx::laps() - laps_from;
    Pacer::enable(false);
    print_drink("at one update every 288 cycles", a);
    print(serial, "  the producer's position from laps() and the count: ", position,
          "; the updates the pace made in the consumer's time: ", due, crlf);
    bench.verdict("a consumer that keeps up reads 131072 elements across 512 laps with every "
                  "step the pace's - no count ahead of its store, nothing skipped, no run torn",
                  a.elements >= (1UL << 17) && a.bad_steps == 0u && a.skips == 0u &&
                      a.torn == 0u && RingView::overruns() == 0u);
    bench.verdict("and every lap was counted: the position laps() and the count give is the "
                  "number of updates the pace made in that time, to within four",
                  position + 4u >= due && position <= due + 4u);
    print(serial, "  the count read 100000 times across ", laps_across, " laps: ", zeros,
          " zeros, highest ", highest, crlf);
    bench.verdict("the count never read above the storage's length, and the reload was seen: a "
                  "wrap reads the length again", highest == ring_length);

    // A new period is loaded by an update event, which with the request
    // enabled moves one sample off the pace - landing some cycles after the
    // store that raised it, so it is waited for and drained before the pace
    // runs.
    // The probe above read no run for a score of laps: the drain counts
    // that lap missed, and the count starts again from zero.
    (void)Pacer::configure(TimConfig{.prescaler = 0, .period = static_cast<uint16_t>(fast - 1u)});
    spin_us<Ruler>(5);
    drain_ring();
    RingView::clear_overruns();
    Pacer::enable(true);
    const Drink b = drink(1UL << 17, fast);
    // The same probe at this pace, where a read lands on a lap's end often
    // enough to catch the count between the decrement and the reload.
    uint32_t fast_zeros = 0;
    uint32_t fast_highest = 0;
    const uint32_t fast_from = RingRx::laps();
    for (uint32_t i = 0; i < 100'000UL; ++i) {
        const uint32_t r = RingRx::remaining();
        if (r == 0u) {
            ++fast_zeros;
        }
        if (r > fast_highest) {
            fast_highest = r;
        }
    }
    const uint32_t fast_across = RingRx::laps() - fast_from;
    Pacer::enable(false);
    print_drink("at one update every 48 cycles", b);
    print(serial, "  the count read 100000 times across ", fast_across, " laps: ", fast_zeros,
          " zeros, highest ", fast_highest, crlf);
    bench.verdict("and at six times the pace - 3 million elements a second - the same: every "
                  "step the pace's, nothing skipped, no run torn",
                  b.elements >= (1UL << 17) && b.bad_steps == 0u && b.skips == 0u &&
                      b.torn == 0u && RingView::overruns() == 0u && fast_highest <= ring_length);

    // A CONSUMER THAT FALLS A LAP AND A HALF BEHIND, now and then: each
    // stall a lap missed, counted once and skipped, and the steps between
    // the skips still the pace's.
    (void)Pacer::configure(TimConfig{.prescaler = 0, .period = static_cast<uint16_t>(slow - 1u)});
    spin_us<Ruler>(5);
    drain_ring();
    RingView::clear_overruns();
    Pacer::enable(true);
    const Drink c = drink(1UL << 15, slow, 4096, lap_cycles + lap_cycles / 2u);
    Pacer::enable(false);
    const uint32_t stalled_overruns = RingView::overruns();
    print_drink("stalling a lap and a half every 4096", c);
    bench.verdict("a consumer that falls a lap and a half behind loses exactly that: one "
                  "overrun counted and skipped per stall, and the steps between the skips "
                  "still the pace's",
                  c.stalls >= 7u && c.skips == c.stalls && stalled_overruns == c.stalls &&
                      c.bad_steps == 0u && c.torn == 0u);

    // A RUN HELD WHILE THE PRODUCER LAPS IT: refused at its release.
    drain_ring();
    RingView::clear_overruns();
    Pacer::enable(true);
    uint8_t refused = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        drain_ring();
        std::span<const uint16_t> held{};
        // A run within a millisecond (an element every 2 us); an empty one
        // is never counted as refused below.
        (void)wait_until<Ruler>(
            [&held] {
                held = RingView::read_span();
                return !held.empty();
            },
            1000u);
        Stopwatch<Ruler> s;
        s.start();
        while (s.elapsed() < lap_cycles + lap_cycles / 2u) {
        }
        if (!held.empty() && !RingView::consume(static_cast<uint32_t>(held.size()))) {
            ++refused;
        }
    }
    const uint32_t held_overruns = RingView::overruns();
    Pacer::enable(false);
    Pacer::interrupts(tim_ude, false);
    print(serial, "  four runs held across a lap and a half: ", refused,
          " refused at their release, overruns ", held_overruns, crlf);
    bench.verdict("a run the producer laps while it is held is refused at its release - "
                  "consume() answers false and counts the overrun - every time",
                  refused == 4u && held_overruns == 4u);

    RingRx::stop();
    const uint8_t masters_after = BusActivity::active();
    print(serial, "  laps counted ", RingRx::laps(), "; bus masters after stop(): ",
          masters_after, crlf);
    bench.verdict("stop() ends the ring and gives its bus-master count back",
                  masters_after == masters_before && RingRx::idle());
    ring_mode = false;
    all_off();
}

// ===========================================================================
// The second controller - the CH32V303's alone (letters k and l)
// ===========================================================================
//
// Every letter below is a template on the controller's number, 2 by
// default, and does its work inside `if constexpr` on the part's count of
// controllers: on a part with one the body is a discarded branch the
// compiler never instantiates, and the image carries none of it - its
// state included, which lives in a template for the same reason.

/// What DMA2's handlers serve besides the bare channel: the two block
/// engines of letter l.
enum class Dma2Role : uint8_t { bare, player, source };

template <uint8_t C = 2>
struct Dma2State {
    static inline volatile uint16_t calls[11] = {};
    static inline volatile uint32_t mask[11] = {};
    static inline volatile Dma2Role role = Dma2Role::bare;
    /// The ping-pong engine's two blocks.
    static inline volatile uint16_t block_a[8] = {};
    static inline volatile uint16_t block_b[8] = {};

    static void note(uint8_t index, uint32_t flags) {
        if (flags != 0u) {
            calls[index] = static_cast<uint16_t>(calls[index] + 1u);
            mask[index] = mask[index] | flags;
        }
    }

    static void reset() {
        role = Dma2Role::bare;
        for (uint8_t i = 0; i < 11u; ++i) {
            calls[i] = 0;
            mask[i] = 0;
        }
    }
};

/// The engines letter l drives: TIM5's update request, which table 11-3
/// puts on DMA2's channel 2, pacing the player, and TIM8's, on DMA2's
/// channel 1, the source - each request its own engine's for the whole
/// image (docs/design/dma.md, "The ownership rule"). Every name here hangs
/// on a template parameter, so nothing of it is formed on a part without
/// the timers or the controller.
template <uint8_t C = 2, uint8_t timer = 5, DmaRequest request = DmaRequest::tim5_up,
          uint8_t source_timer = 8, DmaRequest source_request = DmaRequest::tim8_up>
struct Dma2Engines {
    using Row = DmaRequestOf<request>;
    using SourceRow = DmaRequestOf<source_request>;
    static_assert(Row::controller == C && SourceRow::controller == C,
                  "table 11-3: TIM5's and TIM8's updates are DMA2 requests");
    using Paced = Tim<timer>;
    using SourcePaced = Tim<source_timer>;
    using Player = DmaLoopEngine<C, Row::channel, uint16_t>;
    using Source = DmaPingPongEngine<C, SourceRow::channel, uint16_t>;
    static_assert(dma_claim<DmaSlotId<C, Row::channel>, DmaOwner<DmaPeriph::tim, timer, DmaSignal::update>>);
    static_assert(dma_claim<DmaSlotId<C, SourceRow::channel>,
                            DmaOwner<DmaPeriph::tim, source_timer, DmaSignal::update>>);
};

template <uint8_t C>
void all_off2() {
    if constexpr (C <= device::dma_controller_count) {
        Dma<C>::open();
        Dma<C>::stop_all();
        for (uint8_t ch = 1; ch <= Dma<C>::channel_count; ++ch) {
            Pfic::disable(dma_channel_irq(C, ch));
            Pfic::clear_pending(dma_channel_irq(C, ch));
        }
        Dma2State<C>::reset();
    }
}

// ===========================================================================
// k - requests on DMA2: a timer's update, no pad
// ===========================================================================
/// The staircase of letter e on the second controller: timer `n`'s update
/// request moving one sample of the free-running datum a period. A
/// template on the timer, so that a timer this build's timer driver does
/// not reach is a discarded branch and a line that says so.
template <uint8_t n, DmaRequest request>
void dma2_staircase(const char* verdict) {
    if constexpr (tim_present(n)) {
        using Clocker = Tim<n>;
        using Row = DmaRequestOf<request>;
        using Ch = DmaChannel<Row::controller, Row::channel>;
        static_assert(dma_claim<DmaSlotId<Row::controller, Row::channel>,
                                DmaOwner<DmaPeriph::tim, n, DmaSignal::update>>);
        Datum::init();
        (void)Datum::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                         .period = 0xFFFF});
        Datum::enable(true);
        Clocker::init();
        (void)Clocker::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(Clocker::clock_hz(clock) / 1'000'000u - 1u),
            .period = 499});
        for (uint16_t i = 0; i < sample_count; ++i) {
            work[i] = 0;
        }
        Ch::stop();
        const bool armed = Ch::load(DmaTransfer{
            .peripheral = Datum::cnt_address(),
            .memory = work,
            .count = sample_count,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = DmaWidth::half,
                       .memory_width = DmaWidth::half,
                       .priority = DmaPriority::high},
        });
        Clocker::interrupts(tim_ude, true);
        Clocker::enable(true);
        // Sixteen updates 500 us apart: 8 ms, the budget 200 ms.
        const bool sampled =
            wait_until<Ruler>([] { return Ch::flag(DmaFlag::complete); }, 200'000u);
        Clocker::enable(false);
        Datum::enable(false);
        Ch::stop();
        uint16_t steps_right = 0;
        for (uint16_t i = 1; i < sample_count; ++i) {
            const uint16_t step = static_cast<uint16_t>(work[i] - work[i - 1u]);
            if (staircase_step(step)) {
                ++steps_right;
            }
        }
        print(serial, "  TIM", n, "'s update on DMA", Row::controller, " channel ", Row::channel,
              ": ", work[0], " ", work[1], " ... ", work[sample_count - 1u], crlf);
        bench.verdict(verdict, armed && sampled && steps_right == sample_count - 1u);
        Clocker::release();
        Datum::release();
    } else {
        print(serial, "  TIM", n, ": this build's timer driver does not reach it, so its "
                      "request is not driven here",
              crlf);
    }
}

template <uint8_t C = 2>
void tk_dma2_requests() {
    if constexpr (C <= device::dma_controller_count) {
        all_off();
        all_off2<C>();
        if constexpr (device::has_tim5) {
            dma2_staircase<5, DmaRequest::tim5_up>(
                "TIM5's update request moves a staircase through DMA2's channel 2 (table 11-3): "
                "sixteen samples of another timer's counter, each a period after the last");
        }
        dma2_staircase<6, DmaRequest::tim6_up>(
            "TIM6's update request moves the same staircase through DMA2's channel 3");
        dma2_staircase<7, DmaRequest::tim7_up>(
            "TIM7's update request moves the same staircase through DMA2's channel 4");
        all_off2<C>();
    }
}

// ===========================================================================
// l - the block engines on DMA2, each serviced from its channel's handler
// ===========================================================================
template <uint8_t C = 2>
void tl_dma2_engines() {
    if constexpr (C <= device::dma_controller_count && device::has_tim5 && tim_present(8)) {
        all_off();
        all_off2<C>();
        using E = Dma2Engines<C>;
        using Player = typename E::Player;
        using Source = typename E::Source;
        using S = Dma2State<C>;
        using Paced = typename E::Paced;
        using SourcePaced = typename E::SourcePaced;

        // THE PLAYER: a table of eight compares poured into TIM5's first
        // channel by its own update, for ever - the controller's circular
        // mode, the handler doing nothing but count the laps.
        for (uint16_t i = 0; i < 8u; ++i) {
            work[i] = static_cast<uint16_t>(100u + 10u * i);
        }
        Paced::init();
        (void)Paced::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(Paced::clock_hz(clock) / 1'000'000u - 1u),
            .period = 199});   // an update every 200 us
        (void)Paced::output_channel(
            0, TimChannelConfig{.mode = TimOutputMode::pwm1, .compare = 1, .preload = false});
        S::role = Dma2Role::player;
        Player::arm(Paced::chcvr_address(0), DmaPriority::high);
        const bool playing = Player::start(work, 8);
        Paced::interrupts(tim_ude, true);
        Paced::enable(true);
        Stopwatch<Ruler> w;
        w.start();
        // The laps reached and their time are what the verdict judges.
        (void)wait_until<Ruler>([] { return Player::laps() >= 8u; }, 50'000u);
        const uint32_t laps_us = elapsed_us(w);
        const uint32_t laps = Player::laps();
        const uint32_t landed = Paced::compare(0);
        const bool still_running = Player::running();
        Paced::enable(false);
        Player::stop();
        Paced::release();
        print(serial, "  the loop engine on DMA2 channel ", Player::channel, ": ", laps,
              " laps of eight 200 us updates in ", laps_us, " us, CH1CVR=", landed,
              ", faults ", Player::faults(), crlf);
        // THE BOUND, as letter b's: the sixty-fourth update at 12800 us
        // after CEN, seen poll_late_us at most after the channel's handler
        // counted the lap - a handler more before the count, under 200
        // cycles.
        constexpr uint32_t lap_late_us = (poll_late_cycles + 200u + ticks_per_us - 1u) / ticks_per_us;
        bench.verdict("DmaLoopEngine plays a table for ever on DMA2: eight laps of eight "
                      "updates counted by the channel's own handler at the timer's pace "
                      "(12800 us to within the poll's lateness), the value that landed one "
                      "the table carries",
                      playing && still_running && laps >= 8u && laps_us >= 12'799UL &&
                          laps_us <= 12'800UL + lap_late_us && landed >= 100u &&
                          landed <= 170u && Player::faults() == 0u);

        // THE SOURCE: TIM8's update, on its own channel, filling two
        // caller-owned blocks with samples of a free-running counter - the
        // handler hands a full block over and starts the other, the loop
        // below reads one and releases it.
        Datum::init();
        (void)Datum::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                         .period = 0xFFFF});
        Datum::enable(true);
        SourcePaced::init();
        (void)SourcePaced::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(SourcePaced::clock_hz(clock) / 1'000'000u - 1u),
            .period = 199});
        S::role = Dma2Role::source;
        Source::arm(Datum::cnt_address(), DmaPriority::high);
        const bool streaming = Source::start(S::block_a, S::block_b, 8);
        SourcePaced::interrupts(tim_ude, true);
        SourcePaced::enable(true);
        uint16_t blocks = 0;
        uint16_t first[8] = {};
        Stopwatch<Ruler> s;
        s.start();
        while (blocks < 4u && s.elapsed() < us_cycles<Ruler>(50'000UL)) {
            volatile uint16_t* full = Source::ready();
            if (full != nullptr) {
                if (blocks == 0u) {
                    for (uint8_t i = 0; i < 8u; ++i) {
                        first[i] = full[i];
                    }
                }
                (void)Source::release();
                ++blocks;
            }
        }
        SourcePaced::enable(false);
        Datum::enable(false);
        uint8_t steps = 0;
        for (uint8_t i = 1; i < 8u; ++i) {
            const uint16_t step = static_cast<uint16_t>(first[i] - first[i - 1u]);
            if (step >= 195u && step <= 205u) {
                ++steps;
            }
        }
        print(serial, "  the ping-pong engine on DMA2 channel ", Source::channel, ": ", blocks,
              " blocks of eight, overruns ", Source::overruns(), ", first block ", first[0], " ",
              first[1], " ... ", first[7], crlf);
        bench.verdict("DmaPingPongEngine fills two blocks in turn on DMA2, each a staircase of "
                      "the timer's period, handed over by the handler and released with no "
                      "overrun",
                      streaming && blocks >= 4u && steps == 7u && Source::overruns() == 0u);
        Source::stop();
        SourcePaced::release();
        Datum::release();
        all_off2<C>();
    }
}


/// The letters of the second controller, registered where there is one
/// AND the timers they pace it with: k on TIM5, TIM6 or TIM7, l on TIM5
/// and TIM8 - the CH32V303RC's and VC's, so the CH32V303CB, whose DMA2 has
/// none of them to answer, carries neither. A template, so a part without
/// them carries neither their code nor their prose.
template <bool on = (device::dma_controller_count >= 2u)>
void register_dma2_letters() {
    if constexpr (on && (device::has_tim5 || tim_present(6) || tim_present(7))) {
        bench.letter('k', "DMA2: request-gated transfers from the timers' updates",
                     tk_dma2_requests<>);
    }
    if constexpr (on && device::has_tim5 && tim_present(8)) {
        bench.letter('l', "DMA2: the loop and ping-pong engines", tl_dma2_engines<>);
    }
}

/// A DMA2 channel's handler body, where the part has the controller: the
/// block engine letter l handed the channel to, or the bare channel's
/// counters.
template <uint8_t ch, uint8_t C = 2>
void dma2_vector() {
    if constexpr (C <= device::dma_controller_count) {
        using S = Dma2State<C>;
        if constexpr (device::has_tim5 && tim_present(8)) {
            using E = Dma2Engines<C>;
            if constexpr (ch == E::Row::channel) {
                using Player = typename E::Player;
                if (S::role == Dma2Role::player) {
                    const uint8_t f = Player::service();
                    if ((f & Player::flag_error) != 0u) {
                        Player::fail();
                    } else if ((f & Player::flag_complete) != 0u) {
                        Player::lap();
                    }
                    return;
                }
            }
            if constexpr (ch == E::SourceRow::channel) {
                using Source = typename E::Source;
                if (S::role == Dma2Role::source) {
                    const uint8_t f = Source::service();
                    if ((f & Source::flag_error) != 0u) {
                        Source::fail();
                    } else if ((f & Source::flag_complete) != 0u) {
                        (void)Source::complete();
                    }
                    return;
                }
            }
        }
        S::note(static_cast<uint8_t>(ch - 1u), DmaChannel<C, ch>::isr());
    }
}

/// DMA1's eighth channel, the CH32V203's alone: the CH32V303's DMA1 has
/// seven, and its vector table has no entry for an eighth.
template <uint8_t ch = 8>
void dma1_eighth_vector() {
    if constexpr (dma_channel_exists(1, ch)) {
        note(static_cast<uint8_t>(ch - 1u), DmaChannel<1, ch>::isr());
    }
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf,
          device::dma_controller_count == 2u
              ? "test_vx03_dma_paced - the timers' requests on the two DMA controllers of RM ch. 11"
              : "test_vx03_dma_paced - the timers' requests on the DMA controller of RM ch. 11",
          crlf);
    bench.menu();
}

}  // namespace

// DMA1's vectors, eight on the CH32V203 and seven on the CH32V303, each
// counting its own channel's flags. Channel 2 is TIM2's update, and serves
// letter o's ring while it runs.
BRIO_CH32_VECTOR(dma1_channel1_handler) { note(0, brio::DmaChannel<1, 1>::isr()); }
BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (!ring_vector()) {
        note(1, brio::DmaChannel<1, 2>::isr());
    }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) { note(2, brio::DmaChannel<1, 3>::isr()); }
BRIO_CH32_VECTOR(dma1_channel4_handler) { note(3, brio::DmaChannel<1, 4>::isr()); }
BRIO_CH32_VECTOR(dma1_channel5_handler) { note(4, brio::DmaChannel<1, 5>::isr()); }
BRIO_CH32_VECTOR(dma1_channel6_handler) { note(5, brio::DmaChannel<1, 6>::isr()); }
BRIO_CH32_VECTOR(dma1_channel7_handler) { note(6, brio::DmaChannel<1, 7>::isr()); }
BRIO_CH32_VECTOR(dma1_channel8_handler) { dma1_eighth_vector(); }

// DMA2's eleven, the CH32V303's alone: on the other series these bodies
// are empty and nothing in the vector table names them.
BRIO_CH32_VECTOR(dma2_channel1_handler) { dma2_vector<1>(); }
BRIO_CH32_VECTOR(dma2_channel2_handler) { dma2_vector<2>(); }
BRIO_CH32_VECTOR(dma2_channel3_handler) { dma2_vector<3>(); }
BRIO_CH32_VECTOR(dma2_channel4_handler) { dma2_vector<4>(); }
BRIO_CH32_VECTOR(dma2_channel5_handler) { dma2_vector<5>(); }
BRIO_CH32_VECTOR(dma2_channel6_handler) { dma2_vector<6>(); }
BRIO_CH32_VECTOR(dma2_channel7_handler) { dma2_vector<7>(); }
BRIO_CH32_VECTOR(dma2_channel8_handler) { dma2_vector<8>(); }
BRIO_CH32_VECTOR(dma2_channel9_handler) { dma2_vector<9>(); }
BRIO_CH32_VECTOR(dma2_channel10_handler) { dma2_vector<10>(); }
BRIO_CH32_VECTOR(dma2_channel11_handler) { dma2_vector<11>(); }

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('b', "circular mode, paced by a timer's update request", tb_circular);
    bench.letter('e', "the request path with no wire: a staircase, and the burst engine",
                 te_request);
    bench.letter('o', "the receive ring at speed: a timer's staircase, lap after lap", to_ring);
    register_dma2_letters();

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
