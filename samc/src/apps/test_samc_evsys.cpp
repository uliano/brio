// test_samc_evsys - the reference bench suite for samc/evsys.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and that is not luck - it is the software event
// (29.6.2.12), which is serviced exactly as a generator's would be. An
// event system with no way to inject an event from software would need a
// second peripheral just to be tested; this one only needs a USER, and
// the DMAC is the one this stratum already has.
//
// THE MEASUREMENT IS A DMA TRANSFER THAT HAPPENED. A channel is armed
// with no hardware trigger at all - `dma_trigger_none`, EVACT trigger,
// EVIE set - so the ONLY thing that can move its bytes is an event
// arriving through EVSYS. If the destination buffer changes, the event
// was routed. That is a stronger statement than any status bit: it is
// the fabric doing its job end to end, through a driver that knew
// nothing about it (dmac.md's own gap list says every EVACT value but
// `none` was untested silicon - this is where that stops being true).
//
// What is exercised, letter by letter:
//   a  the fabric: the user multiplexer's off-by-one, the ordering rule,
//      the path/edge legality both ways, and the erratum-1.12.1 refusal
//   b  AN EVENT MOVES BYTES: software event -> channel -> DMAC, with
//      the transfer itself as the witness
//   c  the synchronous and resynchronized paths, which need a channel
//      clock - and the status surface that only they have
//   d  what the asynchronous path does NOT have: CHSTATUS and both
//      interrupt flags read zero - and, measured rather than read
//      anywhere, it does not carry a SOFTWARE event at all
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
using Led = Pin<'B', 23>;

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The fabric under test
//
// DMAC channel 0 is event user m = 5 (table 29-3), and the users are
// numbered so that channel k is user 5 + k. The event channel is 0, whose
// generic clock is EVSYS_GCLK_ID_0.
// ---------------------------------------------------------------------------
constexpr uint8_t dma_ch = 0;
constexpr uint8_t user_dmac_ch0 = 5;
constexpr uint8_t ev_ch = 0;
constexpr uint8_t gen_slow = 5;      // a generator this suite builds for the clock

using Copy = DmaChannel<dma_ch>;
using GenSlow = Gclk<gen_slow>;

// VOLATILE IN BOTH DIRECTIONS, the lesson the DMAC campaign paid for on
// this same target: the compiler cannot see the controller's writes, and
// it cannot see its reads either - it will happily sink the preparation
// of a buffer past the thing that starts the transfer.
constexpr uint16_t payload = 16;
volatile uint8_t src[payload];
volatile uint8_t dst[payload];

void fill_source(uint8_t seed) {
    for (uint16_t i = 0; i < payload; ++i) {
        src[i] = static_cast<uint8_t>(seed + i);
        dst[i] = 0;
    }
}

bool destination_matches(uint8_t seed) {
    for (uint16_t i = 0; i < payload; ++i) {
        if (dst[i] != static_cast<uint8_t>(seed + i)) {
            return false;
        }
    }
    return true;
}

bool destination_untouched() {
    for (uint16_t i = 0; i < payload; ++i) {
        if (dst[i] != 0u) {
            return false;
        }
    }
    return true;
}

/// Arm the DMA channel so that ONLY an event can move it: no hardware
/// trigger source, EVACT = trigger, EVIE set.
bool arm_event_driven_copy(uint8_t seed) {
    fill_source(seed);
    if (!Copy::reset()) {
        return false;
    }
    const DmaChannelConfig cfg{
        .trigger = dma_trigger_none,
        .action = DmaTriggerAction::block,
        .event_action = DmaEventAction::trigger,
        .event_input = true,
    };
    if (!Copy::configure(cfg)) {
        return false;
    }
    const DmaTransfer t{
        .source = &src[0],
        .destination = &dst[0],
        .beats = payload,
        .beat = DmaBeat::byte,
    };
    if (!Copy::load(t)) {
        return false;
    }
    return Copy::enable(true);
}

/// A short wait, long enough for a block of sixteen bytes to move and
/// for any channel clock this suite uses to tick.
void settle() {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < 100'000UL; ++i) {
        sink = sink + 1u;
    }
}

// =============================================================================
// a - the fabric
// =============================================================================
void ta_fabric() {
    Evsys::bus_clock(true);
    Evsys::reset();

    bench.verdict("twelve channels and forty-seven users",
                  Evsys::channel_count == 12u && Evsys::user_count == 47u);
    bench.verdict("a channel past the last is refused",
                  !Evsys::configure(12, EventChannelConfig{}));
    bench.verdict("and a user past the last is refused",
                  !Evsys::attach(47, 0));

    // THE OFF-BY-ONE. USER.CHANNEL holds channel+1 with zero meaning
    // "no channel", and the whole point of the driver's verbs is that a
    // caller never sees it - so this checks BOTH that plain numbers go
    // in and come out, AND that the register really holds the +1.
    bench.verdict("a user connects to a channel by its plain number",
                  Evsys::attach(user_dmac_ch0, 3));
    bench.verdict("and reads back as the same plain number",
                  Evsys::user_channel(user_dmac_ch0) == 3u);
    const uint32_t raw = EVSYS_REGS->EVSYS_USER[user_dmac_ch0];
    print(serial, "  USER[", user_dmac_ch0, "] for channel 3 holds ", raw,
          " - the register wants channel+1 (29.8.9)", crlf);
    bench.verdict("the register itself holds channel + 1", raw == 4u);

    Evsys::disconnect(user_dmac_ch0);
    bench.verdict("disconnecting reports no channel",
                  Evsys::user_channel(user_dmac_ch0) == Evsys::channel_count);

    // The path/edge relationship, which runs BOTH ways, and the erratum.
    bench.verdict("an asynchronous channel with an edge is refused",
                  !Evsys::configure(0, EventChannelConfig{
                      .path = EventPath::asynchronous, .edge = EventEdge::rising}));
    bench.verdict("a synchronous channel with NO edge is refused",
                  !Evsys::configure(0, EventChannelConfig{
                      .path = EventPath::synchronous, .edge = EventEdge::none}));
    bench.verdict("a synchronous channel with a free-running clock is refused "
                  "(erratum 1.12.1, every revision)",
                  !Evsys::configure(0, EventChannelConfig{
                      .path = EventPath::synchronous,
                      .edge = EventEdge::rising,
                      .on_demand = false}));
    bench.verdict("the same on the resynchronized path is fine - the erratum "
                  "is synchronous-only",
                  Evsys::configure(0, EventChannelConfig{
                      .path = EventPath::resynchronized,
                      .edge = EventEdge::rising,
                      .on_demand = false}));
    Evsys::release_channel(0);
}

// =============================================================================
// b - an event moves bytes
// =============================================================================
void tb_event_moves_bytes() {
    Evsys::reset();
    bench.verdict("the DMA channel arms with NO hardware trigger",
                  arm_event_driven_copy(0x40));
    bench.verdict("and nothing has moved yet", destination_untouched());

    // A software event on an unrouted channel must do nothing - which is
    // what makes the next verdict mean something.
    Evsys::trigger(ev_ch);
    settle();
    bench.verdict("a software event on a channel no user listens to moves "
                  "nothing",
                  destination_untouched());

    // Now route it. connect() writes the USER multiplexer first and the
    // channel second, which is 29.6.2.3's order.
    //
    // THE PATH IS SYNCHRONOUS AND THAT IS NOT ARBITRARY: measured here,
    // A SOFTWARE EVENT DOES NOT CROSS AN ASYNCHRONOUS CHANNEL. 29.6.2.12
    // says a software event "can be serviced as any event generator"
    // without qualifying by path, but the asynchronous path has no clock
    // and no edge detector, and a register write has no width of its own
    // to propagate - letter d holds the measurement.
    bench.verdict("the channel's own generic clock is routed",
                  GenSlow::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_ch), gen_slow));
    bench.verdict("the DMAC's channel-0 user connects to event channel 0",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{
                                     .path = EventPath::synchronous,
                                     .edge = EventEdge::rising}));
    // ERRATUM 1.12.4: a freshly configured channel is busy for one
    // channel-clock tick without CHBUSY showing it, so the first trigger
    // is paced rather than issued immediately.
    settle();

    Evsys::trigger(ev_ch);
    settle();

    print(serial, "  after one software event: dst[0..3] = ", dst[0], " ", dst[1],
          " ", dst[2], " ", dst[3], crlf);
    bench.verdict("THE EVENT MOVED THE BYTES - software event to EVSYS to "
                  "DMAC, with no CPU in the path",
                  destination_matches(0x40));
    bench.verdict("and the channel completed rather than erroring",
                  !Copy::fetch_error());

    // The same again with a different pattern, to show it is repeatable
    // and not a one-off left over from arming.
    bench.verdict("the channel re-arms", arm_event_driven_copy(0x90));
    bench.verdict("and is empty again", destination_untouched());
    settle();
    Evsys::trigger(ev_ch);
    settle();
    bench.verdict("a second event moves a second block", destination_matches(0x90));

    Evsys::disconnect(user_dmac_ch0);
    GclkChannel::disconnect(Evsys::gclk_id(ev_ch));
    (void)Copy::enable(false);
}

// =============================================================================
// c - the synchronous and resynchronized paths
// =============================================================================
//
// These need the channel's own generic clock (29.5.3), which is what the
// asynchronous path does without. They are also the only paths with any
// status at all.
void tc_clocked_paths() {
    Evsys::reset();

    // A slow-ish channel clock, so the latency the chapter quotes in
    // GCLK cycles is something the CPU could in principle notice.
    bench.verdict("a generator for the channel clock comes up",
                  GenSlow::configure(GclkConfig{.source = GclkSource::osculp32k}));
    bench.verdict("and the channel's own GCLK channel connects",
                  GclkChannel::connect(Evsys::gclk_id(ev_ch), gen_slow));

    for (const auto path : {EventPath::synchronous, EventPath::resynchronized}) {
        const char* name = path == EventPath::synchronous ? "synchronous"
                                                          : "resynchronized";
        bench.verdict("the channel arms", arm_event_driven_copy(0x20));

        // Both clocked paths need an edge; the software event is a pulse,
        // so a rising edge is what there is to catch.
        const bool routed = Evsys::connect(
            user_dmac_ch0, ev_ch,
            EventChannelConfig{.path = path, .edge = EventEdge::rising});
        bench.verdict("the ", name, routed);
        settle();   // erratum 1.12.4

        Evsys::clear_flags(Evsys::detected_flag(ev_ch) |
                           Evsys::overrun_flag(ev_ch));
        Evsys::trigger(ev_ch);
        settle();

        const bool moved = destination_matches(0x20);
        const bool saw_event = Evsys::detected(ev_ch);
        print(serial, "  ", name, ": bytes moved=", moved ? "yes" : "no",
              " EVD=", saw_event ? "1" : "0",
              " OVR=", Evsys::overrun(ev_ch) ? "1" : "0", crlf);

        bench.verdict("a clocked path carries the event to the DMAC", moved);
        // THE STATUS SURFACE THE ASYNCHRONOUS PATH DOES NOT HAVE: EVD is
        // set when an event coming from the channel is detected, and it
        // is only ever set on these two paths (29.6.2.10).
        bench.verdict("and raises the event-detected flag, which only a "
                      "clocked path has",
                      saw_event);

        Evsys::disconnect(user_dmac_ch0);
        (void)Copy::enable(false);
    }

    GclkChannel::disconnect(Evsys::gclk_id(ev_ch));
}

// =============================================================================
// d - what the asynchronous path does not have
// =============================================================================
//
// 29.6.2.9, .10 and .11 all say the same thing from three directions:
// with an asynchronous path the overrun flag, the event-detected flag
// and the whole channel status read as zero. Code that polls any of them
// to pace an asynchronous channel is polling a constant - which is worth
// proving rather than repeating.
void td_async_is_silent() {
    Evsys::reset();
    bench.verdict("the channel arms", arm_event_driven_copy(0x77));
    bench.verdict("routed asynchronously",
                  Evsys::connect(user_dmac_ch0, ev_ch,
                                 EventChannelConfig{.path = EventPath::asynchronous}));
    settle();

    Evsys::clear_flags(Evsys::detected_flag(ev_ch) | Evsys::overrun_flag(ev_ch));

    // Several events in quick succession - which on a clocked path would
    // be exactly how an overrun is provoked.
    for (uint8_t i = 0; i < 8u; ++i) {
        Evsys::trigger(ev_ch);
    }
    settle();

    const uint32_t chstatus = Evsys::channel_status();
    print(serial, "  after eight back-to-back events: CHSTATUS=", hex(chstatus),
          " EVD=", Evsys::detected(ev_ch) ? "1" : "0",
          " OVR=", Evsys::overrun(ev_ch) ? "1" : "0", crlf);

    // THE FINDING, and the chapter does not have it. 29.6.2.12 says a
    // software event "can be serviced as any event generator" with no
    // mention of the path; measured, EIGHT of them across an
    // asynchronous channel move NOTHING, while one across a synchronous
    // channel moves a whole block (letters b and c). The asynchronous
    // path has no clock and no edge detector, and a register write has
    // no width of its own - so there is nothing to propagate. What is
    // NOT claimed here is anything about a hardware generator on the
    // asynchronous path: this suite has no generator to wire, and the
    // measurement is about the software event alone.
    bench.verdict("A SOFTWARE EVENT DOES NOT CROSS AN ASYNCHRONOUS CHANNEL - "
                  "eight of them moved nothing, where one on a clocked path "
                  "moves a block",
                  destination_untouched());
    bench.verdict("the event-detected flag stays ZERO on an asynchronous "
                  "channel (29.6.2.10)",
                  !Evsys::detected(ev_ch));
    bench.verdict("so does the overrun flag, however many events pass "
                  "(29.6.2.9)",
                  !Evsys::overrun(ev_ch));
    bench.verdict("and CHSTATUS reports neither busy nor ready for it "
                  "(29.6.2.11)",
                  !Evsys::busy(ev_ch) && !Evsys::users_ready(ev_ch));

    Evsys::disconnect(user_dmac_ch0);
    (void)Copy::enable(false);
}

void banner() {
    print(serial, crlf, "test_samc_evsys - SAMC21J18A EVSYS (ch. 29) with the "
          "DMAC as its user, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// The DMAC's line is bound because a transfer that completes raises it,
/// and an unbound vector on this target is a silent death. Nothing here
/// needs the completion - the destination buffer is the witness - so the
/// handler only drains what it is told.
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        (void)irq;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    const bool dma_ok = brio::Dmac::init();
    brio::Nvic::enable(brio::Dmac::irq());
    brio::enable_interrupts();

    bench.letter('a', "the fabric, its off-by-one and its refusals", ta_fabric);
    bench.letter('b', "an event moves bytes, with no CPU in the path",
                 tb_event_moves_bytes);
    bench.letter('c', "the synchronous and resynchronized paths", tc_clocked_paths);
    bench.letter('d', "what the asynchronous path does not have", td_async_is_silent);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " dmac=", dma_ok ? "up" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
