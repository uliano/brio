// test_samc_evsys - the reference bench suite for samc21/evsys.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and that is not luck - it is the software event
// (29.6.2.12), which is serviced exactly as a generator's would be. An
// event system with no way to inject an event from software would need a
// second peripheral just to be tested; this one only needs a USER, and
// a timer counting events is the cheapest one the die offers.
//
// THE MEASUREMENT IS A COUNT OF EVENTS THAT ARRIVED. TC4 runs in COUNT16
// with EVCTRL.EVACT = COUNT and TCEI set (35.6.2.5.3): its counter
// advances on an incoming event and on nothing else - the prescaler is
// bypassed, and no clock tick moves it. Its user, TC4 EVU, is wired to
// the channel under test (table 29-3 gives it all three paths), and the
// counter, read through READSYNC, says how many events crossed. That is
// a stronger statement than any status bit and stronger than "something
// happened": a count of exactly N for N events is the fabric losing
// none and inventing none, end to end, with no CPU in the path.
//
// What is exercised, letter by letter:
//   a  the fabric: the user multiplexer's off-by-one, the ordering rule,
//      the path/edge legality both ways, and the erratum-1.12.1 refusal
//   b  AN EVENT IS COUNTED: software event -> channel -> TC4's event
//      input, with the counter itself as the witness, exactly one count
//      per event
//   c  the synchronous and resynchronized paths, which need a channel
//      clock - and the status surface that only they have
//   d  what the asynchronous path does NOT have: CHSTATUS and both
//      interrupt flags read zero - and that SOFTWARE events on that
//      path reach a TC's event input, every one
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc21/clock.hpp"
#include "samc21/evsys.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
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

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The fabric under test
//
// The witness is TC4, event user 27 (TC4 EVU, table 29-3): TC4 has its
// generic clock channel to itself (35.5.3), so arming and releasing it
// touches no other timer. The event channel is 0, whose generic clock is
// EVSYS_GCLK_ID_0.
// ---------------------------------------------------------------------------
using Witness = Tc<4>;
constexpr uint8_t user_witness = Witness::event_user;
constexpr uint8_t ev_ch = 0;
constexpr uint8_t gen_slow = 5;      // a generator this suite builds for the clock

using GenSlow = Gclk<gen_slow>;

// COUNT16 on the normal-frequency waveform: the count action is refused
// beside a PWM waveform (35.6.2.5.3), and nothing here drives a pad.
constexpr TcConfig witness_cfg{.mode = TcMode::count16,
                               .waveform = TcWaveform::normal_frequency};
constexpr TcEventConfig witness_events{.action = TcEventAction::count,
                                       .input_enable = true};
static_assert(tc_event_config_valid(witness_cfg, witness_events),
              "the witness counts events, which a PWM waveform would refuse");

/// Arm the witness from zero: the software reset inside init() clears
/// COUNT, and EVCTRL is written before the enable because it is
/// enable-protected (35.6.2.1). GCLK_TC from generator 0 clocks the
/// counter's own domain; the events are what it counts.
bool witness_arm() {
    return Witness::init(0) && Witness::configure(witness_cfg) &&
           Witness::event_config(witness_cfg, witness_events) &&
           Witness::enable(true);
}

/// How many events the witness has counted - a READSYNC read (35.6.8).
uint16_t witnessed() { return Witness::count16(); }

/// A short wait, long enough for any channel clock this suite uses to
/// tick several times over.
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
                  Evsys::attach(user_witness, 3));
    bench.verdict("and reads back as the same plain number",
                  Evsys::user_channel(user_witness) == 3u);
    const uint32_t raw = EVSYS_REGS->EVSYS_USER[user_witness];
    print(serial, "  USER[", user_witness, "] for channel 3 holds ", raw,
          " - the register wants channel+1 (29.8.9)", crlf);
    bench.verdict("the register itself holds channel + 1", raw == 4u);

    Evsys::disconnect(user_witness);
    bench.verdict("disconnecting reports no channel",
                  Evsys::user_channel(user_witness) == Evsys::channel_count);

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
// b - an event is counted
// =============================================================================
void tb_event_counted() {
    Evsys::reset();
    bench.verdict("the witness arms: TC4 counts events and nothing else",
                  witness_arm());
    settle();
    bench.verdict("and with no event it has counted none - no clock tick "
                  "moves a counter whose action is COUNT",
                  witnessed() == 0u);

    // A software event on an unrouted channel must reach nobody - which
    // is what makes the next verdicts mean something.
    Evsys::trigger(ev_ch);
    settle();
    bench.verdict("a software event on a channel no user listens to is "
                  "counted by nobody",
                  witnessed() == 0u);

    // Now route it. connect() writes the USER multiplexer first and the
    // channel second, which is 29.6.2.3's order.
    //
    // THE PATH IS SYNCHRONOUS so that this letter rests on the clocked
    // delivery letter c measures on both clocked paths; what the
    // asynchronous path does with a SOFTWARE event is letter d's
    // question, asked there and not assumed here.
    bench.verdict("the channel's own generic clock is routed",
                  GenSlow::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_ch), gen_slow));
    bench.verdict("TC4's event user connects to event channel 0",
                  Evsys::connect(user_witness, ev_ch,
                                 EventChannelConfig{
                                     .path = EventPath::synchronous,
                                     .edge = EventEdge::rising}));
    // ERRATUM 1.12.4: a freshly configured channel is busy for one
    // channel-clock tick without CHBUSY showing it, so the first trigger
    // is paced rather than issued immediately.
    settle();

    Evsys::clear_flags(Evsys::detected_flag(ev_ch) | Evsys::overrun_flag(ev_ch));
    Evsys::trigger(ev_ch);
    settle();
    const uint16_t one = witnessed();
    print(serial, "  after one software event: the witness counts ", one,
          ", EVD=", Evsys::detected(ev_ch) ? "1" : "0", crlf);
    bench.verdict("THE EVENT WAS COUNTED, EXACTLY ONCE - software event to "
                  "EVSYS to TC4, with no CPU in the path",
                  one == 1u);
    bench.verdict("and the channel's event-detected flag agrees from the "
                  "other side of the fabric (29.6.2.10)",
                  Evsys::detected(ev_ch));

    // Repeatable and exact: each event spaced by many channel-clock
    // periods, so every one is a separate pulse the user acknowledges.
    constexpr uint8_t more = 8;
    for (uint8_t i = 0; i < more; ++i) {
        Evsys::trigger(ev_ch);
        settle();
    }
    const uint16_t total = witnessed();
    print(serial, "  after ", more, " more spaced events: ", total, crlf);
    bench.verdict("eight more events are eight more counts - none lost and "
                  "none invented",
                  total == static_cast<uint16_t>(1u + more));
    bench.verdict("and none of them overran the channel (29.6.2.9)",
                  !Evsys::overrun(ev_ch));

    Evsys::disconnect(user_witness);
    GclkChannel::disconnect(Evsys::gclk_id(ev_ch));
    Witness::release();
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
        bench.verdict("the witness arms from zero", witness_arm());

        // Both clocked paths need an edge; the software event is a pulse,
        // so a rising edge is what there is to catch.
        const bool routed = Evsys::connect(
            user_witness, ev_ch,
            EventChannelConfig{.path = path, .edge = EventEdge::rising});
        bench.verdict("the ", name, routed);
        settle();   // erratum 1.12.4

        Evsys::clear_flags(Evsys::detected_flag(ev_ch) |
                           Evsys::overrun_flag(ev_ch));
        Evsys::trigger(ev_ch);
        settle();

        const uint16_t counted = witnessed();
        const bool saw_event = Evsys::detected(ev_ch);
        print(serial, "  ", name, ": counted=", counted,
              " EVD=", saw_event ? "1" : "0",
              " OVR=", Evsys::overrun(ev_ch) ? "1" : "0", crlf);

        bench.verdict("a clocked path carries the event to TC4, counted once",
                      counted == 1u);
        // THE STATUS SURFACE THE ASYNCHRONOUS PATH DOES NOT HAVE: EVD is
        // set when an event coming from the channel is detected, and it
        // is only ever set on these two paths (29.6.2.10).
        bench.verdict("and raises the event-detected flag, which only a "
                      "clocked path has",
                      saw_event);

        Evsys::disconnect(user_witness);
        Witness::release();
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
//
// AND ONE QUESTION THE CHAPTER DOES NOT ANSWER: 29.6.2.12 says a
// software event "can be serviced as any event generator" without
// qualifying by path, while the asynchronous path has no clock and no
// edge detector and a register write has no width of its own. Whether
// the event arrives is the USER's input stage's business, so this letter
// asks it of TC4's and counts the answer - spaced single events first,
// then a back-to-back burst, then a control with the user disconnected:
// TC4 takes every one of either kind.
void td_async_is_silent() {
    Evsys::reset();
    bench.verdict("the witness arms", witness_arm());
    bench.verdict("routed asynchronously",
                  Evsys::connect(user_witness, ev_ch,
                                 EventChannelConfig{.path = EventPath::asynchronous}));
    settle();

    Evsys::clear_flags(Evsys::detected_flag(ev_ch) | Evsys::overrun_flag(ev_ch));

    constexpr uint8_t events = 8;
    for (uint8_t i = 0; i < events; ++i) {
        Evsys::trigger(ev_ch);
        settle();
    }
    const uint16_t spaced = witnessed();

    // Several events in quick succession - which on a clocked path would
    // be exactly how an overrun is provoked.
    for (uint8_t i = 0; i < events; ++i) {
        Evsys::trigger(ev_ch);
    }
    settle();
    const uint16_t burst = static_cast<uint16_t>(witnessed() - spaced);

    const uint32_t chstatus = Evsys::channel_status();
    print(serial, "  software events on the asynchronous path: ", spaced, " of ",
          events, " spaced ones counted, ", burst, " of ", events,
          " back-to-back ones", crlf);
    print(serial, "  after them: CHSTATUS=", hex(chstatus),
          " EVD=", Evsys::detected(ev_ch) ? "1" : "0",
          " OVR=", Evsys::overrun(ev_ch) ? "1" : "0", crlf);

    bench.verdict("A SOFTWARE EVENT DOES CROSS AN ASYNCHRONOUS CHANNEL into TC4's "
                  "event input - every spaced one counted",
                  spaced == events);
    bench.verdict("and so does every one of a back-to-back burst",
                  burst == events);
    bench.verdict("the event-detected flag stays ZERO on an asynchronous "
                  "channel (29.6.2.10)",
                  !Evsys::detected(ev_ch));
    bench.verdict("so does the overrun flag, however many events pass "
                  "(29.6.2.9)",
                  !Evsys::overrun(ev_ch));
    bench.verdict("and CHSTATUS reports neither busy nor ready for it "
                  "(29.6.2.11)",
                  !Evsys::busy(ev_ch) && !Evsys::users_ready(ev_ch));

    // The control: the same spaced events with the user disconnected
    // must count nothing, or no count above means anything.
    Evsys::disconnect(user_witness);
    const uint16_t before = witnessed();
    for (uint8_t i = 0; i < events; ++i) {
        Evsys::trigger(ev_ch);
        settle();
    }
    const uint16_t unhooked = static_cast<uint16_t>(witnessed() - before);
    print(serial, "  with the user disconnected: ", unhooked, " counted", crlf);
    bench.verdict("with TC4's user disconnected the same events count zero - "
                  "whatever was counted above came through the channel",
                  unhooked == 0u);

    Witness::release();
}

void banner() {
    print(serial, crlf, "test_samc_evsys - SAMC21J18A EVSYS (ch. 29) with a "
          "TC counting events as its user, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    brio::enable_interrupts();

    bench.letter('a', "the fabric, its off-by-one and its refusals", ta_fabric);
    bench.letter('b', "an event is counted, with no CPU in the path",
                 tb_event_counted);
    bench.letter('c', "the synchronous and resynchronized paths", tc_clocked_paths);
    bench.letter('d', "what the asynchronous path does not have", td_async_is_silent);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
