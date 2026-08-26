// test_avr_meter - the METER-SAMPLER test SUITE: util/meter_sampler.hpp
// (the MeterLatch that bridges a capture interrupt to the loop, and the
// MeterSampler AO that paces PUBLICATION instead of capture) driven by
// the real hardware meters of avrdx/tcb.hpp.
//
// Reference test of those (docs/design/meters.md): keep it passing.
//
// NOTHING TO WIRE. The instrument is the chip, the technique
// test_avr_timer established: a TCA drives a known waveform onto its own
// WO0 pad and the event system carries that pad's level back into a TCB
// running in a capture mode - a closed loop through the silicon with no
// jumper in it. PD0 is TCA0's WO0 on the PORTD route and goes nowhere on
// this desk.
//
// LIKE test_avr_power, THIS SUITE RUNS THE KERNEL: the object under test
// is an active object, so the samples travel through a real queue, a
// real dispatch and a real Kernel pack (Sink, Sampler). Only the LOOP is
// the suite's - run_ticks() pumps TimeEvents and Kernel::step() for a
// measured number of kernel ticks.
//
// PINS IT CLAIMS: PD0 (TCA0 WO0) and the console (USART2 ALT1, PF4/PF5,
// 460800). PE0 - the wire to the other board - is never touched.
//
// Commands: ? | a a known signal, latched and paced | b staleness and
// the overwrite count | c two sources in one sampler | z = a..c

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "avrdx/tca.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "kernel/kernel.hpp"
#include "util/meter_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = AvrPlatform;
using SysClock = Clock<ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

// ---- the closed loop ----------------------------------------------------------
using GenPin = Pin<'D', 0>;              // TCA0 WO0 on the PORTD route
using Gen = FrequencyGenerator<0, 'D'>;  // a rate, 50 % duty
using Pwm = TcaPwm16<0, 'D', 24000>;     // 1 kHz at div1, width = the duty value

using T0 = Tcb<0>;                       // the frequency meter
using T1 = Tcb<1>;                       // the pulse-width meter
using ChGen = EventChannel<2>;           // PD0's level (PORTC/D live on 2-3)

constexpr uint32_t crystal_hz = SysClock::hz;

// ---- the objects under test ------------------------------------------------------
// One latch per meter. They are the SAME width, so what makes them two
// objects is the id - the type is the latch.
using PeriodLatch = MeterLatch<uint16_t, P, 0>;
using WidthLatch = MeterLatch<uint16_t, P, 1>;

/// The subscriber: it counts and keeps what arrives, per source index.
struct Sink : Fsm<Sink, MeterSample> {
    static inline EventQueue<Event, 8, P> queue;
    static inline uint16_t got[2] = {0, 0};
    static inline uint32_t last[2] = {0, 0};
    static inline uint32_t total = 0;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static void clear() {
        got[0] = got[1] = 0;
        last[0] = last[1] = 0;
        total = 0;
    }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](MeterSample s) {
                if (s.index < 2) {
                    ++got[s.index];
                    last[s.index] = s.value;
                }
                ++total;
                return handled();
            });
    }
};

using Sampler = MeterSampler<P, Subscribers<Sink>, PeriodLatch, WidthLatch>;
using K = Kernel<P, Sink, Sampler>;

// ---- what the ISRs hand over -------------------------------------------------------
// The drivers are untouched: the meter's ISR BODY returns the reading and
// re-arms the capture, and the vector binding stores it into a latch.
// That store is the only new thing in the interrupt - which is the point
// of the latch.
volatile uint16_t captures0 = 0;
volatile uint16_t captures1 = 0;

// ---- the test harness ---------------------------------------------------------------
TestBench<Serial> bench;

void verdict(const char* name, bool ok) { bench.verdict(name, ok); }

bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);
}

/// The kernel loop for a measured span of kernel ticks: mature the time
/// events, serve every queue, and count the wall clock in PIT ticks -
/// the very timebase the sampler's pace is expressed in.
void run_ticks(uint32_t ticks) {
    const uint32_t t0 = P::now();
    while (P::now() - t0 < ticks) {
        TimeEvents<P>::process();
        (void)K::step();
    }
}

/// Everything a letter must not inherit: no waveform, no meter, no
/// route, no pace, both latches empty, the subscriber's tally cleared.
void quiesce() {
    Sampler::stop();
    Tca<0>::disable();
    T0::disable();
    T1::disable();
    T0::enable_capt_interrupt(false);
    T1::enable_capt_interrupt(false);
    ChGen::off();
    cli();
    captures0 = 0;
    captures1 = 0;
    sei();
    TimeEvents<P>::clear_all();
    while (Sink::queue.pop().has_value()) {
    }
    while (Sampler::queue.pop().has_value()) {
    }
    K::init_all();
    PeriodLatch::clear();
    WidthLatch::clear();
    Sink::clear();
}

/// A snapshot of the ISR counter and the latch's own counter taken
/// together, so the arithmetic that relates them is not raced.
struct Tally {
    uint16_t captures;
    uint16_t missed;
};
Tally take_tally() {
    cli();
    const Tally t{captures0, PeriodLatch::missed()};
    sei();
    return t;
}

/// Arming a capture input on a channel whose level is already high reads
/// ONE spurious edge (docs/avrdx/tcb.md); the meters enable their own
/// CAPT interrupt inside init(), so that edge reaches the latch. Let it
/// happen, then wipe it: everything this suite measures is about the
/// steady state that follows. Returns what the arming cost.
uint16_t settle_after_arming() {
    delay_us(clock, 1000);
    cli();
    const uint16_t n = captures0;
    captures0 = 0;
    captures1 = 0;
    sei();
    PeriodLatch::clear();
    WidthLatch::clear();
    return n;
}

// ---- a: a known signal, latched and paced --------------------------------------------

void ta_paced() {
    print(serial, "a TCA0/PD0 -> FrequencyMeter TCB0 -> MeterLatch -> MeterSampler", crlf);

    constexpr uint32_t rates[] = {1000, 5000, 20000};
    constexpr uint32_t pace = 128;              // kernel ticks: 125 ms at 1024 Hz
    constexpr uint32_t window = 1024;           // one second of kernel time

    for (uint32_t hz : rates) {
        quiesce();
        ChGen::source(EvPin<GenPin>{});
        FrequencyMeter<T0>::init(clock, ChGen{});
        verdict("generator accepts the rate", Gen::init(clock, hz));

        Sampler::init(pace);
        console_drain();
        run_ticks(window);
        Sampler::stop();

        const Tally t = take_tally();
        const uint16_t published = Sink::got[0];
        const uint32_t value = Sink::last[0];
        const uint32_t expect = crystal_hz / hz;
        const uint32_t expect_pubs = window / pace;

        print(serial, "  ", hz, " Hz (actual ", Gen::actual_hz(), "): last sample ", value,
              " ticks, expect ", expect, " = ", FrequencyMeter<T0>::hz(
                  static_cast<uint16_t>(value)), " Hz", crlf);
        print(serial, "    over ", window, " kernel ticks: ", t.captures,
              " captures in the ISR, ", published, " samples published (pace ", pace,
              " ticks -> expect ", expect_pubs, ")", crlf);

        verdict("the sample matches the generator within a tick",
                near(static_cast<int32_t>(value), static_cast<int32_t>(expect), 1));
        verdict("the sampler published at ITS rate, not the capture rate",
                near(published, static_cast<int32_t>(expect_pubs), 1));
        verdict("and the wire really did run far faster than that",
                t.captures > 10u * published);
        verdict("every publication carried source index 0", Sink::got[1] == 0 &&
                Sink::total == published);
    }

    // The pace is a knob, not a property of the signal: the same wire at
    // twice the pace publishes twice as much.
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    FrequencyMeter<T0>::init(clock, ChGen{});
    Gen::init(clock, 1000);
    Sampler::init(64);
    console_drain();
    run_ticks(window);
    Sampler::stop();
    const uint16_t fast_pubs = Sink::got[0];
    print(serial, "  same 1 kHz signal at pace 64: ", fast_pubs, " samples (expect ",
          window / 64u, ")", crlf);
    verdict("halving the period doubles the publications",
            near(fast_pubs, static_cast<int32_t>(window / 64u), 1));

    quiesce();
}

// ---- b: staleness and the overwrite count -----------------------------------------------

void tb_stale() {
    print(serial, "b silence when nothing is measured; missed() when the wire outruns the pace",
          crlf);
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    FrequencyMeter<T0>::init(clock, ChGen{});

    // No generator at all: the meter is armed and watching a pad that
    // never moves. The arming edge itself is the one thing that does
    // arrive, and it is wiped before the measurement starts.
    const uint16_t arming = settle_after_arming();
    print(serial, "  arming the capture input cost ", arming,
          " spurious capture(s) (tcb.md: one, on a channel already high)", crlf);
    verdict("the arming edge is the documented one at most", arming <= 1);

    Sampler::init(64);
    console_drain();
    run_ticks(512);
    Sampler::stop();
    Tally t = take_tally();
    print(serial, "  generator stopped, 512 ticks at pace 64: ", t.captures, " captures, ",
          Sink::total, " samples published, missed=", t.missed, crlf);
    verdict("a stale source publishes NOTHING", Sink::total == 0 && Sink::got[0] == 0);
    verdict("no captures arrived to be missed", t.captures == 0 && t.missed == 0);
    verdict("and the latch is honestly empty", !PeriodLatch::fresh());

    // One single edge burst: exactly one sample, then silence again.
    Sink::clear();
    Gen::init(clock, 1000);
    delay_us(clock, 3000);            // a few periods on the wire
    Tca<0>::disable();
    Sampler::init(32);
    console_drain();
    run_ticks(256);
    Sampler::stop();
    print(serial, "  one burst then quiet, 8 sampler ticks: ", Sink::total,
          " sample published, value ", Sink::last[0], crlf);
    verdict("the one value waiting is published once and once only",
            Sink::total == 1 && near(static_cast<int32_t>(Sink::last[0]), 24000, 2));

    // Now the wire outruns the pace by three orders of magnitude.
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    FrequencyMeter<T0>::init(clock, ChGen{});
    Gen::init(clock, 20000);
    Sampler::init(128);
    console_drain();
    run_ticks(1024);
    Sampler::stop();
    t = take_tally();
    const uint16_t published = Sink::got[0];
    print(serial, "  20 kHz against a 128-tick pace: ", t.captures, " captures, ", published,
          " published, missed=", t.missed, crlf);
    verdict("the overwrites are counted, not hidden", t.missed > 100);
    // Every capture either found the cell empty (and was published, or
    // is still waiting at the moment of the reading) or overwrote a
    // value that had not been taken. There is no third case.
    const uint32_t accounted = static_cast<uint32_t>(t.missed) + published;
    print(serial, "    missed + published = ", accounted, " vs ", t.captures,
          " captures (the difference is the value still in the cell)", crlf);
    verdict("missed + published accounts for every capture",
            accounted == t.captures || accounted + 1 == t.captures);
    verdict("and the pace held all the same",
            near(published, static_cast<int32_t>(1024u / 128u), 1));

    quiesce();
}

// ---- c: two sources in one sampler ------------------------------------------------------

void tc_two_sources() {
    print(serial, "c TcaPwm16 PD0 -> FrequencyMeter TCB0 + PulseWidthMeter TCB1, one sampler",
          crlf);

    constexpr uint16_t duties[] = {6000, 12000, 18000};
    for (uint16_t duty : duties) {
        quiesce();
        ChGen::source(EvPin<GenPin>{});
        FrequencyMeter<T0>::init(clock, ChGen{});
        PulseWidthMeter<T1>::init(clock, ChGen{});
        Pwm::init(TcaClock::div1, 0x01);
        Pwm::duty<0>(duty);
        delay_us(clock, 3000);                  // buffered: lands at BOTTOM

        Sampler::init(128);
        console_drain();
        run_ticks(512);
        Sampler::stop();

        cli();
        const uint16_t c0 = captures0;
        const uint16_t c1 = captures1;
        sei();
        const uint32_t period = Sink::last[0];
        const uint32_t width = Sink::last[1];

        print(serial, "  duty ", duty, "/24000: index 0 = ", period, " ticks (",
              Sink::got[0], " samples, ", c0, " captures), index 1 = ", width, " ticks (",
              Sink::got[1], " samples, ", c1, " captures)", crlf);

        verdict("index 0 is the PERIOD, within a tick",
                near(static_cast<int32_t>(period), 24000, 1));
        verdict("index 1 is the WIDTH, within a tick",
                near(static_cast<int32_t>(width), static_cast<int32_t>(duty), 1));
        verdict("both sources published, at the sampler's one pace",
                near(Sink::got[0], 4, 1) && near(Sink::got[1], 4, 1));
        verdict("the labels are the pack order, and nothing else spoke",
                Sink::total == static_cast<uint32_t>(Sink::got[0]) + Sink::got[1]);
    }

    // One source stops, the other does not: the silence is per source.
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    FrequencyMeter<T0>::init(clock, ChGen{});
    PulseWidthMeter<T1>::init(clock, ChGen{});
    // AFTER the meter's init, which arms the CAPT interrupt itself: this
    // is what makes source 1 a source nobody feeds.
    T1::enable_capt_interrupt(false);
    Pwm::init(TcaClock::div1, 0x01);
    Pwm::duty<0>(9000);
    delay_us(clock, 3000);
    settle_after_arming();
    Sampler::init(128);
    console_drain();
    run_ticks(512);
    Sampler::stop();
    print(serial, "  with TCB1's interrupt disabled: index 0 got ", Sink::got[0],
          " samples, index 1 got ", Sink::got[1], crlf);
    verdict("a source nobody feeds stays silent while its neighbour speaks",
            Sink::got[0] > 2 && Sink::got[1] == 0);
    verdict("and the sampler counted exactly what it published",
            Sampler::published() == Sink::total);

    quiesce();
}

// ---- menu --------------------------------------------------------------------------------

void register_tests() {
    bench.letter('a', "a known signal, latched and paced", ta_paced);
    bench.letter('b', "staleness and the overwrite count", tb_stale);
    bench.letter('c', "two sources in one sampler", tc_two_sources);
}

void help() {
    print(serial, "test_avr_meter:", crlf);
    bench.menu();
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

ISR(RTC_PIT_vect) { brio::Ticker::pit(); }

// The ISR glue this suite exists to exercise: the driver's meter body
// returns the reading, the binding drops it into a latch, and that is
// the whole of the interrupt's dealings with the loop.
ISR(TCB0_INT_vect) {
    PeriodLatch::store(brio::FrequencyMeter<T0>::period_ticks());
    ++captures0;
}
ISR(TCB1_INT_vect) {
    WidthLatch::store(brio::PulseWidthMeter<T1>::width_ticks());
    ++captures1;
}
// The other two vectors are bound as a net: an unbound vector on this
// core is a jump to 0, i.e. a silent reset loop.
ISR(TCB2_INT_vect) { (void)brio::Tcb<2>::take_flags(); }
ISR(TCB3_INT_vect) { (void)brio::Tcb<3>::take_flags(); }

int main() {
    const ResetFlags why = Reset::take_flags();

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    K::init_all();
    sei();

    auto board = board_id();
    if (board.empty()) {
        board = "?";
    }
    print(serial, crlf, "test_avr_meter - meter-sampler test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ", hex(SYSCFG.REVID),
          ", RSTFR=", hex(why.raw), ")", crlf);
    print(serial, "  timebase ", Ticker::ticks_per_second,
          " Hz; the loop is TCA0 WO0 on PD0 read back through EVSYS - no wires", crlf);
    register_tests();
    help();

    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            help();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "? for help", crlf);
        }
        bench.prompt();
    }
}
