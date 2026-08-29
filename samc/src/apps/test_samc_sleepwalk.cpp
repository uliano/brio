// test_samc_sleepwalk - the TRANSVERSAL SLEEP test SUITE for the SAM
// C21: not what the Power Manager does (that is test_samc_sleep and
// docs/samc/platform.md), but what every OTHER peripheral does WHILE
// THE CPU IS STOPPED - wake sources, SleepWalking, and the RUNSTDBY /
// ONDEMAND pairs whose tables every chapter prints and no chapter
// measures.
//
// NOTHING TO WIRE. Every instrument is on the die.
//
// THREE THINGS MAKE THIS SUITE POSSIBLE, and they are worth reading
// before any letter below.
//
// 1. THE RULER. Kernel time STOPS in standby (SysTick rides the CPU
//    clock), so nothing timed here may use Ticker::millis(). The
//    stopwatch is a TC0+TC1 32-bit pair on the board's 24 MHz crystal
//    with RUNSTDBY set: XOSC keeps running through a standby whatever
//    RUNSTDBY says (measured in test_samc_sleep), so this counter is
//    the one clock in the building that means the same thing on both
//    sides of a WFI. 42 ns a tick.
//
// 2. THE STIMULUS. A pad cannot be moved by the CPU while the CPU is
//    asleep, and every wireless pin trick this stratum owns (the EIC
//    campaign's pull-walking) is a CPU store. So the pad is walked by
//    HARDWARE, through a chain that is itself a finding:
//
//      TC2 (OSCULP32K, RUNSTDBY)  a square wave that survives standby
//        -> CCL LUT2, combinational   its OUTPUT VALUE is an event
//        -> EVSYS, asynchronous path  no channel clock needed
//        -> PORT event input 0, action OUT, on PA16
//        -> PA16's OUT bit, which under PMUXEN is its PULL's DIRECTION
//        -> the pad walks between the rails, all on its own.
//
//    28.6.4 is what makes it legal: "In Standby mode, only the Out
//    action is possible" - the OUT action is combinational ("sent to
//    the pin without any internal latency") where SET/CLR/TGL cost
//    three clocks the PORT does not have in standby. samc/pin.hpp grew
//    the PORT event-input surface for this, which is also the gap
//    docs/samc/port.md declared.
//
// 3. THE WITNESS. The CPU cannot observe anything during a standby, so
//    every "did it happen while asleep" question is answered by a
//    COUNTER: TC4, RUNSTDBY, on OSCULP32K, either counting an EVSYS
//    event or counting its own generic clock. Read before the sleep and
//    after the wake, its advance is the answer; the same window spent
//    AWAKE is the control that says what the advance should be.
//
// THE ANTI-WEDGE RULE, inherited from test_samc_sleep: every sleeping
// letter arms the WATCHDOG first (it runs on OSCULP32K and survives
// standby) and disarms it at the end, so a wake that never arrives
// costs a reboot and a banner instead of a mute board. Nothing prints
// between arming a sleep and coming back from it - the console is dead
// in standby, and a print is milliseconds even when it is not.
//
// What is exercised, letter by letter:
//   a  THE INSTRUMENT LETTER, and the EIC in standby: the hardware pad
//      walker built and proved, an EXTINT wake, ERRATUM 1.11.6 judged
//      by counting edges across one standby, and which clock a SAMPLED
//      line needs to detect while the CPU is stopped
//   b  the RTC as the wake source: compare, periodic and the calendar
//      alarm, each timed on the crystal
//   c  FREQM: a measurement started awake, finished asleep, and its
//      DONE interrupt as the wake
//   d  the oscillators and the DPLL through a standby - who keeps
//      running for whom, and whether the loop stops
//   e  the ADC: table 38-4's four rows, and a REAL SleepWalking
//      conversion paced by the RTC with no CPU in the loop
//   f  the SDADC and the TSENS, the same shape on their own tables
//   g  the DAC's output buffer through a standby, and erratum 1.9.2
//   h  the AC's two sequences of 40.6.14: continuous in sleep, and the
//      single-shot SleepWalking chain
//   i  CCL 37.6.4 judged: which LUTs keep decoding in standby
//   j  the leftovers: TCC, EVSYS channels, the BODVDD
//   p  (outside z) PM.bus_clock, whose "off" 19.5.2 calls one-way
//
// NOTE for anyone adding a letter: a printed line must NEVER contain
// the two characters "->", because tools/bench.py looks for that arrow
// to find a letter's tally line and truncates the capture on a stray
// one.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/adc.hpp"
#include "samc/ccl.hpp"
#include "samc/clock.hpp"
#include "samc/dac.hpp"
#include "samc/eic.hpp"
#include "samc/evsys.hpp"
#include "samc/freqm.hpp"
#include "samc/nvic.hpp"
#include "samc/osc32kctrl.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"
#include "samc/rtc.hpp"
#include "samc/sdadc.hpp"
#include "samc/sercom.hpp"
#include "samc/sleep.hpp"
#include "samc/supc.hpp"
#include "samc/tc.hpp"
#include "samc/tcc.hpp"
#include "samc/ticker.hpp"
#include "samc/tsens.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = SamPlatform;

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
// The generic clock generators, and what each is for
// ---------------------------------------------------------------------------

constexpr uint8_t gen_xtal = 2;    ///< XOSC 24 MHz, RUNSTDBY: the stopwatch
constexpr uint8_t gen_ulp = 3;     ///< OSCULP32K, RUNSTDBY: the standby-alive clock
constexpr uint8_t gen_eic = 4;     ///< letter a's own, so its RUNSTDBY can move
constexpr uint8_t gen_probe = 5;   ///< whatever letter d or j is probing

constexpr uint32_t crystal_hz = 24'000'000UL;

/// OSC48M's own measured rate (docs/samc/clock.md): 47.755 MHz, 5100
/// ppm below nominal and crystal-referenced. Nothing below is timed on
/// it, but letter d weighs it.
constexpr uint32_t osc48m_hz = 47'755'000UL;

/// The RTC's source rate, measured against the crystal at boot.
uint32_t ulp_hz = 32'900UL;

// ---------------------------------------------------------------------------
// The event fabric
// ---------------------------------------------------------------------------

constexpr uint8_t ev_stim = 0;    ///< the LUT's output level, to the PORT
constexpr uint8_t ev_count = 1;   ///< whatever the witness counter counts
constexpr uint8_t ev_start = 2;   ///< the RTC's periodic pulse, to an analog block

// ---------------------------------------------------------------------------
// The instruments
// ---------------------------------------------------------------------------

/// The stopwatch: TC0 + TC1 as one 32-bit counter on the crystal, with
/// RUNSTDBY, so it counts straight through a standby.
using Watch = Tc<0>;

/// The stimulus source: TC2, a square wave on OSCULP32K that survives a
/// standby. TC2 is not a free choice - Lut<2>'s INSEL "TC" source is
/// TC (2 % 5) = TC2 (37.6.2.4), which is what lets the wave reach the
/// CCL with no pad and no wire.
using Toggler = Tc<2>;

/// The witness: TC4, alone on its own generic clock channel (TC0/TC1
/// share channel 30 and TC2/TC3 share 31, so a fifth instance is the
/// only one that can be re-clocked without stopping a neighbour).
using Counter = Tc<4>;

/// The LUT the stimulus passes through, and the pad it walks.
using Stim = Lut<2>;
using StimPad = Pin<'A', 16>;
using StimLine = ExtInt<StimPad>;   // EXTINT0

// ---------------------------------------------------------------------------
// Shared with the handlers
// ---------------------------------------------------------------------------

volatile bool wake_flag = false;     ///< any wake source sets it
volatile bool rtc_fired = false;
volatile uint32_t rtc_irqs = 0;
volatile uint32_t eic_irqs = 0;
volatile uint32_t freqm_irqs = 0;
volatile uint32_t adc_irqs = 0;
volatile uint32_t sdadc_irqs = 0;
volatile uint32_t tsens_irqs = 0;
volatile uint32_t ac_irqs = 0;
volatile uint32_t supc_irqs = 0;

/// A hard cap on every "wait for the wake" spin: a wake that never
/// arrives must fail a verdict, not hang the bench. (The watchdog is
/// the backstop for the case where the CPU never comes back at all.)
constexpr uint32_t spin_cap = 40'000'000UL;

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

/// A short busy wait, a handful of CPU cycles per turn at 48 MHz.
void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

// ---------------------------------------------------------------------------
// The watchdog backstop
// ---------------------------------------------------------------------------

/// 4096 cycles of the WDT's ~1 kHz clock is about four seconds - longer
/// than any leg here and shorter than bench.py's timeout, so a lost
/// wake reboots the board into its banner instead of leaving it mute.
void watchdog_backstop(bool on) {
    if (on) {
        (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc4096});
    } else {
        (void)Watchdog::disable();
    }
}

// ---------------------------------------------------------------------------
// The stopwatch
// ---------------------------------------------------------------------------

uint32_t watch_now() { return Watch::count32(); }

uint32_t watch_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) /
                                 crystal_hz);
}

bool watch_up() {
    if (!Xosc::init(XoscConfig{.hz = crystal_hz,
                               .startup = 4,
                               .on_demand = false,
                               .run_standby = true})) {
        return false;
    }
    if (!Gclk<gen_xtal>::configure(
            GclkConfig{.source = GclkSource::xosc, .run_standby = true})) {
        return false;
    }
    (void)Watch::enable(false);
    if (!Watch::init(gen_xtal)) {
        return false;
    }
    if (!Watch::configure(TcConfig{.mode = TcMode::count32,
                                   .prescaler = TcPrescaler::div1,
                                   .run_standby = true})) {
        return false;
    }
    return Watch::enable(true);
}

// ---------------------------------------------------------------------------
// The RTC: the wake source and the coarse clock
// ---------------------------------------------------------------------------

uint32_t rtc_now() { return Rtc::count32(); }

uint32_t rtc_ms(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000ULL) / ulp_hz);
}
uint32_t rtc_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) / ulp_hz);
}

/// The RTC in mode 0, counting OSCULP32K tick for tick, with COMP0 as
/// the interrupt. PRESCALER is div1 and not off, deliberately: OFF
/// divides by one AND silences every periodic event (24.6.8.1), and
/// letters e..h are paced by those.
bool rtc_up(const RtcEventConfig& events = {}) {
    if (!Rtc::init()) {
        return false;
    }
    (void)Rtc::enable(false);
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    if (!Rtc::init()) {
        return false;
    }
    const RtcConfig cfg{.mode = RtcMode::count32, .prescaler = RtcPrescaler::div1};
    if (!Rtc::configure(cfg)) {
        return false;
    }
    if (!Rtc::event_config(cfg, events)) {
        return false;
    }
    if (!Rtc::enable(true)) {
        return false;
    }
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::compare0);
    Nvic::enable(Rtc::irq());
    return true;
}

/// Turn the RTC's periodic output `n` into an EVSYS generator (EVCTRL
/// is enable-protected, so the block goes down and up around it). `n`
/// past the last interval turns them all off again.
bool rtc_periodic_event(uint8_t n) {
    if (!Rtc::enable(false)) {
        return false;
    }
    const RtcConfig cfg{.mode = RtcMode::count32, .prescaler = RtcPrescaler::div1};
    const RtcEventConfig ev{
        .periodic_out = n < 8u ? static_cast<uint8_t>(1u << n) : uint8_t{0}};
    if (!Rtc::event_config(cfg, ev)) {
        return false;
    }
    return Rtc::enable(true);
}

/// Arm COMP0 `p` ticks ahead of now, and report whether the compare is
/// still in the future when the arming is done - i.e. whether it is
/// safe to sleep on it.
bool arm_wake(uint32_t p) {
    const uint32_t c = rtc_now();
    rtc_fired = false;
    wake_flag = false;
    Rtc::clear_flags(RtcFlag::compare0);
    if (!Rtc::set_comp32(c + p)) {
        return false;
    }
    return static_cast<int32_t>(rtc_now() - (c + p)) < 0;
}

// ---------------------------------------------------------------------------
// Sleeping
// ---------------------------------------------------------------------------

/// One standby, race-free: PRIMASK is set, the wake condition re-tested,
/// WFI still wakes on a pending interrupt, and unmasking afterwards is
/// what lets the handler run. Pm::sleep() holds erratum 1.8.13's
/// SysTick guard for us.
void sleep_standby_once() {
    __disable_irq();
    if (!wake_flag) {
        Pm::sleep();
    }
    __enable_irq();
}

/// Arm STANDBY once, for a letter that then sleeps repeatedly.
bool arm_standby() { return Pm::set_sleep_mode(SleepMode::standby); }
void disarm_standby() { (void)Pm::set_sleep_mode(SleepMode::idle0); }

/// Stay in STANDBY until the RTC compare `ticks` ahead fires, going
/// back to sleep after any other wake. This is the "spend a known
/// window asleep" primitive every witness measurement is built on.
bool standby_for(uint32_t ticks) {
    if (!arm_wake(ticks) || !arm_standby()) {
        return false;
    }
    uint32_t spins = spin_cap;
    while (!rtc_fired && spins-- != 0u) {
        sleep_standby_once();
    }
    disarm_standby();
    return rtc_fired;
}

/// The same window spent AWAKE, so a witness has a control.
bool poll_for(uint32_t ticks) {
    if (!arm_wake(ticks)) {
        return false;
    }
    uint32_t spins = spin_cap;
    while (!rtc_fired && spins-- != 0u) {
    }
    return rtc_fired;
}

/// Sleep in STANDBY until ANY armed interrupt wakes the CPU, with NO
/// backstop of its own - for the one letter whose subject is the RTC's
/// own calendar, where the compare register IS the alarm register and a
/// backstop would overwrite the thing under test. The watchdog is the
/// only net under this one.
uint32_t standby_bare() {
    if (!arm_standby()) {
        return 0;
    }
    const uint32_t t0 = watch_now();
    uint32_t spins = spin_cap;
    while (!wake_flag && spins-- != 0u) {
        sleep_standby_once();
    }
    const uint32_t t1 = watch_now();
    disarm_standby();
    return wake_flag ? (t1 - t0) : 0u;
}

/// Sleep in STANDBY until ANY armed interrupt wakes the CPU, with the
/// RTC compare `backstop` ticks ahead as the thing that ends the sleep
/// if the source under test never fires. Returns the crystal ticks the
/// sleep took; the caller reads `rtc_fired` to learn whether it was the
/// backstop that ended it, and its own counter to learn whether it was
/// not.
uint32_t standby_until_wake(uint32_t backstop) {
    if (!arm_wake(backstop) || !arm_standby()) {
        return 0;
    }
    const uint32_t t0 = watch_now();
    uint32_t spins = spin_cap;
    while (!wake_flag && spins-- != 0u) {
        sleep_standby_once();
    }
    const uint32_t t1 = watch_now();
    disarm_standby();
    return wake_flag ? (t1 - t0) : 0u;
}

// ---------------------------------------------------------------------------
// The witness counter (TC4)
// ---------------------------------------------------------------------------

/// TC4 counting EVENTS out of generator code `gen_code`, carried on
/// channel `ev_count` over the asynchronous path. The TC's prescaler is
/// bypassed in this mode (35.6.2.5.3) and its generic clock is what
/// resynchronizes the incoming event - hence a generator that survives
/// standby and RUNSTDBY on the counter itself.
bool counter_on_events(uint8_t gen_code, uint8_t gclk_gen, bool run_standby) {
    (void)Counter::enable(false);
    Counter::release();
    if (!Counter::init(gclk_gen)) {
        return false;
    }
    const TcConfig cfg{.mode = TcMode::count16,
                       .prescaler = TcPrescaler::div1,
                       .run_standby = run_standby};
    const TcEventConfig ev{.action = TcEventAction::count, .input_enable = true};
    if (!Counter::configure(cfg) || !Counter::event_config(cfg, ev)) {
        return false;
    }
    // connect() and not configure()+attach(): 29.6.2.3 wants the user
    // multiplexer written BEFORE the channel, and that ordering is the
    // whole reason the driver offers one verb taking both.
    // CHANNELn.RUNSTDBY travels with the counter's own: 29.6.4 says a
    // channel needs it "to be able to run in Standby mode", and table
    // 29-1's only ASYNC row - ONDEMAND 0, RUNSTDBY 0 - reads "Disabled
    // in Standby Sleep mode". Measured, that sentence is about the
    // ASYNCHRONOUS path too, which the table's shape hides (letter j).
    if (!Evsys::connect(Counter::event_user, ev_count,
                        EventChannelConfig{.generator = gen_code,
                                           .path = EventPath::asynchronous,
                                           .run_standby = run_standby})) {
        return false;
    }
    return Counter::enable(true);
}

/// Re-point the witness at another generator without rebuilding it.
bool counter_watch(uint8_t gen_code, bool run_standby = true) {
    return Evsys::connect(Counter::event_user, ev_count,
                          EventChannelConfig{.generator = gen_code,
                                             .path = EventPath::asynchronous,
                                             .run_standby = run_standby});
}

/// TC4 counting its OWN generic clock - the witness that says whether a
/// clock survived a standby at all.
bool counter_on_clock(uint8_t generator, bool run_standby,
                      TcPrescaler prescaler = TcPrescaler::div1024) {
    (void)Counter::enable(false);
    Counter::release();
    if (!Counter::init(generator)) {
        return false;
    }
    const TcConfig cfg{
        .mode = TcMode::count16, .prescaler = prescaler, .run_standby = run_standby};
    if (!Counter::configure(cfg)) {
        return false;
    }
    return Counter::enable(true);
}

void counter_down() {
    (void)Counter::enable(false);
    Evsys::disconnect(Counter::event_user);
    Counter::release();
}

uint16_t counted() { return Counter::count16(); }

/// The counter's advance over one call to `f`, 16-bit wrap included.
uint16_t advance(uint16_t before) { return static_cast<uint16_t>(counted() - before); }

// ---------------------------------------------------------------------------
// The hardware pad walker (see the file header)
// ---------------------------------------------------------------------------

/// TC2 as a square wave of `period` ticks of its generic clock on
/// WO[0]: the 8-bit NPWM shape docs/samc/ccl.md already proved reaches
/// a LUT's INSEL = TC input with no pad in the path. RUNSTDBY is set
/// and its generator is the OSCULP32K one, so the wave survives a
/// standby - which is the whole point of it.
bool toggler_up(uint8_t period, TcPrescaler prescaler = TcPrescaler::div1) {
    (void)Toggler::enable(false);
    (void)Toggler::reset();
    if (!Toggler::init(gen_ulp)) {
        return false;
    }
    if (!Toggler::configure(TcConfig{.mode = TcMode::count8,
                                     .prescaler = prescaler,
                                     .waveform = TcWaveform::normal_pwm,
                                     .run_standby = true})) {
        return false;
    }
    if (!Toggler::set_period8(static_cast<uint8_t>(period - 1u)) ||
        !Toggler::set_cc8(0, static_cast<uint8_t>(period / 2u))) {
        return false;
    }
    return Toggler::enable(true);
}

/// Stop TC2 without touching its generic clock channel: TC2 and TC3
/// SHARE channel 31, so release() would stop the sibling (tc.md).
void toggler_down() {
    (void)Toggler::enable(false);
    (void)Toggler::reset();
}

/// LUT2 passing TC2's WO[0] through combinationally, its OUTPUT VALUE
/// published as an event. Combinational means no GCLK_CCL is needed at
/// all (37.5.3, measured in docs/samc/ccl.md), which is why the walker
/// works in standby with the CCL's own RUNSTDBY clear.
bool stim_lut_up(LutFilter filter = LutFilter::none) {
    Ccl::enable(false);
    if (!Stim::configure(LutConfig{.in0 = LutInput::tc,
                                   .truth = lut_truth_pass(0),
                                   .filter = filter,
                                   .event_out = true},
                         true)) {
        return false;
    }
    Ccl::enable(true);
    return true;
}

/// Change only the arrangement: the action the PORT takes, and whether
/// the pad is the EIC's (PMUXEN, where OUT is the PULL's direction) or
/// PORT's own driven output.
bool walker_arrange(PortEventAction action, bool under_mux) {
    if (!Port<'A'>::configure_event(0, PortEventConfig{.pin = StimPad::pin_number,
                                                       .action = action,
                                                       .enable = true})) {
        return false;
    }
    if (under_mux) {
        // DIR cleared FIRST: an input with a pull is the arrangement the
        // EIC campaign established, and leaving DIR set would confuse
        // "the event reached the pull" with "the event reached a driver
        // that was still connected".
        StimPad::input(PinPull::up);
        StimLine::claim(PinPull::up);   // PULLEN on; OUT is the pull's direction
    } else {
        StimPad::release();
        StimPad::output();
    }
    return true;
}

/// The whole chain from TC2's wave to PA16's pad: the LUT's output
/// value on an asynchronous channel and PORT event input 0 acting on
/// pin 16. What the pad is under - PORT's own driver or the EIC's
/// peripheral function - and which action the event takes are the
/// arrangement letter a measures, so both are arguments.
bool walker_up(uint8_t period, PortEventAction action, bool under_mux) {
    if (!toggler_up(period)) {
        return false;
    }
    if (!Ccl::init(gen_ulp)) {
        return false;
    }
    if (!stim_lut_up()) {
        return false;
    }
    if (!Evsys::connect(Port<'A'>::event_user(0), ev_stim,
                        EventChannelConfig{.generator = Stim::event_generator,
                                           .path = EventPath::asynchronous,
                                           .run_standby = true})) {
        return false;
    }
    return walker_arrange(action, under_mux);
}

void walker_down() {
    Port<'A'>::release_events();
    Evsys::disconnect(Port<'A'>::event_user(0));
    Evsys::release_channel(ev_stim);
    Ccl::enable(false);
    Stim::enable(false);
    Ccl::release();
    toggler_down();
    StimLine::release();
    StimPad::configure({});
}

/// Does the pad actually move? Sample it for `turns` and report whether
/// both levels were seen. The precondition of every letter that uses
/// the walker, and never assumed.
bool pad_walks(uint32_t turns) {
    bool saw_low = false;
    bool saw_high = false;
    for (uint32_t i = 0; i < turns; ++i) {
        if (StimPad::read()) {
            saw_high = true;
        } else {
            saw_low = true;
        }
    }
    return saw_low && saw_high;
}

// ---------------------------------------------------------------------------
// State every letter starts from
// ---------------------------------------------------------------------------

void quiesce() {
    __disable_irq();
    wake_flag = false;
    rtc_fired = false;
    eic_irqs = 0;
    freqm_irqs = 0;
    adc_irqs = 0;
    sdadc_irqs = 0;
    tsens_irqs = 0;
    ac_irqs = 0;
    supc_irqs = 0;
    __enable_irq();
    disarm_standby();
    watchdog_backstop(false);
}

} // namespace

// ===========================================================================
// a: the pad walker, and the EIC in standby
// ===========================================================================

namespace {

/// Count EXTINT0 detections over one window, asleep or awake. The line
/// itself is left configured by the caller; what this does is spend the
/// window and read the witness.
uint16_t eic_edges(bool asleep, uint32_t ticks) {
    const uint16_t before = counted();
    const bool ok = asleep ? standby_for(ticks) : poll_for(ticks);
    if (!ok) {
        return 0xFFFFu;
    }
    return advance(before);
}

/// Reconfigure EXTINT0: sense, async or sampled, its clock, and the
/// event output that feeds the counter. Every EIC configuration
/// register is enable-protected, so the block goes down and up again.
bool line_mode(EicSense sense, bool asynchronous, bool filter, EicClock ck,
               uint8_t generator) {
    (void)Eic::enable(false);
    if (!Eic::clock_select(ck)) {
        return false;
    }
    if (ck == EicClock::gclk) {
        if (!Eic::clock(generator)) {
            return false;
        }
    } else {
        GclkChannel::disconnect(Eic::gclk_id);
    }
    if (!Eic::configure_line(StimLine::line,
                             EicLineConfig{.sense = sense,
                                           .filter = filter,
                                           .asynchronous = asynchronous,
                                           .event_out = true})) {
        return false;
    }
    return Eic::enable(true);
}

/// One arrangement, measured three ways: does the pad move at all, how
/// many detections the EIC makes of it in one window awake, and how
/// many in the same window spent in STANDBY.
struct Leg {
    bool moves = false;
    uint16_t awake = 0;
    uint16_t asleep = 0;
};

Leg measure_leg(PortEventAction action, bool under_mux, uint32_t window) {
    Leg l;
    if (!walker_arrange(action, under_mux)) {
        return l;
    }
    l.moves = pad_walks(200'000);
    Eic::clear_flags(StimLine::mask);
    l.awake = eic_edges(false, window);
    Eic::clear_flags(StimLine::mask);
    l.asleep = eic_edges(true, window);
    return l;
}

void ta_eic() {
    quiesce();

    // The wave: 32 ticks of OSCULP32K, about 970 us, so a standby of a
    // few thousand ticks carries a hundred edges and no print ever has
    // to sit inside one.
    constexpr uint8_t wave = 32;
    constexpr uint32_t window = 3200;               // ~97 ms
    constexpr uint32_t offered = window / wave;     // 100 wave periods

    bool ok = walker_up(wave, PortEventAction::out, false);
    ok = ok && Eic::init();
    ok = ok && counter_on_events(StimLine::event_generator, gen_ulp, true);
    ok = ok && line_mode(EicSense::rising, true, false, EicClock::ulp32k, gen_eic);
    bench.verdict("the pad walker, the EIC and the edge-counting witness come up",
                  ok);
    if (!ok) {
        counter_down();
        (void)Eic::enable(false);
        Eic::release();
        walker_down();
        return;
    }

    // -- the three arrangements ------------------------------------------
    //
    // 28.6.4 gives the PORT four event actions and 28.6.5 separates
    // them: for EVACT = OUT "the output pin follows the event input
    // signal, INDEPENDENTLY OF THE OUT REGISTER VALUE", while SET, CLR
    // and TGL are described as acting on "the output register". Under
    // PMUXEN those are the two different halves of a pad - the OUTPUT
    // DRIVER the peripheral mux has taken away, and the OUT BIT that is
    // still the internal pull's direction (28.6.3.2). And 28.6.4's own
    // note says only OUT survives a standby. All of that is measured
    // here rather than believed.
    watchdog_backstop(true);
    const Leg out_port = measure_leg(PortEventAction::out, false, window);
    const Leg out_mux = measure_leg(PortEventAction::out, true, window);
    const Leg tgl_mux = measure_leg(PortEventAction::toggle, true, window);
    watchdog_backstop(false);

    print(serial, "  ", offered, " wave periods per window. EXTINT detections:",
          crlf);
    print(serial, "    EVACT=OUT, pad PORT-driven   pad ",
          out_port.moves ? "moves" : "still", "   awake ", out_port.awake,
          "  asleep ", out_port.asleep, crlf);
    print(serial, "    EVACT=OUT, pad EIC-owned     pad ",
          out_mux.moves ? "moves" : "still", "   awake ", out_mux.awake,
          "  asleep ", out_mux.asleep, crlf);
    print(serial, "    EVACT=TGL, pad EIC-owned     pad ",
          tgl_mux.moves ? "moves" : "still", "   awake ", tgl_mux.awake,
          "  asleep ", tgl_mux.asleep, crlf);

    bench.verdict("EVACT=OUT moves a PORT-owned pad with no CPU store and no "
                  "wire",
                  out_port.moves);
    bench.verdict("and the EXTINT input does not see that pad: it is a "
                  "peripheral FUNCTION and not a tap on the pin",
                  out_port.awake == 0u);

    // TGL flips the OUT bit once per RISING event edge, so the pad's own
    // period is twice the wave's and a rising-sensed line counts half.
    bench.verdict("EVACT=TGL walks the pull under PMUXEN, at half the wave's "
                  "rate because one toggle is a whole pad period",
                  tgl_mux.moves && near(tgl_mux.awake, offered / 2u, 3));
    bench.verdict("EVACT=OUT reaches an EIC-owned pad as well, and at the "
                  "wave's own rate",
                  out_mux.moves && near(out_mux.awake, offered, 3));

    // Which arrangement, if any, keeps making edges while the CPU is
    // stopped? That is what 28.6.4's note is about, and it is the whole
    // question for everything below.
    const bool out_survives = out_mux.asleep >= offered - 3u;
    const bool tgl_survives = tgl_mux.asleep >= offered / 2u - 3u;
    bench.verdict("28.6.4's note, measured: the OUT action keeps driving a "
                  "pad through a STANDBY",
                  out_survives);
    bench.verdict("and SET/CLR/TGL do not - the note's other half",
                  !tgl_survives);

    // -- the wake, and erratum 1.11.6 ------------------------------------
    if (out_survives) {
        (void)walker_arrange(PortEventAction::out, true);

        // The wake. The line's interrupt is armed and the RTC compare is
        // only the backstop, so a wake that is not the EIC's shows up as
        // rtc_fired.
        watchdog_backstop(true);
        (void)line_mode(EicSense::rising, true, false, EicClock::ulp32k, gen_eic);
        Eic::clear_flags(StimLine::mask);
        StimLine::arm(true);
        Nvic::enable(Eic::irq());
        eic_irqs = 0;
        const uint32_t woke = standby_until_wake(window);
        const bool by_eic = eic_irqs != 0u && !rtc_fired;
        StimLine::arm(false);
        Nvic::disable(Eic::irq());
        watchdog_backstop(false);
        print(serial, "  a pad edge woke the CPU from STANDBY after ",
              watch_us(woke), " us (", eic_irqs, " EIC interrupts, backstop ",
              rtc_fired ? "fired" : "not needed", ")", crlf);
        bench.verdict("an EXTINT line WAKES the device from STANDBY", by_eic);

        // ERRATUM 1.11.6, and the errata matrix marks it LIVE ON EVERY
        // REVISION of E/G/J: "when the asynchronous edge detection is
        // enabled, and the system is in Standby mode, only the first
        // edge will be detected". Three things make the measurement
        // safe. The interrupt is DISARMED, so the EIC cannot end the
        // sleep and the whole window really is one standby; the SAMPLED
        // line is the same measurement with the erratum's own
        // precondition removed; and the CONTROL that the device was
        // asleep at all is two rows above - EVACT=TGL made 50 edges
        // awake and 0 asleep in this very window - plus the kernel tick,
        // which rides SysTick and therefore stops dead in standby.
        watchdog_backstop(true);
        (void)line_mode(EicSense::rising, true, false, EicClock::ulp32k, gen_eic);
        Eic::clear_flags(StimLine::mask);
        const uint32_t tick0 = Ticker::millis();
        const uint16_t async_asleep = eic_edges(true, window);
        const uint32_t tick_span = Ticker::millis() - tick0;
        (void)line_mode(EicSense::rising, false, false, EicClock::ulp32k, gen_eic);
        Eic::clear_flags(StimLine::mask);
        const uint16_t sync_awake = eic_edges(false, window);
        Eic::clear_flags(StimLine::mask);
        const uint16_t sync_asleep = eic_edges(true, window);

        // The second witness, and the one an application would feel:
        // with the interrupt ARMED the loop goes back to standby after
        // every wake, so the count of interrupts over the window is the
        // count of edges that ACTUALLY LEFT the device asleep.
        StimLine::arm(true);
        Nvic::enable(Eic::irq());
        (void)line_mode(EicSense::rising, true, false, EicClock::ulp32k, gen_eic);
        Eic::clear_flags(StimLine::mask);
        eic_irqs = 0;
        (void)standby_for(window);
        const uint32_t async_wakes = eic_irqs;
        (void)line_mode(EicSense::rising, false, false, EicClock::ulp32k, gen_eic);
        Eic::clear_flags(StimLine::mask);
        eic_irqs = 0;
        (void)standby_for(window);
        const uint32_t sync_wakes = eic_irqs;
        StimLine::arm(false);
        Nvic::disable(Eic::irq());
        watchdog_backstop(false);

        print(serial, "  inside ONE standby of ", offered,
              " offered edges: asynchronous detected ", async_asleep,
              ", sampled ", sync_asleep, " (sampled awake ", sync_awake,
              "); wakes: asynchronous ", async_wakes, ", sampled ", sync_wakes,
              crlf);
        print(serial, "  and the kernel tick advanced ", tick_span,
              " ms across that ", rtc_ms(window), " ms window", crlf);
        bench.verdict("the window really was spent in STANDBY: the SysTick "
                      "timebase did not move",
                      tick_span <= 20u);
        bench.verdict("a SAMPLED line detects every edge of a standby on "
                      "CLK_ULP32K",
                      near(sync_asleep, offered, 4) && near(sync_awake, offered, 4));
        bench.verdict("ERRATUM 1.11.6 DOES NOT REPRODUCE at revision F: an "
                      "ASYNCHRONOUS line detects EVERY edge of a standby, not "
                      "just the first",
                      near(async_asleep, offered, 4));
        bench.verdict("and every one of them wakes the device, as many times "
                      "as it is put back to sleep",
                      near(async_wakes, offered, 5) && near(sync_wakes, offered, 5));

        // -- which clock a sampled line needs -----------------------------
        watchdog_backstop(true);
        uint16_t gclk_std = 0xFFFFu;
        uint16_t gclk_no_std = 0xFFFFu;
        if (Gclk<gen_eic>::configure(
                GclkConfig{.source = GclkSource::osculp32k, .run_standby = true}) &&
            line_mode(EicSense::rising, false, false, EicClock::gclk, gen_eic)) {
            Eic::clear_flags(StimLine::mask);
            gclk_std = eic_edges(true, window);
        }
        if (Gclk<gen_eic>::configure(
                GclkConfig{.source = GclkSource::osculp32k, .run_standby = false}) &&
            line_mode(EicSense::rising, false, false, EicClock::gclk, gen_eic)) {
            Eic::clear_flags(StimLine::mask);
            gclk_no_std = eic_edges(true, window);
        }
        watchdog_backstop(false);
        print(serial, "  a sampled line on GCLK_EIC across a standby: "
              "generator RUNSTDBY set ", gclk_std, " edges, clear ",
              gclk_no_std, crlf);
        bench.verdict("a GCLK-clocked sampled line detects in standby when its "
                      "GENERATOR has RUNSTDBY",
                      near(gclk_std, offered, 4));
        // And the surprise: it detects just as well WITHOUT it. The EIC
        // has no RUNSTDBY bit to ask a generator with (see the last
        // verdict of this letter), so the request that keeps GCLK_EIC
        // alive here is unconditional - which is 26.5.2's "all
        // interrupts are available down to STANDBY sleep mode" turned
        // into a clock request, and is not what a reader of table 19-4
        // would predict.
        bench.verdict("AND WITH ITS RUNSTDBY CLEAR TOO: the EIC's clock "
                      "request is honoured in standby with no RUNSTDBY bit "
                      "anywhere in the chain",
                      near(gclk_no_std, offered, 4));
    } else {
        // Kept as five printed, counted, DECLINED verdicts so the
        // letter's tally does not depend on which branch the silicon
        // takes; each claims nothing.
        for (uint8_t i = 0; i < 5; ++i) {
            bench.verdict("the EIC in standby: DECLINED, no wireless stimulus "
                          "on this board can move an EXTINT pad while the CPU "
                          "is stopped",
                          true);
        }
    }

    // The EIC's own half of the question, which needs no stimulus at
    // all: 26.8.1 is SWRST, ENABLE and CKSEL and nothing else, so the
    // block cannot ask a generator to keep running for it.
    const bool no_runstdby =
        (EIC_CTRLA_Msk & ~static_cast<uint32_t>(EIC_CTRLA_SWRST_Msk |
                                                EIC_CTRLA_ENABLE_Msk |
                                                EIC_CTRLA_CKSEL_Msk)) == 0u;
    bench.verdict("the EIC has no RUNSTDBY of its own - CLK_ULP32K or a "
                  "generator that runs in standby is the only way to sample a "
                  "line there",
                  no_runstdby);

    (void)Eic::enable(false);
    Eic::release();
    counter_down();
    walker_down();
    (void)Gclk<gen_eic>::configure(
        GclkConfig{.source = GclkSource::osculp32k, .run_standby = true});
    quiesce();
}

} // namespace

// ===========================================================================
// b: the RTC as the wake source
// ===========================================================================

namespace {

void tb_rtc() {
    quiesce();
    watchdog_backstop(true);

    // -- b1: COMP0 ---------------------------------------------------------
    constexpr uint32_t p = 1000;                   // ~30 ms
    const uint32_t want_us = rtc_us(p);
    uint32_t sum = 0;
    bool all = true;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!arm_wake(p) || !arm_standby()) {
            all = false;
            break;
        }
        const uint32_t t0 = watch_now();
        uint32_t spins = spin_cap;
        while (!rtc_fired && spins-- != 0u) {
            sleep_standby_once();
        }
        const uint32_t t1 = watch_now();
        disarm_standby();
        if (!rtc_fired) {
            all = false;
            break;
        }
        sum += watch_us(t1 - t0);
    }
    const uint32_t comp_us = all ? sum / 4u : 0u;
    print(serial, "  COMP0 woke the CPU from STANDBY in ", comp_us,
          " us (asked for ", want_us, " us), 4 of 4", crlf);
    bench.verdict("an RTC compare wakes the device from STANDBY, on time",
                  all && near(comp_us, want_us, want_us / 50u + 60u));

    // -- b2: a periodic interrupt ------------------------------------------
    //
    // PER5 is 2^6 = 64 source ticks, about 1.9 ms. The interrupt is the
    // wake; the compare is disarmed so nothing else can end the sleep.
    // ITS OWN PERIOD IS MEASURED FIRST, awake, and the standby wake is
    // then judged against that rather than against arithmetic: 24.6.8.1's
    // periodic outputs come off the prescaler and what one of them is
    // worth at PRESCALER = div1 is exactly the sort of thing this suite
    // does not assume.
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::periodic(5));
    Nvic::enable(Rtc::irq());
    uint32_t per_awake = 0;
    {
        wake_flag = false;
        Rtc::clear_flags(RtcFlag::periodic(5));
        while (!wake_flag) {
        }
        const uint32_t t0 = watch_now();
        uint32_t spins = spin_cap;
        wake_flag = false;
        while (!wake_flag && spins-- != 0u) {
        }
        per_awake = watch_us(watch_now() - t0);
    }
    wake_flag = false;
    Rtc::clear_flags(RtcFlag::periodic(5));
    const uint32_t per_ticks = standby_until_wake(4000);
    const uint32_t per_us = watch_us(per_ticks);
    const bool by_per = per_ticks != 0u && !rtc_fired;
    Rtc::disarm(RtcFlag::periodic(5));
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::compare0);
    print(serial, "  PER5 interrupts every ", per_awake,
          " us awake; from STANDBY the first one woke the CPU after ", per_us,
          " us", crlf);
    bench.verdict("an RTC periodic interrupt wakes the device from STANDBY",
                  by_per);
    bench.verdict("and it arrives inside one of its own periods",
                  by_per && per_awake != 0u && per_us <= per_awake + 200u);

    // -- b3: the calendar alarm --------------------------------------------
    //
    // Mode 2 at PRESCALER = div1 makes the calendar run at the source
    // rate, so a "second" is one ULP tick and a "minute" is 60 of them:
    // an alarm masked on SECOND lands within two milliseconds instead of
    // within a minute. And 24.6.2.5's sentence that cost the RTC suite a
    // letter applies here too - the alarm arrives a whole counter period
    // after its match.
    bool mode2 = Rtc::enable(false);
    const RtcConfig cal{.mode = RtcMode::clock, .prescaler = RtcPrescaler::div1};
    mode2 = mode2 && Rtc::configure(cal) && Rtc::enable(true);
    uint32_t alarm_us = 0;
    uint32_t due_us = 0;
    if (mode2) {
        Rtc::disarm(RtcFlag::all);
        Rtc::clear_flags(RtcFlag::all);
        const RtcClockValue now{
            .second = 0, .minute = 0, .hour = 0, .day = 1, .month = 1, .year = 0};
        const RtcClockValue at{
            .second = 30, .minute = 0, .hour = 0, .day = 1, .month = 1, .year = 0};
        mode2 = Rtc::set_clock(now) && Rtc::set_alarm(at) &&
                Rtc::set_alarm_mask(RtcAlarmMask::second);
        Rtc::arm(RtcFlag::alarm0);
        // EVERY ONE OF THOSE WRITES IS SYNCHRONIZED and costs tens of
        // microseconds, which at this rate is fast-seconds - so what is
        // still due is read from the CLOCK itself, immediately before
        // the sleep, and not assumed from the value that was written.
        const RtcClockValue c = Rtc::clock_value();
        const uint32_t due = c.second < 30u ? 30u - c.second : 0u;
        wake_flag = false;
        const uint32_t t = mode2 ? standby_bare() : 0u;
        alarm_us = watch_us(t);
        due_us = rtc_us(due);
        mode2 = mode2 && t != 0u && due != 0u;
        Rtc::disarm(RtcFlag::all);
    }
    print(serial, "  the calendar alarm woke it after ", alarm_us,
          " us, with ", due_us, " us of fast-seconds still due when the "
          "sleep began (24.6.2.5 adds one counter period on top)", crlf);
    bench.verdict("a mode-2 ALARM wakes the device from STANDBY", mode2);
    // THE BAND IS SYMMETRIC AND A FEW COUNTER PERIODS WIDE, because
    // that is the resolution the question can be asked at: the CLOCK is
    // read in whole fast-seconds AND the readable value trails the
    // counter by a constant four ticks (docs/samc/rtc.md), so "what is
    // still due" is itself only known to within several of them.
    // 24.6.2.5's extra period - the RTC suite's own finding - is inside
    // that and is not re-claimed here.
    const uint32_t slack = rtc_us(2) + 200u;
    bench.verdict("and it lands where the calendar says, to the resolution "
                  "the calendar can be read at",
                  mode2 && alarm_us + slack > due_us && alarm_us < due_us + slack);

    watchdog_backstop(false);
    (void)rtc_up();
    quiesce();
}

} // namespace

// ===========================================================================
// c: FREQM finishing a measurement while the CPU sleeps
// ===========================================================================

namespace {

void tc_freqm() {
    quiesce();

    // The reference is the SLOW clock and the measured one is the
    // crystal, which is the opposite of every other use of this meter in
    // the stratum and is the whole point: the window is REFNUM reference
    // periods, so a 32 kHz reference buys milliseconds of measurement
    // where a 24 MHz one buys microseconds. Both generators keep running
    // through a standby, which is what makes the measurement possible at
    // all - the FREQM has no RUNSTDBY bit of its own.
    constexpr uint8_t refnum = 128;
    const bool up = Freqm::init(FreqmConfig{.measured_generator = gen_xtal,
                                            .reference_generator = gen_ulp,
                                            .refnum = refnum});
    bench.verdict("FREQM comes up with the crystal measured against OSCULP32K", up);
    if (!up) {
        Freqm::release();
        return;
    }
    const uint32_t window_us = rtc_us(refnum);
    print(serial, "  one measurement is ", refnum, " reference periods = ",
          window_us, " us", crlf);

    // Awake first, so the number the sleeping one produces has a control.
    const auto awake = Freqm::measure();
    const uint32_t awake_hz =
        awake ? Freqm::to_hz(*awake, ulp_hz, refnum) : 0u;

    // Now asleep. DONE is armed, the RTC is not, so the only thing that
    // can end this standby is the meter finishing.
    watchdog_backstop(true);
    Freqm::clear_flags();
    Freqm::arm(FreqmFlag::done);
    Nvic::enable(Freqm::irq());
    freqm_irqs = 0;
    wake_flag = false;
    Freqm::start();
    const uint32_t slept = standby_until_wake(4000);
    Nvic::disable(Freqm::irq());
    Freqm::disarm(FreqmFlag::done);
    watchdog_backstop(false);

    const uint32_t value = Freqm::value();
    const uint32_t asleep_hz = Freqm::to_hz(value, ulp_hz, refnum);
    const uint32_t slept_us = watch_us(slept);

    print(serial, "  awake ", awake_hz, " Hz; asleep ", asleep_hz, " Hz after ",
          slept_us, " us of STANDBY (", freqm_irqs, " DONE interrupts)", crlf);
    bench.verdict("a FREQM measurement RUNS THROUGH a standby and its DONE "
                  "interrupt is the wake",
                  slept != 0u && freqm_irqs != 0u);
    bench.verdict("the sleeping measurement agrees with the waking one",
                  awake_hz != 0u && asleep_hz != 0u &&
                      near(asleep_hz / 1000u, awake_hz / 1000u, 60u));
    bench.verdict("and the wake arrives when the window ends, not before",
                  near(slept_us, window_us, window_us / 4u + 100u));

    Freqm::release();
    quiesce();
}

} // namespace

// ===========================================================================
// d: the oscillators and the DPLL through a standby
// ===========================================================================

namespace {

/// The witness counting its OWN generic clock over one window.
uint16_t clock_ticks(bool asleep, uint32_t ticks) {
    const uint16_t before = counted();
    const bool ok = asleep ? standby_for(ticks) : poll_for(ticks);
    return ok ? advance(before) : 0xFFFFu;
}

void td_clocks() {
    quiesce();
    watchdog_backstop(true);
    constexpr uint32_t window = 1000;   // ~30 ms

    // -- OSC48M ------------------------------------------------------------
    //
    // Generator 5 takes OSC48M undivided and TC4 counts it at /1024, so
    // one window is about 1400 counts and a 16-bit counter cannot wrap.
    // The generator's own RUNSTDBY is left CLEAR throughout: the
    // question is whether the counter's request is enough on its own,
    // which is the rule test_samc_sleep established on the crystal and
    // this is the internal oscillator's turn to answer.
    bool ok = Gclk<gen_probe>::configure(GclkConfig{.source = GclkSource::osc48m});
    ok = ok && counter_on_clock(gen_probe, true);
    const uint16_t osc_awake = ok ? clock_ticks(false, window) : 0xFFFFu;
    const uint16_t osc_std = ok ? clock_ticks(true, window) : 0xFFFFu;
    const bool ok2 = counter_on_clock(gen_probe, false);
    const uint16_t osc_no_std = ok2 ? clock_ticks(true, window) : 0xFFFFu;

    // ONDEMAND, with the counter asking again. Clock::init() leaves this
    // bit CLEAR on purpose (erratum 1.2.3), so it is put straight back.
    const bool ok3 = counter_on_clock(gen_probe, true);
    Osc48m::on_demand(true);
    const uint16_t osc_ondemand = ok3 ? clock_ticks(true, window) : 0xFFFFu;
    Osc48m::on_demand(false);

    print(serial, "  OSC48M through a ", rtc_ms(window),
          " ms window, counted at /1024: awake ", osc_awake,
          ", asleep with the counter's RUNSTDBY ", osc_std, ", without it ",
          osc_no_std, ", with OSC48MCTRL.ONDEMAND set ", osc_ondemand, crlf);
    bench.verdict("OSC48M keeps running through a standby for a peripheral "
                  "that asks - the counter's own RUNSTDBY is the whole request",
                  near(osc_std, osc_awake, osc_awake / 20u + 4u));
    bench.verdict("and stops for one that does not, generator RUNSTDBY clear",
                  osc_no_std <= 10u);
    bench.verdict("ONDEMAND does not change that: a request is a request",
                  near(osc_ondemand, osc_awake, osc_awake / 20u + 4u));
    bench.verdict("OSC48MCTRL.ONDEMAND is back where Clock::init() leaves it "
                  "(erratum 1.2.3)",
                  !Osc48m::on_demand());

    // -- the FDPLL, and platform.md's open question -------------------------
    //
    // The reference is the crystal divided by 2 x (5 + 1) = 2 MHz and
    // the loop multiplies by 24, so the DCO is a crystal-locked 48 MHz
    // and nothing here depends on an RC. Erratum 1.3.4 wants the lock
    // timer's own clock connected; erratum 1.25.1 is why lock_bypass
    // defaults true.
    constexpr FdpllConfig no_std{
        .reference = DpllReference::xosc,
        .reference_hz = crystal_hz,
        .xosc_div = 5,
        .ldr = 23,
    };
    constexpr FdpllConfig with_std{
        .reference = DpllReference::xosc,
        .reference_hz = crystal_hz,
        .xosc_div = 5,
        .ldr = 23,
        .run_standby = true,
    };
    static_assert(Fdpll::dco_hz(no_std) == 48'000'000);

    uint16_t dpll_awake = 0xFFFFu;
    uint16_t dpll_no_std = 0xFFFFu;
    uint16_t dpll_std = 0xFFFFu;
    bool ready_at_wake = false;
    bool locked_at_wake = false;
    bool dpll_up = Fdpll::lock_timer_clock(gen_ulp) && Fdpll::init(no_std);
    dpll_up = dpll_up && Gclk<gen_probe>::configure(
                             GclkConfig{.source = GclkSource::dpll96m});
    dpll_up = dpll_up && counter_on_clock(gen_probe, true);
    if (dpll_up) {
        dpll_awake = clock_ticks(false, window);
        dpll_no_std = clock_ticks(true, window);
        ready_at_wake = Fdpll::clock_ready();
        locked_at_wake = Fdpll::locked();
    }
    // The same loop with RUNSTDBY. Its own registers are enable-
    // protected, so it goes down and up - and the generator has to be
    // pointed at a RUNNING source first (16.6.2.6), which is what the
    // detour through OSC48M is.
    if (dpll_up) {
        (void)Gclk<gen_probe>::configure(GclkConfig{.source = GclkSource::osc48m});
        (void)Fdpll::stop();
        if (Fdpll::init(with_std) &&
            Gclk<gen_probe>::configure(GclkConfig{.source = GclkSource::dpll96m}) &&
            counter_on_clock(gen_probe, true)) {
            dpll_std = clock_ticks(true, window);
        }
    }
    print(serial, "  the DPLL through the same window: awake ", dpll_awake,
          ", asleep with DPLLCTRLA.RUNSTDBY clear ", dpll_no_std, ", set ",
          dpll_std, "; at that wake CLKRDY ", ready_at_wake ? "1" : "0",
          " and LOCK ", locked_at_wake ? "1" : "0", crlf);
    bench.verdict("the DPLL locks to the crystal and its output counts", dpll_up &&
                      dpll_awake > 100u);
    // THE ANSWER docs/samc/platform.md LEFT OPEN, and it is the opposite
    // of what its own note guessed: the loop does NOT stop and relock.
    // With DPLLCTRLA.RUNSTDBY CLEAR the count across the standby equals
    // the count awake, tick for tick - a loop that had stopped would
    // have lost its whole 30 ms and a relock costs tens of microseconds
    // on top. The peripheral's own request carries the DPLL exactly as
    // it carries OSC48M above, and CLKRDY at the wake is no evidence of
    // either, since it reads set whatever happened.
    bench.verdict("THE FDPLL RUNS THROUGH A STANDBY for a peripheral that "
                  "asks, with DPLLCTRLA.RUNSTDBY CLEAR: it does not stop and "
                  "relock",
                  near(dpll_no_std, dpll_awake, dpll_awake / 20u + 4u));
    bench.verdict("and its CLKRDY reads SET at that wake, so the status bit "
                  "is no evidence either way",
                  ready_at_wake && locked_at_wake);
    bench.verdict("DPLLCTRLA.RUNSTDBY changes nothing measurable when "
                  "something is already asking",
                  near(dpll_std, dpll_awake, dpll_awake / 20u + 4u));
    // ERRATUM 1.3.1 - "when entering Standby mode, the FDPLL is still
    // running EVEN IF NOT REQUESTED BY ANY MODULE, causing extra
    // consumption" - is REVISION B ONLY, and it is a CONSUMPTION claim
    // about the unrequested case. Both halves put it out of this bench's
    // reach: the only witness of a running loop is a peripheral clocked
    // from it, which is itself a request, and sleep current has no meter
    // here. Printed, counted, and claiming nothing.
    bench.verdict("ERRATUM 1.3.1 (the FDPLL running in standby UNREQUESTED): "
                  "DECLINED - it is revision B, it is about consumption, and "
                  "the only witness of a running loop is a request",
                  true);

    (void)Gclk<gen_probe>::configure(GclkConfig{.source = GclkSource::osc48m});
    (void)Fdpll::stop();

    // -- OSC32K -------------------------------------------------------------
    //
    // The 32 kHz roots differ from every other clock here in that
    // OSCULP32K has no RUNSTDBY bit at all - it is simply always on -
    // while OSC32K has both bits. The trim is the factory one, which
    // docs/samc/osc32kctrl.md measured to be worth 44 % of the rate.
    const uint8_t trim = Osc32k::factory_calib();
    uint16_t o32_std = 0xFFFFu;
    uint16_t o32_no_std = 0xFFFFu;
    uint16_t o32_awake = 0xFFFFu;
    bool o32 = Osc32k::init(Osc32kConfig{.calib = trim, .run_standby = true});
    o32 = o32 && Gclk<gen_probe>::configure(GclkConfig{.source = GclkSource::osc32k});
    o32 = o32 && counter_on_clock(gen_probe, true, TcPrescaler::div1);
    if (o32) {
        o32_awake = clock_ticks(false, window);
        o32_std = clock_ticks(true, window);
        // RUNSTDBY clear, with the counter still asking.
        if (Osc32k::init(Osc32kConfig{.calib = trim, .run_standby = false})) {
            o32_no_std = clock_ticks(true, window);
        }
    }
    print(serial, "  OSC32K through the same window: awake ", o32_awake,
          ", asleep with its RUNSTDBY set ", o32_std, ", clear ", o32_no_std,
          crlf);
    bench.verdict("OSC32K comes up on its factory trim and its generator "
                  "counts",
                  o32 && o32_awake > 100u);
    bench.verdict("with OSC32K.RUNSTDBY set it survives a standby",
                  near(o32_std, o32_awake, o32_awake / 20u + 4u));
    bench.verdict("and WITHOUT it, it survives too - the counter's own "
                  "request is what carries it, exactly as with OSC48M",
                  near(o32_no_std, o32_awake, o32_awake / 20u + 4u));

    // The teardown order is the one docs/samc/osc32kctrl.md paid for: a
    // generator may not be left pointing at a source that is about to
    // stop (16.6.2.6).
    (void)Gclk<gen_probe>::configure(GclkConfig{.source = GclkSource::osc48m});
    Osc32k::stop();
    counter_down();
    watchdog_backstop(false);

    // ERRATUM 1.25.2, live on every revision: "the FDPLL96M On Demand
    // mode is not functional in Standby Sleep mode". It is UNREACHABLE
    // BY CONSTRUCTION here - samc/clock.hpp never sets DPLLCTRLA.ONDEMAND
    // and offers no verb that could - so the bit is read instead of
    // provoked.
    const bool ondemand_clear =
        (OSCCTRL_REGS->OSCCTRL_DPLLCTRLA & OSCCTRL_DPLLCTRLA_ONDEMAND_Msk) == 0u;
    bench.verdict("ERRATUM 1.25.2 is unreachable by construction: the driver "
                  "leaves DPLLCTRLA.ONDEMAND clear and has no verb to set it",
                  ondemand_clear);
    quiesce();
}

} // namespace

// ===========================================================================
// e: the ADC - table 38-4, and a real SleepWalking conversion
// ===========================================================================

namespace {

using Converter = Adc<0>;

/// The RTC periodic interval that paces every SleepWalking chain below.
/// Its rate is MEASURED rather than derived - letter b already found
/// that PER5 is 256 source ticks and not the 64 the obvious arithmetic
/// gives.
constexpr uint8_t pace_interval = 2;

/// Count `gen_code` events over one window, asleep or awake.
uint16_t events_over(bool asleep, uint32_t ticks) {
    const uint16_t before = counted();
    const bool ok = asleep ? standby_for(ticks) : poll_for(ticks);
    return ok ? advance(before) : 0xFFFFu;
}

void te_adc() {
    quiesce();
    constexpr uint32_t window = 1000;   // ~30 ms

    // The converter is clocked from the CRYSTAL generator, which runs
    // through a standby whatever anybody asks, so the only thing under
    // test is CTRLA - and its prescaler puts CLK_ADC at 1.5 MHz, inside
    // the range 38.6.2.2 allows.
    const AdcConfig base{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div16,
        .sample_length = 4,
        .run_standby = false,
        .on_demand = false,
        .events = AdcEventControl{.start_in = true, .result_out = true},
    };
    bool ok = Converter::init(gen_xtal, base);
    Converter::select(AdcInput::scaled_supply);
    // 1/4 of VDDANA against VDDANA is a quarter of full scale exactly,
    // which is the cheapest self-check this converter has.
    ok = ok && Converter::init(gen_xtal, base);
    bench.verdict("ADC0 comes up on the crystal with its START event input "
                  "enabled",
                  ok);
    if (!ok) {
        Converter::release();
        return;
    }

    // ERRATUM 1.4.4, all revisions: a SYNCHRONIZED event during a
    // conversion stalls the whole channel, so an ADC event user takes
    // the asynchronous path or nothing - which start_on() enforces.
    const bool paced = rtc_periodic_event(pace_interval);
    // start_on() needs the converter DISABLED - EVCTRL is
    // enable-protected - and it refuses anything but the asynchronous
    // path, which is erratum 1.4.4 as code. Both are exercised here.
    (void)Converter::enable(false);
    const bool refused = !Converter::start_on(
        ev_start,
        EventChannelConfig{.generator = Rtc::periodic_generator(pace_interval),
                           .path = EventPath::resynchronized,
                           .edge = EventEdge::rising});
    const bool routed = Converter::start_on(
        ev_start,
        EventChannelConfig{.generator = Rtc::periodic_generator(pace_interval),
                           .path = EventPath::asynchronous,
                           .run_standby = true});
    (void)Converter::enable(true);
    bench.verdict("and it REFUSES a resynchronized one - erratum 1.4.4 as code",
                  refused);
    const bool witnessed =
        counter_on_events(Converter::resrdy_generator, gen_ulp, true);
    bench.verdict("the RTC publishes its periodic interval as an event", paced);
    bench.verdict("the ADC's START user takes it on the asynchronous path "
                  "erratum 1.4.4 makes mandatory",
                  routed);
    bench.verdict("and the witness counts RESRDY", witnessed);
    ok = paced && routed && witnessed;
    if (!ok) {
        (void)Converter::stop_events();
        (void)rtc_periodic_event(8);
        counter_down();
        Converter::release();
        return;
    }

    watchdog_backstop(true);
    const uint16_t awake = events_over(false, window);

    // Table 38-4's four rows. Only CTRLA moves between them.
    uint16_t row[4] = {0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu};
    for (uint8_t i = 0; i < 4; ++i) {
        AdcConfig c = base;
        c.run_standby = (i & 2u) != 0u;
        c.on_demand = (i & 1u) != 0u;
        if (!Converter::init(gen_xtal, c)) {
            continue;
        }
        Converter::select(AdcInput::scaled_supply);
        Converter::clear_flags(Converter::flag_resrdy | Converter::flag_overrun);
        row[i] = events_over(true, window);
    }
    // Erratum 1.4.5, live on every revision: "ADC SYNCBUSY.SWTRIG becomes
    // stuck to one after wake-up from Standby Sleep mode".
    const uint16_t sync_at_wake = Converter::sync_busy();
    const uint16_t result = Converter::result();
    watchdog_backstop(false);

    print(serial, "  conversions in one ", rtc_ms(window), " ms window: awake ",
          awake, "; asleep by table 38-4 row: RUNSTDBY 0 ONDEMAND 0 ", row[0],
          ", 0/1 ", row[1], ", 1/0 ", row[2], ", 1/1 ", row[3], crlf);
    bench.verdict("the RTC really does start conversions with no CPU in the "
                  "loop",
                  awake > 4u && awake != 0xFFFFu);
    bench.verdict("table 38-4 rows 1 and 2 (RUNSTDBY = 0): nothing converts "
                  "in STANDBY",
                  row[0] <= 2u && row[1] <= 2u);
    bench.verdict("rows 3 and 4 (RUNSTDBY = 1): a REAL SleepWalking "
                  "conversion, at the same rate as awake",
                  near(row[2], awake, awake / 8u + 2u) &&
                      near(row[3], awake, awake / 8u + 2u));
    print(serial, "  the last result read at a wake is ", result, " of ",
          Converter::result_steps(), " (a quarter of full scale is due), and "
          "SYNCBUSY reads ", sync_at_wake, " there", crlf);
    bench.verdict("and the result taken at the wake is the quarter of full "
                  "scale the internal divider owes",
                  near(result, Converter::result_steps() / 4u,
                       Converter::result_steps() / 40u));
    bench.verdict("ERRATUM 1.4.5 (SYNCBUSY.SWTRIG stuck after a standby "
                  "wake) does NOT reproduce at revision F",
                  (sync_at_wake & ADC_SYNCBUSY_SWTRIG_Msk) == 0u);

    (void)Converter::stop_events();
    (void)rtc_periodic_event(8);
    counter_down();
    Converter::release();
    quiesce();
}

} // namespace

// ===========================================================================
// f: the SDADC and the TSENS on their own sleep tables
// ===========================================================================

namespace {

void tf_sdadc_tsens() {
    quiesce();
    constexpr uint32_t window = 1000;   // ~30 ms

    // -- the SDADC, table 39-1 (the ADC's table under another number) ------
    //
    // Free-running rather than event-paced, which is also erratum
    // 1.8.7's own escape: that item is about a DMA write to SWTRIG made
    // while SleepWalking, and a free-running converter writes no
    // trigger at all.
    const SdadcConfig base{
        .reference = SdadcRef::vddana,
        .prescaler = 15,
        .osr = SdadcOsr::osr64,
        .skip_count = 2,
        .free_running = true,
        .run_standby = false,
        .on_demand = false,
        .events = SdadcEventControl{.result_out = true},
    };
    bool ok = Sdadc::init(gen_xtal, base);
    ok = ok && Sdadc::select(0);
    ok = ok && counter_on_events(Sdadc::resrdy_generator, gen_ulp, true);
    bench.verdict("the SDADC comes up free-running with the witness on its "
                  "RESRDY",
                  ok);
    uint16_t sd_awake = 0xFFFFu;
    uint16_t sd_no_std = 0xFFFFu;
    uint16_t sd_std = 0xFFFFu;
    if (ok) {
        watchdog_backstop(true);
        sd_awake = events_over(false, window);
        sd_no_std = events_over(true, window);
        SdadcConfig c = base;
        c.run_standby = true;
        if (Sdadc::init(gen_xtal, c) && Sdadc::select(0)) {
            sd_std = events_over(true, window);
        }
        watchdog_backstop(false);
    }
    print(serial, "  SDADC conversions in one ", rtc_ms(window),
          " ms window: awake ", sd_awake, ", asleep with RUNSTDBY clear ",
          sd_no_std, ", set ", sd_std, crlf);
    bench.verdict("table 39-1 row 1: a free-running SDADC stops in STANDBY "
                  "without RUNSTDBY",
                  ok && sd_no_std <= 2u);
    bench.verdict("and row 3: with RUNSTDBY it converts right through, at "
                  "the same rate",
                  ok && near(sd_std, sd_awake, sd_awake / 8u + 2u));
    Sdadc::release();
    counter_down();

    // -- the TSENS, table 43-1 ---------------------------------------------
    //
    // Four rows here rather than two, because the second knob is
    // CTRLC.FREERUN and not ONDEMAND: with FREERUN clear the block runs
    // "on request", which in a standby means nothing requests it.
    // THE WITNESS HAD TO BE THE WINDOW MONITOR: unlike every other
    // converter here the TSENS publishes NO result-ready event
    // generator (43.6.5 lists WINMON alone), so a window whose lower
    // limit is below the datum's rail matches on every measurement and
    // its event is the count.
    const TsensConfig tbase{
        .calibration = TsensCalibration::factory(),
        .free_running = true,
        .window = TsensWindow::above,
        .window_lower = -8'000'000,
        .run_standby = false,
        .events = TsensEventControl{.window_out = true},
    };
    bool t_ok = Tsens::init(gen_xtal, tbase);
    t_ok = t_ok && counter_on_events(Tsens::window_generator, gen_ulp, true);
    uint16_t ts_awake = 0xFFFFu;
    uint16_t ts_no_std = 0xFFFFu;
    uint16_t ts_std = 0xFFFFu;
    if (t_ok) {
        watchdog_backstop(true);
        ts_awake = events_over(false, window);
        ts_no_std = events_over(true, window);
        TsensConfig c = tbase;
        c.run_standby = true;
        if (Tsens::init(gen_xtal, c)) {
            ts_std = events_over(true, window);
        }
        watchdog_backstop(false);
    }
    print(serial, "  TSENS measurements in one ", rtc_ms(window),
          " ms window: awake ", ts_awake, ", asleep with RUNSTDBY clear ",
          ts_no_std, ", set ", ts_std, crlf);
    bench.verdict("the TSENS comes up free-running on its factory calibration",
                  t_ok && ts_awake > 0u && ts_awake != 0xFFFFu);
    bench.verdict("table 43-1 row 2 (FREERUN 1, RUNSTDBY 0): it stops in "
                  "STANDBY",
                  t_ok && ts_no_std <= 1u);
    bench.verdict("row 4 (FREERUN 1, RUNSTDBY 1): it measures right through "
                  "a standby",
                  t_ok && near(ts_std, ts_awake, ts_awake / 4u + 1u));
    Tsens::release();
    counter_down();
    quiesce();
}

} // namespace

// ===========================================================================
// g: the DAC's output buffer through a standby
// ===========================================================================

namespace {

void tg_dac() {
    quiesce();
    constexpr uint32_t window = 1000;
    constexpr uint16_t code = 512;      // half of the ten-bit scale

    // PA02 is the DAC's VOUT pad AND ADC0's AIN0, so the wire erratum
    // 1.8.9's workaround asks for has zero length. The converter reads
    // THE PAD and not the internal channel, which is that workaround.
    bool ok = Converter::init(gen_xtal,
                              AdcConfig{.reference = Ref::vddana,
                                        .prescaler = AdcPresc::div16,
                                        .sample_length = 16});
    Converter::claim_pad<Pin<'A', 2>>();
    Converter::select(AnalogIn<Pin<'A', 2>>{});
    bench.verdict("ADC0 reads the DAC's own pad, PA02", ok);

    uint16_t before[2] = {0, 0};
    uint16_t after[2] = {0, 0};
    bool empty_after[2] = {false, false};
    bool dac_ok = true;
    for (uint8_t i = 0; i < 2; ++i) {
        const bool std = i == 1u;
        dac_ok = dac_ok && Dac::init(gen_xtal,
                                     DacConfig{.external_output = true,
                                               .run_standby = std});
        dac_ok = dac_ok && Dac::set(code) && Dac::wait_ready();
        Converter::discard(2);
        before[i] = Converter::read();
        // ERRATUM 1.9.2, all revisions: with RUNSTDBY clear and DATABUF
        // written, INTFLAG.EMPTY comes back set from the standby. The
        // buffer is deliberately loaded and left unconsumed - there is
        // no start event here to take it.
        Dac::clear_flags(Dac::flag_empty);
        Dac::buffer(code);
        watchdog_backstop(true);
        (void)standby_for(window);
        watchdog_backstop(false);
        after[i] = Converter::read();
        empty_after[i] = (Dac::flags() & Dac::flag_empty) != 0u;
    }
    print(serial, "  the DAC's pad at code ", code, " of ", Dac::steps,
          ": RUNSTDBY clear ", before[0], " before and ", after[0],
          " after a standby; set ", before[1], " before and ", after[1],
          " after", crlf);
    bench.verdict("the DAC comes up on both settings and the pad reads about "
                  "half scale",
                  dac_ok && near(before[1], Converter::result_steps() / 2u,
                                 Converter::result_steps() / 8u));
    bench.verdict("41.6.6: with CTRLA.RUNSTDBY the output buffer KEEPS ITS "
                  "VALUE across a standby",
                  near(after[1], before[1], Converter::result_steps() / 40u + 8u));
    print(serial, "  INTFLAG.EMPTY after the standby: RUNSTDBY clear ",
          empty_after[0] ? "1" : "0", ", set ", empty_after[1] ? "1" : "0",
          crlf);
    bench.verdict("ERRATUM 1.9.2 REPRODUCES with RUNSTDBY clear, and its own "
                  "control - RUNSTDBY set - does not",
                  empty_after[0] && !empty_after[1]);

    Dac::release();
    Converter::release_pad<Pin<'A', 2>>();
    Converter::release();
    quiesce();
}

} // namespace

// ===========================================================================
// h: the AC's two sequences of 40.6.14
// ===========================================================================

namespace {

using Comparator = AcComparator<0>;
using AcPad = Pin<'A', 4>;    // AIN0, comparator 0's pin0

/// The walker's SECOND event input: PORT event 1, action OUT, on PA04 -
/// the pad the AC's positive input takes. One event input addresses one
/// pin, so the two live side by side on the same channel.
bool ac_pad_walker_up(uint8_t period, TcPrescaler prescaler) {
    if (!toggler_up(period, prescaler) || !Ccl::init(gen_ulp) || !stim_lut_up()) {
        return false;
    }
    if (!Evsys::connect(Port<'A'>::event_user(1), ev_stim,
                        EventChannelConfig{.generator = Stim::event_generator,
                                           .path = EventPath::asynchronous,
                                           .run_standby = true})) {
        return false;
    }
    if (!Port<'A'>::configure_event(1, PortEventConfig{.pin = AcPad::pin_number,
                                                       .action = PortEventAction::out,
                                                       .enable = true})) {
        return false;
    }
    AcPad::output();
    return true;
}

void ac_pad_walker_down() {
    Port<'A'>::release_events();
    Evsys::disconnect(Port<'A'>::event_user(1));
    Evsys::release_channel(ev_stim);
    Ccl::enable(false);
    Stim::enable(false);
    Ccl::release();
    toggler_down();
    AcPad::configure({});
}

void th_ac() {
    quiesce();

    // THE WAVE IS SLOW HERE, and that is the whole design of the letter:
    // 16 counts of OSCULP32K / 64 is a period of about 31 ms, so half a
    // period - the time to the next pad edge after the counter is
    // retriggered - is 15 ms, where entering a standby costs microseconds
    // and the RTC backstop is 90 ms away. Three separated numbers, so
    // "the comparator woke it" and "nothing did" cannot be confused. The
    // first version used a 2 ms wave and was a coin toss: the flip
    // arrived between clearing the flag and the WFI as often as not.
    constexpr uint8_t wave = 16;
    constexpr uint32_t window = 3000;   // ~91 ms

    bool ok = ac_pad_walker_up(wave, TcPrescaler::div64);
    ok = ok && Ac::init(gen_ulp);
    bench.verdict("PA04 is walked by the PORT event and the AC comes up", ok);
    if (!ok) {
        Ac::release();
        ac_pad_walker_down();
        return;
    }

    // -- 40.6.14.1, continuous measurement during sleep --------------------
    //
    // Table 40-1's continuous row is two lines: RUNSTDBY = 0 means
    // "COMPx disabled" and RUNSTDBY = 1 means "GCLK_AC stopped, COMPx
    // enabled" - the comparator watching asynchronously and starting its
    // own clock only when an edge matches. THREE legs, because two are
    // not enough to say which bit did what: the third takes GCLK_AC off
    // a generator that survives standby, and it is the one that shows
    // what COMPCTRL.RUNSTDBY is actually gating.
    // AND EACH LEG IS RUN FOUR TIMES, because the first version of this
    // measurement was a single shot and one of its runs disagreed with
    // the next: a wake counted out of four is a fact, a wake counted
    // once is a coin.
    constexpr uint8_t rounds = 8;
    struct Leg3 {
        bool std;
        uint8_t gen;
        bool gen_std;
        uint32_t woke;
        uint8_t by_ac;
        uint8_t by_rtc;
    };
    Leg3 leg[3] = {
        {false, gen_ulp, true, 0, 0, 0},   // RUNSTDBY 0, clock force-fed
        {true, gen_ulp, true, 0, 0, 0},    // RUNSTDBY 1, clock force-fed
        {false, gen_eic, false, 0, 0, 0},  // RUNSTDBY 0, clock stops too
    };
    for (Leg3& l : leg) {
        if (!Gclk<gen_eic>::configure(GclkConfig{.source = GclkSource::osculp32k,
                                                 .run_standby = l.gen_std}) ||
            !Ac::init(l.gen)) {
            continue;
        }
        if (!Comparator::configure(AcConfig{.positive = AcPositive::pin0,
                                            .negative = AcNegative::vscale,
                                            .interrupt_on = AcInterrupt::toggle,
                                            .run_standby = l.std})) {
            continue;
        }
        Comparator::scaler(31);
        (void)Ac::enable(true);
        (void)Comparator::enable(true);
        for (uint32_t k = 0; k < 200'000u && !Comparator::ready(); ++k) {
        }
        for (uint8_t r = 0; r < rounds; ++r) {
            // The wave restarted from zero, so the next pad edge is half
            // a period away and the sleep is entered long before it.
            (void)Toggler::retrigger();
            Comparator::clear_flag();
            Comparator::arm(true);
            Nvic::enable(Ac::irq());
            ac_irqs = 0;
            watchdog_backstop(true);
            l.woke = standby_until_wake(window);
            watchdog_backstop(false);
            Comparator::arm(false);
            Nvic::disable(Ac::irq());
            // Judged by the LENGTH of the sleep: the comparator comes
            // back the instant the CPU does, so its first flip AFTER a
            // wake is counted whatever happened during the standby. A
            // sleep that ran the whole backstop is a sleep nothing else
            // ended.
            const uint32_t us = watch_us(l.woke);
            if (ac_irqs != 0u && !rtc_fired && us < (rtc_us(window) * 2u) / 3u) {
                l.by_ac = static_cast<uint8_t>(l.by_ac + 1u);
            }
            if (rtc_fired && us > (rtc_us(window) * 2u) / 3u) {
                l.by_rtc = static_cast<uint8_t>(l.by_rtc + 1u);
            }
        }
        (void)Comparator::enable(false);
        (void)Ac::enable(false);
    }
    (void)Gclk<gen_eic>::configure(
        GclkConfig{.source = GclkSource::osculp32k, .run_standby = true});

    print(serial, "  the next pad edge is ", rtc_us(wave * 64u / 2u),
          " us away and the RTC backstop ", rtc_us(window), " us:", crlf);
    print(serial, "    RUNSTDBY 0, GCLK_AC on a standby generator  ",
          leg[0].by_ac, " AC wakes of ", rounds, ", last at ",
          watch_us(leg[0].woke), " us", crlf);
    print(serial, "    RUNSTDBY 1, GCLK_AC on a standby generator  ",
          leg[1].by_ac, " AC wakes of ", rounds, ", last at ",
          watch_us(leg[1].woke), " us", crlf);
    print(serial, "    RUNSTDBY 0, GCLK_AC stopping with the CPU   ",
          leg[2].by_ac, " AC wakes of ", rounds, ", last at ",
          watch_us(leg[2].woke), " us", crlf);
    bench.verdict("40.6.14.1: a CONTINUOUS comparator with RUNSTDBY wakes the "
                  "device from STANDBY on its own edge, every round",
                  leg[1].by_ac == rounds);
    bench.verdict("and with RUNSTDBY clear it is disabled there, exactly as "
                  "table 40-1 says: nothing but the RTC backstop ended those "
                  "sleeps",
                  leg[2].by_ac == 0u && leg[2].by_rtc == rounds);
    // THE THIRD LEG'S BAND IS WIDE ON PURPOSE, and the number behind it
    // is printed above. Force-feeding GCLK_AC from a generator that runs
    // in standby does NOT revive a comparator whose RUNSTDBY is clear -
    // measured 0, 0, 0 and 1 wake out of eight over four runs, against a
    // working comparator's eight out of eight. So the bit gates the
    // comparator and not just its clock; but the rare single wake is
    // real and is recorded rather than rounded away.
    bench.verdict("and COMPCTRL.RUNSTDBY gates the COMPARATOR and not just "
                  "its clock: force-feeding GCLK_AC does not revive it, bar a "
                  "rare stray wake",
                  leg[0].by_ac <= rounds / 2u);

    // -- 40.6.14.2, single-shot SleepWalking -------------------------------
    //
    // The comparator is idle until an event starts it, so the RTC's
    // periodic pulse is the trigger - on the ASYNCHRONOUS path, which is
    // all table 29-3 grants the AC's SOC users. The interrupt is NOT
    // armed: the whole point is that the comparison happens while the
    // CPU stays asleep and the flag is read at the wake.
    bool single = Comparator::configure(
        AcConfig{.positive = AcPositive::pin0,
                 .negative = AcNegative::vscale,
                 .single_shot = true,
                 .interrupt_on = AcInterrupt::end_of_comparison,
                 .run_standby = true});
    single = single && Ac::event_config(AcEventControl{.start_in = 1u});
    single = single && rtc_periodic_event(pace_interval);
    single = single &&
             Evsys::connect(Comparator::start_event_user, ev_start,
                            EventChannelConfig{
                                .generator = Rtc::periodic_generator(pace_interval),
                                .path = EventPath::asynchronous,
                                .run_standby = true});
    Comparator::scaler(31);
    (void)Ac::enable(true);
    (void)Comparator::enable(true);
    Comparator::clear_flag();
    watchdog_backstop(true);
    (void)standby_for(window);
    watchdog_backstop(false);
    const bool flagged = Comparator::flag_set();
    print(serial, "  the single-shot chain: INTFLAG at the wake is ",
          flagged ? "set" : "clear", crlf);
    bench.verdict("40.6.14.2: an RTC event on the ASYNCHRONOUS path starts a "
                  "single-shot comparison DURING a standby",
                  single && flagged);

    Evsys::disconnect(Comparator::start_event_user);
    (void)rtc_periodic_event(8);
    (void)Comparator::enable(false);
    Ac::release();
    ac_pad_walker_down();
    quiesce();
}

} // namespace

// ===========================================================================
// i: CCL 37.6.4 - which LUTs keep decoding in standby
// ===========================================================================

namespace {

/// One standby window, with the LUT configured as the caller says, and
/// the witness counting the LUT's own output events.
uint16_t lut_edges(LutFilter filter, bool ccl_run_standby, bool asleep,
                   uint32_t ticks) {
    Ccl::enable(false);
    if (!Ccl::run_standby(ccl_run_standby)) {
        return 0xFFFEu;
    }
    if (!stim_lut_up(filter)) {
        return 0xFFFEu;
    }
    const uint16_t before = counted();
    const bool ok = asleep ? standby_for(ticks) : poll_for(ticks);
    return ok ? advance(before) : 0xFFFFu;
}

void ti_ccl() {
    quiesce();

    constexpr uint8_t wave = 32;
    constexpr uint32_t window = 3200;
    constexpr uint32_t expect = window / wave;

    // The CCL needs a clock for the filter and the synchronizer, and
    // that clock has to survive a standby when CTRL.RUNSTDBY says so -
    // so it is on the OSCULP32K generator, whose own RUNSTDBY is set.
    bool ok = toggler_up(wave);
    ok = ok && Ccl::init(gen_ulp);
    ok = ok && counter_on_events(Stim::event_generator, gen_ulp, true);
    bench.verdict("the CCL, its clock and the LUT-output witness come up", ok);
    if (!ok) {
        counter_down();
        Ccl::release();
        toggler_down();
        return;
    }

    watchdog_backstop(true);
    const uint16_t comb_awake = lut_edges(LutFilter::none, false, false, window);
    const uint16_t comb_asleep = lut_edges(LutFilter::none, false, true, window);
    const uint16_t sync_awake = lut_edges(LutFilter::sync, false, false, window);
    const uint16_t sync_asleep = lut_edges(LutFilter::sync, false, true, window);
    const uint16_t sync_std = lut_edges(LutFilter::sync, true, true, window);
    const uint16_t filt_asleep = lut_edges(LutFilter::filter, false, true, window);
    const uint16_t filt_std = lut_edges(LutFilter::filter, true, true, window);
    watchdog_backstop(false);

    print(serial, "  LUT output events in one ", rtc_ms(window), " ms window, ",
          expect, " offered:", crlf);
    print(serial, "    combinational          awake ", comb_awake, "  asleep ",
          comb_asleep, crlf);
    print(serial, "    FILTSEL=SYNCH          awake ", sync_awake, "  asleep ",
          sync_asleep, "  asleep with CTRL.RUNSTDBY ", sync_std, crlf);
    print(serial, "    FILTSEL=FILTER                          asleep ",
          filt_asleep, "  asleep with CTRL.RUNSTDBY ", filt_std, crlf);

    bench.verdict("37.6.4 first half: a COMBINATIONAL LUT keeps decoding "
                  "through a standby with no clock at all",
                  near(comb_asleep, expect, 3) && near(comb_awake, expect, 3));
    // "Forced to zero" is judged against a handful and not against
    // exactly zero: the counter is read AFTER the wake, and the LUT
    // starts decoding again the instant the clock comes back, so one or
    // two events at the seam belong to the wake and not to the sleep.
    bench.verdict("37.6.4 second half: a SYNCHRONIZED LUT's output is forced "
                  "to zero in standby without CTRL.RUNSTDBY",
                  sync_asleep <= 3u && near(sync_awake, expect, 3));
    bench.verdict("and CTRL.RUNSTDBY brings it back",
                  near(sync_std, expect, 3));
    bench.verdict("the FILTERED LUT behaves exactly as the synchronized one",
                  filt_asleep <= 3u && near(filt_std, expect, 3));

    Ccl::enable(false);
    (void)Ccl::run_standby(false);
    counter_down();
    Ccl::release();
    Evsys::release_channel(ev_count);
    toggler_down();
    quiesce();
}

} // namespace

// ===========================================================================
// j: the leftovers - the TCC, the EVSYS channel, the BODVDD
// ===========================================================================

namespace {

void tj_leftovers() {
    quiesce();
    constexpr uint8_t wave = 32;
    constexpr uint32_t window = 1000;   // ~30 ms

    // -- 36.6.6, the TCC ----------------------------------------------------
    //
    // One sentence and one bit: "to be able to run in standby the
    // RUNSTDBY bit in the Control A register must be '1'". TCC0 on the
    // OSCULP32K generator, its OVERFLOW published as an event, and the
    // witness counting overflows through a standby.
    using Control = Tcc<0>;
    const TccConfig tbase{.prescaler = TccPrescaler::div1, .run_standby = false};
    bool ok = Control::init(gen_ulp);
    ok = ok && Control::configure(tbase);
    ok = ok && Control::event_config(tbase, TccEventConfig{.overflow_out = true});
    ok = ok && Control::set_period(63);
    ok = ok && Control::enable(true);
    ok = ok && counter_on_events(Control::overflow_generator, gen_ulp, true);
    uint16_t tcc_awake = 0xFFFFu;
    uint16_t tcc_no_std = 0xFFFFu;
    uint16_t tcc_std = 0xFFFFu;
    if (ok) {
        watchdog_backstop(true);
        tcc_awake = events_over(false, window);
        tcc_no_std = events_over(true, window);
        (void)Control::enable(false);
        TccConfig c = tbase;
        c.run_standby = true;
        if (Control::configure(c) &&
            Control::event_config(c, TccEventConfig{.overflow_out = true}) &&
            Control::set_period(63) && Control::enable(true)) {
            tcc_std = events_over(true, window);
        }
        watchdog_backstop(false);
    }
    print(serial, "  TCC0 overflows in one ", rtc_ms(window),
          " ms window: awake ", tcc_awake, ", asleep with RUNSTDBY clear ",
          tcc_no_std, ", set ", tcc_std, crlf);
    bench.verdict("TCC0 counts and publishes its overflow as an event", ok &&
                      tcc_awake > 4u && tcc_awake != 0xFFFFu);
    bench.verdict("36.6.6: without CTRLA.RUNSTDBY the TCC stops in STANDBY",
                  ok && tcc_no_std <= 2u);
    bench.verdict("and with it the counter runs right through, at the same "
                  "rate",
                  ok && near(tcc_std, tcc_awake, tcc_awake / 8u + 2u));
    (void)Control::enable(false);
    Control::release();
    counter_down();

    // -- 29.6.4 and table 29-1, the EVSYS channel ---------------------------
    //
    // THE FINDING THAT BUILT THIS SUITE. Table 29-1 lists FOUR rows and
    // three of them are SYNC/RESYNC, which invites the reading that
    // CHANNELn.RUNSTDBY is a synchronous-path concern - the async path
    // having no clock to keep alive. It is not: 29.6.4's own sentence
    // says a channel needs the bit "to be able to run in Standby mode",
    // the table's one ASYNC row (ONDEMAND 0, RUNSTDBY 0) reads "Disabled
    // in Standby Sleep mode", and here it is measured. Nothing else in
    // the chain moves between the two legs.
    ok = toggler_up(wave) && Ccl::init(gen_ulp) && stim_lut_up();
    ok = ok && counter_on_events(Stim::event_generator, gen_ulp, true);
    uint16_t ch_awake = 0xFFFFu;
    uint16_t ch_std = 0xFFFFu;
    uint16_t ch_no_std = 0xFFFFu;
    if (ok) {
        watchdog_backstop(true);
        ch_awake = events_over(false, window);
        ch_std = events_over(true, window);
        if (counter_watch(Stim::event_generator, false)) {
            ch_no_std = events_over(true, window);
        }
        watchdog_backstop(false);
    }
    print(serial, "  a hardware event over an ASYNCHRONOUS channel in one ",
          rtc_ms(window), " ms window: awake ", ch_awake,
          ", asleep with CHANNEL.RUNSTDBY set ", ch_std, ", clear ", ch_no_std,
          crlf);
    bench.verdict("an asynchronous channel carries events in standby with "
                  "CHANNELn.RUNSTDBY set",
                  ok && near(ch_std, ch_awake, ch_awake / 8u + 2u));
    bench.verdict("AND CARRIES NOTHING WITHOUT IT: table 29-1's rule reaches "
                  "the ASYNCHRONOUS path too, which its own layout hides",
                  ok && ch_no_std <= 2u);
    counter_down();
    Ccl::enable(false);
    Ccl::release();
    toggler_down();

    // -- 22.6.3, the BODVDD in standby --------------------------------------
    //
    // NOTHING IS FORCED. The action is `none` throughout, so a
    // detection is a flag and never a reset, and the board's own boot
    // configuration is restored at the end bit for bit. The level is
    // taken ABOVE the supply so the detector really has something to
    // report; docs/samc/supc.md put this board at about 5.1 V and the
    // level step at 48.7 mV.
    const uint32_t boot_bodvdd = BodVdd::reg();
    bool bod_ok = BodVdd::configure(BodVddConfig{.level = BodVdd::level_max,
                                                 .action = BodAction::none,
                                                 .run_standby = true});
    // The detector has a start-up of its own and 22.8.4's READY is what
    // says it is over, so the flag is read only once the block admits
    // to being ready - the supc campaign's own discipline.
    for (uint32_t i = 0; i < 200'000u && !BodVdd::ready(); ++i) {
    }
    const bool bod_ready = BodVdd::ready();
    Supc::clear_flags(SupcFlag::bodvdd_detect);
    const bool detected_awake = bod_ok && BodVdd::detected();

    // IS THE FLAG A LEVEL OR A TRANSITION? Asked awake, where the
    // answer can be checked: the condition is standing (STATUS.BODVDDDET
    // reads 1), the flag is cleared, and hundreds of sampling periods
    // are allowed to pass.
    Supc::clear_flags(SupcFlag::bodvdd_detect);
    spin(200'000);
    const bool flag_returns = (Supc::flags() & SupcFlag::bodvdd_detect) != 0u;
    const bool still_detected = BodVdd::detected();

    // And the same across a standby, with the detector SAMPLING there
    // (STDBYCFG set, the fastest prescaler - about 60 us on OSCULP32K)
    // and its interrupt armed. If a sampling detector re-raised the flag
    // this would be a wake; the RTC backstop is what ends it instead.
    bool woke_by_bod = false;
    uint32_t bod_sleep = 0;
    if (BodVdd::configure(BodVddConfig{.level = BodVdd::level_max,
                                       .action = BodAction::none,
                                       .run_standby = true,
                                       .sampled_in_standby = true,
                                       .prescaler = BodPrescaler::div2})) {
        Supc::clear_flags(SupcFlag::all);
        Supc::arm(SupcFlag::bodvdd_detect);
        Nvic::enable(Supc::irq());
        supc_irqs = 0;
        watchdog_backstop(true);
        bod_sleep = standby_until_wake(window);
        watchdog_backstop(false);
        woke_by_bod = supc_irqs != 0u && !rtc_fired;
        Supc::disarm(SupcFlag::all);
        Nvic::disable(Supc::irq());
    }

    print(serial, "  BODVDD at level ", BodVdd::level_max,
          " (above this supply): READY ", bod_ready ? "1" : "0",
          ", STATUS.BODVDDDET ", detected_awake ? "1" : "0",
          "; a flag cleared under a standing condition came back ",
          flag_returns ? "yes" : "no",
          "; a sampling detector in STANDBY raised none in ",
          watch_us(bod_sleep), " us", crlf);
    bench.verdict("a BODVDD level above the supply is detected while awake, "
                  "with ACTION = none so nothing resets",
                  bod_ok && detected_awake && still_detected);
    bench.verdict("and INTFLAG.BODVDDDET is a TRANSITION, not a level: "
                  "cleared under a standing condition it does not come back, "
                  "not even to a SAMPLING detector",
                  !flag_returns);
    // Which is exactly why the wake claim cannot be made here: a
    // detection needs the SUPPLY to cross the threshold, or the
    // threshold to move - and the CPU that could move it is asleep.
    // Forcing a brown-out is out of this campaign's scope and this
    // bench has no programmable supply. Printed, counted, claiming
    // nothing about the silicon.
    bench.verdict("the BODVDD as a STANDBY WAKE SOURCE: DECLINED - a "
                  "detection is a supply crossing, and nothing on this board "
                  "can make one while the CPU is stopped",
                  true);
    (void)woke_by_bod;

    // The board's own brown-out configuration back, bit for bit.
    (void)BodVdd::enable(false);
    SUPC_REGS->SUPC_BODVDD = boot_bodvdd;
    bench.verdict("the board's boot BODVDD configuration is restored bit for "
                  "bit",
                  BodVdd::reg() == boot_bodvdd);
    quiesce();
}

} // namespace

// ===========================================================================
// p: PM.bus_clock, whose "off" 19.5.2 calls one-way (OUTSIDE z)
// ===========================================================================

namespace {

void tp_bus_clock() {
    quiesce();

    // 19.5.2: CLK_PM_APB "can only be re-enabled by a system reset" - so
    // a device whose PM bus clock is off cannot be told to sleep again
    // until it reboots. THIS LETTER IS OUTSIDE z because that sentence,
    // if true, costs the suite its sleep verbs until the next reset;
    // the letter therefore ends by putting the bit back and SAYING
    // whether that worked.
    const bool on_at_start = Pm::bus_clock();
    bench.verdict("the PM's bus clock is on out of reset, as table 17-1 says",
                  on_at_start);

    const uint8_t before = Pm::sleepcfg();
    Pm::bus_clock(false);
    const bool off_now = !Pm::bus_clock();
    // A read of a peripheral whose APB clock is off: what comes back is
    // the question, and it is asked with a value already known.
    const uint8_t read_while_off = Pm::sleepcfg();
    Pm::bus_clock(true);
    const bool back_on = Pm::bus_clock();
    const uint8_t read_after = Pm::sleepcfg();

    print(serial, "  SLEEPCFG read ", before, " before, ", read_while_off,
          " with CLK_PM_APB off, ", read_after, " after asking for it back",
          crlf);
    bench.verdict("MCLK's mask bit clears, so the request really was made",
                  off_now);
    bench.verdict("and it comes back: on this silicon 19.5.2's one-way "
                  "sentence is about the CLOCK and not about the MASK",
                  back_on);

    // The proof that matters: can the device still be armed and slept?
    const bool armable = Pm::set_sleep_mode(SleepMode::standby);
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    bench.verdict("SLEEPCFG is writable and readable again, so the PM is "
                  "usable without a reset",
                  armable && read_after == before);

    watchdog_backstop(true);
    const bool slept = standby_for(200);
    watchdog_backstop(false);
    bench.verdict("and a real STANDBY still works afterwards", slept);
    quiesce();
}

} // namespace

// ===========================================================================
// Registration, handlers, main
// ===========================================================================

namespace {

void register_tests() {
    bench.letter('a', "the pad walker, and the EIC in standby", ta_eic);
    bench.letter('b', "the RTC as the wake source", tb_rtc);
    bench.letter('c', "FREQM measuring through a standby", tc_freqm);
    bench.letter('d', "the oscillators and the DPLL through a standby", td_clocks);
    bench.letter('e', "the ADC: table 38-4 and a SleepWalking conversion", te_adc);
    bench.letter('f', "the SDADC and the TSENS on their sleep tables",
                 tf_sdadc_tsens);
    bench.letter('g', "the DAC's output buffer through a standby", tg_dac);
    bench.letter('h', "the AC's two sequences of 40.6.14", th_ac);
    bench.letter('i', "CCL 37.6.4: which LUTs decode in standby", ti_ccl);
    bench.letter('j', "the leftovers: the TCC, an EVSYS channel, the BODVDD",
                 tj_leftovers);
    bench.letter('p', "PM.bus_clock, whose off 19.5.2 calls one-way",
                 tp_bus_clock, false);
}

void help() {
    print(serial, "letters:", crlf);
    bench.menu();
}

const char* cause_name(ResetCause c) {
    switch (c) {
    case ResetCause::power_on: return "POR";
    case ResetCause::brown_out_core: return "BODCORE";
    case ResetCause::brown_out_vdd: return "BODVDD";
    case ResetCause::external: return "EXT";
    case ResetCause::watchdog: return "WDT";
    case ResetCause::system_request: return "SYST";
    default: return "none";
    }
}

} // namespace

extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void RTC_Handler() {
    const uint16_t f = brio::Rtc::flags() & brio::Rtc::armed();
    brio::Rtc::clear_flags(f);
    if ((f & brio::RtcFlag::compare0) != 0u) {
        rtc_fired = true;
    }
    rtc_irqs = rtc_irqs + 1u;
    wake_flag = true;
}

extern "C" void EIC_Handler() {
    brio::Eic::clear_flags(brio::Eic::flags());
    eic_irqs = eic_irqs + 1u;
    wake_flag = true;
}

extern "C" void FREQM_Handler() {
    brio::Freqm::clear_flags();
    freqm_irqs = freqm_irqs + 1u;
    wake_flag = true;
}

extern "C" void AC_Handler() {
    brio::Ac::clear_flags(brio::Ac::flags());
    ac_irqs = ac_irqs + 1u;
    wake_flag = true;
}

extern "C" void SUPC_Handler() {
    brio::Supc::clear_flags();
    supc_irqs = supc_irqs + 1u;
    wake_flag = true;
}

extern "C" void WDT_Handler() {
    brio::Watchdog::clear_flags();
}

extern "C" void HardFault_Handler() { brio::hard_fault_reset<P>(0); }

int main() {
    const ResetCause why = brio::Reset::cause();

    SysClock::init();
    Serial::init(clock, 115200);
    (void)brio::Ticker::init(clock);
    brio::enable_interrupts();

    // APBCMASK's reset value is ZERO on this family, so the whole APBC
    // bus - the EVSYS among it - has to be asked for before a single
    // register of it answers.
    brio::Evsys::bus_clock(true);

    bool rulers = watch_up();
    rulers = rulers && brio::Gclk<gen_ulp>::configure(brio::GclkConfig{
                           .source = brio::GclkSource::osculp32k, .run_standby = true});
    rulers = rulers && brio::Gclk<gen_eic>::configure(brio::GclkConfig{
                           .source = brio::GclkSource::osculp32k, .run_standby = true});
    rulers = rulers && rtc_up();

    // The RTC's own rate, against the crystal: every microsecond printed
    // below traces back to the board's 24 MHz crystal through this one
    // measurement.
    if (rulers) {
        const uint32_t want = 4096u;
        const uint32_t r0 = rtc_now();
        const uint32_t w0 = watch_now();
        while (rtc_now() - r0 < want) {
        }
        const uint32_t wd = watch_now() - w0;
        if (wd != 0u) {
            ulp_hz = static_cast<uint32_t>(
                (static_cast<uint64_t>(want) * crystal_hz) / wd);
        }
    }

    print(serial, crlf,
          "test_samc_sleepwalk - what the peripherals do while the CPU sleeps "
          "(reset ", cause_name(why), ", CPU ", SysClock::hz / 1'000'000u, " MHz)",
          crlf);
    print(serial, "  ruler: a TC pair on the 24 MHz crystal, RUNSTDBY, counting "
                  "through every standby", crlf);
    print(serial, "  the RTC on OSCULP32K measures ", ulp_hz,
          " Hz here; OSC48M is ", osc48m_hz / 1000u,
          " kHz (docs/samc/clock.md). KERNEL TIME STOPS IN STANDBY", crlf);
    if (!rulers) {
        print(serial, "  WARNING: an instrument did not come up", crlf);
    }

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
