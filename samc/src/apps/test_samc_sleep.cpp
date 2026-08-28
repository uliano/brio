// test_samc_sleep - the POWER-MANAGEMENT test SUITE for the SAM C21:
// samc/sleep.hpp (PM, ch. 19 - SLEEPCFG, STDBYCFG and the two errata
// that make standby entry a discipline), the `SamSleepSite` that puts
// util/power.hpp's depth ladder on it, and the kernel half that makes
// the whole model work with no new hook - SamPlatform::idle() taking
// whatever PM.SLEEPCFG holds, and TimeEvents::ticks_to_next().
//
// Reference test of those (docs/design/power.md, docs/samc/platform.md):
// keep it passing.
//
// NOTHING TO WIRE. Every instrument is on the die.
//
// THREE RULERS, because a sleep that stops the CPU clock cannot be timed
// by anything the CPU clock feeds:
//   - the RTC on OSCULP32K DEFINES EVERY SLEEP AND IS THE WAKE SOURCE.
//     Its clock comes from OSC32KCTRL and not from a GCLK generator,
//     which is exactly what table 19-4's note 1 excludes from
//     "SleepWalking" - so it is the one through-standby counter here
//     that does not itself change what a standby costs.
//   - a TC0+TC1 pair at CLK_MAIN is the AWAKE ruler, for intervals that
//     begin after a wake (a clock's restart, a vote round). It is FROZEN
//     in standby, which letter c proves rather than assumes.
//   - a TC2+TC3 pair on the board's 24 MHz CRYSTAL with RUNSTDBY set is
//     the FINE STANDBY ruler, 42 ns a tick, and it is the only thing
//     here that can time a single wake. Letter c builds it one RUNSTDBY
//     bit at a time before anything measured with it is believed.
// Two of the three are calibrated against the third: `fine_hz` is
// OSC48M's own measured rate from docs/samc/clock.md (47.755 MHz, 5100
// ppm SLOW, and that number is crystal-referenced), and the RTC's rate
// is measured against it at boot and printed - so every microsecond
// below traces back to the board's crystal through one cited constant.
//
// WHAT THE WAKE NUMBERS MEAN, and what they cost. Each is the mean of 64
// SINGLE wakes on the crystal ruler: "read the counter, sleep, wake,
// read again", against the identical loop with the sleep replaced by a
// POLL. The RTC compare that ends the wait is common to both, so the
// difference is what the sleep path cost. THE OBVIOUS METHOD DOES NOT
// WORK and the file says so where it is defined: differencing N rounds
// of arm/sleep/wake on the RTC alone measures nothing, because the loop
// LOCKS to the RTC and a sub-tick overhead is quantized away. THE PRICE
// of the method that does work is that a crystal, a generator and a TC
// all running through the standby ARE table 19-4's "SleepWalking", so
// AUTO holds the main regulator throughout: every wake figure here is
// the bill WITH THE SUPPLY ALREADY UP, and letter e is where that shows.
//
// UNLIKE THE OTHER SUITES IN THIS STRATUM, LETTERS g AND h RUN THE
// KERNEL: the object under test there is an active object, so the rounds
// go through real queues, real dispatch and the real Kernel pack (Probe,
// Bus, Pm) - only the LOOP is the suite's.
//
// THE ANTI-WEDGE RULE. A board asleep with its console SERCOM stopped
// prints nothing, so a wake that never arrives would be silence for
// ever. Every sleeping letter therefore arms the WATCHDOG first (it runs
// on OSCULP32K and so survives standby, 23.5.2) and disables it at the
// end: a lost wake costs a reboot and a banner, never a mute board.
// Nothing here prints between arming a sleep and coming back from it.
//
// What is exercised, letter by letter:
//   a  the register surface: the three implemented modes, the Reserved
//      codes, the readback rule TIMED, STDBYCFG's real width, and the
//      ladder mapping this target had to choose
//   b  IDLE0 and IDLE2: the CPU stops and everything else does not
//   c  THE INSTRUMENT LETTER - which RUNSTDBY bit makes a TC count
//      through a standby, proven before anything is believed
//   d  STANDBY: the CPU stops, THE KERNEL TICK FREEZES, and what the
//      wake costs
//   e  the regulator and the RAM back-bias, six combinations measured
//      against a seventh that repeats the first as the noise floor
//   f  what survives a standby: OSC48M, XOSC and the DPLL - and what a
//      sleepwalking task holds up on their behalf
//   g  the manager, awake: the ladder, the voters, the standing
//      restrictions and the deadline guard
//   h  the manager asleep: a real standby round with an RTC wake, the
//      first-event-after-wake contract, and THE TICK RULE
//
// NOTE for anyone adding a letter: a printed line must NEVER contain the
// two characters "->", because tools/bench.py looks for that arrow to
// find a letter's tally line and truncates the capture on a stray one.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/kernel.hpp"
#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/osc32kctrl.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"
#include "samc/rtc.hpp"
#include "samc/sercom.hpp"
#include "samc/sleep.hpp"
#include "samc/supc.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "util/bus_master.hpp"
#include "util/power.hpp"
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
// The rulers
// ---------------------------------------------------------------------------

/// The awake stopwatch: TC0 + TC1 as one 32-bit counter on generator 0,
/// which is CLK_MAIN undivided. It STOPS in standby (no RUNSTDBY
/// anywhere in its chain) and that is deliberate - letter c proves it.
using Fine = Tc<0>;

/// The TC2 + TC3 pair, in two roles. In letter c it is the SUBJECT: a
/// candidate through-standby counter whose whole RUNSTDBY chain is
/// moved one bit at a time. From letter d on it is the INSTRUMENT that
/// experiment earns - the fine ruler that keeps running while the CPU
/// is stopped, on generator 2 sourced from the board's 24 MHz crystal.
using SlowWatch = Tc<2>;
constexpr uint8_t gen_ulp = 3;     ///< letter c's own generator
constexpr uint8_t gen_xtal = 2;    ///< the crystal, for the fine standby ruler
constexpr uint32_t crystal_hz = 24'000'000UL;

/// OSC48M's own rate, MEASURED and crystal-referenced in
/// docs/samc/clock.md: 47.755 MHz, i.e. 5100 ppm below its nominal
/// 48 MHz and inside its own +-5% specification. Every "fine" duration
/// below is computed at this rate rather than at the nominal one.
constexpr uint32_t fine_hz = 47'755'000UL;

/// The RTC's source rate, measured against `fine_hz` at boot.
uint32_t ulp_hz = 32'768UL;

uint32_t fine_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) / fine_hz);
}
uint32_t fine_ns(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000'000ULL) /
                                 fine_hz);
}
uint32_t rtc_ms(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000ULL) / ulp_hz);
}

uint32_t fine_now() { return Fine::count32(); }

// ---------------------------------------------------------------------------
// Shared with the handlers
// ---------------------------------------------------------------------------

volatile bool rtc_fired = false;
volatile uint32_t rtc_irqs = 0;
volatile bool post_on_rtc = false;

volatile bool wdt_warned = false;

/// A hard cap on every "wait for the wake" spin: a wake that never
/// arrives must fail a verdict, not hang the bench. (The watchdog is the
/// backstop for the case where the CPU never comes back at all.)
constexpr uint32_t spin_cap = 40'000'000UL;

// ---------------------------------------------------------------------------
// The system under test, for letters g and h
// ---------------------------------------------------------------------------

/// A bus engine that never touches a wire: start() always goes
/// asynchronous, so the transfer stays in flight until the test posts
/// its completion. That is exactly the state a deep sleep must not be
/// allowed into.
struct FakeBus {
    struct Request {
        uint8_t tag;
        ReplyTo<BusDone> reply;
    };
    static inline uint8_t started = 0;
    static bool start(const Request&) {
        ++started;
        return false;
    }
};

using Bus = BusMaster<FakeBus, P>;

/// The payload of the time event the deadline guard looks at, and of the
/// wake the RTC posts in letter h.
struct Blip {};

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline bool accept = true;
    static inline uint8_t votes = 0;
    static inline bool last_ok = false;
    static inline uint8_t asked = 0;
    static inline SleepDepth asked_depth = SleepDepth::none;
    static inline uint8_t wakes = 0;
    static inline SleepDepth woke_from = SleepDepth::none;
    static inline uint16_t blips = 0;

    static void init() { start(&only); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](SleepVote v) {
                ++votes;
                last_ok = v.ok;
                return handled();
            },
            [](const PrepareSleep& p) {
                ++asked;
                asked_depth = p.depth;
                p.reply.send(SleepVote{accept});
                return handled();
            },
            [](WakeReport w) {
                ++wakes;
                woke_from = w.was;
                return handled();
            },
            [](Blip) {
                ++blips;
                return handled();
            });
    }
};

using Pm_ = PowerManager<P, SamSleepSite, PowerConfig{}, Bus, Probe>;
using K = Kernel<P, Probe, Bus, Pm_>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

const char* cause_name(ResetCause c) {
    switch (c) {
    case ResetCause::unknown: return "unknown";
    case ResetCause::power_on: return "POR";
    case ResetCause::brown_out_core: return "BODCORE";
    case ResetCause::brown_out_vdd: return "BODVDD";
    case ResetCause::external: return "EXT";
    case ResetCause::watchdog: return "WDT";
    case ResetCause::system_request: return "SYST";
    }
    return "?";
}

const char* depth_name(SleepDepth d) {
    switch (d) {
    case SleepDepth::none: return "none";
    case SleepDepth::light: return "light";
    case SleepDepth::standby: return "standby";
    case SleepDepth::deep: return "deep";
    }
    return "?";
}

const char* mode_name(SleepMode m) {
    switch (m) {
    case SleepMode::idle0: return "IDLE0";
    case SleepMode::idle2: return "IDLE2";
    case SleepMode::standby: return "STANDBY";
    }
    return "?";
}

/// Wait until the last byte has left the shifter. NOTHING may be printed
/// between here and a wake: the console's SERCOM has no RUNSTDBY, so a
/// standby stops it mid-character.
void console_drain() {
    uint32_t spins = spin_cap;
    while (!Serial::tx_idle() && spins-- != 0u) {
    }
    spins = spin_cap;
    while (!Serial::Resource::txc_flag() && spins-- != 0u) {
    }
}

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

/// The anti-wedge backstop. 4096 cycles of the WDT's 1.024 kHz clock is
/// about four seconds - longer than any leg here and shorter than
/// bench.py's timeout, so a lost wake reboots the board into its banner
/// instead of leaving it mute.
void watchdog_backstop(bool on) {
    if (on) {
        (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc4096});
    } else {
        (void)Watchdog::disable();
    }
}

// ---- the RTC: the standby ruler and the wake source ------------------------

bool rtc_up() {
    if (!Rtc::init()) {
        return false;
    }
    (void)Rtc::enable(false);
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    if (!Rtc::init()) {
        return false;
    }
    if (!Rtc::configure(RtcConfig{.mode = RtcMode::count32,
                                  .prescaler = RtcPrescaler::div1})) {
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

uint32_t rtc_now() { return Rtc::count32(); }

/// Arm COMP0 `p` ticks ahead of now and report whether the compare is
/// still in the future when the arming is done - i.e. whether it is safe
/// to sleep on it.
bool arm_wake(uint32_t p) {
    const uint32_t c = rtc_now();
    rtc_fired = false;
    Rtc::clear_flags(RtcFlag::compare0);
    if (!Rtc::set_comp32(c + p)) {
        return false;
    }
    return static_cast<int32_t>(rtc_now() - (c + p)) < 0;
}

/// Sleep at the mode already armed, race-free: PRIMASK is set, the wake
/// condition is re-tested, WFI still wakes on a pending interrupt, and
/// unmasking afterwards is what lets the handler run.
void sleep_once() {
    __disable_irq();
    if (!rtc_fired) {
        Pm::sleep();
    }
    __enable_irq();
}

bool wait_wake() {
    uint32_t spins = spin_cap;
    while (!rtc_fired && spins-- != 0u) {
    }
    return rtc_fired;
}

// ---- the fine standby ruler -------------------------------------------------
//
// WHY THE RTC CANNOT MEASURE A WAKE, and what replaces it. The obvious
// method - run N rounds of "arm the compare P ticks ahead, sleep, wake"
// in IDLE and in STANDBY and difference the totals - was built first and
// MEASURES NOTHING: 512 rounds took 10240 RTC ticks in both modes, to
// the tick. The loop LOCKS to the RTC, so a per-round overhead smaller
// than one 30 us tick is quantized away instead of accumulating. What
// the null result does say, and it is worth keeping: at the RTC's own
// granularity a standby round is indistinguishable from an idle one.
//
// So the wake is timed SINGLE-SHOT on a counter that keeps running while
// the CPU is stopped - the TC2+TC3 pair on the board's 24 MHz crystal,
// exactly the instrument letter c proved can exist. Each measurement is
// "read the counter, sleep, wake, read again", and the same measurement
// with the sleep replaced by a POLL is the baseline; the RTC compare
// that ends it is common to both, so the difference is what the sleep
// path cost. 42 ns a tick, averaged over 64 rounds.
//
// WHAT THAT INSTRUMENT COSTS, said out loud: a crystal, a generator and
// a TC all running through the standby ARE the "SleepWalking" of table
// 19-4, so in AUTO the main voltage regulator is used throughout. The
// numbers below are therefore the wake bill WITH THE SUPPLY ALREADY UP -
// which is the honest reading of them, and which is also why letter e
// can see the regulator's own contribution only by moving VREGSMOD away
// from AUTO.

using Deep = SlowWatch;

/// Bring the crystal, its generator and the pair up. With
/// `hold_in_standby` the pair keeps running through a standby and is the
/// fine ruler; without it the pair stops there and makes NO request, so
/// it becomes a passive PROBE of whether the crystal itself survived.
bool deep_build(bool hold_in_standby) {
    if (!Xosc::init(XoscConfig{.hz = crystal_hz,
                               .startup = 4,
                               .on_demand = false,
                               .run_standby = true})) {
        return false;
    }
    if (!Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::xosc})) {
        return false;
    }
    (void)Deep::enable(false);
    if (!Deep::init(gen_xtal)) {
        return false;
    }
    if (!Deep::configure(TcConfig{.mode = TcMode::count32,
                                  .prescaler = TcPrescaler::div1,
                                  .run_standby = hold_in_standby})) {
        return false;
    }
    return Deep::enable(true);
}

bool deep_up() { return deep_build(true); }

/// Is the crystal ISSUING A CLOCK right now? Asked of its own counter
/// and not of a status flag: two synchronized reads a known CLK_MAIN
/// interval apart, and whether the count moved. The bound on each read
/// is short on purpose - a synchronization that cannot complete because
/// the clock is dead must cost microseconds, not milliseconds.
bool crystal_ticking() {
    const uint32_t a = Deep::count32(256);
    const uint32_t f0 = fine_now();
    while (fine_now() - f0 < fine_hz / 40'000u) {   // ~25 us
    }
    return Deep::count32(256) != a;
}

/// TWO reads, and the first is thrown away - and across a standby that
/// is not a nicety but the whole correctness of the reading. See
/// slow_count() above for the measurement behind it: a Tc::count32()
/// returns the value the PREVIOUS read_sync latched, so a single read
/// taken after a wake would answer with the count from BEFORE the sleep.
uint32_t deep_now() {
    (void)Deep::count32();
    return Deep::count32();
}

/// Hand the fine ruler back: the pair stopped and released, then its
/// generator disabled. ORDER MATTERS - a generator may not be left
/// pointing at a source that is about to stop (16.6.2.6).
void deep_down() {
    (void)Deep::enable(false);
    Deep::release();
    (void)Gclk<gen_xtal>::enable(false);
}

uint32_t deep_ns(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000'000ULL) /
                                 crystal_hz);
}

enum class WaitKind : uint8_t { poll, sleep };

/// The mean crystal time from just before the wait to just after the
/// wake, over `n` rounds whose RTC compare is `p` ticks ahead each time.
/// Zero means a round was skipped or a wake never arrived.
uint32_t deep_leg(WaitKind kind, SleepMode mode, uint32_t p, uint16_t n) {
    if (kind == WaitKind::sleep && !Pm::set_sleep_mode(mode)) {
        return 0;
    }
    uint64_t sum = 0;
    for (uint16_t i = 0; i < n; ++i) {
        if (!arm_wake(p)) {
            return 0;
        }
        const uint32_t t0 = deep_now();
        if (kind == WaitKind::sleep) {
            sleep_once();
        }
        if (!wait_wake()) {
            return 0;
        }
        sum += deep_now() - t0;
    }
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    return static_cast<uint32_t>(sum / n);
}

/// The extra time a sleeping wait costs over a polled one, in
/// nanoseconds - the wake bill, signed so a cheaper leg is visible.
int32_t bill_ns(uint32_t leg, uint32_t baseline) {
    const bool up = leg >= baseline;
    const uint32_t ns = deep_ns(up ? leg - baseline : baseline - leg);
    return up ? static_cast<int32_t>(ns) : -static_cast<int32_t>(ns);
}

constexpr uint32_t bill_p = 32;     ///< RTC ticks per round (~970 us)
constexpr uint16_t bill_n = 64;    ///< rounds averaged per leg

/// The state every letter starts from.
void quiesce() {
    __disable_irq();
    Probe::deadline.disarm();
    TimeEvents<P>::clear_all();
    while (Probe::queue.pop().has_value()) {
    }
    while (Bus::queue.pop().has_value()) {
    }
    while (Pm_::queue.pop().has_value()) {
    }
    Probe::accept = true;
    Probe::votes = Probe::asked = Probe::wakes = 0;
    Probe::last_ok = false;
    Probe::asked_depth = SleepDepth::none;
    Probe::woke_from = SleepDepth::none;
    Probe::blips = 0;
    FakeBus::started = 0;
    post_on_rtc = false;
    rtc_fired = false;
    __enable_irq();
    K::init_all();                    // Pm_::init() disarms the site
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    (void)Pm::configure_standby(StandbyConfig{});   // back to the reset value
    Vreg::run_standby(false);
    watchdog_backstop(false);
}

// ---------------------------------------------------------------------------
// a: the register surface
// ---------------------------------------------------------------------------

uint8_t boot_sleepcfg = 0xFF;
uint16_t boot_stdbycfg = 0xFFFF;

void ta_registers() {
    quiesce();

    print(serial, "  at boot SLEEPCFG=", hex(boot_sleepcfg), " STDBYCFG=",
          hex(boot_stdbycfg), crlf);
    bench.verdict("SLEEPCFG comes up at IDLE0, its reset value",
                  boot_sleepcfg == PM_SLEEPCFG_SLEEPMODE_IDLE0_Val);
    // 19.8.2 reset 0x0400: BBIASHS SET, VREGSMOD AUTO. Both halves of
    // erratum 1.8.13's precondition are therefore the DEFAULT state of a
    // program on this part.
    bench.verdict("STDBYCFG comes up at 0x0400: back-bias ON, regulator AUTO",
                  boot_stdbycfg == 0x0400u);
    bench.verdict("PM's bus clock is on out of reset", Pm::bus_clock());

    // The three implemented codes arm and read back.
    bool arm_ok = true;
    uint8_t regs_seen[3] = {0xFF, 0xFF, 0xFF};
    const SleepMode modes[3] = {SleepMode::idle0, SleepMode::idle2, SleepMode::standby};
    for (uint8_t i = 0; i < 3u; ++i) {
        // Start from a DIFFERENT value so the readback has something to
        // wait for.
        (void)Pm::set_sleep_mode(modes[i] == SleepMode::idle0 ? SleepMode::standby
                                                              : SleepMode::idle0);
        arm_ok = Pm::set_sleep_mode(modes[i]) && arm_ok;
        regs_seen[i] = Pm::sleepcfg();
    }
    for (uint8_t i = 0; i < 3u; ++i) {
        print(serial, "  ", mode_name(modes[i]), ": SLEEPCFG=", hex(regs_seen[i]), crlf);
    }
    bench.verdict("all three implemented modes arm and read back",
                  arm_ok && regs_seen[0] == 0x0u && regs_seen[1] == 0x2u &&
                      regs_seen[2] == 0x4u);

    // 19.6.3.3 warns of "a small latency ... due to bridges" and gives no
    // figure. THIS IS THE FIGURE, and it is not small on a 48 MHz CPU.
    // The awake ruler's own cost (reading COUNT is a READSYNC command,
    // 35.6.8) is measured first and subtracted; the arming is then
    // averaged over sixteen alternating writes so the remainder of the
    // ruler is amortized rather than counted.
    const uint32_t z0 = fine_now();
    const uint32_t z1 = fine_now();
    const uint32_t ruler_ns = fine_ns(z1 - z0);
    const uint32_t a0 = fine_now();
    for (uint8_t i = 0; i < 16u; ++i) {
        (void)Pm::set_sleep_mode((i & 1u) != 0u ? SleepMode::standby : SleepMode::idle0);
    }
    const uint32_t a1 = fine_now();
    const uint32_t arm_ns = fine_ns(a1 - a0) / 16u;
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    print(serial, "  one armed mode, readback included, costs ", arm_ns,
          " ns (the ruler's own read costs ", ruler_ns, " ns and is not in that)",
          crlf);
    // Wide on purpose: what is being claimed is an ORDER OF MAGNITUDE -
    // that the bridge latency is microseconds and not the handful of
    // cycles a plain APB store would take, so a WFI issued straight
    // after the store would sleep in the mode that was there before.
    bench.verdict("the bridge latency the chapter warns of is MICROSECONDS, not "
                  "cycles - the readback rule is not a formality",
                  arm_ns > 1000u && arm_ns < 100'000u);

    bench.verdict("the Reserved SLEEPMODE codes are refused",
                  !Pm::set_sleep_mode(static_cast<SleepMode>(1)) &&
                      !Pm::set_sleep_mode(static_cast<SleepMode>(3)) &&
                      !Pm::set_sleep_mode(static_cast<SleepMode>(5)) &&
                      !Pm::set_sleep_mode(static_cast<SleepMode>(7)));
    bench.verdict("and a refusal writes nothing",
                  Pm::sleep_mode() == SleepMode::idle0);

    // STDBYCFG: the two fields, and the width the register really has.
    const bool r_auto = Pm::configure_standby(
                            StandbyConfig{.regulator = VregStandbyMode::automatic}) &&
                        Pm::regulator_mode() == VregStandbyMode::automatic;
    const bool r_perf = Pm::configure_standby(
                            StandbyConfig{.regulator = VregStandbyMode::performance}) &&
                        Pm::regulator_mode() == VregStandbyMode::performance;
    const bool r_lp =
        Pm::configure_standby(StandbyConfig{.regulator = VregStandbyMode::low_power}) &&
        Pm::regulator_mode() == VregStandbyMode::low_power;
    bench.verdict("the three VREGSMOD modes read back", r_auto && r_perf && r_lp);
    bench.verdict("the Reserved fourth is refused, and writes nothing",
                  !Pm::configure_standby(
                      StandbyConfig{.regulator = static_cast<VregStandbyMode>(3)}) &&
                      Pm::regulator_mode() == VregStandbyMode::low_power);

    const bool bb_on = Pm::configure_standby(StandbyConfig{.back_bias = true}) &&
                       Pm::back_bias();
    const bool bb_off = Pm::configure_standby(StandbyConfig{.back_bias = false}) &&
                        !Pm::back_bias();
    bench.verdict("BBIASHS reads back both ways", bb_on && bb_off);

    // Everything outside VREGSMOD and BBIASHS is Reserved; the device
    // header says the register mask is 0x04C0 and the silicon is asked
    // whether it agrees.
    PM_REGS->PM_STDBYCFG = 0xFFFFu;
    const uint16_t wide = Pm::stdbycfg();
    (void)Pm::configure_standby(StandbyConfig{});
    print(serial, "  STDBYCFG after storing 0xFFFF reads ", hex(wide),
          " (header mask ", hex(static_cast<uint16_t>(PM_STDBYCFG_Msk)), ")", crlf);
    bench.verdict("STDBYCFG is three bits wide, whatever is stored into it",
                  wide == (0xFFFFu & PM_STDBYCFG_Msk));
    bench.verdict("and the reset value is restored", Pm::stdbycfg() == 0x0400u);

    // The ladder mapping: this target's own decision, and the first one
    // in brio where the never-deeper rule is not the identity.
    struct Rung {
        SleepDepth asked;
        bool ok;
        uint8_t reg;
        SleepDepth back;
    };
    Rung rungs[4];
    const SleepDepth ladder[4] = {SleepDepth::none, SleepDepth::light,
                                  SleepDepth::standby, SleepDepth::deep};
    for (uint8_t i = 0; i < 4u; ++i) {
        const bool ok = SamSleepSite::arm(ladder[i]);
        rungs[i] = Rung{ladder[i], ok, Pm::sleepcfg(), SamSleepSite::armed()};
    }
    SamSleepSite::disarm();

    for (const Rung& r : rungs) {
        print(serial, "  ", depth_name(r.asked), ": SLEEPCFG=", hex(r.reg), " site=",
              depth_name(r.back), r.ok ? " (armed)" : " (refused)", crlf);
    }
    bench.verdict("none is IDLE0, the reset mode",
                  rungs[0].ok && rungs[0].reg == 0x0u &&
                      rungs[0].back == SleepDepth::none);
    bench.verdict("light is IDLE2, the deepest idle this family has",
                  rungs[1].ok && rungs[1].reg == 0x2u &&
                      rungs[1].back == SleepDepth::light);
    bench.verdict("standby is STANDBY",
                  rungs[2].ok && rungs[2].reg == 0x4u &&
                      rungs[2].back == SleepDepth::standby);
    // THE RULE THAT DOES NOT APPLY ON AVR: this family has no rung below
    // standby, so `deep` maps DOWN and armed() says so.
    bench.verdict("deep maps to standby - never deeper than asked - and reads "
                  "back as standby",
                  rungs[3].ok && rungs[3].reg == 0x4u &&
                      rungs[3].back == SleepDepth::standby);
    bench.verdict("disarm puts the reset mode back",
                  Pm::sleep_mode() == SleepMode::idle0 &&
                      SamSleepSite::armed() == SleepDepth::none);

    quiesce();
}

// ---------------------------------------------------------------------------
// b: IDLE
// ---------------------------------------------------------------------------

/// Turns of a loop that only advances while the CPU runs, over `ticks`
/// RTC ticks. `stop` takes the armed mode; the other leg just spins.
uint32_t idle_turns(bool stop, uint32_t ticks) {
    volatile uint32_t turns = 0;
    const uint32_t t0 = rtc_now();
    while (rtc_now() - t0 < ticks) {
        if (stop) {
            __disable_irq();
            Pm::sleep();     // the SysTick tick is the wake
            __enable_irq();
        }
        turns = turns + 1;
    }
    return turns;
}

void tb_idle() {
    quiesce();
    watchdog_backstop(true);

    // IDLE's wake source here is the kernel tick itself: SysTick keeps
    // running in idle, which is half of what this letter is about.
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    console_drain();
    const uint32_t ticks_window = ulp_hz / 4u;      // a quarter second
    const uint32_t asleep = idle_turns(true, ticks_window);
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    const uint32_t awake = idle_turns(false, ticks_window);

    print(serial, "  over ", rtc_ms(ticks_window), " ms: ", awake,
          " loop turns spinning, ", asleep, " sleeping in IDLE0 (one per wake)", crlf);
    bench.verdict("the CPU really stops in IDLE0", asleep < awake / 20u);
    // 250 ms at 1000 ticks/s is 250 wakes; the loop turns once per wake.
    bench.verdict("and it wakes once per SysTick tick",
                  near(asleep, Ticker::ticks_per_second / 4u,
                       Ticker::ticks_per_second / 20u));

    // What keeps running in IDLE: SysTick (the CPU clock is alive) and a
    // TC on generator 0 with no RUNSTDBY anywhere.
    console_drain();
    if (!arm_wake(ulp_hz / 8u)) {
        bench.verdict("the wake could be armed", false);
    } else {
        (void)Pm::set_sleep_mode(SleepMode::idle0);
        const uint32_t k0 = Ticker::ticks();
        const uint32_t f0 = fine_now();
        const uint32_t r0 = rtc_now();
        sleep_once();
        const bool woke = wait_wake();
        const uint32_t k1 = Ticker::ticks();
        const uint32_t f1 = fine_now();
        const uint32_t r1 = rtc_now();
        const uint32_t slept_ms = rtc_ms(r1 - r0);
        print(serial, "  an IDLE0 sleep of ", slept_ms, " ms: the tick advanced ",
              k1 - k0, " ms and the CLK_MAIN stopwatch ", fine_us(f1 - f0), " us", crlf);
        bench.verdict("one RTC compare is a wake source from IDLE0", woke);
        bench.verdict("SysTick keeps counting through IDLE - the kernel tick is "
                      "not late",
                      near(k1 - k0, slept_ms, slept_ms / 20u + 2u));
        bench.verdict("and so does a TC on generator 0, with no RUNSTDBY anywhere",
                      near(fine_us(f1 - f0) / 1000u, slept_ms, slept_ms / 20u + 2u));
    }

    // IDLE2 is IDLE0 minus the CAN clock, and there is no CAN here: what
    // is checkable is that it sleeps, wakes, and costs the same.
    console_drain();
    const bool ruler = deep_up();
    const uint32_t polled = deep_leg(WaitKind::poll, SleepMode::idle0, bill_p, bill_n);
    const uint32_t l_idle0 = deep_leg(WaitKind::sleep, SleepMode::idle0, bill_p, bill_n);
    const uint32_t l_idle2 = deep_leg(WaitKind::sleep, SleepMode::idle2, bill_p, bill_n);
    const int32_t b0 = bill_ns(l_idle0, polled);
    const int32_t b2 = bill_ns(l_idle2, polled);
    print(serial, "  against a POLLED wait on the same compare, waking from "
          "IDLE0 cost ", b0, " ns and from IDLE2 ", b2, " ns (mean of ", bill_n,
          ")", crlf);
    bench.verdict("the crystal-fed ruler and all three legs ran",
                  ruler && polled != 0u && l_idle0 != 0u && l_idle2 != 0u);
    bench.verdict("waking from IDLE0 costs NOTHING measurable over a polled "
                  "wait - a WFI and an exception entry, and the clocks never "
                  "stopped",
                  b0 > -2000 && b0 < 2000);
    // A FINDING THE CHAPTER DOES NOT HAVE. 19.6.3.3.1 presents IDLE2 as
    // IDLE0 with the CAN clock gated and nothing else - and there is no
    // CAN traffic on this board at all - yet leaving it costs several
    // microseconds more, every time, over three runs.
    print(serial, "  IDLE2 therefore costs ", b2 - b0,
          " ns more to leave than IDLE0, on a board with no CAN traffic", crlf);
    bench.verdict("IDLE2 IS NOT FREE: gating one more clock domain costs "
                  "microseconds at the wake, which chapter 19 does not say",
                  b2 - b0 > 1500 && b2 - b0 < 20000);

    watchdog_backstop(false);
    quiesce();
}

// ---------------------------------------------------------------------------
// c: the instrument letter
// ---------------------------------------------------------------------------

/// Rebuild generator `gen_ulp` on `src` and the TC2+TC3 pair on it,
/// with each of the two RUNSTDBY bits set as asked.
bool slow_watch_up(GclkSource src, bool gen_standby, bool tc_standby) {
    (void)SlowWatch::enable(false);
    if (!Gclk<gen_ulp>::configure(
            GclkConfig{.source = src, .run_standby = gen_standby})) {
        return false;
    }
    if (!SlowWatch::init(gen_ulp)) {
        return false;
    }
    if (!SlowWatch::configure(TcConfig{.mode = TcMode::count32,
                                       .prescaler = TcPrescaler::div1,
                                       .run_standby = tc_standby})) {
        return false;
    }
    return SlowWatch::enable(true);
}

/// TWO reads of the pair, and the first is thrown away.
///
/// MEASURED HERE, and it is a fact about samc/tc.hpp rather than about
/// sleep: the FIRST `Tc::count32()` after a counter is started returns
/// the readable shadow's PREVIOUS content - zero - because READSYNC's
/// result lands after `read_sync()`'s wait has already returned. Four
/// consecutive reads of a pair that had been running for six
/// milliseconds gave 0, 196, 201, 205. From the second read on the lag
/// is one read-duration and constant, so differences are honest; a
/// single first read is not, and would have credited this letter's whole
/// awake set-up to the standby it is trying to measure.
uint32_t slow_count() {
    (void)SlowWatch::count32();
    return SlowWatch::count32();
}

void tc_instrument() {
    quiesce();
    watchdog_backstop(true);

    // OSC32K is the other 32 kHz root, and unlike OSCULP32K it HAS a
    // RUNSTDBY bit - which is the whole question this letter turns out
    // to be about. Trimmed, because an untrimmed OSC32K runs 44 % fast
    // (docs/samc/osc32kctrl.md) and would make every count below a lie.
    const bool osc32k_up = Osc32k::init(Osc32kConfig{.calib = Osc32k::factory_calib(),
                                                     .enable_32k = true,
                                                     .run_standby = true});
    bench.verdict("OSC32K started, trimmed from the factory calibration",
                  osc32k_up);

    struct Leg {
        const char* what;
        GclkSource src;
        bool src_standby;   ///< only meaningful for OSC32K
        bool gen;
        bool tc;
        uint32_t counted;
        uint32_t rtc;
        uint32_t fine;
    };
    Leg legs[] = {
        {"OSCULP32K (no RUNSTDBY bit exists), gen 1, TC 1", GclkSource::osculp32k, false,
         true, true, 0, 0, 0},
        {"OSC32K RUNSTDBY 1,                  gen 1, TC 1", GclkSource::osc32k, true,
         true, true, 0, 0, 0},
        {"OSC32K RUNSTDBY 1,                  gen 0, TC 1", GclkSource::osc32k, true,
         false, true, 0, 0, 0},
        {"OSC32K RUNSTDBY 1,                  gen 1, TC 0", GclkSource::osc32k, true,
         true, false, 0, 0, 0},
        {"OSC32K RUNSTDBY 0,                  gen 1, TC 1", GclkSource::osc32k, false,
         true, true, 0, 0, 0},
        {"OSCULP32K again, last                gen 1, TC 1", GclkSource::osculp32k, false,
         true, true, 0, 0, 0},
    };
    bool built = osc32k_up;
    const uint32_t window = ulp_hz / 32u;    // ~31 ms of standby

    for (Leg& l : legs) {
        if (!built) {
            break;
        }
        if (l.src == GclkSource::osc32k) {
            built = Osc32k::init(Osc32kConfig{.calib = Osc32k::factory_calib(),
                                              .enable_32k = true,
                                              .run_standby = l.src_standby});
        }
        built = built && slow_watch_up(l.src, l.gen, l.tc);
        if (!built) {
            break;
        }
        (void)Pm::set_sleep_mode(SleepMode::standby);
        console_drain();
        if (!arm_wake(window)) {
            built = false;
            break;
        }
        const uint32_t s0 = slow_count();
        const uint32_t f0 = fine_now();
        const uint32_t r0 = rtc_now();
        sleep_once();
        if (!wait_wake()) {
            built = false;
            break;
        }
        l.counted = slow_count() - s0;
        l.fine = fine_now() - f0;
        l.rtc = rtc_now() - r0;
        (void)Pm::set_sleep_mode(SleepMode::idle0);
    }
    bench.verdict("all five configurations could be built and slept", built);

    if (built) {
        for (const Leg& l : legs) {
            print(serial, "  ", l.what, ": counted ", l.counted, " of ", l.rtc,
                  " RTC ticks, awake ", fine_us(l.fine), " us", crlf);
        }
        // THE FINDING, and it is the same one the AVR pass made about a
        // different silicon: THE PERIPHERAL'S OWN RUNSTDBY IS THE WHOLE
        // REQUEST. 19.6.3.3.2 calls the mechanism SleepWalking and
        // describes it from the top down ("a peripheral can run during
        // standby and request its GCLK asynchronous clock, which will
        // wake up the related GCLK and clock source"); what the four
        // legs below add is which of the three RUNSTDBY bits in that
        // chain a program actually has to set. One. The peripheral's.
        bench.verdict("a TC WITHOUT its own RUNSTDBY counts nothing through a "
                      "standby, whatever the generator and the source say",
                      legs[3].counted < 32u);
        bench.verdict("a TC WITH it counts the whole standby",
                      near(legs[1].counted, legs[1].rtc, legs[1].rtc / 16u + 8u));
        bench.verdict("the GENERATOR's own RUNSTDBY is not needed: the "
                      "peripheral's request wakes the generator it asks for",
                      near(legs[2].counted, legs[2].rtc, legs[2].rtc / 16u + 8u));
        bench.verdict("nor is the SOURCE's: the same request reaches all the way "
                      "up and restarts OSC32K itself",
                      near(legs[4].counted, legs[4].rtc, legs[4].rtc / 16u + 8u));
        // And the root that has no such bit to set is no exception -
        // OSCULP32K's register carries neither RUNSTDBY nor ONDEMAND,
        // which could have made it unusable to a GCLK generator in
        // standby. It does not: the always-on oscillator really is
        // always on, and it is the cheapest through-standby source on
        // this die.
        bench.verdict("OSCULP32K serves a generator through standby although it "
                      "has no RUNSTDBY bit at all - twice, first and last",
                      near(legs[0].counted, legs[0].rtc, legs[0].rtc / 16u + 8u) &&
                          near(legs[5].counted, legs[5].rtc, legs[5].rtc / 16u + 8u));

        // The awake ruler is the counter-example, and this is what earns
        // it the right to be trusted for post-wake intervals only.
        uint32_t fine_max = 0;
        for (const Leg& l : legs) {
            fine_max = l.fine > fine_max ? l.fine : fine_max;
        }
        print(serial, "  the CLK_MAIN stopwatch advanced at most ", fine_us(fine_max),
              " us across those five standbys", crlf);
        bench.verdict("the CLK_MAIN stopwatch is FROZEN in standby: it may time "
                      "what happens after a wake, never the sleep itself",
                      fine_us(fine_max) < 2000u);
    }

    // Hand everything back, and IN THE RIGHT ORDER: a generator may not
    // be left pointing at a source that is about to stop (16.6.2.6, the
    // rule docs/samc/osc32kctrl.md records), so the generator moves back
    // to the always-on root first.
    (void)SlowWatch::enable(false);
    SlowWatch::release();
    (void)Gclk<gen_ulp>::configure(GclkConfig{.source = GclkSource::osculp32k});
    Osc32k::stop();

    watchdog_backstop(false);
    quiesce();
}

// ---------------------------------------------------------------------------
// d: standby
// ---------------------------------------------------------------------------

void td_standby() {
    quiesce();
    watchdog_backstop(true);

    // THE FREEZE, quantified. Half a second of RTC time, slept twice:
    // once in IDLE0, once in STANDBY, with the kernel tick read across
    // both.
    struct Freeze {
        uint32_t rtc;
        uint32_t tick;
        bool woke;
    };
    Freeze fr[2];
    const SleepMode two[2] = {SleepMode::idle0, SleepMode::standby};
    const uint32_t half = ulp_hz / 2u;
    for (uint8_t i = 0; i < 2u; ++i) {
        (void)Pm::set_sleep_mode(two[i]);
        console_drain();
        if (!arm_wake(half)) {
            fr[i] = Freeze{0, 0, false};
            continue;
        }
        const uint32_t k0 = Ticker::ticks();
        const uint32_t r0 = rtc_now();
        sleep_once();
        const bool woke = wait_wake();
        fr[i] = Freeze{rtc_now() - r0, Ticker::ticks() - k0, woke};
    }
    (void)Pm::set_sleep_mode(SleepMode::idle0);

    print(serial, "  a sleep of ", rtc_ms(fr[0].rtc), " ms in IDLE0: the tick "
          "advanced ", fr[0].tick, " ms", crlf);
    print(serial, "  a sleep of ", rtc_ms(fr[1].rtc), " ms in STANDBY: the tick "
          "advanced ", fr[1].tick, " ms", crlf);
    bench.verdict("an RTC compare wakes the device from STANDBY", fr[1].woke);
    bench.verdict("and the RTC counted right through it - its clock is not a "
                  "GCLK, so it is not sleepwalking either",
                  near(fr[1].rtc, half, half / 50u));
    // THE TICK RULE, measured: kernel time stands still for exactly as
    // long as the standby lasts.
    bench.verdict("THE KERNEL TICK FREEZES IN STANDBY: SysTick rides the CPU "
                  "clock and the CPU clock stops",
                  fr[1].tick <= 2u);
    // Erratum 1.8.13's workaround holds the SysTick INTERRUPT off across
    // every standby entry in this stratum (samc/ticker.hpp's
    // SysTickInterruptGuard, taken by both SamPlatform::idle() and
    // Pm::sleep()). It has to put it back, or the tick would be lost for
    // good after the first standby - which is a claim with a register
    // behind it.
    bench.verdict("and the erratum 1.8.13 guard gave the SysTick interrupt back "
                  "afterwards",
                  (SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) != 0u);
    bench.verdict("while the same sleep in IDLE0 keeps it to the millisecond",
                  near(fr[0].tick, rtc_ms(fr[0].rtc), rtc_ms(fr[0].rtc) / 20u + 2u));
    print(serial, "  so a standby of ", rtc_ms(fr[1].rtc), " ms makes every armed "
          "time event ", rtc_ms(fr[1].rtc) - fr[1].tick, " ms late", crlf);

    // The loop is frozen too - and to show it, the loop needs a REPEATING
    // wake, because SysTick is not one any more. The RTC's periodic
    // interrupt is: interval 3 fires every 16 source ticks with nothing
    // to re-arm, which is a wake every ~486 us.
    console_drain();
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::periodic(3));
    const uint32_t window = ulp_hz / 4u;
    (void)Pm::set_sleep_mode(SleepMode::standby);
    const uint32_t irq0 = rtc_irqs;
    const uint32_t asleep = idle_turns(true, window);
    const uint32_t wakes = rtc_irqs - irq0;
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    const uint32_t awake = idle_turns(false, window);
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::compare0);
    print(serial, "  over ", rtc_ms(window), " ms: ", awake,
          " loop turns spinning, ", asleep, " sleeping in STANDBY on ", wakes,
          " periodic interrupts", crlf);
    bench.verdict("the loop turns once per wake and not once more",
                  near(asleep, wakes, 2u));
    bench.verdict("the same span spinning turns it a hundred times as often",
                  awake > 100u * (asleep != 0u ? asleep : 1u));

    // THE WAKE BILL.
    console_drain();
    const bool ruler = deep_up();
    const uint32_t polled = deep_leg(WaitKind::poll, SleepMode::idle0, bill_p, bill_n);
    const uint32_t l_idle = deep_leg(WaitKind::sleep, SleepMode::idle0, bill_p, bill_n);
    const uint32_t l_stby = deep_leg(WaitKind::sleep, SleepMode::standby, bill_p, bill_n);
    const int32_t cost_idle = bill_ns(l_idle, polled);
    const int32_t cost_stby = bill_ns(l_stby, polled);
    bench.verdict("the crystal-fed ruler and all three legs ran",
                  ruler && polled != 0u && l_idle != 0u && l_stby != 0u);
    print(serial, "  against a POLLED wait on the same compare: IDLE0 costs ",
          cost_idle, " ns to leave, STANDBY ", cost_stby, " ns (mean of ", bill_n,
          " single wakes on the 24 MHz crystal)", crlf);
    print(serial, "  so STANDBY costs ", cost_stby - cost_idle,
          " ns more to leave than IDLE0, with the supply already up", crlf);
    bench.verdict("leaving STANDBY costs measurably more than leaving IDLE0",
                  cost_stby > cost_idle + 1000);
    // The bill is dominated by the clock chain's restart; letter f
    // itemizes it.
    bench.verdict("and the bill is microseconds, not the milliseconds a crystal "
                  "restart costs on the first target",
                  cost_stby - cost_idle < 1'000'000);

    // A second wake source, and the one that proves the watchdog runs in
    // standby: the early-warning interrupt.
    console_drain();
    (void)Watchdog::disable();
    Watchdog::clear_flags();
    wdt_warned = false;
    const bool armed = Watchdog::arm(WdtConfig{.period = WdtCycles::cyc1024,
                                               .early_warning = true,
                                               .ew_offset = WdtCycles::cyc128});
    Nvic::enable(Watchdog::irq());
    (void)Pm::set_sleep_mode(SleepMode::standby);
    const uint32_t rw0 = rtc_now();
    __disable_irq();
    if (!wdt_warned) {
        Pm::sleep();
    }
    __enable_irq();
    uint32_t spins = spin_cap;
    while (!wdt_warned && spins-- != 0u) {
    }
    const uint32_t rw1 = rtc_now();
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    Watchdog::clear();
    (void)Watchdog::disable();
    Nvic::disable(Watchdog::irq());
    print(serial, "  the watchdog's early warning woke the device after ",
          rtc_ms(rw1 - rw0), " ms (128 cycles of a nominal 1.024 kHz = 125 ms)",
          crlf);
    bench.verdict("the WATCHDOG runs through standby and its early warning is a "
                  "wake source", armed && wdt_warned);
    // OSCULP32K is the watchdog's clock and it is a per-cent-class RC;
    // the band is the oscillator's, not the measurement's.
    bench.verdict("and it arrives about where its 1.024 kHz nominal puts it",
                  near(rtc_ms(rw1 - rw0), 125u, 20u));

    watchdog_backstop(false);
    quiesce();
}

// ---------------------------------------------------------------------------
// e: the regulator and the RAM
// ---------------------------------------------------------------------------

void te_regulator() {
    quiesce();
    watchdog_backstop(true);

    struct Combo {
        const char* what;
        VregStandbyMode mode;
        bool vreg_runstdby;
        bool back_bias;
        int32_t ns;
        bool ok;
    };
    Combo combos[] = {
        {"AUTO,        VREG.RUNSTDBY 0", VregStandbyMode::automatic, false, true, 0, false},
        {"AUTO,        VREG.RUNSTDBY 1", VregStandbyMode::automatic, true, true, 0, false},
        {"PERFORMANCE, VREG.RUNSTDBY 0", VregStandbyMode::performance, false, true, 0, false},
        {"PERFORMANCE, VREG.RUNSTDBY 1", VregStandbyMode::performance, true, true, 0, false},
        {"LP,          VREG.RUNSTDBY 0", VregStandbyMode::low_power, false, true, 0, false},
        {"AUTO, no back-bias (BBIASHS 0)", VregStandbyMode::automatic, false, false, 0, false},
        {"AUTO,        VREG.RUNSTDBY 0, again", VregStandbyMode::automatic, false, true, 0, false},
    };

    // One IDLE leg is the common baseline: the software is identical in
    // every combination, so subtracting it once is enough.
    console_drain();
    const bool ruler = deep_up();
    const uint32_t base = deep_leg(WaitKind::sleep, SleepMode::idle0, bill_p, bill_n);
    const bool base_ok = ruler && base != 0u;

    for (Combo& c : combos) {
        (void)Pm::configure_standby(
            StandbyConfig{.regulator = c.mode, .back_bias = c.back_bias});
        Vreg::run_standby(c.vreg_runstdby);
        console_drain();
        const uint32_t leg = deep_leg(WaitKind::sleep, SleepMode::standby, bill_p,
                                      bill_n);
        c.ok = leg != 0u;
        c.ns = bill_ns(leg, base);
    }
    (void)Pm::configure_standby(StandbyConfig{});
    Vreg::run_standby(false);

    print(serial, "  each row is what ONE standby wake cost more than one IDLE0 "
          "wake, on the crystal ruler - which itself sleepwalks, so AUTO holds "
          "the MAIN regulator throughout", crlf);
    bool all_ok = base_ok;
    for (const Combo& c : combos) {
        print(serial, "  ", c.what, ": ", c.ns, " ns", crlf);
        all_ok = all_ok && c.ok;
    }
    bench.verdict("every combination slept and woke", all_ok);

    if (all_ok) {
        // ERRATUM 1.8.14's fingerprint. Its text says PERFORMANCE mode
        // alone "will wrongly switch to the low-power regulator and keep
        // requesting GCLK0" - and a standby that keeps requesting GCLK0
        // has nothing to restart, so it wakes FAST. That is an
        // observable consequence, and it is what the two PERFORMANCE
        // rows are here to show.
        // THE NOISE FLOOR IS PRINTED, not assumed: the first row is
        // repeated last, and no difference smaller than that repeat's
        // own is claimed below.
        int32_t lo = combos[0].ns;
        int32_t hi = combos[0].ns;
        for (const Combo& c : combos) {
            lo = c.ns < lo ? c.ns : lo;
            hi = c.ns > hi ? c.ns : hi;
        }
        const int32_t repeat = combos[6].ns > combos[0].ns ? combos[6].ns - combos[0].ns
                                                           : combos[0].ns - combos[6].ns;
        print(serial, "  the whole spread is ", hi - lo,
              " ns and the FIRST ROW REPEATED LAST differs from itself by ",
              repeat, " ns", crlf);
        // THE ANSWER, and it is a negative one worth having: this family
        // has no separate regulator bill. On AVR DA/DB the regulator was
        // a 290 us item on top of the oscillator's; here nothing in
        // STDBYCFG or SUPC.VREG moves the wake by more than the repeat's
        // own scatter.
        bench.verdict("NEITHER REGULATOR SETTING NOR BACK-BIAS MOVES THE WAKE: "
                      "the whole spread is inside twice the repeat's own scatter",
                      hi - lo < 2 * repeat + 3000);
        // Erratum 1.8.14's predicted fingerprint - PERFORMANCE alone
        // keeping GCLK0 requested, hence a cheap wake - IS NOT VISIBLE.
        // Reported, not verdicted: the erratum speaks of which regulator
        // is used and of a clock left requested, and this measurement
        // can only see the second of those through the wake time.
        print(serial, "  erratum 1.8.14 predicts PERFORMANCE alone keeps GCLK0 "
              "requested, which would make its wake the cheap one; measured "
              "AUTO ", combos[0].ns, " ns against PERFORMANCE ", combos[2].ns,
              " ns, so no such fingerprint is visible in the wake TIME", crlf);
    }

    watchdog_backstop(false);
    quiesce();
}

// ---------------------------------------------------------------------------
// f: what survives a standby
// ---------------------------------------------------------------------------


/// Time from a wake to a predicate coming true, in microseconds on the
/// awake ruler. The ruler itself is stopped during the standby, so it
/// measures the restart and nothing else.
template <typename Pred>
uint32_t time_to(Pred ready) {
    const uint32_t t0 = fine_now();
    uint32_t spins = spin_cap;
    while (!ready() && spins-- != 0u) {
    }
    return fine_us(fine_now() - t0);
}

void tf_survivors() {
    quiesce();
    watchdog_backstop(true);

    // --- OSC48M, CLK_MAIN's own source -------------------------------------
    //
    // It is the one clock nobody can ask about after a wake - the CPU
    // only runs once it is back - so it is measured by DIFFERENCE: the
    // wake bill of letter d, with and without its RUNSTDBY.
    console_drain();
    const bool ruler = deep_up();
    const uint32_t base = deep_leg(WaitKind::sleep, SleepMode::idle0, bill_p, bill_n);
    Osc48m::run_standby(false);
    const uint32_t l_off = deep_leg(WaitKind::sleep, SleepMode::standby, bill_p, bill_n);
    Osc48m::run_standby(true);
    const uint32_t l_on = deep_leg(WaitKind::sleep, SleepMode::standby, bill_p, bill_n);
    Osc48m::run_standby(false);

    const int32_t cost_off = bill_ns(l_off, base);
    const int32_t cost_on = bill_ns(l_on, base);
    print(serial, "  over an IDLE0 wake, a standby wake costs ", cost_off,
          " ns with OSC48M left to stop and ", cost_on,
          " ns with its RUNSTDBY set", crlf);
    bench.verdict("both bills were measured",
                  ruler && base != 0u && l_off != 0u && l_on != 0u);
    // A NULL RESULT, and the rest of this letter is the explanation:
    // the ruler is itself a sleepwalking task, and a sleepwalking task
    // requesting its APB clock wakes MCLK, generator 0 and the source
    // behind them (19.6.3.3.2). OSC48M is therefore already being held
    // up by the instrument, and its own flag has nothing left to add.
    bench.verdict("OSC48M's own RUNSTDBY changes the wake by nothing at all - "
                  "with a sleepwalking task present the main clock chain is "
                  "already held up by that task's own request",
                  (cost_on > cost_off ? cost_on - cost_off : cost_off - cost_on) <
                      4000);

    // --- what a sleepwalking task holds up, in three legs -------------------
    //
    // The claim above is not left as an interpretation: the crystal is
    // asked the same question three times, once with the instrument
    // running on it and twice without.
    // AND IT IS ASKED OF THE COUNTER, NOT OF A FLAG. XOSCRDY reads set
    // at every one of these wakes, whatever happened to the crystal - a
    // status bit that was never cleared and an oscillator that never
    // stopped look identical from a register. So each leg waits for the
    // crystal's OWN counter to move, which nothing but a live clock can
    // make happen.
    //
    // The three legs differ in WHO IS ASKING for the crystal across the
    // standby: a running TC, an enabled generator alone, or nobody. The
    // last two need the generator DISABLED for the duration and switched
    // back on at the wake, because an enabled generator is itself a
    // request - which is the finding this arrangement exists to isolate.
    struct XoscLeg {
        const char* what;
        bool hold_tc;       ///< the TC keeps running (and so keeps asking)
        bool hold_gen;      ///< the generator stays enabled across the standby
        bool run_standby;   ///< the crystal's own RUNSTDBY
        bool up;
        bool ready_at_wake;
        uint32_t tick_us;
    };
    XoscLeg xl[3] = {
        {"a TC running on it,      XOSC.RUNSTDBY 0", true, true, false, false, false, 0},
        {"nobody asking,           XOSC.RUNSTDBY 0", false, false, false, false, false, 0},
        {"nobody asking,           XOSC.RUNSTDBY 1", false, false, true, false, false, 0},
    };
    for (XoscLeg& l : xl) {
        l.up = Xosc::init(XoscConfig{.hz = crystal_hz,
                                     .startup = 4,
                                     .on_demand = false,
                                     .run_standby = l.run_standby}) &&
               deep_build(l.hold_tc);
        if (!l.up) {
            continue;
        }
        if (!l.hold_gen) {
            (void)Gclk<gen_xtal>::enable(false);
        }
        (void)Pm::set_sleep_mode(SleepMode::standby);
        console_drain();
        if (!arm_wake(ulp_hz / 32u)) {
            l.up = false;
            continue;
        }
        sleep_once();
        if (!wait_wake()) {
            l.up = false;
            continue;
        }
        l.ready_at_wake = Xosc::ready();
        if (!l.hold_gen) {
            (void)Gclk<gen_xtal>::enable(true);
        }
        l.tick_us = crystal_ticking() ? 0u : time_to([] { return crystal_ticking(); });
        (void)Pm::set_sleep_mode(SleepMode::idle0);
    }

    for (const XoscLeg& l : xl) {
        print(serial, "  XOSC after a standby, ", l.what, ": XOSCRDY=",
              l.ready_at_wake ? "set" : "clear", ", its counter first moved ",
              l.tick_us, " us after the wake", crlf);
    }
    bench.verdict("the crystal started in all three configurations",
                  xl[0].up && xl[1].up && xl[2].up);

    // THE CONTROL, because a probe that can only ever say yes proves
    // nothing: the crystal is stopped on purpose and the same question
    // asked again.
    const bool ticking_before = crystal_ticking();
    Xosc::stop();
    const bool ticking_after_stop = crystal_ticking();
    const bool restarted = Xosc::init(XoscConfig{.hz = crystal_hz,
                                                 .startup = 4,
                                                 .on_demand = false,
                                                 .run_standby = true});
    bench.verdict("the probe can say NO: with XOSC deliberately stopped its "
                  "counter does not move, and it moves again once it is restarted",
                  ticking_before && !ticking_after_stop && restarted &&
                      crystal_ticking());

    // THE FINDING, and it contradicts table 19-2. That table gives a
    // clock source with ONDEMAND = 0 and RUNSTDBY = 0 as "Stop" in
    // standby. Measured on the crystal's own counter in three
    // arrangements - a TC still running on it, its generator disabled
    // for the whole standby, and its own RUNSTDBY set - it is ALREADY
    // TICKING at every wake. This is the same shape erratum 1.3.1
    // records for the FDPLL ("still running even if not requested by any
    // module causing extra consumption"), which that document marks
    // revision B only.
    bench.verdict("XOSC KEEPS RUNNING THROUGH A STANDBY whatever RUNSTDBY says "
                  "and whoever is or is not asking for it - table 19-2 says it "
                  "should stop, and on this silicon it does not",
                  xl[0].up && xl[1].up && xl[2].up && xl[0].tick_us == 0u &&
                      xl[1].tick_us == 0u && xl[2].tick_us == 0u);
    // Which is also why the AVR's biggest sleep bill has no counterpart
    // here: there a 24 MHz crystal cost 1.77 ms to restart out of every
    // deep sleep, and no profile touched it.
    print(serial, "  so a standby on this board costs NO crystal restart, where "
          "the same crystal on the first target cost 1.77 ms out of every deep "
          "sleep", crlf);
    // The status flag is a separate trap and gets its own verdict: it is
    // set at the wake in every leg, so it could not have distinguished
    // the cases even if they had differed.
    bench.verdict("XOSCRDY could not have answered this question either way: it "
                  "reads set at every wake, and stays set across a deliberate "
                  "stop until the next enable",
                  xl[0].ready_at_wake && xl[1].ready_at_wake && xl[2].ready_at_wake);

    // --- the DPLL, locked to that crystal, with nothing asking for it ------
    //
    // 48 MHz from the 24 MHz crystal divided to 2 MHz: DIV 5 gives
    // 24e6 / (2 x (5 + 1)) = 2 MHz, and LDR 23 multiplies it by 24.
    struct DpllLeg {
        bool run_standby;
        bool up;
        bool ready_at_wake;
        uint32_t relock_us;
    };
    DpllLeg dl[2] = {{false, false, false, 0}, {true, false, false, 0}};
    for (DpllLeg& l : dl) {
        // The reference has to be running when the loop is enabled AND
        // through the standby, or what is measured is the crystal's
        // restart and not the loop's.
        if (!Xosc::init(XoscConfig{.hz = crystal_hz,
                                   .startup = 4,
                                   .on_demand = false,
                                   .run_standby = true})) {
            continue;
        }
        l.up = Fdpll::init(FdpllConfig{.reference = DpllReference::xosc,
                                       .reference_hz = crystal_hz,
                                       .xosc_div = 5,
                                       .ldr = 23,
                                       .run_standby = l.run_standby});
        if (!l.up) {
            continue;
        }
        (void)Pm::set_sleep_mode(SleepMode::standby);
        console_drain();
        if (!arm_wake(ulp_hz / 32u)) {
            l.up = false;
            continue;
        }
        sleep_once();
        if (!wait_wake()) {
            l.up = false;
            continue;
        }
        l.ready_at_wake = Fdpll::clock_ready();
        l.relock_us = time_to([] { return Fdpll::clock_ready(); });
        (void)Pm::set_sleep_mode(SleepMode::idle0);
    }
    Fdpll::stop();
    deep_down();
    Xosc::stop();

    print(serial, "  FDPLL96M after a standby, nothing asking for it: RUNSTDBY 0 "
          "CLKRDY=", dl[0].ready_at_wake ? "set" : "clear", " then ready in ",
          dl[0].relock_us, " us; RUNSTDBY 1 CLKRDY=",
          dl[1].ready_at_wake ? "set" : "clear", " then ready in ", dl[1].relock_us,
          " us", crlf);
    bench.verdict("the DPLL locked to the crystal in both configurations",
                  dl[0].up && dl[1].up);
    bench.verdict("its output clock is being issued again within microseconds of "
                  "the wake, with RUNSTDBY set or clear",
                  dl[0].up && dl[1].up && dl[0].relock_us < 200u &&
                      dl[1].relock_us < 200u);
    // DECLINED IN PRINT. Whether the loop ever STOPPED is not decided
    // here: DPLLSTATUS.CLKRDY is the only witness this suite has, and
    // the crystal above just showed what such a flag is worth after a
    // wake - nothing here counts the DPLL's output the way the TC counts
    // the crystal's. Erratum 1.3.1 says the FDPLL does keep running in
    // standby unrequested and marks that revision B; the reading here is
    // CONSISTENT with it and is not evidence for it.
    print(serial, "  whether the loop ever stopped is NOT decided here - CLKRDY "
          "is the only witness and this suite counts no DPLL-derived clock", crlf);

    watchdog_backstop(false);
    quiesce();
}

// ---------------------------------------------------------------------------
// g: the manager, awake
// ---------------------------------------------------------------------------

/// The kernel loop, minus the sleep. Bounded, so a self-feeding loop is a
/// failed verdict rather than a hung bench.
void pump() {
    for (uint16_t i = 0; i < 500u; ++i) {
        TimeEvents<P>::process();
        if (!K::step()) {
            return;
        }
    }
}

void ask(SleepDepth d) {
    post<Pm_>(SleepRequested{d, reply_to<Probe, SleepVote>()});
    pump();
}

/// "Awake, no new request": the one event a wake path posts when it has
/// nothing else to say. Any event would disarm the site.
void release_sleep() { ask(SleepDepth::none); }

void tg_manager() {
    quiesce();

    // The ladder through a real round, and the round's own cost.
    struct Leg {
        SleepDepth asked;
        uint8_t reg;
        SleepDepth manager;
        bool ok;
    };
    Leg legs[4];
    const SleepDepth ladder[4] = {SleepDepth::none, SleepDepth::light,
                                  SleepDepth::standby, SleepDepth::deep};
    for (uint8_t i = 0; i < 4u; ++i) {
        Probe::last_ok = false;
        ask(ladder[i]);
        legs[i] = Leg{ladder[i], Pm::sleepcfg(), Pm_::armed_depth(), Probe::last_ok};
        release_sleep();
    }
    for (const Leg& l : legs) {
        print(serial, "  ", depth_name(l.asked), ": SLEEPCFG=", hex(l.reg),
              " manager=", depth_name(l.manager), l.ok ? " (accepted)" : " (refused)",
              crlf);
    }
    bench.verdict("none arms nothing and still replies ok",
                  legs[0].reg == 0x0u && legs[0].manager == SleepDepth::none &&
                      legs[0].ok);
    bench.verdict("light arms IDLE2", legs[1].reg == 0x2u && legs[1].ok);
    bench.verdict("standby arms STANDBY", legs[2].reg == 0x4u && legs[2].ok);
    bench.verdict("deep arms STANDBY and the manager records STANDBY - what the "
                  "target really took, not what was asked",
                  legs[3].reg == 0x4u && legs[3].manager == SleepDepth::standby &&
                      legs[3].ok);

    Probe::asked = 0;
    const uint32_t t0 = fine_now();
    ask(SleepDepth::standby);
    const uint32_t t1 = fine_now();
    print(serial, "  a two-voter round (post to armed, through the kernel) cost ",
          fine_us(t1 - t0), " us", crlf);
    bench.verdict("both stakeholders were asked, at the depth requested",
                  Probe::asked == 1u && Probe::asked_depth == SleepDepth::standby);
    release_sleep();

    // The voters.
    post<Bus>(FakeBus::Request{1, {}});
    pump();
    bench.verdict("the fake engine took the transfer and kept it",
                  FakeBus::started == 1u);
    Probe::asked = 0;
    ask(SleepDepth::standby);
    bench.verdict("a bus mid-transfer refuses and the round aborts",
                  !Probe::last_ok && Pm::sleep_mode() == SleepMode::idle0 &&
                      Pm_::armed_depth() == SleepDepth::none);
    bench.verdict("the other stakeholder was still asked - unanimity, not "
                  "first-no",
                  Probe::asked == 1u);
    post<Bus>(TransferDone{bus_ok});
    pump();
    ask(SleepDepth::standby);
    bench.verdict("with the transfer finished the same request is accepted",
                  Probe::last_ok && Pm::sleep_mode() == SleepMode::standby);
    release_sleep();

    Probe::accept = false;
    Probe::asked = 0;
    ask(SleepDepth::standby);
    bench.verdict("any single refusal ends the round, whoever casts it",
                  !Probe::last_ok && Pm::sleep_mode() == SleepMode::idle0 &&
                      Probe::asked == 1u);
    Probe::accept = true;

    // The deadline guard.
    Probe::deadline.arm_every(1);
    ask(SleepDepth::deep);
    bench.verdict("a deadline nearer than min_deep_ticks refuses a deep request",
                  !Probe::last_ok && Pm::sleep_mode() == SleepMode::idle0);
    ask(SleepDepth::light);
    bench.verdict("the same near deadline lets a light request through",
                  Probe::last_ok && Pm::sleep_mode() == SleepMode::idle2);
    release_sleep();
    Probe::deadline.disarm();
    Probe::deadline.arm_every(1000);
    ask(SleepDepth::standby);
    bench.verdict("a distant deadline lets the same request through",
                  Probe::last_ok && Pm::sleep_mode() == SleepMode::standby);
    release_sleep();
    Probe::deadline.disarm();
    bench.verdict("with nothing armed, ticks_to_next has nothing to say",
                  !TimeEvents<P>::ticks_to_next().has_value());

    // The standing restrictions.
    {
        PowerLock lock = Pm_::restrict(SleepDepth::light);
        bench.verdict("a lock is held and names its ceiling",
                      static_cast<bool>(lock) && Pm_::ceiling() == SleepDepth::light);
        Probe::asked_depth = SleepDepth::none;
        ask(SleepDepth::deep);
        bench.verdict("a deep request is forced down to light",
                      Probe::last_ok && Pm::sleep_mode() == SleepMode::idle2 &&
                          Pm_::armed_depth() == SleepDepth::light);
        bench.verdict("and the stakeholders vote on the CLAMPED depth",
                      Probe::asked_depth == SleepDepth::light);
    }
    bench.verdict("the lock's scope ended, the ladder is whole again",
                  Pm_::ceiling() == SleepDepth::deep);

    const uint8_t wakes_before = Probe::wakes;
    ask(SleepDepth::standby);
    bench.verdict("the request that arrives armed is first of all a wake",
                  Probe::wakes == wakes_before + 1u &&
                      Probe::woke_from == SleepDepth::light);
    release_sleep();

    PowerLock standby_lock = Pm_::restrict(SleepDepth::standby);
    PowerLock light_lock = Pm_::restrict(SleepDepth::light);
    bench.verdict("the shallowest live restriction wins",
                  Pm_::ceiling() == SleepDepth::light);
    light_lock.release();
    bench.verdict("releasing it falls back to the other",
                  Pm_::ceiling() == SleepDepth::standby);
    standby_lock.release();
    bench.verdict("and releasing the last one restores the whole ladder",
                  Pm_::ceiling() == SleepDepth::deep);

    quiesce();
}

// ---------------------------------------------------------------------------
// h: the manager asleep
// ---------------------------------------------------------------------------

void th_asleep() {
    quiesce();
    watchdog_backstop(true);

    // THE PATTERN THIS TARGET REQUIRES, and it is the whole of the v1
    // policy: standby is legitimate when the kernel has NO armed time
    // event, because kernel time stops while the CPU clock does. The
    // question is the kernel's own.
    bench.verdict("with no time event armed the kernel says so, and standby is "
                  "legitimate",
                  !TimeEvents<P>::ticks_to_next().has_value());

    post_on_rtc = true;
    ask(SleepDepth::standby);
    bench.verdict("a two-voter round arms STANDBY through the kernel",
                  Probe::last_ok && Pm::sleep_mode() == SleepMode::standby &&
                      Pm_::armed_depth() == SleepDepth::standby);

    console_drain();
    const bool armed = arm_wake(ulp_hz / 8u);
    const uint32_t k0 = Ticker::ticks();
    const uint32_t r0 = rtc_now();
    // THE KERNEL'S OWN HOOK, not this suite's: idle_if_empty() masks,
    // finds every queue empty and calls SamPlatform::idle(), which takes
    // whatever SLEEPCFG holds. That is the whole of the "no new kernel
    // hook" claim.
    K::idle_if_empty();
    const bool woke = wait_wake();
    const uint32_t k1 = Ticker::ticks();
    const uint32_t r1 = rtc_now();

    bench.verdict("the wake was armed and arrived", armed && woke);
    print(serial, "  the kernel slept ", rtc_ms(r1 - r0), " ms of wall clock and ",
          k1 - k0, " ms of kernel time", crlf);
    bench.verdict("the kernel's idle hook took the STANDBY the manager armed",
                  near(rtc_ms(r1 - r0), 125u, 20u) && (k1 - k0) <= 2u);
    bench.verdict("the site is still armed: a wake that says nothing to the "
                  "manager changes nothing",
                  Pm::sleep_mode() == SleepMode::standby &&
                      Pm_::armed_depth() == SleepDepth::standby);

    // The first event after the wake.
    pump();
    bench.verdict("the wake ISR's event reached its own AO", Probe::blips >= 1u);
    const uint8_t wakes_before = Probe::wakes;
    release_sleep();
    print(serial, "  WakeReport: ", Probe::wakes - wakes_before, " received, was=",
          depth_name(Probe::woke_from), crlf);
    bench.verdict("the first event to reach the MANAGER disarms",
                  Pm::sleep_mode() == SleepMode::idle0 &&
                      SamSleepSite::armed() == SleepDepth::none);
    bench.verdict("and publishes the WakeReport of the depth that was armed",
                  Probe::wakes == wakes_before + 1u &&
                      Probe::woke_from == SleepDepth::standby);
    bench.verdict("the manager forgot the round",
                  Pm_::armed_depth() == SleepDepth::none);

    // THE TICK RULE, made visible: a time event armed across a standby
    // matures LATE by exactly the slept duration. This is the reason the
    // pattern above exists, and it is measured rather than asserted.
    Probe::blips = 0;
    Probe::deadline.arm(50);                 // 50 ms away
    const uint32_t before = TimeEvents<P>::ticks_to_next().value_or(0);
    ask(SleepDepth::standby);
    const bool refused_by_guard = !Probe::last_ok;
    if (!refused_by_guard) {
        console_drain();
        (void)arm_wake(ulp_hz / 4u);          // ~250 ms, five times the deadline
        const uint32_t r2 = rtc_now();
        K::idle_if_empty();
        (void)wait_wake();
        const uint32_t late_ms = rtc_ms(rtc_now() - r2);
        pump();
        print(serial, "  a time event 50 ms away, slept over for ", late_ms,
              " ms: it matured ", late_ms > 50u ? late_ms - 50u : 0u,
              " ms late, and the kernel never knew", crlf);
        bench.verdict("A TIME EVENT SLEPT OVER MATURES LATE BY THE WHOLE SLEPT "
                      "DURATION - which is why the pattern is 'no armed time "
                      "event, then standby'",
                      late_ms > 200u && Probe::blips >= 1u);
        release_sleep();
    }
    print(serial, "  (the deadline guard saw ", before,
          " ticks to the next event, min_deep_ticks 2)", crlf);
    Probe::deadline.disarm();

    watchdog_backstop(false);
    quiesce();
}

// ---------------------------------------------------------------------------
// menu
// ---------------------------------------------------------------------------

void register_tests() {
    bench.letter('a', "the PM register surface and the ladder mapping", ta_registers);
    bench.letter('b', "IDLE0 and IDLE2", tb_idle);
    bench.letter('c', "the instrument: what counts through a standby", tc_instrument);
    bench.letter('d', "STANDBY: the freeze and the wake", td_standby);
    bench.letter('e', "the regulator and the RAM back-bias", te_regulator);
    bench.letter('f', "what survives a standby", tf_survivors);
    bench.letter('g', "the power manager, awake", tg_manager);
    bench.letter('h', "the power manager asleep, and the tick rule", th_asleep);
}

void help() {
    print(serial, "test_samc_sleep:", crlf);
    bench.menu();
}

} // namespace

// The console. Its SERCOM has no RUNSTDBY, so it is stopped for the
// duration of every standby here - which is why nothing is printed
// between arming a sleep and coming back from it.
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

// The wake source, and in letter h also the thing that posts the event
// the kernel wakes up to serve.
extern "C" void RTC_Handler() {
    (void)brio::Rtc::isr();
    rtc_irqs = rtc_irqs + 1;
    rtc_fired = true;
    if (post_on_rtc) {
        brio::post<Probe>(Blip{});
    }
}

// The second standby wake source, and the anti-wedge backstop's own
// vector: the early warning fires before the reset does.
extern "C" void WDT_Handler() {
    (void)brio::Watchdog::isr();
    wdt_warned = true;
}

// The fault body: with C_DEBUGEN cleared (tools/bench.py does that at the
// end of every SAM flash) a BKPT faults instead of halting, and this
// turns the wreck into a reboot with a breadcrumb.
extern "C" void HardFault_Handler() { brio::hard_fault_reset<P>(0); }

int main() {
    const ResetCause why = brio::Reset::cause();

    SysClock::init();
    Serial::init(clock, 115200);
    (void)brio::Ticker::init(clock);
    brio::enable_interrupts();

    boot_sleepcfg = brio::Pm::sleepcfg();
    boot_stdbycfg = static_cast<uint16_t>(brio::Pm::stdbycfg());

    // The awake ruler.
    bool rulers = Fine::init(0) &&
                  Fine::configure(brio::TcConfig{.mode = brio::TcMode::count32,
                                                 .prescaler = brio::TcPrescaler::div1}) &&
                  Fine::enable(true);
    rulers = rulers && rtc_up();

    // The RTC's own rate, against the awake ruler: one cited constant
    // (OSC48M's measured 47.755 MHz) and everything else derived here.
    if (rulers) {
        const uint32_t want = 4096u;
        const uint32_t r0 = rtc_now();
        const uint32_t f0 = fine_now();
        while (rtc_now() - r0 < want) {
        }
        const uint32_t fd = fine_now() - f0;
        if (fd != 0u) {
            ulp_hz = static_cast<uint32_t>(
                (static_cast<uint64_t>(want) * fine_hz) / fd);
        }
    }

    K::init_all();

    print(serial, crlf, "test_samc_sleep - PM (ch. 19) + the SleepSite (reset ",
          cause_name(why), ", CPU ", SysClock::hz / 1'000'000u, " MHz)",
          crlf);
    print(serial, "  rulers: CLK_MAIN stopwatch at ", fine_hz / 1000u,
          " kHz (OSC48M's measured rate, docs/samc/clock.md), RTC on OSCULP32K "
          "measured here at ", ulp_hz, " Hz", crlf);
    print(serial, "  the kernel tick is SysTick at ",
          brio::Ticker::ticks_per_second, " Hz and IT STOPS IN STANDBY", crlf);
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
