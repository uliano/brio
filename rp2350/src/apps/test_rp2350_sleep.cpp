// test_rp2350_sleep - the reference bench suite for this chip's POWER
// CHAPTER: the always-on block whole (datasheet 6.2 the domains and
// their states, 6.3 the core regulator read-only, 6.4 the registers),
// the always-on timer in it (12.10), the three depths of stopping a
// program can arm (6.5: the top-level clock gates, the SLEEP state,
// DORMANT) and the two states that are off the ladder because leaving
// one is a BOOT (6.2.2's P1.m).
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets; a verdict that must differ between them says
// so in its own text. The one that does is what the kernel's TIMEBASE
// does across the SLEEP state, because the two halves count time in
// different hardware: SysTick on one, the platform timer in the SIO on
// the other.
//
// THE TWO RULERS. TIMER0 (rp2350/timer.hpp) counts microseconds off
// clk_ref and is the fine one, but it stops when the crystal does. The
// ALWAYS-ON TIMER counts milliseconds and NEVER stops - not in a
// dormant, not with the switched core unpowered - so it is the only
// witness a deep letter has, and the coarse one. Where both run, both
// are printed.
//
// NOTHING TO WIRE for `z`. The console is the Debug Probe's UART bridge
// on GP0 (TX) / GP1 (RX) = UART0. One by-name letter uses the board's
// own standing wire GP19 -> GP8 to wake itself from a dormant by an
// edge it drives with its own other pad.
//
// A NOTE ON THE DEBUGGER, which letter a measures and every deep letter
// re-states: the debug port's CSYSPWRUPREQ powers every domain and
// inhibits software's own transitions (6.2.3.3). A probe that has
// attached once is likely to leave it asserted when it detaches, and
// then a power-down request is answered with REQ_IGNORED and nothing
// happens. DBG_PWRCFG.IGNORE is the way past it, and each letter says
// which condition it ran under.
//
// What is exercised, letter by letter:
//   a  as found: the power state, the sequencer, the regulator and the
//      brown-out detector decoded, the low-power oscillator's trim, the
//      boot and scratch words, why this boot happened - and THE
//      DEBUGGER'S STANDING POWER-UP REQUEST
//   b  the password: a write without it is dropped and flagged, a write
//      with it takes, and the flag clears
//   c  the always-on timer: the low-power oscillator MEASURED against
//      the crystal, what the divider is told, and two seconds counted
//      against the microsecond ruler
//   d  the alarm as an interrupt, with no sleep anywhere: the latency
//      of the handler's entry, the flag, the disarm before the match
//   e  the tick source moved onto the crystal and back, each measured
//   f  the ladder through the plain site, the gate sets it writes, and
//      the state machine's refusals
//   g  a LIGHT sleep ended by the alarm: the wake latency
//   h  THE SLEEP STATE: a PWM slice with its gate pruned as the
//      instrument, and what the kernel's own timebase does across it
//   j  a STANDBY through the timed site: a time event 300 ms out, the
//      alarm placed, the witness's resync
//   k  the eight scratch words, the four boot words, and what the
//      always-on domain keeps that the switched core does not
//   m  the manager over the timed site: the round, the ceiling, the
//      deadline guard
//
//   n  (by name) A DORMANT ON THE CRYSTAL, woken by the always-on
//      timer's alarm. About 10 s of wall clock:
//          brio run <board> n --app test_rp2350_sleep
//                  --expect="letter done" --timeout 60
//   o  (by name) A DORMANT ON THE CRYSTAL woken by a GPIO edge this
//      board drives itself (GP19 -> GP8), the alarm armed beside it as
//      the net. About 10 s.
//   p  (by name) A DORMANT ON THE RING OSCILLATOR, the crystal left
//      running, woken by the alarm. About 10 s.
//   q  (by name) P1.0: the switched core powered down with every memory
//      retained, woken by the alarm - and THE WAKE IS A BOOT, so the
//      letter resumes from a token like the reset suite's. About 15 s:
//          brio run <board> q --app test_rp2350_sleep
//                  --expect="letter done" --timeout 90
//   r  (by name) P1.7: the deepest state this chip has - the switched
//      core, the XIP cache and both SRAM domains unpowered. The same
//      shape, and only after the rescue has been proven from a P1.m.
//   u  (by name) THE RESCUE NET, dormant: a dormant on the crystal with
//      a SIXTY SECOND alarm under it, entered and announced, so that a
//      rescue over the debug port can be tried against a chip whose
//      clocks are all stopped and which comes back by itself either way.
//   v  (by name) THE RESCUE NET, powered down: the same for P1.0.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/powman.hpp"
#include "rp2350/pwm.hpp"
#include "rp2350/reset.hpp"
#include "rp2350/sleep.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "rp2350/watchdog.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = brio::Rp2350Platform<>;

// The token letter q resumes from, in SRAM the crt never touches - one
// of THREE copies, because which of them crosses a power-down of the
// switched core is the measurement.
struct NoinitToken {
    uint32_t magic;
    uint32_t leg;
    uint32_t tally;
};
[[gnu::section(".noinit")]] inline NoinitToken noinit_token;
inline constexpr uint32_t noinit_magic = 0x51A5'F00Du;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;       // the board's LED: a keystroke marker
using Ruler = Timer<0>;    // the microsecond ruler, which stops with the crystal
using Waker = Pin<19>;     // the standing wire's driven end
using Woken = Pin<8>;      // its other end, which a dormant watches

using Plain = Rp2350SleepSite<SysClock, DormantSource::xosc>;
using PlainRing = Rp2350SleepSite<SysClock, DormantSource::rosc>;
using Timed = Rp2350TimedSleepSite<P, SysClock, DormantSource::xosc>;

/// The gates a standby of this suite keeps: what a program must be to
/// go on being one, the always-on block so the alarm is reached, the
/// microsecond ruler and the console. THE PWM IS LEFT OUT ON PURPOSE -
/// it is the instrument.
constexpr SleepClocks standby_gates =
    sleep_clocks_core | sleep_clocks_powman | sleep_clocks_timer | sleep_clocks_uart0;

/// The instrument: a PWM slice counting clk_sys / 256, which wraps every
/// 111 ms at 150 MHz. Its gate is pruned from the standby set, so what
/// it counted across a sleep says whether the SLEEP state was reached.
using Instrument = PwmSlice<7>;

TestBench<Serial> bench;

// What this boot was told, sampled once in main().
uint32_t boot_causes = 0;
uint32_t boot_last_pwrup = 0;
bool boot_powman_alive = false;
bool boot_scratch_alive = false;
bool boot_noinit_alive = false;
bool boot_debug_pwrup = false;

// A time event to have something armed: the AO it posts to is never
// dispatched by a kernel here, and its queue is read by hand.
struct Sleeper {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Sleeper, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

volatile bool site_alarm_fired = false;
volatile uint32_t alarm_entry_us = 0;
volatile uint32_t alarm_entry_ms = 0;
volatile uint8_t alarm_fires = 0;

uint32_t us_now() { return Ruler::now_low(); }
uint32_t ms_now() { return AonTimer::now_low(); }

const char* arch_name() { return core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33"; }

/// Let the console's transmitter fall silent: its interrupt would
/// otherwise end every sleep below, and a state that stops the clocks
/// must not cut a line in half.
void console_drain() {
    const uint32_t t0 = ms_now();
    while (!Serial::tx_idle() && ms_now() - t0 < 300u) {
    }
}

/// Spin on the microsecond ruler.
void wait_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

/// idle() until `flag` or `max_turns`: the kernel loop's own shape, one
/// sleep a turn.
uint16_t idle_until(volatile bool& flag, uint16_t max_turns = 20000) {
    uint16_t turns = 0;
    while (!flag && turns < max_turns) {
        P::CriticalSection cs;
        P::idle();
        ++turns;
    }
    return turns;
}

/// The state every letter starts from: nothing armed, no wake standing,
/// every gate open, the site disarmed.
void quiesce() {
    Timed::disarm();
    Plain::disarm();
    PlainRing::disarm();
    AonTimer::alarm_enable(false);
    AonTimer::clear_alarm();
    AonTimer::powerup_on_alarm(false);
    DormantWake::disable_all();
    Powman::pwrup_disable_all();
    Powman::clear_request_flags();
    Powman::clear_bad_password();
    Clocks::sleep_enables(sleep_clocks_all);
    site_alarm_fired = false;
    alarm_fires = 0;
    Sleeper::alarm.disarm();
    (void)Sleeper::queue.pop();
}

/// The condition of the debug port, printed by every letter that means
/// to stop: a standing CSYSPWRUPREQ blocks a power-down outright and
/// would fake a measurement of one.
void report_debug_condition() {
    print(serial, "  debug power-up request: ",
          Powman::debug_powerup_pending() ? "STANDING" : "not standing", ", DBG_PWRCFG.IGNORE=",
          Powman::ignoring_debug_powerup() ? 1 : 0, crlf);
}

/// Take the debugger out of the way if it is in it, and say so. Returns
/// true when a power-down may proceed.
bool clear_debug_block() {
    if (!Powman::debug_powerup_pending()) {
        return true;
    }
    Powman::ignore_debug_powerup(true);
    print(serial, "  the probe left CSYSPWRUPREQ asserted: DBG_PWRCFG.IGNORE set, request now ",
          Powman::debug_powerup_pending() ? "STILL STANDING" : "gone", crlf);
    return !Powman::debug_powerup_pending();
}

// =============================================================================
// a - the always-on block as found
// =============================================================================
void ta_as_found() {
    const ChipId id = ChipId::read();
    print(serial, "  chip revision ", hex(id.revision), ", running ", arch_name(), " core ",
          P::core_id(), "; power state P",
          is_low_power_state(Powman::current_state()) ? 1 : 0, ".",
          static_cast<int>(state_code(Powman::current_state()) & 7u), " (STATE=",
          hex(Powman::state_word()), ")", crlf);
    bench.verdict("software runs, so the switched core is powered: the state is a P0.m, and "
                  "with nothing yet powered down it is P0.0",
                  Powman::current_state() == PowerState::p0_0);
    bench.verdict("no transition is in flight and neither BAD_* flag stands",
                  !Powman::changing() && !Powman::waiting() && !Powman::bad_software_request() &&
                      !Powman::bad_hardware_request());
    const bool ignored_at_boot = Powman::request_ignored();
    const bool waiting_at_boot = Powman::powerup_while_waiting();
    Powman::clear_request_flags();
    const bool ignored_after_clear = Powman::request_ignored();
    const bool blocked = Powman::debug_powerup_pending();
    Powman::ignore_debug_powerup(true);
    Powman::clear_request_flags();
    const bool ignored_with_ignore = Powman::request_ignored();
    Powman::ignore_debug_powerup(false);
    print(serial, "  STATE's two write-to-clear flags: REQ_IGNORED read ",
          ignored_at_boot ? "SET" : "clear", " at boot, ",
          ignored_after_clear ? "SET" : "clear", " after a clear with the debug port ",
          blocked ? "asking" : "quiet", ", and ", ignored_with_ignore ? "SET" : "clear",
          " after a clear with DBG_PWRCFG.IGNORE set; PWRUP_WHILE_WAITING read ",
          waiting_at_boot ? "SET" : "clear", " at boot", crlf);
    bench.verdict("REQ_IGNORED CANNOT BE CLEARED WHILE THE DEBUG PORT IS ASKING, BECAUSE "
                  "THE WRITE THAT CLEARS IT IS ITSELF A REQUEST: the two flags live in "
                  "STATE beside REQ, so acknowledging them means storing that register, "
                  "and 6.2.3.1's first rule is that a write to REQ with a power-up request "
                  "pending sets REQ_IGNORED and takes no further action. With "
                  "DBG_PWRCFG.IGNORE set the same store clears it at once - which makes "
                  "the flag a true report of the condition and not a latch a program can "
                  "tidy away",
                  !blocked || (ignored_after_clear && !ignored_with_ignore));

    const SequencerConfig sq = Powman::sequencer();
    print(serial, "  sequencer: clock ", sq.using_fast_powck ? "clk_ref" : "lposc",
          ", regulator ", sq.using_vreg_lp ? "low power" : "normal", ", detector ",
          sq.using_bod_lp ? "low power" : "normal", "; on the way down vreg_lp=",
          sq.use_vreg_lp ? 1 : 0, " bod_lp=", sq.use_bod_lp ? 1 : 0, " lposc_kept=",
          sq.run_lposc_in_lp ? 1 : 0, "; SRAM on the way up: ",
          sq.hw_pwrup_sram0 ? "as found" : "powered", "/",
          sq.hw_pwrup_sram1 ? "as found" : "powered", crlf);
    bench.verdict("the block runs off the reference clock while the switched core is up, and "
                  "falls back to the low-power oscillator when it goes - which is why the "
                  "oscillator is kept running there",
                  sq.using_fast_powck && sq.run_lposc_in_lp);
    bench.verdict("both supplies are in their normal mode with software running, and both are "
                  "set to switch to their low-power one when the core goes down",
                  !sq.using_vreg_lp && !sq.using_bod_lp && sq.use_vreg_lp && sq.use_bod_lp);

    const VregStatus vr = Powman::vreg();
    print(serial, "  regulator: VSEL=", hex(vr.vsel), " = ", vr.mv, " mV, VOUT_OK=",
          vr.vout_ok ? 1 : 0, ", startup=", vr.starting_up ? 1 : 0, ", HIZ=",
          vr.high_impedance ? 1 : 0, ", unlocked=", vr.unlocked ? 1 : 0, ", limit_off=",
          vr.voltage_limit_off ? 1 : 0, ", over-temperature at ", vr.high_temp_c, " C", crlf);
    const VregLowPower lpe = Powman::vreg_low_power_entry();
    const VregLowPower lpx = Powman::vreg_low_power_exit();
    print(serial, "  regulator in a low-power state: ", lpe.mv, " mV ",
          lpe.linear ? "linear" : "switching", "; on the way out ", lpx.mv, " mV ",
          lpx.linear ? "linear" : "switching", crlf);
    bench.verdict("the digital core sits at its nominal 1.10 V and the output is in "
                  "regulation - and NOTHING IN THIS TREE WRITES THAT: the voltage is decoded "
                  "and never set",
                  vr.mv == 1100u && vr.vout_ok && !vr.high_impedance);
    bench.verdict("the regulator has never been unlocked, which is a one-way door",
                  !vr.unlocked && !vr.voltage_limit_off);
    bench.verdict("the sequencer's own low-power setting is the linear mode at the same "
                  "1.10 V, and the way out is the switching mode again",
                  lpe.linear && lpe.mv == 1100u && !lpx.linear && lpx.mv == 1100u);

    const BodStatus bd = Powman::bod();
    print(serial, "  brown-out detector: ", bd.enabled ? "on" : "off", " at ", bd.mv,
          " mV; in a low-power state ", bd.lp_entry_enabled ? "on" : "off", " at ",
          bd.lp_entry_mv, " mV, on the way out ", bd.lp_exit_enabled ? "on" : "off", " at ",
          bd.lp_exit_mv, " mV", crlf);
    bench.verdict("the detector guards the core at 946 mV, WHICH IS NOT THE THRESHOLD 6.4'S "
                  "OWN TABLE MARKS AS THE DEFAULT: the table puts '(default)' on code 01001 "
                  "= 0.860 V, and the register's stated reset value is 0x0b1 - code 01011, "
                  "0.946 V - which is what the silicon reads. The reset value wins and the "
                  "marker is on the wrong row; the same code in the regulator's table IS "
                  "its default, which is probably how the two got confused",
                  bd.enabled && bd.vsel == 0x0Bu && bd.mv == 946u);

    print(serial, "  low-power oscillator: trim ", Lposc::trim(), ", the divider told ",
          AonTimer::declared_hz(), " Hz", crlf);
    print(serial, "  boot words ", hex(Powman::boot_word(0)), " ", hex(Powman::boot_word(1)), " ",
          hex(Powman::boot_word(2)), " ", hex(Powman::boot_word(3)), crlf);
    bench.verdict("the bootrom's four boot-vector words are clear: nothing in this tree "
                  "diverts the boot path",
                  Powman::boot_word(0) == 0u && Powman::boot_word(1) == 0u);

    print(serial, "  this boot: LAST_SWCORE_PWRUP=", hex(boot_last_pwrup),
          ", CURRENT_PWRUP_REQ=", hex(Powman::current_powerup_requests()), ", causes=",
          hex(boot_causes), crlf);
    bench.verdict("BOTH POWER-UP REGISTERS ARE A SET AND NOT A NUMBER: 6.4 lists their "
                  "sources as '0 = chip reset, 1 = pwrup0 ... 6 = alarm_pwrup', which reads "
                  "like an index, and the silicon answers 0x01 for a chip reset and 0x20 "
                  "for the debug port - bit 0 and bit 5, one bit per source in a seven-bit "
                  "field",
                  boot_last_pwrup == pwrup_bit(PowerUpSource::chip_reset) ||
                      boot_last_pwrup == pwrup_bit(PowerUpSource::alarm));
    bench.verdict("the switched core was last powered by a chip reset and not by a wake: "
                  "this image was flashed and started, not resumed",
                  boot_last_pwrup == pwrup_bit(PowerUpSource::chip_reset) ||
                      (boot_causes & ResetCause::swcore_powerdown) != 0u);

    report_debug_condition();
    bench.verdict("the debug port's power-up request is READ, whichever way it reads: a "
                  "standing one blocks every power-down and the deep letters say so",
                  true);
}

// =============================================================================
// b - the password
// =============================================================================
void tb_password() {
    Powman::clear_bad_password();
    bench.verdict("the bad-password flag starts down", !Powman::bad_password());

    // A write with NO password, aimed at the one register where the
    // answer costs nothing either way: BADPASSWD's own bit. Dropped, it
    // RAISES the flag; taken, it would have CLEARED it.
    Powman::probe_password();
    const bool flagged = Powman::bad_password();
    print(serial, "  a write of BADPASSWD with no password: the flag reads ",
          flagged ? "SET - so the write was dropped" : "clear - so the write was TAKEN", crlf);
    bench.verdict("a write whose top sixteen bits are not 0x5AFE is dropped, and raises "
                  "BADPASSWD - which is what makes the password a password",
                  flagged);

    Powman::clear_bad_password();
    bench.verdict("the flag is write-to-clear - AND THE CLEAR NEEDS THE PASSWORD TOO, this "
                  "register being one of the protected ones",
                  !Powman::bad_password());

    // The same store with the password, through the driver's one verb.
    AonTimer::powerup_on_alarm(true);
    const bool took = AonTimer::powerup_on_alarm();
    AonTimer::powerup_on_alarm(false);
    bench.verdict("the same field written through the driver takes, and no flag is raised",
                  took && !AonTimer::powerup_on_alarm() && !Powman::bad_password());

    // The unprotected half: the scratch words take a plain store.
    const uint32_t keep = Powman::scratch(7);
    Powman::scratch(7, 0x1234'5678u);
    const bool plain = Powman::scratch(7) == 0x1234'5678u;
    Powman::scratch(7, keep);
    bench.verdict("the scratch words are the other half of 6.4 - past offset 0xac, so a "
                  "plain 32-bit store reaches them",
                  plain && !Powman::bad_password());
}

// =============================================================================
// c - the timer, and the oscillator it counts
// =============================================================================
void tc_timer() {
    bench.verdict("the always-on timer is running on the low-power oscillator, which is "
                  "where init() left it", AonTimer::running() &&
                                              AonTimer::source() == AonSource::lposc);

    const std::optional<uint32_t> measured = SysClock::count_hz(CountSource::lposc);
    const uint32_t told = AonTimer::declared_hz();
    print(serial, "  the low-power oscillator measured against the crystal: ",
          measured ? *measured : 0u, " Hz against a nominal 32768; the divider is told ",
          told, " Hz", crlf);
    bench.verdict("the frequency counter sees the oscillator at all",
                  measured.has_value() && *measured > 20'000u && *measured < 50'000u);
    bench.verdict("AND IT IS NOT AT ITS NOMINAL RATE: what init() wrote into the divisor is "
                  "the MEASURED rate and not 32.768 kHz, which is the whole reason this "
                  "chapter measures it - a timer told the wrong rate counts the wrong "
                  "milliseconds",
                  measured.has_value() && told + 200u >= *measured && told <= *measured + 200u);

    // Two seconds by the always-on timer against two seconds by the
    // microsecond ruler, which counts the crystal.
    const uint32_t m0 = ms_now();
    const uint32_t u0 = us_now();
    while (ms_now() - m0 < 2000u) {
    }
    const uint32_t du = us_now() - u0;
    const uint32_t dm = ms_now() - m0;
    print(serial, "  ", dm, " ms of the always-on timer took ", du,
          " us of the crystal's ruler", crlf);
    bench.verdict("two seconds of the always-on timer are two seconds of the crystal to "
                  "within one per cent: the trimmed oscillator plus a measured divisor",
                  du > 1'980'000u && du < 2'020'000u);

    // The 64-bit read, and the clear that does not stop the counter.
    bool monotonic = true;
    uint64_t last = AonTimer::now();
    for (uint16_t k = 0; k < 2000u; ++k) {
        const uint64_t n = AonTimer::now();
        if (n < last) { monotonic = false; }
        last = n;
    }
    bench.verdict("2000 reads by 12.10.3's own procedure never go backwards", monotonic);
    bench.verdict("the low half alone is an honest span: the counter is nowhere near its "
                  "first wrap at 49 days", (AonTimer::now() >> 32) == 0u);

    const uint32_t before_clear = AonTimer::now_low();
    AonTimer::clear();
    wait_us(20'000u);
    const uint32_t after_clear = AonTimer::now_low();
    print(serial, "  CLEAR at ", before_clear, " ms: the counter reads ", after_clear,
          " ms 20 ms later, still running", crlf);
    bench.verdict("CLEAR is self-clearing and takes the counter to zero WITHOUT stopping it",
                  after_clear < 60u && after_clear >= 15u);
    bench.verdict("set() is refused while the counter runs, as 12.10.3 requires",
                  !AonTimer::set(0u));
    AonTimer::run(false);
    const bool set_ok = AonTimer::set(1000u);
    AonTimer::run(true);
    wait_us(10'000u);
    bench.verdict("and taken with the counter stopped, after which it counts on from there",
                  set_ok && AonTimer::now_low() >= 1000u && AonTimer::now_low() < 1060u);
}

// =============================================================================
// d - the alarm as an interrupt, with no sleep anywhere
// =============================================================================
void td_alarm() {
    alarm_fires = 0;
    site_alarm_fired = false;
    AonTimer::interrupt(true);
    Irq::enable(AonTimer::irq());

    const uint64_t at = AonTimer::alarm_in(300u);
    const bool armed = AonTimer::alarm_enabled();
    const bool reads_back = AonTimer::alarm_time() == at;
    const uint32_t t0 = us_now();
    while (alarm_fires == 0u && us_now() - t0 < 500'000u) {
    }
    const uint32_t entry_ms = alarm_entry_ms;
    print(serial, "  alarm at ", static_cast<uint32_t>(at), " ms: fired ",
          static_cast<int>(alarm_fires), " time(s), the handler entered at ", entry_ms,
          " ms (", entry_ms - static_cast<uint32_t>(at), " ms after the match)", crlf);
    bench.verdict("the alarm reads back as written and arms", armed && reads_back);
    bench.verdict("it fires ONCE and the handler enters in the same millisecond as the "
                  "match - which is the timer's whole resolution",
                  alarm_fires == 1u && entry_ms - static_cast<uint32_t>(at) <= 1u);
    bench.verdict("the ISR body disarmed it as well as acknowledging it: the match goes on "
                  "being true after the event, so an armed alarm whose flag is merely "
                  "cleared raises it again on the next tick",
                  !AonTimer::alarm_enabled() && !AonTimer::alarm_fired());

    // Disarmed before its time: it never fires.
    alarm_fires = 0;
    (void)AonTimer::alarm_in(200u);
    AonTimer::alarm_enable(false);
    wait_us(400'000u);
    bench.verdict("an alarm disarmed before its match never fires",
                  alarm_fires == 0u && !AonTimer::alarm_fired());

    // The power-up duty is a bit of its own, and it is not the
    // interrupt.
    AonTimer::powerup_on_alarm(true);
    const bool both = AonTimer::powerup_on_alarm() && AonTimer::interrupt_enabled();
    AonTimer::powerup_on_alarm(false);
    bench.verdict("the alarm's two duties are two bits: an interrupt, and a POWER-UP "
                  "REQUEST that brings the switched core back from a state with no "
                  "processor in it",
                  both && !AonTimer::powerup_on_alarm());
    AonTimer::interrupt(false);
    Irq::disable(AonTimer::irq());
}

// =============================================================================
// e - the tick source moved onto the crystal and back
// =============================================================================
void te_source() {
    print(serial, "  the divider is told: low-power oscillator ", AonTimer::lposc_khz_int(),
          ".", AonTimer::lposc_khz_frac(), " kHz, crystal ", AonTimer::xosc_khz_int(), ".",
          AonTimer::xosc_khz_frac(), " kHz", crlf);
    bench.verdict("the crystal's divisor is this board's 12 MHz, written by init() from the "
                  "clock task's own constant and not assumed",
                  AonTimer::xosc_khz_int() == SysClock::xtal_hz / 1000u);

    const bool moved = AonTimer::use_xosc();
    const uint32_t m0 = ms_now();
    const uint32_t u0 = us_now();
    while (ms_now() - m0 < 1000u) {
    }
    const uint32_t du_x = us_now() - u0;
    const bool back = AonTimer::use_lposc();
    print(serial, "  on the crystal: 1000 ms took ", du_x, " us; the source reads ",
          AonTimer::source() == AonSource::lposc ? "lposc again" : "NOT lposc", crlf);
    bench.verdict("the source moves onto the crystal, the select bit clearing itself when "
                  "the change has been made", moved);
    bench.verdict("and on the crystal a second is a second to a part in a thousand - the "
                  "divisor being a whole 12000 kHz against a crystal that is exactly that",
                  du_x > 999'000u && du_x < 1'001'000u);
    bench.verdict("and it moves back onto the low-power oscillator, which is the only "
                  "source that survives a power-down of the switched core",
                  back && AonTimer::source() == AonSource::lposc);
    bench.verdict("neither move needed the counter stopped: 12.10.5 makes both selects live",
                  AonTimer::running());
    bench.verdict("a divisor write is refused while the counter runs off THAT source, and "
                  "taken for the other one",
                  !AonTimer::lposc_khz(32u, 0u) &&
                      AonTimer::xosc_khz(static_cast<uint16_t>(SysClock::xtal_hz / 1000u), 0u));
}

// =============================================================================
// f - the ladder, the gate sets, and the state machine's refusals
// =============================================================================
void tf_ladder() {
    bench.verdict("every gate is open in both masks, which is the reset state and a sleep "
                  "that prunes nothing",
                  Clocks::sleep_enables() == sleep_clocks_all &&
                      Clocks::wake_enables() == sleep_clocks_all);

    Plain::standby_clocks(standby_gates);
    bench.verdict("light arms, writes every gate open and installs no hook",
                  Plain::arm(SleepDepth::light) && Plain::armed() == SleepDepth::light &&
                      Clocks::sleep_enables() == sleep_clocks_all &&
                      P::sleep_hook == nullptr);
    bench.verdict("standby arms and WRITES THE PROGRAM'S PRUNING SET into SLEEP_ENx - which "
                  "is the whole of what this rung is on this chip: no SLEEPDEEP takes part "
                  "in 6.5.2's condition, and the Hazard3 half has no such bit at all",
                  Plain::arm(SleepDepth::standby) && Plain::armed() == SleepDepth::standby &&
                      Clocks::sleep_enables() == (standby_gates | Timed::alarm_gates));
    const bool releases_armed = sleep_releases_power_request();
    Plain::disarm();
    bench.verdict("disarm takes the gates back open", Clocks::sleep_enables() == sleep_clocks_all);
    bench.verdict("AND THE OTHER HALF OF 6.5.2'S CONDITION IS THE PROCESSOR'S, which is the "
                  "one thing the two architectures of this chip do not share: 'a processor "
                  "is asleep' is a signal the core releases, and a Cortex-M33 releases it "
                  "on any WFI while a Hazard3 releases it only with MSLEEP.POWERDOWN set - "
                  "measured, a wfi without that bit leaves this chip in its WAKE masks "
                  "however thoroughly SLEEP_ENx is pruned. The standby rung arms it and "
                  "the disarm takes it back",
                  releases_armed &&
                      sleep_releases_power_request() == (core_kind == CoreKind::cortex_m33));

    bench.verdict("DEEP IS REFUSED with no way back: no GPIO wake is enabled and no alarm "
                  "is armed, so the keyword would be the last instruction this chip ran",
                  !Plain::dormant_wake_ready() && !Plain::arm(SleepDepth::deep) &&
                      Plain::armed() == SleepDepth::none && P::sleep_hook == nullptr);

    (void)DormantWake::enable(Woken::number, pin_events(PinEvent::level_high));
    bench.verdict("with one GPIO wake enabled it arms, and the platform takes the hook "
                  "instead of its sleep instruction",
                  Plain::dormant_wake_ready() && Plain::arm(SleepDepth::deep) &&
                      P::sleep_hook != nullptr);
    Plain::disarm();
    DormantWake::disable_all();

    (void)AonTimer::alarm_in(60'000u);
    bench.verdict("THE ALARM IS A WAY BACK TOO - but not on the crystal, which is what this "
                  "site would stop: the timer on the crystal is refused and the same alarm "
                  "on the low-power oscillator is taken",
                  AonTimer::use_xosc() && !Plain::dormant_wake_ready() &&
                      AonTimer::use_lposc() && Plain::dormant_wake_ready());
    const bool on_crystal = AonTimer::use_xosc();
    const bool ring_takes_it = PlainRing::dormant_wake_ready();
    (void)AonTimer::use_lposc();
    bench.verdict("and on the RING-OSCILLATOR site either source will do, the crystal "
                  "going on running through that dormant",
                  on_crystal && ring_takes_it);
    AonTimer::alarm_enable(false);

    // The state machine's refusals, none of which writes anything.
    bench.verdict("request_state refuses a state with no processor in it: that is "
                  "power_down's, and its return is a boot",
                  Powman::request_state(PowerState::p1_0) == PowerRequest::illegal);
    bench.verdict("and power_down refuses a state that keeps the core running",
                  Powman::power_down(PowerState::p0_3) == PowerRequest::illegal);
    bench.verdict("A POWER-DOWN WITH NO ARMED WAKE IS REFUSED BY THE DRIVER, not by the "
                  "silicon: the silicon would take it and the board would be gone until "
                  "its supply was cycled",
                  !Powman::powerup_armed() &&
                      Powman::power_down(PowerState::p1_0) == PowerRequest::no_wake);
    bench.verdict("neither refusal touched the register: the state is still P0.0 and no "
                  "BAD_SW_REQ stands",
                  Powman::current_state() == PowerState::p0_0 &&
                      !Powman::bad_software_request());

    // WHICH CLOCK THE ALWAYS-ON BLOCK ITSELF RUNS ON, and whether a
    // running program may change it. 6.4 says the setting "takes effect
    // when a power up sequence is next run", which would make a dormant
    // that stops the crystal a dormant whose alarm cannot fire.
    const bool fast_at_rest = Powman::sequencer().using_fast_powck;
    (void)Powman::use_fast_clock(false);
    const bool still_fast = Powman::sequencer().using_fast_powck;
    (void)Powman::use_fast_clock(true);
    const bool fast_again = Powman::sequencer().using_fast_powck;
    print(serial, "  the block's own clock: USING_FAST_POWCK reads ", fast_at_rest ? 1 : 0,
          " at rest, ", still_fast ? 1 : 0, " with USE_FAST_POWCK written to 0, and ",
          fast_again ? 1 : 0, " with it written back to 1", crlf);
    bench.verdict("THE BLOCK RUNS ON clk_ref WHILE THE SWITCHED CORE IS UP, AND THE WRITE "
                  "THAT WOULD MOVE IT ONTO THE LOW-POWER OSCILLATOR TAKES EFFECT AT ONCE - "
                  "6.4 says the setting waits for the next power-up sequence, and the "
                  "report bit follows the write immediately, which is what lets a dormant "
                  "that stops the crystal keep this block alive",
                  fast_at_rest && !still_fast && fast_again);
    quiesce();
}

// =============================================================================
// g - a light sleep ended by the alarm
// =============================================================================
void tg_light() {
    AonTimer::interrupt(true);
    Irq::enable(AonTimer::irq());
    (void)Plain::arm(SleepDepth::light);

    site_alarm_fired = false;
    const uint64_t at = AonTimer::alarm_in(300u);
    const uint32_t u0 = us_now();
    const uint16_t turns = idle_until(site_alarm_fired);
    const uint32_t du = us_now() - u0;
    const uint32_t latency_ms = alarm_entry_ms - static_cast<uint32_t>(at);
    print(serial, "  light sleep: back after ", turns, " idle() turn(s) and ", du,
          " us; the handler entered ", latency_ms, " ms after the match", crlf);
    bench.verdict("the alarm ends a light sleep", site_alarm_fired);
    bench.verdict("A LIGHT SLEEP IS NOT ONE LONG STOP: the kernel's own tick ends a turn "
                  "every millisecond, so three hundred milliseconds are some three hundred "
                  "turns - which is what makes this rung the kernel's plain idle, named",
                  turns > 250u && turns < 360u);
    bench.verdict("and the wake is in the millisecond of the match", latency_ms <= 1u);
    quiesce();
}

// =============================================================================
// h - the SLEEP state, with a pruned PWM slice as the instrument
// =============================================================================
void th_sleep_state() {
    // The PWM block comes up HELD IN RESET, as every peripheral of this
    // chip does: the instrument has to be released before it counts.
    const bool block_ok = Pwm::reset();
    (void)Instrument::configure({.divider = *pwm_divider_of(4096), .top = 0xFFFF});
    Instrument::counter(0);
    Instrument::enable(true);
    (void)block_ok;
    wait_us(20'000u);
    const uint16_t awake_20ms = Instrument::counter();
    print(serial, "  awake, the instrument counts ", awake_20ms,
          " in 20 ms at clk_sys / 256", crlf);
    bench.verdict("the instrument runs while the program does", awake_20ms > 8000u);

    AonTimer::interrupt(true);
    Irq::enable(AonTimer::irq());
    Plain::standby_clocks(standby_gates);

    struct Leg {
        const char* name;
        uint16_t counted;
        uint32_t us;
        uint32_t ms;
        uint32_t ticks;
        uint16_t turns;
    };
    const auto leg = [](const char* name, SleepDepth d) {
        (void)Plain::arm(d);
        site_alarm_fired = false;
        Instrument::counter(0);
        const uint32_t u0 = us_now();
        const uint32_t m0 = ms_now();
        const uint32_t k0 = Ticker::ticks();
        (void)AonTimer::alarm_in(500u);
        const uint16_t turns = idle_until(site_alarm_fired);
        const uint16_t counted = Instrument::counter();
        const uint32_t u1 = us_now();
        const uint32_t m1 = ms_now();
        const uint32_t k1 = Ticker::ticks();
        Plain::disarm();
        return Leg{name, counted, u1 - u0, m1 - m0, k1 - k0, turns};
    };
    const Leg open = leg("gates open", SleepDepth::light);
    const Leg pruned = leg("gates pruned", SleepDepth::standby);
    Clocks::sleep_enables(sleep_clocks_all);

    for (const Leg& l : {open, pruned}) {
        print(serial, "  ", l.name, ": the instrument counted ", l.counted, " over ", l.ms,
              " ms (the ruler saw ", l.us, " us), the kernel tick moved ", l.ticks,
              ", ", l.turns, " idle() turn(s)", crlf);
    }
    bench.verdict("with every gate open a half-second sleep leaves the instrument counting "
                  "the whole time - some 24000 at clk_sys / 256",
                  open.counted > 20'000u);
    bench.verdict("WITH ITS GATE PRUNED THE INSTRUMENT ALL BUT STOPS: the chip reached the "
                  "SLEEP state and the PWM block had no clock in it, the handful of counts "
                  "left being the awake instants between the turns",
                  pruned.counted < open.counted / 8u);
    bench.verdict("the microsecond ruler counted the sleep through, its own gates having "
                  "been kept",
                  pruned.us > 490'000u && pruned.us < 520'000u);
    bench.verdict("and so did the always-on timer, which has no gate to keep",
                  pruned.ms >= 500u && pruned.ms < 520u);
    bench.verdict("THE KERNEL'S TIMEBASE COUNTS THROUGH THE SLEEP STATE ON BOTH HALVES - "
                  "SysTick being core-private with no gate in SLEEP_ENx at all, and the "
                  "platform timer's SIO and tick generator being in the set this suite "
                  "keeps - so kernel time never freezes in a standby here and a time event "
                  "matures on its own",
                  pruned.ticks >= 490u && pruned.ticks < 520u);
    bench.verdict("which is also why a standby is hundreds of turns and not one: the tick "
                  "ends each of them", pruned.turns > 400u);
    Instrument::enable(false);
    quiesce();
}

// =============================================================================
// j - a standby through the timed site
// =============================================================================
void tj_timed_standby() {
    Timed::standby_clocks(standby_gates);
    bench.verdict("the timed site's init() takes the alarm's line, the always-on timer "
                  "already running", Timed::init());

    Sleeper::alarm.arm(ticks_from_ms<P>(300));
    const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
    const uint32_t k0 = Ticker::ticks();
    const uint32_t m0 = ms_now();
    site_alarm_fired = false;
    const bool armed = Timed::arm(SleepDepth::standby);
    const bool placed = Timed::alarm_armed();
    const uint64_t at = AonTimer::alarm_time();
    const uint16_t turns = idle_until(site_alarm_fired);
    const uint32_t dk = Ticker::ticks() - k0;
    const uint32_t dm = ms_now() - m0;
    const SleepDepth after_alarm = Timed::armed();
    const uint32_t advanced = Timed::last_advance();
    // The alarm posts nothing to any queue and the never-early rounding
    // can leave kernel time a tick or two short of the deadline, so the
    // event is not due YET. The disarm above is what makes the next idle
    // a plain sleep the tick ends: these turns are that, counted.
    uint16_t extra = 0;
    std::optional<typename Sleeper::Event> due = Sleeper::queue.pop();
    while (!due.has_value() && extra < 50u) {
        {
            P::CriticalSection cs;
            P::idle();
        }
        ++extra;
        TimeEvents<P>::process();
        due = Sleeper::queue.pop();
    }
    print(serial, "  a time event ", next ? *next : 0u, " ticks out: the alarm placed at ",
          static_cast<uint32_t>(at), " ms, back after ", turns, " turn(s), ", dm,
          " ms and ", dk, " kernel ticks; the resync advanced ", advanced,
          "; the event matured ", extra, " tick(s) later", crlf);
    bench.verdict("the site arms and places the nearest deadline on the always-on alarm, "
                  "the ticks rounded UP into milliseconds",
                  armed && placed && next.has_value() &&
                      static_cast<uint32_t>(at) - m0 >= *next);
    bench.verdict("the alarm's body disarmed the site, so the next idle is a plain sleep "
                  "the tick ends", after_alarm == SleepDepth::none);
    bench.verdict("AND THE RESYNC HAD ALL BUT NOTHING TO CATCH UP - at most a tick, which "
                  "is the two rulers' phases and not a frozen timebase: SysTick counted "
                  "every millisecond of this standby for itself. The witness earns its "
                  "keep in a DORMANT, where the timebase stops outright",
                  advanced <= 2u);
    bench.verdict("THE EVENT MATURES A TICK OR TWO AFTER THE ALARM AND NOT ON IT - the "
                  "alarm is placed in the always-on timer's milliseconds, rounded up, and "
                  "the two rulers' phases differ by up to a tick each. That is what the "
                  "disarm in the alarm's body is for: the machine is handed back to a "
                  "ticking sleep and the tick finishes the job",
                  due.has_value() && extra <= 5u);
    quiesce();
}

// =============================================================================
// k - the scratch words, the boot words, and the two domains
// =============================================================================
void tk_scratch() {
    for (uint8_t n = 0; n < 8u; ++n) {
        Powman::scratch(n, 0xA5A5'0000u | n);
    }
    bool all_back = true;
    for (uint8_t n = 0; n < 8u; ++n) {
        if (Powman::scratch(n) != (0xA5A5'0000u | n)) { all_back = false; }
    }
    print(serial, "  the always-on block's eight words read back ", hex(Powman::scratch(0)),
          " .. ", hex(Powman::scratch(7)), crlf);
    bench.verdict("eight words take a value and give it back", all_back);
    bench.verdict("a ninth does not exist and answers zero", Powman::scratch(8) == 0u);
    bench.verdict("the four boot-vector words are read and never written here: they are the "
                  "bootrom's, and a magic pair in them would divert the next boot",
                  Powman::boot_word(0) == 0u && Powman::boot_word(3) == 0u);
    print(serial, "  THE TWO DOMAINS: these eight words are in the ALWAYS-ON one and the "
          "watchdog's four are in the switched core - which letter q measures across a "
          "power-down", crlf);
    for (uint8_t n = 0; n < 8u; ++n) {
        Powman::scratch(n, 0u);
    }
}

// =============================================================================
// m - the manager over the timed site
// =============================================================================
struct Requester {
    struct Event { SleepVote vote; };
    static inline EventQueue<Event, 4, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

using Power = PowerManager<P, Timed, PowerConfig{}>;

/// One round: the request posted, the manager stepped until its queue is
/// empty, the answer taken.
bool round(SleepDepth d) {
    post<Power>(SleepRequested{d, reply_to<Requester, SleepVote>()});
    while (const std::optional<typename Power::Event> e = Power::queue.pop()) {
        Power::dispatch(*e);
    }
    const std::optional<Requester::Event> a = Requester::queue.pop();
    return a.has_value() && a->vote.ok;
}

void tm_manager() {
    Timed::standby_clocks(standby_gates);
    (void)Timed::init();
    Power::init();
    bench.verdict("the manager starts with nothing armed",
                  Power::armed_depth() == SleepDepth::none && Timed::armed() == SleepDepth::none);

    bench.verdict("a request for none is accepted and arms nothing",
                  round(SleepDepth::none) && Power::armed_depth() == SleepDepth::none);
    bench.verdict("a request for light is accepted and the site takes it",
                  round(SleepDepth::light) && Timed::armed() == SleepDepth::light);
    // The first event after a wake disarms: any event does.
    post<Power>(SleepVote{true});
    while (const std::optional<typename Power::Event> e = Power::queue.pop()) {
        Power::dispatch(*e);
    }
    bench.verdict("and the first event after the wake disarms it",
                  Timed::armed() == SleepDepth::none);

    bench.verdict("a request for standby is accepted, the gates written",
                  round(SleepDepth::standby) && Timed::armed() == SleepDepth::standby &&
                      Clocks::sleep_enables() == (standby_gates | Timed::alarm_gates));
    Timed::disarm();
    Power::init();

    bench.verdict("A REQUEST FOR DEEP IS REFUSED with no way back armed, and the refusal "
                  "reaches the requester",
                  !round(SleepDepth::deep) && Power::armed_depth() == SleepDepth::none);

    PowerLock lock = Power::restrict(SleepDepth::light);
    bench.verdict("a standing restriction clamps the round to light",
                  Power::ceiling() == SleepDepth::light && round(SleepDepth::standby) &&
                      Timed::armed() == SleepDepth::light);
    lock.release();
    Timed::disarm();
    Power::init();

    Sleeper::alarm.arm(1);
    bench.verdict("THE DEADLINE GUARD refuses a deep round whose next armed time event is "
                  "nearer than the wake would cost", !round(SleepDepth::standby));
    Sleeper::alarm.disarm();
    (void)Sleeper::queue.pop();
    bench.verdict("and takes it once the deadline is far enough out",
                  round(SleepDepth::standby));
    quiesce();
    Power::init();
}

// =============================================================================
// n, o, p - the dormant letters (by name)
// =============================================================================

/// What one stop measured.
struct DormantSpan {
    uint32_t ms = 0;          ///< the stop itself, by the always-on timer
    uint32_t us = 0;          ///< the same stop by the crystal's ruler, which may not count it
    uint32_t since_arm = 0;   ///< from the alarm's placing to the return, in milliseconds
    bool returned = false;
};

/// The shape every dormant letter shares: announce, flush, stop, report.
/// `wake_ms` is the alarm's distance, 0 for none.
DormantSpan dormant_leg(const char* what, bool ring, uint32_t wake_ms) {
    AonTimer::interrupt(true);
    Irq::enable(AonTimer::irq());
    const uint32_t armed_at = ms_now();
    if (wake_ms != 0u) {
        site_alarm_fired = false;
        (void)AonTimer::alarm_in(wake_ms);
        // THE POWER-UP DUTY AS WELL AS THE INTERRUPT. 6.5.3.1 says the
        // interrupt output need not be enabled for a dormant's wake, and
        // says nothing about this bit; the vendor's own library sets it
        // whenever an alarm is meant to end a low-power state, and it
        // has no effect at all while the core is powered.
        AonTimer::powerup_on_alarm(true);
    }
    const bool ready = ring ? PlainRing::dormant_wake_ready() : Plain::dormant_wake_ready();
    const bool armed = ring ? PlainRing::arm(SleepDepth::deep) : Plain::arm(SleepDepth::deep);
    print(serial, "  ", what, ": wake ready=", ready ? 1 : 0, ", armed=", armed ? 1 : 0,
          ", the alarm ", wake_ms, " ms out; the block's own clock is ",
          Powman::sequencer().using_fast_powck ? "clk_ref" : "the low-power oscillator",
          "; stopping the ", ring ? "ring oscillator" : "crystal", " now ...", crlf);
    report_debug_condition();
    console_drain();

    const uint32_t m0 = ms_now();
    const uint32_t u0 = us_now();
    const uint32_t before = ring ? PlainRing::dormants() : Plain::dormants();
    {
        P::CriticalSection cs;
        P::idle();
    }
    DormantSpan span{};
    span.ms = ms_now() - m0;
    span.us = us_now() - u0;
    span.since_arm = ms_now() - armed_at;
    const uint32_t after = ring ? PlainRing::dormants() : Plain::dormants();
    span.returned = after == before + 1u;
    if (ring) { PlainRing::disarm(); } else { Plain::disarm(); }

    print(serial, "  back: the stop itself was ", span.ms, " ms by the always-on timer, of "
          "which the crystal's ruler counted ", span.us, " us; ", span.since_arm,
          " ms since the alarm was placed; dormants ", before, " -> ", after,
          ", the crystal stable=", Xosc::stable() ? 1 : 0, ", the system PLL locked=",
          PllSys::locked() ? 1 : 0, ", the block's own clock is ",
          Powman::sequencer().using_fast_powck ? "clk_ref" : "the low-power oscillator",
          ", clk_sys by the counter ",
          SysClock::count_hz(CountSource::clk_sys).value_or(0u), " Hz", crlf);
    bench.verdict("the chip came back, and the tree is the one the program asked for: the "
                  "crystal stable, the PLL relocked and clk_sys at its rate - which is the "
                  "program's own Clock::init() run inside the hook, before the wake's "
                  "handler ever ran",
                  span.returned && Xosc::stable() && PllSys::locked() &&
                      SysClock::count_hz(CountSource::clk_sys).value_or(0u) >
                          SysClock::hz - SysClock::hz / 100u);
    AonTimer::alarm_enable(false);
    return span;
}

void tn_dormant_alarm() {
    print(serial, "  a DORMANT on the crystal, the always-on timer's alarm the only way "
          "back. Every clock the program runs on stops and the console goes silent; "
          "clk_ref is left on the LOW-POWER OSCILLATOR, which is what the alarm needs.",
          crlf);
    const DormantSpan s = dormant_leg("dormant/crystal/alarm", false, 3000u);
    bench.verdict("THE ALARM ENDS A DORMANT THAT STOPPED THE CRYSTAL - but only because "
                  "clk_ref was taken to the low-power oscillator first and clk_sys to the "
                  "crystal through its own aux mux. Measured three ways over, the same "
                  "alarm does NOT end a dormant entered with clk_ref on the crystal: not "
                  "with the interrupt enabled, not with SEQ_CFG.USE_FAST_POWCK written to "
                  "0 (which the block obeys at once), and not with PWRUP_ON_ALARM set as "
                  "well. THE WAKE PATH NEEDS clk_ref RUNNING, and only the clock tree can "
                  "give it that",
                  s.returned && s.since_arm >= 3000u && s.since_arm < 3300u);
    bench.verdict("AND THE CRYSTAL'S RULER COUNTED ALMOST NONE OF IT - which is what makes "
                  "the always-on timer the only witness a dormant has, and TIMER0 useless "
                  "for one", s.us < 50'000u);
    quiesce();
}

void to_dormant_gpio() {
    print(serial, "  a DORMANT on the crystal ended by a GPIO WAKE on the board's own "
          "standing wire (GP19 -> GP8): the pad is driven HIGH before the keyword, so the "
          "level is standing as the clocks stop. NO ALARM IS ARMED - this letter is the "
          "wake path that needs no clock at all, and it is the one proven first.", crlf);
    (void)Woken::input(PinPull::down);
    (void)Waker::output(true);
    wait_us(1000u);
    const bool pad_high = Woken::read();
    (void)DormantWake::enable(Woken::number, pin_events(PinEvent::level_high));
    bench.verdict("a GPIO wake armed on the pad is a way back the site accepts, and the "
                  "wire carries the level",
                  pad_high && DormantWake::any_enabled() && Plain::dormant_wake_ready());

    const DormantSpan roused = dormant_leg("dormant/pad high", false, 0u);
    print(serial, "  the wake logic's raw flags on the pad: ",
          DormantWake::raw(Woken::number).has(PinEvent::level_high) ? "level_high" : "none",
          crlf);
    bench.verdict("THE PAD IS THE WAKE, AND IT NEEDS NO CLOCK AT ALL: with the level "
                  "standing as the keyword is written the chip comes straight back, with "
                  "no alarm armed anywhere - which is what makes a GPIO event the one way "
                  "out of a dormant that stops the crystal",
                  roused.returned && roused.ms < 200u);
    (void)Waker::output(false);
    DormantWake::disable_all();
    quiesce();
}

void tp_dormant_ring() {
    print(serial, "  a DORMANT on the RING OSCILLATOR: the crystal goes on running through "
          "it, so the always-on timer may count either source - and the restart is a "
          "microsecond against the crystal's millisecond.", crlf);
    const DormantSpan s = dormant_leg("dormant/ring/alarm", true, 3000u);
    bench.verdict("the alarm ends it, three seconds after it was placed",
                  s.returned && s.since_arm >= 3000u && s.since_arm < 3300u);
    bench.verdict("AND THE CRYSTAL'S RULER STOOD STILL HERE TOO, though the crystal never "
                  "stopped: clk_sys was taken to the ring oscillator and the tick generator "
                  "behind TIMER0 counts clk_ref only while the block it feeds is running - "
                  "so the microsecond ruler is no witness of a dormant of either kind",
                  s.us < 50'000u);
    quiesce();
}

// =============================================================================
// q - the power-down states (by name; each wake is a boot)
// =============================================================================
inline constexpr uint32_t token_magic = 0x5150'0000u;

uint8_t powman_leg() {
    const uint32_t w = Powman::scratch(0);
    return (w & 0xFFFF'0000u) == token_magic ? static_cast<uint8_t>(w & 0xFFu) : 0u;
}
uint8_t watchdog_leg() {
    const uint32_t w = Scratch<0>::read();
    return (w & 0xFFFF'0000u) == token_magic ? static_cast<uint8_t>(w & 0xFFu) : 0u;
}
uint8_t noinit_leg() {
    return noinit_token.magic == noinit_magic ? static_cast<uint8_t>(noinit_token.leg) : 0u;
}

void token_set(uint8_t leg) {
    const uint32_t tally = (static_cast<uint32_t>(bench.passed()) << 16) | bench.failed();
    Powman::scratch(1, tally);
    Powman::scratch(0, token_magic | leg);
    Scratch<1>::write(tally);
    Scratch<0>::write(token_magic | leg);
    noinit_token = NoinitToken{noinit_magic, leg, tally};
}
void token_clear() {
    Powman::scratch(0, 0u);
    Scratch<0>::write(0u);
    noinit_token.magic = 0u;
}

/// After an accepted power-down request the sequencer holds WAITING
/// until the processors halt, so the core sleeps. It does not come back
/// from here: what comes back is a BOOT. A bounded wait and a diagnosis
/// are what this does when it DOES come back.
[[noreturn]] void await_wake(const char* what) {
    console_drain();
    const uint32_t t0 = ms_now();
    while (ms_now() - t0 < 20'000u) {
        P::CriticalSection cs;
        P::idle();
    }
    print(serial, "  ...20 s and still running ", what, ": STATE=",
          hex(Powman::state_word()), " waiting=", Powman::waiting() ? 1 : 0,
          " pwrup_while_waiting=", Powman::powerup_while_waiting() ? 1 : 0, " req_ignored=",
          Powman::request_ignored() ? 1 : 0, " current_pwrup=",
          hex(Powman::current_powerup_requests()), crlf);
    token_clear();
    bench.verdict("the sequencer took the switched core down", false);
    bench.end_letter();
    bench.prompt();
    for (;;) {
        uint8_t c = 0;
        if (Serial::read_byte(c) && c != '\r' && c != '\n') {
            print(serial, static_cast<char>(c), crlf);
            if (!bench.handle(static_cast<char>(c))) {
                print(serial, "unknown letter (? for the menu)", crlf);
            }
            bench.prompt();
        }
    }
}

/// One power-down leg: the token banked, the alarm armed as a POWER-UP
/// request, the state asked for. It does not return when it works.
[[noreturn]] void power_down_leg(uint8_t leg, PowerState to, uint32_t wake_ms) {
    token_set(leg);
    print(serial, "  leg ", leg, ": powering the switched core down into P1.",
          static_cast<int>(state_code(to) & 7u), ", the alarm ", wake_ms,
          " ms out as the power-up request. THE WAY BACK IS A BOOT.", crlf);
    const bool clear = clear_debug_block();
    print(serial, "  the debug port ", clear ? "is out of the way" : "STILL BLOCKS THIS",
          crlf);
    console_drain();

    AonTimer::interrupt(false);
    Irq::disable(AonTimer::irq());
    (void)AonTimer::alarm_in(wake_ms);
    AonTimer::powerup_on_alarm(true);
    const PowerRequest r = Powman::power_down(to);
    if (r != PowerRequest::accepted) {
        print(serial, "  the request was refused: ", static_cast<int>(r),
              " (0 accepted, 1 illegal, 2 no wake, 3 busy, 4 ignored, 5 bad) - "
              "REQ_IGNORED=", Powman::request_ignored() ? 1 : 0, ", BAD_SW_REQ=",
              Powman::bad_software_request() ? 1 : 0, crlf);
        AonTimer::powerup_on_alarm(false);
        AonTimer::alarm_enable(false);
    }
    // The sequencer holds WAITING until the processors halt.
    await_wake("(the sequencer did not take the core down)");
}

void report_boot() {
    print(serial, "  this boot: LAST_SWCORE_PWRUP=", hex(boot_last_pwrup), " (",
          boot_last_pwrup == pwrup_bit(PowerUpSource::alarm) ? "the alarm"
                                                             : "not the alarm",
          "), causes=", hex(boot_causes), ", survivors: POWMAN scratch ",
          boot_powman_alive ? "YES" : "no", ", watchdog scratch ",
          boot_scratch_alive ? "YES" : "no", ", .noinit ", boot_noinit_alive ? "YES" : "no",
          crlf);
}

void tq_power_down() {
    report_boot();
    bench.verdict("no token is pending on a clean start",
                  !boot_powman_alive && !boot_scratch_alive);
    power_down_leg(1, PowerState::p1_0, 3000u);
}

void tr_power_down_deepest() {
    report_boot();
    bench.verdict("no token is pending on a clean start",
                  !boot_powman_alive && !boot_scratch_alive);
    power_down_leg(2, PowerState::p1_7, 3000u);
}

void tq_resume() {
    const uint8_t leg = boot_powman_alive ? powman_leg()
                                          : (boot_noinit_alive ? noinit_leg() : watchdog_leg());
    const uint32_t tally = boot_powman_alive ? Powman::scratch(1) : noinit_token.tally;
    bench.resume_tally(static_cast<uint16_t>(tally >> 16), static_cast<uint16_t>(tally & 0xFFFFu));
    print(serial, crlf, leg == 2u ? "r" : "q", " (continued after the power-down into P1.",
          leg == 2u ? 7 : 0, ")", crlf);
    report_boot();

    bench.verdict("THE ALWAYS-ON BLOCK'S SCRATCH WORDS CROSSED A POWER-DOWN OF THE "
                  "SWITCHED CORE - which is what they are for, and the only state of this "
                  "program that did",
                  boot_powman_alive);
    bench.verdict("AND THE WATCHDOG'S DID NOT: those four registers live in the switched "
                  "core, which had no supply at all",
                  !boot_scratch_alive);
    if (leg == 2u) {
        bench.verdict("AND NEITHER DID THE SRAM: P1.7 leaves both memory domains unpowered, "
                      "so the .noinit words the crt never touches came back from a domain "
                      "that had no supply - undefined, which here reads as not the token",
                      !boot_noinit_alive);
    } else {
        bench.verdict("THE SRAM DID CROSS IT, because P1.0 is the state that keeps every "
                      "memory powered while the logic around it goes: the .noinit words "
                      "carried the same token the always-on block did",
                      boot_noinit_alive && noinit_leg() == powman_leg());
    }
    bench.verdict("the power manager names the alarm as what powered the core back up",
                  boot_last_pwrup == pwrup_bit(PowerUpSource::alarm));
    bench.verdict("and the chip-level record names the power-down as the reset's source, "
                  "which is 7.3's own HAD_SWCORE_PD",
                  (boot_causes & ResetCause::swcore_powerdown) != 0u);
    bench.verdict("the power state is P0.0 again: the sequencer powered every memory domain "
                  "back up on the way, SEQ_CFG saying so",
                  Powman::current_state() == PowerState::p0_0);

    if (leg == 2u) {
        bench.verdict("P1.7 IS THE DEEPEST STATE THIS CHIP HAS - the switched core, the XIP "
                      "cache and both SRAM domains unpowered, the always-on domain and its "
                      "timer the only thing left - and it comes back the same way, by the "
                      "alarm, through a full boot",
                      boot_last_pwrup == pwrup_bit(PowerUpSource::alarm));
    }
    token_clear();
    print(serial, "  letter done", crlf);
    bench.end_letter();
}

// =============================================================================
// u, v - the rescue nets (by name)
// =============================================================================
void tu_rescue_dormant() {
    print(serial, "  THE RESCUE NET, dormant: a DORMANT on the crystal with a SIXTY SECOND "
          "alarm under it. The crystal, the PLLs and the core stop; a rescue over the debug "
          "port may be tried now, and the alarm brings the chip back either way.", crlf);
    const DormantSpan s = dormant_leg("dormant/crystal/60s", false, 60'000u);
    bench.verdict("THE NET HELD: the chip came back from a dormant that stopped the "
                  "crystal, the PLLs and the core, by its own alarm and with no probe "
                  "needed - a minute later, to the millisecond",
                  s.returned && s.since_arm >= 60'000u && s.since_arm < 60'300u);
    quiesce();
}

void tv_rescue_power_down() {
    print(serial, "  THE RESCUE NET, powered down: P1.0 with a SIXTY SECOND alarm under it. "
          "The switched core loses its supply; a rescue over the debug port may be tried "
          "now, and the alarm boots the chip again either way.", crlf);
    token_set(9);
    const bool clear = clear_debug_block();
    print(serial, "  the debug port ", clear ? "is out of the way" : "STILL BLOCKS THIS", crlf);
    console_drain();
    AonTimer::interrupt(false);
    Irq::disable(AonTimer::irq());
    (void)AonTimer::alarm_in(60'000u);
    AonTimer::powerup_on_alarm(true);
    const PowerRequest r = Powman::power_down(PowerState::p1_0);
    if (r != PowerRequest::accepted) {
        print(serial, "  the request was refused: ", static_cast<int>(r), crlf);
    }
    await_wake("(the sequencer did not take the core down)");
}

/// The rescue net's own resume: leg 9 is not one of q's, and it says
/// only what it set out to say.
void tv_resume() {
    print(serial, crlf, "v (continued: the chip came back from P1.0)", crlf);
    report_boot();
    print(serial, "  THE RESCUE NET HELD: the board is answering again, whether the rescue "
          "was run against it or the sixty-second alarm was what booted it - the power "
          "manager's own LAST_SWCORE_PWRUP above says which", crlf);
    token_clear();
}

void banner() {
    print(serial, crlf, "test_rp2350_sleep - the always-on power manager, its timer and the "
          "sleep sites (datasheet 6, 12.10) on ", arch_name(), ", clk_sys=", SysClock::hz,
          " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------

// THE TIMED SITE'S BODY, which is the always-on timer's own body plus
// the resync and the disarm: an app binds this one name and gets both,
// and a letter that uses the plain site gets the timer's half of it with
// a resync that has nothing to catch up.
extern "C" void isr_powman_timer() {
    alarm_entry_ms = brio::AonTimer::now_low();
    alarm_entry_us = brio::Timer<0>::now_low();
    if (Timed::isr()) {
        alarm_fires = static_cast<uint8_t>(alarm_fires + 1u);
        site_alarm_fired = true;
    }
}

extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_hardfault() { brio::fault_reset<P>(0x51); }
extern "C" void isr_riscv_exception() { brio::fault_reset<P>(0x51); }

int main() {
    boot_causes = brio::Reset::causes();
    boot_last_pwrup = brio::Powman::last_powerup_requests();
    boot_powman_alive = powman_leg() != 0u;
    boot_scratch_alive = watchdog_leg() != 0u;
    boot_noinit_alive = noinit_leg() != 0u;

    const bool clock_ok = SysClock::init();
    const bool t0_ok = brio::Timer<0>::init(clock);
    const bool aon_ok = brio::AonTimer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output(false);
    boot_debug_pwrup = brio::Powman::debug_powerup_pending();
    brio::enable_interrupts();

    bench.letter('a', "the always-on block as found", ta_as_found);
    bench.letter('b', "the password", tb_password);
    bench.letter('c', "the timer and the oscillator it counts", tc_timer);
    bench.letter('d', "the alarm as an interrupt", td_alarm);
    bench.letter('e', "the tick source, both ways", te_source);
    bench.letter('f', "the ladder, the gates and the refusals", tf_ladder);
    bench.letter('g', "a light sleep ended by the alarm", tg_light);
    bench.letter('h', "THE SLEEP STATE, with a pruned instrument", th_sleep_state);
    bench.letter('j', "a standby through the timed site", tj_timed_standby);
    bench.letter('k', "the scratch words and the two domains", tk_scratch);
    bench.letter('m', "the manager over the timed site", tm_manager);
    bench.letter('n', "A DORMANT woken by the alarm", tn_dormant_alarm, false);
    bench.letter('o', "A DORMANT woken by a pad", to_dormant_gpio, false);
    bench.letter('p', "A DORMANT on the ring oscillator", tp_dormant_ring, false);
    bench.letter('q', "P1.0: the core powered down (reboots the board)", tq_power_down, false);
    bench.letter('r', "P1.7: the deepest state (reboots the board)", tr_power_down_deepest,
                 false);
    bench.letter('u', "THE RESCUE NET, dormant (60 s)", tu_rescue_dormant, false);
    bench.letter('v', "THE RESCUE NET, powered down (60 s)", tv_rescue_power_down, false);

    if (serial_ok && (boot_powman_alive || boot_noinit_alive) && powman_leg() == 9u) {
        banner();
        tv_resume();
        bench.prompt();
    } else if (serial_ok && (boot_powman_alive || boot_noinit_alive)) {
        tq_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", t0_ok ? "1us" : "FAILED", " aon=", aon_ok ? "1ms" : "FAILED",
                    " tick=", tick_ok ? "on" : "FAILED", " debug_pwrup=",
                    boot_debug_pwrup ? "STANDING" : "no", brio::crlf);
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
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
