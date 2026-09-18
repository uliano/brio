// test_rp2350_timer - the reference bench suite for this chip's TIME
// AND ITS ENDINGS: the two system timers (datasheet 12.8) with the tick
// generators that feed them (8.5), the watchdog (12.9), and the resets
// chapter whole (7) - the causes in the always-on power manager, the
// power-on state machine a reboot runs, the subsystem reset controller,
// and the two verbs a program has for ending itself.
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets; a verdict that must differ between them says
// so in its own text, and there is exactly one - the fifth leg of the
// reset letter, a processor's own reset request, which the RISC-V half
// has not got because its hart reset controls are the Debug Module's.
//
// NOTHING TO WIRE. The console is the Debug Probe's UART bridge on GP0
// (TX) / GP1 (RX) = UART0, and every counter this suite measures is
// inside the chip.
//
// THE RULER IS TIMER0 (rp2350/timer.hpp): 64 bits of microseconds on a
// tick generator dividing clk_ref, which the clock task puts on the
// crystal undivided - independent of clk_sys, so it judges the other
// counters, the watchdog's decrement rate and the arithmetic of both.
// It shares the crystal with them, so the crystal's own accuracy is
// nobody's verdict here. TIMER1 is the SECOND ruler, and because
// nothing else depends on it, it is the instance every destructive
// letter uses: paused, taken off the tick, locked.
//
// What is exercised, letter by letter:
//   a  the boot story: the chip and its stepping, the architecture, the
//      causes word read twice, the six tick generators, what survived
//      the previous boot, no breadcrumb pending
//   b  three microsecond counters against each other - TIMER0, TIMER1
//      and the platform timer in the SIO - over 200 ms
//   c  the 64-bit read: the raw pair against the latching pair on both
//      instances, and 2000 reads that never go backwards
//   d  TIMER0's four alarms, each on its own interrupt line, with the
//      handler's entry latency measured per alarm
//   e  TIMER1's four alarms, the same on the second instance: lines
//      4..7, which the RP2040 had not
//   f  PAUSE and DBGPAUSE on TIMER1: the count stopped and taken up
//      again, and the two debug bits init() clears
//   g  SOURCE on TIMER1: the counter taken off the tick and onto
//      clk_sys, the ratio against TIMER0 measured
//   h  the tick generators of 8.5: which run, at what divisor, and
//      TIMER1's stopped and restarted with its counter watched
//   j  the watchdog countdown measured against the ruler - the
//      decrement rate as a NUMBER, which is what answers "does this
//      chip have the RP2040's double decrement" - the kick, the
//      remaining time, the tick generator behind it
//   k  LOCKED on TIMER1: the bit 12.8 calls a write guard, tried against
//      four kinds of write, and the subsystem reset controller that is
//      the only thing which takes it down again
//   m  the scratch registers and the atomic register aliases on one
//   n  the panic breadcrumb without a reset
//
//   i  (by name only) FIVE REAL RESETS on the Arm half and four on the
//      RISC-V one. This letter reboots the board once per leg and
//      resumes from a token, so it is NOT in `z`: `z` has to be one
//      console session a tool can judge from a single capture. Run it
//      with
//          brio run <board> i --app test_rp2350_timer
//                  --expect="every leg done" --timeout 120
//      Legs: Reset::software() (the watchdog's trigger under the
//      power-on state machine), a watchdog time-out at 100 ms, a panic
//      through ResetReporter, a breakpoint with no debugger attached
//      through the crt's fault entry, and Reset::core() (the Arm half's
//      SYSRESETREQ). THE TOKEN IS CARRIED TWICE, in the watchdog's
//      scratch registers and in a .noinit record, because which of the
//      two crosses which reset is a MEASUREMENT here: 7.2 says the
//      scratch words are kept by a system reset and lost to a
//      chip-level one, 7.3.1's table lists the watchdog's own PSM reset
//      among the chip-level causes, and the two sentences do not agree.
//      Every leg reports which survivor it found.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/panic.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/reset.hpp"
#include "rp2350/resets.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "rp2350/watchdog.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = brio::Rp2350Platform<>;

// The token letter i resumes from, carried in SRAM the crt never
// touches. Inline, for the reason the other strata's suites give: gcc
// gives an inline variable with a section attribute a COMDAT group like
// the platform's own panic_record_, and a plain one would clash.
struct NoinitToken {
    uint32_t magic;
    uint32_t leg;
    uint32_t tally;
    uint32_t expect;
    uint32_t before;
};
[[gnu::section(".noinit")]] inline NoinitToken noinit_token;
inline constexpr uint32_t noinit_magic = 0xC3A5F00Du;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;          // the WeAct board's LED: a keystroke marker
using Ruler = Timer<0>;       // the microsecond ruler every letter is judged by
using Spare = Timer<1>;       // the instance a destructive letter may disturb

TestBench<Serial> bench;

// What this boot was told, sampled once in main() before anything can
// disturb it.
uint32_t boot_causes = 0;
std::optional<PanicRecord> boot_record;
bool boot_scratch_alive = false;
bool boot_noinit_alive = false;

uint32_t us_now() { return Ruler::now_low(); }

/// Let the console's own transmitter fall silent: its interrupt would
/// otherwise wake every idle() below, and a reboot must not cut a line
/// in half.
void console_drain() {
    const uint32_t t0 = us_now();
    while (!Serial::tx_idle() && us_now() - t0 < 200'000u) {
    }
}

/// Spin on the ruler. Every wait in this suite is bounded by it and not
/// by the kernel tick, which is a different counter on each half.
void wait_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

// ---- the token letter i lives in ------------------------------------------
//
// TWO COPIES, because which one crosses which reset is the measurement.
// Scratch<0> = magic (high half) | leg (low byte); Scratch<1> = the
// tally; Scratch<2> = the panic code and context the leg expects at the
// next boot; Scratch<3> = the causes word standing when the leg started.
// The .noinit record carries the same five values.
inline constexpr uint32_t token_magic = 0x5A23'0000u;

uint8_t scratch_leg() {
    const uint32_t w = Scratch<0>::read();
    return (w & 0xFFFF'0000u) == token_magic ? static_cast<uint8_t>(w & 0xFFu) : 0u;
}
uint8_t noinit_leg() {
    return noinit_token.magic == noinit_magic ? static_cast<uint8_t>(noinit_token.leg) : 0u;
}

void token_set(uint8_t leg, uint8_t code = 0, uint8_t context = 0) {
    const uint32_t tally = (static_cast<uint32_t>(bench.passed()) << 16) | bench.failed();
    const uint32_t expect = (static_cast<uint32_t>(code) << 8) | context;
    Scratch<1>::write(tally);
    Scratch<2>::write(expect);
    Scratch<3>::write(boot_causes);
    Scratch<0>::write(token_magic | leg);
    noinit_token = NoinitToken{noinit_magic, leg, tally, expect, boot_causes};
}
void token_clear() {
    Scratch<0>::write(0);
    noinit_token.magic = 0;
}

void print_causes(uint32_t c) {
    if (c == 0u) {
        print(serial, "none");
        return;
    }
    if (c & ResetCause::power_on) print(serial, "POR ");
    if (c & ResetCause::brown_out) print(serial, "BOR ");
    if (c & ResetCause::run_pin) print(serial, "RUN ");
    if (c & ResetCause::debug_port) print(serial, "DP_REQ ");
    if (c & ResetCause::rescue) print(serial, "RESCUE ");
    if (c & ResetCause::watchdog_powman_async) print(serial, "WD_POWMAN_ASYNC ");
    if (c & ResetCause::watchdog_powman) print(serial, "WD_POWMAN ");
    if (c & ResetCause::watchdog_swcore) print(serial, "WD_SWCORE ");
    if (c & ResetCause::swcore_powerdown) print(serial, "SWCORE_PD ");
    if (c & ResetCause::glitch_detect) print(serial, "GLITCH ");
    if (c & ResetCause::hazard3_sys_reset) print(serial, "HZD_SYS_REQ ");
    if (c & ResetCause::watchdog_psm) print(serial, "WD_PSM ");
    if (c & ResetCause::watchdog_timer) print(serial, "REASON_TIMER ");
    if (c & ResetCause::watchdog_force) print(serial, "REASON_FORCE ");
}

const char* arch_name() { return core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33"; }

void report_boot() {
    print(serial, "  causes: ");
    print_causes(boot_causes);
    print(serial, "(", hex(boot_causes), "); survivors: scratch ",
          boot_scratch_alive ? "YES" : "no", ", .noinit ", boot_noinit_alive ? "YES" : "no",
          "; breadcrumb ");
    if (boot_record) {
        print(serial, "code ", boot_record->code, " context ", hex(boot_record->context));
    } else {
        print(serial, "none");
    }
    print(serial, crlf);
}

// =============================================================================
// a - the boot story
// =============================================================================
void ta_boot() {
    const ChipId id = ChipId::read();
    print(serial, "  chip: manufacturer ", hex(id.manufacturer), " part ", hex(id.part),
          " revision ", hex(id.revision), ", package ",
          ChipId::package_sel() == Package::qfn80 ? 80 : 60, " pins, running ", arch_name(),
          " core ", P::core_id(), crlf);
    bench.verdict("SYSINFO names Raspberry Pi's RP2350",
                  id.manufacturer == ChipId::raspberry_pi && id.part == ChipId::rp2350);

    report_boot();
    const uint32_t c1 = Reset::causes();
    const uint32_t c2 = Reset::causes();
    bench.verdict("the causes are unchanged by reading them",
                  c1 == c2 && c1 == boot_causes);
    bench.verdict("the power manager names a chip-level source for this boot's lineage: "
                  "the board was powered on, and the flash verb's rescue is a chip-level "
                  "reset of its own",
                  (boot_causes & ResetCause::chip_level) != 0u);
    bench.verdict("the rescue flag is down when user code runs: the bootrom reads it "
                  "first and clears it to acknowledge",
                  !Reset::rescue_flag());

    print(serial, "  RESET_DONE=", hex(RESETS->RESET_DONE), " PSM DONE=", hex(Psm::done()),
          " PSM WDSEL=", hex(Psm::watchdog_resets()), " FRCE_OFF=", hex(Psm::held()), crlf);
    bench.verdict("both system timers, the IO and pad banks and UART0 are out of reset, "
                  "released by the drivers this suite started",
                  Resets::released(ResetBlock::timer0 | ResetBlock::timer1 |
                                   ResetBlock::io_bank0 | ResetBlock::pads_bank0 |
                                   ResetBlock::uart0));
    bench.verdict("the power-on state machine holds no stage down - which is also "
                  "erratum RP2350-E19's condition for a reboot that completes",
                  (Psm::held() & ~PsmStage::proc1) == 0u);

    constexpr uint32_t expect_cycles = SysClock::ref_hz / 1'000'000u;
    print(serial, "  tick generators (cycles of clk_ref per microsecond, - = stopped): "
          "proc0 ");
    const auto show = [](bool running, uint32_t cycles) {
        if (running) { print(serial, cycles, " "); } else { print(serial, "- "); }
    };
    show(TickGenerator<TickConsumer::proc0>::running(),
         TickGenerator<TickConsumer::proc0>::cycles());
    print(serial, "proc1 ");
    show(TickGenerator<TickConsumer::proc1>::running(),
         TickGenerator<TickConsumer::proc1>::cycles());
    print(serial, "timer0 ");
    show(Ruler::Tick::running(), Ruler::Tick::cycles());
    print(serial, "timer1 ");
    show(Spare::Tick::running(), Spare::Tick::cycles());
    print(serial, "watchdog ");
    show(Watchdog::Tick::running(), Watchdog::Tick::cycles());
    print(serial, "riscv ");
    show(TickGenerator<TickConsumer::riscv>::running(),
         TickGenerator<TickConsumer::riscv>::cycles());
    print(serial, crlf);
    bench.verdict("every generator this suite started divides clk_ref by the crystal's "
                  "megahertz",
                  Ruler::Tick::running() && Ruler::Tick::cycles() == expect_cycles &&
                      Spare::Tick::running() && Spare::Tick::cycles() == expect_cycles &&
                      Watchdog::Tick::running() && Watchdog::Tick::cycles() == expect_cycles);
    bench.verdict("no breadcrumb is pending on a clean start", !boot_record);
}

// =============================================================================
// b - three microsecond counters against each other
// =============================================================================
void tb_rulers() {
    const uint32_t t0 = Ruler::now_low();
    const uint32_t s0 = Spare::now_low();
    const uint32_t m0 = Mtime::micros();
    wait_us(200'000u);
    const uint32_t t1 = Ruler::now_low();
    const uint32_t s1 = Spare::now_low();
    const uint32_t m1 = Mtime::micros();
    const uint32_t dt = t1 - t0;
    const uint32_t ds = s1 - s0;
    const uint32_t dm = m1 - m0;
    print(serial, "  200 ms: TIMER0 ", dt, " us, TIMER1 ", ds, " us, the platform timer ",
          dm, " us", crlf);
    bench.verdict("TIMER0's span is the 200 ms asked for", dt >= 200'000u && dt < 200'100u);
    bench.verdict("TIMER1 counts the same span within 4 us: two generators, one clk_ref",
                  (ds > dt ? ds - dt : dt - ds) <= 4u);
    bench.verdict("the SIO's platform timer counts it too, within 4 us - a third "
                  "generator of 8.5 and the only counter the two architectures share",
                  (dm > dt ? dm - dt : dt - dm) <= 4u);
    bench.verdict("the platform timer is running (the RISC-V half's kernel timebase "
                  "stands on it; the Arm half starts it as a ruler)", Mtime::running());
}

// =============================================================================
// c - the 64-bit read, both instances
// =============================================================================
void tc_reads() {
    const uint64_t a = Ruler::now();
    const uint64_t b = Ruler::now_latched();
    const uint64_t c = Ruler::now();
    print(serial, "  TIMER0 now() ", static_cast<uint32_t>(a), " latched ",
          static_cast<uint32_t>(b), " now() ", static_cast<uint32_t>(c), " us; high half ",
          static_cast<uint32_t>(a >> 32), crlf);
    bench.verdict("the raw-pair read and the latched read agree within 50 us",
                  b >= a && b - a <= 50u && c >= b && c - b <= 50u);

    bool monotonic = true;
    uint64_t last = Ruler::now();
    for (uint16_t k = 0; k < 2000; ++k) {
        const uint64_t n = Ruler::now();
        if (n < last) { monotonic = false; }
        last = n;
    }
    bench.verdict("2000 raw-pair reads of TIMER0 never go backwards", monotonic);

    bool monotonic1 = true;
    uint64_t last1 = Spare::now();
    for (uint16_t k = 0; k < 2000; ++k) {
        const uint64_t n = Spare::now();
        if (n < last1) { monotonic1 = false; }
        last1 = n;
    }
    bench.verdict("nor 2000 reads of TIMER1", monotonic1);
    bench.verdict("both counters are still in their first 2^32 microseconds, so the low "
                  "half alone is an honest span here",
                  (Ruler::now() >> 32) == 0u && (Spare::now() >> 32) == 0u);
}

// =============================================================================
// d, e - the alarms of each instance, on their own lines
// =============================================================================
volatile uint32_t alarm_entry_us[8] = {};
volatile uint8_t alarm_fires[8] = {};

/// One alarm's round trip: armed two milliseconds out, the handler's
/// entry stamped by the ruler, the flags checked on the way through.
template <uint8_t t, uint8_t a>
void one_alarm() {
    constexpr uint8_t slot = 4u * t + a;
    alarm_fires[slot] = 0;
    Timer<t>::template clear<a>();
    Timer<t>::template interrupt<a>(true);
    Irq::enable(Timer<t>::template irq<a>());
    // THE TARGET IS IN THE INSTANCE'S OWN TIME. The two counters started
    // at different instants, so an alarm of TIMER1 takes a value read
    // from TIMER1, and its handler stamps the same counter: the latency
    // below is a difference within one counter and never across two.
    const uint32_t target = Timer<t>::now_low() + 2000u;
    Timer<t>::template alarm<a>(target);
    const bool was_armed = Timer<t>::template armed<a>();
    const uint32_t read_back = Timer<t>::template alarm_at<a>();
    const uint32_t wait0 = us_now();
    while (alarm_fires[slot] == 0u && us_now() - wait0 < 10'000u) {
    }
    const uint32_t latency = alarm_entry_us[slot] - target;
    print(serial, "  TIMER", t, " alarm ", a, " (line ",
          static_cast<int>(Timer<t>::template irq<a>()), ") at +2000 us: fired ",
          alarm_fires[slot], " time(s), handler entry ", latency, " us after the match",
          crlf);
    bench.verdict("writing the alarm register arms it and reads back as written",
                  was_armed && read_back == target);
    bench.verdict("it fires once and the handler enters within 20 us of the match",
                  alarm_fires[slot] == 1u && latency <= 20u);
    bench.verdict("ARMED clears when it fires", !Timer<t>::template armed<a>());
    bench.verdict("the handler's write-one cleared the raised flag",
                  !Timer<t>::template raised<a>());
    Timer<t>::template interrupt<a>(false);
    Irq::disable(Timer<t>::template irq<a>());
}

/// The three things an alarm does that have nothing to do with firing:
/// a disarm before the match, INTF forcing the masked status, and the
/// raw flag standing while INTE is clear.
template <uint8_t t>
void alarm_extras() {
    constexpr uint8_t slot = 4u * t;
    Timer<t>::template alarm_in<0>(2000u);
    Timer<t>::template disarm<0>();
    wait_us(4000u);
    bench.verdict("an alarm disarmed before its time never fires", alarm_fires[slot] == 1u);
    bench.verdict("and the raw flag stayed down with it", !Timer<t>::template raised<0>());

    Timer<t>::template interrupt<0>(true);
    Timer<t>::template force<0>(true);
    const bool forced = Timer<t>::template pending<0>();
    const bool raw_untouched = !Timer<t>::template raised<0>();
    Timer<t>::template force<0>(false);
    const bool gone = !Timer<t>::template pending<0>();
    Timer<t>::template interrupt<0>(false);
    bench.verdict("INTF forces the masked status without touching INTR, and clearing it "
                  "takes the line back",
                  forced && raw_untouched && gone);
}

void td_timer0_alarms() {
    one_alarm<0, 0>();
    one_alarm<0, 1>();
    one_alarm<0, 2>();
    one_alarm<0, 3>();
    alarm_extras<0>();
}

void te_timer1_alarms() {
    one_alarm<1, 0>();
    one_alarm<1, 1>();
    one_alarm<1, 2>();
    one_alarm<1, 3>();
    alarm_extras<1>();
    bench.verdict("the second instance's four alarms are four interrupt lines of their "
                  "own, 4..7 - what this chip has and the RP2040 had not",
                  static_cast<int>(Timer<1>::irq<0>()) ==
                      static_cast<int>(Timer<0>::irq<0>()) + 4);
}

// =============================================================================
// f - PAUSE and DBGPAUSE on TIMER1
// =============================================================================
void tf_pause() {
    bench.verdict("init() cleared both DBGPAUSE bits: a core halted in a debugger no "
                  "longer freezes the ruler of the other core",
                  !Ruler::debug_paused(0) && !Ruler::debug_paused(1) &&
                      !Spare::debug_paused(0) && !Spare::debug_paused(1));
    Spare::debug_pause(true, true);
    const bool set_back = Spare::debug_paused(0) && Spare::debug_paused(1);
    Spare::debug_pause(false, false);
    bench.verdict("the datasheet's own default is one call away and reads back",
                  set_back && !Spare::debug_paused(0));

    const uint32_t before = Spare::now_low();
    Spare::pause(true);
    const bool paused = Spare::paused();
    const uint32_t at_pause = Spare::now_low();
    wait_us(5000u);
    const uint32_t during = Spare::now_low();
    Spare::pause(false);
    const uint32_t t0 = us_now();
    wait_us(5000u);
    const uint32_t after = Spare::now_low();
    print(serial, "  TIMER1 paused: ", during - at_pause, " us counted over a 5 ms wait; "
          "resumed: ", after - during, " us over the next 5 ms (the ruler saw ",
          us_now() - t0, " us)", crlf);
    bench.verdict("PAUSE reads back", paused && !Spare::paused());
    bench.verdict("a paused counter does not move", during - at_pause <= 2u);
    bench.verdict("it takes up from where it stopped, with nothing caught up",
                  after - during >= 4990u && after - during < 5100u);
    bench.verdict("so the pause is missing from its count for good",
                  (after - before) < 6000u);
}

// =============================================================================
// g - SOURCE: TIMER1 off the tick and onto clk_sys
// =============================================================================
void tg_source() {
    bench.verdict("init() leaves both counters on the microsecond tick",
                  Ruler::source() == TimerSource::tick && Spare::source() == TimerSource::tick);
    Spare::source(TimerSource::sysclk);
    const bool took = Spare::source() == TimerSource::sysclk;
    const uint32_t s0 = Spare::now_low();
    const uint32_t t0 = us_now();
    wait_us(20'000u);
    const uint32_t ds = Spare::now_low() - s0;
    const uint32_t dt = us_now() - t0;
    Spare::source(TimerSource::tick);
    // Per MILLISECOND, not per microsecond: the ratio is 150 here and a
    // truncating division of the raw counts would hide a whole per cent
    // of error inside it. The two spans are read one after the other and
    // not at the same instant, so the ceiling is the slack of those four
    // reads - a tenth of a per cent is far wider than any of them.
    const uint32_t per_ms =
        dt != 0u ? static_cast<uint32_t>(static_cast<uint64_t>(ds) * 1000u / dt) : 0u;
    const uint32_t nominal_per_ms = SysClock::hz / 1000u;
    print(serial, "  TIMER1 on clk_sys: ", ds, " counts in ", dt, " us = ", per_ms,
          " per millisecond against ", nominal_per_ms, " nominal (clk_sys is ",
          SysClock::hz / 1'000'000u, " MHz)", crlf);
    bench.verdict("SOURCE reads back and takes effect", took);
    bench.verdict("a counter on clk_sys counts one per system clock cycle, so its unit "
                  "is the cycle and it moves with every rate change - measured against "
                  "the microsecond ruler to a tenth of a per cent, which is the slack of "
                  "reading the two spans one after the other",
                  per_ms + nominal_per_ms / 1000u >= nominal_per_ms &&
                      per_ms <= nominal_per_ms + nominal_per_ms / 1000u);
    bench.verdict("and the tick is one write away again",
                  Spare::source() == TimerSource::tick);
    const uint32_t back0 = Spare::now_low();
    wait_us(5000u);
    bench.verdict("back on the tick it counts microseconds again",
                  Spare::now_low() - back0 >= 4990u && Spare::now_low() - back0 < 5200u);
}

// =============================================================================
// h - the tick generators of 8.5
// =============================================================================
void th_ticks() {
    bench.verdict("neither core's SysTick generator runs: on this target the Arm "
                  "timebase counts clk_sys cycles and the RISC-V one counts the "
                  "platform timer's own tick, so nothing asked for these two",
                  !TickGenerator<TickConsumer::proc0>::running() &&
                      !TickGenerator<TickConsumer::proc1>::running());

    const uint32_t divider = Spare::Tick::cycles();
    Spare::Tick::stop();
    const bool stopped = !Spare::Tick::running();
    const uint32_t at_stop = Spare::now_low();
    wait_us(5000u);
    const uint32_t frozen = Spare::now_low() - at_stop;
    const bool restarted = Spare::Tick::start(divider);
    const uint32_t r0 = Spare::now_low();
    wait_us(5000u);
    const uint32_t moving = Spare::now_low() - r0;
    print(serial, "  TIMER1's generator stopped: ", frozen,
          " us counted over 5 ms; restarted at ", Spare::Tick::cycles(), " cycles: ",
          moving, " us over the next 5 ms", crlf);
    bench.verdict("stopping a generator stops the counter behind it - the tick is what "
                  "the counter counts, not a clock the block owns",
                  stopped && frozen <= 2u);
    bench.verdict("restarting it takes the counter up again at the same divisor",
                  restarted && Spare::Tick::running() && Spare::Tick::cycles() == divider &&
                      moving >= 4990u && moving < 5100u);
    bench.verdict("the generator's own countdown is below its divisor",
                  Spare::Tick::count() <= divider);
    bench.verdict("a divisor of zero, and one past the nine-bit field, are refused",
                  !Spare::Tick::start(0u) && !Spare::Tick::start(512u));
    (void)Spare::Tick::start(divider);
}

// =============================================================================
// j - the watchdog countdown, measured against the ruler
// =============================================================================
void tj_watchdog() {
    bench.verdict("the watchdog has a tick generator of its own in the TICKS block, and "
                  "init() started it - on the RP2040 this block OWNED the tick and lent "
                  "it to the system timer",
                  Watchdog::Tick::running() &&
                      Watchdog::Tick::cycles() == SysClock::ref_hz / 1'000'000u);

    // A time-out far longer than this letter, and the debug pause bits
    // OFF: PAUSE_JTAG would hold the count while the probe touches the
    // bus fabric, and what is wanted here is the silicon's rate.
    Watchdog::start(8'000'000u, false);
    const bool started = Watchdog::running();
    const uint32_t r0 = Watchdog::remaining_us();
    const uint32_t t0 = us_now();
    wait_us(200'000u);
    const uint32_t r1 = Watchdog::remaining_us();
    const uint32_t dt = us_now() - t0;
    const uint32_t decrements = r0 - r1;
    const uint32_t rate_permille = dt != 0u ? decrements * 1000u / dt : 0u;

    // The tick generator behind the countdown, with the counter watched
    // across a stop.
    const uint32_t divider = Watchdog::Tick::cycles();
    Watchdog::Tick::stop();
    const uint32_t f0 = Watchdog::remaining_us();
    wait_us(5000u);
    const uint32_t frozen = f0 - Watchdog::remaining_us();
    (void)Watchdog::Tick::start(divider);

    Watchdog::kick();
    const uint32_t after_kick = Watchdog::remaining_us();
    Watchdog::stop();
    const bool stopped = !Watchdog::running();

    print(serial, "  countdown: ", decrements, " counts in ", dt, " us = ", rate_permille,
          " per 1000 us; frozen generator: ", frozen, " counts over 5 ms; after a kick ",
          after_kick, " of ", Watchdog::timeout_us(), " us", crlf);
    bench.verdict("the countdown starts and stops - ENABLE clears here, where the other "
                  "families' watchdogs are one-way",
                  started && stopped);
    bench.verdict("IT DECREMENTS ONCE PER MICROSECOND, NOT TWICE: the RP2040's erratum "
                  "E1 doubling is not this chip's, so LOAD is microseconds as written "
                  "and the counter reaches 16.8 s",
                  rate_permille >= 995u && rate_permille <= 1005u);
    bench.verdict("its tick comes from the TICKS block now: stop that generator and the "
                  "countdown stands still",
                  frozen <= 2u);
    bench.verdict("a kick reloads the time-out that was asked for",
                  after_kick > Watchdog::timeout_us() - 20'000u &&
                      after_kick <= Watchdog::timeout_us());

    // A time-out past the counter's reach, asked for and taken back at
    // once: what is judged is the value the driver kept, not a reset.
    Watchdog::start(100'000'000u);
    Watchdog::stop();
    bench.verdict("a time-out longer than the counter reaches is clamped and not wrapped",
                  Watchdog::timeout_us() == Watchdog::max_timeout_us);
    print(serial, "  PSM WDSEL after start(): ", hex(Psm::watchdog_resets()), " (the "
          "reboot selection is ", hex(PsmStage::reboot), ")", crlf);
    bench.verdict("what a time-out would reset is every stage of the power-on state "
                  "machine that a reboot through the bootrom needs - the selection is "
                  "ORed in, so a program may widen it and never narrows it by accident",
                  (Psm::watchdog_resets() & PsmStage::reboot) == PsmStage::reboot);
}

// =============================================================================
// k - LOCKED on TIMER1, and the way back
// =============================================================================
void tk_lock() {
    Spare::disarm<0>();
    Spare::alarm<0>(0xAAAA5555u);
    const bool wrote = Spare::alarm_at<0>() == 0xAAAA5555u;
    bench.verdict("an unlocked timer takes the write", wrote && !Spare::locked());

    Spare::lock();
    const bool locked = Spare::locked();
    Spare::alarm<0>(0x5555AAAAu);
    const bool refused_alarm = Spare::alarm_at<0>() == 0xAAAA5555u;
    Spare::pause(true);
    const bool refused_pause = !Spare::paused();
    Spare::source(TimerSource::sysclk);
    const bool refused_source = Spare::source() == TimerSource::tick;
    Spare::force<1>(true);
    const bool refused_alias = !Spare::pending<1>();
    Spare::force<1>(false);
    Spare::source(TimerSource::tick);
    Spare::pause(false);
    const uint32_t c0 = Spare::now_low();
    wait_us(5000u);
    const uint32_t still_counting = Spare::now_low() - c0;
    print(serial, "  TIMER1 locked: LOCKED reads ", locked ? 1 : 0,
          "; ALARM0 ", refused_alarm ? "refused" : "TOOK the write",
          ", PAUSE ", refused_pause ? "refused" : "TOOK the write",
          ", SOURCE ", refused_source ? "refused" : "TOOK the write",
          ", INTF through the atomic alias ", refused_alias ? "refused" : "TOOK the write",
          crlf);
    print(serial, "  and the counter advanced ", still_counting, " us over 5 ms", crlf);
    bench.verdict("LOCKED takes its write and reads back set", locked);
    bench.verdict("BUT ON THIS STEPPING IT REFUSES NOTHING - a plain register, a register "
                  "that changes the counting, and one written through an atomic alias all "
                  "take their writes with LOCKED set, so a program may not treat the bit "
                  "as a guard: what 12.8 says of it is not what the silicon does",
                  !refused_alarm && !refused_pause && !refused_source && !refused_alias);
    bench.verdict("and the counter goes on running under it, the reads going on answering",
                  still_counting >= 4990u && still_counting < 5100u);

    // The one way back: the subsystem reset controller, which init()
    // cycles for exactly this reason.
    const bool back = Spare::init(clock);
    bench.verdict("init() is the way out, because it CYCLES this block's reset line "
                  "instead of merely releasing it - which is what takes LOCKED down",
                  back && !Spare::locked());
    const uint32_t d0 = Spare::now_low();
    wait_us(5000u);
    bench.verdict("and the counter starts again from zero on its microsecond tick",
                  Spare::now_low() - d0 >= 4990u && Spare::source() == TimerSource::tick);
}

// =============================================================================
// m - the scratch registers and the atomic register aliases
// =============================================================================
void tm_scratch() {
    volatile uint32_t& s = Scratch<3>::reg();
    const uint32_t keep = s;
    Scratch<3>::write(0xF0F0F0F0u);
    const bool round_trip = Scratch<3>::read() == 0xF0F0F0F0u;
    hw_set(s, 0x0000000Fu);
    const uint32_t after_set = s;
    hw_clear(s, 0xF0000000u);
    const uint32_t after_clear = s;
    hw_xor(s, 0x000000FFu);
    const uint32_t after_xor = s;
    hw_write_masked(s, 0x12340000u, 0x0FFF0000u);
    const uint32_t after_masked = s;
    s = keep;
    print(serial, "  set -> ", hex(after_set), " clear -> ", hex(after_clear), " xor -> ",
          hex(after_xor), " masked -> ", hex(after_masked), crlf);
    bench.verdict("a scratch word takes a value and gives it back", round_trip);
    bench.verdict("the SET alias ORs the written bits in", after_set == 0xF0F0F0FFu);
    bench.verdict("the CLR alias clears them", after_clear == 0x00F0F0FFu);
    bench.verdict("the XOR alias flips them", after_xor == 0x00F0F000u);
    bench.verdict("a masked write changes only its mask", after_masked == 0x0234F000u);
}

// =============================================================================
// n - the panic breadcrumb, without a reset
// =============================================================================
void tn_breadcrumb() {
    print(serial, "  panic record at ", hex(reinterpret_cast<uintptr_t>(&P::panic_record())),
          ", the .noinit token at ", hex(reinterpret_cast<uintptr_t>(&noinit_token)), crlf);
    bench.verdict("the .noinit section sits in the main SRAM",
                  reinterpret_cast<uintptr_t>(&noinit_token) >= 0x20000000u &&
                      reinterpret_cast<uintptr_t>(&noinit_token) < 0x20082000u);
    bench.verdict("nothing is pending", !take_panic_record<P>());
    P::panic_record() =
        PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::assert_failed), 0x42};
    const auto taken = take_panic_record<P>();
    bench.verdict("a written record is taken with its code and context",
                  taken && taken->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                      taken->context == 0x42);
    bench.verdict("and taken only once", !take_panic_record<P>());
}

// =============================================================================
// i - the real resets (by name only)
// =============================================================================
void bank(uint8_t leg, uint8_t code = 0, uint8_t context = 0) { token_set(leg, code, context); }

[[noreturn]] void await_reset(const char* what) {
    console_drain();
    wait_us(3'000'000u);
    print(serial, "  ...3 s and still running ", what, crlf);
    for (;;) {
    }
}

/// The processor's own reset request, called only where it exists: the
/// discarded branch of an `if constexpr` inside a template is never
/// instantiated, which is the same refusal the verb itself carries.
template <CoreKind k>
[[noreturn]] void request_core_reset() {
    if constexpr (k == CoreKind::cortex_m33) {
        Reset::core();
    } else {
        for (;;) {
        }
    }
}

[[noreturn]] void leg_force() {
    bank(1);
    print(serial, "  leg 1: Reset::software() - the watchdog's trigger under the "
          "power-on state machine ...", crlf);
    console_drain();
    Reset::software();
}
[[noreturn]] void leg_timeout() {
    bank(2);
    print(serial, "  leg 2: the watchdog started at 100 ms and never kicked ...", crlf);
    console_drain();
    (void)Watchdog::init(clock);
    Watchdog::start(100'000u);
    await_reset("(the watchdog did not fire)");
}
[[noreturn]] void leg_panic() {
    bank(3, static_cast<uint8_t>(PanicCode::queue_overflow), 0x33);
    print(serial, "  leg 3: panic<P, ResetReporter>(queue_overflow, 0x33) ...", crlf);
    console_drain();
    panic<P, ResetReporter>(PanicCode::queue_overflow, 0x33);
}
[[noreturn]] void leg_fault() {
    bank(4, static_cast<uint8_t>(PanicCode::kernel_fault), 0x77);
    print(serial, "  leg 4: a breakpoint instruction with no debugger attached - a "
          "HardFault on one half, the crt's exception trap on the other ...", crlf);
    console_drain();
    P::break_here();
    await_reset("(the breakpoint did not fault: a probe has left halting debug enabled)");
}
[[noreturn]] void leg_core() {
    bank(5);
    print(serial, "  leg 5: Reset::core() - this processor's own reset request ...", crlf);
    console_drain();
    request_core_reset<core_kind>();
}

void finish() {
    token_clear();
    boot_record.reset();   // consumed: a second 'i' in this boot starts clean
    print(serial, "  every leg done", crlf);
    bench.end_letter();
}

void ti_resets() {
    report_boot();
    bench.verdict("no breadcrumb is pending on a clean start", !boot_record);
    leg_force();
}

/// Everything after a reset: called from main() instead of the banner,
/// it either starts the next leg (never returning) or closes the letter.
void ti_resume() {
    const uint8_t leg = boot_scratch_alive ? scratch_leg() : noinit_leg();
    const uint32_t tally = boot_scratch_alive ? Scratch<1>::read() : noinit_token.tally;
    const uint32_t expect = boot_scratch_alive ? Scratch<2>::read() : noinit_token.expect;
    const uint32_t before = boot_scratch_alive ? Scratch<3>::read() : noinit_token.before;
    bench.resume_tally(static_cast<uint16_t>(tally >> 16), static_cast<uint16_t>(tally & 0xFFFFu));
    print(serial, crlf, "i (continued after reset ", leg, ")", crlf);
    report_boot();
    // THE TWO SURVIVORS, EACH ITS OWN VERDICT: a failure here is the
    // finding, not an accident, and the letter goes on through whichever
    // copy did cross.
    bench.verdict("the watchdog's scratch registers crossed this reset - which is 7.2's "
                  "note and not 7.3.1's table",
                  boot_scratch_alive);
    bench.verdict("the .noinit SRAM crossed it too, a fact the datasheet promises nowhere",
                  boot_noinit_alive);
    bench.verdict("and where both crossed they carry the same leg",
                  !(boot_scratch_alive && boot_noinit_alive) || scratch_leg() == noinit_leg());
    if (leg == 0u) {
        print(serial, "  neither survivor crossed: the letter stops here rather than "
              "start leg 1 again", crlf);
        bench.end_letter();
        return;
    }

    // THE CAUSES ARE NOT A CAUSE: every leg compares the word with the
    // one it banked rather than reading it.
    if (leg == 1) {
        bench.verdict("Reset::software() names itself in the watchdog's REASON: FORCE",
                      (boot_causes & ResetCause::watchdog_force) != 0u);
        bench.verdict("REASON holds the LAST watchdog event: TIMER, if it stood, is gone",
                      (boot_causes & ResetCause::watchdog_timer) == 0u);
        // WHAT CHIP_RESET DOES ACROSS A REBOOT. The bit names read like
        // latches and 7.3.3 calls the word the source of the last
        // CHIP-level reset - and this reboot is not one: a watchdog event
        // reaches the chip level only when POWMAN's own WDSEL selects it
        // (6.4, RESET_PSM), a register at its reset value here because
        // nothing in brio writes POWMAN. So the word should come through
        // untouched, and the printed pair is the proof.
        const uint32_t chip_now = boot_causes & ResetCause::chip_level;
        const uint32_t chip_before = before & ResetCause::chip_level;
        print(serial, "  CHIP_RESET before ", hex(chip_before), " after ", hex(chip_now),
              crlf);
        bench.verdict("the power manager does NOT record a reboot: a watchdog event that "
                      "runs the power-on state machine is a SYSTEM reset, and only "
                      "POWMAN's own WDSEL - untouched here - would make it reach the "
                      "chip-level record",
                      (boot_causes & ResetCause::watchdog_psm) == 0u);
        bench.verdict("so the chip-level half crosses the reboot bit for bit, still "
                      "naming the last chip-level reset",
                      chip_now == chip_before);
        leg_timeout();
    }
    if (leg == 2) {
        bench.verdict("a watchdog time-out names itself: REASON.TIMER, FORCE gone",
                      (boot_causes & ResetCause::watchdog_timer) != 0u &&
                          (boot_causes & ResetCause::watchdog_force) == 0u);
        bench.verdict("and it took the same tier as the trigger: a system reset, absent "
                      "from the power manager's chip-level record like that one",
                      (boot_causes & ResetCause::watchdog_psm) == 0u &&
                          (boot_causes & ResetCause::chip_level) ==
                              (before & ResetCause::chip_level));
        leg_panic();
    }
    if (leg == 3) {
        bench.verdict("the breadcrumb of a panic through ResetReporter is read at the "
                      "next boot with its code and context",
                      boot_record && boot_record->code == (expect >> 8) &&
                          boot_record->context == (expect & 0xFFu));
        bench.verdict("its reset was the reboot both halves have: REASON.FORCE",
                      (boot_causes & ResetCause::watchdog_force) != 0u);
        leg_fault();
    }
    if (leg == 4) {
        bench.verdict("a fault through the crt's entry leaves a kernel_fault breadcrumb "
                      "with the context the app gave",
                      boot_record && boot_record->code == (expect >> 8) &&
                          boot_record->context == (expect & 0xFFu));
        bench.verdict("and its reset was the same reboot: REASON.FORCE",
                      (boot_causes & ResetCause::watchdog_force) != 0u);
        if constexpr (core_kind == CoreKind::cortex_m33) {
            leg_core();
        } else {
            bench.verdict("THE HAZARD3 HALF HAS FOUR LEGS AND THE ARM HALF FIVE: a hart "
                          "cannot reset itself, its hartreset and ndmreset being the "
                          "Debug Module's, so Reset::core() is refused at compile time "
                          "here and the reboot both halves share is the watchdog's",
                          core_kind == CoreKind::hazard3);
            finish();
        }
    }
    if (leg == 5) {
        bench.verdict("a processor's own reset request leaves the chip-level latches "
                      "exactly as they were: it is not a chip-level reset",
                      (boot_causes & ResetCause::chip_level) == (before & ResetCause::chip_level));
        bench.verdict("BUT IT CLEARS THE WATCHDOG'S REASON, where the RP2040's did not: "
                      "on this chip a warm reset of either core wipes that register so "
                      "code loaded under a probe does not go on reading an old time-out",
                      (boot_causes & ResetCause::watchdog) == 0u &&
                          (before & ResetCause::watchdog) != 0u);
        finish();
    }
}

void banner() {
    print(serial, crlf, "test_rp2350_timer - the system timers, the tick generators, the "
          "watchdog and the resets (datasheet 12.8, 8.5, 12.9, 7) on ", arch_name(),
          ", clk_sys=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// THE ALARM VECTORS, eight of them: TIMER0's four and TIMER1's four,
// bound by the names the crt gives the lines - the same names on both
// architectures, through the Arm vector table on one and Hazard3's own
// dispatch on the other.

extern "C" void isr_timer0_0() {
    alarm_entry_us[0] = brio::Timer<0>::now_low();
    brio::Timer<0>::clear<0>();
    alarm_fires[0] = alarm_fires[0] + 1;
}
extern "C" void isr_timer0_1() {
    alarm_entry_us[1] = brio::Timer<0>::now_low();
    brio::Timer<0>::clear<1>();
    alarm_fires[1] = alarm_fires[1] + 1;
}
extern "C" void isr_timer0_2() {
    alarm_entry_us[2] = brio::Timer<0>::now_low();
    brio::Timer<0>::clear<2>();
    alarm_fires[2] = alarm_fires[2] + 1;
}
extern "C" void isr_timer0_3() {
    alarm_entry_us[3] = brio::Timer<0>::now_low();
    brio::Timer<0>::clear<3>();
    alarm_fires[3] = alarm_fires[3] + 1;
}
extern "C" void isr_timer1_0() {
    alarm_entry_us[4] = brio::Timer<1>::now_low();
    brio::Timer<1>::clear<0>();
    alarm_fires[4] = alarm_fires[4] + 1;
}
extern "C" void isr_timer1_1() {
    alarm_entry_us[5] = brio::Timer<1>::now_low();
    brio::Timer<1>::clear<1>();
    alarm_fires[5] = alarm_fires[5] + 1;
}
extern "C" void isr_timer1_2() {
    alarm_entry_us[6] = brio::Timer<1>::now_low();
    brio::Timer<1>::clear<2>();
    alarm_fires[6] = alarm_fires[6] + 1;
}
extern "C" void isr_timer1_3() {
    alarm_entry_us[7] = brio::Timer<1>::now_low();
    brio::Timer<1>::clear<3>();
    alarm_fires[7] = alarm_fires[7] + 1;
}

extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

// THE FAULT ENTRY, under BOTH crts' names and with no question asked of
// the preprocessor: the Arm table enters `isr_hardfault`, the RISC-V one
// `isr_riscv_exception`, and each image links the one its crt declares
// while the other is a function nobody calls.
extern "C" void isr_hardfault() { brio::fault_reset<P>(0x77); }
extern "C" void isr_riscv_exception() { brio::fault_reset<P>(0x77); }

int main() {
    boot_causes = brio::Reset::causes();
    boot_record = brio::take_panic_record<P>();
    boot_scratch_alive = scratch_leg() != 0u;
    boot_noinit_alive = noinit_leg() != 0u;

    const bool clock_ok = SysClock::init();
    const bool t0_ok = brio::Timer<0>::init(clock);
    const bool t1_ok = brio::Timer<1>::init(clock);
    const bool mtime_ok = brio::Mtime::start(clock);
    const bool wd_ok = brio::Watchdog::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output(false);
    brio::enable_interrupts();

    bench.letter('a', "the boot story", ta_boot);
    bench.letter('b', "three microsecond counters against each other", tb_rulers);
    bench.letter('c', "the 64-bit read, both instances", tc_reads);
    bench.letter('d', "TIMER0's four alarms and their lines", td_timer0_alarms);
    bench.letter('e', "TIMER1's four alarms and their lines", te_timer1_alarms);
    bench.letter('f', "PAUSE and DBGPAUSE", tf_pause);
    bench.letter('g', "the counter on clk_sys instead of the tick", tg_source);
    bench.letter('h', "the tick generators of 8.5", th_ticks);
    bench.letter('j', "the watchdog countdown, measured", tj_watchdog);
    bench.letter('k', "LOCKED, and the reset controller as the way back", tk_lock);
    bench.letter('m', "the scratch registers and the atomic aliases", tm_scratch);
    bench.letter('n', "the panic breadcrumb, no reset", tn_breadcrumb);
    bench.letter('i', "THE REAL RESETS (reboots the board)", ti_resets, false);

    if (serial_ok && (boot_scratch_alive || boot_noinit_alive)) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", t0_ok ? "1us" : "FAILED", " timer1=", t1_ok ? "1us" : "FAILED",
                    " mtime=", mtime_ok ? "1us" : "FAILED", " watchdog=",
                    wd_ok ? "1us" : "FAILED", " tick=", tick_ok ? "on" : "FAILED", brio::crlf);
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
