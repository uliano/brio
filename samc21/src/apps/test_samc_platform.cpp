// test_samc_platform - the reference bench suite for the SAM C21's
// PLATFORM: the running half (samc21/platform.hpp's idle hook against an
// interrupt's edge, the kernel's turn, delay_us on the SysTick counter)
// and the failing half - samc21/reset.hpp: the reset controller, the
// watchdog, and the panic breadcrumb that has to cross a real reset to be
// worth anything.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and nothing here costs any endurance. Letter w writes
// SysTick's LOAD and VAL to place its edges, so kernel time runs fast
// while it runs.
//
// What is exercised, letter by letter:
//   a  the boot story: RCAUSE read as the EXCLUSIVE register it is (one
//      cause, not a history), and the watchdog's power-on state checked
//      against the NVM User Row that supplied it - a cross-check between
//      two drivers, since samc21/nvm.hpp reads the fuses that samc21/
//      reset.hpp then finds in CTRLA/CONFIG/EWCTRL
//   b  the watchdog as a configurable timer, with nothing allowed to
//      time out: arming, read-back, disabling, the refusals
//   c  what OSCULP32K actually runs at, measured through the early-
//      warning interrupt in both modes - the offset in normal mode and
//      the closed window in window mode
//   d  samc21/delay.hpp's delay_us on the SysTick counter: at-least
//      never early, the sub-tick cap, and the refusals
//
//   w  THE IDLE HOOK AGAINST AN INTERRUPT'S EDGE, PLACED TO THE CYCLE:
//      an edge D cycles after its placement, D = 0..399, then the
//      kernel's own shape - a masked check, idle(), until the edge is
//      served - so the edge walks across every instruction of the idle
//      path and of the loop around it, the WFI included. Seven passes:
//      the kernel's own wake (the SysTick, placed through LOAD) in IDLE0
//      and IDLE2, waiting for one tick and for two (the sleep after a
//      tick was taken); and an NVIC line (a TC match, the tick's
//      interrupt held off so the line is the only wake) in IDLE0, IDLE2
//      and STANDBY - the last one walking idle()'s other branch, the
//      erratum guard's. A TC pair at 48 MHz through standby is each try's
//      stopwatch and its RESCUE: a try the rescue's 50 ms ends is a wake
//      lost for good, one that ends a whole tick late is a wake lost
//      until the next tick. A TC with RUNSTDBY clear, halted in standby,
//      is the witness that the STANDBY pass slept in standby. Prints the
//      cycles from the edge to the caller's loop for an edge already
//      pending and for one that finds the core asleep; the IDLE2 pass
//      again under NVMCTRL's other two SLEEPPRM settings (what the wake
//      from IDLE2 pays is the flash); what SysTick's counter does across
//      a standby, held and unheld; the core's round trip on a pended
//      line, and what an idle() a pending line returns at once costs.
//      What it does not walk: the SysTick's own edge across a STANDBY
//      entry (its interrupt is held off there by design - erratum
//      1.8.13 - so it is no standby wake), and the hard fault that
//      erratum describes.
//   k  THE KERNEL'S TURN: a Tenuto pack of three quiet AOs, two with a
//      periodic time event, turned as Tenuto::run() turns it over 100
//      ticks with the tick the only interrupt - one turn per tick - and
//      what one quiet turn costs when a pending line makes its idle()
//      return at once: the turn every interrupt that wakes a quiet kernel
//      pays.
//   i  (by name only) SIX REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session that a tool can judge
//      from a single capture. Run it with
//          brio run C i --expect="->"
//      Legs: a wrong CLEAR key with the watchdog stopped and then
//      running, a panic through ResetReporter, a deliberate HardFault
//      through hard_fault_reset(), a watchdog time-out, and a window
//      violation.
//   t  the runtime's seven functions (rt/rt.cpp) over every alignment,
//      length and overlap: rt/selftest.hpp's cases, the host suite's own,
//      against the symbols this image links - in `z`
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stddef.h>
#include <stdint.h>

#include <algorithm>
#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "samc21/clock.hpp"
#include "samc21/delay.hpp"
#include "samc21/tc.hpp"
#include "samc21/nvic.hpp"
#include "samc21/nvm.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/osc32kctrl.hpp"
#include "samc21/reset.hpp"
#include "samc21/rtc.hpp"
#include "samc21/sercom.hpp"
#include "samc21/sleep.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "rt/selftest.hpp"

using P = brio::SamPlatform;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter i lives in
//
// INLINE, and in .noinit, for two reasons: the section must survive the
// crt (the linker script marks
// .noinit NOLOAD and startup neither loads nor zeroes it), and gcc gives
// an inline variable with a section attribute a COMDAT group where a
// plain one gets none - the platform's own panic_record_ is a static
// inline member, so this must be inline too or the link fails with a
// section type conflict.
//
// Its magic word is not decoration. Table 18-1 of DS60001479M lists what
// each reset cause resets and has NO SRAM ROW AT ALL, for any source
// including power-on: nothing promises this object survives, so every
// read of it is guarded.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5A11;
inline constexpr uint16_t token_canary = 0xC3A5;

struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t leg;        ///< which reset we are waiting for (0 = none pending)
    uint8_t code;       ///< the PanicCode written before the reset
    uint8_t context;    ///< its context byte
    uint16_t pass;      ///< letter i's tally so far
    uint16_t fail;
};
[[gnu::section(".noinit")]] inline Token token;

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

// What this boot was told, sampled once in main() before anything can
// disturb it.
ResetCause boot_cause = ResetCause::unknown;
uint8_t boot_cause_bits = 0;
std::optional<PanicRecord> boot_record;

// Set by the early-warning handler; read by letter c.
volatile uint32_t ew_cycles = 0;
volatile bool ew_seen = false;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch (the same one the other SAM suites use): the kernel ticker's own cycles(), the tick count
// and the counter's position composed right across the reload
// (util/cycle_count.hpp).
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    return Ticker::cycles();
}

uint32_t cycles_to_ms(uint32_t cycles) { return cycles / (SysClock::hz / 1000UL); }

/// Wait for the console to be physically empty. Called before anything
/// that reboots the board: a ring that still holds bytes loses them, and
/// a suite that loses its own last line is unreadable.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    // The ring being empty only means the last byte reached the shifter.
    // One character at 115200 is 87 us; two milliseconds is comfortably
    // more, and measuring it beats a spin count nobody can check.
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

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

void report_boot() {
    print(serial, "  boot: RCAUSE=", hex(boot_cause_bits), " = ",
          cause_name(boot_cause),
          boot_record ? "; a panic record was pending" : "; no panic record",
          crlf);
}

// =============================================================================
// a - the boot story, and the fuses behind the watchdog
// =============================================================================
void ta_boot() {
    report_boot();

    // RCAUSE IS EXCLUSIVE. 18.8.1: the bit for the source is set and all
    // others are written to zero. This is the fact that most needs
    // asserting: a reset-cause register that ACCUMULATES history is the
    // commoner shape, and the habit travels.
    uint8_t bits = boot_cause_bits, set = 0;
    while (bits != 0u) {
        set = static_cast<uint8_t>(set + (bits & 1u));
        bits = static_cast<uint8_t>(bits >> 1);
    }
    bench.verdict("RCAUSE names exactly one source (it is exclusive, not a "
                  "history)", set == 1u);
    bench.verdict("and cause() agrees with the raw bits",
                  boot_cause != ResetCause::unknown);
    bench.verdict("the two groups of table 18-1 partition it",
                  Reset::power_supply_reset() != Reset::user_reset());
    bench.verdict("RCAUSE is unchanged by reading it (nothing to clear)",
                  Reset::cause_bits() == boot_cause_bits);

    // THE WATCHDOG'S POWER-ON STATE IS A FUSE, and this is where two
    // drivers meet: samc21/nvm.hpp reads the NVM User Row, samc21/reset.hpp
    // reads the registers 23.6.2.2 says are loaded from it. They must
    // agree, and if they ever stop agreeing one of the two decodings is
    // wrong.
    const NvmUserRow fuses = NvmUserRow::read();
    print(serial, "  fuses : wdt=", fuses.wdt_enabled() ? "on" : "off",
          fuses.wdt_always_on() ? " always-on" : "", " per=", fuses.wdt_period(),
          " window=", fuses.wdt_window(), " ewoffset=", fuses.wdt_ew_offset(), crlf);
    print(serial, "  regs  : wdt=", Watchdog::enabled() ? "on" : "off",
          Watchdog::always_on() ? " always-on" : "", " per=",
          static_cast<uint8_t>(Watchdog::period()), " window=",
          static_cast<uint8_t>(Watchdog::window()), " ewoffset=",
          static_cast<uint8_t>(Watchdog::ew_offset()), crlf);

    bench.verdict("CTRLA.ENABLE came from the user row",
                  Watchdog::enabled() == fuses.wdt_enabled());
    bench.verdict("CTRLA.ALWAYSON came from the user row",
                  Watchdog::always_on() == fuses.wdt_always_on());
    bench.verdict("CONFIG.PER came from the user row",
                  static_cast<uint8_t>(Watchdog::period()) == fuses.wdt_period());
    bench.verdict("CONFIG.WINDOW came from the user row",
                  static_cast<uint8_t>(Watchdog::window()) == fuses.wdt_window());
    bench.verdict("EWCTRL.EWOFFSET came from the user row",
                  static_cast<uint8_t>(Watchdog::ew_offset()) ==
                      fuses.wdt_ew_offset());

    // If either of these were true the reset legs could not run at all,
    // so they are worth naming rather than discovering in letter i.
    bench.verdict("the watchdog is not armed by the fuses on this board",
                  !Watchdog::enabled());
    bench.verdict("and not in always-on mode (which only a POR could undo)",
                  !Watchdog::always_on());
}

// =============================================================================
// b - the watchdog as a configurable timer (nothing times out)
// =============================================================================
// RE-RUNNABILITY: letter a compares the WDT registers against the user
// row, and at boot they agree because the silicon loaded them from it -
// but letters b AND c reprogram CONFIG and EWCTRL, so without a restore
// a second z in the same power cycle fails letter a on exactly the
// fields the armings change - and restoring in ONE of them is not
// enough, since the other clobbers the fields again straight after.
// The boot values are captured once in main(), and EVERY arming letter
// ends by putting them back - AFTER the disable's synchronization,
// because a store made while ENABLE's clear is still crossing is
// discarded (measured: an unsynchronized restore stores nothing).
uint8_t wdt_boot_config = 0;
uint8_t wdt_boot_ewctrl = 0;

void restore_wdt_boot_regs() {
    (void)Watchdog::sync();
    WDT_REGS->WDT_CONFIG = wdt_boot_config;
    WDT_REGS->WDT_EWCTRL = wdt_boot_ewctrl;
    (void)Watchdog::sync();
}

void tb_watchdog() {
    // A long period throughout: every arming below is undone well before
    // anything can expire.
    constexpr WdtConfig normal{
        .period = WdtCycles::cyc16384,
        .early_warning = true,
        .ew_offset = WdtCycles::cyc1024,
    };
    bench.verdict("arm() accepts a normal-mode configuration",
                  Watchdog::arm(normal));
    bench.verdict("ENABLE reads back", Watchdog::enabled());
    bench.verdict("PER and EWOFFSET read back",
                  Watchdog::period() == WdtCycles::cyc16384 &&
                      Watchdog::ew_offset() == WdtCycles::cyc1024);
    bench.verdict("WEN is clear in normal mode", !Watchdog::window_mode());
    bench.verdict("the early-warning interrupt is armed",
                  (Watchdog::armed() & WdtFlag::early_warning) != 0u);
    bench.verdict("nothing is left synchronizing", !Watchdog::busy());

    constexpr WdtConfig windowed{
        .period = WdtCycles::cyc16384,
        .window_mode = true,
        .window = WdtCycles::cyc4096,
    };
    bench.verdict("arm() accepts a window-mode configuration",
                  Watchdog::arm(windowed));
    bench.verdict("WEN and WINDOW read back",
                  Watchdog::window_mode() &&
                      Watchdog::window() == WdtCycles::cyc4096);
    bench.verdict("the interrupt is disarmed again",
                  (Watchdog::armed() & WdtFlag::early_warning) == 0u);

    bench.verdict("disable() stops it", Watchdog::disable() && !Watchdog::enabled());
    bench.verdict("and stopping it twice is not an error", Watchdog::disable());

    // THE ONE REFUSAL. 23.6.8.2: in normal mode an early-warning offset
    // at or past the period means the reset arrives first and the
    // interrupt never comes - a caller that asked for a warning would
    // silently get none, so the driver declines instead.
    constexpr WdtConfig useless{
        .period = WdtCycles::cyc256,
        .early_warning = true,
        .ew_offset = WdtCycles::cyc1024,
    };
    bench.verdict("an early warning that could never fire is refused",
                  !Watchdog::arm(useless));
    bench.verdict("and nothing was armed by the refusal", !Watchdog::enabled());
    bench.verdict("the same numbers ARE legal in window mode, where the "
                  "offset is not used",
                  Watchdog::config_valid(WdtConfig{
                      .period = WdtCycles::cyc256,
                      .window_mode = true,
                      .early_warning = true,
                      .ew_offset = WdtCycles::cyc1024}));

    (void)Watchdog::disable();
    bench.verdict("the board is left with the watchdog off", !Watchdog::enabled());
    restore_wdt_boot_regs();
}

// =============================================================================
// c - what OSCULP32K actually runs at
// =============================================================================
//
// The counter clock is nominally 1.024 kHz from OSCULP32K, and 23.5.3
// warns in as many words that "the exact time-out period may vary from
// device-to-device". The early-warning interrupt is a way to see the
// real rate without ever letting the dog bite: it fires at a known
// number of CLK_WDT_OSC cycles and the CPU times it against SysTick.
//
// SINGLE MEASUREMENTS DO NOT GIVE THE RATE, and the whole letter is
// built around that. Arming the watchdog and starting the clock are
// two different instants: the CTRLA and CLEAR writes cross into the
// 1.024 kHz domain and take up to a couple of its cycles to land, so
// every measurement carries a CONSTANT negative offset of a few
// milliseconds. Two measurements at different offsets subtract it away -
// the difference between a 1024-cycle warning and a 512-cycle one is
// exactly 512 cycles, whatever the start cost - and the offset itself
// then falls out as the leftover.
void tc_oscillator() {
    // One measurement: arm with the given early-warning offset, restart
    // the period, and time the interrupt. Returns 0 if it never came.
    const auto measure_normal = [](WdtCycles offset) -> uint32_t {
        const WdtConfig cfg{
            .period = WdtCycles::cyc16384,   // ~16 s, never reached
            .early_warning = true,
            .ew_offset = offset,
        };
        ew_seen = false;
        Watchdog::clear_flags();
        if (!Watchdog::arm(cfg)) {
            return 0;
        }
        const uint32_t t0 = cycles_now();
        Watchdog::clear();
        (void)Watchdog::sync();
        uint32_t waited = 0;
        while (!ew_seen && waited < 5u * SysClock::hz) {
            waited = cycles_now() - t0;
        }
        const uint32_t took = ew_seen ? (ew_cycles - t0) : 0u;
        (void)Watchdog::disable();
        return took;
    };

    const uint32_t short_cycles = cycles_now();
    (void)short_cycles;
    const uint32_t t512 = measure_normal(WdtCycles::cyc512);
    const uint32_t t1024 = measure_normal(WdtCycles::cyc1024);

    print(serial, "  normal mode : 512 cycles -> ", cycles_to_ms(t512),
          " ms, 1024 cycles -> ", cycles_to_ms(t1024), " ms (nominal ",
          wdt_nominal_ms(WdtCycles::cyc512), " and ",
          wdt_nominal_ms(WdtCycles::cyc1024), ")", crlf);

    bench.verdict("the early warning fires at the short offset", t512 != 0u);
    bench.verdict("and at the long one", t1024 != 0u);

    if (t512 != 0u && t1024 != 0u && t1024 > t512) {
        // THE DIFFERENTIAL. 512 cycles, with the arming cost subtracted
        // out because it is the same in both.
        const uint32_t d = t1024 - t512;
        // Hz x 1000, so the result reads in millihertz without floats:
        // 512 cycles in d CPU cycles at SysClock::hz.
        const uint32_t hz_x1000 =
            static_cast<uint32_t>(512ULL * 1000ULL * SysClock::hz / d);
        const uint32_t ms_512 = cycles_to_ms(d);
        print(serial, "  differential: 512 cycles = ", ms_512,
              " ms -> CLK_WDT_OSC ", hz_x1000 / 1000u, ".", hz_x1000 % 1000u,
              " Hz (nominal 1024)", crlf);

        // The constant the two single measurements were both carrying.
        const uint32_t predicted_512 = d;  // 512 cycles cost exactly d
        const uint32_t offset_ms =
            t512 > predicted_512 ? cycles_to_ms(t512 - predicted_512) : 0u;
        const uint32_t shortfall_ms =
            predicted_512 > t512 ? cycles_to_ms(predicted_512 - t512) : 0u;
        print(serial, "  the arming cost that every single measurement carries: ",
              offset_ms != 0u ? offset_ms : shortfall_ms, " ms ",
              offset_ms != 0u ? "late" : "early", crlf);

        // A generous band on the RATE itself: this is a check that the
        // right clock counts the right number of cycles, not a
        // calibration - OSCULP32K's tolerance is wide and the chapter
        // says so.
        bench.verdict("CLK_WDT_OSC is within 10% of its nominal 1.024 kHz",
                      hz_x1000 > 921'600u && hz_x1000 < 1'126'400u);
        bench.verdict("the two offsets differ by exactly 512 cycles' worth "
                      "of time (the ratio is not 2:1, and that is the "
                      "arming cost showing)",
                      t1024 > t512 && t1024 < 2u * t512);
    }

    // WINDOW MODE reads a DIFFERENT register (CONFIG.WINDOW) with the
    // same encoding and the same clock: the warning marks the moment the
    // window opens. Measured once, to show the two registers agree.
    const WdtConfig windowed{
        .period = WdtCycles::cyc16384,
        .window_mode = true,
        .window = WdtCycles::cyc512,
        .early_warning = true,
    };
    ew_seen = false;
    Watchdog::clear_flags();
    const bool armed_window = Watchdog::arm(windowed);
    const uint32_t t1 = cycles_now();
    uint32_t waited = 0;
    while (!ew_seen && waited < 5u * SysClock::hz) {
        waited = cycles_now() - t1;
    }
    const uint32_t window_cycles = ew_seen ? (ew_cycles - t1) : 0u;
    (void)Watchdog::disable();

    print(serial, "  window mode : a 512-cycle closed window opened after ",
          cycles_to_ms(window_cycles), " ms", crlf);
    bench.verdict("arming window mode succeeded", armed_window);
    bench.verdict("the warning marks the window opening", ew_seen);
    bench.verdict("CONFIG.WINDOW counts the same clock as EWCTRL.EWOFFSET",
                  window_cycles != 0u && t512 != 0u &&
                      window_cycles > t512 - t512 / 10u &&
                      window_cycles < t512 + t512 / 10u);

    bench.verdict("the board is left with the watchdog off", !Watchdog::enabled());
    restore_wdt_boot_regs();
}

// =============================================================================
// i - six real resets (outside z: it reboots the board)
// =============================================================================
//
// Each leg banks its number in the .noinit token, triggers a reset, and
// then SPINS - because not every trigger here is instantaneous. A wrong
// key written to CLEAR is synchronized into the 1.024 kHz domain and
// arrives a few milliseconds later, so a leg that does not spin runs on
// past its own trigger and into the next one.

void bank(uint8_t leg) {
    token.magic = token_magic;
    token.canary = token_canary;
    token.leg = leg;
    token.pass = bench.passed();
    token.fail = bench.failed();
}

/// Announce a leg, get the words out, and never come back.
[[noreturn]] void await_reset(const char* what) {
    print(serial, "  ", what, crlf);
    console_drain();
    for (;;) {
    }
}

/// Legs 1 and 2: a deliberately wrong key in CLEAR, with the watchdog
/// stopped and then running. The chapter's sentence lives in the
/// Normal-mode section (23.6.2.4), which leaves the stopped case open.
[[noreturn]] void leg_bad_key(uint8_t leg, bool watchdog_running) {
    bank(leg);
    if (watchdog_running) {
        (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc16384});
    } else {
        (void)Watchdog::disable();
    }
    print(serial, "  leg ", leg, ": a wrong key written to CLEAR, watchdog ",
          watchdog_running ? "RUNNING" : "STOPPED", " ...", crlf);
    console_drain();
    Watchdog::force_reset();
    await_reset("(waiting for it to land - the write is synchronized)");
}

/// Leg 3: panic() with the reporter that resets, so the breadcrumb has
/// to survive a system reset to be read at all.
[[noreturn]] void leg_panic() {
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = 0x5A;
    bank(3);
    print(serial, "  leg 3: panic() through ResetReporter ...", crlf);
    console_drain();
    panic<SamPlatform, ResetReporter>(PanicCode::assert_failed, 0x5A);
}

/// Leg 4: a deliberate HardFault, caught by hard_fault_reset().
///
/// UDF, and it has to be UDF. The obvious candidate - an unaligned
/// volatile word load, which ARMv6-M cannot perform - does NOT fault:
/// gcc knows the constant's misalignment and emits four byte loads with
/// shifts instead, so the "fault" reads valid SRAM and returns. The
/// permanently-undefined instruction is the one thing no compiler can
/// turn into something legal.
[[noreturn]] void leg_fault() {
    token.code = static_cast<uint8_t>(PanicCode::kernel_fault);
    token.context = 0x77;
    bank(4);
    print(serial, "  leg 4: UDF -> HardFault -> hard_fault_reset() ...", crlf);
    console_drain();
    __asm__ volatile("udf #0");
    await_reset("(the undefined instruction did not fault)");
}

/// Leg 5: let the watchdog time out.
[[noreturn]] void leg_timeout() {
    bank(5);
    print(serial, "  leg 5: watchdog at ~8 ms with nothing to feed it ...", crlf);
    console_drain();
    (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc8});
    await_reset("(waiting for the time-out)");
}

/// Leg 6: violate the closed window by feeding the dog too early.
[[noreturn]] void leg_window() {
    bank(6);
    print(serial, "  leg 6: window mode, cleared INSIDE the closed window ...",
          crlf);
    console_drain();
    (void)Watchdog::arm(WdtConfig{
        .period = WdtCycles::cyc16384,
        .window_mode = true,
        .window = WdtCycles::cyc16384,   // ~16 s closed: the clear below is early
    });
    Watchdog::clear();
    await_reset("(waiting for the violation to land)");
}

void ti_resets() {
    report_boot();
    bench.verdict("this boot names a reset source",
                  boot_cause != ResetCause::unknown);
    bench.verdict("no panic record is pending on a clean start", !boot_record);
    bench.verdict("the watchdog is ours to drive",
                  !Watchdog::enabled() && !Watchdog::always_on());
    if (Watchdog::always_on()) {
        print(serial, "  the watchdog is always-on: the reset legs are SKIPPED",
              crlf);
        return;
    }
    leg_bad_key(1, false);
}

/// Everything after a reset. Called from main() instead of the banner,
/// and it either starts the next leg (never returning) or closes the
/// letter.
void ti_resume() {
    bench.resume_tally(token.pass, token.fail);

    const uint8_t leg = token.leg;
    print(serial, crlf, "i (continued after reset ", leg, " of 6)", crlf);
    report_boot();

    if (leg == 1) {
        // THE UNDOCUMENTED CASE, and the answer is that the key bites
        // regardless of CTRLA.ENABLE.
        bench.verdict("a wrong CLEAR key resets even with the watchdog STOPPED "
                      "(23.6.2.4 is not limited to a running watchdog)",
                      boot_cause == ResetCause::watchdog);
        bench.verdict("and the reset controller calls it a WATCHDOG reset",
                      boot_cause == ResetCause::watchdog);
        bench.verdict("with no panic record, since nothing wrote one",
                      !boot_record);
        leg_bad_key(2, true);
    }

    if (leg == 2) {
        bench.verdict("the same key with the watchdog RUNNING resets too "
                      "(the documented case)",
                      boot_cause == ResetCause::watchdog);
        leg_panic();
    }

    if (leg == 3) {
        // NOTE what had to happen for this to work at all: panic() ends
        // in break_here(), which is BKPT, and with no debugger attached
        // that escalates into HardFault_Handler - so the reset here may
        // have come from ResetReporter or from hard_fault_reset(), and
        // either way the RECORD must still say what panic() reported.
        // That is why the fault body refuses to overwrite a valid one.
        bench.verdict("a panic through ResetReporter resets the device",
                      boot_cause == ResetCause::system_request);
        bench.verdict("the breadcrumb survived the reset (SRAM is promised "
                      "nowhere, so this is a measurement)",
                      boot_record.has_value());
        bench.verdict("its code is the one panic() was given",
                      boot_record && boot_record->code == token.code);
        bench.verdict("its context byte came through untouched",
                      boot_record && boot_record->context == token.context);
        bench.verdict("a system reset request and a watchdog reset are "
                      "DISTINGUISHABLE at the next boot",
                      boot_cause == ResetCause::system_request);
        leg_fault();
    }

    if (leg == 4) {
        bench.verdict("a HardFault reaches hard_fault_reset() and resets",
                      boot_cause == ResetCause::system_request);
        bench.verdict("the record it left says kernel_fault",
                      boot_record && boot_record->code == token.code);
        bench.verdict("with the context byte the body was given",
                      boot_record && boot_record->context == token.context);
        leg_timeout();
    }

    if (leg == 5) {
        bench.verdict("a watchdog time-out resets the device",
                      boot_cause == ResetCause::watchdog);
        bench.verdict("and leaves no panic record behind", !boot_record);
        bench.verdict("the watchdog came back at its FUSE setting, not the one "
                      "that bit us",
                      !Watchdog::enabled());
        leg_window();
    }

    // leg 6: the last boot.
    bench.verdict("a clear inside the closed window resets the device",
                  boot_cause == ResetCause::watchdog);
    bench.verdict("the token crossed all six resets intact",
                  token.magic == token_magic && token.canary == token_canary);

    token.leg = 0;
    token.magic = 0;
    bench.end_letter();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_samc_platform - SAMC21J18A RSTC (ch. 18) + WDT "
          "(ch. 23), clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}


// ===========================================================================
// d - delay_us: the SysTick microsecond wait (samc21/delay.hpp)
// ===========================================================================

/// The ruler is a TC0+TC1 pair at CLK_MAIN undivided - the same
/// oscillator SysTick rides, deliberately: what this letter judges is
/// the ARITHMETIC and the wrap handling (an error there is cycles or
/// whole periods, visible on any shared scale), not the oscillator.
void td_delay() {
    using Ruler = Tc<0>;
    (void)Ruler::enable(false);
    bool ruler_ok = Ruler::init(0) &&
                    Ruler::configure(TcConfig{.mode = TcMode::count32,
                                              .prescaler = TcPrescaler::div1}) &&
                    Ruler::enable(true);
    bench.verdict("the TC ruler comes up on generator 0", ruler_ok);
    if (!ruler_ok) {
        return;
    }
    constexpr uint32_t per_us = SysClock::hz / 1'000'000UL;   // 48

    // THE BRACKET'S OWN ZERO, measured first: two back-to-back
    // count32() reads are not free (each is a READSYNC command and its
    // waits), and charging that cost to the delay would put a constant
    // ~11 us of "lateness" on every span and make even a REFUSAL "take"
    // 10 us. The empty bracket is sampled eight
    // times; its MAX bounds what measurement overhead can add, and the
    // at-least verdicts below deliberately use the RAW reading (the
    // bracket only ever adds, so raw >= true is a safe witness).
    uint32_t zero_min = 0xFFFFFFFFu;
    uint32_t zero_max = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const uint32_t t0 = Ruler::count32();
        const uint32_t b = (Ruler::count32() - t0) / per_us;
        if (b < zero_min) zero_min = b;
        if (b > zero_max) zero_max = b;
    }
    print(serial, "  empty measurement bracket: ", zero_min, "..", zero_max,
          " us of its own", crlf);

    // Exactness across the range the cap allows. The upper band pays
    // the bracket plus the call's own overhead (folded conversion, one
    // division by a folded constant, poll granularity) plus at most
    // one tick interrupt per millisecond of wait.
    static const uint32_t spans[] = {5, 30, 100, 500, 900};
    bool all_exact = true;
    for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        const uint32_t t0 = Ruler::count32();
        const bool ok = delay_us(clock, spans[i]);
        const uint32_t took = (Ruler::count32() - t0) / per_us;
        print(serial, "  delay_us(", spans[i], ") -> ", took, " us measured", crlf);
        if (!ok || took < spans[i] || took > spans[i] + zero_max + 10u) all_exact = false;
    }
    bench.verdict("delay_us serves 5..900 us AT LEAST and within the bracket's "
                  "zero plus 10 us on the TC ruler",
                  all_exact);

    // Never early, statistically: two hundred waits landing at whatever
    // SysTick phase the loop reaches them in - the wrap path is walked
    // many times over.
    uint32_t min_took = 0xFFFFFFFFu;
    uint32_t max_took = 0;
    for (uint16_t k = 0; k < 200; ++k) {
        const uint32_t t0 = Ruler::count32();
        (void)delay_us(clock, 50u);
        const uint32_t took = (Ruler::count32() - t0) / per_us;
        if (took < min_took) min_took = took;
        if (took > max_took) max_took = took;
    }
    print(serial, "  200 x delay_us(50): min ", min_took, " max ", max_took, " us", crlf);
    bench.verdict("two hundred 50 us waits: NOT ONE EARLY, whatever the counter "
                  "phase (the wrap arithmetic walked at every offset)",
                  min_took >= 50u);

    // The cap: one tick period or more is refused, and refusing costs
    // nothing measurable. MIN OVER FOUR tries against the bracket's own
    // MIN: a tick interrupt landing inside any single bracket inflates
    // it by its handler, and min-vs-min is what filters that noise out
    // (a refusal is side-effect free, so repeating it is free too).
    uint32_t refusal_us = 0xFFFFFFFFu;
    bool refused = true;
    for (uint8_t k = 0; k < 4; ++k) {
        const uint32_t t0 = Ruler::count32();
        refused = refused && !delay_us(clock, 1000u);
        const uint32_t r = (Ruler::count32() - t0) / per_us;
        if (r < refusal_us) refusal_us = r;
    }
    const bool served_999 = delay_us(clock, 999u);
    print(serial, "  delay_us(1000) refused=", refused, " in ", refusal_us,
          " us (min of 4); delay_us(999) served=", served_999, crlf);
    bench.verdict("the CAP is real: a whole tick (1000 us) is REFUSED spending "
                  "nothing beyond the bracket's own zero - TimeEvent territory - "
                  "while 999 us is served",
                  refused && refusal_us <= zero_min + 3u && served_999);

    // No Ticker, no time: with SysTick stopped the answer is false, at
    // once. The counter holds its VAL across the pause, so the tick
    // slips by only the microseconds of this leg.
    SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_ENABLE_Msk;
    uint32_t stopped_us = 0xFFFFFFFFu;
    bool refused_stopped = true;
    for (uint8_t k = 0; k < 4; ++k) {
        const uint32_t t1 = Ruler::count32();
        refused_stopped = refused_stopped && !delay_us(clock, 100u);
        const uint32_t r = (Ruler::count32() - t1) / per_us;
        if (r < stopped_us) stopped_us = r;
    }
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_ENABLE_Msk;
    print(serial, "  SysTick stopped: refused=", refused_stopped, " in ",
          stopped_us, " us (min of 4)", crlf);
    bench.verdict("with SysTick not running delay_us answers false and spends "
                  "nothing beyond the bracket (a program with no Ticker has no "
                  "clock to count on)",
                  refused_stopped && stopped_us <= zero_min + 3u);

    Ruler::release();
}

// ===========================================================================
// w - the idle hook against an interrupt's edge, placed to the cycle
// ===========================================================================
//
// THE PROMISE asked of the platform (design/kernel.md section 8's idle
// call): idle() returns after any interrupt pending when it is called or
// arriving at ANY instruction of the idle path - the masked check, the
// call, the SLEEPCFG read, the DSB, the WFI - and once per interrupt
// taken. ARMv6-M's WFI wakes on an interrupt that would preempt with
// PRIMASK clear (B1.5.19 of the ARMv6-M ARM), so the promise holds by
// construction on paper; this letter asks the silicon, at every cycle.

/// The TC0+TC1 pair, 32 bits at CLK_MAIN undivided, RUNSTDBY set: each
/// try's STOPWATCH (COUNT from zero at the try's start), its RESCUE (CC0
/// at 50 ms, an interrupt that ends any sleep nothing else would), and,
/// for the line passes, THE EDGE ITSELF (CC1 at D). With RUNSTDBY the
/// pair asks for its clock through a standby (platform.md, "Sleep,
/// peripheral by peripheral"), so the same instrument serves every mode.
using Watch = Tc<0>;
constexpr uint32_t tick_cycles = SysClock::hz / Ticker::ticks_per_second;   // 48 000
constexpr uint32_t rescue_cycles = SysClock::hz / 20u;                      // 50 ms
constexpr uint32_t positions = 400;
constexpr uint32_t give_up = 16;   ///< failures that end a pass early
constexpr uint32_t far_away = 0xFFFF'FF00u;

volatile uint32_t rescue_periods = 0;
volatile uint32_t line_edges = 0;

/// A line no peripheral drives in this image: the software interrupt
/// (`Nvic::set_pending`) whose handler stamps the SysTick counter.
constexpr IRQn_Type spare_line = PTC_IRQn;
volatile uint32_t spare_served = 0;
volatile uint32_t spare_stamp = 0;

bool watch_up() {
    (void)Watch::enable(false);
    const bool ok = Watch::init(0) &&
                    Watch::configure(TcConfig{.mode = TcMode::count32,
                                              .prescaler = TcPrescaler::div1,
                                              .run_standby = true}) &&
                    Watch::set_cc32(0, rescue_cycles) && Watch::set_cc32(1, far_away) &&
                    Watch::enable(true);
    Watch::clear_flags(static_cast<uint8_t>(Watch::overflow_flag | Watch::match_flag(0) |
                                            Watch::match_flag(1)));
    Watch::arm(static_cast<uint8_t>(Watch::match_flag(0) | Watch::match_flag(1)));
    Nvic::clear_pending(Watch::irq());
    Nvic::enable(Watch::irq());
    return ok;
}

void watch_down() {
    Watch::disarm(static_cast<uint8_t>(Watch::match_flag(0) | Watch::match_flag(1)));
    (void)Watch::enable(false);
    Watch::release();
}

/// The SysTick counter's distance from `a` down to `b`, folding one
/// reload (it counts DOWN from LOAD).
uint32_t down_cycles(uint32_t a, uint32_t b) {
    return a >= b ? a - b : a + tick_cycles - b;
}

enum class EdgeSource : uint8_t {
    tick,   ///< the kernel's own wake: SysTick, a core exception
    line,   ///< an NVIC line: the watch's CC1 match
};

struct Try {
    bool rescued;
    uint32_t took;      ///< watch cycles from the try's start to the caller's loop
    uint32_t to_loop;   ///< cycles from the edge to the caller's loop
    uint32_t read;      ///< one synchronized COUNT read, for the line source
    uint32_t ran;       ///< SysTick's cycles over the same span, for the line source
};

/// One try in the kernel's own shape - a masked check, idle(), until
/// `edges` interrupts have been served - with the edge D cycles after the
/// placement.
///
/// THE TICK is placed through LOAD: a write to VAL clears it, the next
/// clock reloads LOAD (ARMv6-M ARM B3.3.1), so with LOAD = D + 1 for that
/// one reload the 1-to-0 transition comes D + 2 cycles after the clear,
/// and LOAD is back at the tick's period before it - so the period after
/// the placed edge is a whole tick again (judged below: the second edge
/// of a two-edge try is a period after the first). The natural edge is
/// kept away (the counter far from zero, PENDSTCLR) so the tick taken is
/// the placed one. These writes shorten the tick they land in, so kernel
/// time runs fast during the letter.
///
/// THE LINE is placed through CC1: COUNT parked far from both matches,
/// CC1 = D, then COUNT written to zero - the match comes D counter cycles
/// after that write lands.
Try edge_try(EdgeSource src, uint32_t d, uint32_t edges) {
    Try r{};
    P::CriticalSection cs;
    const uint32_t r0 = rescue_periods;
    if (src == EdgeSource::line) {
        (void)Watch::set_count32(0x8000'0000u);
        (void)Watch::set_cc32(1, d);
        Watch::clear_flags(Watch::match_flag(1));
        Nvic::clear_pending(Watch::irq());
        const uint32_t e0 = line_edges;
        (void)Watch::set_count32(0);
        const uint32_t v0 = SysTick->VAL;
        for (;;) {
            P::CriticalSection turn;   // the kernel's own shape
            if (line_edges != e0) {
                break;
            }
            P::idle();
        }
        r.ran = down_cycles(v0, SysTick->VAL);
        const uint32_t first = Watch::count32();
        r.took = Watch::count32();
        r.read = r.took - first;
        r.to_loop = first - d;
    } else {
        while (SysTick->VAL < 4096u) {
        }
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
        (void)Watch::set_count32(0);
        const uint32_t t = Ticker::ticks();
        SysTick->LOAD = d + 1u;
        SysTick->VAL = 0u;
        SysTick->LOAD = tick_cycles - 1u;
        for (;;) {
            P::CriticalSection turn;   // the kernel's own shape
            if (Ticker::ticks() - t >= edges) {
                break;
            }
            P::idle();
        }
        r.to_loop = tick_cycles - SysTick->VAL;   // since the reload the edge made
        (void)Watch::count32();
        r.took = Watch::count32();
    }
    r.rescued = rescue_periods != r0;
    if (src == EdgeSource::line) {
        (void)Watch::set_cc32(1, far_away);
    }
    return r;
}

struct Walk {
    uint32_t lost = 0;           ///< tries the rescue ended
    uint32_t late = 0;           ///< tries that ended a whole tick late
    uint32_t first_bad = 0;
    uint32_t fastest = 0xFFFF'FFFFu;
    uint32_t slowest = 0;
    uint32_t pending_to_loop = 0;   ///< D = 0: the edge is in before the check
    uint32_t sleep_entry = 0;       ///< the D of the least to_loop: the sleep's entry
    uint32_t least_to_loop = 0xFFFF'FFFFu;
    uint32_t asleep_to_loop = 0;    ///< the last D: the edge finds the core asleep
    uint32_t read = 0;
    uint32_t took_last = 0;   ///< the last D's took and ran: what SysTick saw of it
    uint32_t ran_last = 0;
    uint32_t tried = 0;
};

Walk walk(EdgeSource src, uint32_t edges) {
    Walk w;
    // A wake lost until the NEXT tick ends a whole period late; the bound
    // sits a quarter period short of that, far above any kept wake (the
    // edges land within D + a few hundred cycles of the period they need).
    const uint32_t late = edges * tick_cycles - tick_cycles * 3u / 4u;
    for (uint32_t d = 0; d < positions; ++d) {
        const Try t = edge_try(src, d, edges);
        ++w.tried;
        if (t.took > w.slowest) {
            w.slowest = t.took;
        }
        if (t.took < w.fastest) {
            w.fastest = t.took;
        }
        const bool bad = t.rescued || t.took > late;
        if (bad) {
            if (w.lost + w.late == 0u) {
                w.first_bad = d;
            }
            if (t.rescued) {
                ++w.lost;
            } else {
                ++w.late;
            }
            if (w.lost + w.late >= give_up) {
                break;
            }
            continue;
        }
        if (d == 0u) {
            w.pending_to_loop = t.to_loop;
            w.read = t.read;
        }
        // While the edge lands before the WFI, every cycle later it lands
        // is a cycle less to the loop, down to the edge that meets the WFI
        // itself; past it the distance is the wake's own, and longer.
        if (t.to_loop < w.least_to_loop) {
            w.least_to_loop = t.to_loop;
            w.sleep_entry = d;
        }
        w.asleep_to_loop = t.to_loop;
        w.took_last = t.took - t.read;
        w.ran_last = t.ran;
    }
    return w;
}

/// The STANDBY witness: TC2, 16 bits at CLK_MAIN with RUNSTDBY CLEAR, so
/// "halted in standby" (35.7.2.1, CTRLA.RUNSTDBY) while the watch runs on
/// - a positive sign that the PM really took the standby. SysTick cannot
/// be that sign: measured below, its counter runs through a standby on
/// this bench.
using Witness = Tc<2>;

struct Spans {
    uint32_t watch;     ///< the watch, from its zero to the first read after the wake
    uint32_t witness;   ///< the witness, across the same try and the two reads around it
};

/// One try with the line D = 2000 cycles out, the witness read on both
/// sides.
Spans witness_try(SleepMode mode) {
    (void)Pm::set_sleep_mode(mode);
    SysTickInterruptGuard quiet;
    P::CriticalSection cs;
    (void)Watch::set_count32(0x8000'0000u);
    (void)Watch::set_cc32(1, 2000u);
    Watch::clear_flags(Watch::match_flag(1));
    Nvic::clear_pending(Watch::irq());
    const uint32_t e0 = line_edges;
    const uint16_t a = Witness::count16();
    (void)Watch::set_count32(0);
    for (;;) {
        P::CriticalSection turn;
        if (line_edges != e0) {
            break;
        }
        P::idle();
    }
    const uint32_t w = Watch::count32();
    const uint16_t b = Witness::count16();
    (void)Watch::set_cc32(1, far_away);
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    return Spans{w, static_cast<uint16_t>(b - a)};
}

/// THE OTHER STANDBY: no clock request of this letter's own holds
/// generator 0 (the watch is released first) and the RTC on OSCULP32K is
/// the wake, 160 of its ticks (4.9 ms) out. The witness, now at CLK_MAIN / 16 so that span
/// fits its sixteen bits, says the standby was entered; SysTick, its
/// period stretched to the whole 24 bits for the measurement (349 ms, so
/// no reload can fold the reading - COUNTFLAG cannot tell, the erratum
/// guard's read of CTRL in idle() clears it), says how far its counter
/// ran through it. Its interrupt is held off either way. The watchdog is
/// the backstop: a wake that never comes reboots the board into its
/// banner instead of leaving it mute.
volatile bool rtc_fired = false;

struct Unheld {
    bool woke;
    uint32_t witness;   ///< the witness's counts across it, CLK_MAIN / 16
    uint32_t systick;   ///< SysTick's cycles across it
};

Unheld unheld_sleep(SleepMode mode) {
    Unheld u{false, 0, 0};
    (void)Rtc::enable(false);
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    if (!Rtc::init() ||
        !Rtc::configure(RtcConfig{.mode = RtcMode::count32, .prescaler = RtcPrescaler::div1}) ||
        !Rtc::enable(true)) {
        return u;
    }
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::compare0);
    Nvic::clear_pending(Rtc::irq());
    Nvic::enable(Rtc::irq());
    (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc4096});
    {
        SysTickInterruptGuard quiet;
        P::CriticalSection cs;
        rtc_fired = false;
        (void)Rtc::set_comp32(Rtc::count32() + 160u);
        (void)Pm::set_sleep_mode(mode);
        SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
        SysTick->VAL = 0u;
        const uint16_t a = Witness::count16();
        const uint32_t v0 = SysTick->VAL;
        for (;;) {
            P::CriticalSection turn;
            if (rtc_fired) {
                break;
            }
            P::idle();
        }
        const uint32_t v1 = SysTick->VAL;
        u.systick = v0 - v1;
        SysTick->LOAD = tick_cycles - 1u;
        SysTick->VAL = 0u;
        u.witness = static_cast<uint16_t>(Witness::count16() - a);
        u.woke = rtc_fired;
        (void)Pm::set_sleep_mode(SleepMode::idle0);
    }
    (void)Watchdog::disable();
    restore_wdt_boot_regs();
    Nvic::disable(Rtc::irq());
    Rtc::disarm(RtcFlag::all);
    Rtc::release();
    return u;
}

void tw_edge() {
    console_drain();
    const bool up = watch_up();
    bench.verdict("the watch (TC0+TC1, CLK_MAIN undivided, RUNSTDBY) comes up", up);
    if (!up) {
        return;
    }

    // WHERE A COUNT READ CAPTURES: COUNT written to zero and read straight
    // back - the value is how far into Watch::count32() its snapshot is
    // taken, which every line figure below carries and is shown net of.
    uint32_t capture = 0xFFFF'FFFFu;
    for (int i = 0; i < 8; ++i) {
        P::CriticalSection cs;
        (void)Watch::set_count32(0);
        capture = std::min(capture, Watch::count32());
    }

    struct Pass {
        const char* name;
        SleepMode mode;
        EdgeSource src;
        uint32_t edges;
    };
    static constexpr Pass passes[] = {
        {"IDLE0, the tick, waiting for one", SleepMode::idle0, EdgeSource::tick, 1},
        {"IDLE0, the tick, waiting for two", SleepMode::idle0, EdgeSource::tick, 2},
        {"IDLE2, the tick, waiting for one", SleepMode::idle2, EdgeSource::tick, 1},
        {"IDLE2, the tick, waiting for two", SleepMode::idle2, EdgeSource::tick, 2},
        {"IDLE0, a TC line", SleepMode::idle0, EdgeSource::line, 1},
        {"IDLE2, a TC line", SleepMode::idle2, EdgeSource::line, 1},
        {"STANDBY, a TC line", SleepMode::standby, EdgeSource::line, 1},
    };
    constexpr size_t pass_count = sizeof(passes) / sizeof(passes[0]);
    Walk walks[pass_count];
    bool armed = true;
    for (size_t i = 0; i < pass_count; ++i) {
        armed = Pm::set_sleep_mode(passes[i].mode) && armed;
        if (passes[i].src == EdgeSource::line) {
            // The line is the ONLY wake: a lost one waits for the rescue,
            // not for the next tick.
            SysTickInterruptGuard quiet;
            walks[i] = walk(passes[i].src, passes[i].edges);
        } else {
            walks[i] = walk(passes[i].src, passes[i].edges);
        }
    }
    (void)Pm::set_sleep_mode(SleepMode::idle0);

    // The positive witness that the STANDBY pass slept in standby.
    (void)Witness::enable(false);
    const bool witness_ok =
        Witness::init(0) &&
        Witness::configure(TcConfig{.mode = TcMode::count16, .prescaler = TcPrescaler::div1}) &&
        Witness::enable(true);
    const Spans in_idle = witness_try(SleepMode::idle0);
    const Spans in_standby = witness_try(SleepMode::standby);

    // WHAT IDLE2 AND STANDBY PAY AT THE WAKE: the same IDLE2 pass again
    // under each of NVMCTRL's two other sleep power settings (27.8.2
    // CTRLB.SLEEPPRM; 27.5.1: STANDBY puts the flash in low power
    // whatever it says), then the register put back as found.
    const uint32_t ctrlb = Nvm::ctrlb();
    const NvmConfig found{
        .wait_states = FlashWaitStates::get(),
        .cache = Nvm::cache_enabled(),
        .read_mode = Nvm::read_mode(),
        .sleep_power = static_cast<NvmSleepPower>((ctrlb & NVMCTRL_CTRLB_SLEEPPRM_Msk) >>
                                                  NVMCTRL_CTRLB_SLEEPPRM_Pos),
        .manual_write = Nvm::manual_write(),
    };
    constexpr NvmSleepPower settings[] = {NvmSleepPower::wake_on_exit, NvmSleepPower::disabled};
    Walk nvm_walks[2];
    for (size_t i = 0; i < 2u; ++i) {
        NvmConfig c = found;
        c.sleep_power = settings[i];
        (void)Nvm::init(c);
        armed = Pm::set_sleep_mode(SleepMode::idle2) && armed;
        nvm_walks[i] = walk(EdgeSource::tick, 1);
    }
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    (void)Nvm::init(found);
    const bool restored = Nvm::ctrlb() == ctrlb;

    watch_down();
    (void)Witness::enable(false);
    const bool slow_witness =
        Witness::configure(TcConfig{.mode = TcMode::count16, .prescaler = TcPrescaler::div16}) &&
        Witness::enable(true);
    const Unheld unheld_idle = unheld_sleep(SleepMode::idle0);
    const Unheld unheld = unheld_sleep(SleepMode::standby);
    (void)Witness::enable(false);
    Witness::release();
    bench.verdict("every mode armed and read back (19.6.3.3)", armed);

    for (size_t i = 0; i < pass_count; ++i) {
        const Walk& w = walks[i];
        print(serial, "  ", passes[i].name, ": ", w.tried, " edge positions, lost ", w.lost,
              ", late ", w.late);
        if (w.lost + w.late != 0u) {
            print(serial, " (first at D=", w.first_bad, ")");
        }
        print(serial, "; ", w.fastest, "..", w.slowest, " cycles a try", crlf);
        const uint32_t net = passes[i].src == EdgeSource::line ? capture : 0u;
        print(serial, "    edge to the caller's loop: ", w.pending_to_loop - net,
              " cycles pending at D=0, ", w.asleep_to_loop - net,
              " finding the core asleep (the least, ", w.least_to_loop - net, ", at D=",
              w.sleep_entry, ")", crlf);
        if (passes[i].src == EdgeSource::line) {
            print(serial, "    the last try: ", w.took_last, " cycles on the watch, ", w.ran_last,
                  " of them on SysTick (a COUNT read is ", w.read, " cycles, its snapshot ",
                  capture, " in)", crlf);
        }
    }
    print(serial, "  STANDBY witness (TC2, RUNSTDBY clear) over a line 2000 cycles out: ",
          in_idle.witness, " of ", in_idle.watch, " watch cycles in IDLE0, ", in_standby.witness,
          " of ", in_standby.watch, " in STANDBY", crlf);
    static const char* const setting_names[] = {"WAKEUPINSTANT", "DISABLED"};
    for (size_t i = 0; i < 2u; ++i) {
        print(serial, "  IDLE2, the tick, NVMCTRL SLEEPPRM ", setting_names[i], ": lost ",
              nvm_walks[i].lost, ", late ", nvm_walks[i].late, "; edge to the loop ",
              nvm_walks[i].asleep_to_loop, " cycles finding the core asleep", crlf);
    }
    print(serial, "  (the image runs with SLEEPPRM ", static_cast<uint8_t>(found.sleep_power),
          ", the reset value WAKEUPACCESS being 0)", crlf);
    print(serial, "  generator 0 unheld, the RTC 160 ticks out: IDLE0 - the witness ",
          unheld_idle.witness, " counts of 16 cycles, SysTick ", unheld_idle.systick,
          " cycles; STANDBY - the witness ", unheld.witness, ", SysTick ", unheld.systick, crlf);
    print(serial, "  (SysTick's counter ", unheld.systick * 2u > unheld_idle.systick ? "RAN" : "STOOD",
          " through that standby, and through the line passes' it ",
          walks[6].took_last - walks[6].ran_last < walks[4].took_last - walks[4].ran_last + 32u
              ? "RAN"
              : "STOOD",
          "; kernel time stands still across a standby either way, the erratum guard "
          "holding the tick's interrupt off)", crlf);
    bench.verdict("the witness came up", witness_ok);
    bench.verdict("STANDBY was entered: the witness stood still across it, by over a "
                  "thousand cycles more than in IDLE0",
                  in_standby.witness + 1000u < in_idle.witness);
    bench.verdict("no lost or late wake under either other SLEEPPRM, and NVMCTRL put back",
                  nvm_walks[0].lost + nvm_walks[0].late + nvm_walks[1].lost + nvm_walks[1].late ==
                          0u &&
                      restored);
    for (size_t i = 0; i < pass_count; ++i) {
        bench.verdict(passes[i].name, walks[i].lost == 0u && walks[i].late == 0u &&
                                          walks[i].tried == positions);
    }
    bench.verdict("the placed tick leaves a whole period behind it (the second edge "
                  "a tick after the first)",
                  walks[1].fastest >= tick_cycles && walks[3].fastest >= tick_cycles);
    bench.verdict("a STANDBY with generator 0 unheld: entered (the witness under a quarter "
                  "of IDLE0's count) and woken by the RTC",
                  slow_witness && unheld.woke && unheld_idle.woke &&
                      unheld.witness * 4u < unheld_idle.witness);
    bench.verdict("the walk reaches past the sleep's entry in every one-edge pass",
                  [&] {
                      for (size_t i = 0; i < pass_count; ++i) {
                          if (passes[i].edges == 1u && walks[i].sleep_entry + 16u >= positions) {
                              return false;
                          }
                      }
                      return true;
                  }());

    // THE CORE'S ROUND TRIP on the spare line, pended under the mask and
    // taken at the unmask: from the cpsie to the handler's first load of
    // the counter, and from there back to the thread. The bracket is two
    // back-to-back counter reads; every figure is the least of sixteen,
    // which filters a tick landing inside one.
    spare_served = 0;
    Nvic::clear_pending(spare_line);
    Nvic::enable(spare_line);
    uint32_t bracket = 0xFFFF'FFFFu;
    uint32_t entry = 0xFFFF'FFFFu;
    uint32_t exit = 0xFFFF'FFFFu;
    uint32_t at_once = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        disable_interrupts();
        const uint32_t b0 = SysTick->VAL;
        const uint32_t b1 = SysTick->VAL;
        Nvic::set_pending(spare_line);
        const uint32_t c0 = SysTick->VAL;
        enable_interrupts();
        const uint32_t c1 = SysTick->VAL;
        const uint32_t h = spare_stamp;
        bracket = std::min(bracket, down_cycles(b0, b1));
        entry = std::min(entry, down_cycles(c0, h));
        exit = std::min(exit, down_cycles(h, c1));

        // An idle() a pending line returns at once: the hook's whole cost
        // with the round trip it unmasks into.
        disable_interrupts();
        Nvic::set_pending(spare_line);
        const uint32_t i0 = SysTick->VAL;
        P::idle();
        const uint32_t i1 = SysTick->VAL;
        at_once = std::min(at_once, down_cycles(i0, i1));
    }
    Nvic::disable(spare_line);
    print(serial, "  the core's round trip on a pended line: ", entry, " cycles to the handler's "
          "first load, ", exit, " from there back to the thread (a counter read is ", bracket,
          ")", crlf);
    print(serial, "  an idle() a pending line returns at once: ", at_once,
          " cycles, the round trip and two counter reads included", crlf);
    bench.verdict("the spare line was taken once per pend (Nvic::set_pending as a "
                  "software interrupt)",
                  spare_served == 32u);
    spare_served = 0;
}

// ===========================================================================
// k - the kernel's turn: how many per interrupt, and what one costs
// ===========================================================================
// A pack shaped like a small program's: three AOs, two of them with a
// periodic time event (500 ms and 1000 ms, so none fires in the 100 ms
// this letter idles), none of them ever posted to.
template <uint8_t N>
struct Quiet {
    struct Event {
        uint8_t n;
    };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Quiet, Event> beat{Event{N}};
    static void init() {
        if constexpr (N != 0u) {
            beat.arm_every(ticks_from_ms<P>(500u * N));
        }
    }
    static void dispatch(const Event&) {}
};
using QuietKernel = Tenuto<P, Quiet<1>, Quiet<2>, Quiet<0>>;

/// One turn of Tenuto::run()'s loop, as it is written there.
[[gnu::always_inline]] inline void kernel_turn() {
    TimeEvents<P>::process();
    if (!QuietKernel::step()) {
        QuietKernel::idle_if_empty();
    }
}

void tk_turns() {
    console_drain();
    QuietKernel::init_all();

    // Turns over 100 quiet ticks: the tick is the only interrupt. The
    // loop starts inside a tick and ends at the wake that completes the
    // hundredth, so a kernel that turns once per wake turns 100 times; one
    // turn of allowance at each end (a wake already pending at the start,
    // a tick shortened by the start) makes the window 99..101.
    const uint32_t t = Ticker::ticks();
    uint32_t turns = 0;
    while (Ticker::ticks() - t < 100u) {
        kernel_turn();
        ++turns;
    }

    // A quiet turn whose idle() a pending line returns at once: the turn
    // an interrupt that wakes a quiet kernel costs, run under the mask
    // (process() and step() save and restore it, the same instructions
    // either way) with the line's round trip inside the idle().
    Nvic::clear_pending(spare_line);
    Nvic::enable(spare_line);
    uint32_t turn_cycles = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        disable_interrupts();
        Nvic::set_pending(spare_line);
        const uint32_t c0 = SysTick->VAL;
        kernel_turn();
        const uint32_t c1 = SysTick->VAL;
        enable_interrupts();
        turn_cycles = std::min(turn_cycles, down_cycles(c0, c1));
    }
    Nvic::disable(spare_line);
    spare_served = 0;
    TimeEvents<P>::clear_all();

    print(serial, "  100 quiet ticks: ", turns, " kernel turns; a quiet turn whose idle() a "
          "pending line returns at once: ", turn_cycles,
          " cycles (the line's round trip and two counter reads included)", crlf);
    bench.verdict("the kernel loop turns once per interrupt (99..101 over 100 ticks)",
                  turns >= 99u && turns <= 101u);
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler is
// a spin loop - so every line this suite can raise is bound.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// Letter w's watch: the placed edge (CC1) and the rescue (CC0).
extern "C" void TC0_Handler() {
    const uint8_t p = Watch::isr();
    if ((p & Watch::match_flag(1)) != 0u) {
        line_edges = line_edges + 1u;
    }
    if ((p & Watch::match_flag(0)) != 0u) {
        rescue_periods = rescue_periods + 1u;
    }
}

/// Letter w's unheld standby: its wake.
extern "C" void RTC_Handler() {
    (void)brio::Rtc::isr();
    rtc_fired = true;
}

/// Letters w and k's spare line: a stamp and a count, nothing else.
extern "C" void PTC_Handler() {
    spare_stamp = SysTick->VAL;
    spare_served = spare_served + 1u;
}

extern "C" void WDT_Handler() {
    if (brio::Watchdog::isr() != 0u) {
        ew_cycles = cycles_now();
        ew_seen = true;
    }
}

/// The whole point of samc21/reset.hpp's fault body: a crash becomes a
/// note the next boot can read, instead of a spin nobody sees.
extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::SamPlatform>(token.context);
}

namespace {

/// t: the runtime's seven functions (rt/rt.cpp) over every alignment,
/// length and overlap - rt/selftest.hpp's cases, the ones the host suite
/// runs, against the symbols this image really links.
void tt_runtime() {
    brio::rt_selftest_all(brio::rt_linked_functions(),
                          [](const char* name, const brio::RtSelftestResult& r) {
        print(serial, "  ", r.cases, " cases", crlf);
        if (r.failures != 0u) {
            print(serial, "  ", r.failures, " failed, the first: ", r.first_what, " ",
                  r.first_a, " ", r.first_b, crlf);
        }
        bench.verdict(name, r.failures == 0u);
    });
}

}  // namespace

int main() {
    // Sampled FIRST: RCAUSE is not cleared by anything, but the panic
    // record is fetch-and-clear and must be taken exactly once.
    boot_cause_bits = brio::Reset::cause_bits();
    boot_cause = brio::Reset::cause();
    boot_record = brio::take_panic_record<brio::SamPlatform>();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    wdt_boot_config = WDT_REGS->WDT_CONFIG;
    wdt_boot_ewctrl = WDT_REGS->WDT_EWCTRL;

    brio::Nvic::enable(WDT_IRQn);
    brio::enable_interrupts();

    bench.letter('a', "the boot story and the watchdog's fuses", ta_boot);
    bench.letter('b', "the watchdog as a configurable timer", tb_watchdog);
    bench.letter('c', "what OSCULP32K really runs at", tc_oscillator);
    bench.letter('d', "delay_us on the SysTick counter (samc21/delay.hpp)", td_delay);
    bench.letter('w', "THE IDLE HOOK against an interrupt's edge, placed to the cycle", tw_edge);
    bench.letter('k', "the kernel's turn: one per interrupt, and what one costs", tk_turns);
    bench.letter('i', "SIX REAL RESETS (reboots the board)", ti_resets, false);
    bench.letter('t', "the runtime's seven functions at every alignment (rt/rt.cpp)", tt_runtime);

    // A pending token means a leg of letter i is waiting to be judged:
    // resume it instead of printing a banner nobody asked for.
    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", " cause=",
              cause_name(boot_cause), crlf);
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
        bench.prompt();
    }
}
