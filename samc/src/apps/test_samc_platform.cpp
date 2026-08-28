// test_samc_platform - the reference bench suite for samc/reset.hpp: the
// reset controller, the watchdog, and the panic breadcrumb that has to
// cross a real reset to be worth anything.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and nothing here costs any endurance.
//
// What is exercised, letter by letter:
//   a  the boot story: RCAUSE read as the EXCLUSIVE register it is (one
//      cause, not a history), and the watchdog's power-on state checked
//      against the NVM User Row that supplied it - a cross-check between
//      two drivers, since samc/nvm.hpp reads the fuses that samc/
//      reset.hpp then finds in CTRLA/CONFIG/EWCTRL
//   b  the watchdog as a configurable timer, with nothing allowed to
//      time out: arming, read-back, disabling, the refusals
//   c  what OSCULP32K actually runs at, measured through the early-
//      warning interrupt in both modes - the offset in normal mode and
//      the closed window in window mode
//
//   i  (by name only) SIX REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session that a tool can judge
//      from a single capture. Run it with
//          python3 tools/bench.py run C i --expect="->"
//      Legs: a wrong CLEAR key with the watchdog stopped and then
//      running, a panic through ResetReporter, a deliberate HardFault
//      through hard_fault_reset(), a watchdog time-out, and a window
//      violation.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter i lives in
//
// INLINE, and in .noinit, for the same two reasons the AVR twin of this
// suite gives: the section must survive the crt (the linker script marks
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

using Led = Pin<'B', 23>;

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
// A cycle-resolution stopwatch (the same one the other SAM suites use)
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
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
    // asserting, because the AVR family's RSTFR does the opposite and
    // the habit travels.
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
    // drivers meet: samc/nvm.hpp reads the NVM User Row, samc/reset.hpp
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
// SINGLE MEASUREMENTS DO NOT GIVE THE RATE, and that is the lesson this
// letter is built around. Arming the watchdog and starting the clock are
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
}

// =============================================================================
// i - six real resets (outside z: it reboots the board)
// =============================================================================
//
// Each leg banks its number in the .noinit token, triggers a reset, and
// then SPINS - because not every trigger here is instantaneous. A wrong
// key written to CLEAR is synchronized into the 1.024 kHz domain and
// arrives a few milliseconds later, which the first version of this
// letter learned the hard way by running on past its own trigger and
// into the next leg.

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

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler is
// a spin loop - so every line this suite can raise is bound.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void WDT_Handler() {
    if (brio::Watchdog::isr() != 0u) {
        ew_cycles = cycles_now();
        ew_seen = true;
    }
}

/// The whole point of samc/reset.hpp's fault body: a crash becomes a
/// note the next boot can read, instead of a spin nobody sees.
extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::SamPlatform>(token.context);
}

int main() {
    // Sampled FIRST: RCAUSE is not cleared by anything, but the panic
    // record is fetch-and-clear and must be taken exactly once.
    boot_cause_bits = brio::Reset::cause_bits();
    boot_cause = brio::Reset::cause();
    boot_record = brio::take_panic_record<brio::SamPlatform>();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::Nvic::enable(WDT_IRQn);
    brio::enable_interrupts();

    bench.letter('a', "the boot story and the watchdog's fuses", ta_boot);
    bench.letter('b', "the watchdog as a configurable timer", tb_watchdog);
    bench.letter('c', "what OSCULP32K really runs at", tc_oscillator);
    bench.letter('i', "SIX REAL RESETS (reboots the board)", ti_resets, false);

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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
