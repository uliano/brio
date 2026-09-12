// test_rp2040_platform - the reference bench suite for the RP2040's
// PLATFORM: the running half (the PRIMASK critical section, the idle
// hook, the SysTick timebase, delay_us, the reset controller, the
// atomic register aliases, the system timer that is this chip's
// microsecond ruler) and the failing half - rp2040/reset.hpp (the
// causes, the software reset, the panic breadcrumb across real
// resets) and rp2040/watchdog.hpp (the tick, the countdown with
// erratum E1's double decrement, the forced reset, the scratch
// registers).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the Debug Probe's UART bridge on GP0
// (TX) / GP1 (RX) = UART0, and every clock this suite measures is
// inside the chip.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp): 64 bits of
// microseconds on the watchdog tick from clk_ref, which is the crystal
// undivided - independent of clk_sys, so it judges SysTick, delay_us
// and the PLL's ratio; it shares the crystal with them, so the
// crystal's own accuracy is nobody's verdict here.
//
// What is exercised, letter by letter:
//   a  the boot story: the chip id, VTOR where the second stage left
//      it, core 0, the causes of this boot read twice, what the reset
//      controller holds released, the tick, no breadcrumb pending
//   b  the critical section: PRIMASK saved and restored, nesting, a
//      masked window seen by the ruler, and the idle hook returning on
//      the tick
//   c  the SysTick timebase: reload, ticks/millis/secs/now coherence,
//      200 ticks against the ruler (the PLL's ratio), and the VAL-delta
//      accumulation delay.hpp is built on
//   d  delay_us on the SysTick counter, bracketed by the ruler: at
//      least, never early, the cap, the no-counter refusal
//   e  the reset controller: a block held and released, RESET_DONE's
//      latency, cycle()
//   f  the atomic aliases (set, clear, xor, masked write) on a scratch
//      register, every one read back
//   g  the panic breadcrumb WITHOUT a reset: written, taken once, gone
//   h  WFI idle over 200 ticks: the awake fraction on the ruler
//   j  the system timer itself: the 64-bit read against the latched
//      one, monotonic, an alarm 2 ms ahead and its interrupt latency,
//      disarm, the raised flag, the force bit
//
//   i  (by name only) FIVE REAL RESETS. This letter reboots the board
//      once per leg and resumes from a token in the watchdog's scratch
//      registers (which survive every reset but RUN and the supply),
//      so it is NOT in `z`: `z` has to be one console session a tool
//      can judge from a single capture. Run it with
//          brio run <board> i --app test_rp2040_platform
//                  --expect="all five legs done" --timeout 120
//      (a marker ending in "> " would stop the capture at the first
//      tally arrow: bin/brio waits for the prompt after the marker)
//      Legs: Reset::core() (SYSRESETREQ: this core alone through the
//      bootrom), Reset::software() (the chip: the watchdog's trigger,
//      REASON.FORCE), a watchdog time-out at 100 ms, a panic through
//      ResetReporter, a HardFault through fault_reset(). Each leg
//      also reports whether a .noinit word survived the reset - a
//      fact this chip promises nowhere - and compares the causes word
//      with the one it banked: the word is a HISTORY (HAD_POR since the
//      power-on, REASON the last watchdog event) that a core reset
//      leaves untouched and a chip reboot marks FORCE.
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/panic.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/reset.hpp"
#include "rp2040/resets.hpp"
#include "rp2040/sysinfo.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "rp2040/watchdog.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P = brio::Rp2040Platform<>;

// A word in .noinit, to learn whether SRAM survives each kind of reset
// (the linker script keeps the section NOLOAD, the crt never touches
// it). Inline, for the reason the other strata's suites give: gcc
// gives an inline variable with a section attribute a COMDAT group
// like the platform's own panic_record_, and a plain one would clash.
[[gnu::section(".noinit")]] inline uint32_t noinit_canary;
inline constexpr uint32_t canary_value = 0xC3A5F00Du;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;   // the Pico's and the WeAct board's LED: a keystroke marker

TestBench<Serial> bench;

// What this boot was told, sampled once in main() before anything can
// disturb it.
uint32_t boot_causes = 0;
std::optional<PanicRecord> boot_record;
bool boot_canary_alive = false;

// ---- the token letter i lives in: the watchdog's SCRATCH0..3 --------------
//
// SCRATCH0 = magic (high half) | leg (low byte); SCRATCH1 = the tally;
// SCRATCH2 = the panic code and context a leg expects at the next boot;
// SCRATCH3 = the causes word standing when the leg started.
// These survive a watchdog reset and a software reset by the chapter's
// own promise (4.7.4), which is why the token is here and the .noinit
// word is only a witness.
inline constexpr uint32_t token_magic = 0x5A12'0000u;

uint8_t token_leg() {
    const uint32_t w = Scratch<0>::read();
    return (w & 0xFFFF'0000u) == token_magic ? static_cast<uint8_t>(w & 0xFFu) : 0u;
}
void token_set(uint8_t leg, uint8_t code = 0, uint8_t context = 0) {
    Scratch<1>::write((static_cast<uint32_t>(bench.passed()) << 16) | bench.failed());
    Scratch<2>::write((static_cast<uint32_t>(code) << 8) | context);
    Scratch<3>::write(boot_causes);
    Scratch<0>::write(token_magic | leg);
}
void token_clear() { Scratch<0>::write(0); }

uint32_t us_now() { return Timer::now_low(); }

/// Let the console's own transmitter fall silent: its interrupt would
/// otherwise wake every idle() below.
void console_drain() {
    const uint32_t t0 = us_now();
    while (!Serial::tx_idle() && us_now() - t0 < 200'000u) {
    }
}

void print_causes(uint32_t c) {
    if (c == 0u) {
        print(serial, "none (a software reset or a debugger's)");
        return;
    }
    if (c & ResetCause::power_on) print(serial, "POR ");
    if (c & ResetCause::run_pin) print(serial, "RUN ");
    if (c & ResetCause::rescue) print(serial, "PSM_RESTART ");
    if (c & ResetCause::watchdog_timer) print(serial, "WATCHDOG_TIMER ");
    if (c & ResetCause::watchdog_force) print(serial, "WATCHDOG_FORCE ");
}

void report_boot() {
    print(serial, "  causes: ");
    print_causes(boot_causes);
    print(serial, "; .noinit canary ", boot_canary_alive ? "SURVIVED" : "gone",
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
          " revision B", id.revision, " gitref ", hex(ChipId::gitref()), crlf);
    bench.verdict("SYSINFO names Raspberry Pi's RP2040",
                  id.manufacturer == ChipId::raspberry_pi && id.part == ChipId::rp2040);

    const uint32_t vtor = SCB->VTOR;
    print(serial, "  VTOR=", hex(vtor), " core ", P::core_id(), crlf);
    bench.verdict("VTOR is the vector table at flash offset 0x100, where the "
                  "second stage left it",
                  vtor == 0x10000100u);
    bench.verdict("this is core 0", P::core_id() == 0u);

    report_boot();
    const uint32_t c1 = Reset::causes();
    const uint32_t c2 = Reset::causes();
    bench.verdict("the causes are unchanged by reading them",
                  c1 == c2 && c1 == boot_causes);

    const uint32_t done = RESETS->RESET_DONE;
    print(serial, "  RESET_DONE=", hex(done), " (the blocks out of reset now)", crlf);
    bench.verdict("the timer, the IO and pad banks and UART0 are out of reset, "
                  "released by the drivers this suite started",
                  Resets::released(ResetBlock::timer | ResetBlock::io_bank0 |
                                   ResetBlock::pads_bank0 | ResetBlock::uart0));
    bench.verdict("the watchdog tick runs at the crystal's megahertz",
                  WatchdogTick::running() && WatchdogTick::cycles() == SysClock::xtal_hz / 1'000'000u);
    bench.verdict("no breadcrumb is pending on a clean start", !boot_record);
}

// =============================================================================
// b - the critical section and the idle hook
// =============================================================================
void tb_critical() {
    bench.verdict("interrupts are enabled when a letter runs", P::interrupts_enabled());

    bool inside = true, nested = true, after_inner = false, after_outer = false;
    {
        P::CriticalSection cs;
        inside = P::interrupts_enabled();
        {
            P::CriticalSection inner;
            nested = P::interrupts_enabled();
        }
        after_inner = P::interrupts_enabled();
    }
    after_outer = P::interrupts_enabled();
    bench.verdict("a critical section masks", !inside);
    bench.verdict("a nested one is still masked", !nested);
    bench.verdict("leaving the INNER scope does not unmask (PRIMASK is saved "
                  "and restored, not cleared)",
                  !after_inner);
    bench.verdict("leaving the outer scope unmasks", after_outer);

    // The tick keeps counting through a masked window - the interrupt is
    // pending, not lost - and the RULER sees the window's true length.
    const uint32_t t0 = Ticker::ticks();
    const uint32_t u0 = us_now();
    {
        P::CriticalSection cs;
        const uint32_t s = us_now();
        while (us_now() - s < 5000u) {
        }
    }
    const uint32_t window_us = us_now() - u0;
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  ", window_us, " us with interrupts masked advanced the tick by ",
          through_mask, " ms", crlf);
    bench.verdict("a masked window LOSES ticks (SysTick's interrupt is a pending "
                  "bit, not a counter - one tick is delivered, the rest coalesce)",
                  window_us >= 5000u && through_mask <= 5u);

    // idle(): WFI, then unmask. Any enabled interrupt wakes it, so the
    // console has to be silent first.
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    const uint32_t u1 = us_now();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        P::idle();
        ++calls;
    }
    const uint32_t idle_us = us_now() - u1;
    print(serial, "  with the console silent, ", calls, " idle() call(s) covered the ",
          idle_us, " us to the next tick", crlf);
    bench.verdict("idle() sleeps and the tick is what brings it back, inside one "
                  "tick period",
                  calls >= 1u && calls < 20u && idle_us <= 2000u);
    bench.verdict("and it comes back with interrupts enabled", P::interrupts_enabled());
}

// =============================================================================
// c - the SysTick timebase
// =============================================================================
void tc_ticker() {
    const uint32_t reload = SysTick->LOAD;
    print(serial, "  SysTick CTRL=", hex(SysTick->CTRL), " LOAD=", reload, " (",
          SysClock::hz / Ticker::ticks_per_second - 1u, " expected)", crlf);
    bench.verdict("the reload is Clock::hz / tps - 1",
                  reload == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on the processor clock with its interrupt armed",
                  (SysTick->CTRL & (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                                    SysTick_CTRL_ENABLE_Msk)) ==
                      (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                       SysTick_CTRL_ENABLE_Msk));

    const uint32_t ticks = Ticker::ticks();
    const uint32_t ms = Ticker::millis();
    const uint32_t secs = Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    print(serial, "  ticks=", ticks, " millis=", ms, " secs=", secs, " now=", stamp, crlf);
    bench.verdict("millis() is ticks() at 1000 Hz (no correction, no drift)",
                  ms >= ticks && ms - ticks <= 2u);
    bench.verdict("secs() is the whole-second part of the same counter",
                  secs == ticks / 1000u || secs == (ticks + 2u) / 1000u);
    bench.verdict("now() agrees with both counters",
                  stamp.seconds == secs || stamp.seconds == secs + 1u);

    // THE RATIO. Two hundred SysTick periods on clk_sys against the
    // ruler on clk_ref: both derive from the crystal, so the error is
    // the PLL's ratio and the reload's rounding, and a microsecond or
    // two of phase at each end.
    const uint32_t target = 200;
    uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    t0 = Ticker::ticks();
    const uint32_t u0 = us_now();
    while (Ticker::ticks() - t0 < target) {
    }
    const uint32_t span = us_now() - u0;
    const uint32_t expected_us = target * 1000u;
    const uint32_t err = span > expected_us ? span - expected_us : expected_us - span;
    print(serial, "  200 ticks = ", span, " us on the ruler, error ", err, " us (",
          err * 1000000ULL / expected_us, " ppm)", crlf);
    bench.verdict("200 ticks of SysTick are 200 ms of the microsecond ruler within "
                  "20 us: the PLL's ratio and the reload are exact",
                  err <= 20u);

    // The VAL-delta accumulation delay.hpp is built on, against the
    // interrupt count over 200 wraps.
    const uint32_t period = reload + 1u;
    uint32_t last = SysTick->VAL;
    uint32_t accumulated = 0;
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < target) {
        const uint32_t now = SysTick->VAL;
        accumulated += (last >= now) ? (last - now) : (last + period - now);
        last = now;
    }
    const uint32_t expected = target * period;
    const uint32_t cerr = accumulated > expected ? accumulated - expected
                                                 : expected - accumulated;
    print(serial, "  200 ticks = ", accumulated, " cycles by VAL deltas, ", expected,
          " expected, error ", cerr, " cycles", crlf);
    bench.verdict("the VAL-delta accumulation tracks the interrupt count over "
                  "200 wraps to under a period's worth of error",
                  cerr < period);
}

// =============================================================================
// d - delay_us (rp2040/delay.hpp) on the ruler
// =============================================================================
void td_delay() {
    uint32_t zero_max = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const uint32_t t0 = us_now();
        const uint32_t b = us_now() - t0;
        if (b > zero_max) zero_max = b;
    }
    print(serial, "  empty measurement bracket: up to ", zero_max, " us of its own", crlf);

    // THE COLD CALL, measured apart: the first delay_us of the program
    // runs its loop out of the flash through an XIP cache that has
    // never seen it, and pays the QSPI fetches - a cost of this chip's
    // memory system, not of the wait, and one a program pays once per
    // eviction. The series below runs warm.
    const uint32_t c0 = us_now();
    (void)delay_us(clock, 5u);
    const uint32_t cold = us_now() - c0;
    const uint32_t w0 = us_now();
    (void)delay_us(clock, 5u);
    const uint32_t warm = us_now() - w0;
    print(serial, "  first delay_us(5) of the program: ", cold, " us (the XIP cache filling); "
          "the second: ", warm, " us", crlf);
    bench.verdict("the cold first call costs the cache fill and the warm one does not: "
                  "the cold call under 40 us, the warm one within 10 us of its span",
                  cold >= 5u && cold <= 40u && warm >= 5u && warm <= 15u);

    static const uint32_t spans[] = {5, 30, 100, 500, 900};
    bool all_exact = true;
    for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        const uint32_t t0 = us_now();
        const bool ok = delay_us(clock, spans[i]);
        const uint32_t took = us_now() - t0;
        print(serial, "  delay_us(", spans[i], ") -> ", took, " us measured", crlf);
        if (!ok || took < spans[i] || took > spans[i] + zero_max + 10u) {
            all_exact = false;
        }
    }
    bench.verdict("delay_us serves 5..900 us AT LEAST and within the bracket's "
                  "zero plus 10 us, on the ruler",
                  all_exact);

    uint32_t min_took = 0xFFFFFFFFu;
    uint32_t max_took = 0;
    for (uint16_t k = 0; k < 200; ++k) {
        const uint32_t t0 = us_now();
        (void)delay_us(clock, 50u);
        const uint32_t took = us_now() - t0;
        if (took < min_took) min_took = took;
        if (took > max_took) max_took = took;
    }
    print(serial, "  200 x delay_us(50): min ", min_took, " max ", max_took, " us", crlf);
    bench.verdict("two hundred 50 us waits: NOT ONE EARLY, whatever the counter phase",
                  min_took >= 50u);

    const uint32_t t0 = Ticker::ticks();
    for (uint16_t k = 0; k < 1000; ++k) {
        (void)delay_us(clock, 100u);
    }
    const uint32_t bulk_ms = Ticker::ticks() - t0;
    print(serial, "  1000 x delay_us(100) = ", bulk_ms, " ms of kernel tick (100 due)", crlf);
    bench.verdict("a thousand 100 us waits are 100 ms of kernel time, never less, "
                  "and the per-call overhead costs under 10%",
                  bulk_ms >= 100u && bulk_ms <= 110u);

    uint32_t refusal_us = 0xFFFFFFFFu;
    bool refused = true;
    for (uint8_t k = 0; k < 4; ++k) {
        const uint32_t t0r = us_now();
        refused = refused && !delay_us(clock, 1000u);
        const uint32_t r = us_now() - t0r;
        if (r < refusal_us) refusal_us = r;
    }
    const bool served_999 = delay_us(clock, 999u);
    print(serial, "  delay_us(1000) refused=", refused, " in ", refusal_us,
          " us (min of 4); delay_us(999) served=", served_999, crlf);
    bench.verdict("the CAP is real: a whole tick is REFUSED spending nothing - "
                  "TimeEvent territory - while 999 us is served",
                  refused && refusal_us <= zero_max + 2u && served_999);

    SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_ENABLE_Msk;
    uint32_t stopped_us = 0xFFFFFFFFu;
    bool refused_stopped = true;
    for (uint8_t k = 0; k < 4; ++k) {
        const uint32_t t1 = us_now();
        refused_stopped = refused_stopped && !delay_us(clock, 100u);
        const uint32_t r = us_now() - t1;
        if (r < stopped_us) stopped_us = r;
    }
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_ENABLE_Msk;
    print(serial, "  SysTick stopped: refused=", refused_stopped, " in ", stopped_us,
          " us (min of 4)", crlf);
    bench.verdict("with SysTick not running delay_us answers false and spends "
                  "nothing (a program with no Ticker has no clock to count on)",
                  refused_stopped && stopped_us <= zero_max + 2u);
}

// =============================================================================
// e - the reset controller
// =============================================================================
void te_resets() {
    // The PWM block: nothing in this suite uses it, and it is held in
    // reset at power-up (2.14.1) - unless a previous life released it.
    const bool before = Resets::released(ResetBlock::pwm);
    print(serial, "  PWM at entry: ", before ? "released" : "held", crlf);

    Resets::hold(ResetBlock::pwm);
    bench.verdict("hold(): the block's RESET bit is set and its RESET_DONE clears",
                  Resets::held(ResetBlock::pwm) && !Resets::released(ResetBlock::pwm));

    const uint32_t t0 = us_now();
    const bool released = Resets::release(ResetBlock::pwm);
    const uint32_t took = us_now() - t0;
    print(serial, "  release(): RESET_DONE within ", took, " us", crlf);
    bench.verdict("release() reports the block ready, and quickly", released && took <= 10u);
    bench.verdict("RESET_DONE stands and RESET is clear",
                  Resets::released(ResetBlock::pwm) && !Resets::held(ResetBlock::pwm));

    bench.verdict("cycle() takes a released block through reset and back",
                  Resets::cycle(ResetBlock::pwm) && Resets::released(ResetBlock::pwm));
    bench.verdict("a block that was never touched stays as it was: the ADC",
                  !Resets::released(ResetBlock::adc) || Resets::released(ResetBlock::adc));
    // Left released: a chapter to come will find it ready.
}

// =============================================================================
// f - the atomic aliases on a scratch register
// =============================================================================
void tf_aliases() {
    volatile uint32_t& s = Scratch<3>::reg();
    const uint32_t keep = s;
    s = 0xF0F0F0F0u;
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
    bench.verdict("the SET alias ORs the written bits in", after_set == 0xF0F0F0FFu);
    bench.verdict("the CLR alias clears them", after_clear == 0x00F0F0FFu);
    bench.verdict("the XOR alias flips them", after_xor == 0x00F0F000u);
    bench.verdict("a masked write changes only its mask", after_masked == 0x0234F000u);
}

// =============================================================================
// g - the panic breadcrumb, without a reset
// =============================================================================
void tg_breadcrumb() {
    print(serial, "  panic record at ", hex(reinterpret_cast<uintptr_t>(&P::panic_record())),
          ", the .noinit canary at ", hex(reinterpret_cast<uintptr_t>(&noinit_canary)), crlf);
    bench.verdict("the .noinit section sits in the main SRAM above .bss",
                  reinterpret_cast<uintptr_t>(&noinit_canary) >= 0x20000000u &&
                      reinterpret_cast<uintptr_t>(&noinit_canary) < 0x20040000u);
    bench.verdict("nothing is pending", !take_panic_record<P>());
    P::panic_record() = PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::assert_failed), 0x42};
    const auto taken = take_panic_record<P>();
    bench.verdict("a written record is taken with its code and context",
                  taken && taken->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                      taken->context == 0x42);
    bench.verdict("and taken only once", !take_panic_record<P>());
}

// =============================================================================
// h - WFI idle over 200 ticks
// =============================================================================
void th_idle() {
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    const uint32_t u0 = us_now();
    uint32_t slept = 0;
    uint32_t calls = 0;
    while (Ticker::ticks() - t0 < 200u) {
        const uint32_t a = us_now();
        P::idle();
        slept += us_now() - a;
        ++calls;
    }
    const uint32_t span = us_now() - u0;
    const uint32_t awake = span - slept;
    print(serial, "  200 ticks: ", calls, " idle() calls, ", span, " us span, ", awake,
          " us awake (", awake * 1000u / span, " permille)", crlf);
    bench.verdict("one idle() per tick, give or take the console's last byte",
                  calls >= 195u && calls <= 210u);
    bench.verdict("the loop is awake under 2 % of the time with nothing to do",
                  awake * 50u < span);
    bench.verdict("200 ticks are 200 ms on the ruler within a millisecond",
                  span >= 199'000u && span <= 201'500u);
}

// =============================================================================
// j - the system timer
// =============================================================================
volatile uint32_t alarm_entry_us = 0;
volatile uint8_t alarm_fires = 0;

void tj_timer() {
    const uint64_t a = Timer::now();
    const uint64_t b = Timer::now_latched();
    const uint64_t c = Timer::now();
    print(serial, "  now() ", static_cast<uint32_t>(a), " latched ", static_cast<uint32_t>(b),
          " now() ", static_cast<uint32_t>(c), " us; high half ", static_cast<uint32_t>(a >> 32),
          crlf);
    bench.verdict("the raw-pair read and the latched read agree within 50 us",
                  b >= a && b - a <= 50u && c >= b && c - b <= 50u);
    bool monotonic = true;
    uint64_t last = Timer::now();
    for (uint16_t k = 0; k < 2000; ++k) {
        const uint64_t n = Timer::now();
        if (n < last) monotonic = false;
        last = n;
    }
    bench.verdict("2000 raw-pair reads never go backwards", monotonic);

    // An alarm two milliseconds ahead: the handler stamps its entry.
    alarm_fires = 0;
    Timer::clear(0);
    Timer::interrupt(0, true);
    Nvic::enable(Timer::irq(0));
    const uint32_t target = us_now() + 2000u;
    Timer::alarm(0, target);
    const bool was_armed = Timer::armed(0);
    const uint32_t wait0 = us_now();
    while (alarm_fires == 0u && us_now() - wait0 < 10'000u) {
    }
    const uint32_t latency = alarm_entry_us - target;
    print(serial, "  alarm 0 at +2000 us: fired ", alarm_fires, " time(s), handler entry ",
          latency, " us after the target", crlf);
    bench.verdict("writing ALARM0 arms it", was_armed);
    bench.verdict("it fires once, and the handler enters within 20 us of the match",
                  alarm_fires == 1u && latency <= 20u);
    bench.verdict("ARMED clears when it fires", !Timer::armed(0));
    bench.verdict("the handler's write-1 cleared the raised flag", !Timer::raised(0));

    // A disarmed alarm does not fire.
    Timer::alarm_in(0, 2000u);
    Timer::disarm(0);
    const uint32_t w1 = us_now();
    while (us_now() - w1 < 4000u) {
    }
    bench.verdict("an alarm disarmed before its time never fires", alarm_fires == 1u);

    // INTF forces the line without the counter's say.
    Nvic::disable(Timer::irq(0));
    Timer::force(0, true);
    const bool forced = Timer::pending(0);
    Timer::force(0, false);
    bench.verdict("INTF forces the masked status and clearing it takes it back",
                  forced && !Timer::pending(0));
    Timer::interrupt(0, false);
    Nvic::clear_pending(Timer::irq(0));
}

// =============================================================================
// i - five real resets (by name only)
// =============================================================================
void bank(uint8_t leg, uint8_t code = 0, uint8_t context = 0) {
    noinit_canary = canary_value;
    token_set(leg, code, context);
}

[[noreturn]] void await_reset(const char* what) {
    console_drain();
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 3'000'000u) {
    }
    print(serial, "  ...3 s and still running ", what, crlf);
    for (;;) {
    }
}

[[noreturn]] void leg_software() {
    bank(1);
    print(serial, "  leg 1: Reset::core() - SYSRESETREQ, this core alone ...", crlf);
    console_drain();
    Reset::core();
}
[[noreturn]] void leg_force() {
    bank(2);
    print(serial, "  leg 2: Reset::software() - the chip through the watchdog's trigger ...", crlf);
    console_drain();
    Watchdog::force_reset();
}
[[noreturn]] void leg_timeout() {
    bank(3);
    print(serial, "  leg 3: the watchdog started at 100 ms and never kicked ...", crlf);
    console_drain();
    Watchdog::start(100'000u);
    await_reset("(the watchdog did not fire)");
}
[[noreturn]] void leg_panic() {
    bank(4, static_cast<uint8_t>(PanicCode::queue_overflow), 0x33);
    print(serial, "  leg 4: panic<P, ResetReporter>(queue_overflow, 0x33) ...", crlf);
    console_drain();
    panic<P, ResetReporter>(PanicCode::queue_overflow, 0x33);
}
[[noreturn]] void leg_fault() {
    bank(5, static_cast<uint8_t>(PanicCode::kernel_fault), 0x77);
    print(serial, "  leg 5: UDF, HardFault, fault_reset() ...", crlf);
    console_drain();
    __asm__ volatile("udf #0");
    await_reset("(the undefined instruction did not fault)");
}

void ti_resets() {
    report_boot();
    bench.verdict("no breadcrumb is pending on a clean start", !boot_record);
    leg_software();
}

/// Everything after a reset: called from main() instead of the banner,
/// it either starts the next leg (never returning) or closes the letter.
void ti_resume() {
    const uint8_t leg = token_leg();
    const uint32_t tally = Scratch<1>::read();
    const uint32_t expect = Scratch<2>::read();
    const uint32_t before = Scratch<3>::read();
    constexpr uint32_t chip_level = ResetCause::power_on | ResetCause::run_pin | ResetCause::rescue;
    bench.resume_tally(static_cast<uint16_t>(tally >> 16), static_cast<uint16_t>(tally & 0xFFFFu));
    print(serial, crlf, "i (continued after reset ", leg, " of 5)", crlf);
    report_boot();
    bench.verdict("the scratch token survived the reset", leg != 0u);

    // THE CAUSES ARE A HISTORY, NOT A CAUSE: CHIP_RESET's HAD_* bits
    // stand since the last chip-level event and WATCHDOG.REASON holds
    // the LAST watchdog event, and a software reset neither sets nor
    // clears any of them - so every leg compares the word with the one
    // it banked, and only the watchdog legs expect a change.
    if (leg == 1) {
        bench.verdict("a core reset leaves the causes word EXACTLY as it was: SYSRESETREQ "
                      "sets no flag and clears none",
                      boot_causes == before);
        bench.verdict("the .noinit word survived a core reset", boot_canary_alive);
        leg_force();
    }
    if (leg == 2) {
        bench.verdict("Watchdog::force_reset() names itself: REASON.FORCE",
                      (boot_causes & ResetCause::watchdog_force) != 0u);
        bench.verdict("REASON holds the LAST watchdog event: TIMER, if it stood, is gone",
                      (boot_causes & ResetCause::watchdog_timer) == 0u);
        bench.verdict("the chip-level flags stand as they were (HAD_POR is the "
                      "power-on's, for the life of the supply)",
                      (boot_causes & chip_level) == (before & chip_level));
        bench.verdict("the .noinit word survived a watchdog reset", boot_canary_alive);
        leg_timeout();
    }
    if (leg == 3) {
        bench.verdict("a watchdog time-out names itself: REASON.TIMER, FORCE gone",
                      (boot_causes & ResetCause::watchdog_timer) != 0u &&
                          (boot_causes & ResetCause::watchdog_force) == 0u);
        bench.verdict("the .noinit word survived it too", boot_canary_alive);
        leg_panic();
    }
    if (leg == 4) {
        bench.verdict("the breadcrumb of a panic through ResetReporter is read at the "
                      "next boot with its code and context",
                      boot_record && boot_record->code == (expect >> 8) &&
                          boot_record->context == (expect & 0xFFu));
        bench.verdict("its reset was the chip's reboot (Reset::software() = the watchdog's "
                      "trigger): REASON.FORCE, the chip-level flags as before",
                      (boot_causes & ResetCause::watchdog_force) != 0u &&
                          (boot_causes & chip_level) == (before & chip_level));
        leg_fault();
    }
    if (leg == 5) {
        bench.verdict("a HardFault through fault_reset() leaves a kernel_fault breadcrumb "
                      "with the context it was given",
                      boot_record && boot_record->code == (expect >> 8) &&
                          boot_record->context == (expect & 0xFFu));
        bench.verdict("and its reset was the chip's reboot too: REASON.FORCE",
                      (boot_causes & ResetCause::watchdog_force) != 0u &&
                          (boot_causes & chip_level) == (before & chip_level));
        token_clear();
        boot_record.reset();  // consumed: a second 'i' in this boot starts clean
        print(serial, "  all five legs done", crlf);
        bench.end_letter();
    }
}

void banner() {
    print(serial, crlf, "test_rp2040_platform - the RP2040 platform, timer, watchdog and "
          "reset (datasheet 2.3, 2.14, 4.6, 4.7), clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_timer_0() {
    alarm_entry_us = brio::Timer::now_low();
    brio::Timer::clear(0);
    alarm_fires = alarm_fires + 1;
}
extern "C" void isr_hardfault() { brio::fault_reset<P>(0x77); }

int main() {
    boot_causes = brio::Reset::causes();
    boot_record = brio::take_panic_record<P>();
    boot_canary_alive = noinit_canary == canary_value;
    noinit_canary = 0;

    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the boot story", ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the SysTick timebase against the ruler", tc_ticker);
    bench.letter('d', "delay_us on the SysTick counter", td_delay);
    bench.letter('e', "the reset controller", te_resets);
    bench.letter('f', "the atomic register aliases", tf_aliases);
    bench.letter('g', "the panic breadcrumb, no reset", tg_breadcrumb);
    bench.letter('h', "WFI idle over 200 ticks", th_idle);
    bench.letter('j', "the system timer and an alarm", tj_timer);
    bench.letter('i', "FIVE REAL RESETS (reboots the board)", ti_resets, false);

    if (serial_ok && token_leg() != 0u) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", crlf);
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
