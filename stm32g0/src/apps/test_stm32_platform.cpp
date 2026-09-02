// test_stm32_platform - the reference bench suite for the STM32G0's
// PLATFORM: the parts of it that were already there (the PRIMASK
// critical section, the idle hook, the SysTick timebase) and the two
// halves this campaign built - stm32g0/reset.hpp (the RCC's reset
// flags, the independent watchdog, the system window watchdog, the
// panic breadcrumb across a real reset) and stm32g0/delay.hpp (the
// microsecond busy-wait).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the Nucleo's own ST-LINK virtual COM
// port (USART2, PA2/PA3, AF1) and every clock this suite measures is
// inside the chip.
//
// What is exercised, letter by letter:
//   a  the boot story: the reset flags read as the ACCUMULATING history
//      they are, PINRSTF seen for what it is (a catch-all), and the two
//      watchdogs' power-on state
//   b  the critical section: PRIMASK saved and restored, nesting, and
//      the idle hook returning on the tick
//   c  the SysTick timebase: ticks/millis/secs/now coherence, and the
//      VAL-delta accumulation that delay.hpp is built on checked
//      against the interrupt count
//   d  delay_us: at least, never early, the cap, the no-Ticker refusal
//   e  the IWDG WITHOUT EVER STARTING IT - the keyed registers, the
//      three update bits crossing into the LSI domain, and LSI weighed
//      by the time those updates take
//   f  the WWDG WITHOUT EVER ACTIVATING IT - the free-running counter,
//      EWIF at 0x40 and the early-wakeup interrupt, all with WDGA clear
//
//   i  (by name only) SIX REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session a tool can judge from a
//      single capture. Run it with
//          python3 tools/bench.py run E i --app test_stm32_platform
//                  --expect="->" --timeout 120
//      Legs: a software reset, an IWDG time-out (which measures the
//      real one and with it LSI), a WWDG window violation, a panic
//      through ResetReporter, a deliberate HardFault through
//      hard_fault_reset(), and a WWDG time-out.
//
// NOTHING IN THIS SUITE FEEDS A WATCHDOG, and that is a measurement
// rather than an oversight. Starting either watchdog is one-way in
// software (RM0444 28.3.1, 29.3.2), so no letter of `z` starts one at
// all - the whole of chapter 29's timing is reachable with WDGA clear,
// and chapter 28's keyed registers with the start key never written.
// Letter i does start both, on legs that are about to be reset by them,
// and its second leg is where "the IWDG cannot be stopped EXCEPT UPON A
// RESET" is proven literal: the boot after an IWDG reset sits for a
// second and a half with nobody refreshing anything and lives.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter i lives in
//
// INLINE, and in .noinit, for the two reasons the other two strata's
// suites give: the section must survive the crt (the linker script
// marks .noinit NOLOAD and startup neither loads nor zeroes it), and
// gcc gives an inline variable with a section attribute a COMDAT group
// where a plain one gets none - the platform's own panic_record_ is a
// static inline member, so this must be inline too or the link fails
// with a section type conflict.
//
// Its magic word is not decoration: RM0444 promises nothing about SRAM
// across a reset, so every read of this object is guarded.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5A12;
inline constexpr uint16_t token_canary = 0xC3A5;

struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t leg;        ///< which reset we are waiting for (0 = none pending)
    uint8_t code;       ///< the PanicCode written before the reset
    uint8_t context;    ///< its context byte
    uint16_t pass;      ///< letter i's tally so far
    uint16_t fail;
    uint32_t flags_before;   ///< the flags standing when the leg started
    uint32_t elapsed_ms;     ///< the IWDG leg's own stopwatch
    uint8_t ewi_seen;        ///< leg 6: did the early warning fire?
    uint8_t ewi_counter;     ///< and what the counter read inside its handler
    uint8_t armed_ok;        ///< leg 2: what Iwdg::arm() answered
};
[[gnu::section(".noinit")]] inline Token token;


namespace {

using namespace brio;

// The console pads: USART2_TX on PA2, USART2_RX on PA3, both AF1
// (DS13560 table 13), which is the ST-LINK's virtual COM port.
constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

using Led = Pin<'A', 5>;   // LD4

TestBench<Serial> bench;

// What this boot was told, sampled once in main() before anything can
// disturb it.
uint32_t boot_flags = 0;
std::optional<PanicRecord> boot_record;
bool boot_lsi_forced = false;   ///< LSIRDY standing with LSION clear


/// Set by the WWDG handler; read by letter f.
volatile uint32_t ewi_cycles = 0;
volatile uint16_t ewi_count = 0;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch, the samc suites' own
//
// ticks x period + the phase SysTick has already counted down. The two
// reads are retried until they belong to the same tick, which is what
// makes the sum monotone across the handler.
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

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;
uint32_t cycles_to_us(uint32_t cycles) { return cycles / cycles_per_us; }

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

void print_flags(uint32_t f) {
    print(serial, hex(f));
    if (f & ResetFlag::low_power) print(serial, " LPWR");
    if (f & ResetFlag::window_watchdog) print(serial, " WWDG");
    if (f & ResetFlag::independent_watchdog) print(serial, " IWDG");
    if (f & ResetFlag::software) print(serial, " SFT");
    if (f & ResetFlag::power) print(serial, " PWR");
    if (f & ResetFlag::pin) print(serial, " PIN");
    if (f & ResetFlag::option_loader) print(serial, " OBL");
    if ((f & ResetFlag::all) == 0u) print(serial, " (none)");
}

void report_boot() {
    print(serial, "  boot: RCC_CSR flags=");
    print_flags(boot_flags);
    print(serial, boot_record ? "; a panic record was pending" : "; no panic record",
          crlf);
}

// =============================================================================
// a - the boot story
// =============================================================================
void ta_boot() {
    report_boot();

    // A reset always leaves a trace: whatever brought the program here,
    // at least one of the seven bits stands. (An ST-LINK "reset run"
    // ends in SYSRESETREQ, so a freshly flashed board shows SFT too.)
    bench.verdict("this boot names at least one reset source",
                  (boot_flags & ResetFlag::all) != 0u);

    // PINRSTF IS A CATCH-ALL, 5.4.24: it is set "when a reset from the
    // PF2-NRST pin occurs or when a system reset is triggered by any
    // other source". So it is present on every boot of this board, and
    // it means "the pin" only when it stands ALONE.
    bench.verdict("PINRSTF stands (it is a catch-all, not just the pin)",
                  (boot_flags & ResetFlag::pin) != 0u);

    // READING IS NOT CLEARING. Nothing but RMVF (or a power reset) takes
    // these bits down - the AVR's RSTFR habit, not the SAM's exclusive
    // RCAUSE - so two reads in a row give the same answer, and so does
    // the sample main() took at boot. THIS LETTER NEVER WRITES RMVF: it
    // would be a one-way loss of the boot's own evidence, and letter i
    // proves the clearing on real resets (legs 2 and 3) where the flags
    // are made again a moment later.
    const uint32_t r1 = Reset::flags();
    const uint32_t r2 = Reset::flags();
    bench.verdict("the flags are unchanged by reading them", r1 == r2);
    bench.verdict("and they still hold what main() sampled at boot",
                  r1 == boot_flags);
    bench.verdict("nothing left RMVF standing", (RCC->CSR & RCC_CSR_RMVF) == 0u);

    // The WWDG's bus clock is CLEAR at reset (5.2.17) unless the
    // WWDG_SW option byte selected a HARDWARE window watchdog, in which
    // case the RCC sets the bit itself - so this is also a reading of
    // that option byte, from the one side this stratum is allowed to
    // read it from.
    print(serial, "  WWDG : bus clock ", Wwdg::bus_clock() ? "ON" : "off",
          " (a hardware watchdog would have set it), WDGA=", Wwdg::enabled(),
          crlf);
    bench.verdict("the WWDG is not a hardware watchdog on this board (one "
                  "would already be feeding-or-dying)",
                  !Wwdg::enabled());

    // THE LSI WITNESS, AND WHY IT IS NOT ONE. 5.4.24 says LSIRDY may
    // stand with LSION clear when the IWDG, the RTC or the CSS on LSE
    // asks for the oscillator - and on this board the RTC does: the RTC
    // domain is not reset by a system reset, and RCC_BDCR comes up with
    // RTCEN set and RTCSEL = LSI. So the running-IWDG question is
    // answered from a .noinit mark instead, and this line prints both so
    // the reader can see the difference.
    print(serial, "  LSI  : LSION=", Rcc::lsi_enabled(), " LSIRDY=",
          Rcc::lsi_ready(), " RCC_BDCR=", hex(RCC->BDCR),
          " (RTCEN+RTCSEL is what forces LSI here)", crlf);
    bench.verdict("LSIRDY stands although LSION is clear, and it is the RTC "
                  "asking: RCC_BDCR survives a system reset with RTCEN set "
                  "and RTCSEL = LSI, so this bit is no watchdog witness",
                  boot_lsi_forced ==
                      ((RCC->BDCR & RCC_BDCR_RTCEN) != 0u &&
                       ((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >> RCC_BDCR_RTCSEL_Pos) == 2u &&
                       !Rcc::lsi_enabled()));
}

// =============================================================================
// b - the critical section and the idle hook
// =============================================================================
void tb_critical() {
    bench.verdict("interrupts are enabled when a letter runs",
                  Stm32Platform::interrupts_enabled());

    bool inside = true, nested = true, after_inner = false, after_outer = false;
    {
        Stm32Platform::CriticalSection cs;
        inside = Stm32Platform::interrupts_enabled();
        {
            Stm32Platform::CriticalSection inner;
            nested = Stm32Platform::interrupts_enabled();
        }
        // THE NESTING TEST, and it is the whole reason the guard saves
        // PRIMASK instead of just clearing it: the inner scope's exit
        // must NOT unmask, because the outer scope is still holding.
        after_inner = Stm32Platform::interrupts_enabled();
    }
    after_outer = Stm32Platform::interrupts_enabled();

    bench.verdict("a critical section masks", !inside);
    bench.verdict("a nested one is still masked", !nested);
    bench.verdict("leaving the INNER scope does not unmask (PRIMASK is "
                  "saved and restored, not cleared)",
                  !after_inner);
    bench.verdict("leaving the outer scope unmasks", after_outer);

    // The tick keeps counting through a masked window - the interrupt is
    // pending, not lost - which is what makes idle() safe below.
    const uint32_t t0 = Ticker::ticks();
    {
        Stm32Platform::CriticalSection cs;
        const uint32_t spin0 = cycles_now();
        while (cycles_now() - spin0 < SysClock::hz / 200u) {   // 5 ms masked
        }
    }
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  5 ms with interrupts masked advanced the tick by ",
          through_mask, " ms", crlf);
    bench.verdict("a masked window LOSES ticks (SysTick's interrupt is a "
                  "pending bit, not a counter - one tick is delivered, the "
                  "rest are coalesced)",
                  through_mask <= 5u);

    // idle(): WFI, then unmask. ANY enabled interrupt wakes it, so the
    // console has to be silent first or the USART's own transmit
    // interrupt returns it in microseconds - which is what the first
    // version of this letter measured (85 us, zero ticks) and reported
    // as a failure of the hook rather than of the test.
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    const uint32_t c1 = cycles_now();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        Stm32Platform::idle();
        ++calls;
    }
    const uint32_t idle_us = cycles_to_us(cycles_now() - c1);
    print(serial, "  with the console silent, ", calls,
          " idle() call(s) covered the ", idle_us, " us to the next tick",
          crlf);
    bench.verdict("idle() sleeps and the tick is what brings it back, inside "
                  "one tick period",
                  calls >= 1u && calls < 20u && idle_us <= 2000u);
    bench.verdict("and it comes back with interrupts enabled",
                  Stm32Platform::interrupts_enabled());
}

// =============================================================================
// c - the SysTick timebase
// =============================================================================
void tc_ticker() {
    // The reload the ticker programmed, against the rate the clock type
    // claims: this is the one number every measurement below rests on.
    const uint32_t reload = SysTick->LOAD;
    print(serial, "  SysTick CTRL=", hex(SysTick->CTRL), " LOAD=", reload,
          " (", SysClock::hz / Ticker::ticks_per_second - 1u, " expected)", crlf);
    bench.verdict("the reload is Clock::hz / tps - 1",
                  reload == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on the processor clock with its "
                  "interrupt armed",
                  (SysTick->CTRL & (SysTick_CTRL_CLKSOURCE_Msk |
                                    SysTick_CTRL_TICKINT_Msk |
                                    SysTick_CTRL_ENABLE_Msk)) ==
                      (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                       SysTick_CTRL_ENABLE_Msk));

    // millis() is exact at 1000 Hz: one tick, one millisecond.
    const uint32_t ticks = Ticker::ticks();
    const uint32_t ms = Ticker::millis();
    const uint32_t secs = Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    print(serial, "  ticks=", ticks, " millis=", ms, " secs=", secs,
          " now=", stamp, crlf);
    bench.verdict("millis() is ticks() at 1000 Hz (no correction, no drift)",
                  ms >= ticks && ms - ticks <= 2u);
    bench.verdict("secs() is the whole-second part of the same counter",
                  secs == ticks / 1000u || secs == (ticks + 2u) / 1000u);
    bench.verdict("now() agrees with both counters",
                  stamp.seconds == secs || stamp.seconds == secs + 1u);

    // THE MEASUREMENT THIS LETTER EXISTS FOR. delay.hpp counts time by
    // accumulating VAL DELTAS with the reload folded in across a wrap;
    // the ticker counts INTERRUPTS. The two are independent readings of
    // the same counter, so summing the deltas over a known number of
    // ticks is what proves the wrap arithmetic - an error there is one
    // whole period, not a rounding.
    const uint32_t target = 200;
    const uint32_t period = reload + 1u;
    uint32_t last = SysTick->VAL;
    uint32_t accumulated = 0;
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < target) {
        const uint32_t now = SysTick->VAL;
        accumulated += (last >= now) ? (last - now) : (last + period - now);
        last = now;
    }
    const uint32_t expected = target * period;
    const uint32_t err = accumulated > expected ? accumulated - expected
                                                : expected - accumulated;
    print(serial, "  200 ticks = ", accumulated, " cycles by VAL deltas, ",
          expected, " expected, error ", err, " cycles (",
          err * 1000000ULL / expected, " ppm)", crlf);
    bench.verdict("the VAL-delta accumulation delay.hpp is built on tracks "
                  "the interrupt count over 200 wraps to under a period's "
                  "worth of error",
                  err < period);
}

// =============================================================================
// d - delay_us (stm32g0/delay.hpp)
// =============================================================================
void td_delay() {
    // THE BRACKET'S OWN ZERO, measured first: two back-to-back
    // cycles_now() readings are not free, and charging their cost to
    // the delay is the mistake the samc twin of this letter paid for.
    uint32_t zero_min = 0xFFFFFFFFu;
    uint32_t zero_max = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        const uint32_t t0 = cycles_now();
        const uint32_t b = cycles_to_us(cycles_now() - t0);
        if (b < zero_min) zero_min = b;
        if (b > zero_max) zero_max = b;
    }
    print(serial, "  empty measurement bracket: ", zero_min, "..", zero_max,
          " us of its own", crlf);

    static const uint32_t spans[] = {5, 30, 100, 500, 900};
    bool all_exact = true;
    for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        const uint32_t t0 = cycles_now();
        const bool ok = delay_us(clock, spans[i]);
        const uint32_t took = cycles_to_us(cycles_now() - t0);
        print(serial, "  delay_us(", spans[i], ") -> ", took, " us measured", crlf);
        if (!ok || took < spans[i] || took > spans[i] + zero_max + 10u) {
            all_exact = false;
        }
    }
    bench.verdict("delay_us serves 5..900 us AT LEAST and within the "
                  "bracket's zero plus 10 us",
                  all_exact);

    // Never early, statistically: two hundred waits landing at whatever
    // SysTick phase the loop reaches them in, so the wrap path is walked
    // many times over.
    uint32_t min_took = 0xFFFFFFFFu;
    uint32_t max_took = 0;
    for (uint16_t k = 0; k < 200; ++k) {
        const uint32_t t0 = cycles_now();
        (void)delay_us(clock, 50u);
        const uint32_t took = cycles_to_us(cycles_now() - t0);
        if (took < min_took) min_took = took;
        if (took > max_took) max_took = took;
    }
    print(serial, "  200 x delay_us(50): min ", min_took, " max ", max_took,
          " us", crlf);
    bench.verdict("two hundred 50 us waits: NOT ONE EARLY, whatever the "
                  "counter phase",
                  min_took >= 50u);

    // A THOUSAND WAITS AGAINST A DIFFERENT COUNTER. The kernel tick is
    // the interrupt's count, not VAL's, so this is the accuracy check
    // that does not share its mechanism with the thing under test.
    const uint32_t t0 = Ticker::ticks();
    for (uint16_t k = 0; k < 1000; ++k) {
        (void)delay_us(clock, 100u);
    }
    const uint32_t bulk_ms = Ticker::ticks() - t0;
    print(serial, "  1000 x delay_us(100) = ", bulk_ms,
          " ms of kernel tick (100 due)", crlf);
    bench.verdict("a thousand 100 us waits are 100 ms of kernel time, never "
                  "less, and the per-call overhead costs under 10%",
                  bulk_ms >= 100u && bulk_ms <= 110u);

    // The cap: one tick period or more is refused, and refusing costs
    // nothing measurable. MIN OVER FOUR against the bracket's own MIN -
    // a tick interrupt landing inside any single bracket inflates it,
    // and a refusal is side-effect free, so repeating it is free.
    uint32_t refusal_us = 0xFFFFFFFFu;
    bool refused = true;
    for (uint8_t k = 0; k < 4; ++k) {
        const uint32_t t0r = cycles_now();
        refused = refused && !delay_us(clock, 1000u);
        const uint32_t r = cycles_to_us(cycles_now() - t0r);
        if (r < refusal_us) refusal_us = r;
    }
    const bool served_999 = delay_us(clock, 999u);
    print(serial, "  delay_us(1000) refused=", refused, " in ", refusal_us,
          " us (min of 4); delay_us(999) served=", served_999, crlf);
    bench.verdict("the CAP is real: a whole tick is REFUSED spending nothing "
                  "beyond the bracket's own zero - TimeEvent territory - "
                  "while 999 us is served",
                  refused && refusal_us <= zero_min + 3u && served_999);

    // No Ticker, no time. The counter holds its VAL across the pause, so
    // the tick slips by only the microseconds of this leg.
    SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_ENABLE_Msk;
    uint32_t stopped_us = 0xFFFFFFFFu;
    bool refused_stopped = true;
    for (uint8_t k = 0; k < 4; ++k) {
        const uint32_t t1 = SysTick->VAL;
        refused_stopped = refused_stopped && !delay_us(clock, 100u);
        const uint32_t t2 = SysTick->VAL;
        const uint32_t r = cycles_to_us(t1 >= t2 ? t1 - t2 : 0u);
        if (r < stopped_us) stopped_us = r;
    }
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_ENABLE_Msk;
    print(serial, "  SysTick stopped: refused=", refused_stopped, " in ",
          stopped_us, " us (min of 4)", crlf);
    bench.verdict("with SysTick not running delay_us answers false and "
                  "spends nothing (a program with no Ticker has no clock to "
                  "count on)",
                  refused_stopped && stopped_us <= 3u);
}

// =============================================================================
// e - the IWDG's keyed registers, and the clock they wait for
// =============================================================================
//
// 28.3.4's write protection has nothing to do with 28.3.1's start key,
// so PR, RLR and WINR can be written by a program that has no intention
// of arming anything - and this letter does exactly that, which is what
// keeps a one-way switch out of `z`.
//
// WHAT IT FOUND, and no chapter says it: with the watchdog STOPPED the
// writes are accepted and each raises its own update bit, and THE
// UPDATE NEVER COMPLETES - the bit stands for ever and the register
// keeps reporting its old value. 5.2.14 is where the reason hides: "if
// the IWDG is started ... the LSI oscillator is forced ON ... after the
// LSI oscillator temporization, THE CLOCK IS PROVIDED TO THE IWDG". So
// LSION buys nothing: the logic that performs the update has no clock
// until the start key is written. That is why every configuration
// sequence in 28.3.2 begins with 0xCCCC, and why this driver's arm()
// starts before it configures - the completing half is proven in letter
// i, whose second leg arms a NON-DEFAULT setting and is reset by it at
// exactly the time that setting predicts.
//
// THE UPDATE BITS ARE SINGLE-USE PER BOOT here, since nothing can clear
// them while the watchdog is stopped: this letter therefore writes RLR
// and nothing else, and uses the still-clear PVU as the witness for the
// key discipline, so that a second run in the same power cycle judges
// exactly the same things.
void te_iwdg() {
    // LSI on, so that the one thing missing below is the start key and
    // not the oscillator.
    Rcc::lsi_enable(true);
    bench.verdict("LSI is running (LSION set and ready)", Rcc::lsi_wait_ready());

    // ---- the key is required ---------------------------------------------
    // 28.3.4: PR, RLR and WINR are writable only after 0xAAAA... no:
    // after 0x5555, and any other key value closes the window again -
    // the 0xAAAA refresh included. A refresh, then a bare store, then
    // the register's own update bit as the witness that nothing moved.
    Iwdg::refresh();
    const uint32_t sr_before = Iwdg::status();
    IWDG->PR = 5u;                      // no unlock: this must land nowhere
    const uint32_t sr_locked = Iwdg::status();
    print(serial, "  a bare PR store after a refresh: SR ", hex(sr_before),
          " -> ", hex(sr_locked), " (PVU must stay clear)", crlf);
    bench.verdict("a protected write made without the 0x5555 key raises no "
                  "update at all - the refresh re-locked the window (28.3.4)",
                  sr_locked == sr_before &&
                      (sr_locked & IWDG_SR_PVU_Msk) == 0u);

    // ---- one keyed write, timed from both ends ---------------------------
    Iwdg::unlock();
    const uint32_t c0 = cycles_now();
    IWDG->RLR = 0x0ABCu;
    uint32_t set_us = 0;
    while ((IWDG->SR & IWDG_SR_RVU_Msk) == 0u) {
        set_us = cycles_to_us(cycles_now() - c0);
        if (set_us > 2000u) {
            break;
        }
    }
    const bool bit_appeared = (IWDG->SR & IWDG_SR_RVU_Msk) != 0u;
    const uint32_t c1 = cycles_now();
    const bool completed = Iwdg::sync(IWDG_SR_RVU_Msk);
    const uint32_t waited_ms = cycles_to_us(cycles_now() - c1) / 1000u;

    print(serial, "  a keyed RLR write raised RVU after ", set_us,
          " us and it ", completed ? "cleared" : "was STILL STANDING",
          " after ", waited_ms, " ms of bounded wait", crlf);
    bench.verdict("a keyed write raises its own update bit in IWDG_SR",
                  bit_appeared);
    // MEASURED, and the opposite of what a domain crossing invites one
    // to expect: the bit is raised BY THE STORE, so the read right after
    // it already sees the bit. Only the CLEARING waits for a clock.
    bench.verdict("and it raises it at once - the read right after the store "
                  "already sees it, so only the CLEARING waits for a clock",
                  set_us <= 2u);
    // THE FINDING.
    bench.verdict("the update NEVER completes while the watchdog is stopped: "
                  "LSI running is not enough, because the clock reaches the "
                  "IWDG only at the start key (5.2.14)",
                  !completed);
    bench.verdict("and configure() says so in bounded time instead of hanging",
                  waited_ms < 3000u && !Iwdg::configure(IwdgConfig{}));

    // 28.4.3's Note: the value read comes from the VDD domain and is
    // valid only once RVU is down - so a stopped watchdog keeps
    // reporting whatever it last loaded, which here is the reset value.
    print(serial, "  RLR reads back ", hex(Iwdg::reload()),
          " (0xABC written), SR=", hex(Iwdg::status()), crlf);
    bench.verdict("and the register keeps reporting its OLD value, since the "
                  "read comes from the VDD domain (28.4.3's Note)",
                  Iwdg::reload() == 0x0FFFu);

    // ---- what is refused --------------------------------------------------
    bench.verdict("a window of zero is refused (no refresh could ever be "
                  "legal - Iwdg::force_reset() is the deliberate spelling)",
                  !Iwdg::configure(IwdgConfig{.window = 0}));
    bench.verdict("a reload wider than twelve bits is refused",
                  !Iwdg::configure(IwdgConfig{.reload = 0x1000}));

    // ---- the debug freeze bit ---------------------------------------------
    // It only answers with RCC_APBENR1.DBGEN on, which main() turned on:
    // a peripheral without its bus clock does not take a store (5.2.17),
    // and the first version of this letter measured exactly that.
    const bool freeze_was = Iwdg::debug_freeze();
    Iwdg::debug_freeze(!freeze_was);
    const bool toggled = Iwdg::debug_freeze() != freeze_was;
    Iwdg::debug_freeze(freeze_was);
    print(serial, "  DBG_APB_FZ1.DBG_IWDG_STOP was ", freeze_was,
          " at entry and is restored to it (40.10.3: zero after a POWER-ON "
          "reset and not reset by a system one, so it holds whatever touched "
          "it last)", crlf);
    bench.verdict("the debug freeze bit is writable and restored", toggled);

    Rcc::lsi_enable(false);
}

// =============================================================================
// f - the WWDG, never activated
// =============================================================================
//
// 29.3.3 says the down-counter is free-running "even if the watchdog is
// disabled", and 29.5.3 says EWIF is set at 0x40 "also if the interrupt
// is not enabled". Taken together they make the whole timing path of
// this chapter measurable with WDGA never written - which is what this
// letter does, so that a reference suite carries no one-way switch.
void tf_wwdg() {
    // The closed gate first: 5.2.17 says a peripheral without its bus
    // clock does not answer. Printed, not judged - the samc precedent
    // for a claim the silicon may honour by luck.
    Wwdg::bus_clock(false);
    const uint32_t dark_cr = WWDG->CR;
    Wwdg::bus_clock(true);
    print(serial, "  CR through a closed clock gate: ", hex(dark_cr),
          ", with the gate open: ", hex(Wwdg::cr()), crlf);
    bench.verdict("the bus clock enables and reads back", Wwdg::bus_clock());
    bench.verdict("WDGA is clear: this letter never activates the watchdog",
                  !Wwdg::enabled());

    // CFR readback. Nothing here is enable-protected and nothing
    // synchronizes: both registers are on PCLK.
    constexpr WwdgConfig cfg{
        .prescaler = WwdgPrescaler::div8,
        .window = 0x5A,
        .early_wakeup = false,
    };
    bench.verdict("configure() accepts a legal window", Wwdg::configure(cfg));
    print(serial, "  CFR=", hex(Wwdg::cfr()), " WDGTB=",
          static_cast<uint32_t>(Wwdg::prescaler()), " W=", hex(Wwdg::window()),
          " EWI=", Wwdg::early_wakeup_enabled(), crlf);
    bench.verdict("WDGTB and W read back",
                  Wwdg::prescaler() == WwdgPrescaler::div8 &&
                      Wwdg::window() == 0x5Au);
    bench.verdict("a window below 0x40 is refused (no refresh could ever be "
                  "legal - Wwdg::force_reset() is the deliberate spelling)",
                  !Wwdg::configure(WwdgConfig{.window = 0x3F}));
    bench.verdict("and a window wider than seven bits is refused",
                  !Wwdg::configure(WwdgConfig{.window = 0x80}));

    // THE FREE-RUNNING COUNTER. At /8 a step is 4096 x 8 / 64 MHz =
    // 512 us, so a few milliseconds of watching must see it move - with
    // WDGA clear throughout.
    Wwdg::refresh(0x7F);
    const uint8_t c_start = Wwdg::counter();
    const uint32_t w0 = cycles_now();
    while (cycles_now() - w0 < SysClock::hz / 250u) {   // 4 ms
    }
    const uint8_t c_later = Wwdg::counter();
    print(serial, "  T went ", hex(c_start), " -> ", hex(c_later),
          " in 4 ms with WDGA CLEAR", crlf);
    bench.verdict("the down-counter free-runs with the watchdog disabled "
                  "(29.3.3), which is what makes this letter safe",
                  c_later < c_start);

    // EWIF AT 0x40, TIMED, at two prescalers. From 0x7F the flag is due
    // 63 steps later; the prediction is the chapter's own formula.
    const auto time_ewif = [](WwdgPrescaler p) -> uint32_t {
        (void)Wwdg::configure(WwdgConfig{.prescaler = p, .window = 0x7F});
        Wwdg::clear_flag();
        Wwdg::refresh(0x7F);
        const uint32_t c0 = cycles_now();
        while (!Wwdg::flag()) {
            if (cycles_now() - c0 > SysClock::hz) {   // one second: give up
                return 0;
            }
        }
        const uint32_t took = cycles_now() - c0;
        Wwdg::clear_flag();
        return cycles_to_us(took);
    };
    const uint32_t ewif8 = time_ewif(WwdgPrescaler::div8);
    const uint32_t ewif64 = time_ewif(WwdgPrescaler::div64);
    const uint32_t step8 = wwdg_step_cycles(WwdgPrescaler::div8) / cycles_per_us;
    const uint32_t step64 = wwdg_step_cycles(WwdgPrescaler::div64) / cycles_per_us;
    const uint32_t due8 = step8 * 63u;
    const uint32_t due64 = step64 * 63u;
    print(serial, "  EWIF from 0x7F: ", ewif8, " us at /8 (", due8,
          " due), ", ewif64, " us at /64 (", due64, " due); one step is ",
          step8, " and ", step64, " us", crlf);
    bench.verdict("EWIF is raised at 0x40 with WDGA CLEAR and the interrupt "
                  "disabled (29.5.3), 63 steps after a refresh",
                  ewif8 != 0u && ewif64 != 0u);
    // THE BAND IS 62 TO 63 STEPS, and the missing step is the chapter's
    // own: 29.3.3 says the timing "varies between a minimum and a
    // maximum value, due to the unknown status of the prescaler when
    // writing to the WWDG_CR register" - the write lands mid-step, so
    // the first decrement comes early by whatever is left of it.
    bench.verdict("and it lands between 62 and 63 steps at both prescalers - "
                  "the missing step is 29.3.3's own unknown prescaler phase",
                  ewif8 >= due8 - step8 && ewif8 <= due8 + 200u &&
                      ewif64 >= due64 - step64 && ewif64 <= due64 + 200u);

    // rc_w0: the flag is cleared by writing ZERO, which is the opposite
    // discipline to every write-one-to-clear register in the other two
    // strata. Writing one must NOT clear it.
    Wwdg::refresh(0x41);   // one step from the warning
    const uint32_t c1 = cycles_now();
    while (!Wwdg::flag() && cycles_now() - c1 < SysClock::hz) {
    }
    const bool flag_up = Wwdg::flag();
    WWDG->SR = WWDG_SR_EWIF;                 // writing ONE has no effect
    const bool survived_one = Wwdg::flag();
    Wwdg::clear_flag();                      // writing ZERO clears it
    const bool cleared_zero = !Wwdg::flag();
    bench.verdict("EWIF stands after a write of ONE (29.5.3: rc_w0, and "
                  "writing 1 has no effect)",
                  flag_up && survived_one);
    bench.verdict("and it is cleared by a write of ZERO", cleared_zero);

    // THE INTERRUPT ITSELF, still with WDGA clear. 29.2's feature list
    // says the early wake-up is "triggered (if enabled and the watchdog
    // activated)" while 29.5.2's bit description says only "when set, an
    // interrupt occurs whenever the counter reaches 0x40" - two readings
    // of the same silicon, and the bench decides which.
    ewi_count = 0;
    Wwdg::clear_flag();
    Nvic::clear_pending(Wwdg::irq());
    Nvic::enable(Wwdg::irq());
    (void)Wwdg::configure(WwdgConfig{.prescaler = WwdgPrescaler::div8,
                                     .window = 0x7F,
                                     .early_wakeup = true});
    Wwdg::refresh(0x7F);
    const uint32_t c2 = cycles_now();
    while (ewi_count == 0u && cycles_now() - c2 < SysClock::hz / 4u) {
    }
    const uint16_t fired = ewi_count;
    const bool flag_after = Wwdg::flag();

    print(serial, "  EWI enabled, WDGA still clear: the handler ran ", fired,
          " time(s) in 250 ms; EWIF=", flag_after, crlf);
    // THE ANSWER, AND IT IS 29.2's. The feature list says the early
    // wake-up is "triggered (if enabled and the watchdog activated)"
    // while 29.5.2's bit description says only "when set, an interrupt
    // occurs whenever the counter reaches 0x40". Measured: the FLAG
    // rises (the letter timed it twice above) and NO REQUEST is made -
    // so the activation gates the interrupt, not the flag, and 29.5.2's
    // sentence is the incomplete one.
    bench.verdict("the early-wakeup INTERRUPT does NOT fire while WDGA is "
                  "clear, though EWIF does rise - 29.2's parenthetical is "
                  "right and 29.5.2's bit description is incomplete",
                  fired == 0u && flag_after);

    // The vector and the ISR body are still exercised, by pending the
    // line by hand over a flag the counter really raised: what the
    // handler does with it is the same code either way, and letter i's
    // sixth leg is where the hardware request itself is proven (with the
    // watchdog activated, on the reset it was going to cause anyway).
    Nvic::set_pending(Wwdg::irq());
    const uint32_t c3 = cycles_now();
    while (ewi_count == 0u && cycles_now() - c3 < SysClock::hz / 100u) {
    }
    bench.verdict("pending the line by hand runs the bound handler",
                  ewi_count != 0u);
    bench.verdict("and the ISR body acknowledged the flag (EWIF is down)",
                  !Wwdg::flag());
    Nvic::disable(Wwdg::irq());
    Nvic::clear_pending(Wwdg::irq());

    bench.verdict("EWI is one-way: a configuration that asks for it off "
                  "cannot take it back (29.5.2, set by software and cleared "
                  "by hardware after a reset)",
                  (Wwdg::configure(WwdgConfig{.window = 0x7F}),
                   Wwdg::early_wakeup_enabled()));

    // The debug freeze twin.
    const bool freeze_was = Wwdg::debug_freeze();
    Wwdg::debug_freeze(!freeze_was);
    const bool toggled = Wwdg::debug_freeze() != freeze_was;
    Wwdg::debug_freeze(freeze_was);
    print(serial, "  DBG_APB_FZ1.DBG_WWDG_STOP was ", freeze_was,
          " at entry and is restored to it", crlf);
    bench.verdict("the debug freeze bit is writable and restored", toggled);

    // Park it slow, so the free-running counter's flag costs as little
    // as possible for the rest of the session.
    (void)Wwdg::configure(WwdgConfig{.prescaler = WwdgPrescaler::div128,
                                     .window = 0x7F});
    Wwdg::clear_flag();
    bench.verdict("the board is left with WDGA still clear", !Wwdg::enabled());
}

// =============================================================================
// i - six real resets (outside z: it reboots the board)
// =============================================================================

void bank(uint8_t leg) {
    token.magic = token_magic;
    token.canary = token_canary;
    token.leg = leg;
    token.pass = bench.passed();
    token.fail = bench.failed();
    token.flags_before = Reset::flags();
}

/// Announce a leg, get the words out, and never come back.
[[noreturn]] void await_reset(const char* what) {
    print(serial, "  ", what, crlf);
    console_drain();
    for (;;) {
    }
}

/// Leg 1: the CPU's own SYSRESETREQ, with the flags cleared first so the
/// next boot sees exactly what this reset raises and nothing else.
[[noreturn]] void leg_software() {
    bank(1);
    print(serial, "  leg 1: Reset::software() with the flags cleared first ...",
          crlf);
    console_drain();
    Reset::clear_flags();
    token.flags_before = 0;
    Reset::software();
}

/// Leg 2: an IWDG time-out, MEASURED.
///
/// THE SETTING IS DELIBERATELY NOT THE RESET ONE - /8 with a reload of
/// 0x0EEE instead of /4 with 0x0FFF - so that the time-out itself says
/// whether the configuration landed: 955 ms nominal against the 512 ms
/// an unconfigured watchdog would take. Together with what arm()
/// answered, that is the other half of letter e's finding: an update
/// completes once the watchdog has been started.
///
/// The flags are deliberately NOT cleared, so the next boot is also
/// where the accumulation question is settled; and the elapsed
/// milliseconds banked in .noinit turn a reboot into a measurement of
/// LSI.
[[noreturn]] void leg_iwdg() {
    bank(2);
    print(serial, "  leg 2: the IWDG at /8 with reload 0xEEE (955 ms "
          "nominal, against 512 unconfigured), flags left standing ...", crlf);
    console_drain();

    token.elapsed_ms = 0;
    token.armed_ok = Iwdg::arm(IwdgConfig{
        .prescaler = IwdgPrescaler::div8,
        .reload = 0x0EEE,
        .window = 0x0FFF,
    }) ? 1u : 0u;
    // THROUGH A VOLATILE POINTER, and it is not decoration: a plain
    // store to a non-volatile object inside a loop that never exits is
    // one gcc is entitled to sink out of the loop - which is to say,
    // never to perform. The first version of this leg banked a
    // beautiful zero.
    volatile uint32_t* const elapsed = &token.elapsed_ms;
    const uint32_t t0 = Ticker::ticks();
    for (;;) {
        *elapsed = Ticker::ticks() - t0;
    }
}

/// Leg 3: a WWDG WINDOW VIOLATION - a refresh made while the counter is
/// above the window - with the flags cleared first, so the next boot
/// also proves RMVF really took the previous ones down.
[[noreturn]] void leg_wwdg_window() {
    bank(3);
    print(serial, "  leg 3: the WWDG refreshed ABOVE its window, flags "
          "cleared first ...", crlf);
    console_drain();
    Reset::clear_flags();
    token.flags_before = 0;

    Wwdg::bus_clock(true);
    (void)Wwdg::configure(WwdgConfig{.prescaler = WwdgPrescaler::div1,
                                     .window = 0x40});
    Wwdg::start(0x7F);    // 0x7F is above W: the refresh below is illegal
    Wwdg::refresh(0x7F);
    await_reset("(waiting for the violation to land)");
}

/// Leg 4: panic() with the reporter that resets, so the breadcrumb has
/// to survive a system reset to be read at all.
[[noreturn]] void leg_panic() {
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = 0x5A;
    bank(4);
    print(serial, "  leg 4: panic() through ResetReporter, flags cleared "
          "first ...", crlf);
    console_drain();
    Reset::clear_flags();
    token.flags_before = 0;
    panic<Stm32Platform, ResetReporter>(PanicCode::assert_failed, 0x5A);
}

/// Leg 5: a deliberate HardFault, caught by hard_fault_reset().
///
/// UDF, and it has to be UDF. The obvious candidate - an unaligned
/// volatile word load, which ARMv6-M cannot perform - does NOT fault:
/// gcc knows the constant's misalignment and emits byte loads with
/// shifts instead. The permanently-undefined instruction is the one
/// thing no compiler can turn into something legal.
[[noreturn]] void leg_fault() {
    token.code = static_cast<uint8_t>(PanicCode::kernel_fault);
    token.context = 0x77;
    bank(5);
    print(serial, "  leg 5: UDF -> HardFault -> hard_fault_reset() ...", crlf);
    console_drain();
    __asm__ volatile("udf #0");
    await_reset("(the undefined instruction did not fault)");
}

/// Leg 6: the WWDG left to run down to 0x3F with nothing refreshing it.
[[noreturn]] void leg_wwdg_timeout() {
    bank(6);
    token.ewi_seen = 0;
    token.ewi_counter = 0;
    print(serial, "  leg 6: the WWDG activated, its EARLY WARNING armed, and "
          "never refreshed ...", crlf);
    console_drain();
    Wwdg::bus_clock(true);
    Wwdg::clear_flag();
    Nvic::clear_pending(Wwdg::irq());
    Nvic::enable(Wwdg::irq());
    // THE ONE PLACE THE HARDWARE EWI REQUEST CAN BE PROVEN WITHOUT
    // COSTING ANYTHING. Activating the WWDG is one-way (29.3.2), so no
    // letter of `z` may do it - but this leg is about to be reset by
    // that very watchdog, so the activation costs nothing here. The
    // handler banks what it saw in .noinit and does NOT refresh: the
    // reset follows one step later, which is the point.
    (void)Wwdg::configure(WwdgConfig{.prescaler = WwdgPrescaler::div128,
                                     .window = 0x7F,
                                     .early_wakeup = true});
    Wwdg::start(0x7F);
    await_reset("(waiting for the warning, then the counter reaching 0x3F)");
}

void ti_resets() {
    report_boot();
    bench.verdict("this boot names a reset source",
                  (boot_flags & ResetFlag::all) != 0u);
    bench.verdict("no panic record is pending on a clean start", !boot_record);
    print(serial, "  NOTE: leg 2 starts the IWDG, and nothing in software "
          "can stop it (RM0444 28.3.1) - the reset it causes is what does, "
          "which is the leg's own last verdict", crlf);
    leg_software();
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
        bench.verdict("Reset::software() resets the device and raises SFTRSTF",
                      (boot_flags & ResetFlag::software) != 0u);
        // THE CATCH-ALL, CAUGHT. 5.4.24 bit 26 says PINRSTF is set by a
        // system reset "from any other source" as well as by the pin,
        // and this is the leg that shows it with nothing else standing.
        print(serial, "  a software reset raised: ");
        print_flags(boot_flags);
        print(serial, crlf);
        bench.verdict("PINRSTF is raised BY THE SOFTWARE RESET TOO - it is a "
                      "catch-all and names the pin only when it stands alone",
                      (boot_flags & ResetFlag::pin) != 0u);
        bench.verdict("and nothing else came with them",
                      (boot_flags & ~(ResetFlag::software | ResetFlag::pin)) == 0u);
        leg_iwdg();
    }

    if (leg == 2) {
        // THE MEASUREMENT. RL + 1 = 3823 counts at LSI/8, so the elapsed
        // milliseconds ARE LSI: f = 3823 x 8 / T.
        const uint32_t ms = token.elapsed_ms;
        const uint32_t lsi = ms != 0u ? 30'584'000UL / ms : 0u;
        print(serial, "  the IWDG bit after ", ms, " ms of kernel tick (",
              iwdg_nominal_ms(IwdgPrescaler::div8, 0x0EEE),
              " nominal, 512 if nothing had landed); arm() answered ",
              token.armed_ok != 0u ? "true" : "false", "; LSI = ", lsi, " Hz",
              crlf);
        bench.verdict("an IWDG time-out resets the device",
                      (boot_flags & ResetFlag::independent_watchdog) != 0u);
        // THE OTHER HALF OF LETTER e's FINDING: once the watchdog has
        // been started the updates DO complete, so arm() came back true
        // and the time-out is the one the configuration asked for - not
        // the 512 ms of the reset values.
        bench.verdict("arm() reported the configuration landed - the updates "
                      "complete once the START key has been written (5.2.14)",
                      token.armed_ok != 0u);
        bench.verdict("and the time-out proves it on the wall clock: it is "
                      "the configured 955 ms nominal and not the 512 ms an "
                      "unconfigured watchdog would take",
                      ms >= 860u && ms <= 1050u);
        bench.verdict("the LSI it implies is inside the datasheet's "
                      "29.5..34 kHz (DS13560 table 46)",
                      lsi >= 29'500u && lsi <= 34'000u);

        // THE OTHER READING OF 28.3.1, and this suite exists to settle
        // it: "Once running, the IWDG cannot be stopped" is followed, in
        // the Stop-mode sections, by "except upon a reset". If the first
        // half were the whole truth this board would now be rebooting
        // every 955 ms with nothing refreshing anything - the SAM's and
        // the AVR's habits, and the widespread STM32 lore. It is not.
        const uint32_t quiet0 = Ticker::ticks();
        while (Ticker::ticks() - quiet0 < 1500u) {
        }
        print(serial, "  1500 ms have passed with NOTHING refreshing the "
              "watchdog, and this line is being printed", crlf);
        bench.verdict("THE RESET STOPPED IT: 28.3.1's 'except upon a reset' "
                      "is literal on this family - a boot after an IWDG reset "
                      "outlives its own time-out with nobody feeding it",
                      Ticker::ticks() - quiet0 >= 1500u);

        // THE ACCUMULATION QUESTION, ANSWERED. This leg did not clear
        // the flags, so leg 1's SFTRSTF must still be standing beside
        // the new IWDGRSTF.
        print(serial, "  flags standing when the leg started: ");
        print_flags(token.flags_before);
        print(serial, crlf);
        bench.verdict("THE FLAGS ACCUMULATE: leg 1's SFTRSTF is still here "
                      "beside the new IWDGRSTF, because nothing wrote RMVF "
                      "in between (5.4.24 - a history, not a cause)",
                      (boot_flags & ResetFlag::software) != 0u &&
                          (boot_flags & ResetFlag::independent_watchdog) != 0u);

        // ...and the other half of fact 3: the IWDG is STILL RUNNING.
        leg_wwdg_window();
    }

    if (leg == 3) {
        bench.verdict("a WWDG refresh above the window resets the device",
                      (boot_flags & ResetFlag::window_watchdog) != 0u);
        bench.verdict("RMVF really took the earlier flags down: neither "
                      "SFTRSTF nor IWDGRSTF is standing",
                      (boot_flags & (ResetFlag::software |
                                     ResetFlag::independent_watchdog)) == 0u);
        bench.verdict("the WWDG came back DISABLED (WDGA is cleared by the "
                      "reset - 29.5.1 - unlike the IWDG)",
                      !Wwdg::enabled());
        leg_panic();
    }

    if (leg == 4) {
        // NOTE what had to happen for this to work at all: panic() ends
        // in break_here(), which is BKPT, and with no debugger attached
        // that escalates into HardFault_Handler - so the reset here may
        // have come from ResetReporter or from hard_fault_reset(), and
        // either way the RECORD must still say what panic() reported.
        // That is why the fault body refuses to overwrite a valid one.
        bench.verdict("a panic through ResetReporter resets the device",
                      (boot_flags & ResetFlag::software) != 0u);
        bench.verdict("the breadcrumb survived the reset (SRAM is promised "
                      "nowhere, so this is a measurement)",
                      boot_record.has_value());
        bench.verdict("its code is the one panic() was given",
                      boot_record && boot_record->code == token.code);
        bench.verdict("its context byte came through untouched",
                      boot_record && boot_record->context == token.context);
        bench.verdict("a software reset and a watchdog reset are "
                      "DISTINGUISHABLE at the next boot",
                      (boot_flags & ResetFlag::watchdog) == 0u);
        leg_fault();
    }

    if (leg == 5) {
        bench.verdict("a HardFault reaches hard_fault_reset() and resets",
                      (boot_flags & ResetFlag::software) != 0u);
        bench.verdict("the record it left says kernel_fault",
                      boot_record && boot_record->code == token.code);
        bench.verdict("with the context byte the body was given",
                      boot_record && boot_record->context == token.context);
        leg_wwdg_timeout();
    }

    // leg 6: the last boot.
    bench.verdict("a WWDG counter reaching 0x3F resets the device",
                  (boot_flags & ResetFlag::window_watchdog) != 0u);
    print(serial, "  the early warning ",
          token.ewi_seen ? "FIRED" : "did not fire",
          " on the way down; the counter read ", hex(token.ewi_counter),
          " inside its handler", crlf);
    bench.verdict("with the watchdog ACTIVATED the early-wakeup interrupt "
                  "really is requested - the half letter f cannot stage, "
                  "paid for by a reset that was going to happen anyway",
                  token.ewi_seen != 0u);
    bench.verdict("and it fired at the warning value: the counter it saw was "
                  "0x40 or the 0x3F it was about to become",
                  token.ewi_counter == 0x40u || token.ewi_counter == 0x3Fu);
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
    print(serial, crlf, "test_stm32_platform - STM32G0B1RE platform + reset "
          "(RM0444 5.1/5.4.24, ch. 28, ch. 29), clk=", SysClock::hz, " Hz",
          crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler
// is a spin loop - so every line this suite can raise is bound.
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void WWDG_IRQHandler() {
    const uint8_t t = brio::Wwdg::counter();
    if (brio::Wwdg::isr()) {
        ewi_cycles = cycles_now();
        ewi_count = static_cast<uint16_t>(ewi_count + 1);
        if (token.magic == token_magic && token.leg == 6) {
            token.ewi_seen = 1;
            token.ewi_counter = t;
        }
    }
}

/// The whole point of stm32g0/reset.hpp's fault body: a crash becomes a
/// note the next boot can read, instead of a spin nobody sees.
extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::Stm32Platform>(token.context);
}

int main() {
    // Sampled FIRST: the flags are not cleared by reading, but the panic
    // record is fetch-and-clear and must be taken exactly once.
    boot_flags = brio::Reset::flags();
    boot_record = brio::take_panic_record<brio::Stm32Platform>();

    // NOTHING FEEDS ANYTHING HERE, and that is a MEASUREMENT and not an
    // oversight: RM0444 28.3.1 says the IWDG "cannot be stopped except
    // upon a reset", and letter i's second leg is where this suite
    // proves the last four words literal - the reset it causes stops
    // it. (The widespread STM32 lore, and the SAM's and AVR's habits,
    // would have this board reboot-looping from then on.)
    boot_lsi_forced = brio::Rcc::lsi_ready() && !brio::Rcc::lsi_enabled();

    const bool clock_ok = SysClock::init();
    // The DBGMCU is an APB peripheral with an enable bit of its own, and
    // it is CLEAR AT RESET: without it the two watchdogs' freeze bits do
    // not take a store (5.2.17), which is what the first run of letters
    // e and f measured before this line existed.
    brio::Rcc::apb1_clock(RCC_APBENR1_DBGEN, true);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the boot story: the flags as the history they are",
                 ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the SysTick timebase and its VAL arithmetic", tc_ticker);
    bench.letter('d', "delay_us on the SysTick counter", td_delay);
    bench.letter('e', "the IWDG, never started", te_iwdg);
    bench.letter('f', "the WWDG, never activated", tf_wwdg);
    bench.letter('i', "SIX REAL RESETS (reboots the board)", ti_resets, false);

    // A pending token means a leg of letter i is waiting to be judged:
    // resume it instead of printing a banner nobody asked for.
    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL64" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " flags=",
                    brio::hex(boot_flags), crlf);
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
