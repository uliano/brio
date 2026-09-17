// test_v203_platform - the reference bench suite for the CH32V203's
// PLATFORM: the running half (the mstatus.MIE critical section, the
// WFE-shaped idle hook, the 64-bit STK timebase, delay_us on its
// counter, the interrupt round trip and what the core's hardware
// prologue buys it, and the microprocessor configuration register the
// crt writes because the vendor does) and the failing half -
// ch32v203/reset.hpp (the reset flags as the history they are, the
// software reset through the core's own controller, the panic
// breadcrumb across a real reset, the fault vector's record with the
// cause the core left in mcause).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the probe's own serial (USART1 on
// PA9/PA10) and every clock this suite measures is inside the chip. The
// LED on PB2 marks a keystroke for a hand at the desk and is judged on
// nothing.
//
// What is exercised, letter by letter:
//   a  the boot story: RCC_RSTSCKR read as the ACCUMULATING history it
//      is, the six flags named, and take_flags() leaving the register
//      clean
//   b  the critical section: mstatus.MIE saved and restored, nesting,
//      and the idle hook - a WFE, not a WFI, on this core - returning
//      on the tick with interrupts enabled
//   c  the STK timebase: the compare the ticker programmed, the high
//      half of a 64-bit counter that never moves under the reload,
//      ticks/millis/secs/now coherence, and the CNT-delta accumulation
//      delay.hpp is built on checked against the interrupt count over
//      200 reloads of the low half
//   d  delay_us: at least, never early, the cap, the refusal, with a
//      100 us and a 900 us wait counted in HCLK cycles
//   e  THE INTERRUPT ROUND TRIP, in HCLK cycles on the STK counter: a
//      line raised by hand (the software interrupt, PFIC IPSR) with the
//      cycle count read at the raise, at the handler's first and last
//      statements, and when the raiser sees the handler done. Entry,
//      body, exit and the whole trip are what the hardware
//      prologue/epilogue is worth: the image says which way it was
//      built (the CH32V203_HPE option), and the same letter on both
//      builds is the measurement the project's default rests on
//   f  corecfgr (CSR 0xBC0), which the crt writes 0x1F into because
//      WCH's own startup file does and no document says what the bits
//      are: one known loop timed with the register as the crt left it
//      and with it cleared, the register restored either way
//   g  the tick rate against the HOST's clock: the letter brackets a
//      span of its own ticks between two console lines, and whoever
//      times those two lines has the ratio
//
//   i  (by name only) THREE REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session a tool can judge from a
//      single capture. Run it with
//          brio run <board> i --app test_v203_platform --expect="->" --timeout 60
//      Legs: a software reset (SFTRSTF at the next boot), a panic
//      through ResetReporter (the breadcrumb read back), and a
//      deliberate fault through ebreak with no debugger attached (the
//      fault vector's own record, kernel_fault, carrying the core's
//      cause byte). BOTH trap entries of this table are bound - the
//      exception vector at index 3 and the breakpoint one at index 9 -
//      and each leaves its own index in the token, which is how the
//      letter says WHICH of them the silicon took.
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/reset.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"
#include "kernel/panic.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v203Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter i lives in: .noinit, so the crt neither loads nor
// zeroes it, and inline because the platform's own breadcrumb is a
// static inline member in the same section (gcc gives both a COMDAT
// group; a plain definition next to them fails the link with a section
// type conflict). Its magic word is not decoration: nothing promises
// SRAM across a reset, so every read of this object is guarded.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5A23;

struct Token {
    uint16_t magic;
    uint8_t leg;        ///< which reset we are waiting for (0 = none pending)
    uint8_t vector;     ///< which vector body ran (leg 3)
    uint16_t pass;      ///< letter i's tally so far
    uint16_t fail;
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial> bench;

// What this boot was told, sampled once in main() before anything can
// disturb it.
uint32_t boot_flags = 0;
std::optional<PanicRecord> boot_record;

// The STK counter as a cycle counter: it runs at HCLK and reloads every
// tick, so two readings less than a tick apart are a cycle count with
// the reload folded in. Only the low half is read - the reload puts the
// counter back to zero long before the high half could move.
uint32_t cycles_now() { return stk()->CNTL; }
uint32_t cycles_between(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMPLR + 1u;
    return to >= from ? to - from : to + period - from;
}
uint32_t cycles_per_us() { return SysClock::hz / 1'000'000u; }

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);   // the last byte's own time on the wire
}

void print_flags(uint32_t flags) {
    print(serial, "    low_power=", (flags & ResetFlag::low_power) != 0u,
          " wwdg=", (flags & ResetFlag::window_watchdog) != 0u,
          " iwdg=", (flags & ResetFlag::independent_watchdog) != 0u,
          " software=", (flags & ResetFlag::software) != 0u,
          " power=", (flags & ResetFlag::power) != 0u,
          " pin=", (flags & ResetFlag::pin) != 0u, crlf);
}

// ---------------------------------------------------------------------------
// a - the boot story
// ---------------------------------------------------------------------------
void ta_boot() {
    print(serial, "  RSTSCKR flags at boot: ", hex(boot_flags), crlf);
    print_flags(boot_flags);
    bench.verdict("some reset flag stood at this boot (they accumulate "
                  "until RMVF, and a power-on sets PORRSTF)",
                  boot_flags != 0u);

    const uint32_t first = Reset::flags();
    const uint32_t second = Reset::flags();
    const uint32_t taken = Reset::take_flags();
    const uint32_t after = Reset::flags();
    print(serial, "  flags()=", hex(first), " again=", hex(second),
          " take_flags()=", hex(taken), " after=", hex(after), crlf);
    bench.verdict("flags() is non-destructive: two reads agree",
                  first == second);
    bench.verdict("take_flags() returns what stood", taken == second);
    bench.verdict("and leaves the register clean", after == 0u);
    bench.verdict("RMVF is back at zero afterwards (a plain bit, not a strobe)",
                  (rcc()->RSTSCKR & rstsckr_rmvf) == 0u);
    bench.verdict("clearing the flags left the LSI bits of the same register "
                  "alone",
                  (rcc()->RSTSCKR & ~(ResetFlag::all | rstsckr_rmvf |
                                      rcc_lsion | rcc_lsirdy)) == 0u);
    bench.verdict("no panic record was pending at this boot (letter i "
                  "is the one that leaves one)",
                  !boot_record.has_value());
}

// ---------------------------------------------------------------------------
// b - the critical section and the idle hook
// ---------------------------------------------------------------------------
void tb_critical() {
    bench.verdict("interrupts are enabled when a letter runs",
                  P::interrupts_enabled());

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
    bench.verdict("leaving the INNER scope does not unmask (MIE is read and "
                  "cleared in one csrrci, and only what was set is restored)",
                  !after_inner);
    bench.verdict("leaving the outer scope unmasks", after_outer);

    // The tick keeps counting through a masked window - the STK interrupt
    // stays pending, it is not lost - and the pending bit coalesces.
    const uint32_t t0 = Ticker::ticks();
    {
        P::CriticalSection cs;
        uint32_t spun = 0;
        uint32_t last = cycles_now();
        while (spun < SysClock::hz / 200u) {   // 5 ms masked
            const uint32_t now = cycles_now();
            spun += cycles_between(last, now);
            last = now;
        }
    }
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  5 ms with interrupts masked advanced the tick by ",
          through_mask, " ms", crlf);
    bench.verdict("a masked window LOSES ticks (the STK interrupt is a "
                  "pending bit: one tick is delivered, the rest coalesce)",
                  through_mask >= 1u && through_mask <= 5u);

    // idle(): a WFE on this core, then unmask. Any enabled interrupt
    // wakes it, so the console must be silent first or the USART's own
    // transmit interrupt returns it in microseconds.
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    const uint32_t c1 = cycles_now();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        {
            P::CriticalSection cs;   // the kernel calls idle() masked
            P::idle();
        }
        ++calls;
    }
    const uint32_t idle_cycles = cycles_between(c1, cycles_now());
    print(serial, "  with the console silent, ", calls,
          " idle() call(s) covered the ", idle_cycles / cycles_per_us(),
          " us to the next tick", crlf);
    bench.verdict("idle() sleeps and the tick is what brings it back, inside "
                  "one tick period",
                  calls >= 1u && calls < 20u);
    bench.verdict("and it comes back with interrupts enabled",
                  P::interrupts_enabled());
}

// ---------------------------------------------------------------------------
// c - the STK timebase
// ---------------------------------------------------------------------------
void tc_ticker() {
    const uint32_t cmp = stk()->CMPLR;
    print(serial, "  STK CTLR=", hex(stk()->CTLR), " CMPLR=", cmp,
          " (", SysClock::hz / Ticker::ticks_per_second - 1u, " expected)",
          " CMPHR=", stk()->CMPHR, " CNTH=", stk()->CNTH, crlf);
    bench.verdict("the compare is Clock::hz / tps - 1",
                  cmp == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on HCLK, reloading, with its interrupt armed",
                  (stk()->CTLR & (stk_ste | stk_stie | stk_stclk | stk_stre |
                                  stk_mode)) ==
                      (stk_ste | stk_stie | stk_stclk | stk_stre));
    bench.verdict("the high half of the 64-bit counter never moves: the reload "
                  "puts the low half back to zero at the compare",
                  stk()->CNTH == 0u && stk()->CMPHR == 0u);

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

    // The CNT-delta accumulation delay.hpp rests on, against the
    // interrupt count over 200 reloads: an error there is a whole
    // period, not a rounding. BOTH ENDS OF THE SPAN ARE A TICK EDGE -
    // the sum of the deltas telescopes, so what it carries beyond the
    // whole periods is the phase the loop started at, and starting on a
    // fresh tick is what makes the residue the loop's own granularity
    // instead of that phase.
    const uint32_t target = 200;
    const uint32_t period = cmp + 1u;
    const uint32_t edge = Ticker::ticks();
    while (Ticker::ticks() == edge) {
    }
    uint32_t last = cycles_now();
    uint32_t accumulated = 0;
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < target) {
        const uint32_t now = cycles_now();
        accumulated += cycles_between(last, now);
        last = now;
    }
    const uint32_t expected = target * period;
    const uint32_t err = accumulated > expected ? accumulated - expected
                                                : expected - accumulated;
    print(serial, "  200 ticks = ", accumulated, " cycles by CNT deltas, ",
          expected, " expected, error ", err, " cycles (",
          err * 1000u / (expected / 1000u), " ppm)", crlf);
    bench.verdict("the CNT-delta accumulation tracks the interrupt count over "
                  "200 reloads to under a period's worth of error",
                  err < period);
}

// ---------------------------------------------------------------------------
// d - delay_us
// ---------------------------------------------------------------------------
void td_delay() {
    // 500 us twenty times is 10 ms: at least ten ticks, never fewer.
    const uint32_t t0 = Ticker::ticks();
    bool served = true;
    for (uint8_t i = 0; i < 20u; ++i) {
        served = delay_us(clock, 500) && served;
    }
    const uint32_t took = Ticker::ticks() - t0;
    print(serial, "  20 x delay_us(500) took ", took, " ticks", crlf);
    bench.verdict("every 500 us wait was served", served);
    bench.verdict("twenty of them are at least 10 ms and no more than 12",
                  took >= 10u && took <= 12u);

    const uint32_t asked_short = 100u * cycles_per_us();
    const uint32_t c0 = cycles_now();
    const bool short_ok = delay_us(clock, 100);
    const uint32_t short_spent = cycles_between(c0, cycles_now());
    print(serial, "  delay_us(100) spent ", short_spent, " cycles (",
          asked_short, " asked)", crlf);
    bench.verdict("100 us is at least what it asked for and under 4 per cent "
                  "over",
                  short_ok && short_spent >= asked_short &&
                      short_spent < asked_short + asked_short / 25u);

    const uint32_t asked_long = 900u * cycles_per_us();
    const uint32_t c1 = cycles_now();
    const bool long_ok = delay_us(clock, 900);
    const uint32_t long_spent = cycles_between(c1, cycles_now());
    print(serial, "  delay_us(900) spent ", long_spent, " cycles (",
          asked_long, " asked)", crlf);
    bench.verdict("900 us - the longest wait that still fits a tick period - "
                  "is at least what it asked for and under 1 per cent over",
                  long_ok && long_spent >= asked_long &&
                      long_spent < asked_long + asked_long / 100u);

    bench.verdict("a wait of one tick period (1000 us) is refused",
                  !delay_us(clock, 1000));
    bench.verdict("and so is anything longer", !delay_us(clock, 50'000));
    bench.verdict("and so is the 65536 us gate the arithmetic rests on",
                  !delay_us(clock, 65'536));
    bench.verdict("a zero wait is served in no time", delay_us(clock, 0));
}

// ---------------------------------------------------------------------------
// e - the interrupt round trip
// ---------------------------------------------------------------------------
volatile uint32_t sw_entry_cycles = 0;
volatile uint32_t sw_exit_cycles = 0;
volatile uint32_t sw_hits = 0;

void te_latency() {
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
    print(serial, "  built with the hardware prologue/epilogue (CH32V203_HPE=ON)", crlf);
#else
    print(serial, "  built with gcc's own prologue/epilogue (CH32V203_HPE=OFF)", crlf);
#endif
    Pfic::enable(Irq::software);

    uint32_t best_entry = 0xFFFF'FFFFu, best_body = 0xFFFF'FFFFu;
    uint32_t best_exit = 0xFFFF'FFFFu, best_trip = 0xFFFF'FFFFu;
    uint32_t worst_entry = 0, worst_trip = 0;
    const uint32_t rounds = 64;
    uint32_t seen = 0;
    for (uint32_t r = 0; r < rounds; ++r) {
        console_drain();
        const uint32_t before = sw_hits;
        sw_entry_cycles = 0;
        sw_exit_cycles = 0;
        // A tick landing inside the window would add its own handler to
        // the measurement: wait for a fresh tick, then there is a whole
        // millisecond of quiet.
        const uint32_t t = Ticker::ticks();
        while (Ticker::ticks() == t) {
        }
        const uint32_t c0 = cycles_now();
        Pfic::set_pending(Irq::software);
        // The pending write is posted and the core runs on until the
        // line is taken: the raiser has to WAIT for the handler, and the
        // reading after the wait carries one spin iteration of its own.
        // The bound is what turns a line this core will not raise into a
        // verdict instead of a hang.
        uint32_t spin = 0;
        while (sw_hits == before && spin < 100'000u) {
            ++spin;
        }
        if (sw_hits == before) {
            break;
        }
        const uint32_t c1 = cycles_now();
        ++seen;
        const uint32_t entry = cycles_between(c0, sw_entry_cycles);
        const uint32_t body = cycles_between(sw_entry_cycles, sw_exit_cycles);
        const uint32_t exit = cycles_between(sw_exit_cycles, c1);
        const uint32_t trip = cycles_between(c0, c1);
        if (entry < best_entry) { best_entry = entry; }
        if (entry > worst_entry) { worst_entry = entry; }
        if (body < best_body) { best_body = body; }
        if (exit < best_exit) { best_exit = exit; }
        if (trip < best_trip) { best_trip = trip; }
        if (trip > worst_trip) { worst_trip = trip; }
    }
    Pfic::disable(Irq::software);

    print(serial, "  ", seen, " of ", rounds, " raises were taken", crlf);
    if (seen == 0u) {
        bench.verdict("the software interrupt was raised by hand and taken",
                      false);
        return;
    }
    print(serial, "  entry (raise -> the handler's first statement): ",
          best_entry, "..", worst_entry, " cycles", crlf);
    print(serial, "  body  (first -> last statement of the handler): ",
          best_body, " cycles", crlf);
    print(serial, "  exit  (last statement -> the raiser sees it done): ",
          best_exit, " cycles, one spin iteration included", crlf);
    print(serial, "  whole trip (raise -> the raiser sees it done): ",
          best_trip, "..", worst_trip, " cycles", crlf);
    bench.verdict("every raise of the software interrupt was taken",
                  seen == rounds);
    bench.verdict("the handler was entered within 100 cycles of the raise",
                  best_entry <= 100u);
    bench.verdict("and the whole trip is under 300 cycles",
                  best_trip < 300u);
}

// ---------------------------------------------------------------------------
// f - the microprocessor configuration register
// ---------------------------------------------------------------------------
uint32_t read_corecfgr() {
    uint32_t v;
    __asm__ volatile("csrr %0, 0xbc0" : "=r"(v));
    return v;
}
void write_corecfgr(uint32_t v) {
    __asm__ volatile("csrw 0xbc0, %0" ::"r"(v) : "memory");
}

/// A loop with a branch and a load in it, the thing a pipeline and a
/// branch predictor are for. The data is a pseudo-random byte stream,
/// so the branch is one a predictor cannot learn; the function is not
/// inlined and its result is consumed, so the measurement has something
/// to measure.
uint8_t work_data[64];
volatile uint32_t work_sink = 0;

[[gnu::noinline]] uint32_t work_loop(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t v = work_data[i & 63u];
        if ((v & 1u) != 0u) {
            acc += v;
        } else {
            acc ^= v;
        }
    }
    return acc;
}

/// The loop's cost in HCLK cycles, measured with interrupts masked so
/// no handler of ours lands inside it.
uint32_t time_work(uint32_t n) {
    P::CriticalSection cs;
    const uint32_t c0 = cycles_now();
    const uint32_t acc = work_loop(n);
    const uint32_t spent = cycles_between(c0, cycles_now());
    work_sink = acc;
    return spent;
}

void tf_corecfgr() {
    uint32_t seed = 0x1357'9BDFu;
    for (uint8_t i = 0; i < 64u; ++i) {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        work_data[i] = static_cast<uint8_t>(seed);
    }
    const uint32_t n = 3000;
    const uint32_t period = stk()->CMPLR + 1u;

    const uint32_t as_found = read_corecfgr();
    print(serial, "  corecfgr as the crt left it: ", hex(as_found), crlf);

    const uint32_t with_bits = time_work(n);
    uint32_t without_bits = 0;
    uint32_t while_cleared = 0;
    {
        P::CriticalSection cs;
        write_corecfgr(0);
        while_cleared = read_corecfgr();
        without_bits = time_work(n);
        write_corecfgr(as_found);
    }
    const uint32_t restored = read_corecfgr();

    print(serial, "  ", n, " iterations of a loop with a branch and a load: ",
          with_bits, " cycles at ", hex(as_found), ", ", without_bits,
          " cycles at ", hex(while_cleared), crlf);
    bench.verdict("the loop ran with the register as the crt left it, inside "
                  "one tick period",
                  with_bits > 0u && with_bits < period);
    bench.verdict("and ran with the register cleared, inside one tick period",
                  without_bits > 0u && without_bits < period);
    bench.verdict("the register takes a zero: the second run really ran "
                  "without the bits",
                  while_cleared == 0u);
    bench.verdict("corecfgr is writable at run time and reads back what was "
                  "written",
                  restored == as_found);
    bench.verdict("the crt's value is the one WCH's own startup file writes",
                  as_found == 0x1Fu);
}

// ---------------------------------------------------------------------------
// g - the tick rate against the host's clock
// ---------------------------------------------------------------------------
void tg_rate() {
    const uint32_t span = 5000;   // ticks, i.e. 5 s if the tick is a ms
    print(serial, "  bracketing ", span,
          " ticks between this line and the next", crlf);
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    const uint32_t s0 = Ticker::secs();
    while (Ticker::ticks() - t0 < span) {
    }
    const uint32_t t1 = Ticker::ticks();
    const uint32_t s1 = Ticker::secs();
    print(serial, "  ", t1 - t0, " ticks and ", s1 - s0,
          " s later (the host's clock times the two lines)", crlf);
    bench.verdict("the counter advanced by exactly the span asked",
                  t1 - t0 == span);
    bench.verdict("the second counter advanced by the same span, in whole "
                  "seconds",
                  s1 - s0 == span / Ticker::ticks_per_second ||
                      s1 - s0 == span / Ticker::ticks_per_second + 1u);
}

// ---------------------------------------------------------------------------
// i - three real resets
// ---------------------------------------------------------------------------
void ti_leg_start(uint8_t leg) {
    token.magic = token_magic;
    token.leg = leg;
    token.vector = 0;
    console_drain();
}

void ti_resets() {
    token.pass = 0;
    token.fail = 0;
    print(serial, "  leg 1: a software reset -> SFTRSTF at the next boot", crlf);
    Reset::clear_flags();
    ti_leg_start(1);
    Reset::software();
}

void ti_judge(const char* what, bool ok) {
    bench.verdict(what, ok);
    if (ok) { ++token.pass; } else { ++token.fail; }
}

void ti_resume() {
    const uint8_t leg = token.leg;
    token.leg = 0;
    bench.reset_tally();
    bench.resume_tally(token.pass, token.fail);
    print(serial, crlf, "-> back from leg ", leg, ": flags=", hex(boot_flags), crlf);
    print_flags(boot_flags);

    switch (leg) {
        case 1:
            ti_judge("leg 1: SFTRSTF stands after Reset::software()",
                     (boot_flags & ResetFlag::software) != 0u);
            ti_judge("leg 1: and no watchdog or low-power flag stands beside it",
                     (boot_flags & (ResetFlag::watchdog | ResetFlag::low_power)) == 0u);
            ti_judge("leg 1: no breadcrumb - a plain reset writes none",
                     !boot_record.has_value());
            print(serial, "  leg 2: panic<ResetReporter>(queue_overflow, 7) -> the "
                          "breadcrumb at the next boot", crlf);
            Reset::clear_flags();
            ti_leg_start(2);
            panic<P, ResetReporter>(PanicCode::queue_overflow, 7);
        case 2:
            ti_judge("leg 2: the panic reset the board (SFTRSTF)",
                     (boot_flags & ResetFlag::software) != 0u);
            ti_judge("leg 2: the breadcrumb crossed the reset",
                     boot_record.has_value());
            ti_judge("leg 2: with the code and context panic() was given",
                     boot_record.has_value() &&
                         boot_record->code == static_cast<uint8_t>(PanicCode::queue_overflow) &&
                         boot_record->context == 7u);
            print(serial, "  leg 3: ebreak with no debugger -> the fault "
                          "vector's own record (kernel_fault)", crlf);
            Reset::clear_flags();
            ti_leg_start(3);
            P::break_here();
            for (;;) {
            }
        case 3:
            print(serial, "  the ebreak was taken to vector ", token.vector,
                  " of the table", crlf);
            if (boot_record.has_value()) {
                print(serial, "  its record: code=", boot_record->code,
                      " context=", hex(boot_record->context), " (",
                      fault_cause_name(boot_record->context), ")", crlf);
            }
            ti_judge("leg 3: the fault body reset the board (SFTRSTF)",
                     (boot_flags & ResetFlag::software) != 0u);
            ti_judge("leg 3: the fault vector wrote its own record",
                     boot_record.has_value() &&
                         boot_record->code == static_cast<uint8_t>(PanicCode::kernel_fault));
            ti_judge("leg 3: carrying the core's own cause - an exception, "
                     "and the breakpoint one",
                     boot_record.has_value() &&
                         !fault_was_interrupt(boot_record->context) &&
                         fault_code(boot_record->context) ==
                             static_cast<uint8_t>(FaultCause::breakpoint));
            bench.end_letter();
            break;
        default:
            print(serial, "  unknown leg ", leg, crlf);
            break;
    }
}

void banner() {
    print(serial, crlf, "test_v203_platform - ", device::part_name,
          " (clk=144 MHz PLL, tick=STK 1000 Hz)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The software interrupt: the first thing it does is read the cycle
/// counter, which is letter e's whole measurement.
extern "C" BRIO_CH32_INTERRUPT void software_handler() {
    sw_entry_cycles = brio::stk()->CNTL;
    brio::Pfic::clear_pending(brio::Irq::software);
    sw_hits = sw_hits + 1u;
    sw_exit_cycles = brio::stk()->CNTL;
}

/// A crash becomes a note the next boot can read (letter i, leg 3).
/// THIS FAMILY'S TABLE HAS TWO ENTRIES A TRAP CAN ARRIVE ON - the
/// exception vector at index 3 and the breakpoint one at index 9 - so
/// both are bound, each leaving its own index in the token before the
/// shared body writes the record and resets.
extern "C" BRIO_CH32_INTERRUPT void fault_handler() {
    token.vector = 3;
    brio::fault_reset<P>();
}

extern "C" BRIO_CH32_INTERRUPT void breakpoint_handler() {
    token.vector = 9;
    brio::fault_reset<P>();
}

int main() {
    // Sampled FIRST: the flags are not cleared by reading, but the panic
    // record is fetch-and-clear and must be taken exactly once.
    boot_flags = brio::Reset::flags();
    boot_record = brio::take_panic_record<P>();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the boot story: the flags as the history they are", ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the STK timebase and its CNT arithmetic", tc_ticker);
    bench.letter('d', "delay_us on the STK counter", td_delay);
    bench.letter('e', "the interrupt round trip, in cycles", te_latency);
    bench.letter('f', "corecfgr: a known loop with its bits and without", tf_corecfgr);
    bench.letter('g', "the tick rate, for the host's clock to judge", tg_rate);
    bench.letter('i', "THREE REAL RESETS (reboots the board)", ti_resets, false);

    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL144" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", " flags=",
                    brio::hex(boot_flags), brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
