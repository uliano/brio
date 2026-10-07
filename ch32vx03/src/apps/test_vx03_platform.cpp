// test_vx03_platform - the reference bench suite for the PLATFORM of the
// CH32V203 and the CH32V303: the running half (the mstatus.MIE critical
// section, the WFE-shaped idle hook, the 64-bit STK timebase, delay_us
// on its counter, the interrupt round trip and what the core's hardware
// prologue buys it, the microprocessor configuration register the crt
// writes because the vendor does, the floating-point unit of the
// CH32V303's QingKe V4F and what a bus master gets while the core
// sleeps) and the failing half -
// ch32vx03/reset.hpp (the reset flags as the history they are, the
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
// THE BANNER carries one probe and no verdict: the word WCH's GPIO
// library reads at 0x40022030 to decide whether a CH32V20x_D6 die answers
// PORTC's top three pins on bits 0..2 (portc_probe() below).
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
//      built (the CH32VX03_HPE option), and the same letter on both
//      builds is the measurement the project's default rests on
//   f  corecfgr (CSR 0xBC0), which the crt writes 0x1F into because
//      WCH's own startup file does and no document says what the bits
//      are: one known loop timed with the register as the crt left it
//      and with it cleared, the register restored either way
//   g  the tick rate against the HOST's clock: the letter brackets a
//      span of its own ticks between two console lines, and whoever
//      times those two lines has the ratio
//   j  (a part with the FPU) THE FLOATING-POINT UNIT'S STATE: mstatus.FS
//      as the crt left it (sampled first thing in main()) and as the
//      letter finds it, set back to Initial and read after a write to
//      fcsr alone - the crt's own last step - and after one fused
//      multiply-add, fcsr's sticky flags after a division by zero and
//      after an inexact quotient (the unit records, it never traps),
//      and the rounding mode written, read back and seen in a result
//   k  (a part with the FPU and TIM6) FLOATING POINT UNDER AN INTERRUPT
//      STORM: the loop keeps twenty float accumulators whose exact
//      values are known while TIM6, at the core's clock, interrupts
//      every 400 cycles - the kernel's tick going on beside it - with a
//      handler doing float arithmetic on twenty locals of its own, bound
//      by the float trampoline (BRIO_CH32_VECTOR_FLOAT, which stores the
//      twenty caller-saved f-registers around its body); every
//      accumulator is checked bit for bit, and every handler checks its
//      own result
//   l  (a part with the FPU) THE ROUND TRIP WITH FLOATING POINT, letter
//      e's method on three handlers: one that touches no FP register,
//      one that does float arithmetic (the float trampoline's twenty
//      stores and loads around its body), and one that CALLS a function
//      the compiler cannot see into - which under the plain trampoline
//      saves no f-register at all, the hardware prologue pushing the
//      integer ones
//   m  THE BUS IN SLEEP: a memory-to-memory DMA1 block of 65535 words
//      between two fixed addresses, the platform's idle() with the
//      bus-master count that keeps it awake set aside, the core counter
//      as a one-shot wake some two milliseconds later and no other
//      interrupt enabled; the channel's remaining count says how much
//      the DMA moved while the core slept, against the same span awake
//   n  THE MASK'S SHADOW: a line raised by hand and, a swept number of
//      instructions later, the global mask (the platform guard's own
//      csrrci) or the line's own enable (PFIC_IRER) taken away - with and
//      without a fence.i after it. The handler records mepc, so every
//      trial says whether the interrupt was taken before the mask, INSIDE
//      the masked region, or after the unmask; the count inside is what
//      RM V2.5's note about a fence.i after a mask is asking about. The
//      image says which way the hardware prologue was built, and the
//      same letter on both builds is the measurement
//   w  THE IDLE HOOK AGAINST THE TICK'S EDGE, PLACED TO THE CYCLE: with
//      the STK running as the timebase, its CNTL written so the compare
//      comes D cycles later, D = 0..599, then the kernel's own shape - a
//      masked check, idle(), until one tick (and, a second pass, two) -
//      with the core's event latch set and clear by hand before each
//      try. The edge so walks across every instruction of the idle path
//      and of the loop around it, the sleep entry included, whatever the
//      code's layout. TIM2 at 1 MHz is the clock of each try and its
//      RESCUE: a try that takes its 50 ms period is a wake idle() lost
//      for good, and one that ends a whole tick period late is a wake
//      lost until the next tick (the bound: 250 us for one tick, 1250 us
//      for two). The writes to CNTL shorten the ticks they land in, so
//      kernel time runs fast during the letter. Prints the cycles from
//      the edge to the handler and to the caller's loop for an edge that
//      finds the core asleep, and what an idle() costs when the latch
//      makes it return at once
//   o  THE KERNEL'S TURN: a Tenuto pack of three quiet AOs, two with a
//      periodic time event, turned as Tenuto::run() turns it over 100
//      ticks with the tick the only interrupt - one turn per tick when
//      idle() consumes the event latch its wake left, two when it does
//      not (judged: 100 to 102, one turn of allowance at each end of the
//      window) - and what one quiet turn costs when the latch makes its
//      idle() return at once: the turn an interrupt would cost twice
//      over. (Not `k`: that letter is the V4F's interrupt storm.)
//
//   i  (by name only) THREE REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session a tool can judge from a
//      single capture. Run it with
//          brio run <board> i --app test_vx03_platform --expect="->" --timeout 60
//      Legs: a software reset (SFTRSTF at the next boot), a panic
//      through ResetReporter (the breadcrumb read back), and a
//      deliberate fault through ebreak with no debugger attached (the
//      fault vector's own record, kernel_fault, carrying the core's
//      cause byte). BOTH trap entries of this table are bound - the
//      exception vector at index 3 and the breakpoint one at index 9 -
//      and each leaves its own index in the token, which is how the
//      letter says WHICH of them the silicon took.
//   t  the runtime's seven functions (rt/rt.cpp) over every alignment,
//      length and overlap: rt/selftest.hpp's cases, the host suite's own,
//      against the symbols this image links - in `z`
//
// build: boards = v203c6,v203c8,v303vc
// build: groups = abcdefgitw,mno
// (on the 32 KB parts the suite is two images, test_vx03_platform-1 and
// -2: the whole is over the CH32V203C6's 28 KB of image; `i` runs as
// `--app test_vx03_platform-1` there)
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <utility>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/reset.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "rt/selftest.hpp"

using P = brio::Ch32vx03Platform<>;
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
    // The pending tick is taken some instructions AFTER the csrsi that
    // unmasks it, not before the next one (measured: read at once, the
    // count has not moved yet), so wait for it - bounded - and say how long.
    const uint32_t unmasked = cycles_now();
    while (Ticker::ticks() == t0 && cycles_between(unmasked, cycles_now()) < 1000u) {
    }
    const uint32_t settle = cycles_between(unmasked, cycles_now());
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  5 ms with interrupts masked advanced the tick by ",
          through_mask, " ms, the tick in within ", settle, " cycles of the unmask", crlf);
    bench.verdict("a masked window LOSES ticks (the STK interrupt is a "
                  "pending bit: one tick is delivered, the rest coalesce)",
                  through_mask >= 1u && through_mask <= 5u);

    // idle(): unmask, then a WFE on this core. Any enabled interrupt
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
    print(serial, "  built with the hardware prologue/epilogue (CH32VX03_HPE=ON)", crlf);
#else
    print(serial, "  built with gcc's own prologue/epilogue (CH32VX03_HPE=OFF)", crlf);
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

// ---------------------------------------------------------------------------
// j, k, l - the floating-point unit (a part with one)
// ---------------------------------------------------------------------------
// mstatus as the crt left it, sampled first thing in main().
uint32_t boot_mstatus = 0;

inline constexpr uint32_t mstatus_fs_mask = 3UL << 13;
uint32_t read_mstatus() {
    uint32_t v;
    __asm__ volatile("csrr %0, mstatus" : "=r"(v));
    return v;
}
uint32_t fs_of(uint32_t mstatus) { return (mstatus & mstatus_fs_mask) >> 13; }
const char* fs_name(uint32_t fs) {
    return fs == 0u ? "Off" : fs == 1u ? "Initial" : fs == 2u ? "Clean" : "Dirty";
}

/// The storm's state, shared with TIM6's handler. While it is on, the
/// handler does float work on TIM6's update instead of letter l's.
struct Storm {
    volatile bool on = false;
    volatile uint32_t hits = 0;
    volatile uint32_t bad = 0;
    volatile uint32_t limit = 0;
    volatile float sink = 0.0f;
};
Storm storm;

/// The handler's inputs, read through volatile so the compiler cannot
/// fold them into constants; the expected sum is exact in float.
volatile float storm_in[20] = {1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,
                               8.0f,  9.0f,  10.0f, 11.0f, 12.0f, 13.0f, 14.0f,
                               15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f};
inline constexpr float storm_expected = 2870.0f;   // sum of i*i, i = 1..20

/// The storm handler's body: twenty float locals, each squared and
/// summed, and the result checked - TIM6's update flag cleared first.
/// Inline into the vector binding's body, under the float trampoline.
template <bool fpu = device::has_fpu, uint8_t N = 6>
[[gnu::always_inline]] inline bool storm_body() {
    if constexpr (fpu && tim_present(N)) {
        if (!storm.on) {
            return false;
        }
        Tim<N>::clear_flags(Tim<N>::update_flag);
        float x0 = storm_in[0], x1 = storm_in[1], x2 = storm_in[2], x3 = storm_in[3];
        float x4 = storm_in[4], x5 = storm_in[5], x6 = storm_in[6], x7 = storm_in[7];
        float x8 = storm_in[8], x9 = storm_in[9], x10 = storm_in[10], x11 = storm_in[11];
        float x12 = storm_in[12], x13 = storm_in[13], x14 = storm_in[14], x15 = storm_in[15];
        float x16 = storm_in[16], x17 = storm_in[17], x18 = storm_in[18], x19 = storm_in[19];
        const float sum = ((x0 * x0 + x1 * x1) + (x2 * x2 + x3 * x3)) +
                          ((x4 * x4 + x5 * x5) + (x6 * x6 + x7 * x7)) +
                          ((x8 * x8 + x9 * x9) + (x10 * x10 + x11 * x11)) +
                          ((x12 * x12 + x13 * x13) + (x14 * x14 + x15 * x15)) +
                          ((x16 * x16 + x17 * x17) + (x18 * x18 + x19 * x19));
        storm.sink = sum;
        if (sum != storm_expected) {
            storm.bad = storm.bad + 1u;
        }
        const uint32_t h = storm.hits + 1u;
        storm.hits = h;
        if (h >= storm.limit) {
            // The storm ends itself: a loop that never ran again would
            // otherwise never end the letter.
            Tim<N>::interrupts(Tim<N>::update_interrupt, false);
            Tim<N>::enable(false);
            storm.on = false;
        }
        return true;
    } else {
        return false;
    }
}

/// Letter l's three handlers' shared state: the cycle counter at the
/// first and the last statement, and a hit count.
struct Trip {
    volatile uint32_t entry = 0;
    volatile uint32_t exit = 0;
    volatile uint32_t hits = 0;
};
Trip trip_fp;
Trip trip_call;
volatile float trip_in[4] = {1.5f, 2.25f, 3.0f, 0.5f};
volatile float trip_out = 0.0f;
volatile uint32_t callee_hits = 0;

/// What the calling handler calls: a function the compiler must assume
/// clobbers every caller-saved register, the twenty f-registers among
/// them under ilp32f.
[[gnu::noinline]] void trip_callee() { callee_hits = callee_hits + 1u; }

template <bool fpu = device::has_fpu>
[[gnu::always_inline]] inline void trip_fp_body() {
    if constexpr (fpu) {
        trip_fp.entry = stk()->CNTL;
        Pfic::clear_pending(Irq::tim6);
        const float a = trip_in[0], b = trip_in[1], c = trip_in[2], d = trip_in[3];
        trip_out = a * b + c * d;
        trip_fp.hits = trip_fp.hits + 1u;
        trip_fp.exit = stk()->CNTL;
    }
}

template <bool fpu = device::has_fpu>
[[gnu::always_inline]] inline void trip_call_body() {
    if constexpr (fpu) {
        trip_call.entry = stk()->CNTL;
        Pfic::clear_pending(Irq::tim7);
        trip_callee();
        trip_call.hits = trip_call.hits + 1u;
        trip_call.exit = stk()->CNTL;
    }
}

template <bool fpu = device::has_fpu>
void tj_fpu_state() {
    if constexpr (fpu) {
        const uint32_t boot_fs = fs_of(boot_mstatus);
        const uint32_t found_fs = fs_of(read_mstatus());
        print(serial, "  mstatus at main(): ", hex(boot_mstatus), " FS=", boot_fs, " (",
              fs_name(boot_fs), ")", crlf);
        print(serial, "  mstatus FS as this letter finds it: ", found_fs, " (",
              fs_name(found_fs), ")", crlf);
        bench.verdict("the crt left the unit on (FS is not Off, or the first float "
                      "instruction would have trapped)", boot_fs != 0u);

        // What the crt's own last FP step does: FS back to Initial by
        // hand, then fcsr written and nothing else. Masked, so that no
        // interrupt comes between the two steps.
        uint32_t fs_csr = 0;
        {
            P::CriticalSection cs;
            __asm__ volatile("csrc mstatus, %0" ::"r"(mstatus_fs_mask) : "memory");
            __asm__ volatile("csrs mstatus, %0" ::"r"(1UL << 13) : "memory");
            __asm__ volatile("csrw fcsr, zero" ::: "memory");
            fs_csr = fs_of(read_mstatus());
        }
        print(serial, "  FS set to 1 (Initial), then fcsr written alone: FS = ", fs_csr, " (",
              fs_name(fs_csr), ")", crlf);
        bench.verdict("a write to fcsr alone moves FS to Dirty - the crt's own last step, "
                      "which is why main() finds the unit Dirty and not Initial",
                      fs_csr == 3u && boot_fs == 3u);

        // Back to Initial by hand, then one fused multiply-add.
        uint32_t fs_before = 0, fs_after = 0;
        float fma_result = 0.0f;
        {
            P::CriticalSection cs;
            __asm__ volatile("csrc mstatus, %0" ::"r"(mstatus_fs_mask) : "memory");
            __asm__ volatile("csrs mstatus, %0" ::"r"(1UL << 13) : "memory");
            fs_before = fs_of(read_mstatus());
            const float a = trip_in[0], b = trip_in[1], c = trip_in[2];
            float r;
            __asm__ volatile("fmadd.s %0, %1, %2, %3" : "=f"(r) : "f"(a), "f"(b), "f"(c));
            fma_result = r;
            fs_after = fs_of(read_mstatus());
        }
        // Printed as thousandths: an integer the verdict can compare,
        // and the letter asks nothing of the float formatter.
        print(serial, "  FS set to ", fs_before, " (", fs_name(fs_before),
              "), then fmadd.s 1.5*2.25+3.0 = ",
              static_cast<int32_t>(fma_result * 1000.0f), "/1000, then FS = ", fs_after, " (",
              fs_name(fs_after), ")", crlf);
        bench.verdict("FS takes Initial by hand", fs_before == 1u);
        bench.verdict("one FP instruction moves it to Dirty (FS = 11)", fs_after == 3u);
        bench.verdict("and the fused multiply-add is exact here (6.375)", fma_result == 6.375f);

        // The sticky flags: division by zero, then an inexact quotient.
        uint32_t flags_dz = 0, flags_nx = 0, frm_back = 0;
        float q_rne = 0.0f, q_rtz = 0.0f;
        {
            P::CriticalSection cs;
            __asm__ volatile("csrw fflags, zero" ::: "memory");
            volatile float one = 1.0f;
            volatile float zero = 0.0f;
            const float inf = one / zero;
            __asm__ volatile("csrr %0, fflags" : "=r"(flags_dz) : "r"(inf) : "memory");
            __asm__ volatile("csrw fflags, zero" ::: "memory");
            volatile float three = 3.0f;
            q_rne = one / three;
            __asm__ volatile("csrr %0, fflags" : "=r"(flags_nx) : "f"(q_rne) : "memory");
            // The rounding mode: toward zero, read back, the same
            // quotient taken again, and the default restored.
            __asm__ volatile("csrw frm, %0" ::"r"(1u) : "memory");
            __asm__ volatile("csrr %0, frm" : "=r"(frm_back) :: "memory");
            q_rtz = one / three;
            // The quotient must be taken BEFORE the mode is restored,
            // and nothing but a use orders a division against a CSR.
            __asm__ volatile("" ::"f"(q_rtz));
            __asm__ volatile("csrw frm, zero" ::: "memory");
            __asm__ volatile("csrw fflags, zero" ::: "memory");
        }
        uint32_t bits_rne, bits_rtz;
        __builtin_memcpy(&bits_rne, &q_rne, 4);
        __builtin_memcpy(&bits_rtz, &q_rtz, 4);
        print(serial, "  1/0 -> fflags ", hex(flags_dz), " (DZ is bit 3), 1/3 -> fflags ",
              hex(flags_nx), " (NX is bit 0)", crlf);
        print(serial, "  frm written 1 (RTZ) reads back ", frm_back, "; 1/3 is ", hex(bits_rne),
              " to nearest and ", hex(bits_rtz), " toward zero", crlf);
        bench.verdict("a division by zero raises DZ and nothing else, and does not trap",
                      flags_dz == 0x08u);
        bench.verdict("an inexact quotient raises NX alone", flags_nx == 0x01u);
        bench.verdict("the rounding mode takes a write and reads it back", frm_back == 1u);
        bench.verdict("and the mode is really used: the two quotients differ in the last "
                      "bit, rounded up and truncated",
                      bits_rne == 0x3EAAAAABu && bits_rtz == 0x3EAAAAAAu);
    }
}

template <bool fpu = device::has_fpu, uint8_t N = 6>
void tk_fpu_storm() {
    if constexpr (fpu && tim_present(N)) {
        using StormTimer = Tim<N>;
        // Twenty accumulators, their starting values and the step read
        // through volatile once, so the compiler keeps them in
        // f-registers and folds nothing.
        volatile float start[20] = {0.0f,  1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,
                                    7.0f,  8.0f,  9.0f,  10.0f, 11.0f, 12.0f, 13.0f,
                                    14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f};
        volatile float vstep = 1.0f;
        volatile uint32_t vn = 200'000u;
        const uint32_t n = vn;
        const float step = vstep;
        float a0 = start[0], a1 = start[1], a2 = start[2], a3 = start[3], a4 = start[4];
        float a5 = start[5], a6 = start[6], a7 = start[7], a8 = start[8], a9 = start[9];
        float a10 = start[10], a11 = start[11], a12 = start[12], a13 = start[13],
              a14 = start[14];
        float a15 = start[15], a16 = start[16], a17 = start[17], a18 = start[18],
              a19 = start[19];

        // The storm: TIM6 counting at the core's clock, its update every
        // `period` cycles raising a handler that does the float work above
        // and ends itself after `limit` interrupts. The kernel's tick goes
        // on beside it, a vector of its own.
        const uint32_t period = 400;
        static_assert(StormTimer::clock_hz(clock) == SysClock::hz,
                      "the storm's period is counted in core cycles");
        console_drain();
        StormTimer::init();
        (void)StormTimer::configure({.prescaler = 0, .period = period - 1u});
        StormTimer::clear_flags(StormTimer::all_flags);
        Pfic::clear_pending(StormTimer::irq());
        Pfic::enable(StormTimer::irq());
        {
            P::CriticalSection cs;
            storm.hits = 0;
            storm.bad = 0;
            storm.limit = 2'000'000u;
            storm.on = true;
            StormTimer::interrupts(StormTimer::update_interrupt, true);
            StormTimer::enable(true);
        }
        for (uint32_t i = 0; i < n; ++i) {
            a0 += step; a1 += step; a2 += step; a3 += step; a4 += step;
            a5 += step; a6 += step; a7 += step; a8 += step; a9 += step;
            a10 += step; a11 += step; a12 += step; a13 += step; a14 += step;
            a15 += step; a16 += step; a17 += step; a18 += step; a19 += step;
        }
        const bool still_on = storm.on;
        uint32_t hits = 0;
        {
            P::CriticalSection cs;
            StormTimer::interrupts(StormTimer::update_interrupt, false);
            StormTimer::enable(false);
            storm.on = false;
            hits = storm.hits;
        }
        Pfic::disable(StormTimer::irq());
        Pfic::clear_pending(StormTimer::irq());
        const uint32_t spent_ms =
            static_cast<uint32_t>((static_cast<uint64_t>(hits) * period) / (SysClock::hz / 1000u));

        const float acc[20] = {a0, a1, a2, a3, a4, a5, a6, a7, a8, a9,
                               a10, a11, a12, a13, a14, a15, a16, a17, a18, a19};
        uint8_t rotted = 0;
        for (uint8_t i = 0; i < 20u; ++i) {
            const float want = static_cast<float>(n + i);
            if (acc[i] != want) {
                uint32_t got_bits, want_bits;
                __builtin_memcpy(&got_bits, &acc[i], 4);
                __builtin_memcpy(&want_bits, &want, 4);
                print(serial, "  accumulator ", i, " is ", hex(got_bits), " where ",
                      hex(want_bits), " was due: the bits ", hex(got_bits ^ want_bits),
                      " rotted", crlf);
                ++rotted;
            }
        }
        print(serial, "  ", n, " iterations of twenty float adds under a storm of ", hits,
              " interrupts, one every ", period, " cycles (", spent_ms, " ms)", crlf);
        print(serial, "  the handler found its own twenty-local sum wrong ", storm.bad,
              " times", crlf);
        bench.verdict("the storm ran for the whole loop and ended with it",
                      still_on && hits > 1000u);
        bench.verdict("every one of the twenty accumulators is exact after the loop",
                      rotted == 0u);
        bench.verdict("and every handler computed its own sum exactly", storm.bad == 0u);
    }
}

/// Letter e's measurement for one line and one handler state.
struct TripResult {
    uint32_t seen = 0;
    uint32_t best_entry = 0xFFFF'FFFFu;
    uint32_t best_body = 0xFFFF'FFFFu;
    uint32_t best_trip = 0xFFFF'FFFFu;
};

TripResult measure_trip(Irq line, Trip& t) {
    TripResult r;
    Pfic::enable(line);
    for (uint32_t round = 0; round < 32u; ++round) {
        console_drain();
        const uint32_t before = t.hits;
        const uint32_t tick = Ticker::ticks();
        while (Ticker::ticks() == tick) {
        }
        const uint32_t c0 = cycles_now();
        Pfic::set_pending(line);
        uint32_t spin = 0;
        while (t.hits == before && spin < 100'000u) {
            ++spin;
        }
        if (t.hits == before) {
            break;
        }
        const uint32_t c1 = cycles_now();
        ++r.seen;
        const uint32_t entry = cycles_between(c0, t.entry);
        const uint32_t body = cycles_between(t.entry, t.exit);
        const uint32_t whole = cycles_between(c0, c1);
        if (entry < r.best_entry) { r.best_entry = entry; }
        if (body < r.best_body) { r.best_body = body; }
        if (whole < r.best_trip) { r.best_trip = whole; }
    }
    Pfic::disable(line);
    return r;
}

template <bool fpu = device::has_fpu>
void tl_fpu_trip() {
    if constexpr (fpu) {
        // The first handler is letter e's own: the software interrupt,
        // which touches no f-register.
        TripResult r0;
        Pfic::enable(Irq::software);
        for (uint32_t round = 0; round < 32u; ++round) {
            console_drain();
            const uint32_t before = sw_hits;
            const uint32_t tick = Ticker::ticks();
            while (Ticker::ticks() == tick) {
            }
            const uint32_t c0 = cycles_now();
            Pfic::set_pending(Irq::software);
            uint32_t spin = 0;
            while (sw_hits == before && spin < 100'000u) {
                ++spin;
            }
            if (sw_hits == before) {
                break;
            }
            const uint32_t c1 = cycles_now();
            ++r0.seen;
            const uint32_t entry = cycles_between(c0, sw_entry_cycles);
            const uint32_t body = cycles_between(sw_entry_cycles, sw_exit_cycles);
            const uint32_t whole = cycles_between(c0, c1);
            if (entry < r0.best_entry) { r0.best_entry = entry; }
            if (body < r0.best_body) { r0.best_body = body; }
            if (whole < r0.best_trip) { r0.best_trip = whole; }
        }
        Pfic::disable(Irq::software);

        const TripResult r1 = measure_trip(irq_present<Irq::tim6>(), trip_fp);
        const TripResult r2 = measure_trip(irq_present<Irq::tim7>(), trip_call);

        print(serial, "  no FP register    : entry ", r0.best_entry, ", body ", r0.best_body,
              ", round trip ", r0.best_trip, " cycles (", r0.seen, "/32 taken)", crlf);
        print(serial, "  float arithmetic  : entry ", r1.best_entry, ", body ", r1.best_body,
              ", round trip ", r1.best_trip, " cycles (", r1.seen, "/32 taken)", crlf);
        print(serial, "  calls a function  : entry ", r2.best_entry, ", body ", r2.best_body,
              ", round trip ", r2.best_trip, " cycles (", r2.seen, "/32 taken)", crlf);
        print(serial, "  the float result the second handler left: ",
              static_cast<int32_t>(trip_out * 1000.0f), "/1000, the callee's count: ",
              callee_hits, crlf);
        bench.verdict("every raise of the three lines was taken",
                      r0.seen == 32u && r1.seen == 32u && r2.seen == 32u);
        bench.verdict("the float handler's arithmetic is right (1.5*2.25 + 3*0.5 = 4.875)",
                      trip_out == 4.875f);
        bench.verdict("the float handler costs the most (its trampoline's twenty saves), and "
                      "one that calls out no less than one that does neither",
                      r1.best_trip > r2.best_trip && r2.best_trip >= r0.best_trip);
    }
}

// ---------------------------------------------------------------------------
// m - the bus in sleep
// ---------------------------------------------------------------------------
volatile uint32_t dma_src_word = 0xA5A5'5A5Au;
volatile uint32_t dma_dst_word = 0;
using SleepDma = DmaChannel<1, 1>;

DmaTransfer sleep_block() {
    return DmaTransfer{.peripheral = &dma_src_word,
                       .memory = &dma_dst_word,
                       .count = 65535,
                       .config = {.direction = DmaDirection::peripheral_to_memory,
                                  .memory_to_memory = true,
                                  .peripheral_increment = false,
                                  .memory_increment = false,
                                  .peripheral_width = DmaWidth::word,
                                  .memory_width = DmaWidth::word,
                                  .priority = DmaPriority::very_high}};
}

void tm_bus_in_sleep() {
    const uint32_t per_ms = SysClock::hz / 1000u;
    const uint32_t span = 2u * per_ms;   // the one-shot wake, in cycles
    const uint32_t saved_cmp = stk()->CMPLR;

    // AWAKE: the same block over the same span, the core spinning.
    SleepDma::stop();
    (void)SleepDma::prepare(sleep_block());
    console_drain();
    uint16_t awake_left = 0;
    {
        P::CriticalSection cs;
        (void)SleepDma::trigger();
        const uint32_t c0 = cycles_now();
        uint32_t spun = 0, last = c0;
        while (spun < span) {
            const uint32_t now = cycles_now();
            spun += cycles_between(last, now);
            last = now;
        }
        awake_left = SleepDma::remaining();
    }
    SleepDma::stop();
    const uint32_t moved_awake = 65535u - awake_left;

    // The mechanism: over a working master idle() returns at once.
    (void)SleepDma::prepare(sleep_block());
    console_drain();
    uint32_t returned_in = 0;
    uint8_t masters = 0;
    {
        P::CriticalSection cs;
        (void)SleepDma::trigger();
        masters = P::bus_masters_active();
        const uint32_t c0 = cycles_now();
        P::idle();
        returned_in = cycles_between(c0, cycles_now());
    }
    SleepDma::stop();

    // ASLEEP: the count set aside so the core really sleeps, the core
    // counter as a one-shot wake one span away, nothing else armed.
    (void)SleepDma::prepare(sleep_block());
    console_drain();
    uint16_t at_sleep = 0, at_wake = 0;
    uint32_t slept = 0;
    uint8_t calls = 0;
    const uint32_t ticks0 = Ticker::ticks();
    {
        P::CriticalSection cs;
        stk()->CTLR = 0;
        stk()->SR = 0;
        stk()->CNTL = 0;
        stk()->CMPLR = span - 1u;
        stk()->CTLR = stk_ste | stk_stie | stk_stclk | stk_stre;
        Pfic::clear_pending(Irq::systick);
        (void)SleepDma::trigger();
        at_sleep = SleepDma::remaining();
        BusActivity::left();
        const uint32_t c0 = cycles_now();
        // As the kernel calls it - masked, and again until the one wake
        // this letter armed has been served: a WFE returns at once on an
        // event latched before it, which is the lost-wakeup guard.
        while (Ticker::ticks() == ticks0 && calls < 20u) {
            {
                P::CriticalSection inner;
                P::idle();
            }
            ++calls;
        }
        at_wake = SleepDma::remaining();
        // The span is a whole period of the one-shot compare per tick
        // served plus the counter's phase: the counter alone, read modulo
        // the period it reloads at, would fold a sleep of exactly that
        // period into a few cycles.
        const uint32_t c1 = cycles_now();
        slept = (Ticker::ticks() - ticks0) * span + c1 - c0;
        BusActivity::entered();
    }
    const uint32_t woke_ticks = Ticker::ticks() - ticks0;
    SleepDma::stop();
    {
        P::CriticalSection cs;
        stk()->CTLR = 0;
        stk()->SR = 0;
        stk()->CNTL = 0;
        stk()->CMPLR = saved_cmp;
        stk()->CTLR = stk_ste | stk_stie | stk_stclk | stk_stre;
    }
    Ticker::advance(1);   // the second millisecond of the span
    const uint32_t moved_asleep = static_cast<uint32_t>(at_sleep) - at_wake;

    print(serial, "  awake: ", moved_awake, " words moved in ", span / per_ms,
          " ms of spinning (", span / (moved_awake != 0u ? moved_awake : 1u),
          " cycles a word)", crlf);
    print(serial, "  over a working master idle() returned in ", returned_in,
          " cycles with ", masters, " master(s) counted", crlf);
    print(serial, "  asleep: ", moved_asleep, " words moved across ", slept / per_ms, ".",
          (slept % per_ms) * 10u / per_ms, " ms of idle() in ", calls, " call(s) (",
          65535u - at_sleep, " already moved at the sleep, ", woke_ticks,
          " tick(s) served)", crlf);
    const bool moves = moved_asleep * 2u > moved_awake;
    print(serial, "  FINDING: in Sleep the DMA ", moves ? "KEEPS MOVING" : "STALLS",
          " - ", moved_asleep, " words asleep against ", moved_awake, " awake", crlf);
    bench.verdict("the block ran awake (the channel and its count work)",
                  moved_awake > 1000u);
    bench.verdict("over a working master the platform's idle() returns at once",
                  masters == 1u && returned_in < per_ms / 10u);
    bench.verdict("with the count set aside idle() slept until the one-shot wake",
                  slept >= span / 2u && woke_ticks == 1u && calls <= 2u);
    bench.verdict(moves ? "the bus serves the DMA while the core sleeps"
                        : "the bus serves the core alone in Sleep (no master moves)",
                  true);
}

// ---------------------------------------------------------------------------
// n - the mask's shadow
// ---------------------------------------------------------------------------

/// The line the trials raise: EXTI2's vector, pended by hand at the PFIC
/// with no EXTI activity at all - a line no other letter of this suite
/// uses, on every device class.
constexpr Irq shadow_line = Irq::exti2;

struct ShadowHit {
    volatile uint32_t hits = 0;
    volatile uint32_t mepc = 0;
    volatile uint32_t mstatus = 0;
};
ShadowHit shadow;

/// What one trial leaves: where the mask and the unmask instructions sit.
struct ShadowMarks {
    uint32_t mask;
    uint32_t unmask;
};

/// How the program takes the line away: the platform guard's own csrrci
/// on mstatus.MIE, or a store into the line's PFIC_IRER bit - each alone
/// and each followed by a fence.i, which is what RM V2.5 asks for.
enum class MaskWay : uint8_t { csr, csr_fence, irer, irer_fence };

/// One trial: the line pended by a store into IPSR, N nops, the mask,
/// eight nops inside the masked region, the unmask, eight nops more. The
/// addresses of the mask and the unmask come back so the handler's mepc
/// can be placed against them.
template <unsigned N, MaskWay W>
[[gnu::noinline]] ShadowMarks shadow_trial(volatile uint32_t* ipsr, volatile uint32_t* irer,
                                           volatile uint32_t* ienr, uint32_t bit) {
    uint32_t a = 0;
    uint32_t c = 0;
    uint32_t junk = 0;
    if constexpr (W == MaskWay::csr || W == MaskWay::csr_fence) {
        __asm__ volatile(
            "la   %[a], 1f\n"
            "la   %[c], 3f\n"
            "sw   %[bit], 0(%[ipsr])\n"
            ".rept %[n]\n"
            "nop\n"
            ".endr\n"
            "1: csrrci %[j], mstatus, 8\n"
            ".if %[f]\n"
            "fence.i\n"
            ".endif\n"
            ".rept 8\n"
            "nop\n"
            ".endr\n"
            "3: csrsi mstatus, 8\n"
            ".rept 8\n"
            "nop\n"
            ".endr\n"
            : [a] "=&r"(a), [c] "=&r"(c), [j] "=&r"(junk)
            : [bit] "r"(bit), [ipsr] "r"(ipsr), [n] "i"(N),
              [f] "i"(W == MaskWay::csr_fence ? 1 : 0)
            : "memory");
    } else {
        __asm__ volatile(
            "la   %[a], 1f\n"
            "la   %[c], 3f\n"
            "sw   %[bit], 0(%[ipsr])\n"
            ".rept %[n]\n"
            "nop\n"
            ".endr\n"
            "1: sw %[bit], 0(%[irer])\n"
            ".if %[f]\n"
            "fence.i\n"
            ".endif\n"
            ".rept 8\n"
            "nop\n"
            ".endr\n"
            "3: sw %[bit], 0(%[ienr])\n"
            ".rept 8\n"
            "nop\n"
            ".endr\n"
            : [a] "=&r"(a), [c] "=&r"(c)
            : [bit] "r"(bit), [ipsr] "r"(ipsr), [irer] "r"(irer), [ienr] "r"(ienr),
              [n] "i"(N), [f] "i"(W == MaskWay::irer_fence ? 1 : 0)
            : "memory");
    }
    (void)junk;
    return {a, c};
}

/// Where a variant's trials landed.
struct ShadowTally {
    uint32_t before = 0;     ///< taken before the mask instruction ran
    uint32_t inside = 0;     ///< taken INSIDE the masked region: the shadow
    uint32_t after = 0;      ///< taken after the unmask
    uint32_t lost = 0;       ///< never taken
    uint32_t inside_mpie0 = 0;   ///< shadow hits whose MPIE says MIE was already clear
    uint32_t first_n = 0xFFFFu;  ///< the smallest nop count that gave a shadow
    uint32_t last_n = 0;
    uint32_t min_depth = 0xFFFFu;   ///< bytes past the mask, the earliest shadow hit
    uint32_t max_depth = 0;         ///< ... and the latest
    uint32_t pend_n = 0xFFFFu;      ///< the smallest nop count taken before the mask
    uint32_t after_min = 0xFFFFu;   ///< bytes past the unmask, the earliest late hit
    uint32_t after_max = 0;         ///< ... and the latest
};

constexpr uint32_t shadow_reps = 250;

using ShadowTrial = ShadowMarks (*)(volatile uint32_t*, volatile uint32_t*, volatile uint32_t*,
                                    uint32_t);

/// One nop count's trials, the classification written once for all of
/// them - the trials are the only thing that has to be a template.
[[gnu::noinline]] void shadow_run(ShadowTrial trial, uint32_t n, ShadowTally& t) {
    const uint32_t line = static_cast<uint32_t>(shadow_line);
    volatile uint32_t* ipsr = &pfic()->IPSR[line >> 5];
    volatile uint32_t* irer = &pfic()->IRER[line >> 5];
    volatile uint32_t* ienr = &pfic()->IENR[line >> 5];
    const uint32_t bit = 1UL << (line & 31u);
    for (uint32_t r = 0; r < shadow_reps; ++r) {
        const uint32_t before = shadow.hits;
        const ShadowMarks m = trial(ipsr, irer, ienr, bit);
        for (uint32_t spin = 0; spin < 1000u && shadow.hits == before; ++spin) {
        }
        if (shadow.hits == before) {
            ++t.lost;
            Pfic::clear_pending(shadow_line);
            continue;
        }
        const uint32_t pc = shadow.mepc;
        if (pc <= m.mask) {
            ++t.before;
            if (n < t.pend_n) {
                t.pend_n = n;
            }
        } else if (pc <= m.unmask) {
            ++t.inside;
            if (((shadow.mstatus >> 7) & 1u) == 0u) {
                ++t.inside_mpie0;
            }
            if (n < t.first_n) {
                t.first_n = n;
            }
            if (n > t.last_n) {
                t.last_n = n;
            }
            if (pc - m.mask < t.min_depth) {
                t.min_depth = pc - m.mask;
            }
            if (pc - m.mask > t.max_depth) {
                t.max_depth = pc - m.mask;
            }
        } else {
            ++t.after;
            if (pc - m.unmask < t.after_min) {
                t.after_min = pc - m.unmask;
            }
            if (pc - m.unmask > t.after_max) {
                t.after_max = pc - m.unmask;
            }
        }
    }
}

template <MaskWay W, unsigned... N>
ShadowTally shadow_sweep(std::integer_sequence<unsigned, N...>) {
    static constexpr ShadowTrial trials[] = {&shadow_trial<N, W>...};
    ShadowTally t;
    for (uint32_t n = 0; n < sizeof...(N); ++n) {
        shadow_run(trials[n], n, t);
    }
    return t;
}

constexpr unsigned shadow_nops = 20;

void report_shadow(const char* name, const ShadowTally& t) {
    const uint32_t all = t.before + t.inside + t.after + t.lost;
    print(serial, "  ", name, ": ", all, " trials - ", t.before, " taken before the mask, ",
          t.inside, " INSIDE it (", t.inside * 1000u / (all == 0u ? 1u : all),
          " per thousand), ", t.after, " after the unmask, ", t.lost, " never", crlf);
    print(serial, "    the pend was taken ahead of the mask from ", t.pend_n,
          " nops of lead on", crlf);
    if (t.inside != 0u) {
        print(serial, "    the shadow: at ", t.first_n, "..", t.last_n, " nops of lead, ",
              t.min_depth, "..", t.max_depth, " bytes past the mask, ", t.inside_mpie0,
              " of them with MPIE clear (MIE already cleared when the trap came)", crlf);
    }
    if (t.after != 0u) {
        print(serial, "    held until the unmask, then taken ", t.after_min, "..", t.after_max,
              " bytes past it", crlf);
    }
}

void tn_shadow() {
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
    print(serial, "  built with the hardware prologue/epilogue (CH32VX03_HPE=ON)", crlf);
#else
    print(serial, "  built with gcc's own prologue/epilogue (CH32VX03_HPE=OFF)", crlf);
#endif
    console_drain();
    Pfic::clear_pending(shadow_line);
    Pfic::enable(shadow_line);
    const auto seq = std::make_integer_sequence<unsigned, shadow_nops>{};
    const ShadowTally csr = shadow_sweep<MaskWay::csr>(seq);
    const ShadowTally csr_fence = shadow_sweep<MaskWay::csr_fence>(seq);
    const ShadowTally irer = shadow_sweep<MaskWay::irer>(seq);
    const ShadowTally irer_fence = shadow_sweep<MaskWay::irer_fence>(seq);
    Pfic::disable(shadow_line);
    Pfic::clear_pending(shadow_line);
    report_shadow("csrrci on mstatus.MIE (the guard's own)", csr);
    report_shadow("csrrci then fence.i", csr_fence);
    report_shadow("a store into PFIC_IRER", irer);
    report_shadow("a store into PFIC_IRER then fence.i", irer_fence);
    const uint32_t each = shadow_reps * shadow_nops;
    const auto whole = [each](const ShadowTally& t) {
        return t.before + t.inside + t.after == each && t.lost == 0u;
    };
    bench.verdict("every trial's interrupt was taken once, before the mask, inside it or after "
                  "the unmask, in all four variants",
                  whole(csr) && whole(csr_fence) && whole(irer) && whole(irer_fence));
    const auto straddles = [](const ShadowTally& t) {
        return t.before != 0u && t.inside + t.after != 0u;
    };
    bench.verdict("the sweep straddled the mask: in every variant some trials were taken before "
                  "it and some were not",
                  straddles(csr) && straddles(csr_fence) && straddles(irer) &&
                      straddles(irer_fence));
    // The count inside is the finding, whichever it is: no threshold is
    // the right one to judge it by.
    bench.verdict("the shadow counted for each variant (the numbers above are the finding)", true);
}

// ---------------------------------------------------------------------------
// w - the idle hook against the tick's edge, placed to the cycle
// ---------------------------------------------------------------------------
using Rescue = Tim<2>;
volatile uint32_t rescue_periods = 0;

/// Letter w's witness in the STK handler: while armed, the handler's
/// first statement records the counter - the cycles since the compare.
volatile bool tick_probe = false;
volatile uint32_t tick_entry = 0;

/// Clear the core's event latch (SETEVENT, then a WFE it ends at once)
/// and set it again if `latched`: the two states an idle() can be
/// entered in. Masked; the read-back puts the store ahead of the wfi.
void event_latch(bool latched) {
    pfic_sctlr() = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend | sctlr_setevent) &
                   ~sctlr_sleeponexit;
    const uint32_t landed = pfic_sctlr();
    __asm__ volatile("wfi" ::"r"(landed) : "memory");
    if (latched) {
        pfic_sctlr() = pfic_sctlr() | sctlr_setevent;
        const uint32_t set = pfic_sctlr();
        __asm__ volatile("" ::"r"(set) : "memory");
    }
}

/// One edge position: the counter written so the compare comes `d`
/// cycles later, then the kernel's loop until `edges` ticks have been
/// served. Masked throughout but for what idle() unmasks.
struct EdgeTry {
    bool lost;           ///< the rescue ended it: a wake lost for good
    bool late;           ///< a whole tick period late: a wake lost until the next tick
    uint32_t took_us;    ///< the try's span on the rescue's 1 MHz count
    uint32_t to_loop;    ///< the counter after the loop: cycles since the edge
    uint32_t to_entry;   ///< the counter at the handler's first statement
};

EdgeTry edge_try(uint32_t d, uint32_t edges, bool latched) {
    EdgeTry r{};
    uint32_t took_us = 0;
    bool rescued = false;
    {
        P::CriticalSection cs;
        event_latch(latched);
        const uint32_t r0 = rescue_periods;
        Rescue::set_count(0);
        const uint32_t t = Ticker::ticks();
        stk()->CNTL = stk()->CMPLR - d;
        for (;;) {
            P::CriticalSection turn;   // the kernel's own shape
            if (Ticker::ticks() - t >= edges) {
                break;
            }
            P::idle();
        }
        r.to_loop = stk()->CNTL;
        took_us = Rescue::count();
        rescued = rescue_periods != r0;
        r.to_entry = tick_entry;
    }
    // A wake lost for good waits for the rescue; one lost until the NEXT
    // tick (a handler run after its edge's latch was consumed, say) ends a
    // whole period late. The edges asked for land within 5 us of the
    // period they need, so the bound sits a quarter period past that.
    r.took_us = took_us;
    r.lost = rescued;
    r.late = !rescued && took_us > edges * 1000u - 750u;
    return r;
}

void tw_edge() {
    Rescue::init();
    (void)Rescue::configure(
        TimConfig{.prescaler = static_cast<uint16_t>(Rescue::clock_hz(clock) / 1'000'000u - 1u),
                  .period = 49'999u});
    Rescue::clear_flags(Rescue::update_flag);
    Rescue::interrupts(Rescue::update_interrupt, true);
    Pfic::clear_pending(Rescue::irq());
    Pfic::enable(Rescue::irq());
    Rescue::enable(true);
    console_drain();
    tick_probe = true;

    constexpr uint32_t positions = 600;
    constexpr uint32_t shown = 6;
    uint32_t lost[2][2] = {{0, 0}, {0, 0}};
    uint32_t late[2][2] = {{0, 0}, {0, 0}};
    uint32_t slowest[2] = {0, 0};
    uint32_t where[2][2][shown] = {};
    EdgeTry asleep{};
    for (uint32_t edges = 1; edges <= 2u; ++edges) {
        for (uint32_t latched = 0; latched < 2u; ++latched) {
            for (uint32_t d = 0; d < positions; ++d) {
                const EdgeTry r = edge_try(d, edges, latched != 0u);
                if (r.took_us > slowest[edges - 1u]) {
                    slowest[edges - 1u] = r.took_us;
                }
                if (r.late) {
                    ++late[edges - 1u][latched];
                }
                if (r.lost || r.late) {
                    uint32_t& n = lost[edges - 1u][latched];
                    if (n < shown) {
                        where[edges - 1u][latched][n] = d;
                    }
                    ++n;
                }
                if (edges == 1u && latched == 0u && d == positions - 1u) {
                    asleep = r;
                }
            }
        }
    }
    tick_probe = false;

    // An idle() the latch makes return at once: its whole cost, the two
    // counter reads around it included.
    uint32_t at_once = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        P::CriticalSection cs;
        event_latch(true);
        const uint32_t c0 = cycles_now();
        P::idle();
        const uint32_t c1 = cycles_now();
        disable_interrupts();
        if (c1 > c0 && c1 - c0 < at_once) {
            at_once = c1 - c0;
        }
    }

    Rescue::enable(false);
    Rescue::interrupts(Rescue::update_interrupt, false);
    Pfic::disable(Rescue::irq());
    Pfic::clear_pending(Rescue::irq());
    Rescue::release();

    for (uint32_t e = 0; e < 2u; ++e) {
        for (uint32_t l = 0; l < 2u; ++l) {
            print(serial, "  waiting for ", e + 1u, " tick(s), latch ", l != 0u ? "set" : "clear",
                  ": ", positions, " edge positions, ", lost[e][l] - late[e][l],
                  " lost for good, ", late[e][l], " a tick late");
            for (uint32_t i = 0; i < lost[e][l] && i < shown; ++i) {
                print(serial, i == 0u ? " at D=" : ",", where[e][l][i]);
            }
            print(serial, crlf);
        }
    }
    print(serial, "  the slowest try: ", slowest[0], " us for one tick, ", slowest[1], " us for two",
          crlf);
    print(serial, "  an edge that finds the core asleep: ", asleep.to_entry,
          " cycles to the handler's first statement, ", asleep.to_loop,
          " to the caller's loop; an idle() the latch returns at once: ", at_once,
          " cycles (two counter reads included)", crlf);
    bench.verdict("no edge position loses the wake or serves it a tick late, waiting for one tick",
                  lost[0][0] + lost[0][1] == 0u);
    bench.verdict("nor waiting for two (the sleep after a tick was taken)",
                  lost[1][0] + lost[1][1] == 0u);
}

// ---------------------------------------------------------------------------
// o - the kernel's turn: how many per interrupt, and what one costs
// ---------------------------------------------------------------------------
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

void to_turns() {
    console_drain();
    QuietKernel::init_all();

    // Turns over 100 quiet ticks: the tick is the only interrupt (the
    // console drained, nothing else enabled). The window opens and closes
    // on a tick read, so a turn may straddle either end: 100 to 102.
    const uint32_t t = Ticker::ticks();
    uint32_t turns = 0;
    while (Ticker::ticks() - t < 100u) {
        kernel_turn();
        ++turns;
    }

    // A quiet turn whose idle() the event latch returns at once - the
    // turn an interrupt costs on top of the one that serves it when the
    // waking edge is left latched - the two counter reads included.
    uint32_t turn_cycles = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        {
            P::CriticalSection cs;
            event_latch(true);
        }
        const uint32_t c0 = cycles_now();
        kernel_turn();
        const uint32_t c1 = cycles_now();
        if (c1 > c0 && c1 - c0 < turn_cycles) {
            turn_cycles = c1 - c0;
        }
    }
    TimeEvents<P>::clear_all();

    print(serial, "  100 quiet ticks: ", turns, " kernel turns; a quiet turn whose idle() returns at once: ",
          turn_cycles, " cycles (two counter reads included)", crlf);
    bench.verdict("the kernel loop turns once per interrupt, not twice (the waking edge's latch "
                  "is consumed)",
                  turns >= 100u && turns <= 102u);
}

/// THE CH32V20x_D6 PORTC PROBE - a bench probe, outside the rule that
/// an app reads no register. WCH's GPIO library (ch32v20x_gpio.c in the
/// CH32V20x EVT) reads the word at 0x40022030 on a CH32V20x_D6 and, when
/// its bits 27:24 are zero, sets MCU_Version = 1 and shifts every PORTC
/// pin mask right by 13: on such a die PC13..PC15 answer on bits 0..2.
/// The reference manual names no register at that address (V2.3's table
/// 3-2 has HSE_CAL_CTRL at 0x4002202C and LSI32K_TUNE at 0x40022036, and
/// V2.5 the same), so the word is named here by that use. Printed and
/// never judged: what a die answers is a fact for the part table to
/// take, not this suite. Read only on the class the library reads it on.
void portc_probe() {
    if constexpr (device::device_class == DeviceClass::v20x_d6) {
        const uint32_t word = *reinterpret_cast<const volatile uint32_t*>(0x40022030UL);
        print(serial, "  the EVT's MCU_Version word at 0x40022030 = ", hex(word),
              ": PORTC shifted by 13 (bits 27:24 zero): ",
              (word & 0x0F000000UL) == 0u ? "yes" : "no", crlf);
    } else {
        print(serial, "  the EVT's MCU_Version word at 0x40022030: a CH32V20x_D6 "
                      "question, not read on this class", crlf);
    }
}

void banner() {
    print(serial, crlf, "test_vx03_platform - ", device::part_name,
          " (clk=144 MHz PLL, tick=STK 1000 Hz)", crlf);
    portc_probe();
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
/// The core counter: the kernel's tick, and letter w's witness.
BRIO_CH32_VECTOR(systick_handler) {
    if (tick_probe) {
        tick_entry = brio::stk()->CNTL;
    }
    brio::Ticker::tick();
}

/// Letter k's storm while it runs, else letter l's float handler raised
/// by hand: the one binding of this suite whose body does float work, so
/// the float trampoline. Empty on a part without the FPU, where no table
/// entry reaches it.
BRIO_CH32_VECTOR_FLOAT(tim6_handler) {
    if (!storm_body()) {
        trip_fp_body();
    }
}

/// Letter l's handler that calls out, raised by hand: a plain
/// trampoline, so the call costs no f-register.
BRIO_CH32_VECTOR(tim7_handler) { trip_call_body(); }

BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }

/// Letter w's rescue and clock: TIM2's update, counted.
BRIO_CH32_VECTOR(tim2_handler) {
    Rescue::clear_flags(Rescue::update_flag);
    rescue_periods = rescue_periods + 1u;
}

/// The software interrupt: the first thing it does is read the cycle
/// counter, which is letter e's whole measurement.
BRIO_CH32_VECTOR(software_handler) {
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
/// Letter n's line: where the trap came from (mepc) and what mstatus
/// said about it, then the pending bit withdrawn - a leaf, so the
/// prologue is the same one on every device class.
BRIO_CH32_VECTOR(exti2_handler) {
    uint32_t pc;
    uint32_t st;
    __asm__ volatile("csrr %0, mepc" : "=r"(pc));
    __asm__ volatile("csrr %0, mstatus" : "=r"(st));
    shadow.mepc = pc;
    shadow.mstatus = st;
    brio::Pfic::clear_pending(shadow_line);
    shadow.hits = shadow.hits + 1u;
}

BRIO_CH32_VECTOR(fault_handler) {
    token.vector = 3;
    brio::fault_reset<P>();
}

BRIO_CH32_VECTOR(breakpoint_handler) {
    token.vector = 9;
    brio::fault_reset<P>();
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
    // Sampled FIRST: the flags are not cleared by reading, but the panic
    // record is fetch-and-clear and must be taken exactly once.
    boot_flags = brio::Reset::flags();
    boot_record = brio::take_panic_record<P>();
    boot_mstatus = read_mstatus();

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
    if constexpr (brio::device::has_fpu) {
        bench.letter('j', "the FPU's state: FS, the sticky flags, the rounding mode",
                     tj_fpu_state<>);
        bench.letter('k', "floating point under an interrupt storm", tk_fpu_storm<>);
        bench.letter('l', "the round trip with floating point, three handlers", tl_fpu_trip<>);
    }
    if constexpr (brio::device::dma_controller_count >= 1u) {
        bench.letter('m', "the bus in sleep: a DMA block across idle()", tm_bus_in_sleep);
    }
    bench.letter('n', "the mask's shadow: a line taken after the instruction that masked it?",
                 tn_shadow);
    bench.letter('w', "the idle hook against the tick's edge, placed to the cycle", tw_edge);
    bench.letter('o', "the kernel's turn: how many per interrupt, what one costs", to_turns);
    bench.letter('i', "THREE REAL RESETS (reboots the board)", ti_resets, false);
    bench.letter('t', "the runtime's seven functions at every alignment (rt/rt.cpp)", tt_runtime);

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
