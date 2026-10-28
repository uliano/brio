// test_x035_platform - the reference bench suite for the PLATFORM of the
// CH32X035: the running half (the mstatus.MIE critical section, the
// WFE-shaped idle hook, the 64-bit STK timebase, delay_us on its counter,
// the interrupt round trip and what the core's hardware prologue buys it,
// the microprocessor configuration register the crt writes because the
// vendor does) and what the crt left in the core's own registers, with
// the reset flags as the history they are.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is USART2 on PA2/PA3, the pads WCH's
// evaluation board brings to its "Serial port 2" header, wired to the
// probe's own serial; every clock this suite measures is inside the chip.
// The LED on PA0 is left undriven, so it stays dark: on WCH's QFN20
// evaluation board it is the header P4's LED1, jumpered to PA0, and it
// lights with the pad LOW.
//
// NOTHING HERE ENTERS STOP OR STANDBY: SLEEPDEEP is never set, and idle()
// is the plain Sleep of RM 2.3.2.
//
// What is exercised, letter by letter:
//   a  the boot story: RCC_RSTSCKR read as the ACCUMULATING history it is,
//      the seven flags named and cleared; the electronic signature (the
//      flash capacity, the unique identifier) and the word the vendor's
//      library reads as the chip identifier
//   b  the critical section: mstatus.MIE saved and restored, nesting, the
//      tick coalescing through a masked window, and the idle hook - a WFE,
//      not a WFI - returning on the tick with interrupts enabled
//   c  the STK timebase: the compare the ticker programmed, the high half
//      of a 64-bit counter that never moves under the reload,
//      ticks/millis/secs/now coherence, and the CNT-delta accumulation
//      delay.hpp is built on checked against the interrupt count over 200
//      reloads of the low half
//   d  delay_us: at least, never early, the cap, the refusal, with a 100 us
//      and a 900 us wait counted in HCLK cycles
//   e  THE INTERRUPT ROUND TRIP, in HCLK cycles on the STK counter: the
//      software interrupt raised by hand (PFIC IPSR), with the cycle count
//      read at the raise, at the handler's first and last statements, and
//      when the raiser sees the handler done. The image says which way it
//      was built (the CH32X035_HPE option)
//   f  corecfgr (CSR 0xBC0), which the crt writes 0x1F into because WCH's
//      own startup file does and no document says what the bits are: one
//      known loop timed with the register as the crt left it and with it
//      cleared, the register restored either way
//   g  the tick rate against the HOST's clock: the letter brackets a span
//      of its own ticks between two console lines, and whoever times those
//      two lines has the ratio - the HSI's own accuracy, since the tick is
//      counted in HSI cycles
//   h  what the crt left in the core: INTSYSCR's hardware stack as the
//      build asked and its nesting OFF, mtvec on the table with both mode
//      bits, mstatus in machine mode, and misa as the core reports it
//   i  THE FETCH AND TRAP COST MODEL test_x035_rubato's bounds are drawn
//      with (its letter c), on hand-placed assembly timed on the bare STK:
//      straight-line code a word every 4 cycles, a taken transfer 8 to a
//      target in one word and 12 to a 32-bit one across two, a not-taken
//      branch 4, a trap's entry 20 from the store, the ecall or the unmask
//      that raises it (taken at the next instruction, mepc read), an MRET
//      8 into code and 24 into a pending interrupt, and what the branch
//      predictor's wrong guess costs - each figure judged to the cycle,
//      with corecfgr as the crt left it and again with it cleared
//   w  THE IDLE HOOK AGAINST THE TICK'S EDGE, PLACED TO THE CYCLE: with
//      the STK running as the timebase, its CNTL written so the compare
//      comes D cycles later, D = 0..599, then the kernel's own shape - a
//      masked check, idle(), until one tick (and, a second pass, two) -
//      with the core's event latch set and clear by hand before each try.
//      The edge so walks across every instruction of the idle path and of
//      the loop around it, the sleep entry included, whatever the code's
//      layout. TIM3 counting at 1 MHz is the clock of each try and its
//      RESCUE: a try that takes its 50 ms period is a wake idle() lost for
//      good, and one that ends a whole tick period late is a wake lost
//      until the next tick (the bound: 250 us for one tick, 1250 us for
//      two). The writes to CNTL shorten the ticks they land in, so kernel
//      time runs fast during the letter. Prints the cycles from the edge
//      to the handler and to the caller's loop for an edge that finds the
//      core asleep, and what an idle() costs when the latch makes it
//      return at once
//   k  THE KERNEL'S TURN: a Tenuto pack of three quiet AOs, two with a
//      periodic time event, turned as Tenuto::run() turns it over 100
//      ticks with the tick the only interrupt - one turn per tick when
//      idle() leaves no event latched behind its wake, two when it does
//      (judged: 100 to 102, one turn of allowance at each end of the
//      window) - and what one quiet turn costs when the latch makes its
//      idle() return at once: the turn an interrupt would cost twice over
//   t  the runtime's seven functions (rt/rt.cpp) over every alignment,
//      length and overlap: rt/selftest.hpp's cases, the host suite's own,
//      against the symbols this image links - in `z`
//
// TIM3 IS A BENCH INSTRUMENT HERE, outside the rule that an app reads no
// register: this stratum has no timer driver (born with its first user),
// and letter w needs one free-running 1 MHz count with an update
// interrupt and nothing else - six registers of RM 13.4, named below.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "rt/selftest.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

extern "C" void _start();

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;

TestBench<Serial> bench;

/// What this boot was told, sampled once in main() before anything can
/// disturb it.
uint32_t boot_flags = 0;

// The STK counter as a cycle counter: it runs at HCLK and reloads every
// tick, so two readings less than a tick apart are a cycle count with the
// reload folded in. Only the low half is read.
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

uint32_t read_csr_misa() {
    uint32_t v;
    __asm__ volatile("csrr %0, misa" : "=r"(v));
    return v;
}
uint32_t read_csr_mtvec() {
    uint32_t v;
    __asm__ volatile("csrr %0, mtvec" : "=r"(v));
    return v;
}
uint32_t read_csr_mstatus() {
    uint32_t v;
    __asm__ volatile("csrr %0, mstatus" : "=r"(v));
    return v;
}
uint32_t read_csr_intsyscr() {
    uint32_t v;
    __asm__ volatile("csrr %0, 0x804" : "=r"(v));
    return v;
}
uint32_t read_corecfgr() {
    uint32_t v;
    __asm__ volatile("csrr %0, 0xbc0" : "=r"(v));
    return v;
}
void write_corecfgr(uint32_t v) {
    __asm__ volatile("csrw 0xbc0, %0" ::"r"(v) : "memory");
}

void print_flags(uint32_t flags) {
    print(serial, "    low_power=", (flags & rcc_lpwrrstf) != 0u,
          " wwdg=", (flags & rcc_wwdgrstf) != 0u,
          " iwdg=", (flags & rcc_iwdgrstf) != 0u,
          " software=", (flags & rcc_sftrstf) != 0u,
          " power=", (flags & rcc_porrstf) != 0u,
          " pin=", (flags & rcc_pinrstf) != 0u,
          " opa=", (flags & rcc_oparstf) != 0u, crlf);
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

    const uint32_t first = Rcc::reset_flags();
    const uint32_t second = Rcc::reset_flags();
    Rcc::clear_reset_flags();
    const uint32_t after = Rcc::reset_flags();
    print(serial, "  reset_flags()=", hex(first), " again=", hex(second),
          " after clear_reset_flags()=", hex(after), " RSTSCKR=", hex(rcc()->RSTSCKR), crlf);
    bench.verdict("reset_flags() is non-destructive: two reads agree", first == second);
    bench.verdict("clear_reset_flags() leaves the flags clean", after == 0u);
    bench.verdict("and the register holds nothing else afterwards: RMVF is a LEVEL on "
                  "this silicon - written 1 it stays 1 - so the verb clears it too, and "
                  "the low 24 bits are reserved",
                  rcc()->RSTSCKR == 0u);

    const uint16_t kbytes = esig_flash_kbytes();
    print(serial, "  ESIG flash capacity: ", kbytes, " KB; unique id ",
          hex(esig_uid_word(0)), " ", hex(esig_uid_word(1)), " ", hex(esig_uid_word(2)), crlf);
    print(serial, "  the word at 0x1FFFF704 (the vendor library's chip id): ",
          hex(chip_id_word()), ", its bits 7:4 = ", (chip_id_word() >> 4) & 0xFu, crlf);
    bench.verdict("the electronic signature gives this part's 62 KB of code flash",
                  kbytes == device::flash_bytes / 1024u);
}

// ---------------------------------------------------------------------------
// b - the critical section and the idle hook
// ---------------------------------------------------------------------------
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
    print(serial, "  5 ms with interrupts masked advanced the tick by ", through_mask,
          " ms, the tick in within ", settle, " cycles of the unmask", crlf);
    bench.verdict("a masked window LOSES ticks (the STK interrupt is a pending "
                  "bit: one tick is delivered, the rest coalesce)",
                  through_mask >= 1u && through_mask <= 5u);

    // idle(): a WFE on this core, then unmask. Any enabled interrupt wakes
    // it, so the console must be silent first.
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
    print(serial, "  with the console silent, ", calls, " idle() call(s) covered the ",
          idle_cycles / cycles_per_us(), " us to the next tick", crlf);
    bench.verdict("idle() sleeps and the tick is what brings it back, inside "
                  "one tick period",
                  calls >= 1u && calls < 20u);
    bench.verdict("and it comes back with interrupts enabled", P::interrupts_enabled());
}

// ---------------------------------------------------------------------------
// c - the STK timebase
// ---------------------------------------------------------------------------
void tc_ticker() {
    const uint32_t cmp = stk()->CMPLR;
    print(serial, "  STK CTLR=", hex(stk()->CTLR), " CMPLR=", cmp, " (",
          SysClock::hz / Ticker::ticks_per_second - 1u, " expected)", " CMPHR=", stk()->CMPHR,
          " CNTH=", stk()->CNTH, crlf);
    bench.verdict("the compare is Clock::hz / tps - 1",
                  cmp == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on HCLK, reloading, with its interrupt armed",
                  (stk()->CTLR & (stk_ste | stk_stie | stk_stclk | stk_stre | stk_mode)) ==
                      (stk_ste | stk_stie | stk_stclk | stk_stre));
    bench.verdict("the high half of the 64-bit counter never moves: the reload "
                  "puts the low half back to zero at the compare",
                  stk()->CNTH == 0u && stk()->CMPHR == 0u);

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

    // The CNT-delta accumulation delay.hpp rests on, against the interrupt
    // count over 200 reloads: both ends of the span are a tick edge, so the
    // residue is the loop's own granularity and not the phase it began at.
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
    const uint32_t err = accumulated > expected ? accumulated - expected : expected - accumulated;
    print(serial, "  200 ticks = ", accumulated, " cycles by CNT deltas, ", expected,
          " expected, error ", err, " cycles", crlf);
    bench.verdict("the CNT-delta accumulation tracks the interrupt count over 200 "
                  "reloads to under a period's worth of error",
                  err < period);
}

// ---------------------------------------------------------------------------
// d - delay_us
// ---------------------------------------------------------------------------
void td_delay() {
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
    print(serial, "  delay_us(100) spent ", short_spent, " cycles (", asked_short, " asked)", crlf);
    bench.verdict("100 us is at least what it asked for and under 4 per cent over",
                  short_ok && short_spent >= asked_short &&
                      short_spent < asked_short + asked_short / 25u);

    const uint32_t asked_long = 900u * cycles_per_us();
    const uint32_t c1 = cycles_now();
    const bool long_ok = delay_us(clock, 900);
    const uint32_t long_spent = cycles_between(c1, cycles_now());
    print(serial, "  delay_us(900) spent ", long_spent, " cycles (", asked_long, " asked)", crlf);
    bench.verdict("900 us - the longest wait that still fits a tick period - is at "
                  "least what it asked for and under 1 per cent over",
                  long_ok && long_spent >= asked_long && long_spent < asked_long + asked_long / 100u);

    bench.verdict("a wait of one tick period (1000 us) is refused", !delay_us(clock, 1000));
    bench.verdict("and so is anything longer", !delay_us(clock, 50'000));
    bench.verdict("and so is the 65536 us gate the arithmetic rests on", !delay_us(clock, 65'536));
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
    print(serial, "  built with the hardware prologue/epilogue (CH32X035_HPE=ON)", crlf);
#else
    print(serial, "  built with gcc's own prologue/epilogue (CH32X035_HPE=OFF)", crlf);
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
        // A tick landing inside the window would add its own handler to the
        // measurement: wait for a fresh tick, then there is a millisecond
        // of quiet.
        const uint32_t t = Ticker::ticks();
        while (Ticker::ticks() == t) {
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
        bench.verdict("the software interrupt was raised by hand and taken", false);
        return;
    }
    print(serial, "  entry (raise -> the handler's first statement): ", best_entry, "..",
          worst_entry, " cycles", crlf);
    print(serial, "  body  (first -> last statement of the handler): ", best_body, " cycles", crlf);
    print(serial, "  exit  (last statement -> the raiser sees it done): ", best_exit,
          " cycles, one spin iteration included", crlf);
    print(serial, "  whole trip (raise -> the raiser sees it done): ", best_trip, "..", worst_trip,
          " cycles", crlf);
    bench.verdict("every raise of the software interrupt was taken", seen == rounds);
    bench.verdict("the handler was entered within 100 cycles of the raise", best_entry <= 100u);
    bench.verdict("and the whole trip is under 300 cycles", best_trip < 300u);
}

// ---------------------------------------------------------------------------
// f - the microprocessor configuration register
// ---------------------------------------------------------------------------
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
    const uint32_t n = 2000;
    const uint32_t period = stk()->CMPLR + 1u;

    const uint32_t as_found = read_corecfgr();
    print(serial, "  corecfgr as the crt left it: ", hex(as_found), crlf);
    bench.verdict("the crt wrote 0x1F into it, WCH's value", as_found == 0x1Fu);

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
    print(serial, "  ", n, " iterations of a loop with a branch and a load: ", with_bits,
          " cycles at ", hex(as_found), ", ", without_bits, " cycles at ", hex(while_cleared), crlf);
    bench.verdict("the loop ran with the register as the crt left it, inside one tick period",
                  with_bits > 0u && with_bits < period);
    bench.verdict("and ran with the register cleared, inside one tick period",
                  without_bits > 0u && without_bits < period);
    bench.verdict("the register takes a zero: the second run really ran without the bits",
                  while_cleared == 0u);
    bench.verdict("and it is back to what the crt wrote", restored == as_found);
}

// ---------------------------------------------------------------------------
// g - the tick against the host's clock
// ---------------------------------------------------------------------------
void tg_bracket() {
    constexpr uint32_t span = 5000;
    print(serial, "  bracket ", span, " ticks: the host's time between this line and "
                  "the next is the tick's rate against the HSI's", crlf);
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < span) {
    }
    print(serial, "  bracket done", crlf);
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < 5u) {
    }
    bench.verdict("the bracket ran to its count", Ticker::ticks() - t0 >= span);
}

// ---------------------------------------------------------------------------
// h - what the crt left in the core
// ---------------------------------------------------------------------------
void th_core() {
    const uint32_t intsyscr = read_csr_intsyscr();
    const uint32_t mtvec = read_csr_mtvec();
    const uint32_t mstatus = read_csr_mstatus();
    const uint32_t misa = read_csr_misa();
    print(serial, "  INTSYSCR=", hex(intsyscr), " mtvec=", hex(mtvec), " mstatus=", hex(mstatus),
          " misa=", hex(misa), crlf);
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
    bench.verdict("HWSTKEN is set: the image was built for the hardware prologue",
                  (intsyscr & 0x1u) != 0u);
#else
    bench.verdict("HWSTKEN is clear: the image was built for gcc's own prologue",
                  (intsyscr & 0x1u) == 0u);
#endif
    bench.verdict("INESTEN is clear: no interrupt nests over another", (intsyscr & 0x2u) == 0u);
    bench.verdict("mtvec points at the table, vectored, with absolute addresses",
                  mtvec == (reinterpret_cast<uint32_t>(&_start) | 3u));
    // The PRIVILEGE is proven by the read itself: mstatus is a machine-mode
    // CSR, and from user mode the csrr above would have trapped instead of
    // arriving here. MPP is printed and not judged - it is the mode an MRET
    // returns to: the privileged specification has MRET leave it at the
    // least-privileged mode the core implements, while the QingKe V4 manual
    // (8.2) keeps it at machine mode, which is what the silicon does -
    // MPP reads 3 after thousands of MRETs (measured on the CH32X035F8U6).
    print(serial, "  mstatus.MPP=", (mstatus >> 11) & 0x3u, " (what the next MRET returns to)",
          crlf);
    bench.verdict("mstatus reads at all - machine mode, a user-mode read traps - with MIE set "
                  "in a letter",
                  (mstatus & mstatus_mie) != 0u);
    bench.verdict("misa reports a 32-bit core with I, M, A and C",
                  (misa >> 30) == 1u && (misa & (1u << 8)) != 0u && (misa & (1u << 12)) != 0u &&
                      (misa & (1u << 0)) != 0u && (misa & (1u << 2)) != 0u);
}

// ---------------------------------------------------------------------------
// w - the idle hook against the tick's edge, placed to the cycle
// ---------------------------------------------------------------------------
/// TIM3, RM 13.4: the general-purpose timer, 16 bits, clocked from HCLK
/// (RM 3.3, the clock tree feeds TIM2/TIM3 from the HB bus; this series
/// has no PB prescaler). Every register is sixteen bits wide and is
/// accessed as such, as the register table lists them.
struct Rescue {
    static constexpr uintptr_t base = 0x40000400UL;
    static volatile uint16_t& reg(uint32_t offset) {
        return *reinterpret_cast<volatile uint16_t*>(base + offset);
    }
    static volatile uint16_t& ctlr1() { return reg(0x00); }
    static volatile uint16_t& dmaintenr() { return reg(0x0C); }
    static volatile uint16_t& intfr() { return reg(0x10); }
    static volatile uint16_t& swevgr() { return reg(0x14); }
    static volatile uint16_t& cnt() { return reg(0x24); }
    static volatile uint16_t& psc() { return reg(0x28); }
    static volatile uint16_t& atrlr() { return reg(0x2C); }
    static constexpr uint16_t cen = 1u << 0;   ///< CTLR1: count
    static constexpr uint16_t urs = 1u << 2;   ///< CTLR1: only an overflow raises UIF's interrupt
    static constexpr uint16_t uie = 1u << 0;   ///< DMAINTENR: the update interrupt
    static constexpr uint16_t uif = 1u << 0;   ///< INTFR: write 0 to clear
    static constexpr uint16_t ug = 1u << 0;    ///< SWEVGR: load PSC and ATRLR now

    /// 1 MHz, a 50 ms period, its update interrupt on the PFIC.
    static void start() {
        rcc()->APB1PCENR = rcc()->APB1PCENR | rcc_pb1_tim3;
        ctlr1() = urs;
        psc() = static_cast<uint16_t>(SysClock::hz / 1'000'000u - 1u);
        atrlr() = 49'999u;
        swevgr() = ug;   // PSC is preloaded (13.4.9): an update makes it current
        intfr() = 0;
        dmaintenr() = uie;
        Pfic::clear_pending(Irq::tim3);
        Pfic::enable(Irq::tim3);
        ctlr1() = static_cast<uint16_t>(urs | cen);
    }
    static void stop() {
        ctlr1() = 0;
        dmaintenr() = 0;
        intfr() = 0;
        Pfic::disable(Irq::tim3);
        Pfic::clear_pending(Irq::tim3);
        rcc()->APB1PCENR = rcc()->APB1PCENR & ~rcc_pb1_tim3;
    }
};
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
        Rescue::cnt() = 0;
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
        took_us = Rescue::cnt();
        rescued = rescue_periods != r0;
        r.to_entry = tick_entry;
    }
    // A wake lost for good waits for the rescue; one lost until the NEXT
    // tick ends a whole period late. The edges asked for land within 13 us
    // of the period they need, so the bound sits a quarter period past that.
    r.took_us = took_us;
    r.lost = rescued;
    r.late = !rescued && took_us > edges * 1000u - 750u;
    return r;
}

void tw_edge() {
    Rescue::start();
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
    Rescue::stop();

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
// k - the kernel's turn: how many per interrupt, and what one costs
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

void tk_turns() {
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

    print(serial, "  100 quiet ticks: ", turns,
          " kernel turns; a quiet turn whose idle() returns at once: ", turn_cycles,
          " cycles (two counter reads included)", crlf);
    bench.verdict("the kernel loop turns once per interrupt, not twice (no event is left "
                  "latched behind the wake)",
                  turns >= 100u && turns <= 102u);
}

void banner() {
    print(serial, crlf, "test_x035_platform on ", device::part_name, " - the platform", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

// The vectors this program owns.
BRIO_CH32_VECTOR(systick_handler) {
    if (tick_probe) {
        tick_entry = brio::stk()->CNTL;
    }
    brio::Ticker::tick();
}

/// Letter w's rescue and clock: TIM3's update, counted.
BRIO_CH32_VECTOR(tim3_handler) {
    Rescue::intfr() = 0;
    rescue_periods = rescue_periods + 1u;
}

BRIO_CH32_VECTOR(usart2_handler) { (void)Serial::isr(); }

BRIO_CH32_VECTOR(software_handler) {
    sw_entry_cycles = brio::stk()->CNTL;
    brio::Pfic::clear_pending(brio::Irq::software);
    sw_hits = sw_hits + 1u;
    sw_exit_cycles = brio::stk()->CNTL;
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

namespace {

// ---------------------------------------------------------------------------
// i - the fetch and trap cost model
// ---------------------------------------------------------------------------
// THE NUMBERS test_x035_rubato's bounds are drawn with (that suite's letter
// c states the model, and every bound of it counts a path of its release
// listing with these figures), each measured here on hand-placed assembly
// timed on the STK's CNTL read bare, at 48 MHz with the flash at two wait
// states (RM 20.3.1):
//   - straight-line code issues a word every 4 cycles: a 32-bit
//     instruction 4, at either alignment, a compressed one 2;
//   - a taken transfer issues its target 8 cycles after it when the target
//     sits in one word (a compressed instruction, or a 32-bit one on a
//     word), 12 when the target is a 32-bit instruction across two words;
//     a not-taken conditional branch costs its word, 4; a call and its
//     return 8 each;
//   - a trap issues the handler's first instruction 20 cycles after the
//     instruction that raises it: the store that pends the software
//     interrupt or a peripheral line, an ecall, the unmask that finds the
//     software interrupt pending - and is taken at the instruction after
//     the store or the unmask, so nothing retires behind either
//     (ch32x035/schedule.hpp's request rests on that);
//   - an MRET issues its target 8 cycles after it, and the first
//     instruction of a handler whose line it finds pending (the tail) 24;
//   - the branch predictor corecfgr's 0x1F turns on: right, it costs
//     nothing against the figures above; wrong, it costs the cycles this
//     letter prints - one branch that falls through after being guessed
//     taken up to 8, one taken after being guessed to fall through
//     nothing - and the Rubato bounds allow 4 at each predicted site of a
//     path, calls and returns counted, which is how each row is judged.
// The same figures are then measured with corecfgr cleared (written and
// restored with interrupts masked, as letter f does): every one the same
// but where a transfer the core resolves when it executes - a taken
// conditional branch, a jump through a register - shares its fetch word
// with a load or a store before it, which costs 4 more without the
// predictor (two rows show it, both settings judged). With 0x1F and the
// guess right, that pairing costs what the model says, which is what the
// Rubato bounds count.
//
// HOW EACH IS READ. A run of N units and a run of 2N units of the same
// pattern, each the least of 32 timed calls with interrupts masked: the
// call, the return and the two reads are the same in both, so the
// difference is N units and nothing else, and it is judged to the cycle.
// The trap sequences hold the counter's address in a register before the
// trigger, so a stamp is one 32-bit load: the stamp before the trigger is
// one word ahead of it, and the stamp in the handler is its first
// instruction; the ruler row measures that word (two loads back to back)
// and each trap figure is the stamps' difference less it. Every trip of a
// trap sequence is judged, not only the least: no predicted site lies in
// those windows, so nothing may move them. A trip the tick landed in is
// dropped (the tick count read around it).
//
// WHERE THE TRAPS LAND. The ecall and TIM2's capture-compare line (no
// timer runs: the line is pended by a store, TIM2's clock stays gated)
// are bound in the crt's table to the handlers below. The software
// interrupt is letter e's, bound to a C handler; to time it from its first
// instruction, mtvec is pointed for the measurement at the letter's own
// table in the flash - the crt's with entry 14 replaced - and put back.
// TIM2's line timed through both tables says what the table's place
// costs: nothing.
//
// The handlers run only over the letter's own sequences, which keep in
// registers what the handlers need (a5 the STK, a4 the record, a3 the
// PFIC's pending banks, a2 and a1 the two lines' bits, a6 the return, a7
// the tail asked for) and leave the handlers t3 and s2..s5 (saved and
// restored around each sequence); with the hardware prologue the core
// saves the caller-saved registers, and without it nothing the handlers
// clobber is live, so the letter runs in both images.
//
// WHERE THE LETTER'S OWN CODE LIES: after every other function of the
// image. On this core an instruction's cost depends on its address modulo
// four (docs/ch32x035/platform.md, "A handler in SRAM"), so code added in
// front of another letter's would move that letter's figures. The linker
// script lays every `.text` input section first and `.rodata` after them
// (ch32x035/ld/ch32x035f8.ld), so the letter's functions and its assembly
// share one section named under `.rodata` with the code flags ("ax"):
// still code in the flash, behind everything else. main() reaches it
// through one call, four bytes, which keeps every function after main()
// at its address modulo four. And what the letter calls leaves the
// compiler's layout of the others as it was: its verdicts take the bench's
// one-name overload (Named, below), and no print of it has the argument
// types of letter f's first print - one that did moved letter f to
// another place in the image, read in the listing.
#define COST_PLACED gnu::section(".rodata.cost_letter")

static_assert(static_cast<uint8_t>(Irq::software) == 14u && static_cast<uint8_t>(Irq::tim2_cc) == 51u,
              "the letter's assembly pends line 14 (IPSR0 bit 14) and line 51 (IPSR1 bit 19)");
static_assert(static_cast<uint8_t>(Irq::ecall_machine) == 5u && static_cast<uint8_t>(Irq::systick) == 12u &&
                  static_cast<uint8_t>(Irq::usart2) == 39u && static_cast<uint8_t>(Irq::tim3) == 54u &&
                  vector_count == 55u,
              "the letter's table binds these entries where the crt's does");
static_assert(__builtin_offsetof(StkRegs, CNTL) == 8u && __builtin_offsetof(PficRegs, IPSR) == 0x200u &&
                  __builtin_offsetof(PficRegs, IPRR) == 0x280u,
              "the letter's assembly addresses CNTL at 0xE000F008 and the pending banks at 0xE000E200/280");

/// One timed pattern: `count` units after `prefix`, then `suffix` and the
/// return, a compressed `c.jr ra` - at the same alignment in both lengths
/// of a pattern, since every unit is a whole number of words.
#define COST_SEQ(name, prefix, count, unit, suffix)                                   \
    ".balign 4\n.globl " #name "\n.type " #name ", @function\n" #name ":\n" prefix    \
    ".rept " #count "\n" unit ".endr\n" suffix ".option rvc\nc.jr ra\n"
#define COST_PAIR(name, prefix, n, n2, unit, suffix) \
    COST_SEQ(name##_n, prefix, n, unit, suffix) COST_SEQ(name##_2n, prefix, n2, unit, suffix)

/// A trap sequence's opening: s2..s5 saved, set to the "never stamped"
/// value, the registers the handlers read loaded, the return label (9:)
/// in a6. `tail` = 1 asks TIM2's handler to pend the software interrupt.
#define COST_TRAP_OPEN(name, tail)                                                     \
    ".balign 4\n.globl " #name "\n.type " #name ", @function\n" #name ":\n.option rvc\n" \
    "addi sp, sp, -16\nsw s2, 0(sp)\nsw s3, 4(sp)\nsw s4, 8(sp)\nsw s5, 12(sp)\n"       \
    "li s2, -1\nli s3, -1\nli s4, -1\nli s5, -1\nmv a4, a0\n"                           \
    "li a5, 0xE000F000\nli a3, 0xE000E200\nlui a2, 0x4\nlui a1, 0x80\nli a7, " #tail     \
    "\nla a6, 9f\n.balign 4\n.option norvc\n"
/// Its close: sixteen compressed no-ops the core would run if it took the
/// trap late (it does not: the handlers record mepc), the return label,
/// the stamp there, the record written, s2..s5 restored.
#define COST_TRAP_CLOSE                                                                 \
    ".option rvc\n.rept 16\nc.nop\n.endr\n.balign 4\n9:\n.option norvc\nlw t2, 8(a5)\n"  \
    ".option rvc\nsw t0, 0(a4)\nsw s2, 4(a4)\nsw s3, 8(a4)\nsw t2, 12(a4)\nsw s4, 16(a4)\n" \
    "sw s5, 20(a4)\nlw s2, 0(sp)\nlw s3, 4(sp)\nlw s4, 8(sp)\nlw s5, 12(sp)\n"           \
    "addi sp, sp, 16\nc.jr ra\n"

__asm__(".pushsection .rodata.cost_letter,\"ax\",@progbits\n"
        ".option push\n"
        ".option norelax\n"
        // The ruler: two loads of CNTL back to back, one word apart.
        ".balign 4\n.globl cost_ruler\n.type cost_ruler, @function\ncost_ruler:\n.option rvc\n"
        "li a5, 0xE000F000\n.balign 4\n.option norvc\nlw t0, 8(a5)\nlw t1, 8(a5)\n"
        ".option rvc\nsw t0, 0(a0)\nsw t1, 4(a0)\nc.jr ra\n"
        // Straight-line code: 32-bit on a word, 32-bit half a word off, compressed.
        COST_PAIR(cost_w32, ".option norvc\n", 32, 64, "addi t0, t0, 1\n", "")
        COST_PAIR(cost_w32h, ".option rvc\nc.nop\n.option norvc\n", 32, 64, "addi t0, t0, 1\n", "")
        COST_PAIR(cost_w16, ".option rvc\n", 32, 64, "c.addi t0, 1\n", "")
        // Taken transfers: a c.j to a compressed target, a j to a 32-bit
        // target on a word and across two words, a taken conditional branch.
        COST_PAIR(cost_jc, ".option rvc\n", 16, 32, "c.j 1f\n.2byte 0\n1:\n", "")
        COST_PAIR(cost_jw, ".option norvc\n", 16, 32, "j 1f\n.4byte 0\n1:\n", "")
        COST_PAIR(cost_jx, ".option rvc\nc.nop\n.option norvc\n", 16, 32, "j 1f\n.4byte 0\n1:\n", "")
        COST_PAIR(cost_bt, ".option norvc\n", 16, 32, "beqz zero, 1f\n.4byte 0\n1:\n", "")
        // A not-taken conditional branch.
        COST_PAIR(cost_bn, ".option norvc\n", 16, 32, "bnez zero, 2f\n", "2:\n")
        // A call to a leaf on a word and its return to a 32-bit instruction on a word.
        COST_PAIR(cost_call, ".option norvc\naddi t1, ra, 0\n", 16, 32, "jal ra, cost_leaf\n",
                  "addi ra, t1, 0\n")
        ".balign 4\n.globl cost_leaf\n.type cost_leaf, @function\ncost_leaf:\n.option rvc\nc.jr ra\n"
        // The pairing the two settings part on: a taken conditional branch
        // sharing its word with a store before it (the store below the
        // stack pointer, where nothing lives), and a call through a
        // register sharing its word with a load before it.
        COST_PAIR(cost_stb, ".option rvc\nc.li a4, 0\n.option norvc\naddi a3, sp, -16\n.option rvc\nc.nop\n",
                  16, 32, "c.sw a4, 0(a3)\nc.beqz a4, 1f\n.4byte 0\n1:\n", "")
        COST_PAIR(cost_ldj, ".option norvc\naddi t1, ra, 0\naddi a3, sp, -16\nla a5, cost_leaf\n.option rvc\n",
                  16, 32, "c.lw a2, 0(a3)\nc.jalr a5\n", ".option norvc\naddi ra, t1, 0\n")
        // The predictor: sixteen conditional branches whose direction is
        // a0 (32-bit over a 32-bit instruction; compressed over two
        // compressed ones), and sixteen calls through a0 to one of two leaves.
        ".balign 4\n.globl cost_pred_b32\n.type cost_pred_b32, @function\ncost_pred_b32:\n"
        ".option norvc\n.rept 16\nbnez a0, 1f\naddi zero, zero, 0\n1:\n.endr\n.option rvc\nc.jr ra\n"
        ".balign 4\n.globl cost_pred_b16\n.type cost_pred_b16, @function\ncost_pred_b16:\n"
        ".option rvc\n.rept 16\nc.bnez a0, 1f\nc.nop\nc.nop\n1:\n.endr\nc.jr ra\n"
        // One branch alone, in three layouts: compressed on a word over two
        // compressed instructions; 32-bit half a word off, and on a word,
        // over a 32-bit instruction. Four compressed no-ops follow, so
        // both ways reach the return through the same words.
        ".balign 4\n.globl cost_pred_one16\n.type cost_pred_one16, @function\ncost_pred_one16:\n"
        ".option rvc\nc.bnez a0, 1f\nc.nop\nc.nop\n1:\n.rept 4\nc.nop\n.endr\n.balign 4\nc.jr ra\n"
        ".balign 4\n.globl cost_pred_one32h\n.type cost_pred_one32h, @function\ncost_pred_one32h:\n"
        ".option rvc\nc.nop\n.option norvc\nbnez a0, 1f\naddi zero, zero, 0\n1:\n.option rvc\n"
        ".rept 4\nc.nop\n.endr\n.balign 4\nc.jr ra\n"
        ".balign 4\n.globl cost_pred_one32\n.type cost_pred_one32, @function\ncost_pred_one32:\n"
        ".option norvc\nbnez a0, 1f\naddi zero, zero, 0\n1:\n.option rvc\n"
        ".rept 4\nc.nop\n.endr\n.balign 4\nc.jr ra\n"
        ".balign 4\n.globl cost_pred_jalr\n.type cost_pred_jalr, @function\ncost_pred_jalr:\n"
        ".option norvc\naddi t1, ra, 0\n.rept 16\njalr ra, 0(a0)\n.endr\naddi ra, t1, 0\n"
        ".option rvc\nc.jr ra\n"
        ".balign 4\n.globl cost_leaf_a\n.type cost_leaf_a, @function\ncost_leaf_a:\n.option rvc\nc.jr ra\n"
        ".balign 4\n.globl cost_leaf_b\n.type cost_leaf_b, @function\ncost_leaf_b:\n.option rvc\nc.jr ra\n"
        // The traps. A store pends the software interrupt (IPSR0, bit 14).
        COST_TRAP_OPEN(cost_trap_sw, 0)
        "lw t0, 8(a5)\nsw a2, 0(a3)\n.globl cost_after_sw_store\ncost_after_sw_store:\n"
        COST_TRAP_CLOSE
        // A store pends TIM2's capture-compare line (IPSR1, bit 19).
        COST_TRAP_OPEN(cost_trap_line, 0)
        "lw t0, 8(a5)\nsw a1, 4(a3)\n"
        COST_TRAP_CLOSE
        // The same, its handler pending the software interrupt before its MRET.
        COST_TRAP_OPEN(cost_trap_tail, 1)
        "lw t0, 8(a5)\nsw a1, 4(a3)\n"
        COST_TRAP_CLOSE
        // An environment call.
        COST_TRAP_OPEN(cost_trap_ecall, 0)
        "lw t0, 8(a5)\necall\n"
        COST_TRAP_CLOSE
        // The software interrupt pended under the mask, taken at the unmask.
        COST_TRAP_OPEN(cost_trap_unmask, 0)
        "csrci mstatus, 8\nsw a2, 0(a3)\nlw t0, 8(a5)\ncsrsi mstatus, 8\n"
        ".globl cost_after_unmask\ncost_after_unmask:\n"
        COST_TRAP_CLOSE
        // The software interrupt's handler in the letter's table: the
        // first stamp, mepc recorded and pointed at the return, the line's
        // pending bit cleared (IPRR0), the last stamp, MRET.
        ".balign 4\n.globl cost_software\n.type cost_software, @function\ncost_software:\n"
        ".option norvc\nlw s2, 8(a5)\ncsrr t3, mepc\nsw t3, 24(a4)\ncsrw mepc, a6\n"
        "sw a2, 0x80(a3)\nlw s3, 8(a5)\nmret\n"
        // The ecall's, in both tables.
        ".balign 4\n.globl ecall_m_handler\n.type ecall_m_handler, @function\necall_m_handler:\n"
        ".option norvc\nlw s2, 8(a5)\ncsrr t3, mepc\nsw t3, 24(a4)\ncsrw mepc, a6\nlw s3, 8(a5)\nmret\n"
        // TIM2's capture-compare line's, in both tables: its own pending
        // bit cleared (IPRR1), the software interrupt pended if a7 asks.
        ".balign 4\n.globl tim2_cc_handler\n.type tim2_cc_handler, @function\ntim2_cc_handler:\n"
        ".option norvc\nlw s4, 8(a5)\ncsrw mepc, a6\nsw a1, 0x84(a3)\nbeqz a7, 8f\nsw a2, 0(a3)\n"
        "8:\nlw s5, 8(a5)\nmret\n"
        // The letter's table: the crt's 55 words with entry 14 replaced
        // (every entry the crt aliases to default_handler is that here;
        // entry 0, the crt's jump, is never read through mtvec).
        ".balign 4\n.globl cost_vectors\ncost_vectors:\n"
        ".word 0, 0, nmi_handler, fault_handler, 0, ecall_m_handler, 0, 0\n"
        ".word ecall_u_handler, breakpoint_handler, 0, 0, systick_handler, 0, cost_software, 0\n"
        ".word default_handler, default_handler, default_handler, 0\n"   // 16..19
        ".rept 19\n.word default_handler\n.endr\n"        // 20..38
        ".word usart2_handler\n"                          // 39
        ".rept 11\n.word default_handler\n.endr\n"        // 40..50
        ".word tim2_cc_handler, default_handler, default_handler, tim3_handler\n"   // 51..54
        ".option pop\n"
        ".popsection\n");

#undef COST_TRAP_CLOSE
#undef COST_TRAP_OPEN
#undef COST_PAIR
#undef COST_SEQ

}  // namespace

// The letter's assembly, by its symbols (C linkage, outside the unnamed namespace).
extern "C" {
void cost_ruler(uint32_t* two);
void cost_w32_n();
void cost_w32_2n();
void cost_w32h_n();
void cost_w32h_2n();
void cost_w16_n();
void cost_w16_2n();
void cost_jc_n();
void cost_jc_2n();
void cost_jw_n();
void cost_jw_2n();
void cost_jx_n();
void cost_jx_2n();
void cost_bt_n();
void cost_bt_2n();
void cost_bn_n();
void cost_bn_2n();
void cost_stb_n();
void cost_stb_2n();
void cost_ldj_n();
void cost_ldj_2n();
void cost_call_n();
void cost_call_2n();
void cost_pred_b32(uint32_t);
void cost_pred_b16(uint32_t);
void cost_pred_jalr(uint32_t);
void cost_pred_one16(uint32_t);
void cost_pred_one32h(uint32_t);
void cost_pred_one32(uint32_t);
void cost_leaf_a();
void cost_leaf_b();
void cost_trap_sw(void* stamps);
void cost_trap_line(void* stamps);
void cost_trap_tail(void* stamps);
void cost_trap_ecall(void* stamps);
void cost_trap_unmask(void* stamps);
void cost_after_sw_store();
void cost_after_unmask();
extern const uint32_t cost_vectors[];
}

namespace {

/// What a trap sequence writes: the four stamps of the thread and the
/// software interrupt's (or the ecall's) handler, TIM2's two, and the
/// mepc the handler found. 0xFFFFFFFF is "never stamped": CNTL never
/// reaches it under the ticker's compare.
struct CostStamps {
    uint32_t before;       ///< the thread, one word before the trigger
    uint32_t first;        ///< the software interrupt's or the ecall's handler, first instruction
    uint32_t last;         ///< its last stamp, the word before its MRET
    uint32_t after;        ///< the thread, the return label's first instruction
    uint32_t line_first;   ///< TIM2's handler, first instruction
    uint32_t line_last;    ///< its last stamp, the word before its MRET
    uint32_t mepc;         ///< what the software interrupt's or the ecall's handler read
};
static_assert(sizeof(CostStamps) == 28u, "the assembly writes offsets 0..24");
constexpr uint32_t never_stamped = 0xFFFF'FFFFu;

/// A verdict's name in its two spellings, one a setting, built at compile
/// time: the letter runs every figure twice, and a name assembled at run
/// time would take the bench's other verdict overload, an instantiation
/// the rest of the image does not have (the placement above).
struct Named {
    const char* base;
    const char* on;    ///< corecfgr 0x1F
    const char* off;   ///< corecfgr cleared
    const char* pick(bool predictor_on) const { return predictor_on ? on : off; }
};
#define COST_NAMED(s) Named{s, s " (corecfgr 0x1F)", s " (corecfgr 0)"}

struct Span {
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    void add(uint32_t v) {
        if (v < least) { least = v; }
        if (v > most) { most = v; }
    }
};

/// The least and most of 32 calls of a pattern, interrupts masked, the
/// counter read either side of the call. One instance, so both lengths
/// of a pattern are timed by the same instructions.
[[gnu::noinline, COST_PLACED]] Span cost_time(void (*f)()) {
    Span s;
    for (uint32_t i = 0; i < 32u; ++i) {
        P::CriticalSection cs;
        const uint32_t a = stk()->CNTL;
        f();
        const uint32_t b = stk()->CNTL;
        s.add(cycles_between(a, b));
    }
    return s;
}

/// One call of a predictor pattern with its argument, timed the same way.
[[gnu::noinline, COST_PLACED]] uint32_t cost_time_once(void (*f)(uint32_t), uint32_t arg) {
    P::CriticalSection cs;
    const uint32_t a = stk()->CNTL;
    f(arg);
    const uint32_t b = stk()->CNTL;
    return cycles_between(a, b);
}

struct CostRow {
    Named name;
    void (*n)();
    void (*n2)();
    uint32_t units;       ///< how many more units the second run has
    uint32_t model;       ///< cycles a unit, by the model, corecfgr 0x1F
    uint32_t model_off;   ///< and with corecfgr cleared
    uint32_t sites;       ///< predicted sites a unit (transfers and branches)
};

constexpr CostRow cost_rows[] = {
    {COST_NAMED("32-bit instructions on a word"), cost_w32_n, cost_w32_2n, 32, 4, 4, 0},
    {COST_NAMED("32-bit instructions half a word off"), cost_w32h_n, cost_w32h_2n, 32, 4, 4, 0},
    {COST_NAMED("compressed instructions"), cost_w16_n, cost_w16_2n, 32, 2, 2, 0},
    {COST_NAMED("a c.j to a target in one word"), cost_jc_n, cost_jc_2n, 16, 8, 8, 1},
    {COST_NAMED("a j to a 32-bit target on a word"), cost_jw_n, cost_jw_2n, 16, 8, 8, 1},
    {COST_NAMED("a j to a 32-bit target across two words"), cost_jx_n, cost_jx_2n, 16, 12, 12, 1},
    {COST_NAMED("a taken conditional branch, its target on a word"), cost_bt_n, cost_bt_2n, 16, 8, 8, 1},
    {COST_NAMED("a not-taken conditional branch"), cost_bn_n, cost_bn_2n, 16, 4, 4, 1},
    {COST_NAMED("a call and its return, both targets on a word"), cost_call_n, cost_call_2n, 16, 16, 16, 2},
    // A transfer the core resolves when it executes - a conditional
    // branch, a jump through a register - waits 4 cycles more behind a
    // load or a store fetched in the same word, unless the predictor has
    // already sent the fetch to the target: so 8 with corecfgr's 0x1F,
    // 12 with it cleared (a direct jump, resolved at its decode, does not
    // wait). The store and its branch, one word, then the target's word.
    {COST_NAMED("a store and a taken branch in one word"), cost_stb_n, cost_stb_2n, 16, 8, 12, 1},
    // The load and its call, one word: the call 8 (12 cleared), the
    // leaf's return to the next unit 8.
    {COST_NAMED("a load and a call through a register in one word"), cost_ldj_n, cost_ldj_2n, 16, 16, 20, 2},
};

/// The fetch rows: the difference of the two lengths' least judged to the
/// cycle; each length's spread judged against the predictor's allowance,
/// 4 at each predicted site of the window - the pattern's own and the
/// call into it and its return - and, with the predictor off, nothing:
/// no guess, so no history, so every call alike.
[[COST_PLACED]] void cost_fetch(bool predictor_on) {
    for (const CostRow& r : cost_rows) {
        const Span a = cost_time(r.n);
        const Span b = cost_time(r.n2);
        const uint32_t diff = b.least - a.least;
        const uint32_t model = predictor_on ? r.model : r.model_off;
        print(serial, "  ", r.name.base, ": ", r.units, " units ", a.least, "..", a.most, ", ",
              2u * r.units, " units ", b.least, "..", b.most, ": ", diff, " for the ", r.units,
              " more, the model ", r.units * model, " (", model, " each)", crlf);
        const uint32_t per_site = predictor_on ? 4u : 0u;
        const uint32_t spread_a = per_site * (2u + r.units * r.sites);
        const uint32_t spread_b = per_site * (2u + 2u * r.units * r.sites);
        bench.verdict(r.name.pick(predictor_on),
                      diff == r.units * model && a.most - a.least <= spread_a &&
                          b.most - b.least <= spread_b);
    }
}

/// One trap sequence, 256 trips, interrupts enabled: the console drained,
/// the two lines enabled, mtvec on the letter's table if asked, every
/// change undone after. Trips a tick landed in are dropped.
struct TrapRun {
    Span entry;        ///< first - before
    Span line_entry;   ///< line_first - before
    Span mret;         ///< after - last
    Span tail;         ///< first - line_last
    uint32_t kept = 0;
    uint32_t ticked = 0;
    uint32_t untaken = 0;
    uint32_t mepc = 0;
    bool mepc_steady = true;
};

[[COST_PLACED]] TrapRun cost_trap(void (*f)(void*), bool letter_table) {
    TrapRun r;
    const uint32_t crt_mtvec = read_csr_mtvec();
    console_drain();
    {
        P::CriticalSection cs;
        if (letter_table) {
            __asm__ volatile("csrw mtvec, %0" ::"r"(reinterpret_cast<uint32_t>(cost_vectors) | 3u)
                             : "memory");
        }
        Pfic::clear_pending(Irq::software);
        Pfic::clear_pending(Irq::tim2_cc);
        Pfic::enable(Irq::software);
        Pfic::enable(Irq::tim2_cc);
    }
    for (uint32_t i = 0; i < 256u; ++i) {
        CostStamps s{};
        s.mepc = 0;
        const uint32_t t = Ticker::ticks();
        f(&s);
        if (Ticker::ticks() != t) {
            ++r.ticked;
            continue;
        }
        const bool sw_side = s.first != never_stamped && s.last != never_stamped;
        const bool line_side = s.line_first != never_stamped && s.line_last != never_stamped;
        if (!sw_side && !line_side) {
            ++r.untaken;
            continue;
        }
        ++r.kept;
        if (sw_side) {
            r.entry.add(cycles_between(s.before, s.first));
            r.mret.add(cycles_between(s.last, s.after));
            if (r.kept > 1u && s.mepc != r.mepc) {
                r.mepc_steady = false;
            }
            r.mepc = s.mepc;
        }
        if (line_side) {
            r.line_entry.add(cycles_between(s.before, s.line_first));
            if (sw_side) {
                r.tail.add(cycles_between(s.line_last, s.first));
            } else {
                r.mret.add(cycles_between(s.line_last, s.after));
            }
        }
    }
    {
        P::CriticalSection cs;
        Pfic::disable(Irq::software);
        Pfic::disable(Irq::tim2_cc);
        Pfic::clear_pending(Irq::software);
        Pfic::clear_pending(Irq::tim2_cc);
        __asm__ volatile("csrw mtvec, %0" ::"r"(crt_mtvec) : "memory");
    }
    return r;
}

/// A trap figure: the span less the ruler's word, every kept trip the
/// model's.
[[COST_PLACED]] bool trap_figure(const Named& name, bool predictor_on, const TrapRun& r,
                                 const Span& s, uint32_t ruler, uint32_t model) {
    const uint32_t least = s.least - ruler;
    const uint32_t most = s.most - ruler;
    print(serial, "  ", name.base, ": ", least, "..", most, " over ", r.kept, " trips (", r.ticked,
          " with a tick, dropped; ", r.untaken, " not taken), the model ", model, crlf);
    const bool ok = r.kept > 0u && r.untaken == 0u && least == model && most == model;
    bench.verdict(name.pick(predictor_on), ok);
    return ok;
}

/// The predictor rows: trained 32 calls one way, one call the other way,
/// then 32 calls that way (its trained cost the least of them), sixteen
/// rounds of both directions; the extra a direction costs right after the
/// other's training, against its trained cost.
struct PredictorRow {
    Named name;
    void (*f)(uint32_t);
    uint32_t arg[2];
    const char* way[2];
    uint32_t sites;   ///< the pattern's predicted sites
};

[[COST_PLACED]] void cost_predictor(const PredictorRow& row, bool predictor_on) {
    uint32_t trained[2] = {0xFFFF'FFFFu, 0xFFFF'FFFFu};
    uint32_t flipped_least[2] = {0xFFFF'FFFFu, 0xFFFF'FFFFu};
    uint32_t flipped_most[2] = {0, 0};
    for (uint32_t round = 0; round < 16u; ++round) {
        for (uint32_t dir = 0; dir < 2u; ++dir) {
            for (uint32_t k = 0; k < 32u; ++k) {
                (void)cost_time_once(row.f, row.arg[1u - dir]);   // the other way's training
            }
            const uint32_t fl = cost_time_once(row.f, row.arg[dir]);
            if (fl < flipped_least[dir]) { flipped_least[dir] = fl; }
            if (fl > flipped_most[dir]) { flipped_most[dir] = fl; }
            for (uint32_t k = 0; k < 32u; ++k) {
                const uint32_t c = cost_time_once(row.f, row.arg[dir]);
                if (c < trained[dir]) { trained[dir] = c; }
            }
        }
    }
    bool ok = true;
    uint32_t worst = 0;
    for (uint32_t dir = 0; dir < 2u; ++dir) {
        const bool below = flipped_least[dir] < trained[dir];
        const uint32_t lo = below ? 0u : flipped_least[dir] - trained[dir];
        const uint32_t hi = flipped_most[dir] - trained[dir];
        print(serial, "  ", row.name.base, ", ", row.way[dir], ": trained ", trained[dir],
              ", right after the other way's training +", lo, "..+", hi, crlf);
        if (below) {
            ok = false;   // the trained least must be the least
        }
        if (hi > worst) { worst = hi; }
    }
    // With the predictor on: 4 at each of the window's predicted sites,
    // the pattern's and the call into it and its return - the allowance
    // the Rubato bounds carry, counted as they count it, over a path's
    // sites. With it off: no guess, so no history, so nothing extra.
    const uint32_t allowance = predictor_on ? 4u * (row.sites + 2u) : 0u;
    print(serial, "  ", row.name.base, ": the most the wrong guesses cost, ", worst,
          " cycles at ", row.sites, " site(s) of the pattern; the allowance ", allowance, crlf);
    bench.verdict(row.name.pick(predictor_on), ok && worst <= allowance);
}

constexpr PredictorRow predictor_rows[] = {
    {COST_NAMED("16 32-bit branches over a 32-bit instruction"), cost_pred_b32, {1u, 0u},
     {"taken", "not taken"}, 16},
    {COST_NAMED("16 compressed branches over two compressed instructions"), cost_pred_b16, {1u, 0u},
     {"taken", "not taken"}, 16},
    {COST_NAMED("one compressed branch on a word over two compressed instructions"), cost_pred_one16,
     {1u, 0u}, {"taken", "not taken"}, 1},
    {COST_NAMED("one 32-bit branch half a word off over a 32-bit instruction"), cost_pred_one32h,
     {1u, 0u}, {"taken", "not taken"}, 1},
    {COST_NAMED("one 32-bit branch on a word over a 32-bit instruction"), cost_pred_one32, {1u, 0u},
     {"taken", "not taken"}, 1},
};

[[COST_PLACED]] void cost_pass(bool predictor_on) {
    uint32_t two[2] = {0, 0};
    uint32_t ruler = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        {
            P::CriticalSection cs;
            cost_ruler(two);
        }
        if (cycles_between(two[0], two[1]) < ruler) {
            ruler = cycles_between(two[0], two[1]);
        }
    }
    print(serial, "  the ruler: two loads of CNTL one word apart read ", ruler,
          " cycles apart (the model 4: a 32-bit load is its word)", crlf);
    bench.verdict(COST_NAMED("the ruler's two bare reads are a word, 4 cycles, apart")
                      .pick(predictor_on),
                  ruler == 4u);

    cost_fetch(predictor_on);

    const TrapRun sw = cost_trap(cost_trap_sw, true);
    trap_figure(COST_NAMED("a store pends the software interrupt: to the handler's first instruction"),
                predictor_on, sw, sw.entry, ruler, 20u);
    trap_figure(COST_NAMED("an MRET into code: to its target"), predictor_on, sw, sw.mret, ruler,
                8u);
    const TrapRun line_crt = cost_trap(cost_trap_line, false);
    trap_figure(COST_NAMED("a store pends TIM2's line, through the crt's table"), predictor_on,
                line_crt, line_crt.line_entry, ruler, 20u);
    const TrapRun line_own = cost_trap(cost_trap_line, true);
    trap_figure(COST_NAMED("the same through the letter's table"), predictor_on, line_own,
                line_own.line_entry, ruler, 20u);
    const TrapRun ecall = cost_trap(cost_trap_ecall, false);
    trap_figure(COST_NAMED("an ecall: to its handler's first instruction"), predictor_on, ecall,
                ecall.entry, ruler, 20u);
    const TrapRun unmask = cost_trap(cost_trap_unmask, true);
    trap_figure(COST_NAMED("the unmask that finds the software interrupt pending: to its handler"),
                predictor_on, unmask, unmask.entry, ruler, 20u);
    const TrapRun tail = cost_trap(cost_trap_tail, true);
    trap_figure(COST_NAMED("an MRET into a pending interrupt: to that handler's first instruction"),
                predictor_on, tail, tail.tail, ruler, 24u);

    const uint32_t after_store = reinterpret_cast<uint32_t>(&cost_after_sw_store);
    const uint32_t after_unmask = reinterpret_cast<uint32_t>(&cost_after_unmask);
    print(serial, "  mepc in the handler: ", hex(sw.mepc), " after the store at ",
          hex(after_store - 4u), ", ", hex(unmask.mepc), " after the unmask at ",
          hex(after_unmask - 4u), crlf);
    bench.verdict(COST_NAMED("the trap is taken at the instruction after the store, and after the "
                             "unmask")
                      .pick(predictor_on),
                  sw.mepc_steady && unmask.mepc_steady && sw.mepc == after_store &&
                      unmask.mepc == after_unmask);

    for (const PredictorRow& row : predictor_rows) {
        cost_predictor(row, predictor_on);
    }
    const PredictorRow jalr{COST_NAMED("16 calls through a register"),
                            cost_pred_jalr,
                            {reinterpret_cast<uint32_t>(&cost_leaf_a),
                             reinterpret_cast<uint32_t>(&cost_leaf_b)},
                            {"to one leaf", "to the other"},
                            16};
    cost_predictor(jalr, predictor_on);
}

[[COST_PLACED]] void ti_cost() {
    const uint32_t as_found = read_corecfgr();
    print(serial, "  corecfgr as the crt left it: ", hex(as_found), ", WCH's value 0x1F", crlf);
    bench.verdict("corecfgr holds the crt's 0x1F", as_found == 0x1Fu);
    print(serial, "  with corecfgr as the crt left it:", crlf);
    cost_pass(true);

    print(serial, "  with corecfgr cleared:", crlf);
    uint32_t cleared = 0;
    {
        P::CriticalSection cs;
        write_corecfgr(0);
        cleared = read_corecfgr();
    }
    cost_pass(false);
    {
        P::CriticalSection cs;
        write_corecfgr(as_found);
    }
    const uint32_t restored = read_corecfgr();
    print(serial, "  corecfgr read ", hex(cleared), " cleared and ", hex(restored), " after", crlf);
    bench.verdict("the register took the zero and is back to what the crt wrote",
                  cleared == 0u && restored == as_found);
}

/// main()'s one call into the letter (the placement above says why).
[[gnu::noinline, COST_PLACED]] void register_cost_letter() {
    bench.letter('i', "the fetch and trap cost model the Rubato bounds use", ti_cost);
}

}  // namespace

int main() {
    boot_flags = brio::Rcc::reset_flags();
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the boot story: the reset flags and the signature", ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the STK timebase", tc_ticker);
    bench.letter('d', "delay_us: at least, never early, capped", td_delay);
    bench.letter('e', "the interrupt round trip", te_latency);
    bench.letter('f', "corecfgr, as the crt left it and cleared", tf_corecfgr);
    bench.letter('g', "the tick against the host's clock", tg_bracket);
    bench.letter('h', "what the crt left in the core's registers", th_core);
    register_cost_letter();
    bench.letter('w', "the idle hook against the tick's edge, placed to the cycle", tw_edge);
    bench.letter('k', "the kernel's turn: how many per interrupt, what one costs", tk_turns);
    bench.letter('t', "the runtime's seven functions at every alignment (rt/rt.cpp)", tt_runtime);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", brio::crlf);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
