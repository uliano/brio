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
// The LED on PA0 marks a keystroke for a hand at the desk and is judged on
// nothing: on WCH's QFN20 evaluation board it is the header P4's LED1,
// jumpered to PA0, and it lights with the pad LOW.
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
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

extern "C" void _start();

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;
using Led = Pin<'A', 0>;

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
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  5 ms with interrupts masked advanced the tick by ", through_mask, " ms", crlf);
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

void banner() {
    print(serial, crlf, "test_x035_platform on ", device::part_name, " - the platform", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

// The vectors this program owns.
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void software_handler() {
    sw_entry_cycles = brio::stk()->CNTL;
    brio::Pfic::clear_pending(brio::Irq::software);
    sw_hits = sw_hits + 1u;
    sw_exit_cycles = brio::stk()->CNTL;
}

int main() {
    boot_flags = brio::Rcc::reset_flags();
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(true);
    brio::enable_interrupts();

    bench.letter('a', "the boot story: the reset flags and the signature", ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the STK timebase", tc_ticker);
    bench.letter('d', "delay_us: at least, never early, capped", td_delay);
    bench.letter('e', "the interrupt round trip", te_latency);
    bench.letter('f', "corecfgr, as the crt left it and cleared", tf_corecfgr);
    bench.letter('g', "the tick against the host's clock", tg_bracket);
    bench.letter('h', "what the crt left in the core's registers", th_core);

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
