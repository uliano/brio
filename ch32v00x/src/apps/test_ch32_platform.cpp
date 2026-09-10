// test_ch32_platform - the reference bench suite for the CH32V00x's
// PLATFORM: the running half (the mstatus.MIE critical section, the
// WFE-shaped idle hook, the STK timebase, delay_us on its counter, the
// interrupt round trip and what the hardware prologue buys it) and the
// failing half - ch32v00x/reset.hpp (the reset flags as the history they
// are, the software reset, the panic breadcrumb across a real reset,
// the fault vector's own record).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the WCH-Link's own serial (USART1 on
// PD5/PD6) and every clock this suite measures is inside the chip. The
// LED on PC0 marks a keystroke for a hand at the desk and is judged on
// nothing.
//
// What is exercised, letter by letter:
//   a  the boot story: RCC_RSTSCKR read as the ACCUMULATING history it
//      is, PINRSTF naming the pin alone (this family's flag is NOT the
//      STM32's catch-all: a software reset leaves SFTRSTF standing
//      without it), and take_flags() leaving the register clean
//   b  the critical section: mstatus.MIE saved and restored, nesting,
//      and the idle hook - a WFE, not a WFI, on this core - returning on
//      the tick with interrupts enabled
//   c  the STK timebase: ticks/millis/secs/now coherence, the compare
//      the ticker programmed, and the CNT-delta accumulation delay.hpp
//      is built on checked against the interrupt count over 200 wraps
//   d  delay_us: at least, never early, the cap, the refusal
//   e  THE INTERRUPT ROUND TRIP, in HCLK cycles on the STK counter: a
//      line raised by hand (the software interrupt, PFIC IPSR) with the
//      cycle count read at the raise, at the handler's first and last
//      statements, and when the raiser sees the handler done. Entry,
//      body, exit and the whole trip are what the hardware
//      prologue/epilogue is worth: the image says which way it was
//      built (CH32V00X_HPE), and the same letter on both builds is the
//      measurement the project's default rests on.
//
//   i  (by name only) THREE REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session a tool can judge from a
//      single capture. Run it with
//          brio run <board> i --app test_ch32_platform --expect="->" --timeout 60
//      Legs: a software reset (SFTRSTF at the next boot), a panic through
//      ResetReporter (the breadcrumb read back), and a deliberate fault
//      through ebreak with no debugger attached (the fault vector's
//      own record, kernel_fault).
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/reset.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/panic.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter i lives in: .noinit, so the crt neither loads nor
// zeroes it, and inline because the platform's own breadcrumb is a
// static inline member in the same section (gcc gives both a COMDAT
// group; a plain definition next to them fails the link with a section
// type conflict). Its magic word is not decoration: nothing promises
// SRAM across a reset, so every read of this object is guarded.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5A06;

struct Token {
    uint16_t magic;
    uint8_t leg;        ///< which reset we are waiting for (0 = none pending)
    uint8_t code;       ///< the PanicCode written before the reset
    uint8_t context;    ///< its context byte
    uint16_t pass;      ///< letter i's tally so far
    uint16_t fail;
    uint32_t flags_before;
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

TestBench<Serial> bench;

// What this boot was told, sampled once in main() before anything can
// disturb it.
uint32_t boot_flags = 0;
std::optional<PanicRecord> boot_record;

// The STK counter as a cycle counter: it runs at HCLK and reloads every
// tick, so two readings less than a tick apart are a cycle count with
// the reload folded in.
uint32_t cycles_now() { return stk()->CNT; }
uint32_t cycles_between(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMP + 1u;
    return to >= from ? to - from : to + period - from;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);   // the last byte's own time on the wire
}

// ---------------------------------------------------------------------------
// a - the boot story
// ---------------------------------------------------------------------------
void ta_boot() {
    print(serial, "  RSTSCKR flags at boot: ", hex(boot_flags), crlf);
    print(serial, "    power=", (boot_flags & ResetFlag::power) != 0u,
          " pin=", (boot_flags & ResetFlag::pin) != 0u,
          " software=", (boot_flags & ResetFlag::software) != 0u,
          " iwdg=", (boot_flags & ResetFlag::independent_watchdog) != 0u,
          " wwdg=", (boot_flags & ResetFlag::window_watchdog) != 0u, crlf);
    bench.verdict("some reset flag is standing at boot (they accumulate "
                  "until RMVF, and a power-on sets PORRSTF)",
                  boot_flags != 0u);
    // The board arrives here from the probe's post-programming reset,
    // which is a system reset: SFTRSTF stands, and PINRSTF does not.
    bench.verdict("PINRSTF names the pin alone: a software reset does not "
                  "raise it (unlike the STM32's catch-all)",
                  (boot_flags & ResetFlag::software) == 0u ||
                      (boot_flags & ResetFlag::pin) == 0u);

    const uint32_t before = Reset::flags();
    const uint32_t taken = Reset::take_flags();
    const uint32_t after = Reset::flags();
    print(serial, "  take_flags(): before=", hex(before), " taken=", hex(taken),
          " after=", hex(after), crlf);
    bench.verdict("flags() is non-destructive", before == boot_flags);
    bench.verdict("take_flags() returns what stood", taken == before);
    bench.verdict("and leaves the register clean", after == 0u);
    bench.verdict("RMVF is back at zero afterwards (a plain bit, not a strobe)",
                  (rcc()->RSTSCKR & rstsckr_rmvf) == 0u);
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
        const uint32_t spin0 = cycles_now();
        uint32_t spun = 0;
        uint32_t last = spin0;
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
          " idle() call(s) covered the ", idle_cycles / (SysClock::hz / 1'000'000u),
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
    const uint32_t cmp = stk()->CMP;
    print(serial, "  STK CTLR=", hex(stk()->CTLR), " CMP=", cmp,
          " (", SysClock::hz / Ticker::ticks_per_second - 1u, " expected)", crlf);
    bench.verdict("the compare is Clock::hz / tps - 1",
                  cmp == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on HCLK, reloading, with its interrupt armed",
                  (stk()->CTLR & (stk_ste | stk_stie | stk_stclk | stk_stre)) ==
                      (stk_ste | stk_stie | stk_stclk | stk_stre));

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
    // interrupt count over 200 wraps: an error there is a whole period,
    // not a rounding.
    const uint32_t target = 200;
    const uint32_t period = cmp + 1u;
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
          expected, " expected, error ", err, " cycles", crlf);
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

    const uint32_t c0 = cycles_now();
    const bool one = delay_us(clock, 100);
    const uint32_t spent = cycles_between(c0, cycles_now());
    print(serial, "  delay_us(100) spent ", spent, " cycles (4800 asked)", crlf);
    bench.verdict("100 us is at least 4800 cycles at 48 MHz, and under 5200",
                  one && spent >= 4800u && spent < 5200u);

    bench.verdict("a wait of one tick period (1000 us) is refused",
                  !delay_us(clock, 1000));
    bench.verdict("and so is anything longer", !delay_us(clock, 50'000));
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
    print(serial, "  built with the hardware prologue/epilogue (CH32V00X_HPE=ON)", crlf);
#else
    print(serial, "  built with gcc's own prologue/epilogue (CH32V00X_HPE=OFF)", crlf);
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
        while (sw_hits == before) {
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
// i - three real resets
// ---------------------------------------------------------------------------
void ti_leg_start(uint8_t leg) {
    token.magic = token_magic;
    token.leg = leg;
    token.flags_before = boot_flags;
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

    switch (leg) {
        case 1:
            ti_judge("leg 1: SFTRSTF stands after Reset::software()",
                     (boot_flags & ResetFlag::software) != 0u);
            ti_judge("leg 1: and PINRSTF is NOT raised by it (the pin flag "
                     "names the pin alone on this family)",
                     (boot_flags & ResetFlag::pin) == 0u);
            ti_judge("leg 1: no breadcrumb - a plain reset writes none",
                     !boot_record.has_value());
            print(serial, "  leg 2: panic<ResetReporter>(queue_overflow, 7) -> the "
                          "breadcrumb at the next boot", crlf);
            Reset::clear_flags();
            ti_leg_start(2);
            panic<P, ResetReporter>(PanicCode::queue_overflow, 7);
        case 2:
            ti_judge("leg 2: the reporter reset the board (SFTRSTF)",
                     (boot_flags & ResetFlag::software) != 0u);
            ti_judge("leg 2: the breadcrumb crossed the reset",
                     boot_record.has_value());
            ti_judge("leg 2: with the code and context panic() was given",
                     boot_record.has_value() &&
                         boot_record->code == static_cast<uint8_t>(PanicCode::queue_overflow) &&
                         boot_record->context == 7u);
            print(serial, "  leg 3: ebreak with no debugger -> the fault vector's "
                          "record (kernel_fault, context 0x51)", crlf);
            Reset::clear_flags();
            token.context = 0x51;
            ti_leg_start(3);
            P::break_here();
            for (;;) {
            }
        case 3:
            ti_judge("leg 3: the fault body reset the board (SFTRSTF)",
                     (boot_flags & ResetFlag::software) != 0u);
            ti_judge("leg 3: the fault vector wrote its own record",
                     boot_record.has_value() &&
                         boot_record->code == static_cast<uint8_t>(PanicCode::kernel_fault));
            ti_judge("leg 3: with the context the app handed the body",
                     boot_record.has_value() && boot_record->context == 0x51u);
            bench.end_letter();
            break;
        default:
            print(serial, "  unknown leg ", leg, crlf);
            break;
    }
}

void banner() {
    print(serial, crlf, "test_ch32_platform - CH32V006K8 (clk=48 MHz PLL, tick=STK 1000 Hz)",
          crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The software interrupt: the first thing it does is read the cycle
/// counter, which is letter e's whole measurement.
extern "C" BRIO_CH32_INTERRUPT void software_handler() {
    sw_entry_cycles = brio::stk()->CNT;
    brio::Pfic::clear_pending(brio::Irq::software);
    sw_hits = sw_hits + 1u;
    sw_exit_cycles = brio::stk()->CNT;
}

/// A crash becomes a note the next boot can read (letter i, leg 3).
extern "C" BRIO_CH32_INTERRUPT void fault_handler() {
    brio::fault_reset<P>(token.magic == token_magic ? token.context : 0u);
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
    bench.letter('i', "THREE REAL RESETS (reboots the board)", ti_resets, false);

    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL48" : "FAILED", " tick=",
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
        bench.prompt();
    }
}
