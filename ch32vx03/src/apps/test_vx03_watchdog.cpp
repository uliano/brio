// test_vx03_watchdog - the reference bench suite for the CH32V203's two
// WATCHDOGS: ch32vx03/watchdog.hpp over RM ch. 7 (the independent one)
// and ch. 8 (the window one).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT A WATCHDOG COSTS TO TEST. Starting either one is a one-way door:
// the independent watchdog cannot be stopped by anything but a reset,
// and the window one only by closing its clock gate or resetting the
// block. So `z` holds every letter that costs the board NOTHING - the
// registers, the key protection, the free-running counter, the window
// predicate, the early-wake-up flag and both time-out arithmetics, all
// measured with neither watchdog armed - and THE RESETS ARE TWO LETTERS
// BY NAME, w and v, which reboot the board once per leg and resume from
// a .noinit token:
//     brio run <board> w --app test_vx03_watchdog --expect="->" --timeout 60
//     brio run <board> v --app test_vx03_watchdog --expect="->" --timeout 60
// Their legs are the ones only a reset can prove: the independent
// watchdog's time-out MEASURED against the tick and IWDGRSTF at the next
// boot, the window one's time-out and WWDGRSTF, a refresh BEFORE the
// window opening resetting the board on the spot, and the early wake-up
// interrupt running one tick before the reset it did not prevent.
//
// NOTHING IS WIRED AND NOTHING IS PRESSED. Both blocks are internal:
// the independent one counts the LSI, whose rate is a part fact with a
// spread of better than two to one (device::lsi_min_hz..lsi_max_hz), so
// what this suite MEASURES is that rate, by timing the watchdog against
// the core's own tick - and every nominal time-out is printed beside
// the measured one.
//
// What is exercised, letter by letter:
//   a  THE WINDOW WATCHDOG'S BLOCK, with nothing armed: what its clock
//      gate does and does not silence, the reset values, the
//      configuration read back, and the counter that does NOT run
//      while WDGA is clear - against 8.2.1's own sentence
//   b  ITS COUNTER ARMED and kept alive by hand: the tick rate at
//      three prescalers timed against the STK, the window predicate
//      over the moving counter, the early-wake-up flag caught and
//      answered, and the block's reset line as the way back
//   c  THE INDEPENDENT WATCHDOG'S REGISTERS, with the watchdog NOT
//      started: they need their own oscillator running before a keyed
//      write arrives at all, the key protection, the update flags, and
//      running() false while the program still owns the LSI
//   d  THE ARITHMETIC of both blocks against what the part states: the
//      LSI's three corners, the twelve-bit reload, the seven-bit
//      counter and the four PCLK1 prescalers
//   w  (by name) THE INDEPENDENT WATCHDOG'S RESET: armed for a fifth of
//      a second, refreshed for a while, then left - the time-out
//      measured against the tick and IWDGRSTF read at the next boot
//   v  (by name) THE WINDOW WATCHDOG'S THREE RESETS: the time-out from
//      a refresh, a refresh BEFORE the window opens, and the early
//      wake-up interrupt taken one tick before the reset
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/reset.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "ch32vx03/watchdog.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32vx03Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token the two by-name letters live in: .noinit, so the crt
// neither loads nor zeroes it, and inline because the platform's own
// breadcrumb is a static inline member in the same section. Its magic
// word is not decoration - nothing promises SRAM across a reset.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5A57;

struct Token {
    uint16_t magic;
    char letter;      ///< which by-name letter is running ('\0' = none)
    uint8_t leg;      ///< which reset we are waiting for
    uint16_t pass;    ///< the letter's tally so far
    uint16_t fail;
    /// What the legs measure: the last mark written before the reset.
    /// VOLATILE, and that is not decoration - the loop that writes them
    /// never exits, so a compiler free to sink the stores past its end
    /// would leave both at zero and the measurement would be a zero
    /// (measured, exactly that way).
    volatile uint32_t elapsed_ms;
    volatile uint32_t marks;        ///< how many marks the loop managed
    volatile uint32_t handler_calls;   ///< what the early-wake-up vector counted
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial> bench;

/// The reset flags as this boot found them, read and cleared once.
uint32_t boot_flags = 0;

/// What the window watchdog's vector counted (letter v's third leg).
volatile uint32_t ewi_calls = 0;

/// PCLK1 as the clock task computed it - the window watchdog's own
/// clock, and the one number its arithmetic needs.
constexpr uint32_t pclk1 = SysClock::pclk1_hz;
constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000u;

/// The core's counter as a stopwatch, the ruler both blocks are weighed
/// against: STK counts to its reload (one tick) and starts again, so a
/// span is accumulated poll by poll with one period folded in across
/// each wrap.
class Stopwatch {
public:
    Stopwatch() { start(); }

    void start() {
        last_ = stk()->CNTL;
        acc_ = 0;
    }

    uint32_t cycles() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_;
    }

    uint32_t us() { return cycles() / ticks_per_us; }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

void wait_us(uint32_t us) {
    Stopwatch w;
    while (w.us() < us) {
    }
}

/// The window watchdog back to the state a cold boot leaves, without
/// resetting the board: the block's own reset line, which 8.2.1 names
/// as the way to stop it.
void wwdg_off() {
    Wwdg::init();
    Wwdg::clear_flag();
    ewi_calls = 0;
    Pfic::disable(Irq::wwdg);
    Pfic::clear_pending(Irq::wwdg);
}

void print_flags(uint32_t f) {
    print(serial, "  reset flags:", (f & ResetFlag::power) != 0u ? " POR" : "",
          (f & ResetFlag::pin) != 0u ? " PIN" : "",
          (f & ResetFlag::software) != 0u ? " SFT" : "",
          (f & ResetFlag::independent_watchdog) != 0u ? " IWDG" : "",
          (f & ResetFlag::window_watchdog) != 0u ? " WWDG" : "",
          (f & ResetFlag::low_power) != 0u ? " LPWR" : "", crlf);
}

// ===========================================================================
// a - the window watchdog's block, with nothing armed
// ===========================================================================
void ta_wwdg_block() {
    // THE CLOCK GATE IS HALF A SILENCE on this family: with WWDGEN
    // clear the registers still READ (their reset values come back,
    // where another family's answer nothing) and a write is DROPPED.
    // What the gate really holds is the block's clock, which is what
    // 8.2.1 offers it for - a way to suspend a watchdog whose enable
    // bit is one-way.
    wwdg_off();
    Wwdg::bus_clock(false);
    const bool closed = !Wwdg::bus_clock();
    const uint16_t ctlr_closed = Wwdg::ctlr();
    const uint16_t cfgr_closed = Wwdg::cfgr();
    Wwdg::regs().CFGR = 0x006Au;
    const uint16_t written_closed = Wwdg::cfgr();
    Wwdg::bus_clock(true);
    const uint16_t after_open = Wwdg::cfgr();
    Wwdg::reset();
    const uint16_t ctlr_reset = Wwdg::ctlr();
    const uint16_t cfgr_reset = Wwdg::cfgr();
    print(serial, "  gate closed: CTLR=", hex(ctlr_closed), " CFGR=", hex(cfgr_closed),
          ", a write reads back ", hex(written_closed), "; gate open: ", hex(after_open),
          "; after the block's reset: CTLR=", hex(ctlr_reset), " CFGR=", hex(cfgr_reset),
          crlf);
    bench.verdict("with the clock gate CLOSED the registers still READ their reset value "
                  "0x007F and a write is dropped - the gate is on the block's clock, so "
                  "what answers is the read path alone",
                  closed && ctlr_closed == 0x7Fu && cfgr_closed == 0x7Fu &&
                      written_closed == 0x7Fu && after_open == 0x7Fu);
    bench.verdict("and the block's own reset line puts both back to 0x007F - the way to "
                  "disarm a watchdog whose WDGA is one-way (8.2.1)",
                  ctlr_reset == 0x7Fu && cfgr_reset == 0x7Fu);

    // The configuration, read back field by field.
    const bool wrote = Wwdg::configure({.prescaler = WwdgPrescaler::div4,
                                        .window = 0x5A,
                                        .early_wakeup = false});
    const bool fields = Wwdg::prescaler() == WwdgPrescaler::div4 && Wwdg::window() == 0x5Au &&
                        !Wwdg::early_wakeup_enabled() && !Wwdg::enabled();
    const bool refused = !Wwdg::configure({.window = 0x20});
    print(serial, "  CFGR written /4 window 0x5A: reads ", hex(Wwdg::cfgr()),
          ", WDGA still clear", crlf);
    bench.verdict("the prescaler, the window and the early-wake-up enable read back as "
                  "written, and a window at or below 0x3F is REFUSED - no counter value "
                  "is ever inside one",
                  wrote && fields && refused);

    // THE COUNTER DOES NOT FREE-RUN, against 8.2.1's own sentence and
    // exactly like the sister family's block: with WDGA clear it holds
    // what the last write put in it.
    wwdg_off();
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div1, .window = 0x7F});
    Wwdg::refresh(0x7F);
    const uint8_t first = Wwdg::counter();
    wait_us(20000);
    const uint8_t later = Wwdg::counter();
    print(serial, "  with WDGA CLEAR: the counter read ", first, " then ", later,
          " twenty milliseconds later - ", 20000u / (4096u * 1000000u / pclk1),
          " ticks of its own clock", crlf);
    bench.verdict("the seven-bit counter does NOT count with the watchdog disarmed - the "
                  "chapter says it free-runs and this silicon holds it, as the sister "
                  "family's does",
                  first == 0x7Fu && later == 0x7Fu);
    wwdg_off();
}

// ===========================================================================
// b - the armed counter: its rate, its window, its early wake-up
// ===========================================================================

/// Time the armed counter's fall from 0x7F to `down_to`, refreshing it
/// the moment it arrives - so the watchdog never reaches 0x3F and the
/// board is never reset. Microseconds, zero if it fell too far.
uint32_t fall_us(WwdgPrescaler p, uint8_t down_to) {
    (void)Wwdg::configure({.prescaler = p, .window = 0x7F});
    Wwdg::refresh(0x7F);
    Stopwatch w;
    while (Wwdg::counter() > down_to && w.us() < 200u * 1000u) {
    }
    const uint32_t seen = w.us();
    const bool safe = Wwdg::counter() > 0x40u;
    Wwdg::refresh(0x7F);
    return safe ? seen : 0u;
}

void tb_wwdg_armed() {
    wwdg_off();
    // ARMED, AND NOT ONE CONSOLE LINE WHILE IT IS. WDGA is one-way in
    // software, the block's whole time-out at its fastest prescaler is
    // under four milliseconds, and a line of this console at 115200 is
    // seven - so every measurement below is taken first and printed
    // afterwards, with the block's reset line disarming the watchdog in
    // between. (Printing between two measurements is how this letter
    // rebooted the board the first time.)
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div1, .window = 0x7F});
    Wwdg::start(0x7F);
    const bool armed = Wwdg::enabled();
    const uint8_t at_start = Wwdg::counter();
    wait_us(500);
    const uint8_t after = Wwdg::counter();
    Wwdg::refresh(0x7F);

    // ITS RATE, at three of the four prescalers: 0x7F down to 0x50 is
    // 47 ticks of PCLK1 / 4096 / 2^WDGTB.
    constexpr WwdgPrescaler rates[] = {WwdgPrescaler::div1, WwdgPrescaler::div2,
                                       WwdgPrescaler::div8};
    uint32_t seen[3] = {0, 0, 0};
    uint32_t want[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3u; ++i) {
        seen[i] = fall_us(rates[i], 0x50);
        want[i] = 47u * wwdg_cycles_per_tick(rates[i]) / (pclk1 / 1000000u);
    }

    // THE WINDOW PREDICATE over the moving counter. THE ORDER IS THE
    // POINT: the counter is refreshed while the window is still wide
    // open, and the window is narrowed AFTERWARDS - narrowing it first
    // and then refreshing is a refresh outside the window, which is
    // this watchdog's other way of resetting the board.
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F});
    Wwdg::refresh(0x7F);
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x60});
    const bool too_early = !Wwdg::in_window();
    Stopwatch t;
    while (Wwdg::counter() > 0x60u && t.us() < 200u * 1000u) {
    }
    const bool inside = Wwdg::in_window();
    const uint8_t at_window = Wwdg::counter();
    Wwdg::refresh(0x7F);

    // THE EARLY WAKE-UP FLAG, caught and answered: EWIF is raised at
    // 0x40 whether or not its interrupt is enabled (8.3.3), and what
    // follows it is ONE TICK - at /8 that is nearly half a millisecond,
    // which a polling loop has no trouble with.
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F});
    Wwdg::clear_flag();
    Wwdg::refresh(0x7F);
    Stopwatch e;
    while (!Wwdg::flag() && e.us() < 200u * 1000u) {
    }
    const uint32_t to_flag = e.us();
    const bool raised = Wwdg::flag();
    Wwdg::refresh(0x7F);              // the answer, well inside its one tick
    Wwdg::clear_flag();
    const bool cleared = !Wwdg::flag();
    const uint32_t flag_want = 63u * wwdg_cycles_per_tick(WwdgPrescaler::div8) /
                               (pclk1 / 1000000u);

    // The way back, which is what let this letter arm it at all.
    Wwdg::reset();
    const bool disarmed = !Wwdg::enabled();
    wait_us(20000);
    const bool still_here = !Wwdg::enabled();

    // Now the console.
    print(serial, "  WDGA set: the counter left ", at_start, " and was at ", after,
          " half a millisecond later", crlf);
    bench.verdict("the counter starts falling when the watchdog is armed, and that is the "
                  "only time it moves", armed && after < at_start);
    uint8_t rates_ok = 0;
    for (uint8_t i = 0; i < 3u; ++i) {
        if (seen[i] * 100u > want[i] * 97u && seen[i] * 97u < want[i] * 100u) {
            ++rates_ok;
        }
        print(serial, "  WDGTB /", 1u << static_cast<uint8_t>(rates[i]),
              ": 0x7F to 0x50 in ", seen[i], " us, ", want[i], " us from PCLK1 = ",
              pclk1 / 1000000u, " MHz", crlf);
    }
    bench.verdict("the counter's tick is the peripheral clock over 4096 and over WDGTB, "
                  "within three per cent of the core's own counter at three of the four "
                  "prescalers", rates_ok == 3u);
    print(serial, "  a window at 0x60: refused at 0x7F, allowed at ", at_window, crlf);
    bench.verdict("in_window() refuses while the counter is above the window and allows it "
                  "between the window and 0x3F", too_early && inside && at_window <= 0x60u);
    print(serial, "  EWIF with its interrupt DISABLED: raised ", to_flag,
          " us after the refresh (", flag_want, " us nominal), then cleared by writing zero",
          crlf);
    bench.verdict("the early-wake-up flag is raised when the counter reaches 0x40 whether "
                  "or not the interrupt is enabled, sixty-three ticks after a refresh, and "
                  "a zero clears it",
                  raised && cleared && to_flag * 100u > flag_want * 97u &&
                      to_flag * 97u < flag_want * 100u);
    bench.verdict("the block's reset line clears WDGA and the board is still here - the "
                  "only software way out of an armed window watchdog",
                  disarmed && still_here);
    wwdg_off();
}

// ===========================================================================
// c - the independent watchdog's registers, unarmed
// ===========================================================================
void tc_iwdg_registers() {
    // ITS REGISTERS NEED THEIR OSCILLATOR. The update the unlock key
    // opens is a crossing into the LSI's domain, so with the LSI
    // stopped nothing arrives and both update flags stand for ever -
    // which the chapter never says.
    Rcc::lsi_stop();
    const bool lsi_off = !Rcc::lsi_ready();
    const uint16_t reload_before = static_cast<uint16_t>(Iwdg::regs().RLDR & 0x0FFFu);
    (void)Iwdg::configure({.prescaler = IwdgPrescaler::div16, .reload = 0x0123});
    const uint16_t status_stopped = Iwdg::status();
    const bool never_settles = !Iwdg::wait_idle(iwdg_pvu | iwdg_rvu, 20000);
    const uint16_t reload_stopped = static_cast<uint16_t>(Iwdg::regs().RLDR & 0x0FFFu);
    print(serial, "  with the LSI stopped: RLDR was ", reload_before,
          ", a keyed write of 0x123 leaves ", reload_stopped, " and STATR=",
          hex(status_stopped), " standing", crlf);
    bench.verdict("with the LSI stopped a keyed write NEVER ARRIVES and PVU/RVU stand set - "
                  "the registers live in the oscillator's domain, and the chapter does not "
                  "say so",
                  lsi_off && never_settles && reload_stopped == reload_before);

    // With the oscillator running, the same write lands and the flags
    // go. This is what a program does when it wants the setting ready
    // before it commits - and what arm() does by starting the watchdog,
    // which forces the LSI on by itself.
    const bool lsi_up = Rcc::lsi_start();
    const bool settled = Iwdg::wait_idle();
    (void)Iwdg::configure({.prescaler = IwdgPrescaler::div16, .reload = 0x0123});
    const bool settled_again = Iwdg::wait_idle();
    const uint16_t reload_running = Iwdg::reload();
    const IwdgPrescaler p = Iwdg::prescaler();
    print(serial, "  with the LSI running: RLDR reads ", reload_running, " and PSCR /",
          iwdg_prescaler_divider(p), ", STATR=", hex(Iwdg::status()), crlf);
    bench.verdict("with the oscillator running the same write arrives, both flags clear "
                  "within their bounded wait, and the prescaler and reload read back whole",
                  lsi_up && settled && settled_again && reload_running == 0x0123u &&
                      p == IwdgPrescaler::div16);

    // THE KEY IS THE ONLY WAY IN, AND THE UNLOCK STANDS UNTIL ANOTHER
    // KEY CLOSES IT: after configure() the window is still open, so a
    // naked store lands; the refresh key re-locks it (7.3.1's "a write
    // access to this register with a different value breaks the
    // sequence"), and the same store then changes nothing. Both halves
    // are measured with the oscillator running, so what refuses is the
    // LOCK and not the clock.
    Iwdg::regs().RLDR = 0x0456;
    (void)Iwdg::wait_idle();
    const uint16_t while_open = Iwdg::reload();
    Iwdg::refresh();                       // the key that closes the window
    Iwdg::regs().RLDR = 0x0789;
    (void)Iwdg::wait_idle();
    const uint16_t after_relock = Iwdg::reload();
    print(serial, "  with the window still open a naked write of 0x456 lands (RLDR reads ",
          while_open, "); after a refresh, a naked write of 0x789 leaves ", after_relock,
          crlf);
    bench.verdict("the unlock key opens the two registers until ANOTHER key is written - a "
                  "refresh closes them again, and a store with no key before it then "
                  "changes nothing",
                  while_open == 0x0456u && after_relock == 0x0456u);

    // AND IT IS NOT RUNNING. There is no bit that says so; what says it
    // is the LSI, which the silicon forces on when the watchdog starts
    // (3.3.5.4) and which the PROGRAM is holding on here.
    const bool running = Iwdg::running();
    print(serial, "  the LSI: LSION ", Rcc::lsi_enabled() ? "set by this program" : "clear",
          ", LSIRDY ", Rcc::lsi_ready() ? "set" : "clear", " - the watchdog reads as ",
          running ? "RUNNING" : "not running", crlf);
    bench.verdict("writing the prescaler and the reload does NOT start the watchdog, and "
                  "running() says so: the oscillator is on because this program asked, not "
                  "because the watchdog forced it",
                  !running);

    // The board back as it was found: the reset values, the LSI off.
    (void)Iwdg::configure({.prescaler = IwdgPrescaler::div4, .reload = 0x0FFF});
    (void)Iwdg::wait_idle();
    Rcc::lsi_stop();
}

// ===========================================================================
// d - the arithmetic of both blocks
// ===========================================================================
void td_arithmetic() {
    // The independent watchdog's, at the part's three LSI corners: the
    // same setting is a different time-out at each, which is why the
    // helpers take the rate and no driver of this stratum pretends to
    // know it.
    constexpr IwdgPrescaler p = IwdgPrescaler::div32;
    constexpr uint16_t reload = 249;
    const uint32_t at_min = iwdg_timeout_ms(p, reload, device::lsi_min_hz);
    const uint32_t at_typ = iwdg_timeout_ms(p, reload, device::lsi_typ_hz);
    const uint32_t at_max = iwdg_timeout_ms(p, reload, device::lsi_max_hz);
    print(serial, "  /32 with a reload of 249: ", at_max, " ms at the LSI's fast corner (",
          device::lsi_max_hz / 1000u, " kHz), ", at_typ, " ms typical, ", at_min,
          " ms at the slow one (", device::lsi_min_hz / 1000u, " kHz)", crlf);
    bench.verdict("the independent watchdog's time-out is the LSI's to within the part's "
                  "own rated spread - better than two to one here, and the reason a "
                  "program refreshes against the FAST corner",
                  at_max < at_typ && at_typ < at_min && at_min > at_max * 2u);

    // The reload that reaches a time asked for, never early.
    const uint16_t need = iwdg_reload_for(p, 200'000u, device::lsi_max_hz);
    const uint32_t reached = iwdg_timeout_us(p, need, device::lsi_max_hz);
    const uint16_t too_long = iwdg_reload_for(IwdgPrescaler::div4, 60'000'000u,
                                              device::lsi_typ_hz);
    print(serial, "  200 ms at the fast corner wants a reload of ", need, " (", reached,
          " us), and a minute at /4 is refused: ", hex(too_long), crlf);
    bench.verdict("a reload is the SMALLEST whose time-out reaches what was asked, and a "
                  "request past the twelve-bit field is refused rather than truncated",
                  reached >= 200'000u && need <= 0x0FFFu && too_long == 0xFFFFu);

    // The window watchdog's, against the clock task's own PCLK1.
    const uint32_t full = wwdg_timeout_us(pclk1, WwdgPrescaler::div8, 0x7F);
    const uint32_t wait_for_window = wwdg_window_wait_us(pclk1, WwdgPrescaler::div8, 0x7F,
                                                         0x60);
    print(serial, "  the window watchdog at /8 from 0x7F: ", full,
          " us to the reset, and ", wait_for_window, " us before a window at 0x60 opens",
          crlf);
    bench.verdict("its time-out is (T[5:0] + 1) ticks of PCLK1 / 4096 / 2^WDGTB and the "
                  "window is the wait before a refresh is legal - both computed from the "
                  "clock task's own PCLK1",
                  full == 64u * wwdg_cycles_per_tick(WwdgPrescaler::div8) * 1000u /
                              (pclk1 / 1000u) &&
                      wait_for_window == 31u * wwdg_cycles_per_tick(WwdgPrescaler::div8) *
                                             1000u / (pclk1 / 1000u));
}

// ===========================================================================
// w - the independent watchdog's reset, by name
// ===========================================================================
void mark_leg(char letter, uint8_t leg) {
    token.magic = token_magic;
    token.letter = letter;
    token.leg = leg;
    token.elapsed_ms = 0;
    token.marks = 0;
    token.handler_calls = 0;
    // The console must be empty before the board goes: a byte still in
    // the transmit register is a byte lost.
    while (!Serial::tx_idle()) {
    }
}

void judge(const char* what, bool ok) {
    bench.verdict(what, ok);
    if (ok) {
        ++token.pass;
    } else {
        ++token.fail;
    }
}

/// Spin until the watchdog resets the board, writing how long it has
/// been into the token as it goes: what survives in there is the
/// TIME-OUT, measured against the kernel's own tick.
[[noreturn]] void spin_until_reset() {
    const uint32_t start = Ticker::ticks();
    for (;;) {
        token.elapsed_ms = Ticker::ticks() - start;
        token.marks = token.marks + 1u;
    }
}

void tw_iwdg_reset() {
    token.pass = 0;
    token.fail = 0;
    print(serial, "  leg 1: the independent watchdog armed for a fifth of a second, "
                  "refreshed ten times, then left", crlf);
    Reset::clear_flags();
    (void)Iwdg::arm({.prescaler = IwdgPrescaler::div32, .reload = 249});
    // Ten refreshes at a tenth of the nominal time-out, to prove the
    // refresh is what keeps the board alive.
    for (uint8_t i = 0; i < 10u; ++i) {
        wait_us(20000);
        Iwdg::refresh();
    }
    print(serial, "  it survived ten refreshes; now nothing refreshes it", crlf);
    // THE LAST REFRESH GOES AFTER THE CONSOLE IS DRAINED: a line at
    // 115200 is seven milliseconds, and what is being measured is the
    // span from a refresh to the reset.
    mark_leg('w', 1);
    Iwdg::refresh();
    spin_until_reset();
}

void tw_resume() {
    const uint32_t ms = token.elapsed_ms;
    print(serial, crlf, "-> back from the independent watchdog's reset after ", ms,
          " ms (", token.marks, " marks)", crlf);
    print_flags(boot_flags);
    judge("the independent watchdog reset the board, and IWDGRSTF says so at the next boot",
          (boot_flags & ResetFlag::independent_watchdog) != 0u);
    judge("and no other source claims it - not the window watchdog, not a software reset",
          (boot_flags & (ResetFlag::window_watchdog | ResetFlag::software)) == 0u);
    const uint32_t fast = iwdg_timeout_ms(IwdgPrescaler::div32, 249, device::lsi_max_hz);
    const uint32_t slow = iwdg_timeout_ms(IwdgPrescaler::div32, 249, device::lsi_min_hz);
    print(serial, "  the part's rated corners for that setting: ", fast, " to ", slow,
          " ms; the LSI it implies is ", 250u * 32u * 1000u / (ms == 0u ? 1u : ms),
          " Hz", crlf);
    judge("the measured time-out falls inside the part's own rated LSI spread - which is "
          "what the arithmetic promises and all it promises",
          ms >= fast && ms <= slow);
    judge("the board ran for ten refreshes before that, so a refreshed watchdog does not "
          "reset anything", token.marks > 0u);
    bench.end_letter();
}

// ===========================================================================
// v - the window watchdog's three resets, by name
// ===========================================================================
void tv_wwdg_reset() {
    token.pass = 0;
    token.fail = 0;
    print(serial, "  leg 1: the window watchdog armed at /8 with no window, refreshed "
                  "ten times, then left", crlf);
    Reset::clear_flags();
    wwdg_off();
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F});
    Wwdg::start(0x7F);
    for (uint8_t i = 0; i < 10u; ++i) {
        wait_us(10000);
        Wwdg::refresh(0x7F);
    }
    print(serial, "  it survived ten refreshes; now nothing refreshes it", crlf);
    mark_leg('v', 1);
    Wwdg::refresh(0x7F);        // the mark and the refresh, in that order
    spin_until_reset();
}

void tv_resume() {
    const uint32_t ms = token.elapsed_ms;
    switch (token.leg) {
        case 1: {
            print(serial, crlf, "-> back from the window watchdog's time-out after ", ms,
                  " ms", crlf);
            print_flags(boot_flags);
            judge("the window watchdog reset the board and WWDGRSTF says so at the next boot",
                  (boot_flags & ResetFlag::window_watchdog) != 0u);
            judge("and the independent one had nothing to do with it",
                  (boot_flags & ResetFlag::independent_watchdog) == 0u);
            const uint32_t want = wwdg_timeout_us(pclk1, WwdgPrescaler::div8, 0x7F) / 1000u;
            print(serial, "  its own arithmetic says ", want, " ms from the last refresh",
                  crlf);
            judge("the measured time-out is the one PCLK1 and WDGTB compute, to a "
                  "millisecond",
                  ms + 2u >= want && ms <= want + 2u);
            print(serial, "  leg 2: a refresh BEFORE the window opens - the other way to "
                          "fail this watchdog", crlf);
            Reset::clear_flags();
            wwdg_off();
            (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x50});
            Wwdg::start(0x7F);
            mark_leg('v', 2);
            // The counter is at 0x7F and the window opens at 0x50: this
            // refresh is early, and an early refresh IS the reset.
            Wwdg::refresh(0x7F);
            spin_until_reset();
        }
        case 2: {
            print(serial, crlf, "-> back from the early refresh after ", ms, " ms", crlf);
            print_flags(boot_flags);
            judge("a refresh outside the window resets the board at once - the window's "
                  "whole point, and the half a plain watchdog has not got",
                  (boot_flags & ResetFlag::window_watchdog) != 0u);
            judge("and it did so IMMEDIATELY, not after the counter's own fall",
                  ms <= 2u);
            print(serial, "  leg 3: the early wake-up interrupt, one tick before the reset "
                          "it does not prevent", crlf);
            Reset::clear_flags();
            wwdg_off();
            (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8,
                                   .window = 0x7F,
                                   .early_wakeup = true});
            Pfic::enable(Irq::wwdg);
            Wwdg::start(0x7F);
            mark_leg('v', 3);
            Wwdg::refresh(0x7F);
            spin_until_reset();
        }
        case 3: {
            print(serial, crlf, "-> back from the early wake-up after ", ms,
                  " ms, the handler having run ", token.handler_calls,
                  " time(s) before the reset", crlf);
            print_flags(boot_flags);
            judge("the early-wake-up interrupt ran before the reset - its vector is this "
                  "block's own, and its body cleared the flag",
                  token.handler_calls >= 1u);
            judge("and the reset came anyway, the handler not having refreshed",
                  (boot_flags & ResetFlag::window_watchdog) != 0u);
            const uint32_t want = wwdg_timeout_us(pclk1, WwdgPrescaler::div8, 0x7F) / 1000u;
            judge("at the same time-out as the first leg, the interrupt costing nothing",
                  ms + 2u >= want && ms <= want + 2u);
            bench.end_letter();
            break;
        }
        default:
            print(serial, "  unknown leg ", token.leg, crlf);
            break;
    }
}

void banner() {
    print(serial, crlf, "test_vx03_watchdog on ", device::part_name,
          " - the two watchdogs (RM ch. 7 and 8)", crlf,
          "  z costs the board nothing: neither watchdog is armed in it", crlf,
          "  w and v REBOOT the board once per leg - run them by name", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The window watchdog's one vector: the early wake-up, one counter
/// tick before the reset. This body does NOT refresh - the leg that
/// arms it is about the interrupt arriving, not about surviving.
extern "C" BRIO_CH32_INTERRUPT void wwdg_handler() {
    if (brio::Wwdg::isr()) {
        ewi_calls = ewi_calls + 1u;
        // The counter in .bss is this boot's; the one in the token is
        // what crosses the reset that follows.
        token.handler_calls = token.handler_calls + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the window watchdog's block: the gate, the reset values, the "
                      "counter that does not run", ta_wwdg_block);
    bench.letter('b', "the armed counter: its rate, its window, its early wake-up",
                 tb_wwdg_armed);
    bench.letter('c', "the independent watchdog's registers, with nothing armed",
                 tc_iwdg_registers);
    bench.letter('d', "the arithmetic of both blocks against the part's own numbers",
                 td_arithmetic);
    bench.letter('w', "THE INDEPENDENT WATCHDOG'S RESET (reboots the board)", tw_iwdg_reset,
                 false);
    bench.letter('v', "THE WINDOW WATCHDOG'S THREE RESETS (reboots the board)", tv_wwdg_reset,
                 false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        print_flags(boot_flags);
        if (token.magic == token_magic && token.letter != '\0') {
            const char letter = token.letter;
            token.letter = '\0';
            bench.reset_tally();
            bench.resume_tally(token.pass, token.fail);
            if (letter == 'w') {
                tw_resume();
            } else {
                tv_resume();
            }
        } else {
            banner();
        }
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched",
                    brio::crlf);
        bench.prompt();
    }
}
