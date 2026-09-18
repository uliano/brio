// test_rp2350_platform - the reference bench suite for the RP2350's
// PLATFORM, AND THE FIRST PROOF THAT ONE SOURCE RUNS ON TWO INSTRUCTION
// SETS: the identity the chip reports of itself, the critical section,
// the kernel timebase against the microsecond ruler, the kernel's time
// events, the idle path and the lost-wakeup window it must not have, the
// reset controller, the atomic register aliases, the panic breadcrumb,
// the interrupt path bound by ONE NAME on both halves, and erratum
// RP2350-E9 shown on a pad.
//
// Every letter runs on both architectures and is judged the same way.
// Where a verdict must say something different of the two halves, its own
// text says it - which happens twice here: the timebase (SysTick under
// the Cortex-M33, the platform timer's comparator under Hazard3) and the
// rule that keeps the idle path free of a lost wakeup (ARM's, that a
// pending interrupt wakes WFI through PRIMASK; datasheet 3.8.5's, that
// wfi ignores mstatus.MIE).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is a Debug Probe's UART bridge on GP0 (TX)
// / GP1 (RX) = UART0, every clock this suite measures is inside the chip,
// and the one pad it drives (GP22) is left free on purpose.
//
// THE RULER IS THE PLATFORM TIMER (rp2350/mtime.hpp): a 64-bit counter in
// the SIO on a tick divided out of clk_ref, which is the crystal
// undivided - independent of clk_sys, so it judges the kernel timebase
// and the PLL's ratio alike; it shares the crystal with them, so the
// crystal's own accuracy is nobody's verdict here. It is ALSO the kernel
// timebase on the Hazard3 half, where letter c therefore judges the
// comparator against the counter it rides on, and says so.
//
// What is exercised, letter by letter:
//   a  the boot story: SYSINFO's chip id and STEPPING (what the errata
//      are keyed by), the package the die reports against the one this
//      image was built for, whether this is silicon at all, which
//      architecture and which core is running, what the reset controller
//      holds released, and what this boot inherited in .noinit
//   b  the critical section: masked, nested, the inner scope that does
//      not unmask, the outer one that does; a masked window measured on
//      the ruler and the ticks it coalesces
//   c  the kernel timebase: ticks/millis/secs/now coherence and 200 ticks
//      against the ruler
//   d  the ruler itself: running, the 64-bit read against the low half,
//      monotonic over 2000 reads, and its rate against the timebase
//   e  the reset controller: a block held and released, RESET_DONE's
//      latency, cycle()
//   f  the atomic aliases (set, clear, xor, masked write) on a register a
//      driver hands out, every one read back
//   g  the panic breadcrumb WITHOUT a reset: written, taken once, gone
//   h  the idle path over 200 ticks: one call per tick, the awake
//      fraction on the ruler
//   j  THE LOST-WAKEUP WINDOW, which this platform promises not to have:
//      a line made pending with interrupts MASKED, then idle() - it must
//      come back in microseconds and not at the next tick
//   k  ERRATUM RP2350-E9 on a free pad: an idle level is HISTORY here and
//      not a measurement, so the letter drives the pad first and reads it
//      through each pull in turn
//   m  the interrupt path end to end on a line that reaches no hardware:
//      enabled, raised in software, served by a handler bound by ONE NAME
//      on both architectures
//   n  the kernel's time events: armed, due, fired, disarmed, and
//      ticks_to_next() against the deadline
//
//   r  (by name only) THE BREADCRUMB ACROSS A PROCESSOR RESET. The reset
//      chapter of this target is not written yet, so nothing in the image
//      can reboot it: this letter ARMS a .noinit canary and a panic
//      record and prints that it has, and the SAME letter run after a
//      processor reset from the debug port judges what survived. Not in
//      `z`, because `z` has to be one console session a tool can judge
//      from a single capture.
//   x  (by name only) break_here(), which halts under a probe and faults
//      without one - either way the console stops, so it is nobody's `z`
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/resets.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P = brio::Rp2350Platform<>;

// A word in .noinit, to learn what survives which reset (the linker
// script keeps the section NOLOAD, the crt never touches it). Inline, for
// the reason the other strata's suites give: gcc gives an inline variable
// with a section attribute a COMDAT group like the platform's own
// panic_record_, and a plain one would clash.
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

using Led = Pin<25>;    // the WeAct board's LED: a keystroke marker
using Free = Pin<22>;   // a pad this bench leaves unwired, for letter k

TestBench<Serial, 16> bench;

// What this boot inherited, sampled once in main() before anything can
// disturb it.
std::optional<PanicRecord> boot_record;
bool boot_canary_alive = false;

uint32_t us_now() { return Mtime::micros(); }

void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

/// Let the console's own transmitter fall silent: its interrupt would
/// otherwise wake every idle() below.
void console_drain() {
    const uint32_t t0 = us_now();
    while (!Serial::tx_idle() && us_now() - t0 < 500'000u) {
    }
}

const char* arch_name() {
    return core_kind == CoreKind::hazard3 ? "RISC-V Hazard3" : "Arm Cortex-M33";
}
const char* timebase_name() {
    return core_kind == CoreKind::hazard3 ? "the platform timer's comparator"
                                          : "SysTick";
}

// The line that reaches no hardware (datasheet 3.2), for letters m and j.
constexpr IRQn_Type spare_line = SPARE_IRQ_0_IRQn;
volatile uint32_t spare_served = 0;

// =============================================================================
// a - the boot story
// =============================================================================
void ta_boot() {
    const ChipId id = ChipId::read();
    print(serial, "  chip: manufacturer ", hex(id.manufacturer), " part ", hex(id.part),
          " stepping ", hex(id.revision), " gitref ", hex(ChipId::gitref()), crlf);
    bench.verdict("SYSINFO names Raspberry Pi's RP2350",
                  id.manufacturer == ChipId::raspberry_pi && id.part == ChipId::rp2350);
    bench.verdict("the stepping is one appendix C names (A2, A3 or A4), which is what "
                  "every erratum of this chip is keyed by",
                  id.revision == ChipId::revision_a2 || id.revision == ChipId::revision_a3 ||
                      id.revision == ChipId::revision_a4);
    // A TRAP, MEASURED: this chip has TWO registers called PLATFORM, and
    // the one SYSINFO carries (12.15.1, offset 0x08) is the PRE-PRODUCTION
    // indicator - on this silicon the whole register reads zero, ASIC bit
    // included. The register that answers "am I an ASIC" is TBMAN's, whose
    // ASIC bit the data sheet gives a reset value of 1; ChipId::asic()
    // reads that one.
    bench.verdict("this is real silicon by the testbench manager's PLATFORM register - the "
                  "one that answers; SYSINFO's register of the same name is the "
                  "pre-production indicator and reads all-zero on a production part",
                  ChipId::asic());

    const Package reported = ChipId::package_sel();
    print(serial, "  package: the die reports ",
          reported == Package::qfn60 ? "QFN-60" : "QFN-80", ", this image was built for ",
          gpio_count, " GPIO of a possible ", gpio_count_max, crlf);
    bench.verdict("the package the die reports is the one this image was built for - a "
                  "build that states none would compile for the larger and refuse the "
                  "extra pads here instead",
                  !package_known || package_gpio_count(reported) == gpio_count);

    print(serial, "  architecture: ", arch_name(), ", core ", P::core_id(), ", timebase ",
          timebase_name(), crlf);
    bench.verdict("the core this image runs on is core 0, by SIO's CPUID - the register "
                  "and the answer are the same whichever architecture reads them",
                  P::core_id() == 0u && P::core_id() == core_id());
    bench.verdict("the platform type IS the core: on_own_core() agrees with CPUID",
                  P::on_own_core());

    print(serial, "  RESET_DONE=", hex(RESETS->RESET_DONE), " (the blocks out of reset now)",
          crlf);
    bench.verdict("the IO and pad banks, SYSINFO and UART0 are out of reset, released by "
                  "the drivers this suite started",
                  Resets::released(ResetBlock::io_bank0 | ResetBlock::pads_bank0 |
                                   ResetBlock::sysinfo | ResetBlock::uart0));

    print(serial, "  .noinit canary ", boot_canary_alive ? "SURVIVED" : "gone",
          "; breadcrumb ");
    if (boot_record) {
        print(serial, "code ", boot_record->code, " context ", hex(boot_record->context));
    } else {
        print(serial, "none");
    }
    print(serial, crlf);
}

// =============================================================================
// b - the critical section
// =============================================================================
void tb_critical() {
    bench.verdict("interrupts are enabled when a letter runs", P::interrupts_enabled());

    bool inside = true, nested = true, after_inner = true, after_outer = false;
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
    bench.verdict("leaving the INNER scope does not unmask (the mask is SAVED and "
                  "restored, not cleared)",
                  !after_inner);
    bench.verdict("leaving the outer scope unmasks", after_outer);

    // The ruler counts through a masked window - it is hardware, not an
    // interrupt - and the kernel tick coalesces, because its interrupt is
    // a pending bit and not a counter.
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
    bench.verdict("the ruler counts through a masked window, and the kernel tick does "
                  "not: five milliseconds of ruler, at most a handful of ticks",
                  window_us >= 5000u && through_mask <= 6u);
}

// =============================================================================
// c - the kernel timebase against the ruler
// =============================================================================
void tc_ticker() {
    const uint32_t ticks = Ticker::ticks();
    const uint32_t ms = Ticker::millis();
    const uint32_t secs = Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    print(serial, "  ticks=", ticks, " millis=", ms, " secs=", secs, " now=", stamp,
          " on ", timebase_name(), crlf);
    bench.verdict("millis() is ticks() at 1000 Hz (no correction, no drift)",
                  ms >= ticks && ms - ticks <= 2u);
    bench.verdict("secs() is the whole-second part of the same counter",
                  secs == ticks / 1000u || secs == (ticks + 2u) / 1000u);
    bench.verdict("now() agrees with both counters",
                  stamp.seconds == secs || stamp.seconds == secs + 1u);
    bench.verdict("the platform's tick rate is the timebase's", P::ticks_per_second == 1000u &&
                  Ticker::ticks_per_second == 1000u);

    // THE RATE. Two hundred kernel ticks against the ruler. On the
    // Cortex-M33 half the two are independent - SysTick rides clk_sys off
    // the PLL, the ruler rides clk_ref off the crystal - so the error is
    // the PLL's ratio and the reload's rounding. On the Hazard3 half the
    // timebase IS a comparator on this very counter, so the letter judges
    // the comparator's arithmetic rather than a ratio, and cannot be off
    // by more than the handler's own latency.
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
          static_cast<uint32_t>(err * 1000000ULL / expected_us), " ppm) - ",
          core_kind == CoreKind::hazard3
              ? "the comparator against the counter it rides"
              : "SysTick on the PLL against the ruler on the crystal",
          crlf);
    bench.verdict("200 ticks of the kernel timebase are 200 ms of the ruler within "
                  "100 us",
                  err <= 100u);
}

// =============================================================================
// d - the ruler itself
// =============================================================================
void td_ruler() {
    bench.verdict("the counter is enabled and its tick generator running", Mtime::running());

    const uint64_t a = Mtime::now();
    const uint32_t l = Mtime::micros();
    const uint64_t c = Mtime::now();
    print(serial, "  now() ", static_cast<uint32_t>(a), " micros() ", l, " now() ",
          static_cast<uint32_t>(c), " us; high half ", static_cast<uint32_t>(a >> 32), crlf);
    bench.verdict("the 64-bit read and the low half agree within 50 us and never go "
                  "backwards",
                  l >= static_cast<uint32_t>(a) && l - static_cast<uint32_t>(a) <= 50u &&
                      c >= a && c - a <= 50u);

    bool monotonic = true;
    uint64_t last = Mtime::now();
    for (uint16_t k = 0; k < 2000; ++k) {
        const uint64_t n = Mtime::now();
        if (n < last) {
            monotonic = false;
        }
        last = n;
    }
    bench.verdict("2000 reads of the 64-bit counter never go backwards", monotonic);

    // A millisecond of kernel tick against the ruler: the two clocks are
    // the same crystal, so this is the tick's arithmetic and nothing else.
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    const uint32_t u0 = us_now();
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < 20u) {
    }
    const uint32_t span = us_now() - u0;
    print(serial, "  20 ticks = ", span, " us of ruler", crlf);
    bench.verdict("a microsecond of the ruler is a thousandth of a kernel tick, within "
                  "one per cent over twenty ticks",
                  span >= 19'800u && span <= 20'200u);
}

// =============================================================================
// e - the reset controller
// =============================================================================
void te_resets() {
    // The PWM block: nothing in this suite uses it, and it is held in
    // reset at power-up (7.5) - unless a previous life released it.
    const bool before = Resets::released(ResetBlock::pwm);
    print(serial, "  PWM at entry: ", before ? "released" : "held", crlf);

    Resets::hold(ResetBlock::pwm);
    bench.verdict("hold(): the block's RESET bit is set and its RESET_DONE clears",
                  Resets::held(ResetBlock::pwm) && !Resets::released(ResetBlock::pwm));

    const uint32_t t0 = us_now();
    const bool released = Resets::release(ResetBlock::pwm);
    const uint32_t took = us_now() - t0;
    print(serial, "  release(): RESET_DONE within ", took, " us", crlf);
    bench.verdict("release() reports the block ready, and quickly", released && took <= 20u);
    bench.verdict("RESET_DONE stands and RESET is clear",
                  Resets::released(ResetBlock::pwm) && !Resets::held(ResetBlock::pwm));
    bench.verdict("cycle() takes a released block through reset and back",
                  Resets::cycle(ResetBlock::pwm) && Resets::released(ResetBlock::pwm));
    bench.verdict("releasing a block twice is a confirmation and not an error",
                  Resets::release(ResetBlock::pwm));
    Resets::hold(ResetBlock::pwm);   // left as it was found
}

// =============================================================================
// f - the atomic register aliases
// =============================================================================
void tf_aliases() {
    // A register a DRIVER hands out: UART1's integer baud divisor, in a
    // block this letter owns from its reset state and puts back. Sixteen
    // bits wide, which is what the patterns below stay inside.
    using U1 = Pl011<1>;
    if (!U1::reset()) {
        bench.verdict("UART1 out of reset, to lend a register", false);
        return;
    }
    volatile uint32_t& r = U1::regs().UARTIBRD;
    r = 0xF0F0u;
    hw_set(r, 0x000Fu);
    const uint32_t after_set = r;
    hw_clear(r, 0xF000u);
    const uint32_t after_clear = r;
    hw_xor(r, 0x00FFu);
    const uint32_t after_xor = r;
    hw_write_masked(r, 0x1234u, 0x0FF0u);
    const uint32_t after_masked = r;
    print(serial, "  set -> ", hex(after_set), " clear -> ", hex(after_clear), " xor -> ",
          hex(after_xor), " masked -> ", hex(after_masked), crlf);
    bench.verdict("the SET alias ORs the written bits in", after_set == 0xF0FFu);
    bench.verdict("the CLR alias clears them", after_clear == 0x00FFu);
    bench.verdict("the XOR alias flips them", after_xor == 0x0000u);
    bench.verdict("a masked write changes only its mask", after_masked == 0x0230u);
    U1::hold();
}

// =============================================================================
// g - the panic breadcrumb, without a reset
// =============================================================================
void tg_breadcrumb() {
    print(serial, "  panic record at ", hex(reinterpret_cast<uintptr_t>(&P::panic_record())),
          ", the .noinit canary at ", hex(reinterpret_cast<uintptr_t>(&noinit_canary)), crlf);
    bench.verdict("the .noinit section sits in the SRAM this chip starts at 0x20000000",
                  reinterpret_cast<uintptr_t>(&noinit_canary) >= 0x20000000u &&
                      reinterpret_cast<uintptr_t>(&noinit_canary) < 0x20082000u);
    bench.verdict("nothing is pending", !take_panic_record<P>());
    P::panic_record() =
        PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::assert_failed), 0x42};
    const auto taken = take_panic_record<P>();
    bench.verdict("a written record is taken with its code and context",
                  taken && taken->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                      taken->context == 0x42);
    bench.verdict("and taken only once", !take_panic_record<P>());
}

// =============================================================================
// h - the idle path over 200 ticks
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
                  calls >= 190u && calls <= 215u);
    bench.verdict("the loop is awake under 2 % of the time with nothing to do",
                  awake * 50u < span);
    bench.verdict("200 ticks are 200 ms on the ruler within two milliseconds",
                  span >= 198'000u && span <= 202'000u);
    bench.verdict("and idle() comes back with interrupts enabled", P::interrupts_enabled());
}

// =============================================================================
// j - the lost-wakeup window this platform promises not to have
// =============================================================================
void tj_lost_wakeup() {
    console_drain();
    Irq::clear_pending(spare_line);
    Irq::enable(spare_line);
    spare_served = 0;

    // Land just after a tick, so the tick is a full period away and
    // cannot be what brings idle() back.
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }

    // THE WINDOW: the line is made pending with interrupts MASKED, which
    // is exactly the kernel loop's state between looking at its queues
    // and sleeping. The sleep must not happen - or must end at once.
    disable_interrupts();
    Irq::set_pending(spare_line);
    const uint32_t a = us_now();
    P::idle();
    const uint32_t took = us_now() - a;

    spin_us(200);
    print(serial, "  a line pended with interrupts masked, then idle(): back in ", took,
          " us, the handler ran ", spare_served, " time(s) - ",
          core_kind == CoreKind::hazard3
              ? "datasheet 3.8.5: wfi ignores mstatus.MIE"
              : "ARM's rule: a pending interrupt wakes WFI through PRIMASK",
          crlf);
    bench.verdict("idle() comes straight back from a wakeup raised INSIDE the mask, in "
                  "well under a tick period: this platform has no lost-wakeup window",
                  took < 500u);
    bench.verdict("and the handler ran once, after the unmask", spare_served == 1u);
    Irq::disable(spare_line);
}

// =============================================================================
// k - erratum RP2350-E9 on a free pad
// =============================================================================
void tk_erratum_e9() {
    const ChipId id = ChipId::read();

    // Driven LOW, then released to an input with a pull-DOWN: the pad has
    // no charge to leak from and the pull-down holds it.
    (void)Free::output(false);
    spin_us(1000);
    (void)Free::input(PinPull::down);
    spin_us(5000);
    const bool low_after_low = Free::read();

    // Driven HIGH, then released to the SAME pull-down. On a part where
    // E9 is live the input stage leaks enough to hold the pad high
    // against its own pull-down; where it is fixed, the pull-down wins.
    (void)Free::output(true);
    spin_us(1000);
    (void)Free::input(PinPull::down);
    spin_us(5000);
    const bool high_after_high = Free::read();

    // And through the pull-UP, which wins either way - which is why a
    // program that must read a floating pad on this silicon uses the
    // pull-up and reads a LOW as its signal.
    (void)Free::output(false);
    spin_us(1000);
    (void)Free::input(PinPull::up);
    spin_us(5000);
    const bool up_after_low = Free::read();

    print(serial, "  GP", Free::number, " driven low then pulled down reads ",
          low_after_low ? "1" : "0", "; driven high then pulled down reads ",
          high_after_high ? "1" : "0", "; driven low then pulled UP reads ",
          up_after_low ? "1" : "0", crlf);
    bench.verdict("a pad driven low and released to a pull-down stays low",
                  !low_after_low);
    bench.verdict("the pull-up wins over a pad's history, which is why a program that "
                  "must read a floating pad here uses it and reads a LOW as the signal",
                  up_after_low);
    if (id.revision == ChipId::revision_a2) {
        bench.verdict("RP2350-E9 IS LIVE on this stepping: a pad driven HIGH and released "
                      "to its own pull-down does not come down - an idle level here is "
                      "HISTORY and not a measurement",
                      high_after_high);
    } else {
        bench.verdict("this stepping is past RP2350-E9: the pull-down brings a pad that "
                      "was driven high back down",
                      !high_after_high);
    }
    (void)Free::release();
    bench.verdict("released, the pad is ISOLATED again with its pull-down and its input "
                  "buffer off - this chip's reset state",
                  Free::isolated());
}

// =============================================================================
// m - the interrupt path, end to end, through one name
// =============================================================================
void tm_interrupt_path() {
    spare_served = 0;
    Irq::clear_pending(spare_line);
    bench.verdict("the line starts disabled and not pending",
                  !Irq::pending(spare_line));

    Irq::enable(spare_line);
    bench.verdict("enable() is read back by the controller", Irq::enabled(spare_line));

    Irq::set_pending(spare_line);
    spin_us(200);
    print(serial, "  a spare line (one that reaches no hardware) raised in software: the "
          "handler ran ", spare_served, " time(s), bound by the name isr_spare_0 through ",
          core_kind == CoreKind::hazard3 ? "Hazard3's own dispatch" : "the vector table",
          crlf);
    bench.verdict("it is served exactly once, by a handler an app bound BY NAME - the "
                  "same name on both architectures, the interrupt numbering being shared",
                  spare_served == 1u);
    bench.verdict("and the handler's clear took the pending bit down",
                  !Irq::pending(spare_line));

    Irq::disable(spare_line);
    bench.verdict("disable() is read back too", !Irq::enabled(spare_line));
    Irq::set_pending(spare_line);
    spin_us(200);
    bench.verdict("a DISABLED line still latches its pending bit and does not run",
                  Irq::pending(spare_line) && spare_served == 1u);
    Irq::clear_pending(spare_line);
    bench.verdict("and clear_pending() takes it back down", !Irq::pending(spare_line));
}

// =============================================================================
// n - the kernel's time events
// =============================================================================
struct Tick {};
struct Probe {
    using Event = Tick;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

void tn_time_events() {
    TimeEvent<P, Probe, Tick> one{Tick{}};
    TimeEvent<P, Probe, Tick> many{Tick{}};
    while (Probe::queue.pop()) {
    }

    bench.verdict("a time event starts disarmed", !one.armed() && !many.armed());
    bench.verdict("with nothing armed the loop has no deadline to wait for",
                  !TimeEvents<P>::ticks_to_next().has_value());

    one.arm(ticks_from_ms<P>(50));
    bench.verdict("arming it makes it armed", one.armed());
    const auto due = TimeEvents<P>::ticks_to_next();
    print(serial, "  one-shot armed 50 ms out: ticks_to_next() says ",
          due ? *due : 0u, crlf);
    bench.verdict("ticks_to_next() is the power model's one question, and it answers the "
                  "deadline just armed",
                  due && *due <= 50u && *due >= 45u);

    // Nothing fires until process() runs, and process() is the loop's.
    TimeEvents<P>::process();
    bench.verdict("nothing has fired yet", !Probe::queue.pop().has_value() && one.armed());

    uint32_t fired = 0;
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 120u) {
        TimeEvents<P>::process();
        while (Probe::queue.pop()) {
            ++fired;
        }
    }
    print(serial, "  120 ms later the one-shot had fired ", fired, " time(s) and is ",
          one.armed() ? "still armed" : "disarmed", crlf);
    bench.verdict("a one-shot fires exactly once and disarms itself",
                  fired == 1u && !one.armed());

    // A periodic: drift-free, so N periods are N firings and not N - 1.
    many.arm_every(ticks_from_ms<P>(10));
    fired = 0;
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < 105u) {
        TimeEvents<P>::process();
        while (Probe::queue.pop()) {
            ++fired;
        }
    }
    const uint32_t span = Ticker::ticks() - t1;
    print(serial, "  a 10 ms periodic fired ", fired, " time(s) in ", span, " ms", crlf);
    bench.verdict("a periodic fires once per period, drift-free: ten firings in 105 ms",
                  fired >= 9u && fired <= 11u);
    many.disarm();
    bench.verdict("disarmed, it is out of the list and the loop has no deadline again",
                  !many.armed() && !TimeEvents<P>::ticks_to_next().has_value());
}

// =============================================================================
// r - the breadcrumb across a processor reset (by name)
// =============================================================================
void tr_across_reset() {
    if (boot_canary_alive) {
        bench.verdict("the .noinit word survived the reset: SRAM keeps its contents "
                      "through a processor reset, which resets the cores and leaves the "
                      "memories, the clock tree and the peripherals alone",
                      boot_canary_alive);
        bench.verdict("and the panic breadcrumb crossed it with its code and context",
                      boot_record &&
                          boot_record->code == static_cast<uint8_t>(PanicCode::queue_overflow) &&
                          boot_record->context == 0x33);
        noinit_canary = 0;
        boot_record.reset();
        boot_canary_alive = false;
        print(serial, "  disarmed", crlf);
        return;
    }
    noinit_canary = canary_value;
    P::panic_record() =
        PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::queue_overflow), 0x33};
    print(serial, "  armed: a .noinit canary and a panic record are in SRAM. Reset the "
          "board with a PROCESSOR reset from the debug port (not a rescue, which clears "
          "the memories) and run r again.", crlf);
    bench.verdict("the canary and the record are in place", noinit_canary == canary_value);
}

// =============================================================================
// x - break_here (by name)
// =============================================================================
void tx_break() {
    print(serial, "  break_here(): this halts under a probe with halting debug on, and "
          "faults without one. Either way the console stops here.", crlf);
    console_drain();
    P::break_here();
    print(serial, "  ... and it came back, which means neither happened", crlf);
    bench.verdict("break_here() returned instead of halting or faulting", false);
}

void banner() {
    print(serial, crlf, "test_rp2350_platform - the RP2350 platform on ", arch_name(),
          " (datasheet 3.1, 3.8, 7.5, 12.15), clk_sys=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES: a Cortex-M vector-table slot on one half,
// an entry of Hazard3's own dispatch on the other.
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_spare_0() {
    spare_served = spare_served + 1u;
    brio::Irq::clear_pending(spare_line);
}

int main() {
    boot_record = brio::take_panic_record<P>();
    boot_canary_alive = noinit_canary == canary_value;

    const bool clock_ok = SysClock::init();
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the boot story: identity, stepping, package, architecture", ta_boot);
    bench.letter('b', "the critical section and its nesting", tb_critical);
    bench.letter('c', "the kernel timebase against the ruler", tc_ticker);
    bench.letter('d', "the microsecond ruler itself", td_ruler);
    bench.letter('e', "the reset controller", te_resets);
    bench.letter('f', "the atomic register aliases", tf_aliases);
    bench.letter('g', "the panic breadcrumb, no reset", tg_breadcrumb);
    bench.letter('h', "the idle path over 200 ticks", th_idle);
    bench.letter('j', "THE LOST-WAKEUP WINDOW: a line pended inside the mask", tj_lost_wakeup);
    bench.letter('k', "erratum RP2350-E9 on a free pad", tk_erratum_e9);
    bench.letter('m', "the interrupt path through one name", tm_interrupt_path);
    bench.letter('n', "the kernel's time events", tn_time_events);
    bench.letter('r', "the breadcrumb across a PROCESSOR reset (two runs)", tr_across_reset,
                 false);
    bench.letter('x', "break_here() (stops the console)", tx_break, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " tick=",
                    tick_ok ? "1kHz" : "FAILED", brio::crlf);
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
