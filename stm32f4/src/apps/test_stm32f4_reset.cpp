// test_stm32f4_reset - the reference bench suite for the STM32F4's
// FAILING half: stm32f4/reset.hpp (the RCC's reset flags, the
// independent watchdog, the system window watchdog, the four fault
// vectors of this core and the panic breadcrumb across a real reset).
// The running half - the critical section, the timebase, delay_us, the
// clock tree - is test_stm32f4_platform's.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the board's own serial bridge and
// every clock this suite measures is inside the chip.
//
// What is exercised, letter by letter:
//   a  the boot story: the reset flags read as the ACCUMULATING history
//      they are, the LSI witness, the WWDG at rest and the two debug
//      freeze bits a debugger may have left standing
//   b  the IWDG WITHOUT EVER STARTING IT - the keyed registers, the two
//      update bits crossing into the LSI domain, and what the LSI's own
//      startup costs
//   c  the WWDG WITHOUT EVER ACTIVATING IT - the free-running counter,
//      the window predicate, EWIF at 0x40 and the early-wakeup
//      interrupt, all with WDGA clear
//   d  the four fault vectors WITHOUT FAULTING - the three configurable
//      faults' enables, the two CCR traps, the status registers
//
//   i  (by name only) SEVEN REAL RESETS. This letter reboots the board
//      once per leg and resumes from a .noinit token, so it is NOT in
//      `z`: `z` has to be one console session a tool can judge from a
//      single capture. Run it with
//          brio run <board> i --app test_stm32f4_reset
//                  --expect="fail" --timeout 120
//      Legs: a software reset, an IWDG time-out (which measures the real
//      one and with it LSI), a WWDG window violation, a panic through
//      ResetReporter, a deliberate HardFault through hard_fault_reset(),
//      a UsageFault with the fault enabled and its record read, and a
//      WWDG time-out with the early warning armed.
//
// NOTHING IN `z` STARTS A WATCHDOG, and that is a measurement rather
// than an oversight. Starting either is one-way in software (RM0090
// 21.3, 22.3), so no letter of `z` starts one at all - the whole of
// chapter 22's timing is reachable with WDGA clear, and chapter 21's
// keyed registers with the start key never written. Letter i does start
// both, on legs that are about to be reset by them.
//
// build: boards = f429zi,f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/panic.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/reset.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter i lives in
//
// INLINE, and in .noinit, for the two reasons the other strata's suites
// give: the section must survive the crt (the linker script marks
// .noinit NOLOAD and startup neither loads nor zeroes it), and gcc gives
// an inline variable with a section attribute a COMDAT group where a
// plain one gets none - the platform's own panic_record_ is a static
// inline member, so this must be inline too or the link fails with a
// section type conflict.
//
// Its magic word is not decoration: RM0090 promises nothing about SRAM
// across a reset, so every read of this object is guarded.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0xF429;
inline constexpr uint16_t token_canary = 0xC3A5;

/// What the IWDG's own reset did to it - the question leg 2 answers by
/// sitting still, and the state machine that survives EITHER answer
/// (a wait that never returns is itself the measurement).
enum class Quiet : uint8_t {
    untried = 0,   ///< the silence has not been attempted yet
    trying = 1,    ///< banked just before it: a boot that finds this died in it
    survived = 2,  ///< three time-outs passed with nothing refreshing
    bitten = 3,    ///< the watchdog reset the board again during the silence
};

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
    Quiet quiet;             ///< leg 2: the silence after the watchdog's reset
    uint8_t armed;           ///< leg 2: what Iwdg::arm() answered
    uint8_t armed_running;   ///< leg 2: Iwdg::running() with the watchdog live
    uint8_t ewi_seen;        ///< leg 7: did the early warning fire?
    uint8_t ewi_counter;     ///< and what the counter read inside its handler
    uint32_t fault_cfsr;     ///< legs 5 and 6: the wreck, read in the handler
    uint32_t fault_hfsr;
    uint32_t fault_address;
};
[[gnu::section(".noinit")]] inline Token token;


namespace {

using namespace brio;

using P = Stm32f4Platform<>;

#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
using Led = Pin<'A', 5>;
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// What this boot was told, sampled once in main() before anything can
// disturb it.
uint32_t boot_flags = 0;
std::optional<PanicRecord> boot_record;
bool boot_lsi_on = false;        ///< LSION as the boot found it
bool boot_lsi_ready = false;     ///< LSIRDY as the boot found it
bool boot_wwdg_clocked = false;  ///< RCC_APB1ENR.WWDGEN as the boot found it
uint32_t boot_shcsr = 0;         ///< the three configurable faults' enables
FaultRecord boot_fault{};        ///< CFSR/HFSR as the boot found them

/// Set when letter i's second leg has left a watchdog running that its
/// own reset did not stop: from then on the console loop feeds it, so
/// the remaining legs measure what they mean to measure.
bool feed_iwdg = false;

/// Set by the WWDG handler; read by letter c.
volatile uint16_t ewi_count = 0;

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch: ticks x period + the phase SysTick has
// already counted down. The two reads are retried until they belong to
// the same tick, which is what makes the sum monotone across the
// handler.
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

uint32_t cycles_to_us(uint32_t cycles) { return cycles / cycles_per_us; }

/// Keep a watchdog alive that a reset did not stop (see feed_iwdg). A
/// no-op on a stopped watchdog: 0xAAAA into a key register nothing is
/// counting is not an error, it is nothing.
void service() {
    if (feed_iwdg) {
        Iwdg::refresh();
    }
}

/// Wait for the console to be physically empty. Called before anything
/// that reboots the board: a ring that still holds bytes loses them, and
/// a suite that loses its own last line is unreadable.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
        service();
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
    if (f & ResetFlag::power_on) print(serial, " POR");
    if (f & ResetFlag::pin) print(serial, " PIN");
    if (f & ResetFlag::brown_out) print(serial, " BOR");
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

    // READING IS NOT CLEARING. Nothing but RMVF (or a power reset) takes
    // these bits down - this is an ACCUMULATING history and not one
    // exclusive cause - so two reads in a row give the same answer, and
    // so does the sample main() took at boot. THIS LETTER NEVER WRITES
    // RMVF: it would be a one-way loss of the boot's own evidence, and
    // letter i proves the clearing on real resets (legs 2 and 3) where
    // the flags are made again a moment later.
    const uint32_t r1 = Reset::flags();
    const uint32_t r2 = Reset::flags();
    bench.verdict("the flags are unchanged by reading them", r1 == r2);
    bench.verdict("and they still hold what main() sampled at boot", r1 == boot_flags);
    bench.verdict("nothing left RMVF standing", (RCC->CSR & RCC_CSR_RMVF) == 0u);

    // PINRSTF is described as the pin's alone here (7.3.21), but the
    // NRST pad is driven low by the internal reset sources too - letter
    // i's first leg is where a SOFTWARE reset shows what it raises
    // beside SFTRSTF. Printed here, judged there.
    print(serial, "  pin_only(these flags)=", Reset::pin_only(boot_flags),
          " - PINRSTF names the pin only when it stands alone", crlf);
    bench.verdict("pin_only() is the predicate and not a bit: it is false for "
                  "any reading with a second flag in it",
                  Reset::pin_only(ResetFlag::pin) &&
                      !Reset::pin_only(ResetFlag::pin | ResetFlag::software));

    // THE LSI WITNESS, AND WHAT IT IS WORTH. A started IWDG forces the
    // LSI on and cannot be stopped (7.2.9), so LSIRDY standing with
    // LSION clear is the only sign this silicon offers that a watchdog
    // is running. THE OTHER CANDIDATE REQUESTOR DOES NOT SPOIL IT: this
    // board's backup domain - which no system reset touches - holds
    // RCC_BDCR with RTCEN set and RTCSEL naming the LSI, and LSIRDY is
    // clear all the same, so an RTC's standing select does not force the
    // oscillator on this family. Both halves are printed; the verdict is
    // that nothing is forcing it at this boot.
    const bool rtc_asks_lsi =
        (RCC->BDCR & RCC_BDCR_RTCEN) != 0u &&
        ((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >> RCC_BDCR_RTCSEL_Pos) == 2u;
    print(serial, "  LSI  : LSION=", boot_lsi_on, " LSIRDY=", boot_lsi_ready,
          " RCC_BDCR=", hex(RCC->BDCR), " (the RTC domain ",
          rtc_asks_lsi ? "IS" : "is NOT", " asking for LSI)", crlf);
    bench.verdict("nothing forces the LSI at this boot - no watchdog is "
                  "running, and an RTC select that survives every system reset "
                  "does not raise LSIRDY by itself, which is what leaves "
                  "Iwdg::running() a usable witness here",
                  !boot_lsi_ready && !boot_lsi_on);

    // The WWDG's bus clock is CLEAR at reset. On this family the option
    // byte selects a hardware INDEPENDENT watchdog, not a hardware
    // window one, so nothing but software ever sets this bit.
    print(serial, "  WWDG : bus clock at boot ", boot_wwdg_clocked ? "ON" : "off",
          ", WDGA=", Wwdg::enabled(), crlf);
    bench.verdict("the window watchdog is not activated at boot (WDGA is "
                  "cleared by every reset - 22.6.1)",
                  !Wwdg::enabled());

    // THE DEBUG FREEZE BITS. DBGMCU is at 0xE0042000, in the core's
    // private peripheral space: no bus-clock enable in front of it, so
    // reading it needs nothing but the read - which the IDCODE proves,
    // since this program opened no gate for it. What the two freeze bits
    // hold is the DEBUGGER's business (OpenOCD's stm32f4x.cfg sets them
    // at every attach and they survive every reset but a power-on), so
    // they are printed and not judged.
    print(serial, "  DBGMCU IDCODE=", hex(DBGMCU->IDCODE), " APB1FZ=",
          hex(DBGMCU->APB1FZ), ": DBG_IWDG_STOP=", Iwdg::debug_frozen(),
          " DBG_WWDG_STOP=", Wwdg::debug_frozen(),
          " (the debugger's bits; they freeze the counters only while the "
          "core is HALTED)", crlf);
    bench.verdict("the DBGMCU answers with no bus-clock enable of its own - "
                  "its IDCODE names this part",
                  (DBGMCU->IDCODE & 0xFFFu) == (DBGMCU->IDCODE & 0xFFFu) &&
                      (DBGMCU->IDCODE & 0xFFFu) != 0u);
}

// =============================================================================
// b - the IWDG, never started
// =============================================================================
//
// 21.3.2's write protection has nothing to do with the start key, so PR
// and RLR can be written by a program that has no intention of arming
// anything - and this letter does exactly that, which is what keeps a
// one-way switch out of `z`.
//
// WHAT IT FOUND, and no chapter says it: with the watchdog STOPPED the
// keyed writes are accepted and each raises its own update bit, and THE
// UPDATE NEVER COMPLETES - the bit stands for ever and the register
// keeps reporting its old value. 7.2.9 is where the reason hides: "If
// the independent watchdog is started ... the LSI oscillator is forced
// ON ... After the LSI oscillator temporization, THE CLOCK IS PROVIDED
// TO THE IWDG". So LSION buys nothing: the logic that performs the
// update has no clock until the start key is written. That is why this
// driver's arm() starts before it configures - the completing half is
// proven in letter i, whose second leg arms a NON-DEFAULT setting and is
// reset by it at exactly the time that setting predicts.
//
// THE UPDATE BITS ARE SINGLE-USE PER BOOT here, since nothing can clear
// them while the watchdog is stopped: this letter therefore writes RLR
// and nothing else, and uses the still-clear PVU as the witness for the
// key discipline, so that a second run in the same power cycle judges
// exactly the same things.
void tb_iwdg() {
    // The LSI first, and timed: the datasheets give 15 us typical, 40 us
    // maximum for the startup, which is the only part of this letter
    // that is a physical measurement.
    Rcc::lsi_enable(false);
    const uint32_t c0 = cycles_now();
    Rcc::lsi_enable(true);
    const bool lsi_up = Rcc::lsi_wait_ready();
    const uint32_t lsi_us = cycles_to_us(cycles_now() - c0);
    print(serial, "  LSI started in ", lsi_us, " us (the datasheet's LSI table: "
          "15 us typical, 40 us max)", crlf);
    bench.verdict("the LSI comes up when LSION asks for it", lsi_up);
    bench.verdict("and it takes the datasheet's tens of microseconds, not "
                  "milliseconds",
                  lsi_us < 1000u);

    // ---- the key is required ---------------------------------------------
    // 21.3.2: PR and RLR are writable only after 0x5555, and any other
    // key value closes the window again - the 0xAAAA refresh included.
    // A refresh, then a bare store, then the register's own update bit
    // as the witness that nothing moved.
    Iwdg::refresh();
    const uint32_t sr_before = Iwdg::status();
    IWDG->PR = 5u;                      // no unlock: this must land nowhere
    const uint32_t sr_locked = Iwdg::status();
    print(serial, "  a bare PR store after a refresh: SR ", hex(sr_before), " -> ",
          hex(sr_locked), " (PVU must stay clear)", crlf);
    bench.verdict("a protected write made without the 0x5555 key raises no "
                  "update at all - the refresh re-locked the window (21.3.2)",
                  sr_locked == sr_before && (sr_locked & IWDG_SR_PVU_Msk) == 0u);

    // ---- one keyed write, timed from both ends ---------------------------
    Iwdg::unlock();
    const uint32_t c1 = cycles_now();
    IWDG->RLR = 0x0ABCu;
    uint32_t set_us = 0;
    while ((IWDG->SR & IWDG_SR_RVU_Msk) == 0u) {
        set_us = cycles_to_us(cycles_now() - c1);
        if (set_us > 2000u) {
            break;
        }
    }
    const bool bit_appeared = (IWDG->SR & IWDG_SR_RVU_Msk) != 0u;
    const uint32_t c2 = cycles_now();
    const bool completed = Iwdg::sync(IWDG_SR_RVU_Msk);
    const uint32_t waited_ms = cycles_to_us(cycles_now() - c2) / 1000u;

    print(serial, "  a keyed RLR write raised RVU after ", set_us, " us and it ",
          completed ? "cleared" : "was STILL STANDING", " after ", waited_ms,
          " ms of bounded wait", crlf);
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
                  "the LSI running is not enough, because the clock reaches "
                  "the IWDG only at the start key (7.2.9)",
                  !completed);
    print(serial, "  RLR reads back ", hex(Iwdg::reload()), " (0xABC written), SR=",
          hex(Iwdg::status()), crlf);
    bench.verdict("and the register keeps reporting its OLD value, since the "
                  "read comes from the VDD domain (21.4.3's Note)",
                  Iwdg::reload() == 0x0FFFu);

    // configure() on the same stopped watchdog: it must SAY SO instead
    // of hanging, which is the whole reason its waits are bounded.
    const uint32_t c3 = cycles_now();
    const bool cfg_ok = Iwdg::configure(IwdgConfig{IwdgPrescaler::div64, 0x0777});
    const uint32_t cfg_ms = cycles_to_us(cycles_now() - c3) / 1000u;
    print(serial, "  configure(/64, 0x777) on a stopped watchdog -> ", cfg_ok,
          " after ", cfg_ms, " ms; PR=", Iwdg::prescaler_bits(), " RLR=",
          hex(Iwdg::reload()), " (that setting would be ",
          iwdg_nominal_ms(IwdgPrescaler::div64, 0x0777), " ms at 32 kHz)", crlf);
    bench.verdict("configure() answers false in bounded time rather than "
                  "hanging, and the registers are untouched - arm(), which "
                  "starts first, is the only order that works here",
                  !cfg_ok && cfg_ms < 3000u && Iwdg::reload() == 0x0FFFu);

    // ---- what is refused --------------------------------------------------
    bench.verdict("a reload wider than twelve bits is refused",
                  !Iwdg::configure(IwdgConfig{IwdgPrescaler::div4, 0x1000}));
    bench.verdict("a reload of ZERO is not (it is table 107's min column, the "
                  "shortest time-out the part has, and force_reset() is what "
                  "uses it)",
                  iwdg_config_valid(IwdgConfig{IwdgPrescaler::div4, 0}));

    // ---- the arithmetic against RM0090 table 107 --------------------------
    print(serial, "  table 107 at 32 kHz: /4 x 0xFFF = ",
          iwdg_nominal_ms(IwdgPrescaler::div4, 0x0FFF), " ms, /256 x 0xFFF = ",
          iwdg_nominal_ms(IwdgPrescaler::div256, 0x0FFF), " ms", crlf);
    bench.verdict("iwdg_nominal_ms reproduces table 107's max column",
                  iwdg_nominal_ms(IwdgPrescaler::div4, 0x0FFF) == 512u &&
                      iwdg_nominal_ms(IwdgPrescaler::div256, 0x0FFF) == 32768u);

    // The watchdog was never started, so this letter leaves the board
    // exactly as it found it but for the LSI, which letter i needs off
    // to read its witness.
    bench.verdict("nothing here started the watchdog: the LSI is on because "
                  "THIS letter asked for it, and Iwdg::running() says so",
                  !Iwdg::running() && Rcc::lsi_enabled());
    Rcc::lsi_enable(false);
}

// =============================================================================
// c - the WWDG, never activated
// =============================================================================
//
// 22.3 says the down-counter is free-running "even if the watchdog is
// disabled", and 22.6.3 says EWIF is set at 0x40 "also if the interrupt
// is not enabled". Taken together they make the whole timing path of
// this chapter measurable with WDGA never written - which is what this
// letter does, so that a reference suite carries no one-way switch.
void tc_wwdg() {
    constexpr uint32_t pclk1 = SysClock::pclk1_hz;

    // The closed gate first: a peripheral without its bus clock does not
    // answer. Printed, not judged: a claim the silicon may honour by
    // luck is no verdict.
    Wwdg::bus_clock(false);
    const uint32_t dark_cr = WWDG->CR;
    Wwdg::bus_clock(true);
    print(serial, "  CR through a closed clock gate: ", hex(dark_cr),
          ", with the gate open: ", hex(Wwdg::cr()), crlf);
    bench.verdict("the bus clock enables and reads back", Wwdg::bus_clock());
    bench.verdict("WDGA is clear: this letter never activates the watchdog",
                  !Wwdg::enabled());

    // CFR readback. Nothing here is enable-protected and nothing
    // synchronizes: both registers are on PCLK1.
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
                  Wwdg::prescaler() == WwdgPrescaler::div8 && Wwdg::window() == 0x5Au);
    bench.verdict("a window below 0x40 is refused (no refresh could ever be "
                  "legal - Wwdg::force_reset() is the deliberate spelling)",
                  !Wwdg::configure(WwdgConfig{WwdgPrescaler::div1, 0x3F}));
    bench.verdict("and a window wider than seven bits is refused",
                  !Wwdg::configure(WwdgConfig{WwdgPrescaler::div1, 0x80}));

    // THE FREE-RUNNING COUNTER, and the WINDOW PREDICATE with it. At /8
    // a step is 4096 x 8 / PCLK1; from 0x7F the counter needs 0x7F -
    // 0x5A = 37 steps to enter the window, and in_window() is what says
    // it has.
    Wwdg::refresh(0x7F);
    const uint8_t c_start = Wwdg::counter();
    const bool closed_at_once = !Wwdg::in_window();
    const uint32_t w0 = cycles_now();
    uint32_t open_us = 0;
    uint8_t c_open = 0;
    while (cycles_now() - w0 < SysClock::hz / 10u) {   // 100 ms at most
        if (Wwdg::in_window()) {
            open_us = cycles_to_us(cycles_now() - w0);
            c_open = Wwdg::counter();
            break;
        }
    }
    const uint8_t c_later = Wwdg::counter();
    print(serial, "  T went ", hex(c_start), " -> ", hex(c_later),
          " with WDGA CLEAR; the window opened after ", open_us, " us at T=",
          hex(c_open), " (", wwdg_timeout_us(pclk1, WwdgPrescaler::div8, 0x7F) -
                             wwdg_timeout_us(pclk1, WwdgPrescaler::div8, 0x5A),
          " us due for 37 steps)", crlf);
    bench.verdict("the down-counter free-runs with the watchdog disabled "
                  "(22.3), which is what makes this letter safe",
                  c_later < c_start);
    bench.verdict("in_window() is false above the window and true inside it, "
                  "at the counter value the window names",
                  closed_at_once && c_open != 0u && c_open <= 0x5Au && c_open > 0x3Fu);

    // EWIF AT 0x40, TIMED, at two prescalers. From 0x7F the flag is due
    // 63 steps later; the prediction is the chapter's own formula - and
    // table 109's, which 22.4's worked example contradicts (the example
    // prints a quarter of what its own formula gives).
    const auto time_ewif = [](WwdgPrescaler p) -> uint32_t {
        (void)Wwdg::configure(WwdgConfig{p, 0x7F, false});
        Wwdg::clear_flag();
        Wwdg::refresh(0x7F);
        const uint32_t t0 = cycles_now();
        while (!Wwdg::flag()) {
            if (cycles_now() - t0 > SysClock::hz) {   // one second: give up
                return 0;
            }
        }
        const uint32_t took = cycles_now() - t0;
        Wwdg::clear_flag();
        return cycles_to_us(took);
    };
    const uint32_t ewif1 = time_ewif(WwdgPrescaler::div1);
    const uint32_t ewif8 = time_ewif(WwdgPrescaler::div8);
    // THE STEP IS PCLK1'S, NOT THE CORE'S - this family's APB1 runs at a
    // quarter of HCLK at the top of the ladder, and a WWDG prediction
    // made on the core clock is out by that factor.
    const uint32_t step1 = wwdg_step_cycles(WwdgPrescaler::div1) / (pclk1 / 1'000'000u);
    const uint32_t step8 = wwdg_step_cycles(WwdgPrescaler::div8) / (pclk1 / 1'000'000u);
    // wwdg_timeout_us() gives the time to the RESET, T[5:0] + 1 = 64
    // steps from 0x7F; the WARNING is one step before it.
    const uint32_t due1 = wwdg_timeout_us(pclk1, WwdgPrescaler::div1, 0x7F) - step1;
    const uint32_t due8 = wwdg_timeout_us(pclk1, WwdgPrescaler::div8, 0x7F) - step8;
    print(serial, "  EWIF from 0x7F: ", ewif1, " us at /1 (", due1,
          " due, 63 steps), ", ewif8, " us at /8 (", due8,
          " due); one step is ", step1, " and ", step8, " us at PCLK1 = ", pclk1,
          " Hz, and the reset would follow one step later", crlf);
    bench.verdict("EWIF is raised at 0x40 with WDGA CLEAR and the interrupt "
                  "disabled (22.6.3), 63 steps after a refresh",
                  ewif1 != 0u && ewif8 != 0u);
    // THE BAND IS ONE STEP WIDE EITHER WAY, and 22.3 says why: the
    // timing "varies between a minimum and a maximum value due to the
    // unknown status of the prescaler when writing to the WWDG_CR
    // register" - the write lands mid-step, so the first decrement comes
    // early or late by whatever is left of it. Measured, it lands
    // exactly on 63 steps at /8 and one step late at /1.
    bench.verdict("and it lands within one step of the 63 due at both "
                  "prescalers - the slack is 22.3's own unknown prescaler "
                  "phase, and table 109's arithmetic is the one that holds",
                  ewif1 + step1 >= due1 && ewif1 <= due1 + step1 + 200u &&
                      ewif8 + step8 >= due8 && ewif8 <= due8 + step8 + 200u);

    // rc_w0: the flag is cleared by writing ZERO, which is the opposite
    // discipline to every write-one-to-clear register in this stratum.
    Wwdg::refresh(0x41);   // one step from the warning
    const uint32_t c1 = cycles_now();
    while (!Wwdg::flag() && cycles_now() - c1 < SysClock::hz) {
    }
    const bool flag_up = Wwdg::flag();
    WWDG->SR = WWDG_SR_EWIF;                 // writing ONE has no effect
    const bool survived_one = Wwdg::flag();
    Wwdg::clear_flag();                      // writing ZERO clears it
    const bool cleared_zero = !Wwdg::flag();
    bench.verdict("EWIF stands after a write of ONE (22.6.3: rc_w0, and writing "
                  "1 has no effect)",
                  flag_up && survived_one);
    bench.verdict("and it is cleared by a write of ZERO", cleared_zero);

    // THE INTERRUPT ITSELF, still with WDGA clear. 22.2's feature list
    // says the early wake-up is "triggered (if enabled and the watchdog
    // activated)" while 22.6.2's bit description says only "when set, an
    // interrupt occurs whenever the counter reaches the value 0x40" -
    // two readings of the same silicon, and the bench decides which.
    ewi_count = 0;
    Wwdg::clear_flag();
    Nvic::clear_pending(Wwdg::irq());
    Nvic::enable(Wwdg::irq());
    (void)Wwdg::configure(WwdgConfig{WwdgPrescaler::div8, 0x7F, true});
    Wwdg::refresh(0x7F);
    const uint32_t c2 = cycles_now();
    while (ewi_count == 0u && cycles_now() - c2 < SysClock::hz / 4u) {
    }
    const uint16_t fired = ewi_count;
    const bool flag_after = Wwdg::flag();
    print(serial, "  EWI enabled, WDGA still clear: the handler ran ", fired,
          " time(s) in 250 ms; EWIF=", flag_after, crlf);
    bench.verdict("the early-wakeup INTERRUPT does NOT fire while WDGA is "
                  "clear, though EWIF does rise - 22.2's parenthetical is right "
                  "and 22.6.2's bit description is incomplete",
                  fired == 0u && flag_after);

    // The vector and the ISR body are still exercised, by pending the
    // line by hand over a flag the counter really raised: what the
    // handler does with it is the same code either way, and letter i's
    // last leg is where the hardware request itself is proven (with the
    // watchdog activated, on the reset it was going to cause anyway).
    // The counter free-runs below 0x40 and wraps, raising EWIF again every
    // 128 steps (5 ms at /1): the flag is read RIGHT after the handler,
    // with the counter just refreshed so no new raise lands in between.
    Wwdg::refresh(0x7Fu);
    Nvic::set_pending(Wwdg::irq());
    const uint32_t c3 = cycles_now();
    while (ewi_count == 0u && cycles_now() - c3 < SysClock::hz / 100u) {
    }
    const bool flag_after_isr = Wwdg::flag();
    bench.verdict("pending the line by hand runs the bound handler", ewi_count != 0u);
    bench.verdict("and the ISR body acknowledged the flag (EWIF is down)", !flag_after_isr);
    Nvic::disable(Wwdg::irq());
    Nvic::clear_pending(Wwdg::irq());

    bench.verdict("EWI is one-way: a configuration that asks for it off cannot "
                  "take it back (22.6.2, set by software and cleared by "
                  "hardware after a reset)",
                  (Wwdg::configure(WwdgConfig{WwdgPrescaler::div8, 0x7F, false}),
                   Wwdg::early_wakeup_enabled()));

    // Park it slow, so the free-running counter's flag costs as little
    // as possible for the rest of the session.
    (void)Wwdg::configure(WwdgConfig{WwdgPrescaler::div8, 0x7F, false});
    Wwdg::clear_flag();
    bench.verdict("the board is left with WDGA still clear", !Wwdg::enabled());
}

// =============================================================================
// d - the fault vectors, without faulting
// =============================================================================
void td_faults() {
    print(serial, "  at boot: SHCSR=", hex(boot_shcsr), " CFSR=", hex(boot_fault.cfsr),
          " HFSR=", hex(boot_fault.hfsr), crlf);
    bench.verdict("the three configurable faults are DISABLED at reset, so "
                  "every fault escalates to HardFault until a program says "
                  "otherwise (PM0214 4.3.10)",
                  (boot_shcsr & (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
                                 SCB_SHCSR_USGFAULTENA_Msk)) == 0u);
    bench.verdict("and a clean boot's fault status is empty", boot_fault.empty());

    Faults::enable(true, true, true);
    const bool all_on = Faults::mem_enabled() && Faults::bus_enabled() && Faults::usage_enabled();
    Faults::enable(false, true, false);
    const bool bus_only = !Faults::mem_enabled() && Faults::bus_enabled() && !Faults::usage_enabled();
    Faults::enable(false, false, false);
    const bool all_off = !Faults::mem_enabled() && !Faults::bus_enabled() && !Faults::usage_enabled();
    print(serial, "  SHCSR after enable(1,1,1) / (0,1,0) / (0,0,0): ", all_on, " ",
          bus_only, " ", all_off, crlf);
    bench.verdict("the three enables are independent and both ways",
                  all_on && bus_only && all_off);

    const bool div_was = Faults::divide_by_zero_trap();
    const bool unalign_was = Faults::unaligned_trap();
    Faults::divide_by_zero_trap(true);
    Faults::unaligned_trap(true);
    const bool traps_on = Faults::divide_by_zero_trap() && Faults::unaligned_trap();
    Faults::divide_by_zero_trap(false);
    Faults::unaligned_trap(false);
    const bool traps_off = !Faults::divide_by_zero_trap() && !Faults::unaligned_trap();
    print(serial, "  CCR=", hex(SCB->CCR), " (DIV_0_TRP and UNALIGN_TRP were ",
          div_was, " and ", unalign_was, " at entry)", crlf);
    bench.verdict("the two CCR traps arm and disarm - a divide by zero and an "
                  "unaligned access are UsageFaults only when they are on",
                  traps_on && traps_off);
    Faults::divide_by_zero_trap(div_was);
    Faults::unaligned_trap(unalign_was);

    // The status registers, still with nothing having faulted: read is
    // non-destructive, clear takes down what stands, and the record's
    // predicates decode the words the core would have written.
    const FaultRecord r = Faults::read();
    Faults::clear();
    const FaultRecord after = Faults::read();
    bench.verdict("reading the fault status changes nothing and clearing it "
                  "leaves it empty",
                  r.empty() && after.empty() && !after.address_valid());
    bench.verdict("the record decodes the core's own bit positions",
                  FaultRecord{SCB_CFSR_DIVBYZERO_Msk, 0, 0}.usage_fault() &&
                      !FaultRecord{SCB_CFSR_DIVBYZERO_Msk, 0, 0}.mem_fault() &&
                      FaultRecord{0, SCB_HFSR_FORCED_Msk, 0}.escalated());

    // The boot state is restored: this letter must leave the board as
    // any other program would find it.
    bench.verdict("the letter leaves the three faults disabled, as the reset "
                  "left them",
                  (SCB->SHCSR & (SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
                                 SCB_SHCSR_USGFAULTENA_Msk)) == 0u);
}

// =============================================================================
// i - seven real resets (outside z: it reboots the board)
// =============================================================================

/// The IWDG setting leg 2 arms: deliberately NOT the reset one - /8 with
/// a reload of 0x0EEE instead of /4 with 0x0FFF - so that the time-out
/// itself says whether the configuration landed (955 ms nominal against
/// the 512 ms an unconfigured watchdog would take).
constexpr IwdgPrescaler leg_prescaler = IwdgPrescaler::div8;
constexpr uint16_t leg_reload = 0x0EEE;
/// RL + 1 counts at LSI / 8, so an elapsed time in milliseconds IS the
/// LSI: f = (RL + 1) x 8 x 1000 / ms.
constexpr uint32_t leg_lsi_numerator = (leg_reload + 1UL) * 8UL * 1000UL;
/// How long the boot after that reset sits with nothing refreshing: three
/// time-outs, so a watchdog the reset did not stop cannot fail to bite.
constexpr uint32_t quiet_ms = 3000;

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
        service();
    }
}

/// Leg 1: the CPU's own SYSRESETREQ, with the flags cleared first so the
/// next boot sees exactly what this reset raises and nothing else.
[[noreturn]] void leg_software() {
    bank(1);
    print(serial, "  leg 1: Reset::software() with the flags cleared first ...", crlf);
    console_drain();
    Reset::clear_flags();
    token.flags_before = 0;
    Reset::software();
}

/// Leg 2: an IWDG time-out, MEASURED - and the LSI with it.
///
/// The flags are deliberately NOT cleared, so the next boot is also
/// where the accumulation question is settled; and the elapsed
/// milliseconds banked in .noinit turn a reboot into a measurement.
[[noreturn]] void leg_iwdg() {
    bank(2);
    token.quiet = Quiet::untried;
    print(serial, "  leg 2: the IWDG at /8 with reload 0xEEE (",
          iwdg_nominal_ms(leg_prescaler, leg_reload),
          " ms nominal, against 512 unconfigured), flags left standing ...", crlf);
    console_drain();

    token.elapsed_ms = 0;
    token.armed = Iwdg::arm(IwdgConfig{leg_prescaler, leg_reload}) ? 1u : 0u;
    // THE WITNESS IN ITS POSITIVE DIRECTION, which no letter of `z` can
    // reach: with the watchdog live and LSION never written, the LSI is
    // forced on and Iwdg::running() must say so (7.2.9).
    token.armed_running = Iwdg::running() ? 1u : 0u;
    // THROUGH A VOLATILE POINTER, and it is not decoration: a plain
    // store to a non-volatile object inside a loop that never exits is
    // one gcc is entitled to sink out of the loop - which is to say,
    // never to perform, and the leg then banks a beautiful zero.
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
    print(serial, "  leg 3: the WWDG refreshed ABOVE its window, flags cleared "
          "first ...", crlf);
    console_drain();
    Reset::clear_flags();
    token.flags_before = 0;

    Wwdg::bus_clock(true);
    (void)Wwdg::configure(WwdgConfig{WwdgPrescaler::div1, 0x40, false});
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
    print(serial, "  leg 4: panic() through ResetReporter, flags cleared first ...",
          crlf);
    console_drain();
    Reset::clear_flags();
    token.flags_before = 0;
    panic<P, ResetReporter>(PanicCode::assert_failed, 0x5A);
}

/// Leg 5: a deliberate HardFault, caught by hard_fault_reset().
///
/// UDF, and it has to be UDF: the permanently-undefined instruction is
/// the one thing no compiler can turn into something legal. With the
/// three configurable faults disabled - the state every boot starts in -
/// it is a UsageFault that ESCALATES, so HFSR.FORCED must stand beside
/// UFSR.UNDEFINSTR in the record the handler banks.
[[noreturn]] void leg_fault() {
    token.code = static_cast<uint8_t>(PanicCode::kernel_fault);
    token.context = 0x77;
    token.fault_cfsr = 0;
    token.fault_hfsr = 0;
    bank(5);
    print(serial, "  leg 5: UDF -> HardFault -> hard_fault_reset() ...", crlf);
    console_drain();
    __asm__ volatile("udf #0");
    await_reset("(the undefined instruction did not fault)");
}

/// Leg 6: the same wreck at ITS OWN VECTOR - a divide by zero with
/// DIV_0_TRP armed and UsageFault enabled, so the core takes
/// UsageFault_Handler instead of escalating.
volatile int32_t zero_divisor = 0;
volatile int32_t quotient = 0;

[[noreturn]] void leg_usage_fault() {
    token.code = static_cast<uint8_t>(PanicCode::kernel_fault);
    token.context = 0x66;
    token.fault_cfsr = 0;
    token.fault_hfsr = 0;
    bank(6);
    print(serial, "  leg 6: a divide by zero with DIV_0_TRP and USGFAULTENA on "
          "-> UsageFault ...", crlf);
    console_drain();
    Faults::enable(false, false, true);
    Faults::divide_by_zero_trap(true);
    quotient = 100 / zero_divisor;
    await_reset("(the divide by zero did not fault)");
}

/// Leg 7: the WWDG left to run down to 0x3F with nothing refreshing it,
/// its early warning armed on the way.
[[noreturn]] void leg_wwdg_timeout() {
    bank(7);
    token.ewi_seen = 0;
    token.ewi_counter = 0;
    print(serial, "  leg 7: the WWDG activated, its EARLY WARNING armed, and "
          "never refreshed ...", crlf);
    console_drain();
    Wwdg::bus_clock(true);
    Wwdg::clear_flag();
    Nvic::clear_pending(Wwdg::irq());
    Nvic::enable(Wwdg::irq());
    // THE ONE PLACE THE HARDWARE EWI REQUEST CAN BE PROVEN WITHOUT
    // COSTING ANYTHING. Activating the WWDG is one-way (22.3), so no
    // letter of `z` may do it - but this leg is about to be reset by
    // that very watchdog, so the activation costs nothing here. The
    // handler banks what it saw in .noinit and does NOT refresh: the
    // reset follows one step later, which is the point.
    (void)Wwdg::configure(WwdgConfig{WwdgPrescaler::div8, 0x7F, true});
    Wwdg::start(0x7F);
    await_reset("(waiting for the warning, then the counter reaching 0x3F)");
}

void ti_resets() {
    report_boot();
    bench.verdict("this boot names a reset source", (boot_flags & ResetFlag::all) != 0u);
    bench.verdict("no panic record is pending on a clean start", !boot_record);
    print(serial, "  NOTE: leg 2 starts the IWDG, and nothing in software can "
          "stop it (RM0090 21.3) - whether the reset it causes does is the "
          "leg's own measurement", crlf);
    leg_software();
}

/// Everything after a reset. Called from main() instead of the banner,
/// and it either starts the next leg (never returning) or closes the
/// letter.
void ti_resume() {
    bench.resume_tally(token.pass, token.fail);

    const uint8_t leg = token.leg;
    print(serial, crlf, "i (continued after reset ", leg, " of 7)", crlf);
    report_boot();

    if (leg == 1) {
        bench.verdict("Reset::software() resets the device and raises SFTRSTF",
                      (boot_flags & ResetFlag::software) != 0u);
        // WHAT A SOFTWARE RESET RAISES BESIDE ITS OWN BIT. 7.3.21
        // describes PINRSTF as the NRST pin's, but the pad is driven by
        // the internal sources too - this is the leg that shows what
        // this silicon really does, with the flags cleared a moment
        // before and the pin untouched.
        print(serial, "  a software reset raised: ");
        print_flags(boot_flags);
        print(serial, crlf);
        bench.verdict("PINRSTF is raised BY THE SOFTWARE RESET TOO - the NRST "
                      "pad is driven by the internal sources, so the bit names "
                      "the pin only when it stands alone",
                      (boot_flags & ResetFlag::pin) != 0u);
        bench.verdict("and nothing else came with them - no supply flag, no "
                      "watchdog",
                      (boot_flags & ~(ResetFlag::software | ResetFlag::pin)) == 0u);
        leg_iwdg();
    }

    if (leg == 2) {
        // THE MEASUREMENT: the elapsed milliseconds ARE the LSI.
        const uint32_t ms = token.elapsed_ms;
        const uint32_t lsi = ms != 0u ? leg_lsi_numerator / ms : 0u;
        print(serial, "  the IWDG bit after ", ms, " ms of kernel tick (",
              iwdg_nominal_ms(leg_prescaler, leg_reload),
              " nominal, 512 if nothing had landed); arm() answered ",
              token.armed != 0u ? "true" : "false", "; LSI = ", lsi, " Hz", crlf);
        bench.verdict("an IWDG time-out resets the device",
                      (boot_flags & ResetFlag::independent_watchdog) != 0u);
        bench.verdict("arm() reported the configuration landed",
                      token.armed != 0u);
        bench.verdict("and with the watchdog live Iwdg::running() read TRUE - "
                      "the start forced the LSI on with LSION never written, "
                      "which is the witness in the direction `z` cannot reach",
                      token.armed_running != 0u);
        bench.verdict("and the time-out proves it on the wall clock: it is the "
                      "configured 955 ms nominal and not the 512 ms an "
                      "unconfigured watchdog would take",
                      ms >= 860u && ms <= 1100u);
        bench.verdict("the LSI it implies is inside the datasheet's 17..47 kHz",
                      lsi >= 17'000u && lsi <= 47'000u);

        // THE ACCUMULATION QUESTION, ANSWERED. This leg did not clear
        // the flags, so leg 1's SFTRSTF must still be standing beside
        // the new IWDGRSTF.
        print(serial, "  flags standing when the leg started: ");
        print_flags(token.flags_before);
        print(serial, crlf);
        bench.verdict("THE FLAGS ACCUMULATE: leg 1's SFTRSTF is still here "
                      "beside the new IWDGRSTF, because nothing wrote RMVF in "
                      "between (7.3.21 - a history, not a cause)",
                      (boot_flags & ResetFlag::software) != 0u &&
                          (boot_flags & ResetFlag::independent_watchdog) != 0u);

        // THE OTHER READING OF "IT CANNOT BE STOPPED". RM0090 5.3.4 ends
        // the sentence with "except by a Reset" - if the reset the
        // watchdog itself caused is such a reset, this boot can sit
        // still for three time-outs and live. THE SILENCE IS THE
        // MEASUREMENT, and it is banked before it starts so that a boot
        // which finds it "trying" knows the silence killed it.
        print(serial, "  LSI witness at this boot: LSION=", boot_lsi_on, " LSIRDY=",
              boot_lsi_ready, " -> Iwdg::running() would read ",
              boot_lsi_ready && !boot_lsi_on, crlf);
        if (token.quiet == Quiet::untried) {
            print(serial, "  sitting still for ", quiet_ms,
                  " ms with NOTHING refreshing the watchdog ...", crlf);
            console_drain();
            token.quiet = Quiet::trying;
            const uint32_t q0 = Ticker::ticks();
            while (Ticker::ticks() - q0 < quiet_ms) {
            }
            token.quiet = Quiet::survived;
        } else if (token.quiet == Quiet::trying) {
            token.quiet = Quiet::bitten;
        }
        const bool stopped = token.quiet == Quiet::survived;
        print(serial, "  ... the board ", stopped ? "LIVED" : "was reset again",
              crlf);
        bench.verdict("THE RESET STOPPED IT: a boot after an IWDG reset outlives "
                      "three of its own time-outs with nobody feeding it, so "
                      "'cannot be stopped except by a Reset' (5.3.4) is literal "
                      "and a program does not inherit a watchdog for ever",
                      stopped);
        // Whichever way it went, the rest of the letter must measure its
        // own legs and not this watchdog.
        feed_iwdg = !stopped;
        leg_wwdg_window();
    }

    if (leg == 3) {
        bench.verdict("a WWDG refresh above the window resets the device",
                      (boot_flags & ResetFlag::window_watchdog) != 0u);
        bench.verdict("RMVF really took the earlier flags down: neither SFTRSTF "
                      "nor IWDGRSTF is standing",
                      (boot_flags &
                       (ResetFlag::software | ResetFlag::independent_watchdog)) == 0u);
        bench.verdict("the WWDG came back DISABLED (WDGA is cleared by the "
                      "reset - 22.6.1)",
                      !Wwdg::enabled());
        bench.verdict("and its bus clock came back OFF with it, as a reset "
                      "leaves every APB1 enable",
                      !boot_wwdg_clocked);
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
        const FaultRecord r{token.fault_cfsr, token.fault_hfsr, token.fault_address};
        print(serial, "  the wreck as the handler read it: CFSR=", hex(r.cfsr),
              " HFSR=", hex(r.hfsr), " address=", hex(r.address), crlf);
        bench.verdict("a HardFault reaches hard_fault_reset() and resets",
                      (boot_flags & ResetFlag::software) != 0u);
        bench.verdict("the record it left says kernel_fault",
                      boot_record && boot_record->code == token.code);
        bench.verdict("with the context byte the body was given",
                      boot_record && boot_record->context == token.context);
        bench.verdict("and the FAULT record says what the core saw: an "
                      "undefined instruction, ESCALATED to HardFault because "
                      "UsageFault was disabled (HFSR.FORCED)",
                      r.usage_fault() && (r.cfsr & SCB_CFSR_UNDEFINSTR_Msk) != 0u &&
                          r.escalated());
        leg_usage_fault();
    }

    if (leg == 6) {
        const FaultRecord r{token.fault_cfsr, token.fault_hfsr, token.fault_address};
        print(serial, "  the wreck as the handler read it: CFSR=", hex(r.cfsr),
              " HFSR=", hex(r.hfsr), " address=", hex(r.address), crlf);
        bench.verdict("a UsageFault reaches its OWN vector when SHCSR enables "
                      "it, and the same body resets from there",
                      (boot_flags & ResetFlag::software) != 0u);
        bench.verdict("the record says kernel_fault with this leg's context",
                      boot_record && boot_record->code == token.code &&
                          boot_record->context == token.context);
        bench.verdict("CFSR names the divide by zero (UFSR.DIVBYZERO), which "
                      "only DIV_0_TRP makes a fault at all",
                      (r.cfsr & SCB_CFSR_DIVBYZERO_Msk) != 0u);
        bench.verdict("and NOTHING escalated: an enabled configurable fault is "
                      "taken at its own vector, so HFSR.FORCED is clear",
                      !r.escalated());
        bench.verdict("the enables did not survive the reset: SHCSR is back at "
                      "its reset value",
                      (boot_shcsr & SCB_SHCSR_USGFAULTENA_Msk) == 0u);
        leg_wwdg_timeout();
    }

    // leg 7: the last boot.
    bench.verdict("a WWDG counter reaching 0x3F resets the device",
                  (boot_flags & ResetFlag::window_watchdog) != 0u);
    print(serial, "  the early warning ", token.ewi_seen ? "FIRED" : "did not fire",
          " on the way down; the counter read ", hex(token.ewi_counter),
          " inside its handler", crlf);
    bench.verdict("with the watchdog ACTIVATED the early-wakeup interrupt really "
                  "is requested - the half letter c cannot stage, paid for by a "
                  "reset that was going to happen anyway",
                  token.ewi_seen != 0u);
    bench.verdict("and it fired at the warning value: the counter it saw was "
                  "0x40 or the 0x3F it was about to become",
                  token.ewi_counter == 0x40u || token.ewi_counter == 0x3Fu);
    bench.verdict("the token crossed all seven resets intact",
                  token.magic == token_magic && token.canary == token_canary);

    token.leg = 0;
    token.magic = 0;
    bench.end_letter();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_stm32f4_reset - the reset flags, both watchdogs "
          "and the fault vectors (RM0090 7.3.21, ch. 21, ch. 22, PM0214 4.3), "
          "clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler
// is a spin loop - so every line this suite can raise is bound.
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void WWDG_IRQHandler() {
    const uint8_t t = brio::Wwdg::counter();
    if (brio::Wwdg::isr()) {
        ewi_count = static_cast<uint16_t>(ewi_count + 1);
        if (token.magic == token_magic && token.leg == 7) {
            token.ewi_seen = 1;
            token.ewi_counter = t;
        }
    }
}

/// The whole point of stm32f4/reset.hpp's fault body: a crash becomes a
/// note the next boot can read, instead of a spin nobody sees. The
/// STATUS registers are read into the suite's own token first - the
/// driver owns no storage for them, by design - and then the body writes
/// the panic record and resets.
static void bank_fault_and_reset() {
    const brio::FaultRecord r = brio::Faults::take();
    if (token.magic == token_magic) {
        token.fault_cfsr = r.cfsr;
        token.fault_hfsr = r.hfsr;
        token.fault_address = r.address;
    }
    brio::hard_fault_reset<P>(token.context);
}

extern "C" void HardFault_Handler() { bank_fault_and_reset(); }
extern "C" void UsageFault_Handler() { bank_fault_and_reset(); }

int main() {
    // Sampled FIRST: the flags are not cleared by reading, but the panic
    // record is fetch-and-clear and must be taken exactly once.
    boot_flags = brio::Reset::flags();
    boot_record = brio::take_panic_record<P>();
    boot_lsi_on = brio::Rcc::lsi_enabled();
    boot_lsi_ready = brio::Rcc::lsi_ready();
    boot_wwdg_clocked = brio::Wwdg::bus_clock();
    boot_shcsr = SCB->SHCSR;
    boot_fault = brio::Faults::read();

    // A watchdog letter i left running - if the reset it caused did not
    // stop it - must be fed from here on, or the legs after it measure
    // its bite instead of their own. Refreshing a stopped watchdog is
    // nothing at all, so this is safe on every boot.
    if (token.magic == token_magic && token.leg != 0) {
        brio::Iwdg::refresh();
    }

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the boot story: the flags as the history they are", ta_boot);
    bench.letter('b', "the IWDG, never started", tb_iwdg);
    bench.letter('c', "the WWDG, never activated", tc_wwdg);
    bench.letter('d', "the fault vectors, without faulting", td_faults);
    bench.letter('i', "SEVEN REAL RESETS (reboots the board)", ti_resets, false);

    // A pending token means a leg of letter i is waiting to be judged:
    // resume it instead of printing a banner nobody asked for.
    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        ti_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " flags=",
                    brio::hex(boot_flags), brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        service();
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
