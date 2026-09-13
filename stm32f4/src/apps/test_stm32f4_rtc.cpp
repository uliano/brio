// test_stm32f4_rtc - the reference bench suite for the STM32F4's RTC and
// the backup domain it lives in: stm32f4/rtc.hpp (RM0383 ch. 17 on the
// bench part, RM0090 ch. 26 and RM0390 ch. 22 on its siblings) and, with
// it, the RCC_BDCR half of the clock chapter that only this domain can
// reach.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, AND THE RULER IS THE CORE'S OWN SYSTICK. Every
// frequency here is a ratio against HCLK, which comes from the board's
// HSE crystal through the PLL, so a crystal weighs a crystal; the core's
// own absolute error is stated where it matters and cancels out of every
// comparison that is a difference. The instrument is RTC_SSR: it counts
// down at ck_apre and RELOADS at each calendar update, so a polling loop
// catches the 1 Hz edge to within its own iteration - a few hundred
// nanoseconds at 100 MHz, well under a part per million over a
// two-second window. The suite therefore runs with BYPSHAD = 1 (a shadow
// copy is refreshed only every two RTCCLK cycles, which would put 30 ppm
// of jitter on every edge); letter e is where the shadow path is
// exercised, and it restores the bypass on its way out.
//
// NO FLASH IS WRITTEN and no option byte is touched.
//
// WHAT IT COSTS THE BOARD, said once: letter c RESETS THE BACKUP DOMAIN
// (RCC_BDCR.BDRST) twice, because RTCSEL is one-way and weighing the LSI
// means selecting it - and a domain reset wipes the calendar, the
// prescalers, the alarms and THE TWENTY BACKUP REGISTERS. Letter l and
// the reboot letter v are written to be run after that, not before, and
// z runs them in that order.
//
// THE PAD. RTC_AF1 is PC13 on every part of this family: the tamper
// input, the timestamp input and RTC_OUT (the calibration output and the
// alarm output) are all that one pin. It carries the LED on the black
// pill (lit when low), the user button on a Nucleo-64, and toggling it is
// ES0287 2.2.10 ("PC13 signal transitions disturb LSE") - so letter m
// weighs the crystal before and after driving it, and letters w and x,
// which leave the pad in the RTC's hands, sit outside z.
//
// What is exercised, letter by letter:
//   a  the domain: the two locks (PWR_CR.DBP and the RTC's own WPR keys),
//      what each of them covers, RTCSEL as a ONE-WAY choice, RTCEN
//   b  LSE: how long this board's crystal takes to start, what it
//      measures against the core clock, and the two register rules
//   c  LSI weighed the same way - which costs two domain resets
//   d  the calendar: BCD both ways, INITS, and four boundaries crossed
//      one second at a time
//   e  the shadow registers against BYPSHAD: RSF, the read cost, the
//      coherence rule and ES0287 2.8.2's re-read
//   f  both alarms: the match, the LATENCY from the match to the
//      handler, the sub-second comparison, and the chapter's own
//      caution about a seconds match under a narrow PREDIV_S
//   g  the wake-up timer on three of its clocks, each period weighed,
//      plus the write window and the two refusals
//   h  ES0287 2.8.4 STAGED: consecutive initialization-mode entries with
//      and without the driver's workaround
//   i  smooth calibration MEASURED, not asserted: CALP and CALM at both
//      ends, each weighed as the length of the calendar's second
//   j  coarse calibration: the register, the three refusals, and the
//      interlock between the two calibrators
//   k  the sub-second register and RTC_SHIFTR: a shift seen as the
//      LENGTH of the one second it lands in
//   l  the twenty backup registers, and which lock covers them
//   m  the RTC_OUT pad: the calibration output COUNTED on PC13's own
//      input register, the alarm output, and the crystal weighed again
//      afterwards (ES0287 2.2.10)
//   n  the timestamp on RTC_AF1, and what this board can and cannot
//      stage on that pad with no wire
//
//   v  (by name only) THE SURVIVAL LETTER. Writes a token into the backup
//      registers, and on its next run - after a system reset, which a
//      re-flash of the same image ends with - checks that it came back.
//      Run it twice with the flash in between:
//          brio run <board> v --app test_stm32f4_rtc
//          brio flash <board> test_stm32f4_rtc
//          brio run <board> v --app test_stm32f4_rtc
//   w  (by name only) THE ERASE. Arms a real tamper on RTC_AF1 - which
//      this board can do with no wire, and which SPENDS the twenty backup
//      registers - and takes the timestamp the tamper writes.
//   x  (by name only) the 1 Hz calibration output LEFT ON, for a human to
//      watch on the LED. It outlives a reset - RTC_CR is not in a system
//      reset's scope - so main() takes the pad back at every boot.
//
// build: boards = f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/rtc.hpp"
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

/// RTC_AF1: the one pad this whole chapter's inputs and outputs share.
using RtcPad = Pin<rtc_af1_port, rtc_af1_pin>;

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// The nominal crystal: the board file's fact, not the driver's.
constexpr uint32_t lse_nominal_hz = 32768;

// ---------------------------------------------------------------------------
// The ruler: SysTick cycles, whole milliseconds from the kernel tick and
// the rest from VAL.
// ---------------------------------------------------------------------------

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// A wait in whole milliseconds on the kernel tick. delay_us() is capped
/// below one SysTick period by construction (stm32f4/delay.hpp), so
/// anything longer is the ticker's business.
void wait_ms(uint32_t ms) {
    const uint32_t started = Ticker::ticks();
    while (Ticker::ticks() - started <= ms) {
    }
}

/// A monotonic cycle count. THE PENDING BIT IS THE RACE: SysTick can wrap
/// between the VAL read and the tick read, and the handler that would
/// advance the tick has not run yet - so ICSR's PENDSTSET is consulted
/// and the tick corrected by hand. Interrupts are masked for the handful
/// of cycles the read takes, which delays the tick and never loses it.
uint64_t cycles_now() {
    const uint32_t period = systick_period();
    P::CriticalSection cs;
    uint32_t v = SysTick->VAL;
    uint32_t t = Ticker::ticks();
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0u) {
        t += 1u;
        v = SysTick->VAL;
    }
    return static_cast<uint64_t>(t) * period + (period - 1u - v);
}

/// Parts per million by which `cycles` differs from `expected`, positive
/// when there are FEWER cycles than expected - i.e. when whatever was
/// being weighed ran FAST.
int32_t ppm_of(uint64_t cycles, uint64_t expected) {
    if (expected == 0u || cycles == 0u) {
        return 0;
    }
    const int64_t delta = static_cast<int64_t>(expected) - static_cast<int64_t>(cycles);
    return static_cast<int32_t>((delta * 1'000'000LL) / static_cast<int64_t>(expected));
}

/// Print a whole number of hertz with three decimals, from a count of
/// millihertz.
void print_hz(uint64_t millihertz) {
    print(serial, static_cast<uint32_t>(millihertz / 1000u), ".");
    const uint32_t frac = static_cast<uint32_t>(millihertz % 1000u);
    if (frac < 100u) {
        print(serial, "0");
    }
    if (frac < 10u) {
        print(serial, "0");
    }
    print(serial, frac, " Hz");
}

/// Catch the RTC_SSR reload that opens a second and the one that closes
/// the n-th, and return the cycles between them. False when the RTC is
/// not counting.
bool measure_seconds(uint8_t n, uint32_t& cycles) {
    const uint32_t started = Ticker::ticks();
    const uint32_t budget = 1500u * (static_cast<uint32_t>(n) + 2u);
    uint16_t prev = Rtc::subsecond();
    uint64_t first = 0;
    uint8_t seen = 0;
    while (Ticker::ticks() - started < budget) {
        const uint16_t now = Rtc::subsecond();
        if (now > prev) {   // the counter counts DOWN: a rise is a reload
            const uint64_t at = cycles_now();
            if (seen == 0u) {
                first = at;
            }
            ++seen;
            if (seen == static_cast<uint8_t>(n + 1u)) {
                cycles = static_cast<uint32_t>(at - first);
                return true;
            }
        }
        prev = now;
    }
    return false;
}

/// Wait, on the kernel tick, for one of the RTC's event flags to stand.
bool wait_flag(uint32_t mask, uint32_t budget_ms) {
    const uint32_t started = Ticker::ticks();
    while (Ticker::ticks() - started < budget_ms) {
        if (Rtc::flag(mask)) {
            return true;
        }
    }
    return false;
}

/// Wait for the CALENDAR's seconds field to change and stamp it. Slower
/// to read than RTC_SSR (a whole coherent reading, some 180 cycles) and
/// immune to what a shift does to the sub-second counter, which a
/// reload-watcher reads as a second of its own.
bool wait_calendar_second(uint64_t& at, uint32_t budget_ms = 2500) {
    RtcReading r{};
    if (!Rtc::read(r)) {
        return false;
    }
    const uint8_t from = r.time.second;
    const uint32_t started = Ticker::ticks();
    while (Ticker::ticks() - started < budget_ms) {
        if (Rtc::read(r) && r.time.second != from) {
            at = cycles_now();
            return true;
        }
    }
    return false;
}

/// Wait for the next RTC_SSR reload and stamp it.
bool wait_second_edge(uint64_t& at, uint32_t budget_ms = 1500) {
    const uint32_t started = Ticker::ticks();
    uint16_t prev = Rtc::subsecond();
    while (Ticker::ticks() - started < budget_ms) {
        const uint16_t now = Rtc::subsecond();
        if (now > prev) {
            at = cycles_now();
            return true;
        }
        prev = now;
    }
    return false;
}

// ---------------------------------------------------------------------------
// The domain, brought up on LSE - what main() does and what letter c has
// to undo after it has been on the LSI.
// ---------------------------------------------------------------------------

constexpr RtcDateTime boot_time{12, 0, 0, 1, 6, 24, 6};   // Saturday 1 June 2024, noon

/// Cycles the crystal took from LSEON to LSERDY at the last bring-up, and
/// whether it was already running when the program found it - which it is
/// after every reset that is not a power-on, the domain being outside the
/// system reset's scope.
uint32_t lse_start_cycles = 0;
bool lse_was_running = false;

bool bring_up(RtcClockSource source) {
    RtcDomain::pwr_bus_clock(true);
    if (!RtcDomain::unlock(true)) {
        return false;
    }
    if (RtcDomain::selected() != source) {
        RtcDomain::reset();
    }
    lse_was_running = RtcDomain::lse_ready();
    const uint64_t t0 = cycles_now();
    if (source == RtcClockSource::lse) {
        RtcDomain::lse_enable(true);
        if (!RtcDomain::lse_wait_ready()) {
            return false;
        }
    } else {
        Rcc::lsi_enable(true);
        if (!Rcc::lsi_wait_ready()) {
            return false;
        }
    }
    lse_start_cycles = static_cast<uint32_t>(cycles_now() - t0);
    if (!RtcDomain::select(source) || !RtcDomain::open(source)) {
        return false;
    }
    Rtc::bypass_shadow(true);
    // TAKE RTC_AF1 BACK. RTC_CR is not reset by a system reset, so a
    // calibration or alarm output left on the pad by letter x is still
    // there at the next boot - and a pad toggling at 512 Hz is ES0287
    // 2.2.10's own hazard to the crystal every other letter weighs.
    (void)Rtc::calibration_output(false);
    Rtc::alarm_output(RtcOutput::off);
    if (!Rtc::calendar_set() && !Rtc::init(RtcPrescalers{}, boot_time)) {
        return false;
    }
    return Rtc::wait_sync();
}

// ---------------------------------------------------------------------------
// What the three handlers leave behind
// ---------------------------------------------------------------------------

volatile uint32_t alarm_served = 0;
volatile uint32_t alarm_count = 0;
volatile uint64_t alarm_at = 0;
volatile uint32_t wakeup_served = 0;
volatile uint32_t wakeup_count = 0;
volatile uint64_t wakeup_at = 0;
volatile uint32_t stamp_served = 0;
volatile uint32_t stamp_count = 0;
volatile uint64_t stamp_at = 0;
RtcReading stamp_reading{};

void arm_vectors() {
    Nvic::enable(Rtc::alarm_irq());
    Nvic::enable(Rtc::wakeup_irq());
    Nvic::enable(Rtc::tamper_stamp_irq());
}

// =============================================================================
// a - the domain and its two locks
// =============================================================================
void ta_domain() {
    print(serial, "  BDCR ", hex(RtcDomain::bdcr()), " PWR_CR ", hex(PWR->CR), " ISR ",
          hex(Rtc::isr()), " PRER ", hex(Rtc::prer()), crlf);

    bench.verdict("the PWR block's bus clock is open", RtcDomain::pwr_bus_clock());
    bench.verdict("RTCSEL is the LSE and RTCEN is set",
                  RtcDomain::selected() == RtcClockSource::lse && RtcDomain::enabled());
    bench.verdict("re-selecting the source already in force is a no-op and true",
                  RtcDomain::select(RtcClockSource::lse));
    bench.verdict("selecting a DIFFERENT source is refused: RTCSEL is one-way",
                  !RtcDomain::select(RtcClockSource::lsi));

    // DBP: the domain's own lock, over RCC_BDCR and the backup registers.
    const uint32_t keep = Rtc::backup(0);
    bench.verdict("DBP is open", RtcDomain::unlocked());
    (void)RtcDomain::unlock(false);
    bench.verdict("and it closes", !RtcDomain::unlocked());
    (void)Rtc::backup(0, 0xDEADBEEFu);
    const bool locked_out = Rtc::backup(0) != 0xDEADBEEFu;
    (void)RtcDomain::unlock(true);
    (void)Rtc::backup(0, 0xDEADBEEFu);
    const bool let_in = Rtc::backup(0) == 0xDEADBEEFu;
    (void)Rtc::backup(0, keep);
    bench.verdict("a backup-register write with DBP CLOSED does not land", locked_out);
    bench.verdict("the same write with DBP open does", let_in);

    // The RTC's own key, over RTC_CR and its neighbours - proven on CALR,
    // the one register a wrong value cannot hurt.
    const uint32_t calr_was = Rtc::calr();
    Rtc::lock();
    Rtc::calr_unprotected(0x0000'0100u);
    const bool key_holds = Rtc::calr() == calr_was;
    Rtc::unlock();
    Rtc::calr_unprotected(0x0000'0100u);
    const bool key_opens = Rtc::calr() == 0x0000'0100u;
    const bool blocked_after = Rtc::recalibration_pending();
    Rtc::lock();
    bench.verdict("a store into RTC_CALR with the key LOCKED is dropped", key_holds);
    bench.verdict("the same store after 0xCA then 0x53 lands", key_opens);
    // 17.6.4: the store opens a recalibration window in which the
    // register "is blocked" - which is why calibrate() waits RECALPF out
    // and calr_unprotected(), the bench's own back door, does not.
    bench.verdict("and it leaves RECALPF standing, so the next raw store would be lost",
                  blocked_after);
    bench.verdict("the driver's calibrate() waits that window out and puts the register back",
                  Rtc::calibrate(RtcCalibration{}) && Rtc::calr() == 0u);

    // RTC_TAFCR sits outside the key (17.3.5's exception list).
    const bool tamp_ts_was = Rtc::timestamp_on_tamper();
    Rtc::lock();
    Rtc::timestamp_on_tamper(!tamp_ts_was);
    const bool tafcr_open = Rtc::timestamp_on_tamper() == !tamp_ts_was;
    Rtc::timestamp_on_tamper(tamp_ts_was);
    bench.verdict("RTC_TAFCR is OUTSIDE the key: a write lands with it locked", tafcr_open);

    // And so does the flag half of RTC_ISR.
    Rtc::clear_flags(RtcFlag::all);
    bench.verdict("no event flag stands after a clear with the key locked",
                  (Rtc::status() & RtcFlag::all) == 0u);
}

// =============================================================================
// b - the LSE crystal: how long it takes and what it is worth
// =============================================================================
void tb_lse() {
    const uint32_t cycles_per_ms = SysClock::hz / 1000u;
    if (lse_was_running) {
        print(serial, "  at boot the crystal was ALREADY running: the backup domain is outside "
                      "a system reset's scope", crlf);
    } else {
        print(serial, "  at boot LSERDY came ", lse_start_cycles / cycles_per_ms,
              " ms after LSEON", crlf);
    }

    // The two register rules, checked while the oscillator runs.
    bench.verdict("LSEBYP is refused while the oscillator is on", !RtcDomain::lse_bypass(true));
    if constexpr (RtcDomain::has_lse_mode()) {
        bench.verdict("LSEMOD is refused while the oscillator is on",
                      !RtcDomain::lse_mode(LseMode::high_drive));
    } else {
        bench.verdict("this part's RCC_BDCR has no LSEMOD, and the verb says so",
                      !RtcDomain::lse_mode(LseMode::high_drive));
    }

    // Stop it and start it again, with the ruler running.
    RtcDomain::lse_enable(false);
    uint32_t spins = 0;
    while (RtcDomain::lse_ready() && spins < RtcDomain::ready_spins) {
        ++spins;
    }
    const bool stopped = !RtcDomain::lse_ready();
    bench.verdict("LSEON clear brings LSERDY down", stopped);
    bench.verdict("with the oscillator stopped, LSEBYP is writable",
                  RtcDomain::lse_bypass(true) && RtcDomain::lse_bypass());
    (void)RtcDomain::lse_bypass(false);
    bench.verdict("and writable back", !RtcDomain::lse_bypass());

    const uint64_t t0 = cycles_now();
    RtcDomain::lse_enable(true);
    const bool back = RtcDomain::lse_wait_ready();
    const uint32_t start_cycles = static_cast<uint32_t>(cycles_now() - t0);
    print(serial, "  a cold start took ", start_cycles / cycles_per_ms, " ms (",
          start_cycles / (SysClock::hz / 1'000'000u), " us)", crlf);
    bench.verdict("the crystal starts again", back);
    bench.verdict("and it takes between a millisecond and four seconds",
                  start_cycles > cycles_per_ms && start_cycles < 4u * SysClock::hz);

    (void)Rtc::wait_sync();

    // The crystal, weighed against the core's own clock over two seconds.
    uint32_t cycles = 0;
    const bool measured = measure_seconds(2, cycles);
    bench.verdict("two calendar seconds can be caught on RTC_SSR's reload", measured);
    if (!measured) {
        return;
    }
    const uint32_t per_second = rtc_cycles_per_second(Rtc::prescalers());
    const uint64_t millihertz =
        (static_cast<uint64_t>(per_second) * 2ULL * SysClock::hz * 1000ULL) / cycles;
    const int32_t ppm = ppm_of(cycles, 2ULL * SysClock::hz);
    print(serial, "  two seconds = ", cycles, " core cycles; RTCCLK = ");
    print_hz(millihertz);
    print(serial, ", ", ppm, " ppm fast against the core clock", crlf);
    bench.verdict("the crystal is within 200 ppm of 32768 Hz",
                  millihertz > 1000ULL * (lse_nominal_hz - 7u) &&
                      millihertz < 1000ULL * (lse_nominal_hz + 7u));
    bench.verdict("and the calendar's second is within 200 ppm of the core's",
                  ppm > -200 && ppm < 200);
}

// =============================================================================
// c - the LSI on the same ruler, and what it costs
// =============================================================================
void tc_lsi() {
    print(serial, "  this letter resets the backup domain twice: the calendar, "
                  "the alarms and the twenty backup registers are spent", crlf);

    const bool to_lsi = bring_up(RtcClockSource::lsi);
    bench.verdict("a domain reset lets RTCSEL move, and the LSI is selected",
                  to_lsi && RtcDomain::selected() == RtcClockSource::lsi);
    bench.verdict("the domain reset wiped the backup registers", Rtc::backup(0) == 0u);

    uint32_t cycles = 0;
    if (to_lsi && measure_seconds(2, cycles)) {
        const uint32_t per_second = rtc_cycles_per_second(Rtc::prescalers());
        const uint64_t millihertz =
            (static_cast<uint64_t>(per_second) * 2ULL * SysClock::hz * 1000ULL) / cycles;
        const int32_t off = static_cast<int32_t>(
            ((static_cast<int64_t>(millihertz) - 1000LL * lse_nominal_hz) * 1'000'000LL) /
            (1000LL * lse_nominal_hz));
        print(serial, "  the LSI measures ");
        print_hz(millihertz);
        print(serial, ", ", off, " ppm off 32768 (RM0383 says 32 kHz nominal, 17..47 kHz "
                      "over the range)", crlf);
        bench.verdict("the LSI is inside its own datasheet window",
                      millihertz > 17'000'000ULL && millihertz < 47'000'000ULL);
        bench.verdict("and it is no crystal: more than a thousand ppm off nominal",
                      off > 1000 || off < -1000);
    } else {
        bench.verdict("two calendar seconds on the LSI", false);
        bench.verdict("the LSI is inside its own datasheet window", false);
    }

    const bool back = bring_up(RtcClockSource::lse);
    bench.verdict("and the domain goes back to the crystal",
                  back && RtcDomain::selected() == RtcClockSource::lse);
    bench.verdict("with a calendar running again", Rtc::calendar_set());
}

// =============================================================================
// d - the calendar, BCD both ways and four boundaries
// =============================================================================
bool set_and_wait(const RtcDateTime& from, RtcDateTime& landed) {
    if (!Rtc::init(Rtc::prescalers(), from)) {
        return false;
    }
    uint64_t at = 0;
    if (!wait_second_edge(at, 2000)) {
        return false;
    }
    (void)delay_us(clock, 500);   // let the shadow settle behind the reload
    RtcReading r{};
    if (!Rtc::read(r)) {
        return false;
    }
    landed = r.time;
    return true;
}

void td_calendar() {
    const RtcDateTime want{13, 45, 7, 29, 2, 24, 4};
    bench.verdict("the calendar takes a setting in initialization mode",
                  Rtc::init(Rtc::prescalers(), want));
    RtcReading r{};
    const bool got = Rtc::read(r);
    print(serial, "  read back ", r.time.year, "-", r.time.month, "-", r.time.day, " ",
          r.time.hour, ":", r.time.minute, ":", r.time.second, " weekday ", r.time.weekday,
          " ss ", r.subsecond, crlf);
    bench.verdict("and gives it back through the BCD registers unchanged",
                  got && r.time.hour == 13 && r.time.minute == 45 && r.time.day == 29 &&
                      r.time.month == 2 && r.time.year == 24 && r.time.weekday == 4);
    bench.verdict("INITS says the calendar has been set", Rtc::calendar_set());
    bench.verdict("a calendar write outside initialization mode is refused",
                  !Rtc::set_calendar(want));
    bench.verdict("and an impossible date is refused inside it too",
                  !Rtc::init(Rtc::prescalers(), RtcDateTime{0, 0, 0, 31, 2, 24, 1}));

    RtcDateTime landed{};
    bench.verdict("23:59:59 rolls into midnight and the next day",
                  set_and_wait(RtcDateTime{23, 59, 59, 1, 6, 24, 6}, landed) &&
                      landed.hour == 0 && landed.minute == 0 && landed.second == 0 &&
                      landed.day == 2 && landed.month == 6);
    bench.verdict("28 February of a leap year is followed by the 29th",
                  set_and_wait(RtcDateTime{23, 59, 59, 28, 2, 24, 3}, landed) &&
                      landed.day == 29 && landed.month == 2 && landed.weekday == 4);
    bench.verdict("28 February of an ordinary year is followed by 1 March",
                  set_and_wait(RtcDateTime{23, 59, 59, 28, 2, 25, 5}, landed) &&
                      landed.day == 1 && landed.month == 3);
    bench.verdict("31 December carries the year",
                  set_and_wait(RtcDateTime{23, 59, 59, 31, 12, 24, 2}, landed) &&
                      landed.day == 1 && landed.month == 1 && landed.year == 25);
}

// =============================================================================
// e - the shadow registers against BYPSHAD
// =============================================================================
void te_shadow() {
    bench.verdict("this program's PCLK1 clears 17.3.6's seven-times rule",
                  rtc_shadow_read_allowed(SysClock::pclk1_hz, lse_nominal_hz));

    // The bypass path first - the one the rest of this suite reads in.
    bench.verdict("BYPSHAD is set", Rtc::bypass_shadow());
    const uint64_t b0 = cycles_now();
    RtcReading r{};
    bool ok = true;
    for (uint8_t i = 0; i < 16u; ++i) {
        ok = ok && Rtc::read(r);
    }
    const uint32_t bypass_cost = static_cast<uint32_t>((cycles_now() - b0) / 16u);
    bench.verdict("sixteen coherent readings come back with the shadows bypassed", ok);

    // And the shadow path.
    Rtc::bypass_shadow(false);
    bench.verdict("wait_sync clears RSF and waits for the next copy", Rtc::wait_sync());
    bench.verdict("RSF stands after it", Rtc::synchronized());
    const uint64_t s0 = cycles_now();
    ok = true;
    for (uint8_t i = 0; i < 16u; ++i) {
        ok = ok && Rtc::read(r);
    }
    const uint32_t shadow_cost = static_cast<uint32_t>((cycles_now() - s0) / 16u);
    bench.verdict("and sixteen through the shadows, with ES0287 2.8.2's re-read", ok);
    print(serial, "  a coherent read costs ", shadow_cost,
          " cycles through the shadows (SSR, TR, DR and SSR again) and ", bypass_cost,
          " with them bypassed (the whole triple twice)", crlf);

    // Coherence: the second never goes backwards across a reading, and the
    // sub-second agrees with the second it came with.
    uint8_t last = r.time.second;
    bool monotonic = true;
    const uint32_t started = Ticker::ticks();
    while (Ticker::ticks() - started < 1200u) {
        if (!Rtc::read(r)) {
            monotonic = false;
            break;
        }
        const uint8_t now = r.time.second;
        if (now != last && now != static_cast<uint8_t>((last + 1u) % 60u)) {
            monotonic = false;
            break;
        }
        last = now;
    }
    bench.verdict("over a second of shadow reads the calendar never jumps", monotonic);
    bench.verdict("and the sub-second stays inside PREDIV_S",
                  r.subsecond <= Rtc::prescalers().sync);

    Rtc::bypass_shadow(true);
    bench.verdict("the bypass is restored for the rest of the suite", Rtc::bypass_shadow());
}

// =============================================================================
// f - both alarms
// =============================================================================
void tf_alarms() {
    Rtc::clear_alarm(RtcAlarmId::a);
    Rtc::clear_alarm(RtcAlarmId::b);
    bench.verdict("with an alarm disabled its write flag rises",
                  (Rtc::isr() & RTC_ISR_ALRAWF) != 0u);

    // Alarm A on a stated second, with the seconds field compared.
    RtcReading now{};
    bench.verdict("the calendar can be read", Rtc::read(now));
    const uint8_t target = static_cast<uint8_t>((now.time.second + 3u) % 60u);
    alarm_count = 0;
    alarm_served = 0;
    RtcAlarm a{};
    a.second = target;
    a.mask_seconds = false;
    bench.verdict("alarm A takes a seconds-only match", Rtc::set_alarm(RtcAlarmId::a, a));
    bench.verdict("its EXTI line is open", Rtc::wake_line_is_open(Rtc::alarm_exti_line));

    // Follow the second edges until the handler fires: the last edge
    // before it IS the match, so the difference is the flag's latency.
    uint64_t last_edge = 0;
    const uint32_t started = Ticker::ticks();
    while (alarm_count == 0u && Ticker::ticks() - started < 5000u) {
        uint64_t at = 0;
        if (wait_second_edge(at, 1500)) {
            if (alarm_count == 0u) {
                last_edge = at;
            }
        }
    }
    const bool fired = alarm_count != 0u;
    bench.verdict("and it fires", fired);
    if (fired) {
        const uint32_t latency = static_cast<uint32_t>(alarm_at - last_edge);
        print(serial, "  alarm A landed ", latency, " core cycles (",
              latency / (SysClock::hz / 1'000'000u), " us) after the second's edge", crlf);
        bench.verdict("the handler served alarm A and nothing else",
                      alarm_served == RtcFlag::alarm_a);
        bench.verdict("and the latency is under a millisecond", latency < SysClock::hz / 1000u);
        bench.verdict("the flag is down again", !Rtc::flag(RtcFlag::alarm_a));
    } else {
        bench.verdict("the handler served alarm A and nothing else", false);
        bench.verdict("and the latency is under a millisecond", false);
        bench.verdict("the flag is down again", false);
    }
    Rtc::clear_alarm(RtcAlarmId::a);

    // Alarm B with the SUB-SECOND field: half a second into every second.
    const uint16_t half = static_cast<uint16_t>((Rtc::prescalers().sync + 1u) / 2u);
    RtcAlarm b{};
    b.subsecond_mask = 15;
    b.subsecond = half;
    bench.verdict("alarm B takes a sub-second match every second",
                  Rtc::set_alarm(RtcAlarmId::b, b));
    uint64_t edge = 0;
    const bool got_edge = wait_second_edge(edge, 1500);
    // The count is zeroed AFTER the edge: the alarm fires every second,
    // and a firing that preceded the edge would measure a negative offset.
    alarm_count = 0;
    const uint32_t waited = Ticker::ticks();
    while (alarm_count == 0u && Ticker::ticks() - waited < 1500u) {
    }
    if (got_edge && alarm_count != 0u) {
        const uint32_t offset = static_cast<uint32_t>(alarm_at - edge);
        const uint32_t ms = offset / (SysClock::hz / 1000u);
        print(serial, "  alarm B landed ", ms, " ms into the second (500 due)", crlf);
        bench.verdict("and it lands half a second in, to a few milliseconds",
                      ms >= 495u && ms <= 505u);
        bench.verdict("the handler served alarm B", (alarm_served & RtcFlag::alarm_b) != 0u);
    } else {
        bench.verdict("and it lands half a second in, to a few milliseconds", false);
        bench.verdict("the handler served alarm B", false);
    }
    Rtc::clear_alarm(RtcAlarmId::b);
    bench.verdict("both alarms are off", !Rtc::alarm_enabled(RtcAlarmId::a) &&
                                             !Rtc::alarm_enabled(RtcAlarmId::b));

    // The chapter's caution, as a refusal: a seconds match wants
    // PREDIV_S >= 2.
    const RtcPrescalers keep = Rtc::prescalers();
    RtcReading save{};
    (void)Rtc::read(save);
    const bool narrowed = Rtc::init(RtcPrescalers{127, 1}, save.time);
    const bool refused = !Rtc::set_alarm(RtcAlarmId::a, a);
    (void)Rtc::init(keep, save.time);
    bench.verdict("a seconds match under PREDIV_S = 1 is refused (17.3.3's caution)",
                  narrowed && refused);
    bench.verdict("and the prescalers are back", Rtc::prescalers().sync == keep.sync);
}

// =============================================================================
// g - the wake-up timer
// =============================================================================
bool weigh_wakeup(RtcWakeupClock c, uint32_t reload, uint8_t periods, uint32_t& cycles) {
    wakeup_count = 0;
    if (!Rtc::set_wakeup(c, reload)) {
        return false;
    }
    uint64_t first = 0;
    uint32_t seen = 0;
    const uint32_t started = Ticker::ticks();
    while (Ticker::ticks() - started < 1500u * (periods + 2u)) {
        const uint32_t n = wakeup_count;
        if (n != seen) {
            seen = n;
            if (seen == 1u) {
                first = wakeup_at;
            } else if (seen == static_cast<uint32_t>(periods) + 1u) {
                cycles = static_cast<uint32_t>(wakeup_at - first);
                return true;
            }
        }
    }
    return false;
}

void report_wakeup(const char* name, RtcWakeupClock c, uint32_t reload, uint8_t periods,
                   uint32_t expected_rtcclk_per_period) {
    uint32_t cycles = 0;
    if (!weigh_wakeup(c, reload, periods, cycles)) {
        bench.verdict(name, false);
        return;
    }
    const uint64_t expected = (static_cast<uint64_t>(expected_rtcclk_per_period) * periods *
                               SysClock::hz) /
                              lse_nominal_hz;
    const int32_t ppm = ppm_of(cycles, expected);
    print(serial, "  ", name, ": ", periods, " periods = ", cycles, " cycles, ",
          static_cast<uint32_t>(expected), " due, ", ppm, " ppm fast", crlf);
    bench.verdict(name, ppm > -300 && ppm < 300);
}

void tg_wakeup() {
    Rtc::clear_wakeup();
    bench.verdict("with the timer disabled WUTWF rises", Rtc::wakeup_write_allowed());
    bench.verdict("a reload past the 16-bit field is refused",
                  !Rtc::set_wakeup(RtcWakeupClock::div16, 0x10000u));
    bench.verdict("and RTCCLK/2 with a reload of zero is refused (17.6.6)",
                  !Rtc::set_wakeup(RtcWakeupClock::div2, 0u));

    report_wakeup("RTCCLK/16, 2048 ticks = one second", RtcWakeupClock::div16, 2047, 2, 32768);
    report_wakeup("RTCCLK/2, 16384 ticks = one second", RtcWakeupClock::div2, 16383, 2, 32768);
    report_wakeup("ck_spre, one tick = one second", RtcWakeupClock::ck_spre, 0, 2, 32768);

    bench.verdict("the wake-up EXTI line is open", Rtc::wake_line_is_open(Rtc::wakeup_exti_line));
    bench.verdict("the handler served the wake-up flag and nothing else",
                  wakeup_served == RtcFlag::wakeup);
    Rtc::clear_wakeup();
    bench.verdict("and the timer stops", !Rtc::wakeup_enabled());
}

// =============================================================================
// h - ES0287 2.8.4 staged
// =============================================================================
uint8_t hammer_init(bool with_workaround, uint8_t rounds) {
    uint8_t corrupted = 0;
    const RtcDateTime first{1, 2, 3, 4, 5, 6, 7};
    const RtcDateTime second{11, 22, 33, 14, 11, 21, 1};
    for (uint8_t i = 0; i < rounds; ++i) {
        if (!Rtc::enter_init()) {
            return 0xFF;
        }
        (void)Rtc::set_calendar(first);
        if (with_workaround) {
            (void)Rtc::exit_init();
        } else {
            Rtc::exit_init_raw();
        }
        // Straight back in: the erratum's window is one to two RTCCLK
        // cycles wide, which is where this lands with no workaround.
        if (!Rtc::enter_init()) {
            return 0xFF;
        }
        (void)Rtc::set_calendar(second);
        (void)Rtc::exit_init();
        RtcReading r{};
        if (!Rtc::read(r) || r.time.hour != second.hour || r.time.minute != second.minute ||
            r.time.second != second.second || r.time.day != second.day ||
            r.time.month != second.month || r.time.year != second.year) {
            ++corrupted;
        }
    }
    return corrupted;
}

void th_init_erratum() {
    constexpr uint8_t rounds = 200;
    const uint8_t bare = hammer_init(false, rounds);
    const uint8_t fixed_ = hammer_init(true, rounds);
    print(serial, "  ", rounds, " consecutive initialization pairs: ", bare,
          " corrupted with the bare exit, ", fixed_, " with ES0287 2.8.4's workaround", crlf);
    bench.verdict("the initialization mode is reachable at all", bare != 0xFF && fixed_ != 0xFF);
    bench.verdict("with the workaround, no calendar is corrupted", fixed_ == 0);

    RtcReading r{};
    (void)Rtc::read(r);
    (void)Rtc::init(Rtc::prescalers(), boot_time);
    bench.verdict("and the calendar can be set again afterwards", Rtc::calendar_set());
}

// =============================================================================
// i - smooth calibration, measured
// =============================================================================
int32_t weigh_calibration(const RtcCalibration& c, uint8_t seconds) {
    if (!Rtc::calibrate(c)) {
        return INT32_MIN;
    }
    // Let the setting take: 17.3.11 gives it three ck_apre cycles, and the
    // masked pulses are spread over the whole window.
    wait_ms(50);
    uint32_t cycles = 0;
    if (!measure_seconds(seconds, cycles)) {
        return INT32_MIN;
    }
    return ppm_of(cycles, static_cast<uint64_t>(seconds) * SysClock::hz);
}

void ti_calibration() {
    const int32_t base = weigh_calibration(RtcCalibration{}, 2);
    const int32_t plus = weigh_calibration(RtcCalibration{.plus = true}, 2);
    const int32_t half = weigh_calibration(RtcCalibration{.minus = 256}, 2);
    const int32_t minus = weigh_calibration(RtcCalibration{.minus = 511}, 2);
    (void)Rtc::calibrate(RtcCalibration{});

    print(serial, "  the calendar's second, in ppm against the core clock: ", base,
          " uncalibrated, ", plus, " with CALP, ", half, " with CALM 256, ", minus,
          " with CALM 511", crlf);
    print(serial, "  the arithmetic says ", rtc_calibration_ppb(RtcCalibration{.plus = true}) / 1000,
          ", ", rtc_calibration_ppb(RtcCalibration{.minus = 256}) / 1000, " and ",
          rtc_calibration_ppb(RtcCalibration{.minus = 511}) / 1000, " ppm", crlf);

    bench.verdict("all four windows can be weighed",
                  base != INT32_MIN && plus != INT32_MIN && half != INT32_MIN &&
                      minus != INT32_MIN);
    bench.verdict("CALP speeds the calendar by 488 ppm, to within 30",
                  plus - base > 458 && plus - base < 518);
    bench.verdict("CALM 256 slows it by 244 ppm, to within 30",
                  base - half > 214 && base - half < 274);
    bench.verdict("CALM 511 slows it by 487 ppm, to within 30",
                  base - minus > 457 && base - minus < 517);
    bench.verdict("and the register is back at zero", Rtc::calr() == 0u);
    bench.verdict("the 16-second window's stuck bit is refused",
                  !Rtc::calibrate(RtcCalibration{.minus = 1,
                                                 .window = RtcCalibrationWindow::seconds16}));
    bench.verdict("the 8-second window's two are refused",
                  !Rtc::calibrate(RtcCalibration{.minus = 2,
                                                 .window = RtcCalibrationWindow::seconds8}));
    bench.verdict("a legal 8-second setting is not",
                  Rtc::calibrate(RtcCalibration{.minus = 4,
                                                .window = RtcCalibrationWindow::seconds8}));
    (void)Rtc::calibrate(RtcCalibration{});
}

// =============================================================================
// j - coarse calibration and the interlock
// =============================================================================
/// Wait RECALPF out on the kernel tick - up to three ck_apre periods
/// (about twelve milliseconds at 32768 Hz) after a write to RTC_CALR.
bool wait_recalibration(uint32_t budget_ms = 100) {
    const uint32_t started = Ticker::ticks();
    while (Rtc::recalibration_pending() && Ticker::ticks() - started < budget_ms) {
    }
    return !Rtc::recalibration_pending();
}

void tj_coarse() {
    bench.verdict("coarse calibration is refused outside initialization mode",
                  !Rtc::coarse_calibrate(true, RtcCoarseCalibration{false, 4}));

    RtcReading r{};
    (void)Rtc::read(r);
    bench.verdict("a smooth setting goes in with the calendar running",
                  Rtc::calibrate(RtcCalibration{.minus = 8}));
    bench.verdict("and RECALPF clears within three ck_apre periods", wait_recalibration());

    bench.verdict("initialization mode opens", Rtc::enter_init());
    bench.verdict("with RTC_CALR non-zero the coarse calibrator is refused",
                  !Rtc::coarse_calibrate(true, RtcCoarseCalibration{false, 4}));
    // THE CALENDAR IS STOPPED IN HERE, and ck_apre with it, so the flag a
    // CALR write raises never falls: the smooth calibrator cannot be
    // written twice in initialization mode. Measured, not deduced.
    const bool wrote_in_init = Rtc::calibrate(RtcCalibration{});
    const bool cleared_in_init = wait_recalibration(200);
    const bool cleared_running = Rtc::exit_init() && wait_recalibration(200);
    bench.verdict("a smooth setting can be written with the calendar stopped", wrote_in_init);
    bench.verdict("and RECALPF falls once the calendar runs again", cleared_running);
    bench.verdict("initialization mode opens again", Rtc::enter_init());
    bench.verdict("with the smooth one at zero it is accepted",
                  Rtc::coarse_calibrate(true, RtcCoarseCalibration{true, 7}));
    bench.verdict("DCE is set and RTC_CALIBR holds the setting",
                  Rtc::coarse_calibration_enabled() && Rtc::coarse_calibration().negative &&
                      Rtc::coarse_calibration().steps == 7);
    bench.verdict("and now the SMOOTH one is refused", !Rtc::calibrate(RtcCalibration{}));
    print(serial, "  DC 7 negative is ", rtc_coarse_calibration_ppb(RtcCoarseCalibration{true, 7}),
          " ppb, DC 31 positive is ",
          rtc_coarse_calibration_ppb(RtcCoarseCalibration{false, 31}), " ppb", crlf);
    print(serial, "  RECALPF after an RTC_CALR write with the calendar STOPPED: ",
          cleared_in_init ? "clears" : "stands", "; with it running: ",
          cleared_running ? "clears" : "stands", crlf);

    // The PREDIV_A caution, staged by narrowing the asynchronous prescaler.
    bench.verdict("the coarse calibrator is turned off", Rtc::coarse_calibrate(false));
    (void)Rtc::set_prescalers(RtcPrescalers{3, 8191});
    bench.verdict("under PREDIV_A = 3 it is refused (17.3.10's caution)",
                  !Rtc::coarse_calibrate(true, RtcCoarseCalibration{false, 4}));
    (void)Rtc::set_prescalers(RtcPrescalers{});
    bench.verdict("initialization mode closes", Rtc::exit_init());
    bench.verdict("the smooth calibrator is writable again", Rtc::calibrate(RtcCalibration{}));
    (void)Rtc::init(RtcPrescalers{}, r.time);
}

// =============================================================================
// k - the sub-second register and a shift
// =============================================================================
/// Time `changes` transitions of the calendar's seconds field across a
/// shift issued right after the one that opens the window.
///
/// ONE FOR A DELAY AND TWO FOR AN ADVANCE, and the asymmetry is the
/// register's: SUBFS only pushes the next update back, while ADD1S bumps
/// the calendar THE INSTANT the shift lands and then pushes the update
/// back as well. So a delayed clock shows one long second and an advanced
/// one shows two seconds inside one and a half.
int32_t weigh_shift(bool add1s, uint16_t subfs, uint8_t changes) {
    uint64_t before = 0;
    if (!wait_calendar_second(before)) {
        return INT32_MIN;
    }
    if (!Rtc::shift(add1s, subfs)) {
        return INT32_MIN;
    }
    uint64_t after = 0;
    for (uint8_t i = 0; i < changes; ++i) {
        if (!wait_calendar_second(after)) {
            return INT32_MIN;
        }
    }
    // The shift takes up to a second of RTCCLK to land; SHPF is the
    // completion, and the second it lands in is the one just measured.
    const uint32_t started = Ticker::ticks();
    while (Rtc::shift_pending() && Ticker::ticks() - started < 2000u) {
    }
    return static_cast<int32_t>((after - before) / (SysClock::hz / 1000u));
}

void tk_shift() {
    const uint16_t sync = Rtc::prescalers().sync;
    const uint16_t s0 = Rtc::subsecond();
    wait_ms(20);
    const uint16_t s1 = Rtc::subsecond();
    print(serial, "  RTC_SSR read ", s0, " then ", s1, " twenty milliseconds later (PREDIV_S ",
          sync, ")", crlf);
    bench.verdict("the sub-second counter counts DOWN and stays inside PREDIV_S",
                  s0 <= sync && s1 <= sync && s1 != s0);
    bench.verdict("no shift is pending", !Rtc::shift_pending());

    const uint16_t half = static_cast<uint16_t>((sync + 1u) / 2u);
    const int32_t delayed = weigh_shift(false, half, 1);
    const int32_t advanced = weigh_shift(true, half, 2);
    print(serial, "  SUBFS alone: one calendar second took ", delayed,
          " ms. ADD1S with it: two took ", advanced, " ms", crlf);
    bench.verdict("SUBFS alone DELAYS the clock by half a second: the second it lands "
                  "in is 1500 ms long",
                  delayed > 1420 && delayed < 1580);
    bench.verdict("ADD1S with the same SUBFS ADVANCES it by half: two seconds inside 1500 ms",
                  advanced > 1420 && advanced < 1580);
    bench.verdict("a SUBFS past the fifteen-bit field is refused", !Rtc::shift(false, 0x8000u));
    bench.verdict("and the shift has completed", !Rtc::shift_pending());
}

// =============================================================================
// l - the twenty backup registers
// =============================================================================
void tl_backup() {
    print(serial, "  this part has ", Rtc::backup_count, " backup registers", crlf);
    bench.verdict("the header's own count is twenty", Rtc::backup_count == 20);

    bool all_back = true;
    for (uint8_t i = 0; i < Rtc::backup_count; ++i) {
        all_back = all_back && Rtc::backup(i, 0xA5000000u | i);
    }
    for (uint8_t i = 0; i < Rtc::backup_count; ++i) {
        all_back = all_back && Rtc::backup(i) == (0xA5000000u | i);
    }
    bench.verdict("every one of them holds a word of its own", all_back);
    bench.verdict("an index past the end is refused, both ways",
                  !Rtc::backup(Rtc::backup_count, 0u) && Rtc::backup(Rtc::backup_count) == 0u);

    // The key does not cover them; DBP does.
    Rtc::lock();
    const bool key_free = Rtc::backup(1, 0x11223344u) && Rtc::backup(1) == 0x11223344u;
    bench.verdict("the RTC's own key does NOT cover the backup registers", key_free);
    (void)RtcDomain::unlock(false);
    (void)Rtc::backup(1, 0x55667788u);
    const bool dbp_covers = Rtc::backup(1) == 0x11223344u;
    (void)RtcDomain::unlock(true);
    bench.verdict("but DBP does", dbp_covers);
    (void)Rtc::backup(1, 0xA5000001u);
    bench.verdict("and the word is what it was", Rtc::backup(1) == 0xA5000001u);
}

// =============================================================================
// m - the RTC_OUT pad
// =============================================================================
/// Count rising edges on RTC_AF1's own input register over `ms`, and give
/// back the cycles the window really lasted.
uint32_t count_pad_edges(uint32_t ms, uint32_t& cycles) {
    uint32_t edges = 0;
    bool last = RtcPad::read();
    const uint64_t t0 = cycles_now();
    const uint32_t started = Ticker::ticks();
    while (Ticker::ticks() - started < ms) {
        const bool now = RtcPad::read();
        if (now && !last) {
            ++edges;
        }
        last = now;
    }
    cycles = static_cast<uint32_t>(cycles_now() - t0);
    return edges;
}

void tm_output() {
    // The crystal, before the pad is asked to do anything (ES0287 2.2.10).
    uint32_t quiet_cycles = 0;
    const bool quiet = measure_seconds(2, quiet_cycles);
    const int32_t quiet_ppm = quiet ? ppm_of(quiet_cycles, 2ULL * SysClock::hz) : 0;

    RtcPad::input();
    bench.verdict("RTC_AF1 reads its pad through the GPIO's input register either way",
                  RtcPad::read() || !RtcPad::read());

    bench.verdict("the 512 Hz output is refused unless PREDIV_A is 127",
                  Rtc::prescalers().async != 127u ||
                      Rtc::calibration_output(true, RtcCalibOutput::hz512));
    uint32_t window = 0;
    const uint32_t fast = count_pad_edges(1000, window);
    const uint32_t fast_hz = static_cast<uint32_t>(
        (static_cast<uint64_t>(fast) * SysClock::hz) / (window == 0u ? 1u : window));
    print(serial, "  RTC_CALIB counted ", fast, " rising edges in ", window, " cycles = ", fast_hz,
          " Hz (512 due)", crlf);
    bench.verdict("and it is 512 Hz on the pad, to one per cent",
                  fast_hz > 507u && fast_hz < 517u);

    bench.verdict("the 1 Hz output takes PREDIV_S ending in 0xFF",
                  Rtc::calibration_output(true, RtcCalibOutput::hz1));
    const uint32_t slow = count_pad_edges(4200, window);
    print(serial, "  and at COSEL = 1, ", slow, " edges in 4.2 s (4 due)", crlf);
    bench.verdict("which is the calendar's own second on a pin", slow == 4u);

    bench.verdict("the output turns off", Rtc::calibration_output(false) &&
                                              !Rtc::calibration_output());

    // The alarm output: the pad follows the wake-up flag.
    Rtc::alarm_output(RtcOutput::wakeup, false, true);
    bench.verdict("OSEL reads back as the wake-up output",
                  Rtc::alarm_output() == RtcOutput::wakeup);
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 0, false);
    wait_ms(50);
    Rtc::clear_flags(RtcFlag::wakeup);
    wait_ms(20);
    const bool low_while_clear = !RtcPad::read();
    const bool flag_came = wait_flag(RtcFlag::wakeup, 1500);
    wait_ms(1);
    const bool high_while_set = flag_came && RtcPad::read();
    print(serial, "  RTC_ALARM on the pad: ", low_while_clear ? "low" : "high",
          " with WUTF clear, ", high_while_set ? "high" : "low", " with it set", crlf);
    bench.verdict("the alarm output follows the flag it was pointed at",
                  low_while_clear && high_while_set);
    Rtc::alarm_output(RtcOutput::off);
    Rtc::clear_wakeup();
    bench.verdict("and it goes back off", Rtc::alarm_output() == RtcOutput::off);

    // The crystal again, after all that toggling.
    uint32_t noisy_cycles = 0;
    const bool noisy = measure_seconds(2, noisy_cycles);
    const int32_t noisy_ppm = noisy ? ppm_of(noisy_cycles, 2ULL * SysClock::hz) : 0;
    print(serial, "  the crystal weighed ", quiet_ppm, " ppm before the pad moved and ", noisy_ppm,
          " ppm after (ES0287 2.2.10)", crlf);
    bench.verdict("the crystal still runs after RTC_AF1 has been driven",
                  quiet && noisy && noisy_ppm > -200 && noisy_ppm < 200);

    Led::output(true);   // the LED off on the black pill, where this pad is it
}

// =============================================================================
// n - the timestamp on RTC_AF1
// =============================================================================
/// Make a rising edge on RTC_AF1 with nothing attached to it. The pad is
/// driven low while the timestamp is OFF - the one state in which the
/// core certainly owns it - and then released into the RTC, so the edge
/// comes either from the core's own store or from the board's pull-up,
/// whichever owns the pad by then.
void make_pad_edge() {
    Rtc::timestamp_enable(false);
    RtcPad::output(false);
    wait_ms(5);
    Rtc::timestamp_enable(true, false);   // rising edge
    RtcPad::set();
}

void tn_timestamp() {
    Rtc::clear_flags(RtcFlag::timestamp | RtcFlag::timestamp_overflow);
    Rtc::timestamp_interrupt(false);
    Rtc::timestamp_enable(false);
    bench.verdict("the timestamp is off to start with", !Rtc::timestamp_enabled());

    // Who owns the pad, with the timestamp off and with it on: RM0383's
    // table 25 gives the RTC "TIMESTAMP input floating", and this is what
    // that costs a program that wanted to drive the pin.
    RtcPad::output(false);
    wait_ms(2);
    const bool core_owns_it_off = !RtcPad::read();
    RtcPad::set();
    wait_ms(2);
    const bool core_drives_high = RtcPad::read();
    Rtc::timestamp_enable(true, false);
    RtcPad::clear();
    wait_ms(20);
    const bool core_owns_it_on = !RtcPad::read();
    print(serial, "  with the timestamp off the core drives RTC_AF1 ",
          core_owns_it_off && core_drives_high ? "both ways" : "not at all",
          "; with it on the pad reads ", core_owns_it_on ? "what the core writes" : "HIGH ANYWAY",
          crlf);
    bench.verdict("with no RTC input enabled, RTC_AF1 is an ordinary GPIO",
                  core_owns_it_off && core_drives_high);
    bench.verdict("and with the timestamp enabled the RTC takes the pad (table 25)",
                  !core_owns_it_on);

    // The polled path: with no interrupt, TSF stands and the timestamp
    // registers stay frozen for the reader.
    Rtc::clear_flags(RtcFlag::timestamp | RtcFlag::timestamp_overflow);
    uint64_t made = 0;
    Rtc::timestamp_enable(false);
    RtcPad::output(false);
    wait_ms(5);
    made = cycles_now();
    Rtc::timestamp_enable(true, false);
    RtcPad::set();
    const bool took = wait_flag(RtcFlag::timestamp, 500);
    bench.verdict("an edge on RTC_AF1 raises TSF with no wire on the board", took);
    if (took) {
        const uint32_t latency = static_cast<uint32_t>(cycles_now() - made);
        const RtcReading ts = Rtc::timestamp();
        RtcReading now{};
        (void)Rtc::read(now);
        print(serial, "  the timestamp reads ", ts.time.hour, ":", ts.time.minute, ":",
              ts.time.second, " ss ", ts.subsecond, ", the calendar ", now.time.hour, ":",
              now.time.minute, ":", now.time.second, "; the flag came ",
              latency / (SysClock::hz / 1000u), " ms after the pad was released", crlf);
        bench.verdict("the frozen registers carry the calendar the event happened at",
                      ts.time.second == now.time.second ||
                          ts.time.second == static_cast<uint8_t>((now.time.second + 59u) % 60u));
        bench.verdict("and its sub-second is inside PREDIV_S",
                      ts.subsecond <= Rtc::prescalers().sync);
    } else {
        bench.verdict("the frozen registers carry the calendar the event happened at", false);
        bench.verdict("and its sub-second is inside PREDIV_S", false);
    }

    // A second event while TSF stands is an overflow.
    make_pad_edge();
    const bool overflowed = wait_flag(RtcFlag::timestamp_overflow, 500);
    bench.verdict("a second event while TSF stands raises TSOVF", overflowed);

    // And the interrupt path, where the flag is cleared for you: the
    // handler has to take the reading BEFORE the ISR body runs.
    Rtc::clear_flags(RtcFlag::timestamp | RtcFlag::timestamp_overflow);
    stamp_count = 0;
    Rtc::timestamp_interrupt(true);
    bench.verdict("TSIE is on and the shared EXTI line is open",
                  Rtc::wake_line_is_open(Rtc::tamper_stamp_exti_line));
    make_pad_edge();
    const uint32_t started = Ticker::ticks();
    while (stamp_count == 0u && Ticker::ticks() - started < 500u) {
    }
    const bool served = stamp_count != 0u;
    const RtcReading kept = stamp_reading;
    const RtcReading after = Rtc::timestamp();
    bench.verdict("the handler serves the timestamp", served &&
                                                          (stamp_served & RtcFlag::timestamp) != 0u);
    print(serial, "  the handler kept ", kept.time.hour, ":", kept.time.minute, ":",
          kept.time.second, "; the registers read ", after.time.hour, ":", after.time.minute,
          ":", after.time.second, " once the flag was cleared", crlf);
    bench.verdict("what it read before clearing the flag is a time", served && kept.time.month >= 1u);
    bench.verdict("and the registers are EMPTY after the clear: TSF is what freezes them",
                  after.time.hour == 0u && after.time.minute == 0u && after.time.second == 0u &&
                      after.subsecond == 0u);

    Rtc::timestamp_interrupt(false);
    Rtc::timestamp_enable(false);
    Rtc::clear_flags(RtcFlag::timestamp | RtcFlag::timestamp_overflow);
    bench.verdict("the timestamp goes off and its flags with it",
                  !Rtc::timestamp_enabled() && !Rtc::flag(RtcFlag::timestamp) &&
                      !Rtc::flag(RtcFlag::timestamp_overflow));
    Led::output(true);
}

// =============================================================================
// v - the backup registers across a reset (outside z)
// =============================================================================
constexpr uint32_t token_magic = 0x5F4B4231u;

void tv_survival() {
    const uint32_t magic = Rtc::backup(0);
    if (magic != token_magic) {
        for (uint8_t i = 0; i < Rtc::backup_count; ++i) {
            (void)Rtc::backup(i, 0x0BADC0DEu + i);
        }
        (void)Rtc::backup(0, token_magic);
        print(serial, "  leg 1: the token is written. Re-flash this same image and run v "
                      "again - the flash ends in a system reset, which the backup domain "
                      "does not see.", crlf);
        bench.verdict("the token went in", Rtc::backup(0) == token_magic);
        return;
    }
    bool intact = true;
    for (uint8_t i = 1; i < Rtc::backup_count; ++i) {
        intact = intact && Rtc::backup(i) == 0x0BADC0DEu + i;
    }
    print(serial, "  leg 2: the token survived, and so did ", Rtc::backup_count - 1u,
          " words behind it", crlf);
    bench.verdict("the backup registers came through a system reset intact", intact);
    bench.verdict("and the calendar with them", Rtc::calendar_set());
    (void)Rtc::backup(0, 0u);
    print(serial, "  the token is cleared: the next v starts leg 1 again", crlf);
}

// =============================================================================
// w - the erase: a real tamper (outside z)
// =============================================================================
void tw_tamper() {
    print(serial, "  this letter SPENDS the twenty backup registers", crlf);
    for (uint8_t i = 0; i < Rtc::backup_count; ++i) {
        (void)Rtc::backup(i, 0xC0FFEE00u + i);
    }
    bench.verdict("the registers carry a token to lose", Rtc::backup(3) == 0xC0FFEE03u);

    RtcPad::input();
    print(serial, "  RTC_AF1 floats ", RtcPad::read() ? "high" : "low", " on this board", crlf);

    Rtc::clear_flags(RtcFlag::tamper);
    (void)Rtc::tamper_disarm(1);
    bench.verdict("the block's mode is writable while nothing is armed",
                  Rtc::tamper_config(TamperConfig{TamperFilter::samples2, TamperSampling::div256,
                                                  TamperPrecharge::cycles8, true}));
    Rtc::timestamp_on_tamper(true);
    Rtc::tamper_interrupt(true);
    stamp_count = 0;

    // The filtered detector precharges the pad through the internal
    // pull-up and samples it at TAMPFREQ, so "staying high" fires after
    // two samples with nothing attached at all.
    const uint64_t armed_at = cycles_now();
    bench.verdict("tamper 1 arms", Rtc::tamper_arm(TamperInput{
                                       1, TamperTrigger::high_level_or_falling_edge}));
    bench.verdict("and the mode is refused while it is armed",
                  !Rtc::tamper_config(TamperConfig{}));
    const bool fired = wait_flag(RtcFlag::tamper1, 2000) || stamp_count != 0u;
    if (fired) {
        const uint32_t latency = static_cast<uint32_t>(stamp_at - armed_at);
        print(serial, "  the tamper fired ", latency / (SysClock::hz / 1000u),
              " ms after arming (two samples at 128 Hz is 16 ms)", crlf);
    }
    bench.verdict("a filtered high level fires the tamper with nothing on the pad", fired);
    bench.verdict("and the twenty backup registers are gone",
                  Rtc::backup(0) == 0u && Rtc::backup(19) == 0u);
    bench.verdict("TAMPTS filled the timestamp registers", Rtc::flag(RtcFlag::timestamp));
    const RtcReading ts = Rtc::timestamp();
    print(serial, "  the tamper's timestamp reads ", ts.time.hour, ":", ts.time.minute, ":",
          ts.time.second, crlf);

    Rtc::tamper_interrupt(false);
    Rtc::timestamp_on_tamper(false);
    bench.verdict("the input disarms", Rtc::tamper_disarm(1));
    Rtc::clear_flags(RtcFlag::all);
    bench.verdict("and every flag is down", (Rtc::status() & RtcFlag::all) == 0u);
    Led::output(true);
}

// =============================================================================
// x - the 1 Hz left on the pad, for a human (outside z)
// =============================================================================
void tx_show() {
    RtcPad::input();
    const bool on = Rtc::calibration_output(true, RtcCalibOutput::hz1);
    print(serial, "  RTC_CALIB is on at 1 Hz on RTC_AF1 - the LED on a black pill. "
                  "Any other letter takes the pad back.", crlf);
    bench.verdict("the 1 Hz output is on", on && Rtc::calibration_output());
}

void banner() {
    print(serial, crlf,
          "test_stm32f4_rtc - the RTC and the backup domain, weighed on SysTick", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void RTC_Alarm_IRQHandler() {
    const uint32_t served = brio::Rtc::alarm_isr();
    if (served != 0u) {
        alarm_at = cycles_now();
        alarm_served = served;
        alarm_count = alarm_count + 1u;
    }
}
extern "C" void RTC_WKUP_IRQHandler() {
    const uint32_t served = brio::Rtc::wakeup_isr();
    if (served != 0u) {
        wakeup_at = cycles_now();
        wakeup_served = served;
        wakeup_count = wakeup_count + 1u;
    }
}
extern "C" void TAMP_STAMP_IRQHandler() {
    // THE TIMESTAMP REGISTERS ARE FROZEN ONLY WHILE TSF STANDS (17.6.13:
    // "it is cleared when TSF bit is reset"), and the ISR body's first
    // act is to clear the flags it serves - so a handler that means to
    // keep the reading takes it FIRST. Measured: read afterwards, it is
    // all zeros.
    const brio::RtcReading ts = brio::Rtc::timestamp();
    const uint32_t served = brio::Rtc::tamper_stamp_isr();
    if (served != 0u) {
        stamp_at = cycles_now();
        stamp_served = served;
        stamp_reading = ts;
        stamp_count = stamp_count + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(true);
    brio::enable_interrupts();

    const bool domain_ok = bring_up(brio::RtcClockSource::lse);
    arm_vectors();

    bench.letter('a', "the domain and its two locks", ta_domain);
    bench.letter('b', "the LSE crystal: the start-up and the frequency", tb_lse);
    bench.letter('c', "the LSI weighed (and two domain resets)", tc_lsi);
    bench.letter('d', "the calendar, BCD both ways and four boundaries", td_calendar);
    bench.letter('e', "the shadow registers against BYPSHAD", te_shadow);
    bench.letter('f', "both alarms, the latency and the sub-second match", tf_alarms);
    bench.letter('g', "the wake-up timer on three clocks", tg_wakeup);
    bench.letter('h', "ES0287 2.8.4 staged: consecutive initializations", th_init_erratum);
    bench.letter('i', "smooth calibration, measured", ti_calibration);
    bench.letter('j', "coarse calibration and the interlock", tj_coarse);
    bench.letter('k', "the sub-second register and a shift", tk_shift);
    bench.letter('l', "the twenty backup registers", tl_backup);
    bench.letter('m', "the RTC_OUT pad: the outputs counted", tm_output);
    bench.letter('n', "the timestamp on RTC_AF1", tn_timestamp);
    bench.letter('v', "the backup registers across a reset", tv_survival, false);
    bench.letter('w', "THE ERASE: a real tamper", tw_tamper, false);
    bench.letter('x', "the 1 Hz left on the pad", tx_show, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED",
                    " rtc=", domain_ok ? "LSE" : "FAILED", brio::crlf);
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
        bench.prompt();
    }
}
