// test_v203_sleep - the reference bench suite for the CH32V203's POWER
// chapter and the two sleep sites over it: ch32v203/pwr.hpp over RM
// ch. 2 (the three modes, the regulator, the supply monitor, the two
// flags, the wake-up pad), ch32v203/sleep.hpp over util/power.hpp's
// site contract, and ch32v203/bus_activity.hpp - the count of bus
// masters other than the core, which on this family decides whether a
// sleep is legal at all.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT RULER MEASURES A SLEEP. The core's counter stops with HCLK in a
// Stop, so the only clock left running is the RTC's: the crystal over
// thirty-two, a tick of 976 us with the prescaler's own down-counter
// under it, which makes the finest span this silicon can measure while
// its core is stopped ONE RTCCLK PERIOD - thirty microseconds. Every
// span printed below in those units is therefore true to within one of
// them, and where a finer answer was wanted the same sleep is taken
// sixteen times and the difference divided.
//
// NO CURRENT IS MEASURED HERE, and this desk has no meter: what the
// low-power regulator and the RAM's low-voltage mode are worth in
// microamps is the one question this suite cannot ask. What it can ask
// is time, flags and counters.
//
// ONE WIRE, AND THE SUITE SAYS SO WHEN IT IS NOT THERE. Letter h - and
// the third leg of letter w - want a rising edge on PA0, the EXTI line
// that is also the WKUP pad, from an instrument outside this board;
// with nothing driving it they report that no edge came and take the
// RTC's backstop instead. THE PAD IS ALSO USART2's CTS, so an
// instrument that holds it at a level is an instrument the serial
// suite's flow-control letter will fail against: the two want the wire
// in opposite states and only one of them can have it at a time.
//
// What is exercised, letter by letter:
//   a  WHAT THE BOOT FOUND: the debug module's three low-power bits
//      (a probe holding the clocks up makes every timing below a
//      fiction), the two flags and their write-one clears, the mode
//      pair written and read back, the Stop's two prices and their
//      interlock, the retention bits and the regulator trims
//   b  THE SUPPLY MONITOR: the eight thresholds walked from the bottom
//      up with PVDO read at each - which brackets this board's rail
//      between two of them - and EXTI line 16 delivering the crossing
//   c  SLEEP, AND THE COUNT OF BUS MASTERS: a light sleep ended by the
//      tick, an idle() that RETURNS AT ONCE with a DMA channel enabled
//      (and the transfer running on across it), the same with the USB
//      controller attached, and the count back at zero after each
//   d  STOP: on the main regulator and on the low-power one, woken by
//      the RTC's alarm through EXTI line 17 - the span by the RTC's own
//      counter against the counts asked, the wake's cost as an upper
//      bound, the synchronization a wake owes, the tree coming back on
//      the HSI and the time the task takes to put it back, and the tick
//      the deep branch of the idle path drops
//   e  THE TIMED SITE UNDER A REAL KERNEL: a time event five hundred
//      milliseconds out, the loop idling into a Stop, the alarm placed
//      by the site and its counter read as the witness - the event
//      served on the WALL and never early
//   f  THE VOTE ROUND: one not-ok ends it, a standing lock clamps the
//      depth, the deadline guard refuses a deep round before anyone is
//      asked, and an arm over a running DMA channel is refused by the
//      silicon's own fact
//   h  A STOP ENDED FROM OUTSIDE: the pad's EXTI line against the RTC
//      backstop, and the wake-up pad's own flag
//   w  (by name, ENTERS STANDBY - the board comes back through a reset)
//      the RTC alarm as the Standby exit, SBF and the reset chapter's
//      low-power flag at the next boot, the SRAM the retention bit
//      kept; then the same with the line armed as an EVENT, which
//      table 2-1's note says returns instead
//   g  (by name, RESETS THE BOARD) the independent watchdog across a
//      Stop: armed short, a Stop asked for long, and the RTC - which
//      counts through both - saying which of the two arrived first
//
// build: boards = v203c6,v203c8
// build: groups = abcdh,efgw
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/dma.hpp"
#include "ch32v203/exti.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/pwr.hpp"
#include "ch32v203/reset.hpp"
#include "ch32v203/rtc.hpp"
#include "ch32v203/sleep.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"
#include "ch32v203/usb.hpp"
#include "ch32v203/watchdog.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v203Platform<>;

/// The board's high-speed crystal, and a rate the USB divider takes
/// whole - the controller is one of the two bus masters this chapter
/// counts, and letter c attaches it.
inline constexpr uint32_t xtal_hz = 8'000'000;
using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000, xtal_hz>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token the two by-name letters live in: .noinit, which the crt
// neither loads nor zeroes - and which a Standby keeps only as far as
// the retention bit says, one of the things letter w measures.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5B71;

struct Token {
    uint16_t magic;
    char letter;
    uint8_t leg;
    uint16_t pass;
    uint16_t fail;
    uint32_t rtc_at_arm;    ///< the RTC count before a sleep that may end in a reset
    uint32_t rtc_at_wake;   ///< and at a wake that did not
    uint32_t pattern;       ///< what the SRAM held before a Standby
    uint8_t returned;       ///< the deep mode came back without a reset
    uint8_t turns;          ///< how many instructions it took to stay stopped
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;
using KeyPad = Pin<'A', 0>;      ///< the WKUP pad, and EXTI line 0
using KeyInt = ExtInt<KeyPad>;

TestBench<Serial> bench;

using Plain = Ch32v203SleepSite<SysClock>;
using Timed = Ch32v203TimedSleepSite<P, SysClock>;
using Usb = Usbd<>;
using Copier = DmaChannel<1>;

/// The RTC's tick, and the prescaler under it: the site's own, because
/// the site is what programmed the block.
constexpr uint32_t tr_hz = Timed::tr_hz;
constexpr uint32_t tr_div = Timed::prescaler + 1u;
constexpr uint32_t rtcclk_hz = tr_hz * tr_div;

/// What the boot found, read before this program opened anything.
uint32_t boot_flags = 0;
uint32_t boot_ctlr = 0;
uint32_t boot_csr = 0;
uint32_t boot_sctlr = 0;
uint32_t boot_dbg = 0;
bool boot_pwr_gate = false;
bool site_ready = false;

/// Which body the alarm's vector runs: the raw recorder of letter d or
/// the timed site's own four acts.
volatile bool timed_round = false;
volatile uint32_t alarm_hits = 0;
volatile uint8_t alarm_sws = 0;
volatile uint32_t exti0_hits = 0;
volatile uint32_t pvd_hits = 0;

// ---------------------------------------------------------------------------
// Rulers
// ---------------------------------------------------------------------------

/// The core's counter as a stopwatch: STK counts to its reload and
/// starts again, so a span is accumulated poll by poll with one period
/// folded in across each wrap. It STOPS with HCLK in a Stop, which is
/// the whole reason the RTC is the other ruler here.
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

    uint32_t us(uint32_t hz) { return cycles() / (hz / 1'000'000u); }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

/// RTCCLK periods as microseconds, and as milliseconds: the units every
/// span this suite takes of a stopped core is measured in.
constexpr uint32_t fine_us(uint32_t fine) {
    return static_cast<uint32_t>((static_cast<uint64_t>(fine) * 1'000'000u) / rtcclk_hz);
}
constexpr uint32_t fine_ms(uint32_t fine) { return fine / (rtcclk_hz / 1000u); }

/// Let the console finish before the clocks stop: a Stop would freeze
/// the transmitter mid-character, and the wake comes back with a
/// divisor meant for twelve times the HSI.
void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 400);
}

/// Wait, awake, for a predicate - bounded, so a wedge ends a letter and
/// not the session. Microseconds waited, or zero if it never came.
template <class F>
uint32_t wait_for(F ready, uint32_t bound_us, uint32_t hz = SysClock::hz) {
    Stopwatch w;
    for (;;) {
        if (ready()) {
            return w.us(hz) + 1u;
        }
        if (w.us(hz) > bound_us) {
            return 0;
        }
    }
}

/// Place the alarm `counts` ahead of now, with the two flags and the
/// pending bit cleared first. The count it was placed at, or zero if
/// the block refused the write.
uint32_t place_alarm(uint32_t counts) {
    Rtc::clear(rtc_alrf);
    (void)Exti::clear(Rtc::wake_line);
    Pfic::clear_pending(Irq::rtc_alarm);
    if (!Rtc::synchronize()) {
        return 0;
    }
    const uint32_t at = Rtc::count() + counts;
    return Rtc::alarm(at) ? at : 0u;
}

// ===========================================================================
// a - what the boot found, the mode pair, the two flags
// ===========================================================================
void ta_found() {
    // THE FIRST THING, because everything timed below depends on it: a
    // debugger that keeps FCLK and HCLK alive in a low-power mode makes
    // a sleep a fiction, and this board is measured with a probe
    // attached.
    print(serial, "  DBGMCU_CR=", hex(boot_dbg), " -> sleep=", Pwr::debug_in_sleep() ? 1 : 0,
          " stop=", Pwr::debug_in_stop() ? 1 : 0, " standby=", Pwr::debug_in_standby() ? 1 : 0,
          crlf);
    bench.verdict("the debug module is NOT holding the clocks up - a probe attached to this "
                  "part leaves DBGMCU_CR's three low-power bits alone, so what the letters "
                  "below time is the silicon's own sleep and not the debugger's",
                  !Pwr::debug_holds_clocks());

    print(serial, "  at this boot: PWR gate ", boot_pwr_gate ? "OPEN" : "shut", ", CTLR=",
          hex(boot_ctlr), " CSR=", hex(boot_csr), " SCTLR=", hex(boot_sctlr), ", SBF=",
          (boot_csr & pwr_sbf) != 0u ? 1 : 0, " WUF=", (boot_csr & pwr_wuf) != 0u ? 1 : 0,
          crlf);
    bench.verdict("the PWR block's bus clock is SHUT out of reset, which is why every verb "
                  "of this driver opens it before it reads - a read that skipped the gate "
                  "would report a state the silicon never held",
                  !boot_pwr_gate);
    bench.verdict("and the mode pair out of reset is Sleep: SLEEPDEEP clear in the core's "
                  "own register and PDDS clear in this block's, which is what makes the "
                  "kernel's plain idle a Sleep and nothing deeper",
                  (boot_sctlr & sctlr_sleepdeep) == 0u && (boot_ctlr & pwr_pdds) == 0u);

    // The pair, written and read back. arm() takes PDDS down for a
    // shallow rung too, which is what makes "nothing armed here reaches
    // Standby" a fact of the registers.
    const bool to_stop = Pwr::arm(PwrMode::stop) && Pwr::mode() == PwrMode::stop;
    const bool to_standby = Pwr::arm(PwrMode::standby) && Pwr::mode() == PwrMode::standby;
    const bool pdds_up = (Pwr::ctlr() & pwr_pdds) != 0u;
    const bool to_sleep = Pwr::arm(PwrMode::sleep) && Pwr::mode() == PwrMode::sleep;
    const bool pdds_down = (Pwr::ctlr() & pwr_pdds) == 0u;
    bench.verdict("the three modes are ONE PAIR of bits and the driver writes and reads "
                  "them together - Stop, Standby and back to Sleep, with PDDS raised by the "
                  "middle one and taken down again by the last",
                  to_stop && to_standby && pdds_up && to_sleep && pdds_down);
    bench.verdict("and a Sleep armed by this driver cannot be a Standby by accident: the "
                  "shallow rung clears PDDS as well as SLEEPDEEP (RM 2.3.2's entry "
                  "conditions, both of them)",
                  pdds_down && (pfic_sctlr() & sctlr_sleepdeep) == 0u);

    // The Stop's two prices, and the interlock on the second.
    const bool lp = Pwr::stop_config(StopConfig{StopRegulator::low_power, false}) &&
                    Pwr::stop_config().regulator == StopRegulator::low_power;
    const bool ramlv_refused = !Pwr::stop_config(StopConfig{StopRegulator::main, true});
    const bool untouched = Pwr::stop_config().regulator == StopRegulator::low_power &&
                           !Pwr::stop_config().ram_low_voltage;
    const bool back = Pwr::stop_config(StopConfig{}) &&
                      Pwr::stop_config().regulator == StopRegulator::main;
    bench.verdict("LPDS is set and read back, and RAMLV WITHOUT it is refused with nothing "
                  "written - the chapter's own note on the bit, enforced here rather than "
                  "trusted to the caller",
                  lp && ramlv_refused && untouched && back);

    // What a Standby keeps, per class. On the D6 the first pair governs
    // the whole array and the second does not exist.
    Pwr::retain_ram(true);
    const bool kept = Pwr::retain_ram();
    Pwr::retain_ram_on_vbat(true);
    const bool kept_vbat = Pwr::retain_ram_on_vbat();
    Pwr::retain_upper_ram(true);
    const bool upper = Pwr::retain_upper_ram();
    Pwr::retain_ram_on_vbat(false);
    print(serial, "  retention: the first bank covers ", Pwr::ram_retention_bytes,
          " B of this part's ", device::sram_bytes, " B; the second bank ",
          Pwr::has_upper_ram_retention ? "exists" : "is the CH32V203RB's alone", crlf);
    bench.verdict("the retention bits take a write and read back, and the upper bank's two "
                  "answer false on this device class instead of writing a reserved bit - "
                  "2.4.1 keys them by class and the driver folds the part table",
                  kept && kept_vbat && (upper == Pwr::has_upper_ram_retention) &&
                      !Pwr::retain_ram_on_vbat());

    print(serial, "  the regulator's trims (EXTEN, not this block): LDOTRIM=",
          Pwr::core_voltage(), " ULLDOTRIM=", Pwr::low_power_voltage(), crlf);
    bench.verdict("the core regulator's trim reads its documented reset code of 2 (1.1 V), "
                  "which is the one fact about it this chapter states",
                  Pwr::core_voltage() == 2u);

    // WUF, AND WHAT RAISES IT. 2.4.2 gives the flag two sources - the
    // wake-up pad and the RTC alarm - and the alarm is the one that
    // needs no wire. Whether an alarm that arrives while the core is
    // AWAKE raises it is the question; the pad's half is letter h's and
    // a Standby exit's is letter w's.
    if (site_ready) {
        Pwr::clear_flags();
        const bool clear_before = !Pwr::wakeup_flag();
        // The alarm's own vector is already armed by the site's init, and
        // its body clears ALRF - so what says the alarm arrived is the
        // handler's own count and not the flag it acknowledged.
        timed_round = false;
        alarm_hits = 0;
        const uint32_t at = place_alarm(3);
        const uint32_t took = wait_for([] { return alarm_hits != 0u; }, 20'000u);
        const bool raised = Pwr::wakeup_flag();
        (void)at;
        Rtc::clear(rtc_alrf);
        (void)Exti::clear(Rtc::wake_line);
        Pfic::clear_pending(Irq::rtc_alarm);
        print(serial, "  an alarm three counts out arrived after ", took,
              " us with the core AWAKE: WUF ", raised ? "SET" : "clear", crlf);
        bench.verdict("AN ALARM THAT ARRIVES WITH THE CORE AWAKE DOES NOT RAISE WUF: 2.4.2 "
                      "gives the flag two sources, and the alarm's half of it is a WAKE and "
                      "not an alarm - what a program reads WUF for is how it came back, and "
                      "the two letters that stop the core are where it is read",
                      clear_before && took != 0u && !raised);
        bench.verdict("and the two clear bits read back as zero always, so a "
                      "read-modify-write of PWR_CTLR acknowledges nothing by accident",
                      (Pwr::ctlr() & (pwr_cwuf | pwr_csbf)) == 0u);

        // The window the alarm's own write costs, against the count it
        // is placed in: what makes a floor of four counts a margin.
        Stopwatch aw;
        const bool wrote = Rtc::alarm(Rtc::count() + 1000u);
        const uint32_t alarm_us = aw.us(SysClock::hz);
        print(serial, "  Rtc::alarm() takes ", alarm_us, " us to land, against a count of ",
              1'000'000u / tr_hz, " us", crlf);
        bench.verdict("the alarm's write window is a fraction of ONE count of the ruler it "
                      "is placed on, which is what makes the site's floor of four counts a "
                      "margin and not a guess",
                      wrote && alarm_us * 4u < 1'000'000u / tr_hz);
        Rtc::clear(rtc_alrf);
    }
}

// ===========================================================================
// b - the supply monitor
// ===========================================================================
void tb_pvd() {
    print(serial, "  FEATURE_SIGN=", hex(feature_sign()), " -> this die is specified from ",
          Pwr::supply_level() == PwrSupplyLevel::from_2v4 ? "2.4 V" : "1.8 V",
          ", so level0 is ", Pwr::pvd_rising_mv(PvdLevel::level0), " mV rising and level7 ",
          Pwr::pvd_rising_mv(PvdLevel::level7), " mV", crlf);
    bench.verdict("the PVD's millivolts are a fact of the DIE and not of the part number, "
                  "and 33.2.3's inversion test says whether the word may be believed - the "
                  "driver reads it and answers one of the two tables",
                  pvd_rising_mv(PvdLevel::level7, PwrSupplyLevel::from_2v4) >
                      pvd_rising_mv(PvdLevel::level0, PwrSupplyLevel::from_2v4));

    // EXTI line 16 armed as an interrupt on both edges: the crossing of
    // the threshold is what the line delivers, and the sweep below is
    // what crosses it.
    pvd_hits = 0;
    (void)Exti::clear(Pwr::pvd_exti_line);
    Pfic::clear_pending(Irq::pvd);
    Pfic::enable(Irq::pvd);
    (void)Pwr::arm_pvd(true);

    bool monotonic = true;
    bool low_seen = false;
    uint16_t last_not_low = 0;
    uint16_t first_low = 0;
    for (uint8_t code = 0; code < pvd_level_count; ++code) {
        const PvdLevel level = static_cast<PvdLevel>(code);
        (void)Pwr::pvd(true, level);
        (void)delay_us(clock, 200);
        if (Pwr::supply_low()) {
            if (!low_seen) {
                first_low = Pwr::pvd_rising_mv(level);
            }
            low_seen = true;
        } else {
            if (low_seen) {
                monotonic = false;
            }
            last_not_low = Pwr::pvd_rising_mv(level);
        }
    }
    const uint32_t edges = pvd_hits;
    (void)Pwr::pvd(false);
    (void)Pwr::arm_pvd(false);
    Pfic::disable(Irq::pvd);

    print(serial, "  the ladder: not low up to ", last_not_low, " mV");
    if (low_seen) {
        print(serial, ", low from ", first_low, " mV - the rail is between them", crlf);
    } else {
        print(serial, ", never low: the whole ladder is under this board's rail", crlf);
    }
    print(serial, "  the sweep raised ", edges, " crossing(s) on EXTI line 16", crlf);
    bench.verdict("PVDO is a LEVEL and not a latch, and walked from the bottom up it never "
                  "reads low here - which places this board's rail ABOVE the highest "
                  "threshold of the die's own table, and leaves the crossing itself to a "
                  "supply this desk cannot vary",
                  monotonic && !low_seen && last_not_low == Pwr::pvd_rising_mv(PvdLevel::level7));

    // The line the detector's output is wired to, proved where the
    // analogue crossing cannot be: 9.5.1's software trigger reaches the
    // same flag and the same vector, and an armed line is what it needs
    // (a line enabled nowhere raises nothing here - exti.hpp's fact 5).
    pvd_hits = 0;
    (void)Pwr::pvd(true, pvd_level_lowest);
    (void)Pwr::arm_pvd(true);
    Pfic::enable(Irq::pvd);
    const bool triggered = Exti::trigger(Pwr::pvd_exti_line);
    const uint32_t forced = wait_for([] { return pvd_hits != 0u; }, 10'000u);
    (void)Pwr::pvd(false);
    (void)Pwr::arm_pvd(false);
    Pfic::disable(Irq::pvd);
    print(serial, "  the software trigger on line 16 reached the PVD's own vector in ",
          forced, " us", crlf);
    bench.verdict("and the detector's output is wired to EXTI line 16, which reaches a "
                  "vector of its own: the line armed and raised by the software trigger "
                  "delivered, and the driver's handler body cleared the flag - the path a "
                  "real supply excursion would take",
                  triggered && forced != 0u && !Exti::pending(Pwr::pvd_exti_line));
    bench.verdict("with the detector disarmed the line is quiet again and PVDE reads clear",
                  !Pwr::pvd());
}

// ===========================================================================
// c - Sleep, and the count of bus masters
// ===========================================================================

/// The two cells the memory-to-memory block moves between. Neither end
/// increments, so 65535 items are milliseconds of work and two bytes of
/// RAM - which is what a 10 KB part can afford.
volatile uint8_t source_cell = 0;
volatile uint8_t sink_cell = 0;

DmaTransfer long_copy() {
    return DmaTransfer{
        .peripheral = &source_cell,
        .memory = &sink_cell,
        .count = 65535u,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = true,
                   .peripheral_increment = false,
                   .memory_increment = false,
                   .peripheral_width = DmaWidth::byte,
                   .memory_width = DmaWidth::byte,
                   .priority = DmaPriority::low},
    };
}

/// Eight idle() calls in a row, the cycles they cost and the ticks they
/// covered: the shape of the question "did the loop sleep?".
struct IdleCost {
    uint32_t cycles;
    uint32_t ticks;
};

IdleCost eight_idles() {
    const uint32_t t0 = Ticker::ticks();
    Stopwatch w;
    for (uint8_t i = 0; i < 8u; ++i) {
        P::CriticalSection cs;
        P::idle();
    }
    const uint32_t cycles = w.cycles();
    return IdleCost{cycles, Ticker::ticks() - t0};
}

void tc_sleep() {
    bench.verdict("with nothing but the core on the bus the count of active masters is "
                  "zero - which is the only value at which a sleep of any depth is legal "
                  "on this family",
                  P::bus_masters_active() == 0u);

    const bool light = Plain::arm(SleepDepth::light);
    bench.verdict("the ladder's shallow rung is the plain Sleep, and armed() answers NONE "
                  "for it: between Sleep and Stop this family has nothing, so light maps "
                  "to what the kernel's idle does anyway and the site says so",
                  light && Plain::armed() == SleepDepth::none && Pwr::mode() == PwrMode::sleep);

    console_drain();
    const uint32_t t1 = Ticker::ticks();
    uint8_t calls = 0;
    Stopwatch w;
    while (Ticker::ticks() == t1 && calls < 20u) {
        {
            P::CriticalSection cs;
            P::idle();
        }
        ++calls;
    }
    const uint32_t slept_us = w.us(SysClock::hz);
    Plain::disarm();
    print(serial, "  armed light: ", calls, " idle() call(s) and ", slept_us,
          " us to the next tick", crlf);
    bench.verdict("a light sleep is ended by the kernel's own tick, inside one tick period, "
                  "and the tick went on counting through it - the core clock is gated and "
                  "nothing else is",
                  calls >= 1u && calls < 20u && Ticker::ticks() > t1);

    // A DMA channel is a bus master, and in a sleep of any depth here it
    // would get no cycle at all. So the channel counts itself, and the
    // idle path reads the count.
    Dma::open();
    Copier::stop();
    source_cell = 0xA5;
    sink_cell = 0;
    const bool quiet_before = P::bus_masters_active() == 0u;
    const bool loaded = Copier::load(long_copy());
    const uint8_t with_dma = P::bus_masters_active();
    const uint16_t left0 = Copier::remaining();
    const IdleCost dma_idle = eight_idles();
    const uint16_t left1 = Copier::remaining();
    const uint32_t done_us = wait_for([] { return (Copier::flags() & DmaFlag::complete) != 0u; },
                                      200'000u);
    const uint8_t moved = sink_cell;
    const uint16_t left2 = Copier::remaining();
    const uint8_t still_counted = P::bus_masters_active();
    Copier::stop();
    const uint8_t after_stop = P::bus_masters_active();

    print(serial, "  a memory-to-memory block of 65535 items: the count reads ", with_dma,
          ", eight idle() calls cost ", dma_idle.cycles, " HCLK cycles over ",
          dma_idle.ticks, " tick(s), and the channel went from ", left0, " to ", left1,
          " items left", crlf);
    print(serial, "  the block finished in ", done_us, " us with ", left2,
          " left and the sink holding ", hex(moved), "; EN still up counts ", still_counted,
          ", stop() brings it to ", after_stop, crlf);
    bench.verdict("A CHANNEL COUNTS ITSELF: with EN up the platform reports one active bus "
                  "master, and it is the transition that is counted, in the one verb every "
                  "engine and every task goes through",
                  quiet_before && loaded && with_dma == 1u);
    bench.verdict("and idle() RETURNS AT ONCE over it - eight calls cost a handful of "
                  "cycles each instead of the millisecond a tick-ended sleep costs, the "
                  "loop spins, and the transfer ran on across them",
                  dma_idle.cycles < 4000u && dma_idle.ticks <= 1u && left1 < left0);
    bench.verdict("the block completed with its data moved - which is what a sleep taken "
                  "over it would have broken and not merely slowed",
                  done_us != 0u && left2 == 0u && moved == 0xA5u);
    bench.verdict("A CHANNEL ITS OWNER HAS NOT TAKEN DOWN STILL COUNTS, because EN stays "
                  "set when a block completes here - the count says what the registers "
                  "say - and stop() is what releases it",
                  still_counted == 1u && after_stop == 0u);

    // The other master on this part: the USB device controller, from the
    // pull-up to the detach. The controller is not brought up here - the
    // count is held by the pull-up itself, which is what a host sees.
    const bool usb_quiet = P::bus_masters_active() == 0u;
    Usb::connect(true);
    const uint8_t with_usb = P::bus_masters_active();
    const IdleCost usb_idle = eight_idles();
    Usb::connect(false);
    const uint8_t after_detach = P::bus_masters_active();
    print(serial, "  the USB pull-up: the count reads ", with_usb,
          ", eight idle() calls cost ", usb_idle.cycles, " cycles, the detach leaves ",
          after_detach, crlf);
    bench.verdict("AN ATTACHED CONTROLLER HOLDS ONE COUNT from its pull-up to its detach, "
                  "and while it does the idle path does not sleep - which is what turns "
                  "the old rule 'a program with USB never idles' into a mechanism the "
                  "program does not have to keep",
                  usb_quiet && with_usb == 1u && usb_idle.cycles < 4000u &&
                      usb_idle.ticks <= 1u && after_detach == 0u);
}

// ===========================================================================
// d - Stop, on both regulators
// ===========================================================================

/// One Stop, measured on the only ruler that survives it.
struct StopRun {
    bool ok = false;
    uint32_t counts = 0;      ///< asked for
    uint32_t span = 0;        ///< arm to the synchronized read at the wake, in RTCCLK periods
    uint32_t overshoot = 0;   ///< past the alarm's own boundary, same units
    uint32_t sync_us = 0;     ///< what the first synchronize() after the wake cost
    uint8_t sws = 0xFF;       ///< SYSCLK as the wake found it
    uint32_t ticks = 0;       ///< what the kernel's own timebase counted across it
    uint8_t turns = 0;        ///< idle() calls it took, the stale latched event included
};

/// The count and the phase of the tick in progress, read together: two
/// registers of one clock domain, so the phase is read on both sides and
/// a reload between them is a re-read.
struct RtcNow {
    uint32_t count;
    uint32_t phase;
};

RtcNow rtc_now() {
    for (uint8_t i = 0; i < 4u; ++i) {
        const uint32_t d1 = Rtc::divider();
        const uint32_t c = Rtc::count();
        const uint32_t d2 = Rtc::divider();
        if (d2 <= d1) {
            return RtcNow{c, tr_div - 1u - d2};
        }
    }
    return RtcNow{Rtc::count(), 0};
}

/// Arm `depth`, stop, and measure. The clock is NOT put back: a burst
/// leaves the tree on the HSI so that every repeat pays the same, and
/// the caller restores when it wants to print.
StopRun one_stop(SleepDepth depth, uint32_t counts) {
    StopRun r{};
    r.counts = counts;
    Rtc::clear(rtc_alrf);
    (void)Exti::clear(Rtc::wake_line);
    Pfic::clear_pending(Irq::rtc_alarm);
    if (!Rtc::synchronize()) {
        return r;
    }
    const RtcNow at_arm = rtc_now();
    const uint32_t at = at_arm.count + counts;
    if (!Rtc::alarm(at)) {
        return r;
    }
    const uint32_t t0 = Ticker::ticks();
    // Both counters, because what ends the wait is either of them and a
    // letter that ran before this one must not decide it.
    alarm_hits = 0;
    exti0_hits = 0;
    alarm_sws = 0xFF;
    if (!Plain::arm(depth)) {
        return r;
    }
    // THE FIRST WFE MAY NOT SLEEP AT ALL, and that is the idiom and not
    // a fault: this core's idle path latches events, so an interrupt
    // that turned pending between the decision to sleep and the
    // instruction ends the wait at once - the console's own byte is
    // usually the one. The loop is what a kernel does anyway: it idles
    // again until the thing it is waiting for has happened.
    while (alarm_hits == 0u && exti0_hits == 0u && r.turns < 40u) {
        {
            P::CriticalSection cs;
            P::idle();   // the Stop; the handler runs as this returns
        }
        ++r.turns;
    }
    // Back, on the HSI, with the STK counting its cycles again.
    Stopwatch w;
    const bool synced = Rtc::synchronize();
    r.sync_us = w.us(brio::hsi_hz);
    const RtcNow at_wake = rtc_now();
    r.ticks = Ticker::ticks() - t0;
    r.sws = alarm_sws;
    r.span = (at_wake.count - at_arm.count) * tr_div + at_wake.phase - at_arm.phase;
    r.overshoot = (at_wake.count - at) * tr_div + at_wake.phase;
    r.ok = synced && (alarm_hits != 0u || exti0_hits != 0u);
    return r;
}

/// The same, sixteen times, adding the overshoots: a wake that costs
/// less than one period of the ruler is still visible in the sum.
uint32_t stop_burst(SleepDepth depth, uint8_t times, uint32_t counts, bool& ok) {
    uint32_t total = 0;
    ok = true;
    for (uint8_t i = 0; i < times; ++i) {
        const StopRun r = one_stop(depth, counts);
        ok = ok && r.ok;
        total += r.overshoot;
    }
    return total;
}

void td_stop() {
    if (!site_ready) {
        bench.verdict("the timed site's RTC is up (the crystal, a 1024 Hz tick)", false);
        return;
    }
    // The alarm's own vector is what ends a Stop, and in this letter it
    // records rather than restores.
    timed_round = false;
    (void)Rtc::arm_wake(true);
    Pfic::enable(Irq::rtc_alarm);

    // FIRST, THE SAME ALARM TAKEN AWAKE. Everything below is measured
    // from the count the alarm names, and the block's own lag to that
    // count is not the wake's: one alarm served with the core running
    // is what separates them, on the same ruler.
    alarm_hits = 0;
    (void)Rtc::synchronize();
    const RtcNow before = rtc_now();
    const uint32_t awake_at = before.count + 4u;
    (void)Rtc::alarm(awake_at);
    (void)wait_for([] { return alarm_hits != 0u; }, 20'000u);
    const RtcNow after = rtc_now();
    const uint32_t awake_lag = (after.count - awake_at) * tr_div + after.phase;
    print(serial, "  an alarm served AWAKE arrives ", fine_us(awake_lag),
          " us past the count it names - about one count of the ruler, which is this "
          "block's own lag and not a sleep's", crlf);
    bench.verdict("the alarm event is raised one count of the ruler AFTER the value "
                  "written, not on it - measured awake, and subtracted from every span "
                  "below so that what is left is the sleep's own",
                  awake_lag >= tr_div / 2u && awake_lag < tr_div * 2u);

    console_drain();
    const StopRun main_run = one_stop(SleepDepth::standby, 205);
    const uint8_t sws_at_wake = main_run.sws;
    Stopwatch rw;
    Plain::disarm();   // the clock task re-run: this is the restore
    const uint32_t restore_cycles = rw.cycles();
    const bool back = Rcc::sysclk_status() == SysClock::sysclk_source;

    const uint32_t wake_cost =
        main_run.overshoot > awake_lag ? main_run.overshoot - awake_lag : 0u;
    print(serial, "  a Stop on the MAIN regulator asked for ", main_run.counts,
          " counts (", fine_ms(main_run.counts * tr_div), " ms): it slept ",
          fine_us(main_run.span), " us in ", main_run.turns,
          " idle() turn(s), overshooting the alarm by ", fine_us(main_run.overshoot),
          " us - of which ", fine_us(wake_cost),
          " us is the wake itself; and the first read owed ", main_run.sync_us,
          " us of RTC synchronization", crlf);
    print(serial, "  at the wake SYSCLK was ", sws_at_wake == 0u ? "the HSI" : "NOT the HSI",
          " and the kernel's tick had advanced by ", main_run.ticks,
          "; the tree came back in ", restore_cycles / (brio::hsi_hz / 1'000'000u),
          " us of HSI time", crlf);
    bench.verdict("A STOP IS ENDED BY AN EXTI LINE and by nothing else this program armed: "
                  "the RTC's alarm on line 17 - the path that needs no clock - brought the "
                  "core back, and the span the RTC counted is the one that was asked for",
                  main_run.ok && main_run.span >= main_run.counts * tr_div);
    bench.verdict("and it lands LATE and never early: past the block's own lag the wake "
                  "itself costs a few periods of the ruler, which is the finest this "
                  "silicon can say with its core stopped",
                  main_run.overshoot >= awake_lag && wake_cost < tr_div);
    bench.verdict("WHAT COMES BACK IS NOT WHAT WENT IN: the core resumes on the HSI with "
                  "the PLL off, and the site's disarm() is what re-runs the clock task and "
                  "puts the program's own rate back",
                  sws_at_wake == static_cast<uint8_t>(SysclkSource::hsi) && back);
    bench.verdict("the kernel's timebase stood still for the whole of it - the STK counts "
                  "HCLK, which a Stop stops - so the plain site's honest restriction is "
                  "that a program with armed deadlines must not take one",
                  main_run.ticks <= 1u);
    bench.verdict("and the counter's first read after the wake is synchronized before it "
                  "is believed: the resync costs at most one RTCCLK period, which is what "
                  "6.2.3 asks for and what the site pays unconditionally",
                  main_run.sync_us != 0u && main_run.sync_us <= 100u);

    // THE TWO DEEP RUNGS ARE ONE MODE WITH TWO PRICES, and the only
    // thing this desk can measure of the price is the wake. Sixteen
    // short Stops of each, the tree left on the HSI throughout so that
    // both bursts pay exactly the same for everything but LPDS.
    console_drain();
    bool main_ok = false;
    bool lp_ok = false;
    const uint32_t burst_main = stop_burst(SleepDepth::standby, 16u, 8u, main_ok);
    const uint32_t burst_lp = stop_burst(SleepDepth::deep, 16u, 8u, lp_ok);
    Plain::disarm();
    const uint32_t per_main = fine_us(burst_main * 4u) / 64u;
    const uint32_t per_lp = fine_us(burst_lp * 4u) / 64u;
    const uint32_t extra = burst_lp > burst_main ? fine_us((burst_lp - burst_main) * 4u) / 64u
                                                 : 0u;
    print(serial, "  sixteen Stops of each: the main regulator overshot by ", burst_main,
          " RTCCLK periods in all (", per_main, " us a wake), the low-power one by ",
          burst_lp, " (", per_lp, " us a wake) - ", extra, " us more", crlf);
    bench.verdict("both Stop configurations wake: the ladder's two deep rungs are ONE MODE "
                  "WITH TWO PRICES - the same SLEEPDEEP with PDDS clear, LPDS the only "
                  "difference - and both come back through the same line",
                  main_ok && lp_ok);
    bench.verdict("and what the low-power regulator costs at the wake is the difference "
                  "printed above, measured over sixteen wakes because one is under the "
                  "resolution of the only ruler a stopped core has",
                  burst_lp + 16u >= burst_main);

    const bool clock_back = Rcc::sysclk_status() == SysClock::sysclk_source;
    bench.verdict("the tree is the program's own again after the burst, and the console is "
                  "legible, which is the same fact twice",
                  clock_back && Plain::armed() == SleepDepth::none);
}

// ===========================================================================
// e and f - a Stop under a real kernel, and the vote round
// ===========================================================================
struct Blip {};

/// The stakeholder: a voter, a deadline's owner, and the thing whose
/// first event after a wake ends the round.
struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline uint16_t blips = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline uint16_t asked = 0;
    static inline bool last_ok = false;
    static inline bool refuse = false;
    static inline uint32_t blip_count = 0;    ///< the RTC's count when the event matured
    static inline uint32_t blip_tick = 0;
    static inline SleepDepth last_report = SleepDepth::none;
    static inline SleepDepth last_asked = SleepDepth::none;

    static void clear() {
        blips = wakes = votes = asked = 0;
        last_ok = false;
        refuse = false;
        blip_count = 0;
        blip_tick = 0;
        last_report = SleepDepth::none;
        last_asked = SleepDepth::none;
    }

    static void init() { start(&only); }
    static Status only(const Event& e);
};

using Manager = PowerManager<P, Timed, PowerConfig{}, Probe>;
using Kernel = Tenuto<P, Probe, Manager>;

Probe::Status Probe::only(const Event& e) {
    return match(e,
        [](Entry) { return handled(); },
        [](Exit) { return handled(); },
        [](SleepVote v) {
            ++votes;
            last_ok = v.ok;
            return handled();
        },
        [](const PrepareSleep& p) {
            ++asked;
            last_asked = p.depth;
            p.reply.send(SleepVote{!refuse});
            return handled();
        },
        [](WakeReport w) {
            ++wakes;
            last_report = w.was;
            return handled();
        },
        [](Blip) {
            // THE MODEL'S CONVENTION, and here it is load-bearing: a wake
            // path with nothing to say says it with SleepRequested{none}.
            // Without it the manager never learns the machine came back.
            ++blips;
            blip_tick = Ticker::ticks();
            blip_count = Rtc::count();
            post<Manager>(SleepRequested{SleepDepth::none, reply_to<Probe, SleepVote>()});
            return handled();
        });
}

/// The loop a program would really have, run until the deadline matures
/// or the guard expires.
void pump_until_blip(uint32_t guard_ms) {
    const uint32_t start = Rtc::count();
    const uint32_t bound = (guard_ms * tr_hz) / 1000u;
    while (Probe::blips == 0u && (Rtc::count() - start) < bound) {
        TimeEvents<P>::process();
        if (!Kernel::step()) {
            Kernel::idle_if_empty();
        }
    }
    while (Kernel::step()) {
    }
}

void te_kernel() {
    if (!site_ready) {
        bench.verdict("the timed site's RTC is up (the crystal, a 1024 Hz tick)", false);
        return;
    }
    timed_round = true;
    Pfic::enable(Irq::rtc_alarm);
    Kernel::init_all();
    Probe::clear();

    print(serial, "  the ruler: ", tr_hz, " RTC ticks a second out of ", rtcclk_hz,
          " Hz, so a 500 ms deadline asks for ", Timed::counts_for(500u),
          " counts and a second of them is worth ", Timed::ticks_for(tr_hz),
          " kernel ticks", crlf);
    bench.verdict("the site's two conversions round the way the kernel's time contract "
                  "allows: the counts for a deadline rounded UP so a wake is late, the "
                  "ticks of a span rounded DOWN so a resync is short",
                  Timed::counts_for(500u) >= 512u && Timed::ticks_for(1024u) <= 1000u);

    console_drain();
    const uint32_t c0 = Rtc::count();
    const uint32_t k0 = Ticker::ticks();
    Probe::deadline.arm(500u);
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip(3000u);
    Probe::deadline.disarm();

    const uint32_t wall_ms = ((Probe::blip_count - c0) * 1000u) / tr_hz;
    const uint32_t kernel_ms = Probe::blip_tick - k0;
    const uint32_t advance = Timed::last_advance();
    const uint32_t placed = Timed::last_counts();
    print(serial, "  the round: ", Probe::votes, " vote(s), ", Probe::wakes,
          " wake report(s) at depth ", static_cast<uint32_t>(Probe::last_report),
          "; the alarm was placed ", placed, " counts out and the resync handed the "
          "ticker ", advance, " tick(s)", crlf);
    print(serial, "  a 500 ms event matured after ", wall_ms, " ms of WALL and ",
          kernel_ms, " ms of kernel tick", crlf);
    bench.verdict("the vote round ran under a real kernel, the site was armed at the depth "
                  "the target really took, and the loop's own idle path is what stopped "
                  "the machine",
                  Probe::votes >= 1u && Probe::asked >= 1u && Probe::blips == 1u);
    bench.verdict("THE DEADLINE WAS MET ON THE WALL: the RTC's alarm was placed where the "
                  "kernel's nearest deadline was and its counter was read as the witness, "
                  "so a Stop no longer costs the program the time it slept",
                  wall_ms >= 500u && wall_ms < 700u);
    bench.verdict("and it was met in KERNEL time too, late and never early - the resync "
                  "advanced the tick by the frozen span, less what the counter itself "
                  "served",
                  kernel_ms >= 500u && kernel_ms < 700u && advance != 0u);
    bench.verdict("the first event after the wake ended the round: nothing polled, the "
                  "manager disarmed on its own and the clock is the program's again",
                  Probe::wakes >= 1u && Timed::armed() == SleepDepth::none &&
                      Rcc::sysclk_status() == SysClock::sysclk_source);
    timed_round = false;
}

void tf_votes() {
    if (!site_ready) {
        bench.verdict("the timed site's RTC is up (the crystal, a 1024 Hz tick)", false);
        return;
    }
    timed_round = false;
    Kernel::init_all();
    Probe::clear();

    // One not-ok ends the round.
    Probe::refuse = true;
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    for (uint8_t i = 0; i < 8u && Kernel::step(); ++i) {
    }
    bench.verdict("ONE not-ok ends the round: the site is not armed and the requester is "
                  "told, which is the whole of the model's unanimity rule",
                  Probe::asked == 1u && Probe::votes == 1u && !Probe::last_ok &&
                      Timed::armed() == SleepDepth::none);

    // A standing lock clamps the depth the voters are asked for.
    Probe::clear();
    {
        PowerLock lock = Manager::restrict(SleepDepth::light);
        post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
        for (uint8_t i = 0; i < 8u && Kernel::step(); ++i) {
        }
        bench.verdict("a standing lock clamps a deep request to the rung it names, and the "
                      "voter is asked for the CLAMPED depth - a restriction taken from an "
                      "interrupt, where a vote could not be",
                      Probe::last_asked == SleepDepth::light &&
                          Timed::armed() == SleepDepth::none);
    }
    bench.verdict("and the ceiling is back the moment the lock dies",
                  Manager::ceiling() == SleepDepth::deep);

    // The deadline guard, before anyone is asked.
    Probe::clear();
    Kernel::init_all();
    Probe::deadline.arm(1u);
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    for (uint8_t i = 0; i < 8u && Kernel::step(); ++i) {
    }
    const bool guarded = Probe::asked == 0u && !Probe::last_ok;
    Probe::deadline.disarm();
    bench.verdict("a deadline nearer than the manager's floor refuses a deep round BEFORE "
                  "a voter is asked - the guard is the kernel's own ticks_to_next and not "
                  "a target's business",
                  guarded);

    // A round with every vote yes arms the Stop.
    Probe::clear();
    Kernel::init_all();
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    for (uint8_t i = 0; i < 8u && Kernel::step(); ++i) {
    }
    const SleepDepth armed_at = Manager::armed_depth();
    const bool sleepdeep = Pwr::mode() == PwrMode::stop;
    bench.verdict("with every vote yes the site arms, and what it armed is a Stop: "
                  "SLEEPDEEP set with PDDS clear, read back off the silicon",
                  Probe::last_ok && armed_at == SleepDepth::deep && sleepdeep);

    // And the refusal no vote can express: a bus master at work.
    Dma::open();
    source_cell = 0x5A;
    const bool loaded = Copier::load(long_copy());
    const bool refused_by_silicon = !Timed::arm(SleepDepth::deep);
    const bool none_takes = Timed::arm(SleepDepth::none);
    Probe::clear();
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    for (uint8_t i = 0; i < 8u && Kernel::step(); ++i) {
    }
    const bool round_lost = !Probe::last_ok && Timed::armed() == SleepDepth::none;
    (void)wait_for([] { return (Copier::flags() & DmaFlag::complete) != 0u; }, 200'000u);
    Copier::stop();
    bench.verdict("AND THE ONE REFUSAL THAT IS THE SILICON'S: with a channel enabled the "
                  "site says no to every rung, so the round is lost with no voter having "
                  "to know that a sleep here would starve the transfer",
                  loaded && refused_by_silicon && round_lost);
    bench.verdict("while `none` always takes, because disarming must never be refused - "
                  "the manager disarms on the first event after a wake, and a transfer "
                  "started meanwhile must not leave a deep mode standing",
                  none_takes);

    Timed::disarm();
    bench.verdict("the letter leaves the machine shallow again: nothing armed, the clock "
                  "the program's",
                  Timed::armed() == SleepDepth::none &&
                      Rcc::sysclk_status() == SysClock::sysclk_source);
}

// ===========================================================================
// h - a Stop ended from outside, and the wake-up pad
// ===========================================================================
void th_edge() {
    if (!site_ready) {
        bench.verdict("the timed site's RTC is up (the crystal, a 1024 Hz tick)", false);
        return;
    }
    timed_round = false;
    // A PULL-DOWN and not a floating input: the pad has a wire on it,
    // and a wire with nothing driving it collects edges of its own -
    // measured, one inside sixteen milliseconds. Held down, "no edge"
    // means what the letter says it means, and a push-pull instrument
    // still wins against forty kilohms.
    KeyPad::input(PinPull::down);
    const bool claimed = KeyInt::claim(PinPull::down);
    (void)KeyInt::configure(ExtiSense::rising);
    (void)KeyInt::clear();
    (void)KeyInt::arm(true);
    Pfic::clear_pending(Irq::exti0);
    Pfic::enable(Irq::exti0);
    exti0_hits = 0;
    (void)Rtc::arm_wake(true);
    Pfic::enable(Irq::rtc_alarm);

    print(serial, "  PA0 reads ", KeyPad::read() ? 1 : 0,
          " and its line is armed on the rising edge; a Stop with a three-second backstop "
          "starts now - an edge from outside is what this letter is waiting for", crlf);
    console_drain();
    const StopRun r = one_stop(SleepDepth::standby, 3u * tr_hz);
    Plain::disarm();
    const uint32_t edges = exti0_hits;
    const uint32_t alarms = alarm_hits;

    print(serial, "  the Stop lasted ", fine_ms(r.span), " ms and ended with ", edges,
          " pad edge(s) and ", alarms, " alarm(s)", crlf);
    bench.verdict("the line was claimed by this pad and the Stop ended - on the edge or on "
                  "the backstop, and the letter says which",
                  claimed && r.ok);
    if (edges != 0u) {
        bench.verdict("A PAD'S EXTI LINE ENDS A STOP, and long before the backstop: the "
                      "edge detection is asynchronous and needs no clock, which is what "
                      "makes a pin a wake source with the whole tree stopped",
                      fine_ms(r.span) < 2900u);
    } else {
        print(serial, "  no edge arrived: nothing outside this board is driving PA0, so "
              "the pad's half of this letter is SKIPPED and the backstop is what is "
              "measured", crlf);
        bench.verdict("with no instrument on the pad the RTC's backstop is what ended the "
                      "Stop, at the three seconds it was asked for",
                      alarms != 0u && fine_ms(r.span) >= 2900u);
    }

    // The WKUP pad: the same pad under EWUP. The line stays armed
    // beside it, so the letter can tell an edge that arrived from one
    // that never came - which is what makes the answer a measurement.
    Pwr::clear_flags();
    const bool armed_pad = Pwr::wakeup_pin(true);
    exti0_hits = 0;
    const uint32_t waited = wait_for([] { return Pwr::wakeup_flag(); }, 4'000'000u);
    const bool raised = Pwr::wakeup_flag();
    const uint32_t pad_edges = exti0_hits;
    (void)Pwr::wakeup_pin(false);
    Pwr::clear_flags();
    bench.verdict("EWUP claims PA0 as the wake-up pad and reads back - the pad is forced "
                  "to an input with a pull-down by the hardware, which is a state "
                  "ch32v203/pin.hpp never wrote",
                  armed_pad && !Pwr::wakeup_pin());
    print(serial, "  with EWUP set and the core AWAKE: ", pad_edges,
          " edge(s) on the pad in four seconds, WUF ", raised ? "SET" : "clear", crlf);
    if (pad_edges != 0u) {
        bench.verdict("AND THE EDGES DO NOT RAISE WUF WITH THE CORE RUNNING: the pad kept "
                      "reaching its EXTI line while EWUP stood, and the flag never moved - "
                      "on this silicon WUF belongs to the STANDBY EXIT alone, which is "
                      "where the letter that enters one reads it",
                      !raised && waited == 0u);
    } else {
        print(serial, "  no edge arrived on the pad either: the wake-up pad's half is "
              "SKIPPED, and what an instrument driving PA0 would add is in the document",
              crlf);
        bench.verdict("with nothing driving the pad WUF stays clear, which is all that can "
                      "be said without an edge",
                      !raised);
    }
    (void)KeyInt::arm(false);
    Pfic::disable(Irq::exti0);
    KeyInt::release();
    (void)Rtc::arm_wake(true);
}

// ===========================================================================
// w - Standby, by name: the board comes back through a reset
// ===========================================================================
inline constexpr uint32_t standby_pattern = 0xC0FFEE5Au;

void tw_enter(uint8_t leg) {
    token.magic = token_magic;
    token.letter = 'w';
    token.leg = leg;
    token.returned = 0;
    token.turns = 0;
    token.pattern = standby_pattern;
    token.rtc_at_wake = 0;
    Reset::clear_flags();
    Pwr::clear_flags();
    Pwr::retain_ram(true);
    // Three ways of asking one question. The alarm is a DOCUMENTED
    // Standby exit and the first leg takes it as an interrupt; the
    // second arms the same line as an EVENT ALONE, which is what table
    // 2-1's note is about; the third leaves the alarm as a backstop and
    // arms A PAD'S line as an event - an exit the chapter does not list
    // at all.
    (void)Rtc::arm_wake(leg != 2u);
    (void)Rtc::arm_wake_event(true);
    Pfic::clear_pending(Irq::rtc_alarm);
    Pfic::disable(Irq::rtc_alarm);
    if (leg == 3u) {
        KeyPad::input(PinPull::down);
        (void)KeyInt::claim(PinPull::down);
        (void)KeyInt::configure(ExtiSense::rising);
        (void)KeyInt::clear();
        (void)KeyInt::arm(true);     // the flag, so a return can be told from a backstop
        (void)KeyInt::event(true);   // and the event the note is about
        Pfic::clear_pending(Irq::exti0);
        Pfic::disable(Irq::exti0);
    }
    (void)place_alarm(2u * tr_hz);
    token.rtc_at_arm = Rtc::count();
    print(serial, "  leg ", leg, ": the alarm two seconds out as ",
          leg == 1u ? "an interrupt" : (leg == 2u ? "an EVENT alone"
                                                  : "the backstop, with PA0's line as an EVENT"),
          ", the first RAM bank's retention bit set, entering Standby...", crlf);
    console_drain();
    // THE TICK WOULD END IT BEFORE IT BEGAN, and here the platform's
    // idle path is not the one stopping the core: this is the
    // deliberate one-shot, so the caller is what pauses the timebase
    // and takes its pending bit down. A latched event from before does
    // the same, which is why the instruction is taken again until the
    // alarm this letter armed has really fired.
    Ticker::pause();
    Pfic::clear_pending(Irq::systick);
    uint8_t turns = 0;
    while (!Rtc::alarmed() && !(leg == 3u && KeyInt::pending()) && turns < 6u) {
        Plain::enter_standby();
        ++turns;
    }
    // Still here: the note's second exit, and the tree is the HSI's now.
    (void)Plain::resume_clock();
    Ticker::resume();
    token.returned = 1;
    token.turns = turns;
    token.rtc_at_wake = Rtc::count();
}

void tw_judge(uint8_t leg, bool returned, uint32_t flags, uint32_t csr) {
    const uint32_t at_wake = returned ? token.rtc_at_wake : Rtc::count();
    const uint32_t ms = ((at_wake - token.rtc_at_arm) * 1000u) / tr_hz;
    print(serial, "  leg ", leg, ": the machine ", returned ? "RETURNED from" : "came back "
          "through a RESET after", " Standby, ", ms, " ms after the alarm was placed", crlf);
    print(serial, "  PWR_CSR=", hex(csr), " (SBF=", (csr & pwr_sbf) != 0u ? 1 : 0, " WUF=",
          (csr & pwr_wuf) != 0u ? 1 : 0, "), reset flags=", hex(flags), " (LPWRRSTF=",
          (flags & ResetFlag::low_power) != 0u ? 1 : 0, " PORRSTF=",
          (flags & ResetFlag::power) != 0u ? 1 : 0, " PINRSTF=",
          (flags & ResetFlag::pin) != 0u ? 1 : 0, ")", crlf);
    if (!returned) {
        bench.verdict("STANDBY'S EXIT IS A RESET, which is why it is off the ladder: the "
                      "program did not resume after the instruction, it came back through "
                      "the reset vector - and a sleep site that armed one would leave a "
                      "manager waiting for a wake that arrives as a reboot",
                      true);
        bench.verdict("and SBF is how the boot path knows where it came from: PWR_CSR "
                      "survives a Standby wake where every other bit of the block's "
                      "registers does not",
                      (csr & pwr_sbf) != 0u);
        if (leg == 3u && ms < 1900u) {
            bench.verdict("A PAD'S LINE ARMED AS AN EVENT ENDS A STANDBY TOO - an exit "
                          "2.3.4 does not list: the machine came back on an edge, before "
                          "the documented backstop, and it came back the same way, through "
                          "the reset. WUF is what tells the two kinds apart, and it stayed "
                          "clear here where the alarm sets it",
                          (csr & pwr_wuf) == 0u);
        } else if (leg == 3u) {
            print(serial, "  no edge reached the pad while the machine was stopped: the "
                  "question this leg asks is SKIPPED, and the documented backstop is what "
                  "is measured", crlf);
            bench.verdict("the documented alarm ended the Standby at the two seconds it "
                          "was placed and raised WUF, with nothing outside the board "
                          "driving the pad",
                          ms <= 2600u && (csr & pwr_wuf) != 0u);
        } else {
            bench.verdict("the RTC's alarm is a documented Standby exit and it arrived at "
                          "the two seconds it was placed - the backup domain counts through "
                          "a mode in which nothing else does - AND IT RAISES WUF, which the "
                          "same alarm served awake does not: the flag is the wake's",
                          ms >= 1900u && ms <= 2600u && (csr & pwr_wuf) != 0u);
        }
        bench.verdict("the SRAM the retention bit covers came through it: the token this "
                      "letter left in .noinit is the pattern it wrote",
                      token.pattern == standby_pattern);
        // The two flags are write-one-to-clear, and CWUF's own note says
        // the first of them falls two system clocks after the store -
        // which is measurable here and nowhere else, this being the one
        // place the flag really stands.
        Stopwatch cw;
        Pwr::clear_flags();
        const uint32_t polls = wait_for([] { return !Pwr::wakeup_flag(); }, 1000u);
        const uint32_t cycles = cw.cycles();
        print(serial, "  the two clears put both flags down in ", cycles, " HCLK cycles",
              crlf);
        bench.verdict("and the two write-one clears put them down: the flags read clear "
                      "again within the handful of cycles 2.4.1's note on CWUF allows, "
                      "through a store that acknowledges nothing else",
                      polls != 0u && !Pwr::wakeup_flag() && !Pwr::standby_flag());
    } else {
        bench.verdict("TABLE 2-1'S NOTE IS TRUE HERE: a Standby with the wake armed as an "
                      "EVENT returns to the instruction after it instead of resetting the "
                      "part, which is why the driver's door to Standby is not [[noreturn]]",
                      true);
        bench.verdict("and it came back on the HSI like a Stop, with the clock task put "
                      "back by the same verb",
                      Rcc::sysclk_status() == SysClock::sysclk_source);
        bench.verdict("the alarm arrived at the two seconds it was placed",
                      ms >= 1900u && ms <= 2600u);
    }
}

/// The legs from `first` to the last. A leg that RESETS does not come
/// back here - the next boot resumes at the one after it - so what this
/// loop judges in place is only a leg that returned.
void tw_run_from(uint8_t first) {
    for (uint8_t leg = first; leg <= 3u; ++leg) {
        token.pass = bench.passed();
        token.fail = bench.failed();
        tw_enter(leg);
        tw_judge(leg, true, Reset::flags(), Pwr::csr());
    }
    bench.end_letter();
    token.letter = '\0';
}

void tw_standby() {
    if (!site_ready) {
        bench.verdict("the timed site's RTC is up (the crystal, a 1024 Hz tick)", false);
        return;
    }
    bench.reset_tally();
    tw_run_from(1);
}

void tw_resume() {
    const uint8_t leg = token.leg;
    tw_judge(leg, token.returned != 0u, boot_flags, boot_csr);
    if (leg < 3u) {
        tw_run_from(static_cast<uint8_t>(leg + 1u));
        return;
    }
    bench.end_letter();
    token.letter = '\0';
}

// ===========================================================================
// g - the independent watchdog across a Stop, by name
// ===========================================================================
void tg_iwdg() {
    if (!site_ready) {
        bench.verdict("the timed site's RTC is up (the crystal, a 1024 Hz tick)", false);
        return;
    }
    bench.reset_tally();
    token.magic = token_magic;
    token.letter = 'g';
    token.leg = 1;
    token.returned = 0;
    token.pass = 0;
    token.fail = 0;
    Reset::clear_flags();
    timed_round = false;
    (void)Rtc::arm_wake(true);
    Pfic::enable(Irq::rtc_alarm);
    (void)place_alarm(2u * tr_hz);
    token.rtc_at_arm = Rtc::count();
    token.rtc_at_wake = 0;
    print(serial, "  the independent watchdog armed for about 200 ms of its own LSI, and "
          "a Stop asked for two seconds: whichever arrives first is the answer", crlf);
    console_drain();
    alarm_hits = 0;
    (void)Iwdg::arm(IwdgConfig{.prescaler = IwdgPrescaler::div32, .reload = 249});
    Iwdg::refresh();
    (void)Plain::arm(SleepDepth::standby);
    // The same loop every Stop in this suite takes: a latched event
    // ends the first instruction, and what is being waited for is the
    // alarm's own handler.
    uint8_t turns = 0;
    while (alarm_hits == 0u && turns < 6u) {
        {
            P::CriticalSection cs;
            P::idle();
        }
        ++turns;
    }
    token.turns = turns;
    // The Stop ended. Nothing refreshes the watchdog from here, so the
    // reset is a moment away - awake, which is the other answer.
    token.returned = 1;
    token.rtc_at_wake = Rtc::count();
    const uint32_t start = Rtc::count();
    while ((Rtc::count() - start) < 2u * tr_hz) {
    }
    Reset::software();
}

void tg_resume() {
    const uint32_t to_wake = ((token.rtc_at_wake - token.rtc_at_arm) * 1000u) / tr_hz;
    const uint32_t to_reset = ((Rtc::count() - token.rtc_at_arm) * 1000u) / tr_hz;
    print(serial, crlf, "-> back after the watchdog letter: the Stop ",
          token.returned != 0u ? "RAN ITS FULL LENGTH" : "was cut short", crlf);
    print(serial, "  the alarm was placed at 2000 ms; the reset arrived ", to_reset,
          " ms after it");
    if (token.returned != 0u) {
        print(serial, ", the Stop having ended at ", to_wake, " ms", crlf);
    } else {
        print(serial, crlf);
    }
    print(serial, "  reset flags=", hex(boot_flags), " (IWDGRSTF=",
          (boot_flags & ResetFlag::independent_watchdog) != 0u ? 1 : 0, " SFTRSTF=",
          (boot_flags & ResetFlag::software) != 0u ? 1 : 0, ")", crlf);
    bench.verdict("the independent watchdog reset the board, and IWDGRSTF says so at the "
                  "next boot - which is the only observable this counter has, there being "
                  "no way to read it",
                  (boot_flags & ResetFlag::independent_watchdog) != 0u);
    if (token.returned == 0u) {
        bench.verdict("AND IT COUNTED THROUGH THE STOP: the reset arrived while the core "
                      "was stopped, before the alarm that would have ended the sleep - the "
                      "LSI runs in a Stop and this watchdog rides it, so a program that "
                      "sleeps longer than its time-out is reset in its sleep",
                      to_reset < 1900u);
    } else {
        bench.verdict("BUT NOT THROUGH THE STOP: the sleep ran its full two seconds and "
                      "the reset came only once the core was awake again, so this "
                      "watchdog does not count while the core is stopped",
                      to_wake >= 1900u);
    }
    bench.end_letter();
    token.letter = '\0';
}

void banner() {
    print(serial, crlf, "test_v203_sleep on ", device::part_name,
          " - PWR (RM ch. 2) and the two sleep sites", crlf,
          "  z costs the board nothing; w ENTERS STANDBY and g RESETS THE BOARD, by name",
          crlf, "  the RTC: ", site_ready ? "up on the crystal" : "NOT READY",
          ", the ruler ", rtcclk_hz, " Hz", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The alarm's own vector, behind EXTI line 17 - the path that survives
/// a Stop. In a timed round it runs the site's four acts; everywhere
/// else it records what the wake found and acknowledges.
extern "C" BRIO_CH32_INTERRUPT void rtc_alarm_handler() {
    if (timed_round) {
        Timed::isr();
        alarm_hits = alarm_hits + 1u;
        return;
    }
    alarm_sws = static_cast<uint8_t>(brio::Rcc::sysclk_status());
    (void)brio::Rtc::alarm_isr();
    alarm_hits = alarm_hits + 1u;
}

/// The pad an instrument outside this board drives.
extern "C" BRIO_CH32_INTERRUPT void exti0_handler() {
    (void)brio::Exti::clear(0);
    exti0_hits = exti0_hits + 1u;
}

/// The supply monitor's own vector, on EXTI line 16.
extern "C" BRIO_CH32_INTERRUPT void pvd_handler() {
    (void)brio::Pwr::pvd_isr();
    pvd_hits = pvd_hits + 1u;
}

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();

    // BEFORE anything here opens a gate: what the reset left, and what a
    // probe left in the debug module.
    boot_pwr_gate = brio::Pwr::bus_clock();
    boot_dbg = brio::Pwr::debug_cr();
    boot_sctlr = brio::pfic_sctlr();
    boot_ctlr = brio::Pwr::ctlr();
    boot_csr = brio::Pwr::csr();

    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    site_ready = Timed::init();

    bench.letter('a', "what the boot found: the debug bits, the mode pair, the two flags",
                 ta_found);
    bench.letter('b', "the supply monitor: the ladder and EXTI line 16", tb_pvd);
    bench.letter('c', "Sleep, and the count of bus masters (a channel, the USB pull-up)",
                 tc_sleep);
    bench.letter('d', "Stop on both regulators, woken by the RTC alarm", td_stop);
    bench.letter('e', "the timed site under a real kernel: a 500 ms deadline on the wall",
                 te_kernel);
    bench.letter('f', "the vote round, the lock, the deadline guard, the silicon's refusal",
                 tf_votes);
    bench.letter('h', "a Stop ended from outside, and the wake-up pad", th_edge);
    bench.letter('w', "STANDBY, three times (the board comes back through a reset)", tw_standby,
                 false);
    bench.letter('g', "THE INDEPENDENT WATCHDOG ACROSS A STOP (resets the board)", tg_iwdg,
                 false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL96 on the crystal" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", " site=", site_ready ? "RTC" : "FAILED",
                    brio::crlf);
        if (token.magic == token_magic && token.letter == 'w') {
            bench.reset_tally();
            bench.resume_tally(token.pass, token.fail);
            tw_resume();
        } else if (token.magic == token_magic && token.letter == 'g') {
            bench.reset_tally();
            bench.resume_tally(token.pass, token.fail);
            tg_resume();
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
