// test_vx03_clock - the reference bench suite for the CLOCK TREE of the
// CH32V203 and the CH32V303 (RM ch. 3 with the EXTEN bits that belong to
// it - the one simple tree the manual gives both device classes): the
// roots, the PLL and its two input dividers, the switch, the four
// prescalers, the clock output, the security system, the peripheral
// gates - and the dynamic regime over all of it, with the kernel tick
// and the console itself rebased at every step.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. Every clock this suite touches is inside the chip or
// on the board's own crystals (both boards carry an 8 MHz one on
// PD0/PD1 and a 32.768 kHz one on PC14/PC15; letter c selects the
// latter for the RTC when the backup domain stands on no other source,
// a selection that lasts until the domain is reset), the console is the
// probe's serial (USART1 on PA9/PA10), and the clock output's pad PA8
// is counted where a strap to PA6 already exists (the CH32V203C8T6's
// board: SPI1's loop-back node) and only read back elsewhere.
//
// THE CONSOLE IS THE INSTRUMENT AND THE SUBJECT AT ONCE. Every letter
// that moves the rate moves it through a DynamicClock whose users are
// the ticker and this very port, so a letter whose console dies is a
// letter that failed: the PASS line you are reading came out of a
// divisor computed for the rate it names.
//
// HOW A RATE IS MEASURED. Every counter of the core and the timers is
// derived from HCLK, so none of them can judge HCLK. Two instruments do.
// Letter b BRACKETS a span of kernel ticks between two console lines and
// leaves the judging to whoever timed those two lines: the tick is exact
// by construction (ticker.hpp), so the ratio between the span asked and
// the seconds that passed IS the rate's deviation; its verdict is only
// that the tree switched and the console survived, the number is in the
// log. Letter c counts each rung against the RTC on the board's 32.768
// kHz crystal, the one clock the pack does not derive from, and judges
// the core's cycles and the kernel's ticks over that window.
//
// What is exercised, letter by letter:
//   a  the tree as the registers hold it at boot: the root SYSCLK runs
//      on, the four prescaler codes, the USB divider, the PLL's source,
//      multiplier and input divider (the HSI's lives in EXTEN), and the
//      two rates that follow - with the flash's own rule stated, since
//      this family has no wait-state field at all
//   b  every static rate the family offers on this board, each one
//      reached, read back and bracketed for the host's clock to judge:
//      the bare HSI, the PLL on the HSI at four rates (one of them
//      through the HALVED HSI, which is the EXTEN bit), the bare
//      crystal and the PLL on the crystal at four more
//   c  the dynamic regime through its whole pack, up and down: every
//      switch taken, HCLK and the kernel tick counted against the RTC on
//      the 32 kHz crystal at every rung, delay_us timed on the core's
//      counter (never early, not past a sixteenth late), and the console
//      rebased at each step
//   d  the LSI: stopped, started, and the time to LSIRDY measured
//   e  the HSE: the crystal's start-up time to HSERDY measured with the
//      ready INTERRUPT armed over the same ramp, and the HSI stopped and
//      restarted while the tree runs off the crystal
//   f  the clock security system armed over a running crystal: CSSON
//      reads back, the flag stays clear and the non-maskable interrupt
//      never fires
//   g  the clock output on PA8: every source this class has, set and
//      read back, and the pad claimed and released; where PA8 is
//      strapped to PA6 (the CH32V203C8T6's board - probed, not assumed)
//      TIM3 counts what leaves it: the HSE and the HSI
//   h  the peripheral gates: every enable the chapter lists for a block
//      THIS PART has (the part table says which) opened and closed with
//      the register read back, and what the register does with the
//      gates of the blocks it has not got - reported, not judged
//   i  the ready interrupts of the four other roots: the LSI, the PLL
//      (off under the bare crystal, restarted by a switch) and the LSE
//      (behind the backup domain's door, its crystal's start-up timed)
//      each reaching the RCC vector with its own flag, the body leaving
//      none behind - and the HSI (stopped under the crystal's PLL),
//      whose restart mostly raises no flag on this silicon, against the
//      manual: what it raised is printed, not judged
//
// build: boards = v203c6,v203c8,v303cb,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/rtc.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;

// ---------------------------------------------------------------------------
// The rates. Each is a static Clock task - the tuple of root, PLL ratio,
// HPRE, both peripheral prescalers, the ADC divider and the USB one -
// and the pack is how one image reaches them all. The board's crystal is
// 8 MHz, which is also the HSI's rate, so the two roots reach the same
// four PLL rates and the pack names each of them twice: that is what
// set_index() is for.
//
// 52 MHz is not decoration: it is the one rate here that the PLL can
// only make from the HALVED input (4 MHz times thirteen), so it is what
// exercises EXTEN_CTR.HSIPRE in its other position.
// ---------------------------------------------------------------------------
inline constexpr uint32_t xtal_hz = 8'000'000;

using PllHsi144 = Clock<ClockSource::pll, 144'000'000>;
using PllHsi96 = Clock<ClockSource::pll, 96'000'000>;
using PllHsi48 = Clock<ClockSource::pll, 48'000'000>;
using PllHsi52 = Clock<ClockSource::pll, 52'000'000>;
using HsiBare = Clock<ClockSource::internal, 8'000'000>;
using PllXtal144 = Clock<ClockSource::pll, 144'000'000, xtal_hz>;
using PllXtal96 = Clock<ClockSource::pll, 96'000'000, xtal_hz>;
using PllXtal72 = Clock<ClockSource::pll, 72'000'000, xtal_hz>;
using PllXtal48 = Clock<ClockSource::pll, 48'000'000, xtal_hz>;
using XtalBare = Clock<ClockSource::crystal, 8'000'000, xtal_hz>;

using SysClock = DynamicClock<Rates<PllHsi144, PllHsi96, PllHsi52, PllHsi48, HsiBare,
                                    PllXtal144, PllXtal96, PllXtal72, PllXtal48, XtalBare>,
                              Ticker, Serial>;
constexpr SysClock clock;

/// The pack's own names, in its order: the tuple says the rate and the
/// root, but not which oscillator the PLL is fed from, and that is the
/// half this suite is about.
constexpr const char* rate_names[] = {
    "PLL on the HSI",       "PLL on the HSI",       "PLL on the HALVED HSI",
    "PLL on the HSI",       "the bare HSI",         "PLL on the crystal",
    "PLL on the crystal",   "PLL on the crystal",   "PLL on the crystal",
    "the bare crystal",
};
constexpr uint8_t boot_rate = 0;               ///< PllHsi144, what main() enters
constexpr uint8_t xtal_144_rate = 5;           ///< PllXtal144, letter e's crystal tree
constexpr uint8_t xtal_bare_rate = 9;          ///< XtalBare, the tree with the PLL off
static_assert(sizeof(rate_names) / sizeof(rate_names[0]) == SysClock::rate_count);
static_assert(SysClock::rate_count == 2u * xtal_144_rate);   // five HSI trees, five crystal ones

TestBench<Serial> bench;

/// The suite's ruler: the core's cycle count through the ticker. Its
/// rate is the boot rate's - a ruler's rate is a compile-time constant
/// (util/test_wait.hpp) and this clock is dynamic - so it converts
/// microseconds right only at 144 MHz: the boot rate and the crystal's
/// PLL at 144, the two trees the letters wait in. wait_us() below checks
/// the rate in force before it trusts the conversion; the console's
/// drain, which runs at every rate, waits on the tick instead, which the
/// ticker keeps a millisecond at every rate.
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return PllHsi144::hz; }
};
static_assert(PllXtal144::hz == Ruler::hz());

/// The non-maskable interrupt's tally (letter f): the clock security
/// system's own count, and anything else that took that vector.
volatile uint32_t css_events = 0;
volatile uint32_t nmi_others = 0;

/// The RCC vector's tally (letter e): how many times a root's ready flag
/// raised the line, and which flags the body found standing.
volatile uint32_t rcc_events = 0;
volatile uint32_t rcc_flags = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
/// Every queued byte off the wire (tx_idle() covers the shift register),
/// bounded on the kernel tick because this runs at every rate of the
/// pack: 200 ms, a 2 KB ring at 115200 baud and more.
void console_drain() {
    (void)brio::wait_ticks<P>([] { return Serial::tx_idle(); },
                              200u);   // a wedged console shows as missing lines
}

const char* source_name(SysclkSource s) {
    switch (s) {
        case SysclkSource::hsi: return "HSI";
        case SysclkSource::hse: return "HSE";
        case SysclkSource::pll: return "PLL";
    }
    return "?";
}

const char* mco_name(McoSource s) {
    switch (s) {
        case McoSource::none:     return "none";
        case McoSource::sysclk:   return "SYSCLK";
        case McoSource::hsi:      return "HSI";
        case McoSource::hse:      return "HSE";
        case McoSource::pll_div2: return "PLL/2";
    }
    return "?";
}

/// Poll until `ready` answers or `limit_us` passes, and say how long it
/// took in microseconds (`limit_us` on a time-out; the caller judges the
/// condition itself afterwards). On the ruler at 144 MHz; at any other
/// rate the ruler's conversion is wrong, so the wait goes on the kernel
/// tick, in whole milliseconds.
template <typename Fn>
uint32_t wait_us(Fn ready, uint32_t limit_us) {
    if (SysClock::hz() != Ruler::hz()) {
        const uint32_t t0 = Ticker::ticks();
        (void)brio::wait_ticks<P>(ready, limit_us / 1000u + 1u);   // judged by the caller
        return (Ticker::ticks() - t0) * 1000u;
    }
    const auto took = brio::cycles_until<Ruler>(ready, limit_us);
    return took ? *took / (Ruler::hz() / 1'000'000u) : limit_us;
}

/// Move to one rate of the pack, with the console drained first so no
/// byte is in the shift register when the divisor changes.
bool go_to(uint8_t i) {
    console_drain();
    return SysClock::set_index(i);
}

/// The two lines whoever timed them turns into a rate. Between them the
/// program does nothing but count its own ticks.
void bracket(uint8_t i, uint32_t span) {
    print(serial, "  rate ", i, " (", rate_names[i], ", ", SysClock::rate_hz(i),
          " Hz): bracket ", span, " ticks", crlf);
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < span) {
    }
    print(serial, "  rate ", i, " done", crlf);
    // The closing line alone on the wire for a few ticks, as the opening
    // one is: a console bridge that forwards in blocks and flushes when
    // the line goes idle would otherwise hold it until the verdict after
    // it had gone out too, and whoever times the two lines would time
    // that verdict's length as well.
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < 5u) {
    }
}

/// What a rate must be able to say about itself after a switch.
bool rate_in_force(uint8_t i) {
    return SysClock::rate_index() == i && SysClock::hz() == SysClock::rate_hz(i) &&
           Rcc::sysclk_status() == SysClock::rate_source(i);
}

// ---------------------------------------------------------------------------
// a - the tree as the registers hold it
// ---------------------------------------------------------------------------
void ta_tree() {
    print(serial, "  SYSCLK on ", source_name(Rcc::sysclk_status()), ", HCLK ",
          SysClock::hz(), " Hz, PCLK1 ", SysClock::pclk1_hz(), " Hz, PCLK2 ",
          SysClock::pclk2_hz(), " Hz", crlf);
    print(serial, "  HPRE=", Rcc::hpre_code(), " (/", hpre_divider(Rcc::hpre_code()),
          ") PPRE1=", Rcc::ppre1_code(), " (/", ppre_divider(Rcc::ppre1_code()),
          ") PPRE2=", Rcc::ppre2_code(), " (/", ppre_divider(Rcc::ppre2_code()),
          ") ADCPRE=", Rcc::adc_prescaler(), " (/",
          adcpre_divider(Rcc::adc_prescaler()), ")", crlf);
    print(serial, "  USBPRE=", Rcc::usb_prescaler(), " (/",
          usbpre_divider(Rcc::usb_prescaler()), ") -> ", SysClock::usb_hz(),
          " Hz; ADC -> ", SysClock::adc_hz(), " Hz, in spec: ",
          SysClock::adc_in_spec(), crlf);
    print(serial, "  PLL from the ", Rcc::pll_from_hse() ? "HSE" : "HSI",
          ", input divided: ", Rcc::pll_input_divided(), " (the HSI's divider is "
          "EXTEN's HSIPRE), x", Rcc::pll_multiplier(), crlf);
    print(serial, "  HSI trim=", Rcc::hsi_trim(), " factory calibration=",
          hex(Rcc::hsi_calibration()), crlf);
    print(serial, "  no flash wait-state field exists on this family; what the "
                  "flash chapter asks for at this rate is HCLK halved around an "
                  "erase or a program: ", PllHsi144::flash_needs_halving, crlf);

    bench.verdict("the boot rate is in force, and the registers say so",
                  rate_in_force(boot_rate));
    bench.verdict("SWS reports the PLL", Rcc::sysclk_status() == SysclkSource::pll);
    bench.verdict("HPRE leaves SYSCLK undivided at the family ceiling",
                  Rcc::hpre_code() == PllHsi144::hpre_code && hpre_divider(Rcc::hpre_code()) == 1u);
    bench.verdict("PB1 is halved above its cap and PB2 is not",
                  ppre_divider(Rcc::ppre1_code()) == 2u && ppre_divider(Rcc::ppre2_code()) == 1u);
    bench.verdict("PCLK1 is half of HCLK and PCLK2 is all of it",
                  SysClock::pclk1_hz() == SysClock::hz() / 2u &&
                      SysClock::pclk2_hz() == SysClock::hz());
    bench.verdict("the USB divider was programmed before any USB gate could "
                  "open, and gives 48 MHz",
                  usbpre_divider(Rcc::usb_prescaler()) == 3u &&
                      SysClock::usb_hz() == 48'000'000UL);
    bench.verdict("the ADC prescaler is the slowest the field has, and even "
                  "that is above the converter's rating at this rate",
                  Rcc::adc_prescaler() == 3u && !SysClock::adc_in_spec());
    bench.verdict("the PLL is fed by the HSI, whole, times eighteen",
                  !Rcc::pll_from_hse() && !Rcc::pll_input_divided() &&
                      Rcc::pll_multiplier() == 18u);
    bench.verdict("the HSI's trim is the reset value, the centre of the field",
                  Rcc::hsi_trim() == 16u);
    // Two waits of 900 us on STK's own count, which runs whether or not
    // the tick's handler does - not on the ruler, which is built on the
    // tick being judged and would stall with it: 1.8 ms holds at least
    // one tick of 1 ms.
    const uint32_t t0 = Ticker::ticks();
    const bool waited = delay_us(clock, 900) && delay_us(clock, 900);
    bench.verdict("the kernel tick is running on this tree",
                  waited && Ticker::ticks() != t0);
}

// ---------------------------------------------------------------------------
// b - every static rate, bracketed for the host's clock
// ---------------------------------------------------------------------------
void tb_rates() {
    constexpr uint32_t span = 2000;   // ticks, i.e. 2 s if the tick is a ms
    print(serial, "  each rate is entered, read back and bracketed; the number "
                  "is the host's to compute from the two lines", crlf);

    for (uint8_t i = 0; i < SysClock::rate_count; ++i) {
        const bool ok = go_to(i);
        const bool in_force = rate_in_force(i);
        bracket(i, span);
        bench.verdict("the tree took this rate and this line came out of it: ",
                      rate_names[i], ok && in_force);
    }

    (void)go_to(boot_rate);
    bench.verdict("and back to the boot rate", rate_in_force(boot_rate));
}

// ---------------------------------------------------------------------------
// c - the pack up and down
// ---------------------------------------------------------------------------

/// THE INSTRUMENT: the RTC on the board's 32.768 kHz crystal, the one
/// clock in the chip that no rate of the pack derives from. Its count
/// runs at 1024 Hz (a divider of 32), and a rung is timed over 128 of
/// its counts - 125 ms exactly on the crystal.
constexpr uint32_t rtc_tr_hz = 1024u;
constexpr uint32_t rtc_window = 128u;   ///< counts, 125 ms
constexpr uint32_t rtc_window_us = rtc_window * 1'000'000u / rtc_tr_hz;
static_assert(rtc_window_us == 125'000u);

/// Bring the RTC up on the LSE, the backup domain left on its source:
/// RTCSEL is one-way, and a domain that stands on another source is not
/// wiped here (that would cost the backup registers) - the rung's rate
/// is then not judged, and the letter says so.
bool rtc_ruler_up() {
    if (!RtcDomain::unlock(true)) {
        return false;
    }
    RtcDomain::lse_enable(true);
    // A 32 kHz crystal starts in hundreds of milliseconds to seconds
    // (letter i times it): five seconds on the tick.
    if (!brio::wait_ticks<P>([] { return RtcDomain::lse_ready(); }, 5000u)) {
        return false;
    }
    if (!RtcDomain::open(RtcClockSource::lse)) {
        return false;
    }
    return Rtc::synchronize() &&
           Rtc::configure(RtcConfig{.prescaler = 32'768u / rtc_tr_hz - 1u});
}

/// One rung's reading against the RTC: HCLK as the core's counter
/// measures it, and the kernel ticks that passed, over the same window.
struct RungReading {
    uint32_t hclk = 0;    ///< Hz
    uint32_t ticks = 0;   ///< in rtc_window_us
    bool ok = false;      ///< both edges seen within their bounds
};

RungReading rung_by_rtc() {
    RungReading r;
    if (!Rtc::synchronize()) {
        return r;
    }
    // THE START IS AN EDGE OF THE COUNT, found as the first read above
    // every read before it - twice, because the first sample may itself
    // be one of the stale copies count() can return (up to three counts
    // low, ch32vx03/rtc.hpp), and then its first "edge" is no edge. A
    // stale read is never ABOVE the count, so the running maximum only
    // moves at a real edge once one fresh read has been seen. An edge
    // comes every 0.98 ms: five ticks bound each.
    uint32_t hi = Rtc::count();
    for (uint8_t edges = 0; edges < 2u; ++edges) {
        const uint32_t from = hi;
        const bool edge = brio::wait_ticks<P>(
            [&] {
                const uint32_t c = Rtc::count();
                if (static_cast<int32_t>(c - hi) > 0) {
                    hi = c;
                }
                return hi != from;
            },
            5u);
        if (!edge) {
            return r;
        }
    }
    brio::Stopwatch<Ruler> w;   // cycles only: right at any rate, the period constant in the window
    w.start();
    const uint32_t t0 = Ticker::ticks();
    // AND THE END IS THE FIRST READ 128 COUNTS ON, never an unsigned
    // difference across a stale read (since_clamped). 125 ms, bounded at
    // twice that on the tick.
    const bool end = brio::wait_ticks<P>(
        [&] { return brio::since_clamped(Rtc::count(), hi) >= rtc_window; }, 250u);
    const uint32_t cycles = w.elapsed();
    r.ticks = Ticker::ticks() - t0;
    r.hclk = cycles * (rtc_tr_hz / rtc_window);   // cycles in 1/8 s
    r.ok = end;
    return r;
}

/// THE BOUND on a rung's measured HCLK, in parts per ten thousand. The
/// instrument: each end of the window is an RTC edge seen through a read
/// that may be a stale copy for up to three RTCCLK periods (92 us, the
/// lag rtc.hpp measured), so each end can be that late - 2 x 92 us of
/// 125 ms, 15 parts in ten thousand; the poll turn at each end (one
/// count() and one cycles(), under 100 cycles at 8 MHz) adds under 2.
/// Then the two clocks. A crystal rung is the HSE against the LSE: two
/// quartz crystals, each taken at 100 ppm with its temperature stability -
/// an ASSUMPTION, the boards name no part (docs/boards/) - 2 more. An HSI
/// rung is the HSI's own accuracy after calibration at 0..70 C against
/// the LSE's 1: CH32V203 datasheet table 4-13, -1.0 to +1.6 %, on the
/// CH32V203s; CH32V303 datasheet table 4-14, -1.8 to +1.8 %, on the
/// CH32V303s - asymmetric, so an HSI rung's bound has a fast side and a
/// slow side.
constexpr uint32_t bound_xtal_pptt = 15u + 2u + 2u;
constexpr bool v303_datasheet = device::device_class == DeviceClass::v30x_d8;
constexpr uint32_t hsi_fast_pptt = v303_datasheet ? 180u : 160u;
constexpr uint32_t hsi_slow_pptt = v303_datasheet ? 180u : 100u;
constexpr uint32_t bound_hsi_fast_pptt = 15u + 2u + 1u + hsi_fast_pptt;
constexpr uint32_t bound_hsi_slow_pptt = 15u + 2u + 1u + hsi_slow_pptt;

/// The tick's bound over the window, in ticks: the same rate error over
/// 125 ticks, rounded up, plus the one tick of quantization the window's
/// two ends carry against the tick's own phase.
constexpr uint32_t tick_slack(uint32_t pptt) {
    return 1u + (125u * pptt + 9'999u) / 10'000u;
}

/// delay_us(500) timed on the core's counter: never early (the cycles the
/// rate in force asks), and on a run no tick lengthened, not more than a
/// sixteenth over - under the closest pair of distinct rates in the pack
/// (52 against 48 MHz, 8.3 %), so a factor taken for another rate fails;
/// the wait's own lateness is some 40 cycles at 144 MHz
/// (qingke/delay.hpp), cycles that do not grow at a slower rate (the
/// image runs from the zero-wait window), and the ruler's two reads
/// around it add some tens more, against the 250 a sixteenth allows at
/// 8 MHz. Up to
/// eight tries for a quiet run - a wait of half a tick period carries a
/// tick one try in two, so eight leave one rung in 256 unjudged - the
/// earliest bound judged on every try.
struct DelayReading {
    uint32_t cycles = 0;   ///< of the last try
    bool early = false;    ///< any try under the asked cycles
    bool quiet = false;    ///< the last try carried no tick
    bool served = true;    ///< delay_us() never refused
};

DelayReading delay_by_counter(uint32_t rate_hz) {
    DelayReading d;
    const uint32_t asked = 500u * (rate_hz / 1'000'000u);
    for (uint8_t t = 0; t < 8u && !d.quiet; ++t) {
        const brio::TickWitness<P> witness;
        brio::Stopwatch<Ruler> w;
        w.start();
        d.served = delay_us(clock, 500) && d.served;
        d.cycles = w.elapsed();
        d.quiet = witness.quiet();
        d.early = d.early || d.cycles < asked;
    }
    return d;
}

void tc_dynamic() {
    const bool locked_before = !RtcDomain::unlocked();
    const bool rtc_ok = rtc_ruler_up();
    bool up_ok = true;
    bool hclk_ok = true;
    bool tick_ok = true;
    bool delay_never_early = true;
    bool delay_near = true;
    uint8_t delay_unjudged = 0;
    bool readings_ok = true;

    // One rung: measured first, printed after (a console line going out
    // inside the window would put its handler there).
    auto rung = [&](uint8_t i, const char* dir, bool in_force) {
        const uint32_t asked = SysClock::rate_hz(i);
        // The pack's first five rates are the HSI's trees, the last five
        // the crystal's (rate_names).
        const bool hsi_rung = i < xtal_144_rate;
        const uint32_t fast_pptt = hsi_rung ? bound_hsi_fast_pptt : bound_xtal_pptt;
        const uint32_t slow_pptt = hsi_rung ? bound_hsi_slow_pptt : bound_xtal_pptt;
        const uint32_t pptt = fast_pptt > slow_pptt ? fast_pptt : slow_pptt;
        RungReading r;
        if (rtc_ok) {
            r = rung_by_rtc();
        }
        const DelayReading d = delay_by_counter(asked);
        const uint32_t off = r.hclk > asked ? r.hclk - asked : asked - r.hclk;
        const uint32_t toff = r.ticks > 125u ? r.ticks - 125u : 125u - r.ticks;
        const bool near = off <= (asked / 10'000u) * (r.hclk > asked ? fast_pptt : slow_pptt);
        const bool tick_near = toff <= tick_slack(pptt);
        const uint32_t asked_cycles = 500u * (asked / 1'000'000u);
        if (rtc_ok) {
            readings_ok = readings_ok && r.ok;
            hclk_ok = hclk_ok && r.ok && near;
            tick_ok = tick_ok && r.ok && tick_near;
        }
        delay_never_early = delay_never_early && d.served && !d.early;
        if (!d.quiet) {
            ++delay_unjudged;
        }
        delay_near = delay_near && (!d.quiet || d.cycles <= asked_cycles + asked_cycles / 16u);
        print(serial, "  ", dir, " ", i, " -> ", SysClock::hz(), " Hz (", rate_names[i], "), ",
              in_force ? "in force" : "NOT IN FORCE", crlf);
        if (rtc_ok) {
            print(serial, "      against the RTC: HCLK ", r.hclk, " Hz (bound +", fast_pptt,
                  " -", slow_pptt, " in 10000), ", r.ticks, " ticks in 125 ms (bound +-",
                  tick_slack(pptt), ")",
                  r.ok ? "" : ", AN EDGE NEVER CAME", crlf);
        }
        print(serial, "      delay_us(500): ", d.cycles, " cycles for ", asked_cycles, " asked",
              d.quiet ? "" : " (a tick inside every try: not judged late)", crlf);
    };

    for (uint8_t i = 0; i < SysClock::rate_count; ++i) {
        const bool ok = go_to(i) && rate_in_force(i);
        up_ok = ok && up_ok;
        rung(i, "up  ", ok);
    }
    bench.verdict("every rate of the pack entered in order, upwards through "
                  "the list",
                  up_ok);

    bool down_ok = true;
    for (uint8_t k = SysClock::rate_count; k > 0; --k) {
        const uint8_t i = static_cast<uint8_t>(k - 1u);
        const bool ok = go_to(i) && rate_in_force(i);
        down_ok = ok && down_ok;
        rung(i, "down", ok);
    }
    bench.verdict("and back down through it, the console rebased at every step",
                  down_ok);
    if (rtc_ok) {
        bench.verdict("the RTC's edges came at every rung (the window 128 counts of 1024 Hz)",
                      readings_ok);
        bench.verdict("HCLK counted against the RTC on the 32 kHz crystal is the rate asked at "
                      "every rung: within 0.19 % on the crystal's rungs, and on the HSI's within "
                      "the part's datasheet accuracy and the instrument's 0.18 %",
                      hclk_ok);
        bench.verdict("the kernel tick kept its millisecond at every rung, counted against the "
                      "RTC: 125 ticks in 125 ms within the rung's bound",
                      tick_ok);
    } else {
        bench.skip("the rungs' rates against the RTC: the LSE did not start, or the backup "
                   "domain stands on another source (not wiped here)");
    }
    bench.verdict("delay_us(500) was served and never early at any rate, dispatching on the "
                  "pack's index",
                  delay_never_early);
    print(serial, "  delay_us(500) rungs with a tick inside all eight tries, so not judged "
                  "late: ", delay_unjudged, " of ", 2u * SysClock::rate_count, crlf);
    bench.verdict("and not more than a sixteenth late on a run no tick lengthened",
                  delay_near);

    bench.verdict("set() by rate refuses a rate the pack does not name",
                  !SysClock::set(37'000'000UL) && !SysClock::set_index(99));

    (void)go_to(boot_rate);
    bench.verdict("the boot rate is back", rate_in_force(boot_rate));
    bench.verdict("restore() is a readback when the rate is already in force",
                  SysClock::restore() && rate_in_force(boot_rate));
    if (locked_before) {
        (void)RtcDomain::unlock(false);   // the domain's door as it was found
    }
}

// ---------------------------------------------------------------------------
// d - the LSI
// ---------------------------------------------------------------------------
void td_lsi() {
    (void)Rcc::lsi_start();
    bench.verdict("the LSI starts and reports itself ready", Rcc::lsi_ready());

    Rcc::lsi_stop();
    const uint32_t fall = wait_us([] { return !Rcc::lsi_ready(); }, 20'000);
    print(serial, "  LSIRDY fell ", fall, " us after LSION was cleared", crlf);
    bench.verdict("stopping it drops LSIRDY", !Rcc::lsi_ready());

    Rcc::lsi_enable(true);
    const uint32_t rise = wait_us([] { return Rcc::lsi_ready(); }, 20'000);
    print(serial, "  LSIRDY rose ", rise, " us after LSION was set (the "
                  "datasheet's figure is 230 us with the LSE running and 5 ms "
                  "without)", crlf);
    bench.verdict("and starting it raises LSIRDY again", Rcc::lsi_ready());

    print(serial, "  its RATE is nominal only - ", device::lsi_min_hz, " to ",
          device::lsi_max_hz, " Hz, typically ", device::lsi_typ_hz,
          " - and nothing in this chapter can count it: the watchdog and the "
          "RTC are what will", crlf);
}

// ---------------------------------------------------------------------------
// e - the HSE, and the HSI stopped under a tree that does not need it
// ---------------------------------------------------------------------------
void te_hse() {
    Rcc::hse_stop();
    const uint32_t fall = wait_us([] { return !Rcc::hse_ready(); }, 20'000);
    print(serial, "  HSERDY fell ", fall, " us after HSEON was cleared", crlf);

    // The ready INTERRUPT, armed over the same ramp: the flag that is
    // about to rise is the one the RCC vector reports.
    Rcc::clear_interrupt_flags(rcc_intr_flags);
    rcc_events = 0;
    rcc_flags = 0;
    Rcc::ready_interrupts(rcc_hserdyie);
    Pfic::enable(Irq::rcc);

    Rcc::hse_enable(false);
    const uint32_t rise = wait_us([] { return Rcc::hse_ready(); }, 100'000);
    print(serial, "  HSERDY rose ", rise, " us after HSEON was set (the "
                  "datasheet's figure for an 8 MHz crystal is ",
          device::device_class == DeviceClass::v30x_d8 ? "1.5 ms typical and 4 at most"
                                                       : "2.5 ms",
          ")", crlf);
    bench.verdict("the board's crystal starts and HSERDY reads back",
                  Rcc::hse_ready());

    print(serial, "  the RCC vector ran ", rcc_events, " time(s), the body "
                  "finding flags ", hex(rcc_flags), crlf);
    bench.verdict("the ready interrupt reached the RCC vector, and the body "
                  "found the HSE's own flag standing",
                  rcc_events != 0u && (rcc_flags & rcc_hserdyf) != 0u);
    Rcc::ready_interrupts(0);
    Pfic::disable(Irq::rcc);
    bench.verdict("and the body left no flag behind, so the line does not "
                  "re-enter",
                  (Rcc::interrupt_flags() & rcc_hserdyf) == 0u);

    // The HSI is only a root while something uses it. Under a tree whose
    // PLL is fed by the crystal, nothing does - so it can be stopped,
    // which is the one way to prove the verb does something.
    const bool moved = go_to(xtal_144_rate) && rate_in_force(xtal_144_rate);
    bench.verdict("the tree moved onto the crystal's PLL", moved);
    Rcc::hsi_stop();
    (void)delay_us(clock, 100);
    const bool hsi_off = !Rcc::hsi_ready();
    print(serial, "  with SYSCLK on the crystal's PLL, HSION cleared -> "
                  "HSIRDY=", Rcc::hsi_ready(), crlf);
    const bool hsi_back = Rcc::hsi_start();
    bench.verdict("the HSI stops while the tree runs off the crystal", hsi_off);
    bench.verdict("and comes back on demand", hsi_back && Rcc::hsi_ready());

    (void)go_to(boot_rate);
    bench.verdict("the boot rate is back, which needs the HSI it just lost",
                  rate_in_force(boot_rate));
    Rcc::hse_stop();
}

// ---------------------------------------------------------------------------
// f - the clock security system
// ---------------------------------------------------------------------------
void tf_css() {
    const uint32_t before = css_events;
    const bool started = Rcc::hse_start(false);
    bench.verdict("the crystal is running, which is what arms the detector",
                  started);

    Rcc::clock_monitor(true);
    bench.verdict("CSSON reads back", Rcc::clock_monitor());
    bench.verdict("and no failure stands", !Rcc::clock_failed());

    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 100u) {
    }
    print(serial, "  100 ms armed over a healthy crystal: CSS events ",
          css_events - before, ", other non-maskable interrupts ", nmi_others, crlf);
    bench.verdict("the non-maskable interrupt never fired", css_events == before);
    bench.verdict("and nothing else took that vector", nmi_others == 0u);

    Rcc::clock_monitor(false);
    bench.verdict("CSSON clears", !Rcc::clock_monitor());
    Rcc::hse_stop();

    print(serial, "  the failure itself is not staged here: it wants the "
                  "crystal killed on the board, and what the silicon does with "
                  "it - SYSCLK back on the HSI, the PLL off, the flag set - is "
                  "in the driver's header and not measured", crlf);
}

// ---------------------------------------------------------------------------
// g - the clock output
// ---------------------------------------------------------------------------

/// WHAT LEAVES THE PAD, counted. PA8 is TIM1's own channel 1 pad, so no
/// timer counts it in place; on the CH32V203C8T6's board PA8 is strapped
/// to PA6 and PA7 (SPI1's loop-back node), and PA6 is TIM3's channel 1 in
/// column 0 - the meter. The strap is a BOARD fact, so it is probed, not
/// assumed: PA8 driven both ways and PA6 read against the opposite pull.
constexpr Pad meter_pad = tim_channel_pad(3, 0, 0);   ///< PA6
static_assert(meter_pad == Pad{'A', 6});
using MeterTim = Tim<3>;
using MeterIn = Pin<meter_pad.port, meter_pad.pin>;
using McoPin = Pin<'A', 8>;

bool mco_strapped() {
    MeterIn::input(PinPull::down);
    McoPin::output(true);
    (void)delay_us(clock, 20);
    const bool high = MeterIn::read();
    MeterIn::input(PinPull::up);
    McoPin::clear();
    (void)delay_us(clock, 20);
    const bool low = !MeterIn::read();
    McoPin::release();
    MeterIn::release();
    return high && low;
}

/// The pad's rate in Hz: TIM3 counting PA6's rising edges in external
/// clock mode 1 over a gate of `gate_cycles` of HCLK, on the ruler (the
/// caller runs it at 144 MHz, the crystal's PLL). Each end reads the
/// counter and the core's clock as one masked pair, so an interrupt
/// cannot fall between them (the mask is a few loads, far under the
/// tick period whose pending flag the ruler's read carries); 16 bits of
/// count is 65 ms at 1 MHz and the gate is sized under it by the caller.
uint32_t mco_count_hz(uint32_t gate_cycles) {
    MeterTim::init();
    (void)MeterTim::remap(0);
    (void)MeterTim::configure({.prescaler = 0, .period = 0xFFFF});
    (void)MeterTim::capture_channel(0, {.select = TimChannelSelect::direct,
                                        .polarity = TimCapturePolarity::rising,
                                        .prescaler = TimCapturePrescaler::every,
                                        .filter = 0,
                                        .enable = true});
    (void)MeterTim::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti1});
    MeterIn::input(PinPull::none);
    MeterTim::set_count(0);
    MeterTim::enable(true);
    (void)delay_us(clock, 50);
    uint32_t c0 = 0, t0 = 0, c1 = 0, t1 = 0;
    {
        P::CriticalSection cs;
        c0 = MeterTim::count();
        t0 = Ruler::now();
    }
    while (Ruler::now() - t0 < gate_cycles) {
    }
    {
        P::CriticalSection cs;
        c1 = MeterTim::count();
        t1 = Ruler::now();
    }
    const uint32_t cycles = t1 - t0;
    MeterTim::enable(false);
    MeterIn::release();
    const uint32_t edges = (c1 - c0) & 0xFFFFu;
    return static_cast<uint32_t>(static_cast<uint64_t>(edges) * SysClock::hz() / cycles);
}

void mco_frequency() {
    if (!mco_strapped()) {
        bench.skip("the pad's frequency: no strap from PA8 to PA6 on this board (driven both "
                   "ways, PA6 did not follow); the CH32V203C8T6's board carries one");
        return;
    }
    // ON THE CRYSTAL'S TREE, so the core's counter is the crystal's too:
    // the HSE counted against it is a ratio of one oscillator to itself,
    // and the HSI against it is the HSI's own error.
    const bool moved = go_to(xtal_144_rate) && rate_in_force(xtal_144_rate);
    // 5 ms of HCLK: 40000 edges of an 8 MHz clock, inside the counter's
    // sixteen bits.
    constexpr uint32_t gate = 720'000u;
    (void)Mco::init(McoSource::hse);
    const uint32_t hse_hz = mco_count_hz(gate);
    (void)Mco::init(McoSource::hsi);
    const uint32_t hsi_hz = mco_count_hz(gate);
    Mco::off();
    (void)go_to(boot_rate);
    const int32_t hsi_ppm = static_cast<int32_t>(
        (static_cast<int64_t>(hsi_hz) - 8'000'000LL) * 1'000'000LL / 8'000'000LL);
    print(serial, "  through the PA8-PA6 strap, TIM3 counting 5 ms of the crystal's tree: "
                  "MCO = HSE reads ", hse_hz, " Hz, MCO = HSI reads ", hsi_hz, " Hz (",
          hsi_ppm, " ppm)", crlf);
    // THE BOUND for the HSE, a ratio of one oscillator to itself: the
    // count is a whole number of edges, one in 40000 or 200 Hz; and the
    // two ends' masked pairs - a counter load beside the ruler's composed
    // read - sit alike but for one retry of that read when a reload falls
    // inside it, half the 28 to 35 cycles test_vx03_rubato's letter d
    // measures for two reads: 18 cycles of the 720000, 200 Hz more.
    constexpr uint32_t hse_window_hz = 200u + 200u;
    bench.verdict("with MCO on the HSE the pad carries the crystal: 8 MHz to within one edge "
                  "of the 5 ms gate and one retry of the ruler's read (400 Hz), counted against "
                  "a core clock made from it",
                  moved && hse_hz + hse_window_hz >= 8'000'000u &&
                      hse_hz <= 8'000'000u + hse_window_hz);
    // The HSI's own: within the datasheet's accuracy after calibration at
    // 0..70 C (hsi_fast_pptt and hsi_slow_pptt above: the CH32V203's -1.0
    // to +1.6 %, the CH32V303's -1.8 to +1.8 %), plus the HSE crystal it is
    // counted against (100 ppm, the assumption above) and the edge (25 ppm).
    constexpr int32_t hsi_fast_ppm = static_cast<int32_t>(hsi_fast_pptt) * 100 + 125;
    constexpr int32_t hsi_slow_ppm = static_cast<int32_t>(hsi_slow_pptt) * 100 + 125;
    bench.verdict("with MCO on the HSI the pad carries the internal oscillator, inside the "
                  "part's datasheet accuracy after calibration, the crystal and the edge beside it",
                  hsi_ppm >= -hsi_slow_ppm && hsi_ppm <= hsi_fast_ppm);
    print(serial, "  SYSCLK and PLL/2 are read back above and not counted: at the rates "
                  "that make them they are 48 to 144 MHz, past both the pad's fastest class "
                  "(50 MHz, the datasheets' I/O AC table) and what a timer clocked at HCLK "
                  "can count of an outside edge", crlf);
}

void tg_mco() {
    if constexpr (!Mco::has_pad) {
        bench.skip("the clock output: this package does not bring PA8 out");
        return;
    }

    constexpr McoSource sources[] = {McoSource::sysclk, McoSource::hsi, McoSource::hse,
                                     McoSource::pll_div2};
    bool all_ok = true;
    for (const McoSource src : sources) {
        const bool claimed = Mco::init(src);
        const bool reads = Mco::source() == src;
        print(serial, "  ", mco_name(src), ": pad claimed ", claimed,
              ", multiplexer reads ", mco_name(Mco::source()), crlf);
        all_ok = claimed && reads && all_ok;
    }
    bench.verdict("every source this device class has is selected and reads "
                  "back",
                  all_ok);

    Mco::off();
    bench.verdict("off() takes the multiplexer to no clock and releases the pad",
                  Mco::source() == McoSource::none);

    mco_frequency();
}

// ---------------------------------------------------------------------------
// i - the ready interrupts of the other four roots
// ---------------------------------------------------------------------------

/// Arm one ready interrupt, run `start`, and wait (bounded, in kernel
/// ticks) for the vector to have run: the flags the body found.
template <typename Start, typename Ready>
uint32_t ready_through_vector(uint32_t enable, Start start, Ready ready, uint32_t limit_ms,
                              uint32_t& took_ms) {
    Rcc::clear_interrupt_flags(rcc_intr_flags);
    rcc_events = 0;
    rcc_flags = 0;
    Rcc::ready_interrupts(enable);
    Pfic::clear_pending(Irq::rcc);
    Pfic::enable(Irq::rcc);
    const uint32_t t0 = Ticker::ticks();
    start();
    while ((rcc_events == 0u || !ready()) && Ticker::ticks() - t0 < limit_ms) {
    }
    took_ms = Ticker::ticks() - t0;
    Rcc::ready_interrupts(0);
    Pfic::disable(Irq::rcc);
    return rcc_events == 0u ? 0u : rcc_flags;
}

void ti_ready() {
    // THE LSI: stopped, then started with LSIRDYIE armed.
    const bool lsi_was = Rcc::lsi_enabled();
    Rcc::lsi_stop();
    (void)wait_us([] { return !Rcc::lsi_ready(); }, 20'000);
    uint32_t ms = 0;
    const uint32_t lsi_f = ready_through_vector(
        rcc_lsirdyie, [] { Rcc::lsi_enable(true); }, [] { return Rcc::lsi_ready(); }, 50, ms);
    print(serial, "  LSI: the vector ran ", rcc_events, " time(s) within ", ms,
          " ms, the body finding ", hex(lsi_f), crlf);
    bench.verdict("the LSI's ready interrupt reaches the RCC vector with LSIRDYF standing, and "
                  "the body leaves no flag behind",
                  (lsi_f & rcc_lsirdyf) != 0u && (Rcc::interrupt_flags() & rcc_lsirdyf) == 0u);
    if (!lsi_was) {
        Rcc::lsi_stop();
    }

    // THE HSI: only stoppable under a tree that does not use it - the
    // crystal's PLL - then started with HSIRDYIE armed.
    const bool moved = go_to(xtal_144_rate) && rate_in_force(xtal_144_rate);
    Rcc::hsi_stop();
    (void)delay_us(clock, 100);
    const bool hsi_off = !Rcc::hsi_ready();
    // WHETHER THE HSI RAISES ITS READY FLAG, which 3.4.3 and 3.3.2 say
    // it does: on both parts it mostly does not - with HSIRDYIE armed
    // the vector does not run, and unarmed the flag does not stand after
    // HSIRDY has risen, the oscillator off for 100 us or for 20 ms - but
    // the CH32V303CBT6 has also raised it at the armed restart, in some
    // runs of the same image and not in others. brio depends on neither
    // answer (hsi_start() polls HSIRDY), so what the restart raised is a
    // printed finding, and the verdict judges the premises: the HSI was
    // stopped and is running again.
    const uint32_t hsi_f = ready_through_vector(
        rcc_hsirdyie, [] { Rcc::hsi_enable(true); }, [] { return Rcc::hsi_ready(); }, 50, ms);
    const uint32_t armed_runs = rcc_events;
    Rcc::hsi_stop();
    // Twenty milliseconds off, on the tick: delay_us() refuses a wait of
    // a tick period or more and would spend nothing.
    (void)brio::wait_ticks<P>([] { return false; }, 20u);   // a fixed span: never true
    const bool hsi_off_long = !Rcc::hsi_ready();
    Rcc::clear_interrupt_flags(rcc_intr_flags);
    Rcc::hsi_enable(true);
    (void)wait_us([] { return Rcc::hsi_ready(); }, 1'000);
    (void)delay_us(clock, 100);
    const uint32_t unarmed_flags = Rcc::interrupt_flags();
    print(serial, "  HSI: stopped under the crystal's PLL (", hsi_off ? "HSIRDY fell" : "HSIRDY STOOD",
          "), restarted with HSIRDYIE armed: the vector ran ", armed_runs, " time(s) in ", ms,
          " ms (body ", hex(hsi_f), "); off 20 ms (", hsi_off_long ? "HSIRDY fell" : "HSIRDY STOOD",
          ") and restarted unarmed: HSIRDY ", Rcc::hsi_ready() ? "set" : "CLEAR", ", the flags ",
          hex(unarmed_flags), crlf);
    const bool raised = armed_runs != 0u || (unarmed_flags & rcc_hsirdyf) != 0u;
    print(serial, "  FINDING: the HSI's restart ", raised ? "RAISED" : "raised no",
          " ready flag in this run (3.4.3 says it raises one): armed ",
          armed_runs != 0u ? "the vector ran" : "no vector", ", unarmed ",
          (unarmed_flags & rcc_hsirdyf) != 0u ? "HSIRDYF stood" : "no HSIRDYF", crlf);
    bench.verdict("the HSI stops under the crystal's PLL, for 100 us and for 20 ms, and runs "
                  "again after each restart",
                  moved && hsi_off && hsi_off_long && Rcc::hsi_ready());

    // THE PLL: off under the bare crystal, then started by the switch
    // back to the boot rate with PLLRDYIE armed - the switch polls
    // PLLRDY in CTLR, the vector reports the interrupt flag beside it.
    const bool bare = go_to(xtal_bare_rate) && rate_in_force(xtal_bare_rate) && !Rcc::pll_ready();
    const uint32_t pll_f = ready_through_vector(
        rcc_pllrdyie, [] { (void)go_to(boot_rate); }, [] { return Rcc::pll_ready(); }, 50, ms);
    print(serial, "  PLL: off under the bare crystal (", bare ? "PLLRDY clear" : "PLLRDY STOOD",
          "), restarted by the switch back to the boot rate: the vector ran ", rcc_events,
          " time(s), the body finding ", hex(pll_f), crlf);
    bench.verdict("the PLL's ready interrupt reaches the RCC vector with PLLRDYF standing, and "
                  "the boot rate is back",
                  bare && (pll_f & rcc_pllrdyf) != 0u && rate_in_force(boot_rate));

    // THE LSE: the backup domain's crystal, behind DBP. Stopped (3.4.9:
    // LSERDY falls six LSE cycles later), then started with LSERDYIE
    // armed, and its start-up - hundreds of milliseconds to seconds for a
    // 32 kHz crystal - waited out in ticks. Left as it was found.
    const bool locked_before = !RtcDomain::unlocked();
    const bool door = RtcDomain::unlock(true);
    const bool lse_was = RtcDomain::lse_enabled();
    RtcDomain::lse_enable(false);
    (void)wait_us([] { return !RtcDomain::lse_ready(); }, 20'000);
    const bool lse_off = !RtcDomain::lse_ready();
    const uint32_t lse_f = ready_through_vector(
        rcc_lserdyie, [] { RtcDomain::lse_enable(true); }, [] { return RtcDomain::lse_ready(); },
        5000, ms);
    print(serial, "  LSE: stopped (", lse_off ? "LSERDY fell" : "LSERDY STOOD",
          "), started: the vector ran ", rcc_events, " time(s), the crystal ready in ", ms,
          " ms, the body finding ", hex(lse_f), crlf);
    bench.verdict("the LSE's ready interrupt reaches the RCC vector with LSERDYF standing",
                  door && lse_off && (lse_f & rcc_lserdyf) != 0u &&
                      (Rcc::interrupt_flags() & rcc_lserdyf) == 0u);
    if (!lse_was) {
        RtcDomain::lse_enable(false);
    }
    if (locked_before) {
        (void)RtcDomain::unlock(false);
    }
}

// ---------------------------------------------------------------------------
// h - the peripheral gates
// ---------------------------------------------------------------------------
struct Gate {
    const char* name;
    Bus bus;
    uint32_t mask;
    bool present;   ///< the part table's word for the block behind it
};

/// Every gate the chapter lists for a block of this stratum's classes,
/// each with the part's own answer. The two the console runs on,
/// USART1's and GPIOA's, are not here.
constexpr Gate gates[] = {
    {"HB DMA1", Bus::hb, rcc_hb_dma1, true},
    {"HB DMA2", Bus::hb, rcc_hb_dma2, device::dma_controller_count >= 2u},
    {"HB SRAM", Bus::hb, rcc_hb_sram, true},
    {"HB CRC", Bus::hb, rcc_hb_crc, true},
    {"HB FSMC", Bus::hb, rcc_hb_fsmc, device::has_fsmc},
    {"HB RNG", Bus::hb, rcc_hb_rng, device::has_rng},
    {"HB SDIO", Bus::hb, rcc_hb_sdio, device::has_sdio},
    {"HB USBFS", Bus::hb, rcc_hb_usbfs, device::has_usbfs},
    {"PB2 AFIO", Bus::pb2, rcc_pb2_afio, true},
    {"PB2 GPIOB", Bus::pb2, rcc_pb2_gpiob, true},
    {"PB2 GPIOC", Bus::pb2, rcc_pb2_gpioc, true},
    {"PB2 GPIOD", Bus::pb2, rcc_pb2_gpiod, true},
    {"PB2 GPIOE", Bus::pb2, rcc_pb2_gpioe, device::has_port('E')},
    {"PB2 ADC1", Bus::pb2, rcc_pb2_adc1, true},
    {"PB2 ADC2", Bus::pb2, rcc_pb2_adc2, device::adc_count >= 2u},
    {"PB2 TIM1", Bus::pb2, rcc_pb2_tim1, true},
    {"PB2 SPI1", Bus::pb2, rcc_pb2_spi1, true},
    {"PB2 TIM8", Bus::pb2, rcc_pb2_tim8, (device::advanced_timer_instances & (1U << 8)) != 0u},
    {"PB2 TIM9", Bus::pb2, rcc_pb2_tim9, (device::advanced_timer_instances & (1U << 9)) != 0u},
    {"PB2 TIM10", Bus::pb2, rcc_pb2_tim10,
     (device::advanced_timer_instances & (1U << 10)) != 0u},
    {"PB1 TIM2", Bus::pb1, rcc_pb1_tim2, true},
    {"PB1 TIM3", Bus::pb1, rcc_pb1_tim3, true},
    {"PB1 TIM4", Bus::pb1, rcc_pb1_tim4, true},
    {"PB1 TIM5", Bus::pb1, rcc_pb1_tim5, device::has_tim5},
    {"PB1 TIM6", Bus::pb1, rcc_pb1_tim6, (device::basic_timer_instances & (1U << 6)) != 0u},
    {"PB1 TIM7", Bus::pb1, rcc_pb1_tim7, (device::basic_timer_instances & (1U << 7)) != 0u},
    {"PB1 WWDG", Bus::pb1, rcc_pb1_wwdg, true},
    {"PB1 SPI2", Bus::pb1, rcc_pb1_spi2, (device::spi_instances & (1U << 2)) != 0u},
    {"PB1 SPI3", Bus::pb1, rcc_pb1_spi3, (device::spi_instances & (1U << 3)) != 0u},
    {"PB1 USART2", Bus::pb1, rcc_pb1_usart2, device::has_usart(2)},
    {"PB1 USART3", Bus::pb1, rcc_pb1_usart3, device::has_usart(3)},
    {"PB1 UART4", Bus::pb1, rcc_pb1_uart4, device::has_usart(4)},
    {"PB1 UART5", Bus::pb1, rcc_pb1_uart5, device::has_usart(5)},
    {"PB1 UART6", Bus::pb1, rcc_pb1_uart6, device::has_usart(6)},
    {"PB1 UART7", Bus::pb1, rcc_pb1_uart7, device::has_usart(7)},
    {"PB1 UART8", Bus::pb1, rcc_pb1_uart8, device::has_usart(8)},
    {"PB1 I2C1", Bus::pb1, rcc_pb1_i2c1, true},
    {"PB1 I2C2", Bus::pb1, rcc_pb1_i2c2, device::i2c_count >= 2u},
    {"PB1 USBD", Bus::pb1, rcc_pb1_usbd, device::has_usbd},
    {"PB1 CAN1", Bus::pb1, rcc_pb1_can1, device::can_count >= 1u},
    {"PB1 BKP", Bus::pb1, rcc_pb1_bkp, true},
    {"PB1 PWR", Bus::pb1, rcc_pb1_pwr, true},
    {"PB1 DAC", Bus::pb1, rcc_pb1_dac, device::has_dac},
};

void th_gates() {
    bool all_ok = true;
    uint32_t tried = 0;
    for (const Gate& g : gates) {
        if (!g.present) {
            continue;
        }
        const bool was = Rcc::enabled(g.bus, g.mask);
        Rcc::enable(g.bus, g.mask);
        const bool on = Rcc::enabled(g.bus, g.mask);
        Rcc::disable(g.bus, g.mask);
        const bool off = !Rcc::enabled(g.bus, g.mask);
        if (was) {
            Rcc::enable(g.bus, g.mask);
        }
        if (!on || !off) {
            print(serial, "  ", g.name, ": opened ", on, " closed ", off, crlf);
        }
        all_ok = on && off && all_ok;
        ++tried;
    }
    print(serial, "  ", tried, " gates opened and closed, each read back", crlf);
    bench.verdict("every gate the chapter lists for this part opens and closes, "
                  "read back one at a time",
                  all_ok);
    print(serial, "  USART1's and GPIOA's gates are the two left alone: this "
                  "console runs on them", crlf);

    for (const Gate& g : gates) {
        if (g.present) {
            continue;
        }
        const bool was = Rcc::enabled(g.bus, g.mask);
        Rcc::enable(g.bus, g.mask);
        const bool on = Rcc::enabled(g.bus, g.mask);
        Rcc::disable(g.bus, g.mask);
        if (was) {
            Rcc::enable(g.bus, g.mask);
        }
        print(serial, "  ", g.name, " (not on this part): the bit holds a one: ", on, crlf);
    }
    // The two resets of a block that is not in use: pulsing them proves
    // the verb reaches the register, and CRC has nothing to lose.
    Rcc::enable(Bus::hb, rcc_hb_crc);
    Rcc::reset(Bus::hb, rcc_hb_crc);
    bench.verdict("a peripheral reset pulse leaves its gate where it was",
                  Rcc::enabled(Bus::hb, rcc_hb_crc));
    Rcc::disable(Bus::hb, rcc_hb_crc);
}

void banner() {
    print(serial, crlf, "test_vx03_clock on ", device::part_name, " - the clock tree",
          crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

// The three vectors this program owns. The non-maskable one is the
// clock security system's: the driver's body says whether the CSS was
// the reason, and only this suite's own counters are touched here.
BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }

BRIO_CH32_VECTOR(rcc_handler) {
    rcc_flags = rcc_flags | brio::Rcc::ready_isr();
    rcc_events = rcc_events + 1u;
}

BRIO_CH32_VECTOR(nmi_handler) {
    if (brio::Rcc::css_isr()) {
        css_events = css_events + 1u;
    } else {
        nmi_others = nmi_others + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the tree as the registers hold it at boot", ta_tree);
    bench.letter('b', "every static rate, bracketed for the host's clock", tb_rates);
    bench.letter('c', "the dynamic pack, up and down", tc_dynamic);
    bench.letter('d', "the LSI and the time to LSIRDY", td_lsi);
    bench.letter('e', "the crystal's start-up, its ready interrupt, and the "
                      "HSI stopped", te_hse);
    bench.letter('f', "the clock security system over a healthy crystal", tf_css);
    bench.letter('g', "the clock output on its pad", tg_mco);
    bench.letter('h', "the peripheral gates, one at a time", th_gates);
    bench.letter('i', "the ready interrupts of the LSI, the HSI, the PLL and the LSE", ti_ready);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL144" : "FAILED", " tick=",
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched",
                    brio::crlf);
        bench.prompt();
    }
}
