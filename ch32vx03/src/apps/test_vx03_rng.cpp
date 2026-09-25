// test_vx03_rng - the reference bench suite for the CH32V303's RANDOM
// NUMBER GENERATOR: ch32vx03/rng.hpp over RM ch. 29, the three registers,
// the clock that runs them and the words they hand over.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// ONE PART'S SUITE. The block is the CH32V303RC's and VC's alone (the
// datasheet's table 2-1-1), so this suite builds for that board and for
// no CH32V203; the driver refuses the type on every other part at
// compile time (test/family_ch32vx03/neg/rng_absent.cpp).
//
// THE CLOCK. Chapter 29 names PLL48CLK - the 48 MHz the PLL makes
// through USBPRE - and the class's own tree (figure 3-3) and the
// datasheet's draw SYSCLK to the TRNG. The two readings predict
// different numbers, and letter c measures them: the time from one word
// to the next in core cycles at three PLL rates (constant if SYSCLK runs
// the block, three to two to one if the 48 MHz does), the same with the
// USB prescaler moved under a running generator, and the generator on
// the bare HSI with no PLL at all. The driver asks nothing of the clock
// tree because of what this letter measures.
//
// THE WORDS ARE MEASURED, NOT CERTIFIED. On the CH32V303VCT6 they fail
// the statistics a random source passes (rng.hpp's fact 7): they come
// from a small set, a chi-square over their bytes is two orders of
// magnitude off, FIPS PUB 140-2's tests do not all pass, and DRDY rises
// over an unchanged word now and then. Letters d to g print the numbers
// and ASSERT THAT FINDING beside the driver's own guarantees; on a die
// whose words pass, those verdicts turn red, and the document is then
// what changes.
//
// NOTHING IS WIRED AND NOTHING WEARS. Every letter costs the board
// nothing, so `z` carries all of them. Letter c moves the clock tree and
// puts it back.
//
// What is exercised, letter by letter:
//   a  THE BLOCK: the HB gate, RNGEN, the time to the enable's first word
//      and that word's value, init()'s discard, the statuses clean, the
//      vector's number
//   b  THE WORD: the time from a read to the next DRDY, the words a second
//      through read(), and the words DRDY hands over unchanged
//   c  THE CLOCK: the word time at 144, 96 and 48 MHz; the USB prescaler
//      moved under a running generator; the bare HSI; the monitor across
//      every switch; the way back
//   d  THE SMALL SET: the distinct words among 4096, and the bit balance
//      of the 32 lanes over 20000
//   e  FIPS PUB 140-2's monobit, poker, runs and long-run tests over
//      20000 bits
//   f  A CHI-SQUARE over the 256 values of 65536 bytes
//   g  THE CONTINUOUS TEST: a hundred thousand words through read(), none
//      equal to its predecessor handed out, the refusals counted
//   h  THE LATCHED FLAGS: a zero clears them, a one sets neither, the
//      reserved bits keep nothing
//   i  THE VECTOR: IE over a running generator, the body taking a word
//      at every entry, and the seed flag a program cannot raise
//   j  THE RECOVERY SEQUENCE on a healthy generator, a word left standing
//      across a disable, the word that outlives the disable, and the gate
//      shut
//
// build: boards = v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/rng.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

// ---------------------------------------------------------------------------
// The rates: three PLL rates whose USB prescaler makes 48 MHz (by 3, 2
// and 1), and the bare HSI, which has no PLL and so no USB clock at all.
// SYSCLK is HCLK at every one of them, which is what lets a word time in
// core cycles be read as a word time in SYSCLK cycles.
// ---------------------------------------------------------------------------
using Top = Clock<ClockSource::pll, 144'000'000>;
using Mid = Clock<ClockSource::pll, 96'000'000>;
using Low = Clock<ClockSource::pll, 48'000'000>;
using Bare = Clock<ClockSource::internal, 8'000'000>;
using SysClock = DynamicClock<Rates<Top, Mid, Low, Bare>, Ticker, Serial>;
constexpr SysClock clock;

constexpr uint8_t top_rate = 0;
constexpr uint8_t mid_rate = 1;
constexpr uint8_t low_rate = 2;
constexpr uint8_t bare_rate = 3;

TestBench<Serial> bench;

/// The vector's tally (letter i).
volatile uint32_t rng_irqs = 0;
volatile uint32_t rng_words = 0;
volatile uint32_t rng_seed_events = 0;
volatile uint32_t rng_clock_events = 0;
volatile uint32_t rng_irq_limit = 0;
volatile uint32_t rng_last = 0;

/// The generator running, whatever the letter before left: every letter
/// can be run alone.
bool running() {
    if (Rng::clock() && Rng::enabled() && !Rng::seed_error() && !Rng::clock_error()) {
        return true;
    }
    return Rng::init();
}

/// The STK's period, for folding a span across one reload.
uint32_t stk_period() { return stk()->CMPLR + 1u; }

uint32_t stk_span(uint32_t from, uint32_t to) {
    return to >= from ? to - from : to + stk_period() - from;
}

/// THE WORD TIME: RNG_DR read (which puts DRDY down), then DRDY polled
/// until the next word stands, in core cycles, with interrupts masked so
/// that a tick cannot land inside a sample. `lost` counts samples in
/// which no word came within the bound.
struct WordTime {
    uint32_t min = 0xFFFFFFFFu;
    uint32_t max = 0;
    uint32_t sum = 0;
    uint32_t taken = 0;
    uint32_t lost = 0;

    uint32_t avg() const { return taken == 0u ? 0u : sum / taken; }
};

WordTime word_time(uint32_t samples) {
    WordTime w;
    InterruptGuard guard;
    for (uint32_t i = 0; i < samples; ++i) {
        (void)Rng::value();
        const uint32_t t0 = stk()->CNTL;
        uint32_t polls = 0;
        bool came = true;
        while (!Rng::ready()) {
            if (++polls > 5'000u) {
                came = false;
                break;
            }
        }
        const uint32_t t1 = stk()->CNTL;
        if (!came) {
            ++w.lost;
            continue;
        }
        const uint32_t dt = stk_span(t0, t1);
        w.min = dt < w.min ? dt : w.min;
        w.max = dt > w.max ? dt : w.max;
        w.sum += dt;
        ++w.taken;
    }
    return w;
}

void print_word_time(const char* label, const WordTime& w) {
    print(serial, "  ", label, ": ", w.taken, " words, ", w.taken == 0u ? 0u : w.min, "/",
          w.avg(), "/", w.max, " cycles min/avg/max");
    if (w.lost != 0u) {
        print(serial, ", ", w.lost, " LOST");
    }
    print(serial, crlf);
}

/// Two word times agree when their averages are within half again of
/// each other - the SYSCLK reading's "the same number of core cycles".
/// The averages wander between 19 and 25 cycles from run to run with
/// where the polls fall, and the other reading predicts three times as
/// many at 144 MHz as at 48, which this bound cannot mistake.
bool same_cycles(const WordTime& a, const WordTime& b) {
    const uint32_t x = a.avg();
    const uint32_t y = b.avg();
    return x != 0u && y != 0u && x * 2u <= y * 3u && y * 2u <= x * 3u;
}

// ===========================================================================
// a - the block
// ===========================================================================
void ta_block() {
    // From nothing: the gate and the generator off, then the staged
    // start, so the time to the first word is a measurement - with the
    // word a disable may leave standing taken before RNGEN is set, so
    // that the word timed is the enable's own.
    Rng::release();
    const bool gate_shut = !Rng::clock();
    Rng::clock(true);
    const bool gate_open = Rng::clock();
    (void)Rng::value();
    const uint32_t sr_idle = Rng::status();
    uint32_t t0 = 0;
    uint32_t t1 = 0;
    uint32_t polls = 0;
    {
        InterruptGuard guard;
        t0 = stk()->CNTL;
        Rng::enable(true);
        while (!Rng::ready() && polls < Rng::spins) {
            ++polls;
        }
        t1 = stk()->CNTL;
    }
    const bool first = Rng::ready();
    const uint32_t first_cycles = stk_span(t0, t1);
    const uint32_t first_word = Rng::value();
    print(serial, "  the gate ", gate_shut ? "shut" : "OPEN", " after release, ",
          gate_open ? "open" : "SHUT", " after clock(true); RNG_SR before RNGEN: ",
          hex(sr_idle), crlf);
    print(serial, "  RNGEN set: the first word ", first ? "stood after " : "NEVER came in ",
          first_cycles, " core cycles (", polls, " polls), and it reads ", hex(first_word), crlf);
    bench.verdict("the HB gate opens and closes the block, and with RNGEN set a first word "
                  "stands in RNG_DR",
                  gate_shut && gate_open && first);
    // The WORD is printed and not judged: a re-enable's first word is
    // the zero letter j measures 256 times over, but the first enable
    // after a reset gave zero at every boot of one day and a nonzero
    // word at every boot after a power cycle - which is why a restart
    // discards two words whatever the first one is.
    bench.verdict("the enable's first word stands - zero or not: a re-enable's is the zero "
                  "of letter j, the first after a reset has been both, and a restart "
                  "discards it either way",
                  first);

    // The driver's own bring-up over it: the discard, the statuses.
    const bool started = Rng::init();
    const uint32_t sr = Rng::status();
    print(serial, "  init(): ", started ? "running" : "FAILED", ", RNG_CR ", hex(Rng::regs().CR),
          ", RNG_SR ", hex(sr), crlf);
    bench.verdict("init() takes a word away and leaves no error standing, current or latched",
                  started && Rng::enabled() &&
                      (sr & (rng_secs | rng_cecs | rng_seis | rng_ceis)) == 0u);

    print(serial, "  the vector: line ", static_cast<uint32_t>(Rng::irq),
          " of the CH32V30x_D8's table", crlf);
    bench.verdict("the block's vector is line 63 of this class's table",
                  static_cast<uint32_t>(Rng::irq) == 63u);
}

// ===========================================================================
// b - the word
// ===========================================================================
void tb_word() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    // DRDY goes down with the read.
    (void)Rng::read_blocking();
    (void)Rng::value();
    const bool down = !Rng::ready();
    const WordTime w = word_time(256);
    print(serial, "  DRDY right after a read of RNG_DR: ", down ? "down" : "STILL UP", crlf);
    print_word_time("from a read to the next DRDY at 144 MHz", w);
    bench.verdict("reading RNG_DR puts DRDY down, and the next word stands tens of core cycles "
                  "later every time", down && w.lost == 0u && w.taken == 256u && w.max < 1000u);

    // The throughput through the driver's own protocol, the refusals
    // told apart: a repeat is the comparison at work, an error is not.
    uint32_t words = 0;
    uint32_t repeats = 0;
    uint32_t errors = 0;
    const uint32_t start = Ticker::millis();
    while (Ticker::millis() - start < 100u) {
        if (Rng::read()) {
            ++words;
            continue;
        }
        const RngError e = Rng::last_error();
        if (e == RngError::repeated) {
            ++repeats;
        } else if (e != RngError::not_ready) {
            ++errors;
        }
    }
    print(serial, "  through read(): ", words * 10u, " words a second, ", repeats * 10u,
          " a second refused as repeats, ", errors, " errors", crlf);
    bench.verdict("read() hands out words at the generator's pace and refuses none for an "
                  "error", words != 0u && errors == 0u);

    // THE RAW WORDS: DRDY polled and RNG_DR read as fast as the core
    // goes, each compared with the one before - the repeats the driver's
    // comparison refuses, seen at the register.
    uint32_t raw = 0;
    uint32_t raw_repeats = 0;
    uint32_t prev = Rng::value();
    {
        InterruptGuard guard;
        while (raw < 20'000u) {
            uint32_t polls = 0;
            while (!Rng::ready() && polls < 5'000u) {
                ++polls;
            }
            const uint32_t v = Rng::value();
            raw_repeats += v == prev ? 1u : 0u;
            prev = v;
            ++raw;
        }
    }
    print(serial, "  20000 words at DRDY's pace, read raw: ", raw_repeats,
          " equal to the word before", crlf);
    bench.verdict("DRDY rises over an UNCHANGED word now and then - what read()'s comparison "
                  "refuses (fact 7)", raw_repeats != 0u);
}

// ===========================================================================
// c - the clock
// ===========================================================================
/// After a clock switch: what the monitor latched across it (cleared),
/// and whether the generator came back BY ITSELF - where it did not,
/// the restart 29.2.2 does not ask for, so that the next measurement has
/// words to time. Both answers are printed.
struct Across {
    bool latched;
    bool resumed;
};

Across across_switch() {
    const bool latched = Rng::clock_error_flag();
    Rng::clear_clock_error();
    const bool resumed = Rng::read_blocking().has_value();
    if (!resumed) {
        (void)Rng::recover();
    }
    return Across{latched, resumed};
}

void print_across(const char* label, const Across& a) {
    print(serial, "  ", label, ": CEIS ", a.latched ? "LATCHED" : "clear", ", the generator ",
          a.resumed ? "went on by itself" : "had to be RESTARTED", crlf);
}

bool quiet(const Across& a) { return !a.latched && a.resumed; }

void tc_clock() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    // Three PLL rates, the generator left running across the switches:
    // every switch parks on the HSI and stops the PLL (clock.hpp), which
    // under chapter 29's reading would take the 48 MHz away for a moment.
    const WordTime w144 = word_time(128);
    const bool mid = SysClock::set_index(mid_rate);
    const Across to_96 = across_switch();
    const WordTime w96 = word_time(128);
    const bool low = SysClock::set_index(low_rate);
    const Across to_48 = across_switch();
    const WordTime w48 = word_time(128);
    const bool top = SysClock::set_index(top_rate);
    const Across to_144 = across_switch();
    print_word_time("at 144 MHz", w144);
    print_word_time("at  96 MHz", w96);
    print_word_time("at  48 MHz", w48);
    print_across("the switch to 96 MHz", to_96);
    print_across("the switch to 48 MHz", to_48);
    print_across("the switch back to 144 MHz", to_144);

    // THE READING: a word time constant in core cycles is SYSCLK's; one
    // that runs three to two to one is the 48 MHz domain's.
    const uint32_t ratio = w48.avg() == 0u ? 0u : (w144.avg() * 100u) / w48.avg();
    print(serial, "  the word time at 144 over the word time at 48, in core cycles: ",
          ratio / 100u, ".", (ratio % 100u) / 10u, (ratio % 10u), crlf);
    bench.verdict("a word came at every one of the three PLL rates",
                  mid && low && top && w144.taken != 0u && w96.taken != 0u && w48.taken != 0u);
    bench.verdict("the word time is the same number of core cycles at 144, 96 and 48 MHz - "
                  "SYSCLK runs the generator (figure 3-3), not chapter 29's PLL48CLK",
                  same_cycles(w144, w96) && same_cycles(w144, w48));

    // THE USB PRESCALER MOVED under the running generator at 144 MHz:
    // 72 MHz and then 144 on its output, where the 48 MHz reading has
    // the generator speed up and the SYSCLK one does not notice.
    // Each write read back: a field the PLL's running state locked would
    // make the two timings below a measurement of nothing.
    const bool div2 = Rcc::usb_prescaler(2) && Rcc::usb_prescaler() == usbpre_code_for(2);
    const WordTime w_72 = word_time(128);
    const Across at_72 = across_switch();
    const bool div1 = Rcc::usb_prescaler(1) && Rcc::usb_prescaler() == usbpre_code_for(1);
    const WordTime w_144 = word_time(128);
    const Across at_144 = across_switch();
    const bool div3 = Rcc::usb_prescaler(3) && Rcc::usb_prescaler() == usbpre_code_for(3);
    print(serial, "  USBPRE written /2, /1 and back to /3 under the running PLL: ",
          div2 ? "taken" : "NOT TAKEN", ", ", div1 ? "taken" : "NOT TAKEN", ", ",
          div3 ? "taken" : "NOT TAKEN", crlf);
    print_word_time("USBPRE /2 (72 MHz out)", w_72);
    print_word_time("USBPRE /1 (144 MHz out)", w_144);
    print_across("USBPRE /2", at_72);
    print_across("USBPRE /1", at_144);
    bench.verdict("the USB prescaler takes /2 and /1 under a running generator and goes back "
                  "to /3, and the word time does not move with it",
                  div2 && div1 && div3 && same_cycles(w144, w_72) && same_cycles(w144, w_144));

    // THE BARE HSI: no PLL, so no USB clock. init() asks nothing of the
    // clock tree, and the block is started there from off.
    const bool bare = SysClock::set_index(bare_rate);
    Rng::release();
    const bool started = Rng::init();
    uint32_t words = 0;
    uint32_t cecs_seen = 0;
    const uint32_t start = Ticker::millis();
    while (Ticker::millis() - start < 5u) {
        const uint32_t sr = Rng::status();
        if ((sr & rng_cecs) != 0u) {
            ++cecs_seen;
        }
        if ((sr & rng_drdy) != 0u) {
            (void)Rng::value();
            ++words;
        }
    }
    const bool ceis_bare = Rng::clock_error_flag();
    // Sampled only where words came: a sample that waits for a word that
    // never comes waits with interrupts masked.
    const WordTime w8 = words != 0u ? word_time(32) : WordTime{};
    const bool back = SysClock::set_index(top_rate);
    const Across from_bare = across_switch();
    const bool cecs_after = Rng::clock_error();
    const bool again = Rng::read_blocking().has_value();
    print(serial, "  on the bare HSI at 8 MHz: init() ", started ? "started the generator"
          : "FAILED", crlf);
    print(serial, "  over 5 ms there: ", words, " words, CECS seen in ", cecs_seen,
          " status reads, CEIS ", ceis_bare ? "LATCHED" : "clear", crlf);
    print_word_time("at 8 MHz on the HSI", w8);
    print_across("back to 144 MHz from the HSI", from_bare);
    print(serial, "  and then: CECS ", cecs_after ? 1 : 0, ", a word through read() ",
          again ? "came" : "did NOT come", crlf);
    bench.verdict("on the bare HSI - no PLL and so no USB clock - init() starts the generator "
                  "and words come with the clock monitor quiet, current and latched",
                  bare && started && words > 100u && cecs_seen == 0u && !ceis_bare);
    bench.verdict("across every switch of the clock task and every move of the prescaler the "
                  "monitor latched nothing and the generator went on by itself",
                  quiet(to_96) && quiet(to_48) && quiet(to_144) && quiet(at_72) &&
                      quiet(at_144) && quiet(from_bare));
    bench.verdict("back at 144 MHz read() hands out words", back && !cecs_after && again);
}

// ===========================================================================
// d - the small set
// ===========================================================================
uint32_t pool[4096];

void shell_sort(uint32_t* a, uint32_t n) {
    static constexpr uint32_t gaps[] = {1750, 701, 301, 132, 57, 23, 10, 4, 1};
    for (const uint32_t g : gaps) {
        for (uint32_t i = g; i < n; ++i) {
            const uint32_t t = a[i];
            uint32_t j = i;
            while (j >= g && a[j - g] > t) {
                a[j] = a[j - g];
                j -= g;
            }
            a[j] = t;
        }
    }
}

void td_small_set() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    // THE DISTINCT WORDS among 4096 handed out by read(): a source of 32
    // random bits a word repeats one among 4096 with odds of about one in
    // half a million.
    uint32_t got = 0;
    for (uint32_t i = 0; i < 4096u; ++i) {
        const auto v = Rng::read_blocking();
        if (!v) {
            break;
        }
        pool[i] = *v;
        ++got;
    }
    shell_sort(pool, got);
    uint32_t distinct = got == 0u ? 0u : 1u;
    uint32_t run = 1;
    uint32_t most = got == 0u ? 0u : 1u;
    uint32_t most_word = got == 0u ? 0u : pool[0];
    for (uint32_t i = 1; i < got; ++i) {
        if (pool[i] == pool[i - 1u]) {
            ++run;
            if (run > most) {
                most = run;
                most_word = pool[i];
            }
        } else {
            run = 1;
            ++distinct;
        }
    }
    print(serial, "  ", got, " words through read(): ", distinct, " distinct, the most frequent ",
          hex(most_word), " seen ", most, " times", crlf);
    bench.verdict("the words come from a SMALL SET: fewer than 4000 distinct among 4096 "
                  "(fact 7)", got == 4096u && distinct < 4000u);

    // The bit balance of each lane over 20000 words, printed: a binomial
    // of 20000 fair bits is 10000 with a standard deviation of 70.7.
    constexpr uint32_t words = 20'000;
    uint32_t ones[32] = {};
    uint32_t taken = 0;
    for (uint32_t i = 0; i < words; ++i) {
        const auto v = Rng::read_blocking();
        if (!v) {
            break;
        }
        ++taken;
        for (uint32_t b = 0; b < 32u; ++b) {
            ones[b] += (*v >> b) & 1u;
        }
    }
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    for (uint32_t b = 0; b < 32u; ++b) {
        lo = ones[b] < lo ? ones[b] : lo;
        hi = ones[b] > hi ? ones[b] : hi;
    }
    constexpr uint32_t expected = words / 2u;
    const uint32_t worst = (expected - lo) > (hi - expected) ? expected - lo : hi - expected;
    print(serial, "  ", taken, " words: the lanes hold ", lo, " to ", hi, " ones of ", words,
          " (", expected, " expected, the worst ", worst, " off - ", (worst * 10u) / 707u, ".",
          ((worst * 100u) / 707u) % 10u, " standard deviations)", crlf);
    bench.verdict("every word of the 20000 came", taken == words);
}

// ===========================================================================
// e - FIPS PUB 140-2's four tests over 20000 bits
// ===========================================================================
uint32_t fips_words[625];

void te_fips() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    uint32_t got = 0;
    for (uint32_t i = 0; i < 625u; ++i) {
        const auto v = Rng::read_blocking();
        if (!v) {
            break;
        }
        fips_words[i] = *v;
        ++got;
    }

    // The monobit test, and the poker test's sixteen nibble counts.
    uint32_t ones = 0;
    uint32_t nibbles[16] = {};
    for (uint32_t i = 0; i < 625u; ++i) {
        uint32_t w = fips_words[i];
        for (uint32_t k = 0; k < 8u; ++k) {
            ++nibbles[(w >> 28) & 0xFu];
            w <<= 4;
        }
        for (uint32_t b = 0; b < 32u; ++b) {
            ones += (fips_words[i] >> b) & 1u;
        }
    }
    uint32_t sq = 0;
    for (uint32_t k = 0; k < 16u; ++k) {
        sq += nibbles[k] * nibbles[k];
    }
    // X = 16/5000 * sum(f^2) - 5000, in hundredths: (16 * sum - 25e6) / 50.
    const int32_t poker100 = (static_cast<int32_t>(16u * sq) - 25'000'000) / 50;

    // The runs, of zeros and of ones, most significant bit first.
    uint32_t runs[2][6] = {};
    uint32_t longest = 0;
    uint32_t current = 0;
    uint32_t length = 0;
    for (uint32_t i = 0; i < 625u; ++i) {
        for (int32_t b = 31; b >= 0; --b) {
            const uint32_t bit = (fips_words[i] >> b) & 1u;
            if (length != 0u && bit == current) {
                ++length;
                continue;
            }
            if (length != 0u) {
                ++runs[current][length >= 6u ? 5u : length - 1u];
                longest = length > longest ? length : longest;
            }
            current = bit;
            length = 1;
        }
    }
    ++runs[current][length >= 6u ? 5u : length - 1u];
    longest = length > longest ? length : longest;

    // FIPS PUB 140-2, 4.9.1: the intervals each run count must fall in.
    constexpr uint32_t run_lo[6] = {2343, 1135, 542, 251, 111, 111};
    constexpr uint32_t run_hi[6] = {2657, 1365, 708, 373, 201, 201};
    bool runs_ok = true;
    for (uint32_t k = 0; k < 6u; ++k) {
        for (uint32_t side = 0; side < 2u; ++side) {
            if (runs[side][k] < run_lo[k] || runs[side][k] > run_hi[k]) {
                runs_ok = false;
            }
        }
    }
    const bool monobit_ok = ones > 9725u && ones < 10275u;
    const bool poker_ok = poker100 > 216 && poker100 < 4617;
    const bool long_ok = longest < 26u;
    print(serial, "  ", got * 32u, " bits: monobit ", ones, " ones (9725..10275), poker ",
          poker100 / 100, ".", (poker100 % 100) / 10, poker100 % 10,
          " (2.16..46.17), longest run ", longest, " (under 26)", crlf);
    print(serial, "  runs of zeros 1..6+:");
    for (uint32_t k = 0; k < 6u; ++k) {
        print(serial, " ", runs[0][k]);
    }
    print(serial, crlf, "  runs of ones  1..6+:");
    for (uint32_t k = 0; k < 6u; ++k) {
        print(serial, " ", runs[1][k]);
    }
    print(serial, crlf, "  monobit ", monobit_ok ? "passes" : "FAILS", ", poker ",
          poker_ok ? "passes" : "FAILS", ", runs ", runs_ok ? "pass" : "FAIL", ", long run ",
          long_ok ? "passes" : "FAILS", crlf);
    bench.verdict("the words FAIL FIPS PUB 140-2 over 20000 bits: not all four of its tests "
                  "pass (fact 7)", got == 625u && !(monobit_ok && poker_ok && runs_ok && long_ok));
}

// ===========================================================================
// f - a chi-square over bytes
// ===========================================================================
uint32_t byte_counts[256];

void tf_chi_square() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    for (uint32_t k = 0; k < 256u; ++k) {
        byte_counts[k] = 0;
    }
    uint32_t got = 0;
    for (uint32_t i = 0; i < 16'384u; ++i) {
        const auto v = Rng::read_blocking();
        if (!v) {
            break;
        }
        ++got;
        ++byte_counts[*v & 0xFFu];
        ++byte_counts[(*v >> 8) & 0xFFu];
        ++byte_counts[(*v >> 16) & 0xFFu];
        ++byte_counts[*v >> 24];
    }
    // 65536 bytes over 256 values: 256 expected in each. The statistic
    // in tenths, sum((c - 256)^2) * 10 / 256.
    uint32_t sum = 0;
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    for (uint32_t k = 0; k < 256u; ++k) {
        const int32_t d = static_cast<int32_t>(byte_counts[k]) - 256;
        sum += static_cast<uint32_t>(d * d);
        lo = byte_counts[k] < lo ? byte_counts[k] : lo;
        hi = byte_counts[k] > hi ? byte_counts[k] : hi;
    }
    const uint32_t chi10 = (sum * 10u) / 256u;
    print(serial, "  ", got * 4u, " bytes: each value seen ", lo, " to ", hi,
          " times (256 expected); chi-square ", chi10 / 10u, ".", chi10 % 10u,
          " on 255 degrees of freedom (a uniform source: 175..350)", crlf);
    bench.verdict("the 256 byte values are NOT uniform: a chi-square past ten times its "
                  "degrees of freedom (fact 7)", got == 16'384u && chi10 > 25'500u);
}

// ===========================================================================
// g - the continuous test
// ===========================================================================
void tg_continuous() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    uint32_t words = 0;
    uint32_t repeats = 0;
    uint32_t errors = 0;
    uint32_t handed_equal = 0;
    uint32_t last = 0;
    bool have_last = false;
    while (words < 100'000u) {
        if (const auto v = Rng::read()) {
            handed_equal += (have_last && *v == last) ? 1u : 0u;
            last = *v;
            have_last = true;
            ++words;
            continue;
        }
        const RngError e = Rng::last_error();
        if (e == RngError::repeated) {
            ++repeats;
        } else if (e != RngError::not_ready) {
            ++errors;
            break;
        }
    }
    print(serial, "  ", words, " words through read(): ", repeats,
          " refused as the word before it again, ", handed_equal,
          " handed out equal to the one before, ", errors, " errors", crlf);
    bench.verdict("of a hundred thousand words read() handed out none equal to the one "
                  "before it, and refused none for an error",
                  words == 100'000u && handed_equal == 0u && errors == 0u);
    bench.verdict("and the comparison it makes on every word refused some - the generator "
                  "repeats a word at DRDY's pace (fact 7)", repeats != 0u);
}

// ===========================================================================
// h - the latched flags
// ===========================================================================
void th_flags() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    Rng::clear_seed_error();
    Rng::clear_clock_error();
    const uint32_t before = Rng::status();

    // A ONE written into each flag, the rest of the register as it reads.
    Rng::regs().SR = Rng::status() | rng_seis;
    const uint32_t after_seis = Rng::status();
    Rng::regs().SR = Rng::status() | rng_ceis;
    const uint32_t after_ceis = Rng::status();
    const bool seis_took = (after_seis & rng_seis) != 0u;
    const bool ceis_took = (after_ceis & rng_ceis) != 0u;
    // The current statuses are read-only: nothing written starts an error.
    const bool current_clean = (after_ceis & (rng_secs | rng_cecs)) == 0u;
    const bool words_flow = Rng::read_blocking().has_value();

    // The two reserved "RW" bits [4:3]: what a store keeps of them.
    constexpr uint32_t reserved = 0x18u;
    Rng::regs().SR = Rng::status() | reserved;
    const uint32_t after_reserved = Rng::status();

    Rng::clear_seed_error();
    Rng::clear_clock_error();
    const uint32_t after_zero = Rng::status();
    print(serial, "  RNG_SR ", hex(before), "; a one written into SEIS reads ", hex(after_seis),
          ", into CEIS ", hex(after_ceis), ", into [4:3] ", hex(after_reserved),
          "; both flags written zero ", hex(after_zero), crlf);
    print(serial, "  SEIS ", seis_took ? "TAKES a one" : "ignores a one", ", CEIS ",
          ceis_took ? "TAKES a one" : "ignores a one", crlf);
    bench.verdict("a zero written into each latched flag leaves it down",
                  (after_zero & rng_latched) == 0u);
    bench.verdict("a ONE written into either sets neither and starts no error - so the "
                  "all-ones clearing store loses no flag, and no program can raise one",
                  !seis_took && !ceis_took && current_clean && words_flow);
    bench.verdict("and the reserved bits [4:3] keep nothing written into them",
                  (after_reserved & reserved) == 0u);
}

// ===========================================================================
// i - the vector
// ===========================================================================
void ti_vector() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    rng_irqs = 0;
    rng_words = 0;
    rng_seed_events = 0;
    rng_clock_events = 0;
    rng_irq_limit = 1000;
    Rng::interrupt(true);
    Pfic::enable(Rng::irq);
    const uint32_t start = Ticker::millis();
    while (rng_irqs < 1000u && Ticker::millis() - start < 50u) {
    }
    const uint32_t ms = Ticker::millis() - start;
    Rng::interrupt(false);
    Pfic::clear_pending(Rng::irq);
    const uint32_t irqs = rng_irqs;
    const uint32_t words = rng_words;
    print(serial, "  IE armed: ", irqs, " entries and ", words, " words taken in ", ms,
          " ms, ", rng_seed_events, " seed and ", rng_clock_events, " clock events", crlf);
    bench.verdict("the vector runs for a ready word and its body takes one at every entry - "
                  "the handler stops the storm itself at a thousand",
                  irqs >= 1000u && words == irqs && rng_seed_events == 0u &&
                      rng_clock_events == 0u);

    // THE SEED ERROR THROUGH THE VECTOR, where a one written into SEIS
    // would raise it (letter h says it does not): the body reports it and
    // clears it in the same store.
    rng_irqs = 0;
    rng_seed_events = 0;
    rng_irq_limit = 4;
    Rng::regs().SR = Rng::status() | rng_seis;
    const bool raised = Rng::seed_error_flag();
    Rng::interrupt(true);
    const uint32_t start2 = Ticker::millis();
    while (rng_irqs < 4u && Ticker::millis() - start2 < 5u) {
    }
    Rng::interrupt(false);
    Pfic::disable(Rng::irq);
    Pfic::clear_pending(Rng::irq);
    const bool cleared = !Rng::seed_error_flag();
    Rng::clear_seed_error();
    print(serial, "  a one written into SEIS: ", raised ? "the flag stood" : "nothing latched",
          "; the vector counted ", rng_seed_events, " seed event(s), the flag ",
          cleared ? "down" : "STILL UP", crlf);
    if (raised) {
        bench.verdict("the vector reports the seed flag software raised, and its body clears "
                      "it", rng_seed_events >= 1u && cleared);
    } else {
        bench.verdict("SEIS takes no one, so no program reaches the seed-error vector - "
                      "the path is compiled and cannot be entered from here",
                      rng_seed_events == 0u);
    }
}

// ===========================================================================
// j - the recovery sequence, the disable and the gate
// ===========================================================================
void tj_recover() {
    if (!running()) {
        bench.verdict("the generator runs", false);
        return;
    }
    // WHAT A DISABLE FREEZES: a running generator left alone with a word
    // standing, RNGEN cleared, RNG_DR read, and DRDY watched with the
    // generator off; then the same with RNG_DR emptied just before the
    // disable, so that the next word is being computed when it comes. A
    // word that landed now would come after the read the restart relies
    // on to make the enable's zero the first word.
    uint32_t late = 0;
    uint32_t late_empty = 0;
    for (uint32_t i = 0; i < 256u; ++i) {
        (void)Rng::read_blocking();
        uint32_t wait = 0;
        while (wait < 2'000u) {
            ++wait;
        }
        bool landed = false;
        {
            InterruptGuard guard;
            Rng::enable(false);
            (void)Rng::value();
            uint32_t polls = 0;
            while (!Rng::ready() && polls < 200u) {
                ++polls;
            }
            landed = Rng::ready();
        }
        late += landed ? 1u : 0u;
        Rng::enable(true);
        (void)Rng::read_blocking();
        {
            InterruptGuard guard;
            (void)Rng::value();
            Rng::enable(false);
            (void)Rng::value();
            uint32_t polls = 0;
            while (!Rng::ready() && polls < 200u) {
                ++polls;
            }
            landed = Rng::ready();
        }
        late_empty += landed ? 1u : 0u;
        Rng::enable(true);
    }
    (void)Rng::init();
    print(serial, "  RNGEN cleared and RNG_DR read, 256 times over a standing word and 256 over "
          "one being computed: a word landed after the read ", late, " and ", late_empty,
          " times", crlf);
    bench.verdict("RNGEN cleared freezes the generator: no word lands after the read that "
                  "follows the disable, whether a word stood or was being computed",
                  late == 0u && late_empty == 0u);

    // 29.2.2's sequence on a healthy generator, over and over: harmless,
    // and the only form of it a program can stage.
    uint32_t recovered = 0;
    uint32_t came = 0;
    uint32_t zeros = 0;
    for (uint32_t i = 0; i < 256u; ++i) {
        recovered += Rng::recover() ? 1u : 0u;
        const auto v = Rng::read_blocking();
        if (v) {
            ++came;
            zeros += *v == 0u ? 1u : 0u;
        }
    }
    print(serial, "  recover() 256 times: ", recovered, " ran, ", came,
          " first words came after them, ", zeros, " of those zero", crlf);
    bench.verdict("the seed-error recovery - SEIS cleared, RNGEN cleared and set, two words "
                  "discarded - leaves a generator that hands out words, and not once is the "
                  "first of them the enable's zero", recovered == 256u && came == 256u &&
                                                     zeros == 0u);

    // A WORD LEFT STANDING ACROSS A DISABLE, then init(): the stale word
    // and the enable's zero are both kept from the caller.
    uint32_t stale = 0;
    uint32_t restarted = 0;
    uint32_t first_came = 0;
    uint32_t first_zero = 0;
    for (uint32_t i = 0; i < 64u; ++i) {
        (void)Rng::read_blocking();
        uint32_t polls = 0;
        while (!Rng::ready() && polls < Rng::spins) {
            ++polls;
        }
        Rng::enable(false);
        stale += Rng::ready() ? 1u : 0u;
        restarted += Rng::init() ? 1u : 0u;
        const auto first = Rng::read_blocking();
        if (first) {
            ++first_came;
            first_zero += *first == 0u ? 1u : 0u;
        }
    }
    print(serial, "  a word standing across a disable, then init(), 64 times: DRDY stood ",
          stale, " times, init() ran ", restarted, ", ", first_came, " first words, ",
          first_zero, " of them zero", crlf);
    bench.verdict("init() over a word left standing by a disable restarts the generator and "
                  "hands out neither that word's successor zero nor the stale one",
                  stale == 64u && restarted == 64u && first_came == 64u && first_zero == 0u);

    // A word computed before the disable.
    (void)Rng::read_blocking();
    uint32_t polls = 0;
    while (!Rng::ready() && polls < Rng::spins) {
        ++polls;
    }
    Rng::enable(false);
    const bool stood = Rng::ready();
    const uint32_t kept = Rng::value();
    uint32_t more = 0;
    for (uint32_t i = 0; i < 10'000u; ++i) {
        if (Rng::ready()) {
            ++more;
            (void)Rng::value();
        }
    }
    print(serial, "  with RNGEN cleared over a ready word: DRDY ", stood ? "stood" : "fell",
          ", the word read ", hex(kept), ", ", more, " words after it", crlf);
    bench.verdict("with RNGEN cleared the word that stood stays readable and no new word "
                  "comes", stood && more == 0u);

    // The gate shut: what the registers answer then.
    Rng::release();
    const bool shut = !Rng::clock();
    const uint32_t cr = Rng::regs().CR;
    const uint32_t sr = Rng::status();
    print(serial, "  release(): the gate ", shut ? "shut" : "OPEN", ", RNG_CR reads ", hex(cr),
          ", RNG_SR ", hex(sr), " behind it", crlf);
    const bool again = Rng::init();
    bench.verdict("release() shuts the gate, and init() brings the block back from there",
                  shut && again);
}

void banner() {
    print(serial, crlf, "test_vx03_rng on ", device::part_name,
          " - the random number generator (RM ch. 29)", crlf,
          "  z costs the board nothing; c moves the clock tree and puts it back", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The generator's vector: the three sources reported, the latched ones
/// cleared by the body, the word taken so DRDY goes down - and the storm
/// stopped by the handler itself once the letter's count is reached.
extern "C" BRIO_CH32_INTERRUPT void rng_handler() {
    const brio::RngEvent e = brio::Rng::isr();
    if (e.ready) {
        rng_last = brio::Rng::value();
        rng_words = rng_words + 1u;
    }
    if (e.seed_error) {
        rng_seed_events = rng_seed_events + 1u;
    }
    if (e.clock_error) {
        rng_clock_events = rng_clock_events + 1u;
    }
    const uint32_t n = rng_irqs + 1u;
    rng_irqs = n;
    if (n >= rng_irq_limit) {
        brio::Rng::interrupt(false);
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block: the gate, the enable's first word, the discard, the vector",
                 ta_block);
    bench.letter('b', "the word: DRDY's cycle, the words a second, the unchanged ones", tb_word);
    bench.letter('c', "the clock: three rates, the USB prescaler, the bare HSI", tc_clock);
    bench.letter('d', "the small set: distinct words among 4096, the 32 lanes", td_small_set);
    bench.letter('e', "FIPS PUB 140-2's four tests over 20000 bits", te_fips);
    bench.letter('f', "a chi-square over 65536 bytes", tf_chi_square);
    bench.letter('g', "the continuous test over a hundred thousand words", tg_continuous);
    bench.letter('h', "the latched flags: a zero clears them, a one sets neither", th_flags);
    bench.letter('i', "the vector, and the seed flag through it", ti_vector);
    bench.letter('j', "the recovery sequence, a stale word, the disable and the gate",
                 tj_recover);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED", " tick=",
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched",
                    brio::crlf);
        bench.prompt();
    }
}
