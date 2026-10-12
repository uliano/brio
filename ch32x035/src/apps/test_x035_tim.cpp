// test_x035_tim - the reference bench suite for the CH32X035's advanced-
// control timers (RM ch. 12) in the subset brio/ch32x035/tim.hpp drives:
// the time base and its shadows, the outputs in every OCxM mode, the
// complementary outputs behind MOE, one-pulse mode, the compare's DMA
// request and the remap - every verdict read off a pad or a register,
// with NO WIRE of its own.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// THE PADS. TIM1 under its code 001 (8.3.2.1) puts CH3 on PB11 - a header
// pin and nothing else - and its complementary outputs C2N and C3N on PB0
// and PB1: on the desk those two are the UART wires to the CH32V006, so
// the suite runs with that board DARK (its console, or its usart_peer
// searching: both leave PB0 and PB1 floating inputs there). A pad driven
// by its alternate function reads its own level on INDR, which is the
// instrument: a level is read, and an edge is stamped on the core's STK
// counter (HCLK) by a loop polling the port, interrupts masked.
//
// What is exercised, letter by letter:
//   a  the block: every register's reset value on both timers - CTLR1
//      reading 0x2000, its "reserved" bit 13 set, and DMAADR (the burst's
//      window onto CTLR1) with it - bit 13 writable both ways and kept by
//      configure(), the prescaler taken at the update and not before,
//      INTFR written zero to clear, a software compare event raising
//      nothing on an output channel, the remap field written and refused
//      beyond its codes
//   b  the levels: a timer that never ran driving its output low, the
//      complementary output alone following OCxREF on a running counter,
//      the HOLD while the counter is stopped, CCxNP, MOE letting the output
//      go (floating) and - bit 13 cleared - driving the idle level OIS2N,
//      CCxE and CCxNE together as complements
//   c  the toggle: a free-running toggle's half period on the pad, AN
//      EDGE LIST streamed by DMA into CH2CVR (channel 3) and CH3CVR
//      (channel 6) on each compare's own request - every interval on the
//      pad against the counts asked - and the shortest interval the
//      channel keeps
//   d  one pulse: OPM's pulse width on the pad and CEN cleared by the
//      update; active and inactive on match, frozen both ways, toggle,
//      PWM mode 2
//   e  the DMA requests: CCxDE on each channel of both timers and UDE,
//      each match or update moving one item on the channel figure 9-1
//      names, and a software update raising the update's request
//
// THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
// that drives a link pad from a sagging rail refuses every letter.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/tim.hpp"
#include "ch32x035/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;

TestBench<Serial, 8> bench;
bool supply_ok = true;

using T1 = Tim<1>;
using T2 = Tim<2>;
constexpr uint8_t code = 1;   ///< TIM1_RM 001: CH3 PB11, C2N PB0, C3N PB1
using Ch3Pad = TimPad<afio_tim1_pads(code).ch3>;
using C2nPad = TimPad<afio_tim1_pads(code).ch2n>;
using C3nPad = TimPad<afio_tim1_pads(code).ch3n>;
static_assert(Ch3Pad::selection == Pad{'B', 11} && C2nPad::selection == Pad{'B', 0} &&
              C3nPad::selection == Pad{'B', 1});
constexpr uint32_t pb11 = 1UL << 11;
constexpr uint32_t pb0 = 1UL << 0;
constexpr uint32_t pb1 = 1UL << 1;

bool pad(uint32_t mask) { return (Port<'B'>::in() & mask) != 0u; }

/// Everything back: both timers reset and gated off, the remap at its
/// default, the three pads floating inputs.
void all_off() {
    T1::release();
    T2::release();
    (void)T1::remap(0);
    Ch3Pad::release();
    C2nPad::release();
    C3nPad::release();
}

void settle() { (void)delay_us(clock, 5); }

// ---- time ----------------------------------------------------------------------

/// STK's raw count: HCLK cycles, reloading at the tick's compare (every
/// millisecond), so a difference under a millisecond unwraps exactly.
uint32_t stk_now() { return stk()->CNTL; }
uint32_t stk_since(uint32_t from, uint32_t to) {
    return to >= from ? to - from : to + stk()->CMPLR + 1u - from;
}

bool near(int32_t a, int32_t b, int32_t tol) { return a - b <= tol && b - a <= tol; }

/// THE SAMPLER: port B's input register read `n` times back to back into
/// a buffer, four reads a turn of the loop, interrupts masked, the whole
/// run timed on STK - one sample every `cycles / n` cycles (about
/// sixteen: each read crosses to the peripheral bus), so an edge is placed
/// to one sample. The run must stay under STK's period (a millisecond),
/// which reloads with the tick masked: 2048 samples do. Aligned so the
/// loop's fetch does not move with the code around it
/// (docs/ch32x035/spi.md).
constexpr uint16_t samples_most = 2048;
uint16_t samples[samples_most];

struct Sampled {
    uint32_t cycles = 0;   ///< the run's length on STK
    uint16_t n = 0;
};

[[gnu::noinline, gnu::aligned(32)]] Sampled sample_port_b(uint16_t n) {
    volatile uint32_t* const indr = &Port<'B'>::regs().INDR;
    InterruptGuard guard;
    const uint32_t s0 = stk_now();
    for (uint16_t i = 0; i < n; i = static_cast<uint16_t>(i + 4u)) {
        samples[i] = static_cast<uint16_t>(*indr);
        samples[i + 1u] = static_cast<uint16_t>(*indr);
        samples[i + 2u] = static_cast<uint16_t>(*indr);
        samples[i + 3u] = static_cast<uint16_t>(*indr);
    }
    const uint32_t s1 = stk_now();
    return Sampled{stk_since(s0, s1), n};
}

/// The edges of one pad in the last sampled run, as sample indices.
uint8_t edges_of(const Sampled& r, uint16_t mask, uint16_t* at, uint8_t most) {
    uint8_t k = 0;
    for (uint16_t i = 1; i < r.n && k < most; ++i) {
        if (((samples[i] ^ samples[i - 1u]) & mask) != 0u) {
            at[k++] = i;
        }
    }
    return k;
}

/// A sample count in cycles: index x run / n, rounded.
uint32_t cycles_of(const Sampled& r, uint32_t idx) {
    return static_cast<uint32_t>((static_cast<uint64_t>(idx) * r.cycles + r.n / 2u) / r.n);
}

/// The counter running at PSC 47, far from any compare: the state in which
/// a forced level reaches the pad (the outputs hold while it is stopped).
void run_far() {
    (void)T1::configure({.prescaler = 47, .period = 0xFFFF});
    T1::enable(true);
}

// ---------------------------------------------------------------------------
// a - the block
// ---------------------------------------------------------------------------
template <typename T>
bool reset_values() {
    T::init();
    TimRegs& r = T::regs();
    const bool zero = r.CTLR2 == 0u && r.SMCFGR == 0u && r.DMAINTENR == 0u && r.INTFR == 0u &&
                      r.CHCTLR1 == 0u && r.CHCTLR2 == 0u && r.CCER == 0u && r.CNT == 0u && r.PSC == 0u &&
                      r.RPTCR == 0u && r.CHCVR[0] == 0u && r.CHCVR[1] == 0u && r.CHCVR[2] == 0u &&
                      r.CHCVR[3] == 0u && r.BDTR == 0u && r.DMACFGR == 0u && r.SPEC == 0u;
    print(serial, "  TIM", T::instance, " after its reset: CTLR1 ", hex(r.CTLR1), " ATRLR ", hex(r.ATRLR), crlf);
    if (!zero) {
        const volatile uint16_t* w = &r.CTLR1;
        print(serial, "   ");
        for (uint8_t i = 0; i < sizeof(TimRegs) / 2u; i += 2u) {
            if (w[i] != 0u) {
                print(serial, " +", hex(static_cast<uint32_t>(2u * i)), "=", hex(w[i]));
            }
        }
        print(serial, crlf);
    }
    // DMAADR is the burst's window onto the register DBA names - CTLR1 at
    // DBA's reset value - so it reads what CTLR1 reads.
    return zero && r.ATRLR == 0xFFFFu && r.CTLR1 == tim_ctlr1_reserved13 && r.DMAADR == r.CTLR1;
}

/// HCLK cycles a count, timed on STK over about 100 us.
uint32_t cycles_per_count_x100() {
    const uint16_t c0 = T1::count();
    const uint32_t s0 = stk_now();
    while (stk_since(s0, stk_now()) < 4800u) {
    }
    const uint16_t c1 = T1::count();
    const uint32_t s1 = stk_now();
    const uint16_t d = static_cast<uint16_t>(c1 - c0);
    return d == 0u ? 0u : (100u * stk_since(s0, s1)) / d;
}

void ta_block() {
    all_off();
    // MEASURED: CTLR1 resets to 0x2000 - 12.4.1 calls bit 13 reserved and
    // its table says zero.
    bench.verdict("TIM1 at its reset: every register the table's but CTLR1, which reads 0x2000 (bit 13 set), and "
                  "DMAADR, the burst's window, which reads it too",
                  reset_values<T1>());
    bench.verdict("TIM2 the same", reset_values<T2>());
    T1::regs().CTLR1 = 0;
    const uint16_t cleared = T1::regs().CTLR1;
    T1::regs().CTLR1 = tim_ctlr1_reserved13;
    const uint16_t set = T1::regs().CTLR1;
    (void)T1::configure({.prescaler = 3});
    const uint16_t kept = T1::regs().CTLR1;
    print(serial, "  bit 13 written 0 reads ", hex(cleared), ", written 1 reads ", hex(set),
          "; after configure() ", hex(kept), crlf);
    bench.verdict("bit 13 is writable both ways, and configure() keeps it as it reads",
                  cleared == 0u && set == tim_ctlr1_reserved13 && (kept & tim_ctlr1_reserved13) != 0u);

    // The prescaler's shadow: PSC 47, then PSC 0 written with no update -
    // the count goes on at 48 cycles a count - then an update takes it.
    (void)T1::configure({.prescaler = 47, .period = 0xFFFF});
    T1::enable(true);
    const uint32_t at_47 = cycles_per_count_x100();
    T1::set_prescaler(0);
    const uint32_t before_update = cycles_per_count_x100();
    T1::update();
    const uint32_t after_update = cycles_per_count_x100();
    T1::enable(false);
    print(serial, "  HCLK cycles a count x100: PSC 47 ", at_47, ", PSC 0 written ", before_update, ", after UG ",
          after_update, crlf);
    bench.verdict("the counter at PSC 47 counts one every 48 cycles", near(static_cast<int32_t>(at_47), 4800, 48));
    bench.verdict("PSC written alone is not taken before an update (12.2.3)",
                  near(static_cast<int32_t>(before_update), 4800, 48));
    bench.verdict("and an update (UG) takes it: one count a cycle", near(static_cast<int32_t>(after_update), 100, 2));

    // INTFR: write zero to clear.
    T1::update();
    const bool raised = T1::flag(T1::update_flag);
    T1::regs().INTFR = 0xFFFFu;
    const bool kept_ones = T1::flag(T1::update_flag);
    T1::clear_flags(T1::compare_flag(0));
    const bool kept_other = T1::flag(T1::update_flag);
    T1::clear_flags(T1::update_flag);
    const bool clear = !T1::flag(T1::update_flag);
    bench.verdict("UG raises UIF; ones written keep it, another flag's clear keeps it, its own zero clears it",
                  raised && kept_ones && kept_other && clear);

    // A software compare event on an output channel: MEASURED, nothing.
    T1::init();
    (void)T1::output_channel(1, {.mode = TimOutputMode::frozen, .compare = 50, .preload = false, .enable = false});
    T1::regs().SWEVGR = static_cast<uint16_t>(tim_cc1g << 1);
    const bool stopped_flag = T1::flag(T1::compare_flag(1));
    T1::regs().CHCVR[1] = 60000u;
    T1::enable(true);
    T1::regs().SWEVGR = static_cast<uint16_t>(tim_cc1g << 1);
    const bool running_flag = T1::flag(T1::compare_flag(1));
    T1::enable(false);
    print(serial, "  CC2G on an output channel: CC2IF ", stopped_flag, " stopped, ", running_flag, " running", crlf);
    bench.verdict("CCxG on an output channel raises no CCxIF, counter stopped or running (12.4.6 says it does)",
                  !stopped_flag && !running_flag);

    // The remap field.
    const bool wrote = T1::remap(code) && T1::remap() == code && Afio::remap_code(Remap::tim1) == code;
    const bool refused = !T1::remap(8) && T1::remap() == code;
    const bool tim2 = T2::remap(2) && T2::remap() == 2u && T2::remap(0);
    bench.verdict("TIM1_RM written 001 reads 001; a code beyond the field is refused and writes nothing",
                  wrote && refused);
    bench.verdict("TIM2_RM written and read back", tim2);
    all_off();
}

// ---------------------------------------------------------------------------
// b - the levels
// ---------------------------------------------------------------------------
void tb_levels() {
    all_off();
    T1::init();
    (void)T1::remap(code);
    // THE NEVER-RUN TIMER: CH2 forced active, C2NE alone, MOE, the pad
    // pre-charged HIGH as a GPIO first - the counter never started.
    (void)T1::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    T1::main_output(true);
    Pin<'B', 0>::output(true);
    settle();
    const bool claimed = C2nPad::claim();
    settle();
    const bool never_run = pad(pb0);
    print(serial, "  C2N forced active, the counter never started, the pad pre-charged high: PB0 ",
          never_run ? "HIGH" : "LOW", crlf);
    bench.verdict("a timer that never ran DRIVES its output low: a forced level waits for the counter",
                  claimed && !never_run);
    run_far();
    settle();
    const bool running_high = pad(pb0);
    (void)T1::output_mode(1, TimOutputMode::force_inactive);
    settle();
    const bool running_low = !pad(pb0);
    bench.verdict("the counter running, the complementary output ALONE is OCxREF: forced active high, "
                  "forced inactive low", running_high && running_low);
    // THE HOLD: high, the counter stopped, a forced low written meanwhile.
    (void)T1::output_mode(1, TimOutputMode::force_active);
    settle();
    T1::enable(false);
    (void)delay_us(clock, 500);
    const bool held = pad(pb0);
    (void)T1::output_mode(1, TimOutputMode::force_inactive);
    settle();
    const bool held_through = pad(pb0);
    T1::enable(true);
    settle();
    const bool taken = !pad(pb0);
    bench.verdict("the counter stopped, the output HOLDS its level - 500 us, and through a forced level written "
                  "meanwhile, which the running counter then takes", held && held_through && taken);
    // CC2NP inverts it.
    (void)T1::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .complementary_active_low = true,
                                 .enable = false, .complementary_enable = true});
    settle();
    const bool inverted = !pad(pb0);
    bench.verdict("CC2NP set: forced active drives C2N low", inverted);
    // MOE, the gate, on a running counter: cleared, the output is LET GO -
    // a level charged onto the pad through the GPIO stays there, either
    // way - and set, OCxREF is back. Then CTLR1's bit 13 (reserved in
    // 12.4.1, one out of reset) cleared: MOE clear DRIVES the idle level
    // OIS2N, written into CTLR2 by hand (the brake unit is not driven).
    auto charged = [](bool level) {
        Pin<'B', 0>::output(level);
        (void)delay_us(clock, 20);
        (void)C2nPad::claim();
        (void)delay_us(clock, 20);
        return pad(pb0);
    };
    run_far();
    (void)T1::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    settle();
    const bool gate_open = pad(pb0);
    T1::main_output(false);
    settle();
    const bool float_low = !charged(false);
    const bool float_high = charged(true);
    T1::main_output(true);
    settle();
    const bool reopened = pad(pb0);
    print(serial, "  MOE clear: C2N keeps a charge of either level (", float_low && float_high ? "floats" : "DRIVEN",
          "); set: ", reopened ? "high" : "low", " again", crlf);
    bench.verdict("MOE clear lets C2N go - the pad floats - and MOE set gives OCxREF back", gate_open && float_low &&
                                                                                           float_high && reopened);
    bool drives = true;
    for (const bool ois : {true, false}) {
        T1::regs().CTLR1 = static_cast<uint16_t>(T1::regs().CTLR1 & ~tim_ctlr1_reserved13);
        T1::regs().CTLR2 = static_cast<uint16_t>(ois ? (2u << 10) : 0u);   // OIS2N
        T1::main_output(false);
        settle();
        drives = drives && charged(!ois) == ois;
        T1::main_output(true);
    }
    T1::regs().CTLR1 = static_cast<uint16_t>(T1::regs().CTLR1 | tim_ctlr1_reserved13);
    T1::regs().CTLR2 = 0;
    bench.verdict("CTLR1's bit 13 cleared: MOE clear DRIVES C2N at its idle level OIS2N, against a charge of the "
                  "other level, both ways", drives);
    // CH3 on PB11 and C3N on PB1, alone and together.
    (void)Ch3Pad::claim();
    (void)C3nPad::claim();
    (void)T1::output_channel(2, {.mode = TimOutputMode::force_active, .preload = false, .enable = true});
    settle();
    const bool ch3_alone = pad(pb11);
    (void)T1::output_channel(2, {.mode = TimOutputMode::force_active, .preload = false, .enable = true,
                                 .complementary_enable = true});
    settle();
    const bool pair_active = pad(pb11) && !pad(pb1);
    (void)T1::output_mode(2, TimOutputMode::force_inactive);
    settle();
    const bool pair_inactive = !pad(pb11) && pad(pb1);
    T1::enable(false);
    bench.verdict("CH3 alone on PB11 follows a forced active level", ch3_alone);
    bench.verdict("CC3E and CC3NE together: C3N the complement of CH3 at both forced levels",
                  pair_active && pair_inactive);
    all_off();
}

// ---------------------------------------------------------------------------
// c - the toggle and the edge list
// ---------------------------------------------------------------------------
using Ch2Edges = DmaTxEngine<T1::compare_dma_channel(1), uint32_t>;
using Ch3Edges = DmaTxEngine<T1::compare_dma_channel(2), uint32_t>;
static_assert(Ch2Edges::channel == 3 && Ch3Edges::channel == 6, "figure 9-1: TIM1's CH2 on 3, CH3 on 6");

/// Start an edge list of compare values (counts from zero) on channel
/// `ch` through engine E at PSC `psc`: the first value stored by hand,
/// the rest - and an end marker half the counter beyond, which no match
/// reaches - streamed on the channel's request. The counter left running.
template <typename E>
void start_list(uint8_t ch, const uint32_t* list, uint8_t n, uint16_t psc) {
    static uint32_t marks[17];
    for (uint8_t i = 0; i < n; ++i) {
        marks[i] = list[i];
    }
    marks[n] = (list[n - 1u] + 0x8000u) & 0xFFFFu;
    // The level forced HIGH on a running counter, then the list.
    run_far();
    (void)T1::output_mode(ch, TimOutputMode::force_active);
    T1::enable(false);
    (void)T1::configure({.prescaler = psc, .period = 0xFFFF});
    E::arm(T1::ccr_address(ch), 0u, DmaPriority::very_high);
    (void)E::start(std::span<const uint32_t>(&marks[1], n));
    (void)T1::set_compare(ch, static_cast<uint16_t>(marks[0]));
    (void)T1::output_mode(ch, TimOutputMode::toggle);
    T1::clear_flags(T1::compare_flag(ch));
    T1::interrupts(T1::compare_dma(ch), true);
    T1::enable(true);
}

template <typename E>
void stop_list(uint8_t ch) {
    T1::interrupts(T1::compare_dma(ch), false);
    (void)E::abandon();
    T1::enable(false);
}

/// The worst |interval - asked| of the list on the pad, in cycles; -1 when
/// an edge is missing. The intervals measured are printed.
int32_t judge_list(const Sampled& r, uint16_t mask, const uint32_t* list, uint8_t n, uint16_t psc) {
    uint16_t at[20] = {};
    const uint8_t k = edges_of(r, mask, at, 20);
    print(serial, "    ", k, " edges, a sample every ", r.cycles * 100u / r.n, " hundredths of a cycle; the intervals:");
    if (k != n) {
        print(serial, " (", n, " expected)", crlf);
        return -1;
    }
    int32_t worst = 0;
    for (uint8_t i = 1; i < n; ++i) {
        const int32_t d = static_cast<int32_t>(cycles_of(r, at[i]) - cycles_of(r, at[i - 1u]));
        const int32_t asked = static_cast<int32_t>((list[i] - list[i - 1u]) * (psc + 1u));
        print(serial, " ", d);
        const int32_t err = d > asked ? d - asked : asked - d;
        worst = err > worst ? err : worst;
    }
    print(serial, crlf);
    return worst;
}

void tc_toggle() {
    all_off();
    T1::init();
    (void)T1::remap(code);
    Dma::open();
    (void)T1::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    (void)T1::output_channel(2, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    T1::main_output(true);
    (void)C2nPad::claim();
    (void)C3nPad::claim();
    // A free-running toggle: PSC 0, period 999, compare 500 - one toggle a
    // period, 1000 cycles between edges.
    (void)T1::configure({.prescaler = 0, .period = 999});
    (void)T1::set_compare(1, 500);
    (void)T1::output_mode(1, TimOutputMode::toggle);
    T1::enable(true);
    const Sampled r0 = sample_port_b(samples_most);
    T1::enable(false);
    uint16_t at[12] = {};
    const uint8_t k = edges_of(r0, static_cast<uint16_t>(pb0), at, 12);
    const uint32_t span = k >= 2u ? cycles_of(r0, at[k - 1u]) - cycles_of(r0, at[0]) : 0u;
    const uint32_t per = k >= 2u ? span / (k - 1u) : 0u;
    print(serial, "  the free-running toggle: ", k, " edges, ", per, " cycles apart (1000 asked), a sample every ",
          r0.cycles * 100u / r0.n, " hundredths of a cycle", crlf);
    bench.verdict("a toggle at each match: an edge every period, 1000 cycles within 0.5 %",
                  k >= 5u && near(static_cast<int32_t>(per), 1000, 5));

    // The edge list at PSC 2 (three cycles a count), on C2N (DMA channel 3)
    // and C3N (channel 6).
    static constexpr uint32_t list[] = {100, 300, 364, 864, 1864, 1900, 2400, 4400, 4500, 5000};
    constexpr uint8_t n = sizeof(list) / sizeof(list[0]);
    start_list<Ch2Edges>(1, list, n, 2);
    const Sampled r2 = sample_port_b(samples_most);
    stop_list<Ch2Edges>(1);
    const int32_t worst2 = judge_list(r2, static_cast<uint16_t>(pb0), list, n, 2);
    // One sample is the instrument's grain: an edge is placed to it, an
    // interval to two.
    const int32_t grain = static_cast<int32_t>(2u * r2.cycles / r2.n + 1u);
    print(serial, "  C2N's list: worst ", worst2, " cycles off (the sampler's grain ", grain, ")", crlf);
    bench.verdict("C2N's edge list streamed by DMA channel 3 on CH2's request: every interval as asked, to the "
                  "sampler's grain", worst2 >= 0 && worst2 <= grain);
    start_list<Ch3Edges>(2, list, n, 2);
    const Sampled r3 = sample_port_b(samples_most);
    stop_list<Ch3Edges>(2);
    const int32_t worst3 = judge_list(r3, static_cast<uint16_t>(pb1), list, n, 2);
    bench.verdict("C3N's on DMA channel 6 on CH3's request, the same", worst3 >= 0 && worst3 <= grain);

    // The engine's edge: lists whose intervals shrink, at PSC 0 - the
    // smallest interval at which every value of the list is taken.
    uint8_t keeps = 0;
    for (const uint8_t gap : {24u, 16u, 12u, 10u, 8u, 6u, 4u}) {
        uint32_t tight[8];
        for (uint8_t i = 0; i < 8u; ++i) {
            tight[i] = 200u + static_cast<uint32_t>(i) * gap;
        }
        start_list<Ch2Edges>(1, tight, 8, 0);
        while (T1::count() < 2000u) {
        }
        const uint16_t left = DmaChannel<Ch2Edges::channel>::count();
        stop_list<Ch2Edges>(1);
        print(serial, "  intervals of ", gap, " cycles: ", 8u - left, " of 8 values taken", crlf);
        if (left == 0u) {
            keeps = gap;
        }
    }
    print(serial, "  the smallest interval the channel keeps: ", keeps, " HCLK cycles", crlf);
    bench.verdict("the channel keeps up with intervals of 12 HCLK cycles and less (every value of the list taken)",
                  keeps != 0u && keeps <= 12u);
    all_off();
}

// ---------------------------------------------------------------------------
// d - one pulse, and the match modes
// ---------------------------------------------------------------------------
void td_one_pulse() {
    all_off();
    T1::init();
    (void)T1::remap(code);
    (void)T1::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    T1::main_output(true);
    (void)C2nPad::claim();
    run_far();
    // OPM + PWM mode 1: high while the count is under the compare, low from
    // it to the reload, the counter stopping itself at the update - and
    // HOLDING the level it had there.
    constexpr uint16_t delay_counts = 100;
    constexpr uint16_t width = 2400;
    T1::enable(false);
    (void)T1::configure({.prescaler = 0, .period = static_cast<uint16_t>(delay_counts + width - 1u), .one_pulse = true});
    (void)T1::set_compare(1, delay_counts);
    (void)T1::output_mode(1, TimOutputMode::pwm1);
    T1::enable(true);
    const Sampled r = sample_port_b(samples_most);
    const bool stopped = !T1::enabled();
    uint16_t at[4] = {};
    const uint8_t k = edges_of(r, static_cast<uint16_t>(pb0), at, 4);
    const uint32_t w = k >= 2u ? cycles_of(r, at[1]) - cycles_of(r, at[0]) : 0u;
    const int32_t grain = static_cast<int32_t>(2u * r.cycles / r.n + 1u);
    print(serial, "  one pulse: ", k, " edges, the low run ", w, " cycles (", width, " asked), CEN after it ",
          stopped ? "clear" : "SET", ", the pad after it ", pad(pb0) ? "high" : "low", crlf);
    bench.verdict("PWM mode 1 under OPM: one low pulse of the counts asked, to the sampler's grain",
                  k == 2u && near(static_cast<int32_t>(w), width, grain));
    bench.verdict("and the update stops the counter (CEN cleared by the hardware, 12.4.1)", stopped);

    // The match modes, each from a forced level on a running counter.
    struct ModeCase {
        TimOutputMode mode;
        bool from;
        bool expect;
        const char* name;
    };
    static constexpr ModeCase cases[] = {
        {TimOutputMode::active_on_match, false, true, "active on match: low, then high at the match"},
        {TimOutputMode::inactive_on_match, true, false, "inactive on match: high, then low at the match"},
        {TimOutputMode::frozen, true, true, "frozen: high stays high through the match"},
        {TimOutputMode::frozen, false, false, "frozen: low stays low through the match"},
        {TimOutputMode::toggle, false, true, "toggle: low, then high at the match"},
    };
    for (const ModeCase& mc : cases) {
        run_far();
        (void)T1::set_compare(1, 60000u);
        (void)T1::output_mode(1, mc.from ? TimOutputMode::force_active : TimOutputMode::force_inactive);
        settle();
        const bool before = pad(pb0);
        const uint16_t at_count = static_cast<uint16_t>(T1::count() + 100u);
        (void)T1::set_compare(1, at_count);
        (void)T1::output_mode(1, mc.mode);
        const bool pre_match = pad(pb0);
        while (static_cast<uint16_t>(T1::count() - at_count) > 0x8000u) {
        }
        settle();
        const bool after = pad(pb0);
        T1::enable(false);
        bench.verdict(mc.name, before == mc.from && pre_match == mc.from && after == mc.expect);
    }
    // PWM mode 2: low while the count is under the compare, high from it.
    (void)T1::configure({.prescaler = 47, .period = 999});
    (void)T1::set_compare(1, 400);
    (void)T1::output_mode(1, TimOutputMode::pwm2);
    T1::enable(true);
    while (T1::count() < 100u || T1::count() > 300u) {
    }
    const bool under = pad(pb0);
    while (T1::count() < 500u || T1::count() > 900u) {
    }
    const bool over = pad(pb0);
    T1::enable(false);
    bench.verdict("PWM mode 2: low under the compare, high from it", !under && over);
    all_off();
}

// ---------------------------------------------------------------------------
// e - the DMA requests
// ---------------------------------------------------------------------------
/// One row of figure 9-1: CCxDE (or UDE when ch is 4) on timer n, the
/// counter at PSC 0 over a period of 100 counts with the compare at 50 -
/// a match and an update each period - and three items on the table's
/// channel, none moved before the request is enabled.
template <uint8_t n, uint8_t ch>
bool dma_row() {
    using T = Tim<n>;
    constexpr uint8_t dch = ch < 4u ? T::compare_dma_channel(ch) : T::update_dma_channel;
    using E = DmaTxEngine<dch, uint32_t>;
    constexpr uint8_t target = ch < 4u ? ch : 3u;
    T::init();
    (void)T::configure({.prescaler = 0, .period = 99});
    (void)T::output_channel(target, {.mode = TimOutputMode::frozen, .compare = 50, .preload = false, .enable = false});
    E::arm(T::ccr_address(target), 0u, DmaPriority::high);
    static const uint32_t values[3] = {50, 50, 50};
    const bool started = E::start(std::span<const uint32_t>(values));
    T::enable(true);
    (void)delay_us(clock, 20);
    const uint16_t idle_left = DmaChannel<dch>::count();
    T::interrupts(ch < 4u ? T::compare_dma(ch) : T::update_dma, true);
    (void)delay_us(clock, 20);
    const uint16_t left = DmaChannel<dch>::count();
    T::interrupts(ch < 4u ? T::compare_dma(ch) : T::update_dma, false);
    T::enable(false);
    (void)E::abandon();
    T::release();
    const bool ok = started && idle_left == 3u && left == 0u;
    print(serial, "    TIM", n);
    if (ch < 4u) {
        print(serial, " CH", static_cast<uint8_t>(ch + 1u));
    } else {
        print(serial, " UP");
    }
    print(serial, " on DMA channel ", dch, ": ", 3u - idle_left, " moved before the enable, ", idle_left - left,
          " in the next 20 periods", crlf);
    return ok;
}

/// UG with UDE: does the software update raise the update's request?
template <uint8_t n>
uint16_t ug_moves() {
    using T = Tim<n>;
    constexpr uint8_t dch = T::update_dma_channel;
    using E = DmaTxEngine<dch, uint32_t>;
    T::init();
    (void)T::configure({.prescaler = 0, .period = 0xFFFF});
    E::arm(T::ccr_address(3), 0u, DmaPriority::high);
    static const uint32_t values[3] = {1, 2, 3};
    (void)E::start(std::span<const uint32_t>(values));
    T::interrupts(T::update_dma, true);
    for (uint8_t k = 0; k < 3u; ++k) {
        T::update();
        (void)delay_us(clock, 2);
    }
    const uint16_t left = DmaChannel<dch>::count();
    T::interrupts(T::update_dma, false);
    (void)E::abandon();
    T::release();
    return static_cast<uint16_t>(3u - left);
}

void te_dma() {
    all_off();
    Dma::open();
    bool t1 = dma_row<1, 0>();
    t1 = dma_row<1, 1>() && t1;
    t1 = dma_row<1, 2>() && t1;
    t1 = dma_row<1, 3>() && t1;
    t1 = dma_row<1, 4>() && t1;
    bench.verdict("TIM1: CC1..CC4 on DMA channels 2, 3, 6, 4 and the update on 5, three items for three matches",
                  t1);
    bool t2 = dma_row<2, 0>();
    t2 = dma_row<2, 1>() && t2;
    t2 = dma_row<2, 2>() && t2;
    t2 = dma_row<2, 3>() && t2;
    t2 = dma_row<2, 4>() && t2;
    bench.verdict("TIM2: CC1..CC4 on DMA channels 5, 7, 1, 7 and the update on 2, the same", t2);
    const uint16_t ug1 = ug_moves<1>();
    const uint16_t ug2 = ug_moves<2>();
    print(serial, "  three software updates (UG) with UDE moved ", ug1, " items on TIM1's channel, ", ug2,
          " on TIM2's", crlf);
    bench.verdict("a software update raises the update's DMA request: one item for each UG, both timers",
                  ug1 == 3u && ug2 == 3u);
    all_off();
}


void banner() {
    print(serial, crlf, "test_x035_tim on ", device::part_name, " - the advanced-control timers", crlf, crlf);
    print(serial, "  supply at boot: ", supply_ok ? "above 3.0 V" : "UNDER 3.0 V, every letter refused",
          " (the voltage detector at PLS 10); PB0/PB1 are the link wires: the far board dark", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    brio::Pwr::pvd(brio::PvdLevel::v3_0);
    (void)brio::delay_us(clock, 200);
    supply_ok = !brio::Pwr::supply_low();

    bench.letter('a', "the block: reset values, the reserved bit, the shadows, the flags, the remap", ta_block);
    bench.letter('b', "the levels: forced, complementary, MOE and the idle levels", tb_levels);
    bench.letter('c', "the toggle, and an edge list streamed by DMA", tc_toggle);
    bench.letter('d', "one pulse, and the match modes", td_one_pulse);
    bench.letter('e', "the DMA requests of both timers, figure 9-1", te_dma);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW", brio::crlf);
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
        } else if (!supply_ok) {
            brio::print(serial, "refused: the supply is under 3.0 V", brio::crlf);
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
