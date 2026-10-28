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
//   f  the time base's other modes, every interval between flags stamped
//      on the STK: THE REPETITION COUNTER on both timers, edge-aligned and
//      centre-aligned; CENTRE-ALIGNED counting's three compare-flag
//      policies; THE DOWN COUNTER; UDIS - configure() asked for it, no
//      update at an overflow, a PSC written meanwhile not taken, the next
//      overflow after it is cleared; URS - UG loading the shadows with no
//      UIF, an overflow still raising it
//   g  TIM2'S OUTPUTS ON A PAD: C2N on PB12 - a header pin and nothing
//      else - under TIM2_RM 101 and 110, forced levels, CC2NP and a
//      toggle read off the pad. Both codes put channel 1's CH1 and C1N on
//      the debug port (PC19, PC18): the letter never enables channel 1
//      and never claims those pads
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

/// Everything back: both timers reset and gated off, both remap fields at
/// their default, the four pads floating inputs.
void all_off() {
    T1::release();
    T2::release();
    (void)T1::remap(0);
    (void)T2::remap(0);
    Ch3Pad::release();
    C2nPad::release();
    C3nPad::release();
    Pin<'B', 12>::release();
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


// ---------------------------------------------------------------------------
// f - the time base's other modes: the repetition counter, centre-aligned
// counting, the down counter, UDIS and URS
// ---------------------------------------------------------------------------

/// THE BOUND OF EVERY INTERVAL BELOW. Both the timers and the STK count
/// HCLK, so an interval between two flags is an exact number of counter
/// clocks and what a stamp adds is its poll's LATENESS - from the flag's
/// rise to the stamp after the poll saw it: at most one whole turn of
/// wait_flag()'s loop (an INTFR read across the bus and Ticker::cycles()'s
/// five reads, from a flash behind two wait states) with whatever vector
/// lands in it - the tick's, the console being drained first - and then
/// the stamp's own read, under one plain turn. An interval is the
/// difference of two latenesses, so it is good to the largest turn plus
/// the smallest. Both are MEASURED in this image, on this very loop
/// waiting for a flag that never rises over three tick periods (a tick's
/// vector lands in at least one turn of it), not assumed: a turn's cost
/// is the layout's (docs/ch32x035/platform.md, a flash loop costs by its
/// address mod 4).
uint32_t stamp_slack_cycles = 0;

bool near_cycles(uint32_t got, uint32_t want) {
    return got + stamp_slack_cycles >= want && got <= want + stamp_slack_cycles;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);   // the last byte's own time on the wire
}

/// Wait for a flag, bounded on the STK.
template <typename T>
bool wait_flag(uint16_t flag, uint32_t limit_us) {
    const uint32_t c0 = Ticker::cycles();
    while (!T::flag(flag)) {
        if (Ticker::cycles() - c0 > limit_us * (SysClock::hz / 1'000'000u)) {
            return false;
        }
    }
    return true;
}

/// n intervals between rises of a flag, in HCLK cycles on the STK, and the
/// counter's direction read at each rise (dir[0] at the first).
template <typename T, uint8_t n>
bool flag_intervals(uint16_t flag, uint32_t (&gaps)[n], TimDirection (&dir)[n + 1u], uint32_t limit_us) {
    T::clear_flags(flag);
    if (!wait_flag<T>(flag, limit_us)) {
        return false;
    }
    uint32_t last = Ticker::cycles();
    dir[0] = T::direction();
    T::clear_flags(flag);
    for (uint8_t i = 0; i < n; ++i) {
        if (!wait_flag<T>(flag, limit_us)) {
            return false;
        }
        const uint32_t now = Ticker::cycles();
        dir[i + 1u] = T::direction();
        T::clear_flags(flag);
        gaps[i] = now - last;
        last = now;
    }
    return true;
}

constexpr uint32_t us_cycles = SysClock::hz / 1'000'000u;

/// The largest and the smallest turn of wait_flag()'s loop over three tick
/// periods, TIM1's CC4IF - a channel no letter sets - its flag: the
/// timeout's own reads are the stamps.
void measure_slack() {
    T1::init();
    T1::clear_flags(0xFFFFu);
    uint32_t most = 0;
    uint32_t least = 0xFFFF'FFFFu;
    const uint32_t c0 = Ticker::cycles();
    uint32_t last = c0;
    while (!T1::flag(T1::compare_flag(3))) {
        const uint32_t now = Ticker::cycles();
        const uint32_t step = now - last;
        last = now;
        most = step > most ? step : most;
        least = step != 0u && step < least ? step : least;
        if (now - c0 > 3u * (SysClock::hz / 1000u)) {
            break;
        }
    }
    T1::release();
    stamp_slack_cycles = most + least;
    print(serial, "  the poll's turn: ", least, " cycles, ", most, " with the tick's vector in it - every interval "
          "below good to ", stamp_slack_cycles, crlf);
}

/// THE REPETITION COUNTER (12.3.1, 12.4.13): one update in RPTCR + 1
/// periods, edge-aligned at 100 us a period, and centre-aligned, where an
/// overflow and an underflow each count (the counter climbs 0..ATRLR - 1
/// and falls ATRLR..1, 12.3.1: 100 us from an overflow to the next
/// underflow at ATRLR 100) - four intervals at each setting against
/// (RPTCR + 1) x 100 us. Centre-aligned with RPTCR even the updates fall
/// on overflows and underflows alternately, odd on one kind alone: the
/// direction at each rise says which.
template <typename T>
uint8_t repetition_round(uint8_t& cases) {
    struct Case {
        TimAlignment align;
        uint8_t rep;
    };
    constexpr Case table[] = {{TimAlignment::edge, 0},        {TimAlignment::edge, 1},
                              {TimAlignment::edge, 4},        {TimAlignment::edge, 255},
                              {TimAlignment::center_both, 0}, {TimAlignment::center_both, 1},
                              {TimAlignment::center_both, 3}};
    uint8_t exact = 0;
    cases = 0;
    for (const Case& c : table) {
        ++cases;
        T::init();
        const uint16_t arr = c.align == TimAlignment::edge ? 99u : 100u;
        (void)T::configure({.prescaler = 47, .period = arr, .alignment = c.align, .repetition = c.rep});
        T::enable(true);
        uint32_t gaps[4] = {};
        TimDirection dir[5] = {};
        const uint32_t per_event = (static_cast<uint32_t>(c.rep) + 1u) * 100u;   // microseconds
        const bool got = flag_intervals<T, 4>(T::update_flag, gaps, dir, per_event + 100u);
        T::enable(false);
        bool ok = got && T::repetition() == c.rep;
        for (const uint32_t g : gaps) {
            ok = ok && near_cycles(g, per_event * us_cycles);
        }
        bool alternating = true;
        bool same = true;
        for (uint8_t i = 1; i < 5u; ++i) {
            alternating = alternating && dir[i] != dir[i - 1u];
            same = same && dir[i] == dir[0];
        }
        if (c.align != TimAlignment::edge) {
            ok = ok && (c.rep % 2u == 0u ? alternating : same);
        } else {
            ok = ok && same && dir[0] == TimDirection::up;
        }
        if (ok) {
            ++exact;
        }
        print(serial, "  TIM", T::instance, c.align == TimAlignment::edge ? " edge-aligned" : " centre-aligned",
              ", RPTCR ", c.rep, ": updates ", gaps[0], " ", gaps[1], " ", gaps[2], " ", gaps[3],
              " cycles apart against ", per_event * us_cycles,
              c.align == TimAlignment::edge ? ""
              : alternating                 ? ", directions alternating"
              : same ? (dir[0] == TimDirection::down ? ", every one counting down (an overflow's)"
                                                     : ", every one counting up (an underflow's)")
                     : ", DIRECTIONS MIXED",
              got ? "" : " (A FLAG DID NOT COME)", crlf);
        T::release();
    }
    return exact;
}

/// CENTRE-ALIGNED COUNTING's three compare-flag policies (CTLR1.CMS,
/// 12.4.1): the counter climbs 0..ATRLR and falls back, and the modes
/// differ only in WHEN CC1IF rises - counting down (01), up (10) or both
/// (11). At 1 MHz, ATRLR 500 and a compare of 200 the up-count meets 200
/// at 200 us and the down-count at 800 us of each 1000: a flag a period
/// apart in modes 1 and 2, the direction at each rise the mode's own, and
/// in mode 3 gaps alternating 600 us (up to down: 300 + 300) and 400 (down
/// to up: 200 + 200).
bool centre_flags(TimAlignment mode, const char* name) {
    T1::init();
    (void)T1::configure({.prescaler = 47, .period = 500, .alignment = mode});
    (void)T1::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 200, .enable = false});
    T1::update();
    T1::clear_flags(0xFFFFu);
    T1::enable(true);
    uint32_t gaps[6] = {};
    TimDirection dir[7] = {};
    const bool got = flag_intervals<T1, 6>(T1::compare_flag(0), gaps, dir, 2000);
    T1::enable(false);
    uint8_t ups = 0;
    for (const TimDirection d : dir) {
        ups = static_cast<uint8_t>(ups + (d == TimDirection::up ? 1u : 0u));
    }
    bool ok = got;
    if (mode == TimAlignment::center_both) {
        // Which half the first gap is depends on where the poll began: the
        // gaps alternate, each the one its starting direction says.
        for (uint8_t i = 0; i < 6u; ++i) {
            const uint32_t want = dir[i] == TimDirection::up ? 600u * us_cycles : 400u * us_cycles;
            ok = ok && near_cycles(gaps[i], want) && dir[i + 1u] != dir[i];
        }
    } else {
        const TimDirection want_dir = mode == TimAlignment::center_down ? TimDirection::down : TimDirection::up;
        for (uint8_t i = 0; i < 6u; ++i) {
            ok = ok && near_cycles(gaps[i], 1000u * us_cycles);
        }
        for (const TimDirection d : dir) {
            ok = ok && d == want_dir;
        }
    }
    print(serial, "  ", name, ": CC1IF ", gaps[0] / us_cycles, " ", gaps[1] / us_cycles, " ", gaps[2] / us_cycles,
          " ", gaps[3] / us_cycles, " ", gaps[4] / us_cycles, " ", gaps[5] / us_cycles, " us apart, ", ups,
          " of 7 rises counting up", got ? "" : " (A FLAG DID NOT COME)", crlf);
    T1::release();
    return ok;
}

/// HCLK cycles a count x100, over about 100 us on the STK with interrupts
/// masked, the counter counting either way and wrapping at its period: the
/// counts in the window are taken modulo ATRLR + 1, so a window must hold
/// fewer counts than a period (every caller's period is 1000 counts or
/// more, the window 200 at most). Good to one count in the window - 1 %
/// at 48 cycles a count, the bound the verdicts take (x100: 48 at PSC 47,
/// 24 at PSC 23).
template <typename T>
uint32_t rate_x100() {
    InterruptGuard guard;
    const uint32_t wrap = static_cast<uint32_t>(T::period()) + 1u;
    const uint16_t c0 = T::count();
    const uint32_t s0 = stk_now();
    while (stk_since(s0, stk_now()) < 4800u) {
    }
    const uint16_t c1 = T::count();
    const uint32_t s1 = stk_now();
    const uint32_t d = T::direction() == TimDirection::down ? (c0 + wrap - c1) % wrap : (c1 + wrap - c0) % wrap;
    return d == 0u ? 0u : (100u * stk_since(s0, s1)) / d;
}

void tf_time_base() {
    all_off();
    console_drain();
    measure_slack();
    console_drain();
    uint8_t cases = 0;
    const uint8_t exact1 = repetition_round<T1>(cases);
    console_drain();
    const uint8_t exact2 = repetition_round<T2>(cases);
    bench.verdict("THE REPETITION COUNTER, TIM1: one update in RPTCR + 1 periods - edge-aligned at 0, 1, 4 and 255, "
                  "centre-aligned at 0, 1 and 3 with an overflow and an underflow counting one each, the "
                  "directions alternating at an even RPTCR and alike at an odd one - every interval within the "
                  "measured slack of the counts",
                  exact1 == cases);
    bench.verdict("TIM2's, the same seven settings, the same bound", exact2 == cases);

    console_drain();
    const bool m1 = centre_flags(TimAlignment::center_down, "mode 1 (CMS 01)");
    const bool m2 = centre_flags(TimAlignment::center_up, "mode 2 (CMS 10)");
    const bool m3 = centre_flags(TimAlignment::center_both, "mode 3 (CMS 11)");
    bench.verdict("CENTRE-ALIGNED, the three compare-flag policies: CC1IF only counting down in mode 1, only "
                  "counting up in mode 2, a period of 2 x ATRLR apart; on both in mode 3, the gaps alternating "
                  "600 and 400 us - every gap within the slack",
                  m1 && m2 && m3);

    // THE DOWN COUNTER (DIR 1, edge-aligned): it falls to 0 and reloads
    // with ATRLR, an underflow - and an update - every ATRLR + 1 counts,
    // and a compare raises CC1IF once a period as on the way up; 1 ms a
    // period. A UG reloads it with ATRLR (12.3.1: "the counter will start
    // counting again from the current auto-reload value") WHILE IT RUNS,
    // and a UG on a STOPPED counter leaves the count where it is, either
    // way (measured) - so configure() writes the start itself: ATRLR here,
    // and the first underflow comes a whole period after the enable.
    T1::init();
    (void)T1::configure({.prescaler = 47, .period = 999});
    T1::set_count(500);
    T1::update();
    const uint16_t ug_up_stopped = T1::count();
    (void)T1::configure({.prescaler = 47, .period = 999, .direction = TimDirection::down});
    const uint16_t loaded = T1::count();
    T1::clear_flags(T1::update_flag);
    const uint32_t enabled_at = Ticker::cycles();
    T1::enable(true);
    const bool first_came = wait_flag<T1>(T1::update_flag, 1100);
    const uint32_t first_after = Ticker::cycles() - enabled_at;
    T1::set_count(500);
    T1::update();
    const uint16_t ug_running = T1::count();
    T1::enable(false);
    T1::set_count(500);
    T1::update();
    const uint16_t ug_stopped = T1::count();
    (void)T1::output_channel(0, {.mode = TimOutputMode::frozen, .compare = 30, .preload = false, .enable = false});
    T1::enable(true);
    const uint32_t down_rate = rate_x100<T1>();
    console_drain();
    uint32_t ugaps[4] = {};
    TimDirection udir[5] = {};
    const bool ugot = flag_intervals<T1, 4>(T1::update_flag, ugaps, udir, 1100);
    uint32_t cgaps[4] = {};
    TimDirection cdir[5] = {};
    const bool cgot = flag_intervals<T1, 4>(T1::compare_flag(0), cgaps, cdir, 1100);
    T1::enable(false);
    bool down_ok = ugot && cgot;
    for (uint8_t i = 0; i < 4u; ++i) {
        down_ok = down_ok && near_cycles(ugaps[i], 1000u * us_cycles) && near_cycles(cgaps[i], 1000u * us_cycles);
    }
    for (uint8_t i = 0; i < 5u; ++i) {
        down_ok = down_ok && udir[i] == TimDirection::down && cdir[i] == TimDirection::down;
    }
    print(serial, "  down-counting: the counter at ", loaded, " after configure(); a UG from 500 leaves ",
          ug_stopped, " stopped and ", ug_running, " running (", ug_up_stopped,
          " stopped counting up); the first underflow ", first_after,
          " cycles after the enable; ", down_rate,
          " hundredths of a cycle a count downwards; updates ", ugaps[0], " ", ugaps[1], " ", ugaps[2], " ",
          ugaps[3], " and CC1IF ", cgaps[0], " ", cgaps[1], " ", cgaps[2], " ", cgaps[3], " cycles apart against ",
          1000u * us_cycles, crlf);
    bench.verdict("A UG reloads a running down counter with ATRLR, and leaves a stopped counter's count alone "
                  "counting either way",
                  ug_running >= 990u && ug_running <= 999u && ug_stopped == 500u && ug_up_stopped == 500u);
    bench.verdict("THE DOWN COUNTER: configure() starts it at ATRLR, the first underflow a whole period after "
                  "the enable (within the slack and a count), the count falling at the prescaled rate",
                  loaded == 999u && first_came &&
                      near(static_cast<int32_t>(first_after), 48000, static_cast<int32_t>(stamp_slack_cycles + 48u)) &&
                      near(static_cast<int32_t>(down_rate), 4800, 48));
    bench.verdict("an underflow every ATRLR + 1 counts, a compare's flag once a period, the direction reading "
                  "down - within the slack", down_ok);
    T1::release();

    // UDIS (12.4.1): the update event disabled - no UIF at an overflow and
    // the shadows not loaded, until it is cleared. First what configure()
    // leaves when it is asked for UDIS: its own UG must still have loaded
    // the prescaler's shadow, or the counter runs at the reset's PSC 0.
    // 1 ms a period at PSC 47; PSC 23 halves the count's length.
    T1::init();
    (void)T1::configure({.prescaler = 47, .period = 999, .update_disable = true});
    T1::enable(true);
    const uint32_t udis_rate = rate_x100<T1>();
    T1::clear_flags(T1::update_flag);
    (void)delay_us(clock, 900);   // the delays stay under a tick period (delay.hpp)
    (void)delay_us(clock, 900);
    (void)delay_us(clock, 900);
    (void)delay_us(clock, 900);   // three overflows and more
    const bool udis_quiet = !T1::flag(T1::update_flag);
    T1::set_prescaler(23);
    (void)delay_us(clock, 900);
    (void)delay_us(clock, 900);   // at least one overflow: no update, so the shadow stays 47
    const uint32_t udis_kept = rate_x100<T1>();
    T1::regs().CTLR1 = static_cast<uint16_t>(T1::regs().CTLR1 & ~tim_udis);
    const bool udis_cleared_flag = wait_flag<T1>(T1::update_flag, 1100);
    const uint32_t after_udis = rate_x100<T1>();
    T1::enable(false);
    print(serial, "  UDIS: configure() asked for it - ", udis_rate, " hundredths of a cycle a count; UIF ",
          udis_quiet ? "quiet" : "RAISED", " over three overflows; PSC 23 written: ", udis_kept,
          "; UDIS cleared: UIF ", udis_cleared_flag ? "at the next overflow" : "NEVER", ", then ", after_udis, crlf);
    bench.verdict("configure() with UDIS loads the prescaler all the same: the counter at PSC 47 counts one every "
                  "48 cycles", near(static_cast<int32_t>(udis_rate), 4800, 48));
    bench.verdict("UDIS set: no UIF at an overflow, and a PSC written meanwhile is not taken",
                  udis_quiet && near(static_cast<int32_t>(udis_kept), 4800, 48));
    bench.verdict("UDIS cleared: the next overflow raises UIF and takes the PSC written",
                  udis_cleared_flag && near(static_cast<int32_t>(after_udis), 2400, 24));
    T1::release();

    // URS (12.4.1): with it set, UG updates - the shadows loaded - without
    // raising UIF; an overflow still raises it.
    T1::init();
    (void)T1::configure({.prescaler = 47, .period = 999, .update_on_overflow_only = true});
    T1::enable(true);
    T1::clear_flags(T1::update_flag);
    T1::set_prescaler(23);
    T1::update();
    const bool urs_ug_flag = T1::flag(T1::update_flag);
    const uint32_t urs_rate = rate_x100<T1>();
    uint32_t rgaps[2] = {};
    TimDirection rdir[3] = {};
    const bool rgot = flag_intervals<T1, 2>(T1::update_flag, rgaps, rdir, 700);
    T1::enable(false);
    print(serial, "  URS: UG raised UIF ", urs_ug_flag, ", the rate after it ", urs_rate,
          " hundredths of a cycle a count; overflows ", rgaps[0], " ", rgaps[1], " cycles apart", crlf);
    bench.verdict("URS set: UG raises no UIF but loads the shadows (PSC 23 taken: one count every 24 cycles)",
                  !urs_ug_flag && near(static_cast<int32_t>(urs_rate), 2400, 24));
    bench.verdict("and an overflow still raises UIF, a period of 1000 counts apart",
                  rgot && near_cycles(rgaps[0], 500u * us_cycles) && near_cycles(rgaps[1], 500u * us_cycles));
    all_off();
}

// ---------------------------------------------------------------------------
// g - TIM2's outputs on a pad: C2N on PB12
// ---------------------------------------------------------------------------
using T2C2n5 = TimPad<afio_tim2_pads(5).ch2n>;
using T2C2n6 = TimPad<afio_tim2_pads(6).ch2n>;
static_assert(T2C2n5::selection == Pad{'B', 12} && T2C2n6::selection == Pad{'B', 12},
              "TIM2_RM 101 and 11x put C2N on PB12");
constexpr uint32_t pb12 = 1UL << 12;

/// TIM2's C2N under one code: forced active and inactive on a running
/// counter read off PB12, CC2NP inverting, and a free-running toggle's
/// half period on the pad. Channel 1 - whose CH1 and C1N these codes put
/// on the debug port's PC19 and PC18 - is never enabled, and its pads are
/// never claimed (TimPad refuses them while the port is the probe's).
template <typename Pad2>
bool tim2_c2n(uint8_t code) {
    T2::init();
    const bool remapped = T2::remap(code);
    (void)T2::configure({.prescaler = 47, .period = 0xFFFF});
    (void)T2::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    T2::main_output(true);
    T2::enable(true);
    const bool claimed = Pad2::claim();
    settle();
    const bool high = pad(pb12);
    (void)T2::output_mode(1, TimOutputMode::force_inactive);
    settle();
    const bool low = !pad(pb12);
    (void)T2::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false,
                                 .complementary_active_low = true, .enable = false, .complementary_enable = true});
    settle();
    const bool inverted = !pad(pb12);
    // A toggle at PSC 0 and a period of 1000: an edge every 1000 cycles.
    T2::enable(false);
    (void)T2::output_channel(1, {.mode = TimOutputMode::force_active, .preload = false, .enable = false,
                                 .complementary_enable = true});
    (void)T2::configure({.prescaler = 0, .period = 999});
    (void)T2::set_compare(1, 500);
    (void)T2::output_mode(1, TimOutputMode::toggle);
    T2::enable(true);
    const Sampled r = sample_port_b(samples_most);
    T2::enable(false);
    uint16_t at[12] = {};
    const uint8_t k = edges_of(r, static_cast<uint16_t>(pb12), at, 12);
    const uint32_t span = k >= 2u ? cycles_of(r, at[k - 1u]) - cycles_of(r, at[0]) : 0u;
    const uint32_t per = k >= 2u ? span / (k - 1u) : 0u;
    const bool ch1_off = !T2::channel_enabled(0) && !T2::complementary_enabled(0);
    print(serial, "  TIM2_RM ", code, ": C2N on PB12 forced active ", high ? "high" : "LOW", ", inactive ",
          low ? "low" : "HIGH", ", CC2NP ", inverted ? "low" : "HIGH", "; the toggle ", k, " edges ", per,
          " cycles apart (1000 asked)", crlf);
    Pad2::release();
    T2::release();
    (void)T2::remap(0);
    return remapped && claimed && high && low && inverted && ch1_off && k >= 5u &&
           near(static_cast<int32_t>(per), 1000, 5);
}

void tg_tim2_pad() {
    all_off();
    const bool code5 = tim2_c2n<T2C2n5>(5);
    bench.verdict("TIM2_RM 101: C2N reaches PB12 - forced active high, inactive low, CC2NP inverting, a toggle "
                  "an edge every 1000 cycles within 0.5 %",
                  code5);
    const bool code6 = tim2_c2n<T2C2n6>(6);
    bench.verdict("TIM2_RM 110: the same on the same pad", code6);
    bench.verdict("and the remap field back at 000, the debug port the probe's",
                  T2::remap() == 0u && Afio::debug_port_enabled());
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

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_VECTOR(usart2_handler) { (void)Serial::isr(); }

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
    bench.letter('f', "the time base: repetition, centre-aligned, down, UDIS, URS", tf_time_base);
    bench.letter('g', "TIM2's outputs on a pad: C2N on PB12", tg_tim2_pad);

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
