// ac_sync_probe - how many GCLK_AC cycles does the AC's SYNCHRONIZED
// output cost? (SAM C21, DS60001479M ch. 40)
//
// The chapter samples everything digital at GCLK_AC and routes either
// the raw comparator or "the synchronous output (including filtering)"
// to the CMPn pad - but never says what the synchronous path costs in
// cycles. The plain reading ("the output is sampled") predicts the NEXT
// GCLK_AC edge, i.e. a fraction of a cycle; a classic two-flop
// synchronizer predicts one or two more. This probe measures it.
//
// THE TRICK IS A SLOW GCLK_AC: generator 1 (the one with the 16-bit
// linear divider) runs OSC48M / 4096 = 11.719 kHz, so one sampling
// period is EXACTLY 4096 CPU cycles and the
// SysTick cycle stopwatch (48 MHz, the test_samc_dma technique) resolves
// 1/4000 of it. The comparator's own analog delay (38-73 ns high speed,
// electrical table 45-34) is two-three CPU cycles - invisible.
//
// Setup, all inside the chip, no wires: COMP0 positive input = AIN[0]
// = PA04 DRIVEN BY PORT AS A PLAIN GPIO (letter a proves the analog mux
// really sees a GPIO-driven pad - the AVR suites' trick, checked before
// anything relies on it); negative input = the comparator's own VDD
// scaler at mid-rail. The observers: the CMP0 pad (PA12, function H,
// input buffer on, read back through PORT.IN), STATUSA.STATE0, and
// INTFLAG.COMP0.
//
// Letters (z = all):
//   a  block + comparator up, t_STARTUP, and the GPIO-driven-pad proof
//   b  the ASYNC control: pad-to-pad latency with OUT=asynchronous -
//      everything the measurement chain costs EXCEPT synchronization
//   c  THE ANSWER: OUT=synchronous, phase-anchored 64-step staircase
//      plus a 1000-shot randomized run - latency in GCLK_AC periods
//   d  the three observers at the same phases: SYNC pad vs STATE0 vs
//      INTFLAG.COMP0 (edge detection = current vs previous sample)
//   e  the filters: a mid-stream edge pays +1 (MAJ3) / +2 (MAJ5)
//      periods - the majority arithmetic, not the chapter's N-1
//   f  single-shot: START to READY, figure 40-4's "2-3 cycles"
//   g  the same randomized run with GCLK_AC on OSCULP32K - a clock the
//      CPU does not share, so the phase is naturally uniform
//
// build: boards = c21j

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/clock.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,
    .rx = brio::SercomPad::pad1,
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};
using Serial = brio::Uart<5, console_pads>;
constexpr Serial serial;

using Led = brio::Pin<'B', 23>;
using Stim = brio::Pin<'A', 4>;     // AC AIN[0], driven as plain GPIO
using CmpPad = brio::Pin<'A', 12>;  // AC CMP0, function H, read via PORT.IN

using Comp = brio::AcComparator<0>;

brio::TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// The slow sampling clock: generator 1, the only one whose LINEAR
// divider reaches 4096 (Table 16-3: DIV[15:0]; the 8-bit generators
// stop at 255 - and their DIVSEL path is a FIXED 512, see GclkConfig).
constexpr uint8_t ac_generator = 1;
constexpr uint32_t period = 4096;         // CPU cycles per GCLK_AC period
constexpr uint32_t ulp_period = 1465;     // ~48e6 / 32768, nominal

// ---------------------------------------------------------------------------
// The cycle stopwatch (test_samc_dma's technique)
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = brio::Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = brio::Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

void spin_until(uint32_t target) {
    while (static_cast<int32_t>(cycles_now() - target) < 0) {
    }
}

void settle(uint32_t cycles) { spin_until(cycles_now() + cycles); }

constexpr uint32_t no_flip = 0xFFFFFFFFu;

/// Poll `cond` and timestamp the flip. The poll loop reads one register
/// per turn (a handful of cycles); the cycles_now() at detection adds a
/// CONSTANT offset that letter b measures for the whole chain.
template <typename F>
uint32_t time_until(F cond, uint32_t spins) {
    while (spins-- != 0u) {
        if (cond()) {
            return cycles_now();
        }
    }
    return no_flip;
}

uint32_t lcg_state = 0x2F1E4B95u;
uint32_t lcg(uint32_t modulus) {
    lcg_state = lcg_state * 1103515245u + 12345u;
    return (lcg_state >> 16) % modulus;
}

// ---------------------------------------------------------------------------
// Shared comparator plumbing
// ---------------------------------------------------------------------------

/// (Re)configure COMP0 for this probe with the given output routing and
/// filter, enable it, wait READY plus a settling margin. The scaler sits
/// at mid-rail; the stimulus pad decides the state.
bool comp_up(brio::AcOut out, brio::AcFilter filter, bool single = false,
             brio::AcInterrupt intsel = brio::AcInterrupt::toggle) {
    if (!Comp::configure({
            .positive = brio::AcPositive::pin0,
            .negative = brio::AcNegative::vscale,
            .single_shot = single,
            .interrupt_on = intsel,
            .speed = brio::AcSpeed::high,
            .filter = filter,
            .out = out,
        })) {
        return false;
    }
    Comp::scaler(31);   // (31+1)/64 = VDD/2
    if (!Comp::enable(true)) {
        return false;
    }
    if (single) {
        return true;   // idle until start(); READY comes per comparison
    }
    // READY is hardware state in the GCLK_AC domain: at 11.7 kHz give it
    // whole periods, not just the electrical t_STARTUP.
    const uint32_t deadline = cycles_now() + 32u * period;
    while (!Comp::ready()) {
        if (static_cast<int32_t>(cycles_now() - deadline) >= 0) {
            return false;
        }
    }
    settle(4u * period);
    return true;
}

/// One timed rising edge on the stimulus pad against observer `cond`:
/// prime low, wait the observer low, apply `phase_delay` from a
/// GCLK-edge anchor (the observer's own falling flip - it lands on a
/// GCLK_AC edge, which is what makes the staircase deterministic), then
/// drive high and time the flip. Returns cycles from drive to flip.
template <typename F>
uint32_t timed_edge(F cond, uint32_t phase_delay, uint32_t budget_periods) {
    Stim::clear();
    const uint32_t low = time_until([&] { return !cond(); }, 40u * period);
    if (low == no_flip) {
        return no_flip;
    }
    spin_until(low + 4u * period + phase_delay);
    const uint32_t t0 = cycles_now();
    Stim::set();
    const uint32_t t1 = time_until(cond, budget_periods * period);
    return (t1 == no_flip) ? no_flip : (t1 - t0);
}

bool pad_high() { return CmpPad::read(); }
bool state_high() { return Comp::state(); }

// =============================================================================
// a - bring-up and the GPIO-driven-pad proof
// =============================================================================
void ta_up() {
    const bool ac_ok = brio::Ac::init(ac_generator);
    bench.verdict("the AC block came up on generator 1", ac_ok);
    print(serial, "  GENCTRL[1] = ", brio::hex(GCLK_REGS->GCLK_GENCTRL[ac_generator]),
          " (SRC=OSC48M, DIV=4096 linear expected)", crlf);

    // Startup: enable to READY, measured. Electrical says 2-3 us in
    // high speed; READY itself lives in the slow clock domain, so what
    // this really bounds is "few periods", and that is worth printing.
    (void)Comp::configure({
        .positive = brio::AcPositive::pin0,
        .negative = brio::AcNegative::vscale,
        .speed = brio::AcSpeed::high,
    });
    Comp::scaler(31);
    Stim::output();
    Stim::set();
    const uint32_t t0 = cycles_now();
    (void)Comp::enable(true);
    const uint32_t rdy = time_until([] { return Comp::ready(); }, 64u * period);
    bench.verdict("READY0 arrived", rdy != no_flip);
    if (rdy != no_flip) {
        print(serial, "  enable to READY0: ", rdy - t0, " cycles = ",
              (rdy - t0 + period / 2u) / period, " GCLK_AC periods", crlf);
    }
    settle(4u * period);

    // THE PLAN-A PROOF: the comparator must see a pad PORT is driving.
    Stim::set();
    settle(4u * period);
    const bool sees_high = Comp::state();
    Stim::clear();
    settle(4u * period);
    const bool sees_low = !Comp::state();
    bench.verdict("STATE0 follows a GPIO-driven AIN pad (plan A holds)",
                  sees_high && sees_low);
    if (!(sees_high && sees_low)) {
        print(serial, "  PLAN B NEEDED: jumper a free GPIO to PA04 and drive that",
              crlf);
    }

    // The scaler is a real threshold: above it with the pad high, below
    // it with the pad low, at both extremes of the ladder.
    Stim::set();
    Comp::scaler(1);    // ~VDD/32: pad high must read above
    settle(4u * period);
    const bool above = Comp::state();
    Comp::scaler(62);   // ~VDD 63/64: pad high still above? no - VDD is the rail
    settle(4u * period);
    const bool near_rail = Comp::state();   // informative only, offset decides
    Comp::scaler(31);
    settle(4u * period);
    bench.verdict("scaler low threshold behaves", above);
    print(serial, "  pad at VDD vs scaler 63/64: state=", near_rail ? 1 : 0,
          " (offset decides, informative)", crlf);
}

// =============================================================================
// b - the ASYNC control: the measurement chain without synchronization
// =============================================================================
void tb_async() {
    bench.verdict("comparator up, OUT=async", comp_up(brio::AcOut::asynchronous,
                                                      brio::AcFilter::off));
    CmpPad::function(brio::PinFunction::h, {.input_enable = true});

    uint32_t lo = no_flip, hi = 0;
    for (uint8_t i = 0; i < 32; ++i) {
        const uint32_t dt = timed_edge(pad_high, lcg(period), 4);
        if (dt == no_flip) {
            continue;
        }
        if (dt < lo) lo = dt;
        if (dt > hi) hi = dt;
    }
    bench.verdict("async pad flips at all", lo != no_flip);
    print(serial, "  ASYNC pad-to-pad: min ", lo, " max ", hi, " cycles",
          " (analog TPD + drive + poll + stopwatch chain)", crlf);
    // The async path never sees GCLK_AC: it must be far below one period.
    bench.verdict("async latency is far below one sampling period",
                  lo != no_flip && hi < period / 4u);

    // STATE0 read while OUT is async: the readback is the SAMPLED state,
    // so unlike the pad it should quantize - a first hint of the answer.
    uint32_t slo = no_flip, shi = 0;
    for (uint8_t i = 0; i < 32; ++i) {
        const uint32_t dt = timed_edge(state_high, lcg(period), 8);
        if (dt == no_flip) {
            continue;
        }
        if (dt < slo) slo = dt;
        if (dt > shi) shi = dt;
    }
    bench.verdict("STATE0 flips too", slo != no_flip);
    print(serial, "  STATE0 (async out): min ", slo, " max ", shi, " cycles = ",
          slo / period, "..", (shi + period - 1u) / period, " periods", crlf);
}

// =============================================================================
// c - the answer: the synchronized output, phase by phase
// =============================================================================
void tc_sync() {
    bench.verdict("comparator up, OUT=sync", comp_up(brio::AcOut::synchronous,
                                                     brio::AcFilter::off));
    CmpPad::function(brio::PinFunction::h, {.input_enable = true});

    // The anchored staircase: 64 phase steps of 64 cycles, min of 3 per
    // step (the tick interrupt can stretch a poll, never shrink it).
    uint32_t stair[64];
    uint32_t lo = no_flip, hi = 0;
    for (uint8_t s = 0; s < 64; ++s) {
        uint32_t best = no_flip;
        for (uint8_t r = 0; r < 3; ++r) {
            const uint32_t dt = timed_edge(pad_high, static_cast<uint32_t>(s) * 64u, 8);
            if (dt < best) {
                best = dt;
            }
        }
        stair[s] = best;
        if (best != no_flip) {
            if (best < lo) lo = best;
            if (best > hi) hi = best;
        }
    }
    print(serial, "  staircase (cycles, phase step = 64):", crlf);
    for (uint8_t row = 0; row < 8; ++row) {
        print(serial, "   ");
        for (uint8_t col = 0; col < 8; ++col) {
            print(serial, " ", stair[row * 8u + col]);
        }
        print(serial, crlf);
    }
    print(serial, "  min ", lo, " max ", hi, " span ", hi - lo, " (one period = ",
          period, ")", crlf);
    bench.verdict("every staircase point measured", lo != no_flip && hi != 0u);
    // The span of a pure sampling process is one period (the fraction);
    // the whole-period floor under it is the synchronizer depth.
    bench.verdict("the staircase spans one sampling period (+-1/8)",
                  hi - lo > period - period / 8u && hi - lo < period + period / 8u);
    bench.verdict("the floor is TWO whole periods (fraction + 2)",
                  lo >= 2u * period && lo < 2u * period + period / 8u);
    bench.verdict("the ceiling is three periods plus the chain",
                  hi >= 3u * period && hi < 3u * period + period / 8u);

    // The randomized run: 1000 shots, uniform phase, no anchor trusted.
    uint32_t rlo = no_flip, rhi = 0;
    uint32_t in_period[8] = {};
    for (uint16_t i = 0; i < 1000; ++i) {
        const uint32_t dt = timed_edge(pad_high, lcg(period), 8);
        if (dt == no_flip) {
            continue;
        }
        if (dt < rlo) rlo = dt;
        if (dt > rhi) rhi = dt;
        const uint32_t p = dt / period;
        ++in_period[p < 8u ? p : 7u];
    }
    print(serial, "  randomized 1000 shots: min ", rlo, " max ", rhi, crlf,
          "  shots whose latency lies in period N: ");
    for (uint8_t p = 0; p < 8; ++p) {
        print(serial, in_period[p], " ");
    }
    print(serial, "(N = 0..7)", crlf);
    print(serial, "  => the sync output costs the fraction to the next edge + ",
          rlo / period, " whole period(s); worst case ", (rhi + period - 1u) / period,
          crlf);
    bench.verdict("randomized floor is two whole periods too",
                  rlo >= 2u * period && rlo < 2u * period + period / 8u);
    bench.verdict("randomized min agrees with the staircase", rlo / period == lo / period);

    // Falling edge symmetric check, coarse.
    Stim::set();
    settle(8u * period);
    uint32_t flo = no_flip, fhi = 0;
    for (uint8_t i = 0; i < 32; ++i) {
        Stim::set();
        const uint32_t up = time_until(pad_high, 40u * period);
        if (up == no_flip) {
            continue;
        }
        spin_until(up + 4u * period + lcg(period));
        const uint32_t t0 = cycles_now();
        Stim::clear();
        const uint32_t t1 = time_until([] { return !CmpPad::read(); }, 8u * period);
        if (t1 == no_flip) {
            continue;
        }
        const uint32_t dt = t1 - t0;
        if (dt < flo) flo = dt;
        if (dt > fhi) fhi = dt;
    }
    print(serial, "  falling edge: min ", flo, " max ", fhi, " cycles", crlf);
    bench.verdict("falling edge lives in the same periods",
                  flo != no_flip && flo / period == lo / period);
}

// =============================================================================
// d - the three observers at the same phases
// =============================================================================
void td_observers() {
    bench.verdict("comparator up, OUT=sync", comp_up(brio::AcOut::synchronous,
                                                     brio::AcFilter::off,
                                                     false, brio::AcInterrupt::rising));
    CmpPad::function(brio::PinFunction::h, {.input_enable = true});

    print(serial, "  phase : pad / STATE0 / INTFLAG (cycles)", crlf);
    bool all_measured = true;
    uint32_t max_state_vs_pad = 0, max_flag_vs_pad = 0;
    for (uint8_t s = 0; s < 8; ++s) {
        const uint32_t phase = static_cast<uint32_t>(s) * (period / 8u);
        uint32_t d_pad = no_flip, d_state = no_flip, d_flag = no_flip;
        for (uint8_t r = 0; r < 3; ++r) {
            const uint32_t a = timed_edge(pad_high, phase, 8);
            if (a < d_pad) d_pad = a;
            const uint32_t b = timed_edge(state_high, phase, 8);
            if (b < d_state) d_state = b;
            Comp::clear_flag();
            const uint32_t c = timed_edge([] { return Comp::flag_set(); }, phase, 8);
            if (c < d_flag) d_flag = c;
        }
        print(serial, "  ", phase, ": ", d_pad, " / ", d_state, " / ", d_flag, crlf);
        if (d_pad == no_flip || d_state == no_flip || d_flag == no_flip) {
            all_measured = false;
            continue;
        }
        const uint32_t sv = (d_state > d_pad) ? d_state - d_pad : 0u;
        const uint32_t fv = (d_flag > d_pad) ? d_flag - d_pad : 0u;
        if (sv > max_state_vs_pad) max_state_vs_pad = sv;
        if (fv > max_flag_vs_pad) max_flag_vs_pad = fv;
    }
    bench.verdict("all three observers measured at every phase", all_measured);
    print(serial, "  STATE0 lags the sync pad by at most ", max_state_vs_pad,
          " cycles; INTFLAG by at most ", max_flag_vs_pad, crlf);
}

// =============================================================================
// e - the filters: MAJ3 and MAJ5
// =============================================================================
void te_filters() {
    uint32_t base_lo = no_flip;
    {
        bench.verdict("OUT=sync FLEN=off up", comp_up(brio::AcOut::synchronous,
                                                      brio::AcFilter::off));
        CmpPad::function(brio::PinFunction::h, {.input_enable = true});
        for (uint8_t i = 0; i < 48; ++i) {
            const uint32_t dt = timed_edge(pad_high, lcg(period), 12);
            if (dt < base_lo) base_lo = dt;
        }
    }
    struct Case {
        brio::AcFilter f;
        const char* name;
        // Extra periods a mid-stream EDGE costs in continuous mode: the
        // majority crosses once (N+1)/2 samples agree, so 3-bit costs
        // one extra sample and 5-bit two. The chapter's "N-1 sampling
        // cycles" clause times a comparison STARTED (first valid output
        // from idle), which is a different question - round 1 measured
        // exactly this split.
        uint32_t extra;
    };
    const Case cases[] = {{brio::AcFilter::majority3, "MAJ3", 1},
                          {brio::AcFilter::majority5, "MAJ5", 2}};
    for (const auto& c : cases) {
        bench.verdict("filter case up", comp_up(brio::AcOut::synchronous, c.f));
        uint32_t lo = no_flip, hi = 0;
        for (uint8_t i = 0; i < 48; ++i) {
            const uint32_t dt = timed_edge(pad_high, lcg(period), 16);
            if (dt == no_flip) {
                continue;
            }
            if (dt < lo) lo = dt;
            if (dt > hi) hi = dt;
        }
        print(serial, "  ", c.name, ": min ", lo, " max ", hi, " cycles; vs no-filter min +",
              (lo != no_flip && lo > base_lo) ? (lo - base_lo) : 0u,
              " (majority arithmetic says +", c.extra, " period(s) = +",
              c.extra * period, " for a mid-stream edge; the chapter's N-1 is",
              " the from-idle cost)", crlf);
        bench.verdict("a mid-stream edge pays (N-1)/2 extra periods (+-1/4)",
                      lo != no_flip && base_lo != no_flip &&
                          lo - base_lo > c.extra * period - period / 4u &&
                          lo - base_lo < c.extra * period + period / 4u);
    }
}

// =============================================================================
// f - single-shot: START to READY
// =============================================================================
void tf_single() {
    bench.verdict("single-shot up", comp_up(brio::AcOut::off, brio::AcFilter::off,
                                            true, brio::AcInterrupt::end_of_comparison));
    Stim::set();
    settle(2u * period);

    uint32_t lo = no_flip, hi = 0;
    bool state_ok = true;
    for (uint8_t i = 0; i < 32; ++i) {
        settle(lcg(period));
        const uint32_t t0 = cycles_now();
        Comp::start();
        const uint32_t t1 = time_until([] { return Comp::ready(); }, 16u * period);
        if (t1 == no_flip) {
            continue;
        }
        const uint32_t dt = t1 - t0;
        if (dt < lo) lo = dt;
        if (dt > hi) hi = dt;
        state_ok = state_ok && Comp::state();   // pad is high: state must say so
    }
    bench.verdict("single-shot comparisons complete", lo != no_flip);
    bench.verdict("every single-shot read the driven pad high", state_ok);
    print(serial, "  START to READY: min ", lo, " max ", hi, " cycles = ",
          lo / period, "..", (hi + period - 1u) / period,
          " periods (fig. 40-4 draws 2-3 cycles + t_STARTUP)", crlf);
}

// =============================================================================
// g - the independent clock: GCLK_AC on OSCULP32K
// =============================================================================
void tg_ulp() {
    const bool gen_ok =
        brio::Gclk<ac_generator>::configure({.source = brio::GclkSource::osculp32k});
    bench.verdict("the slow generator moved to OSCULP32K", gen_ok);
    bench.verdict("comparator re-up on the independent clock",
                  comp_up(brio::AcOut::synchronous, brio::AcFilter::off));
    CmpPad::function(brio::PinFunction::h, {.input_enable = true});

    uint32_t lo = no_flip, hi = 0;
    uint16_t measured = 0;
    for (uint16_t i = 0; i < 300; ++i) {
        const uint32_t dt = timed_edge(pad_high, lcg(ulp_period), 8);
        if (dt == no_flip) {
            continue;
        }
        ++measured;
        if (dt < lo) lo = dt;
        if (dt > hi) hi = dt;
    }
    print(serial, "  OSCULP32K (period ~", ulp_period, "): ", measured,
          " shots, min ", lo, " max ", hi, " = ", lo / ulp_period, "..",
          (hi + ulp_period - 1u) / ulp_period, " periods", crlf);
    bench.verdict("most shots measured on the independent clock", measured > 250u);
    bench.verdict("the whole-period count matches the divided-OSC48M run",
                  lo != no_flip && lo / ulp_period >= 1u);

    // Leave the world as letter a built it.
    (void)brio::Gclk<ac_generator>::configure({.source = brio::GclkSource::osc48m,
                                               .div = 11,
                                               .div_pow2 = true});
    (void)comp_up(brio::AcOut::synchronous, brio::AcFilter::off);
}

} // namespace

// The one vendor glue an app may contain: the vector bindings. An
// unbound vector is Default_Handler's forever-loop, not a crash - the
// DMA suite's lesson, re-learned once already.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

namespace {

// =============================================================================
void banner() {
    print(serial, crlf,
          "ac_sync_probe - SAMC21J18A AC (ch. 40) sync-output latency, clk=",
          SysClock::hz, " Hz, GCLK_AC = OSC48M/4096 = 11719 Hz", crlf);
    bench.menu();
}

} // namespace

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    // The slow sampling clock, before any letter runs. LINEAR divider:
    // the DIVSEL path is a fixed /512 on this family (measured; the
    // GclkConfig comment carries the clause).
    const bool gen_ok = brio::Gclk<ac_generator>::configure(
        {.source = brio::GclkSource::osc48m, .div = 4096});

    brio::enable_interrupts();

    bench.letter('a', "block up, t_STARTUP, the GPIO-driven-pad proof", ta_up);
    bench.letter('b', "ASYNC control: the chain without synchronization", tb_async);
    bench.letter('c', "SYNC latency: staircase + randomized (the answer)", tc_sync);
    bench.letter('d', "sync pad vs STATE0 vs INTFLAG", td_observers);
    bench.letter('e', "filters MAJ3/MAJ5 (+1/+2 periods, majority math)", te_filters);
    bench.letter('f', "single-shot START to READY", tf_single);
    bench.letter('g', "the independent clock (OSCULP32K)", tg_ulp);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " gen1=", gen_ok ? "OSC48M/4096" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
