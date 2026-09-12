// test_ch32_tim - the reference bench suite for the CH32V00x's timers:
// ch32v00x/tim.hpp's Tim<1>, Tim<2> and Tim3 over RM ch. 11, 12 and 13,
// most of it with NO WIRE - one timer clocking another, gating another,
// the break staged from a pad's own pull - and one jumper for the
// captures. TIM3 is the CH32V006's: letters e and x, and the TIM3
// clauses of letter a, build where the part has it; the rest is the
// same on both parts.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE JUMPER, for letters f and g only:
//
//   PD2 (TIM1_CH1, the PWM out)  <->  PD4 (TIM2_CH1, the capture in)
//
// main() probes it (PD2 driven both ways as a GPIO, PD4 read) and the
// two letters decline with the reason when it is missing. PC2
// (TIM1_BKIN) stays unwired: letter i stages the break from the pad's
// internal pull. PC0 is the LED.
//
// What is exercised, letter by letter:
//   a  the blocks, WIRELESS: the reset values of the three, the facts
//      as refusals (a pair on TIM1's channel 4, a repetition on TIM2,
//      TIM3's DMA request off channel 1), the arithmetic
//   b  THE TIME BASE against the STK: TIM2 at 1 MHz read against the
//      cycle counter over 100 ms, and a TimPeriodicTick at 1 kHz
//      counted for 200 ms
//   c  THE PERIODS: edge-aligned ATRLR + 1 counts per update; centre-
//      aligned measured (2 x ATRLR or 2 x (ATRLR + 1) - the chapter
//      does not say); the repetition counter dividing TIM1's updates
//   d  ONE TIMER MEASURING ANOTHER, no wire: TIM2 counting TIM1's
//      update on ITR0 (TimEventCounter), TIM1 counting TIM2's on ITR1,
//      and TIM2 GATED by TIM1's OC1REF measuring a 25% duty internally
//   e  TIM3: clocked by TIM1's trigger (external clock mode 1), its
//      count against TIM1's updates; its channel 3 match as a DMA
//      request (table 8-2: DMA channel 1) - which serves ONCE
//   x  (outside z) what re-arms TIM3's request, probed nine ways: only
//      the RCC reset pulse does; the same request on TIM2 paces a
//      channel every period, into its own compare register or a sink
//   f  THE CAPTURES on the jumper: TimPeriodMeter reading TIM1's PWM
//      period and width at three duties; TimIntervalMeter's edges
//   g  ONE PULSE on the jumper: a TimOnePulse of a known width fired
//      by software, its width captured
//   i  THE BREAK, wireless: PC2 pulled up with BKP active-high drops
//      MOE and raises BIF; pulled down, the outputs come back under AOE
//
// build: boards = v006k8,v003f4
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

using T1 = Tim<1>;
using T2 = Tim<2>;
using OutPad = TimPad<tim_default_pads::tim1_ch1>;   // PD2
using InPad = TimPad<tim_default_pads::tim2_ch1_etr>;   // PD4
using BreakPad = TimPad<tim_default_pads::tim1_bkin>;   // PC2

TestBench<Serial> bench;

volatile uint32_t t1_updates = 0;
volatile uint32_t t2_updates = 0;
volatile uint32_t t2_captures = 0;
volatile uint16_t last_period = 0;
volatile uint16_t last_width = 0;
volatile uint32_t t1_breaks = 0;
bool jumper_present = false;

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// Cycles since boot from the tick count and the STK's own counter. The
/// counter reloads at CMP and raises CNTIF; the handler that counts the
/// tick runs an interrupt latency LATER, so a sample taken in between
/// would read one period low (measured: about one sample in a million,
/// and a window that starts within a period of the wrap then ends at
/// once, its unsigned difference wrapped). CNTIF read on both sides of
/// CNT says whether the wrap is already in the counter and not yet in
/// the tick.
uint32_t cycles_now() {
    const uint32_t period = stk()->CMP + 1u;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const bool wrapped0 = (stk()->SR & stk_cntif) != 0u;
        const uint32_t cnt = stk()->CNT;
        const bool wrapped1 = (stk()->SR & stk_cntif) != 0u;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1 && wrapped0 == wrapped1) {
            return (t0 + (wrapped1 ? 1u : 0u)) * period + cnt;
        }
    }
}

/// An EXACT window, on the cycle counter: settle_ms() starts at a random
/// point inside a millisecond and its window is short by up to one, which
/// at 1000 events a window reads as five missing.
void spin_cycles(uint32_t cycles) {
    const uint32_t s0 = cycles_now();
    while (cycles_now() - s0 < cycles) {
    }
}
constexpr uint32_t cycles_100ms = 4'800'000UL;

/// The sensor's pull is set AGAINST the driven level, so a floating pad
/// echoing its neighbour is not taken for the wire (a wire beats a pull).
bool probe_jumper() {
    InPad::pin::input(PinPull::down);
    OutPad::pin::output(true);
    (void)delay_us(clock, 5);
    const bool high = InPad::pin::read();
    InPad::pin::input(PinPull::up);
    OutPad::pin::clear();
    (void)delay_us(clock, 5);
    const bool low = !InPad::pin::read();
    OutPad::pin::release();
    InPad::pin::release();
    return high && low;
}

bool need_jumper() {
    if (jumper_present) {
        return true;
    }
    jumper_present = probe_jumper();
    if (!jumper_present) {
        print(serial, "  SKIPPED, no verdict claimed: no jumper between PD2 (TIM1_CH1) and PD4 "
                      "(TIM2_CH1)", crlf);
    }
    return jumper_present;
}

void all_off() {
    Pfic::disable(T1::update_irq());
    Pfic::disable(T1::cc_irq());
    Pfic::disable(T1::break_irq());
    Pfic::disable(T2::update_irq());
    Pfic::disable(Irq::dma1_channel1);
    T1::init();
    T2::init();
#if BRIO_CH32_HAS_TIM3
    Tim3::init();
#endif
    DmaChannel<1>::stop();
    OutPad::release();
    InPad::release();
    BreakPad::release();
    t1_updates = 0;
    t2_updates = 0;
    t2_captures = 0;
    t1_breaks = 0;
}

// ===========================================================================
// a - the blocks, wireless
// ===========================================================================

void ta_blocks() {
    all_off();
    print(serial, "  TIM1: CTLR1=", hex(T1::regs().CTLR1), " ATRLR=", hex(T1::regs().ATRLR), " BDTR=",
          hex(T1::regs().BDTR), "; TIM2: ATRLR=", hex(T2::regs().ATRLR), " DTCR=", hex(T2::regs().BDTR));
#if BRIO_CH32_HAS_TIM3
    print(serial, "; TIM3: CTLR1=", hex(Tim3::regs().CTLR1), " ATRLR=", hex(Tim3::regs().ATRLR), crlf);
    bench.verdict("the reset values are the tables' (ATRLR 0xFFFF on all three, the rest zero)",
                  T1::regs().CTLR1 == 0u && T1::regs().ATRLR == 0xFFFFu && T1::regs().BDTR == 0u &&
                      T2::regs().ATRLR == 0xFFFFu && Tim3::regs().ATRLR == 0xFFFFu);
    bench.verdict("the facts as refusals: a repetition on TIM2, a break on TIM2, a pair dead time on "
                  "TIM1, TIM3's DMA request off channels 3 and 4",
                  !T2::configure({.period = 10, .repetition = 1}) && !T2::break_dead_time({}) &&
                      !T1::pair_dead_time(0, {.ticks = 2}) && !Tim3::dma_request(0, true) &&
                      Tim3::dma_request(2, true));
#else
    print(serial, " (this part has no TIM3, and no dead time on TIM2)", crlf);
    bench.verdict("the reset values are the tables' (ATRLR 0xFFFF on both, the rest zero)",
                  T1::regs().CTLR1 == 0u && T1::regs().ATRLR == 0xFFFFu && T1::regs().BDTR == 0u &&
                      T2::regs().ATRLR == 0xFFFFu);
    bench.verdict("the facts as refusals: a repetition on TIM2, a break on TIM2, a pair dead time on "
                  "TIM1 and on TIM2",
                  !T2::configure({.period = 10, .repetition = 1}) && !T2::break_dead_time({}) &&
                      !T1::pair_dead_time(0, {.ticks = 2}) && !T2::pair_dead_time(0, {.ticks = 2}));
#endif
    bench.verdict("the refusals of the chapter: a period of zero, a gated mode on TI1's edge "
                  "detector, an ITR nothing is wired to",
                  !T1::configure({.period = 0}) &&
                      !T1::slave({.mode = TimSlaveMode::gated, .trigger = TimTrigger::ti1_edge}) &&
                      !T1::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::itr0}) &&
                      T1::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::itr1}));
    // A configuration reads back.
    (void)T1::configure({.prescaler = 47, .period = 999, .alignment = TimAlignment::center_up,
                         .clock_division = TimClockDivision::div2, .auto_reload_preload = true,
                         .repetition = 3, .capture_level = true, .outputs_float_when_stopped = true});
    print(serial, "  TIM1 configured: CTLR1=", hex(T1::regs().CTLR1), " PSC=", T1::prescaler(), " ATRLR=",
          T1::period(), " RPTCR=", T1::repetition(), " CNT=", T1::count(), crlf);
    bench.verdict("PSC, ATRLR, RPTCR and every CTLR1 field land as spelled, the counter zeroed, "
                  "UIF cleared",
                  T1::prescaler() == 47u && T1::period() == 999u && T1::repetition() == 3u &&
                      T1::regs().CTLR1 == (tim_arpe | (2u << 5) | (1u << 8) | tim_caplvl | tim_oe_mode) &&
                      !T1::flag(T1::update_flag));
    bench.verdict("the dead-time arithmetic: 130 ticks asks code 0x81 (129 -> 130), the top is 1008",
                  tim_dead_time_code(130) == 0x81u && tim_dead_time_ticks(0xFF) == 1008u);
    print(serial, "  jumper PD2-PD4: ", jumper_present ? "present" : "ABSENT", crlf);
    all_off();
}

// ===========================================================================
// b - the time base against the STK
// ===========================================================================

void tb_timebase() {
    all_off();
    // TIM2 at 1 MHz (PSC 47), free-running to 0xFFFF: over 100 ms the
    // count should advance 100000 modulo 65536, read against the STK.
    (void)T2::configure({.prescaler = 47, .period = 0xFFFF});
    T2::enable(true);
    const uint16_t c0 = T2::count();
    const uint32_t s0 = cycles_now();
    while (cycles_now() - s0 < 4'800'000UL) {
    }
    const uint16_t c1 = T2::count();
    const uint32_t s1 = cycles_now();
    T2::enable(false);
    const uint32_t stk_us = (s1 - s0) / 48u;
    const uint32_t tim_us = static_cast<uint16_t>(c1 - c0) + 65536UL;   // one wrap in 100 ms
    print(serial, "  TIM2 at 1 MHz over ", stk_us, " us of STK: ", tim_us, " counts (one wrap folded in)", crlf);
    bench.verdict("the counter runs at HCLK / (PSC + 1) against the STK within 0.1%",
                  tim_us >= stk_us - stk_us / 1000u && tim_us <= stk_us + stk_us / 1000u);

    // A 1 kHz tick from TIM1 (PSC 47, ATRLR 999), counted for 200 ms.
    t1_updates = 0;
    Pfic::enable(T1::update_irq());
    (void)TimPeriodicTick<T1>::setup(47, 999);
    settle_ms(200);
    TimPeriodicTick<T1>::stop();
    Pfic::disable(T1::update_irq());
    print(serial, "  TIM1 ticking at 1 kHz for 200 ms: ", t1_updates, " update interrupts", crlf);
    bench.verdict("a TimPeriodicTick at 1 kHz delivers 200 (+-1) updates in 200 ms",
                  t1_updates >= 199u && t1_updates <= 201u);
    all_off();
}

// ===========================================================================
// c - the periods
// ===========================================================================

uint32_t updates_in_100ms(uint16_t prescaler, uint16_t period, TimAlignment alignment, uint8_t repetition) {
    t1_updates = 0;
    (void)T1::configure({.prescaler = prescaler, .period = period, .alignment = alignment,
                         .repetition = repetition});
    T1::clear_flags(T1::update_flag);
    T1::interrupts(T1::update_interrupt, true);
    Pfic::enable(T1::update_irq());
    T1::enable(true);
    spin_cycles(cycles_100ms);
    T1::enable(false);
    Pfic::disable(T1::update_irq());
    T1::interrupts(T1::update_interrupt, false);
    return t1_updates;
}

void tc_periods() {
    all_off();
    // Edge-aligned: PSC 47 (1 MHz), ATRLR 99: 100 counts a period, 1000 updates in 100 ms.
    const uint32_t edge = updates_in_100ms(47, 99, TimAlignment::edge, 0);
    // Centre-aligned with the same ATRLR, an update at BOTH ends of the
    // triangle: 2 x 99 = 198 counts a period gives 1010 updates in 100
    // ms, 2 x 100 = 200 gives 1000.
    const uint32_t center = updates_in_100ms(47, 99, TimAlignment::center_up, 0);
    const uint32_t per_update_x10 = center != 0u ? (100'000UL * 10u) / center : 0u;   // counts x 10
    print(serial, "  edge-aligned ATRLR 99: ", edge, " updates in 100 ms (1000 expected: ATRLR + 1 counts)",
          crlf);
    print(serial, "  centre-aligned ATRLR 99: ", center, " updates in 100 ms -> ", per_update_x10 / 10u, ".",
          per_update_x10 % 10u, " counts an update, two updates a period (2 x ATRLR = 198 -> 1010, "
          "2 x (ATRLR + 1) = 200 -> 1000)", crlf);
    bench.verdict("an edge-aligned period is ATRLR + 1 counts (1000 +- 5 updates)", edge >= 995u && edge <= 1005u);
    const bool two_arr = center >= 1008u && center <= 1012u;
    const bool two_arr_plus = center >= 998u && center <= 1002u;
    bench.verdict("a centre-aligned period was measured and is one of the two the arithmetic allows",
                  two_arr || two_arr_plus);
    print(serial, "  -> the centre-aligned period is ", two_arr ? "2 x ATRLR" : "2 x (ATRLR + 1)",
          " counts on this silicon", crlf);
    // The repetition counter: RPTCR 4 -> one update every 5 periods.
    const uint32_t rep = updates_in_100ms(47, 99, TimAlignment::edge, 4);
    print(serial, "  edge-aligned with RPTCR 4: ", rep, " updates (200 expected)", crlf);
    bench.verdict("the repetition counter divides the updates by RPTCR + 1", rep >= 199u && rep <= 201u);
    all_off();
}

// ===========================================================================
// d - one timer measuring another
// ===========================================================================

void td_cross() {
    all_off();
    // TIM1 updates at 10 kHz (PSC 47, ATRLR 99) on TRGO; TIM2 counts them.
    (void)T1::configure({.prescaler = 47, .period = 99});
    (void)T1::master(TimMasterMode::update);
    (void)TimEventCounter<T2>::setup(TimTrigger::itr0);
    T1::enable(true);
    spin_cycles(cycles_100ms);
    const uint16_t counted = TimEventCounter<T2>::count();
    T1::enable(false);
    print(serial, "  TIM2 counting TIM1's update on ITR0: ", counted, " in 100 ms (1000 expected)", crlf);
    bench.verdict("TIM2 counts TIM1's updates on ITR0 - a frequency measured with no wire and no CPU",
                  counted >= 998u && counted <= 1002u);

    // The other way: TIM2 updates at 5 kHz, TIM1 counts on ITR1.
    all_off();
    (void)T2::configure({.prescaler = 47, .period = 199});
    (void)T2::master(TimMasterMode::update);
    (void)TimEventCounter<T1>::setup(TimTrigger::itr1);
    T2::enable(true);
    spin_cycles(cycles_100ms);
    const uint16_t counted2 = TimEventCounter<T1>::count();
    T2::enable(false);
    print(serial, "  TIM1 counting TIM2's update on ITR1: ", counted2, " in 100 ms (500 expected)", crlf);
    bench.verdict("and TIM1 counts TIM2's on ITR1 - table 11-2's two links both live",
                  counted2 >= 498u && counted2 <= 502u);

    // Gated: TIM1's OC1REF (a 25% PWM, no pad) gates TIM2's own clock.
    all_off();
    (void)T1::configure({.prescaler = 0, .period = 999});   // 48 kHz
    (void)T1::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 250, .enable = false});
    (void)T1::master(TimMasterMode::oc1ref);
    (void)TimGatedCounter<T2>::setup(TimTrigger::itr0, 0);   // counts HCLK while gated open
    T1::enable(true);
    spin_cycles(480'000UL);   // 10 ms
    T1::enable(false);
    T2::enable(false);
    const uint16_t gated = TimGatedCounter<T2>::count();
    // 10 ms at 48 MHz is 480000 cycles, a quarter of which is 120000: the
    // 16-bit counter wraps, so the ratio is read from the count folded
    // once per 65536 - 120000 mod 65536 = 54464.
    print(serial, "  TIM2 gated by TIM1's OC1REF at 25% for 10 ms: count ", gated, " (120000 mod 65536 = 54464)",
          crlf);
    bench.verdict("a duty cycle measured INTERNALLY: the gated count is a quarter of the window within 1%",
                  gated >= 54464u - 1200u && gated <= 54464u + 1200u);
    all_off();
}

// ===========================================================================
// e - TIM3
// ===========================================================================

#if BRIO_CH32_HAS_TIM3
void te_tim3() {
    all_off();
    // TIM1's TRGO at 10 kHz clocks TIM3 (external clock mode 1).
    (void)T1::configure({.prescaler = 47, .period = 99});
    (void)T1::master(TimMasterMode::update);
    (void)Tim3::configure({.period = 0xFFFF, .clocked_by_tim1 = true});
    Tim3::enable(true);
    T1::enable(true);
    spin_cycles(cycles_100ms);
    const uint16_t counted = Tim3::count();
    T1::enable(false);
    Tim3::enable(false);
    print(serial, "  TIM3 clocked by TIM1's trigger: ", counted, " in 100 ms (1000 expected)", crlf);
    bench.verdict("TIM3 counts TIM1's trigger in external clock mode 1", counted >= 998u && counted <= 1002u);

    // TIM3 on CK_INT (48 MHz, no prescaler): 0xFFFF + 1 counts a period.
    (void)Tim3::configure({.period = 47999});   // 1 kHz updates
    Tim3::enable(true);
    const uint16_t a = Tim3::count();
    (void)delay_us(clock, 500);
    const uint16_t b = Tim3::count();
    Tim3::enable(false);
    print(serial, "  TIM3 on CK_INT: ", static_cast<uint16_t>(b - a), " counts in 500 us (24000 expected)", crlf);
    bench.verdict("TIM3 counts HCLK undivided (13.2.3: no prescaler)",
                  static_cast<uint16_t>(b - a) >= 23'900u && static_cast<uint16_t>(b - a) <= 24'200u);

    // Channel 3's match as a DMA request: DMA channel 1 (table 8-2) moves
    // one word per match from a source into a sink, no CPU in the path -
    // that is the chapter's promise (13.3.1). MEASURED: the request
    // serves ONE transfer after a reset of the block and never another
    // (letter x probes what might re-arm it: nothing but the RCC reset
    // pulse). The verdict records the count either way.
    static const uint32_t source = 0xC0FFEE01u;
    static volatile uint32_t sink = 0;
    uint16_t served_max = 0;
    for (uint8_t preload = 0; preload < 2u; ++preload) {
        sink = 0;
        DmaChannel<1>::stop();
        Tim3::reset();
        (void)Tim3::configure({.period = 4799});   // 10 kHz
        (void)Tim3::compare_preload(2, preload != 0u);
        (void)Tim3::set_compare(2, 100);
        (void)Tim3::dma_request(2, true);
        const bool loaded = DmaChannel<1>::load(DmaTransfer{
            .peripheral = &sink, .memory = const_cast<uint32_t*>(&source), .count = 1000,
            .config = {.direction = DmaDirection::memory_to_peripheral, .memory_to_memory = false,
                       .peripheral_increment = false, .memory_increment = false,
                       .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word}});
        Tim3::enable(true);
        const uint32_t t0 = cycles_now();
        while (!DmaChannel<1>::flag(DmaFlag::complete) && cycles_now() - t0 < 3u * cycles_100ms) {
        }
        const uint32_t took_us = (cycles_now() - t0) / 48u;
        const uint16_t left = DmaChannel<1>::count();
        Tim3::enable(false);
        (void)Tim3::dma_request(2, false);
        DmaChannel<1>::stop();
        const uint16_t served = static_cast<uint16_t>(1000u - left);
        print(serial, "  TIM3 CH3 match -> DMA channel 1 (OC3PE ", preload, "): ", served, " of 1000 requests served in ",
              took_us / 1000u, " ms (100 expected for 1000), sink=", hex(sink), crlf);
        if (loaded && served > served_max) {
            served_max = served;
        }
    }
    print(serial, "  -> ", served_max == 1u
                               ? "TIM3'S DMA REQUEST IS ONE-SHOT ON THIS SILICON: one transfer at the first "
                                 "match after a reset of the block, none after (letter x)"
                               : (served_max >= 997u ? "TIM3's match paces the channel as 13.3.1 says"
                                                     : "an unexpected count"),
          crlf);
    bench.verdict("TIM3's channel 3 match reaches DMA channel 1 (the sink holds the source) - how often "
                  "is the finding above",
                  served_max >= 1u && sink == source);
    all_off();
}

#endif

// ===========================================================================
// f - the captures on the jumper
// ===========================================================================

using Meter = TimPeriodMeter<T2>;
using Edges = TimIntervalMeter<T2, 0>;

void tf_capture() {
    if (!need_jumper()) {
        return;
    }
    all_off();
    // TIM1 CH1 on PD2: 10 kHz PWM (PSC 0, ATRLR 4799) at three duties.
    OutPad::claim();
    (void)T1::configure({.prescaler = 0, .period = 4799, .auto_reload_preload = true});
    (void)T1::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 1200});
    (void)T1::main_output(true);
    T1::enable(true);
    // TIM2 CH1 on PD4 measures it, at HCLK, both channels.
    InPad::claim_input();
    (void)Meter::setup(0, 0, false);
    T2::interrupts(Meter::period_interrupt, true);
    Pfic::enable(T2::cc_irq());
    const uint16_t duties[] = {1200, 2400, 4320};
    uint8_t exact = 0;
    for (const uint16_t d : duties) {
        (void)T1::set_compare(0, d);
        settle_ms(20);
        t2_captures = 0;
        settle_ms(20);
        const uint16_t period = last_period;
        const uint16_t width = last_width;
        const bool ok = period >= 4799u && period <= 4801u && width >= d - 2u && width <= d + 2u;
        print(serial, "  duty ", d, "/4800: period ", period, " width ", width, " (", t2_captures,
              " captures in 20 ms)", ok ? "  exact" : "  OFF", crlf);
        if (ok) {
            ++exact;
        }
    }
    Pfic::disable(T2::cc_irq());
    bench.verdict("TimPeriodMeter reads TIM1's period and high time to the count at three duties, "
                  "through the jumper",
                  exact == 3u);
    bench.verdict("about 200 captures in 20 ms at 10 kHz", t2_captures >= 195u && t2_captures <= 205u);

    // A CHCVR read clears the channel's flag (11.4.5): what lets a
    // poller wait on the flag and take the value in one verb.
    (void)Edges::setup(0, 0);
    T2::clear_flags(Edges::capture_flag);
    while (!T2::flag(Edges::capture_flag)) {
    }
    (void)T2::compare(0);
    const bool read_cleared = !T2::flag(Edges::capture_flag);
    bench.verdict("reading CHCVR clears the channel's capture flag", read_cleared);
    // The interval meter: rising edges 4800 apart, on the free-running
    // counter Edges::setup's configure() re-established after the
    // period meter's reset-on-TI1.
    Edges::restart();
    uint8_t good = 0;
    uint16_t lo = 0xFFFFu;
    uint16_t hi = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        while (!T2::flag(Edges::capture_flag)) {
        }
        if (const auto d = Edges::interval()) {
            if (*d >= 4799u && *d <= 4801u) {
                ++good;
            }
            lo = *d < lo ? *d : lo;
            hi = *d > hi ? *d : hi;
        }
    }
    print(serial, "  TimIntervalMeter: ", good, " of 7 intervals at 4800 counts (", lo, "..", hi,
          "; the first has no predecessor)", crlf);
    bench.verdict("TimIntervalMeter reads the period between rising edges", good == 7u);
    all_off();
}

// ===========================================================================
// g - one pulse on the jumper
// ===========================================================================

void tg_one_pulse() {
    if (!need_jumper()) {
        return;
    }
    all_off();
    OutPad::claim();
    InPad::claim_input();
    // A 100 us pulse after a 50 us delay, at 1 MHz (PSC 47).
    (void)TimOnePulse<T1, 0>::setup(47, 50, 100);
    // TIM2 CH1 captures the rising edge, CH2 the falling one (PWM input
    // mode without the slave reset: the difference is the width).
    (void)T2::configure({.prescaler = 47, .period = 0xFFFF});
    (void)T2::capture_channel(0, {.select = TimChannelSelect::direct, .polarity = TimCapturePolarity::rising});
    (void)T2::capture_channel(1, {.select = TimChannelSelect::indirect, .polarity = TimCapturePolarity::falling});
    T2::clear_flags(T2::compare_flag(0) | T2::compare_flag(1));
    T2::enable(true);
    TimOnePulse<T1, 0>::fire();
    settle_ms(5);
    const bool rose = T2::flag(T2::compare_flag(0));
    const bool fell = T2::flag(T2::compare_flag(1));
    const uint16_t up = T2::compare(0);
    const uint16_t down = T2::compare(1);
    const uint16_t width = static_cast<uint16_t>(down - up);
    print(serial, "  one pulse: rose ", rose ? "yes" : "no", " fell ", fell ? "yes" : "no", " width ", width,
          " us (100 asked), the timer ", TimOnePulse<T1, 0>::busy() ? "STILL RUNNING" : "stopped itself", crlf);
    bench.verdict("one pulse of 100 us fired by software, captured within a microsecond",
                  rose && fell && width >= 99u && width <= 101u);
    bench.verdict("and the counter stopped itself at the update (OPM)", !TimOnePulse<T1, 0>::busy());
    all_off();
}

// ===========================================================================
// i - the break, wireless
// ===========================================================================

void ti_break() {
    all_off();
    // A PWM on TIM1 with MOE, the break input on PC2 active HIGH, the pad
    // pulled DOWN: no break.
    BreakPad::claim_input(PinPull::down);
    (void)T1::configure({.prescaler = 47, .period = 999});
    (void)T1::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500, .enable = false});
    (void)T1::break_dead_time({.main_output_enable = true, .automatic_output_enable = false, .break_enable = true,
                               .break_active_high = true});
    T1::clear_flags(T1::break_flag);
    T1::interrupts(T1::break_interrupt, true);
    Pfic::enable(T1::break_irq());
    T1::enable(true);
    settle_ms(5);
    const bool moe_before = T1::main_output();
    const uint32_t breaks_before = t1_breaks;
    // Now pull the pad UP: the break input goes active.
    BreakPad::claim_input(PinPull::up);
    settle_ms(5);
    const bool moe_after = T1::main_output();
    const uint32_t breaks_after = t1_breaks;
    print(serial, "  BKIN pulled down: MOE=", moe_before, " breaks=", breaks_before, "; pulled up: MOE=",
          moe_after, " breaks=", breaks_after, crlf);
    bench.verdict("with the pad low the outputs run (MOE set, no break)", moe_before && breaks_before == 0u);
    bench.verdict("the pad's own pull-up is a break: MOE dropped by hardware and BIF raised (the "
                  "handler silencing itself, since BIF cannot clear while the input stands)",
                  !moe_after && breaks_after == 1u);
    print(serial, "  BIF with the input still active: ", T1::flag(T1::break_flag) ? "STANDS (cannot be cleared)"
                                                                                  : "cleared", crlf);
    // Pulled down again with AOE: MOE comes back at the next update by
    // itself.
    Pfic::disable(T1::break_irq());
    (void)T1::break_dead_time({.main_output_enable = false, .automatic_output_enable = true, .break_enable = true,
                               .break_active_high = true});
    BreakPad::claim_input(PinPull::down);
    settle_ms(5);
    print(serial, "  pulled down under AOE: MOE=", T1::main_output(), crlf);
    bench.verdict("under AOE the outputs come back by themselves once the break clears", T1::main_output());
    all_off();
}

// ===========================================================================
// x - TIM3's DMA request, probed (outside z)
// ===========================================================================

#if BRIO_CH32_HAS_TIM3
void tx_tim3_dma_probe() {
    all_off();
    static const uint32_t source = 0xC0FFEE01u;
    static volatile uint32_t sink = 0;
    struct Variant {
        const char* name;
        uint8_t ch;
        uint16_t period;
        uint16_t compare;
        bool by_tim1;
    };
    const Variant variants[] = {
        {"ch3 cmp 100 of 4799", 2, 4799, 100, false},
        {"ch4 cmp 100 of 4799", 3, 4799, 100, false},
        {"ch3 cmp 4799 of 4799 (the top)", 2, 4799, 4799, false},
        {"ch3 cmp 0 of 4799", 2, 4799, 0, false},
        {"ch3 cmp 50 of 99, clocked by TIM1 at 10 kHz", 2, 99, 50, true},
    };
    for (const auto& v : variants) {
        all_off();
        if (v.by_tim1) {
            (void)T1::configure({.prescaler = 47, .period = 99});
            (void)T1::master(TimMasterMode::update);
            T1::enable(true);
        }
        sink = 0;
        (void)Tim3::configure({.period = v.period, .clocked_by_tim1 = v.by_tim1});
        (void)Tim3::set_compare(v.ch, v.compare);
        (void)Tim3::dma_request(v.ch, true);
        using Ch1 = DmaChannel<1>;
        using Ch4 = DmaChannel<4>;
        const DmaTransfer t{.peripheral = &sink, .memory = const_cast<uint32_t*>(&source), .count = 1000,
                            .config = {.direction = DmaDirection::memory_to_peripheral,
                                       .peripheral_increment = false, .memory_increment = false,
                                       .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word}};
        if (v.ch == 2u) { Ch1::stop(); (void)Ch1::load(t); } else { Ch4::stop(); (void)Ch4::load(t); }
        Tim3::enable(true);
        spin_cycles(cycles_100ms);
        const uint16_t left = v.ch == 2u ? Ch1::count() : Ch4::count();
        const uint16_t dmaen = Tim3::regs().DMAINTENR;
        const uint16_t cnt = Tim3::count();
        // Re-arm the request bit and look again: a self-clearing enable
        // would serve one more.
        (void)Tim3::dma_request(v.ch, false);
        (void)Tim3::dma_request(v.ch, true);
        spin_cycles(cycles_100ms);
        const uint16_t left2 = v.ch == 2u ? Ch1::count() : Ch4::count();
        Tim3::enable(false);
        print(serial, "  ", v.name, ": ", 1000u - left, " served in 100 ms, DMAINTENR=", hex(dmaen), " CNT=", cnt,
              "; after re-arming CCxDE: ", 1000u - left2, " total", crlf);
    }
    // The transfer aimed at the timer's OWN compare register - the F1
    // lineage clears a channel's event by an access to CHxCVR, and a
    // request that is never cleared may be one the controller serves
    // once.
    for (uint8_t which = 0; which < 2u; ++which) {
        all_off();
        static const uint16_t value = 100;
        using Ch1 = DmaChannel<1>;
        Ch1::stop();
        volatile void* target = nullptr;
        if (which == 0u) {
            (void)Tim3::configure({.period = 4799});
            (void)Tim3::set_compare(2, 100);
            (void)Tim3::dma_request(2, true);
            target = &Tim3::regs().CH3CVR;
        } else {
            (void)T2::configure({.prescaler = 0, .period = 4799});
            (void)T2::output_channel(2, {.mode = TimOutputMode::frozen, .compare = 100, .enable = false});
            T2::interrupts(T2::compare_dma(2), true);
            target = T2::ccr_address(2);
        }
        (void)Ch1::load(DmaTransfer{.peripheral = target, .memory = const_cast<uint16_t*>(&value), .count = 1000,
                                    .config = {.direction = DmaDirection::memory_to_peripheral,
                                               .peripheral_increment = false, .memory_increment = false,
                                               .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::half}});
        if (which == 0u) { Tim3::enable(true); } else { T2::enable(true); }
        spin_cycles(cycles_100ms);
        const uint16_t left = Ch1::count();
        print(serial, "  ", which == 0u ? "TIM3 ch3" : "TIM2 ch3", " match -> DMA ch1 writing the timer's own CHxCVR: ",
              1000u - left, " served in 100 ms", crlf);
    }
    // And TIM2's request aimed at a plain sink: served once (an edge
    // the controller latched) or for ever (a level held high)?
    {
        all_off();
        static const uint16_t value2 = 100;
        static volatile uint16_t sink2 = 0;
        using Ch1 = DmaChannel<1>;
        Ch1::stop();
        (void)T2::configure({.prescaler = 0, .period = 4799});
        (void)T2::output_channel(2, {.mode = TimOutputMode::frozen, .compare = 100, .enable = false});
        T2::interrupts(T2::compare_dma(2), true);
        (void)Ch1::load(DmaTransfer{.peripheral = &sink2, .memory = const_cast<uint16_t*>(&value2), .count = 1000,
                                    .config = {.direction = DmaDirection::memory_to_peripheral,
                                               .peripheral_increment = false, .memory_increment = false,
                                               .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::half}});
        T2::enable(true);
        spin_cycles(cycles_100ms);
        const uint16_t left = Ch1::count();
        const bool cc3if = T2::flag(T2::compare_flag(2));
        print(serial, "  TIM2 ch3 match -> DMA ch1 writing a plain sink: ", 1000u - left, " served in 100 ms, CC3IF ",
              cc3if ? "standing" : "clear", crlf);
        // Cleared by software every period: does the request come back?
        T2::clear_flags(T2::compare_flag(2));
        const uint16_t before = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);   // one period
            T2::clear_flags(T2::compare_flag(2));
        }
        const uint16_t after = Ch1::count();
        print(serial, "  ... with CC3IF cleared by software every period: ", before - after, " more served in 10 periods",
              crlf);
    }
    // TIM3 again, the compare register REWRITTEN by software every
    // period: a request re-armed by the write would serve one more each
    // time. And CCxDE toggled every period, the same question.
    {
        all_off();
        static const uint32_t src3 = 0x11223344u;
        static volatile uint32_t sink3 = 0;
        using Ch1 = DmaChannel<1>;
        Ch1::stop();
        (void)Tim3::configure({.period = 4799});
        (void)Tim3::set_compare(2, 100);
        (void)Tim3::dma_request(2, true);
        (void)Ch1::load(DmaTransfer{.peripheral = &sink3, .memory = const_cast<uint32_t*>(&src3), .count = 1000,
                                    .config = {.direction = DmaDirection::memory_to_peripheral,
                                               .peripheral_increment = false, .memory_increment = false,
                                               .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word}});
        Tim3::enable(true);
        spin_cycles(48'000);
        const uint16_t a0 = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            (void)Tim3::set_compare(2, 100);
        }
        const uint16_t a1 = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            (void)Tim3::dma_request(2, false);
            (void)Tim3::dma_request(2, true);
        }
        const uint16_t a2 = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            Tim3::enable(false);
            Tim3::enable(true);
        }
        const uint16_t a3 = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            Tim3::set_count(0);
        }
        const uint16_t a4 = Ch1::count();
        volatile uint16_t sinkr = 0;
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            sinkr = Tim3::compare(2);   // a READ of the compare register
        }
        const uint16_t a5 = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            Tim3::regs().DMAINTENR = Tim3::regs().DMAINTENR;   // a write of the enable register as it stands
            Tim3::regs().CTLR1 = Tim3::regs().CTLR1;
        }
        const uint16_t a6 = Ch1::count();
        for (uint8_t k = 0; k < 10u; ++k) {
            spin_cycles(4800);
            Tim3::reset();   // the RCC pulse, then the same configuration again
            (void)Tim3::configure({.period = 4799});
            (void)Tim3::set_compare(2, 100);
            (void)Tim3::dma_request(2, true);
            Tim3::enable(true);
        }
        const uint16_t a7 = Ch1::count();
        Tim3::enable(false);
        (void)sinkr;
        print(serial, "  TIM3 ch3 -> DMA: ", 1000u - a0, " at the start; +", a0 - a1, " with CH3CVR rewritten each "
              "period; +", a1 - a2, " with CC3DE toggled; +", a2 - a3, " with CEN toggled; +", a3 - a4,
              " with CNT zeroed; +", a4 - a5, " with CH3CVR read; +", a5 - a6, " with CTLR1/DMAINTENR rewritten; +",
              a6 - a7, " with the RCC reset pulse and a reconfiguration", crlf);
    }
    // The control: the same block loaded with NO request enabled anywhere.
    {
        all_off();
        static const uint32_t src4 = 0x55AA55AAu;
        static volatile uint32_t sink4 = 0;
        using Ch1 = DmaChannel<1>;
        using Ch4 = DmaChannel<4>;
        Ch1::stop();
        Ch4::stop();
        const DmaTransfer t{.peripheral = &sink4, .memory = const_cast<uint32_t*>(&src4), .count = 1000,
                            .config = {.direction = DmaDirection::memory_to_peripheral,
                                       .peripheral_increment = false, .memory_increment = false,
                                       .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word}};
        (void)Ch1::load(t);
        spin_cycles(48'000);
        const uint16_t l1 = Ch1::count();
        Ch1::stop();
        sink4 = 0;
        (void)Ch4::load(t);
        spin_cycles(48'000);
        const uint16_t l4 = Ch4::count();
        Ch4::stop();
        print(serial, "  CONTROL, no request enabled anywhere: DMA ch1 served ", 1000u - l1, ", ch4 served ", 1000u - l4,
              " (sink=", hex(sink4), ")", crlf);
    }
    all_off();
    print(serial, "  (a probe: no verdict)", crlf);
}
#endif

void banner() {
    print(serial, crlf, "test_ch32_tim - ", device::part_name, " TIM1, TIM2 and TIM3 (RM ch. 11, 12, 13)", crlf);
    print(serial, "  the jumper for f and g: PD2 (TIM1_CH1) <-> PD4 (TIM2_CH1); ", jumper_present ? "PRESENT"
                                                                                                      : "ABSENT",
          crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void tim1_up_handler() {
    if ((T1::isr() & T1::update_flag) != 0u) {
        t1_updates = t1_updates + 1u;
    }
}
// BIF cannot be cleared while the break input is still active (the
// F1 lineage's rule), so a level held on the pad would re-enter this
// handler for ever: it counts once and silences its own enable.
extern "C" BRIO_CH32_INTERRUPT void tim1_brk_handler() {
    if ((T1::isr() & T1::break_flag) != 0u) {
        t1_breaks = t1_breaks + 1u;
        T1::interrupts(T1::break_interrupt, false);
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim2_handler() {
    const uint16_t f = T2::isr();
    if ((f & T2::update_flag) != 0u) {
        t2_updates = t2_updates + 1u;
    }
    if ((f & Meter::period_flag) != 0u) {
        last_period = Meter::period_ticks();
        last_width = Meter::width_ticks();
        t2_captures = t2_captures + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();
    jumper_present = probe_jumper();

    bench.letter('a', "the blocks, wireless: reset values, the facts as refusals, the arithmetic", ta_blocks);
    bench.letter('b', "the time base against the STK, and a 1 kHz tick counted", tb_timebase);
    bench.letter('c', "the periods: edge, centre-aligned measured, the repetition counter", tc_periods);
    bench.letter('d', "one timer measuring another with no wire: ITR both ways, gated duty", td_cross);
#if BRIO_CH32_HAS_TIM3
    bench.letter('e', "TIM3: clocked by TIM1, on CK_INT, its match pacing a DMA channel", te_tim3);
#endif
    bench.letter('f', "THE CAPTURES on the jumper: period meter at three duties, interval meter", tf_capture);
    bench.letter('g', "ONE PULSE on the jumper, fired by software and captured", tg_one_pulse);
    bench.letter('i', "the break, wireless: the pad's pull as BKIN, MOE dropped, AOE", ti_break);
#if BRIO_CH32_HAS_TIM3
    bench.letter('x', "TIM3's DMA request probed five ways (no verdict)", tx_tim3_dma_probe, false);
#endif

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
