// test_ch32_adc - the reference bench suite for the CH32V00x's ADC:
// ch32v00x/adc.hpp over RM ch. 9 of each part, measured with NO WIRE -
// the internal reference as the known voltage, the pads' own pulls as
// two more, the timers as triggers, the DMA as the reader, and on the
// CH32V006 the watchdog that resets the chip judged at the next boot.
// Every count is spoken as a fraction of the part's full scale (12 bits
// on the CH32V006, 10 on the CH32V003), so one source judges both.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. PD2 (ADC_IN3) and PC4 (ADC_IN2) are read through
// their internal pulls; PD5/PD6 are the console's and are never claimed;
// PC0 is the LED.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reset values, the prescaler's two-level
//      code, the conversion arithmetic, the refusals
//   b  VREFINT (channel 8): the supply derived from the 1.2 V reference
//      at every sample time, judged against the PVD's own bracket of
//      the supply (the one independent witness a wireless board has),
//      its spread over 64 readings, and channel 9 - the OPA's output
//      for the record on the CH32V006, Vcal at 2/4 and 3/4 of AVDD
//      judged on the CH32V003
//   c  THE RATE: continuous conversions of VREFINT into a 256-word DMA
//      block at three sample times and two clocks, timed on the STK
//      against tCONV = sample + the part's tail (12.5 or 11 ADCCLK)
//   d  THE TRIGGERS: TIM1's TRGO at 10 kHz pacing the rule group (1000
//      conversions in 100 ms through the DMA), TIM2's TRGO, TIM1's CC1
//      and CC2; the INJECTION group paced by TIM3's CC1 where there is
//      a TIM3 and by TIM2's CC3 otherwise (JEOC counted)
//   e  THE INJECTION GROUP: four slots of VREFINT with four offsets,
//      the signed results, the automatic injection after the rule
//      group, the alignment
//   f  THE WATCHDOGS: watchdog 0 around VREFINT's count (inside: quiet;
//      outside: AWD and its interrupt), then on the CH32V006 the
//      WATCHDOG SCAN - one watchdog per rank of a scanned sequence, the
//      three exercised - and on the CH32V003 the refusal of 1 and 2
//   x  (outside z, CH32V006) the probe that found the scan's semantics
//   y  (outside z) THE STALL hunted: a triggered, DMA-served run that
//      stops converting, in six arrangements of a hundred runs
//   g  THE PADS through their pulls: IN3 (PD2) and IN2 (PC4) pulled up
//      read near full scale, pulled down near zero
//   h  THE SAMPLER: util/analog_sampler.hpp's AnalogSampler walking
//      VREFINT and IN3, its AnalogSample events counted per input
//   r  (CH32V006) THE WATCHDOG RESET (reboots the board): watchdog 0
//      armed with AWD0_RST_EN and VREFINT out of its window; the next
//      boot reads ADCRSTF
//
// build: boards = v006k8,v003f4
// build: groups = abgef,cd,hy
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32v00x/adc.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/reset.hpp"
#include "ch32v00x/sleep.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "util/analog_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

// The token letter r leaves for the next boot (.noinit, inline, volatile
// - test_ch32_watchdog says why).
inline constexpr uint16_t token_magic = 0x5A08;
struct Token {
    uint16_t magic;
    uint8_t letter;
    uint8_t leg;
    uint16_t pass;
    uint16_t fail;
    uint16_t counts;
};
[[gnu::section(".noinit")]] inline volatile Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

using In3 = AnalogIn<Pin<'D', 2>>;
using In2 = AnalogIn<Pin<'C', 4>>;
using T1 = Tim<1>;
using T2 = Tim<2>;
using Reader = DmaChannel<1>;

TestBench<Serial> bench;

uint32_t boot_flags = 0;
volatile uint32_t adc_interrupts = 0;
volatile uint32_t awd_interrupts = 0;
volatile uint32_t jeoc_interrupts = 0;
uint16_t block[256];

constexpr uint8_t code_div8 = 0x18;    // 6 MHz on both parts
/// The fastest legal ADCCLK: /2 = 24 MHz on the CH32V006, /4 = 12 MHz
/// on the CH32V003 (the datasheet's bound).
constexpr uint8_t code_fast = adc_prescaler_for(SysClock::hz, adc_max_clock_hz)->code;
constexpr uint32_t fast_hz = adc_clock_hz(SysClock::hz, code_fast);
/// The middle of either sample-time table: 28.5 cycles on the CH32V006,
/// 30 on the CH32V003.
constexpr AdcSampleTime sample_mid = static_cast<AdcSampleTime>(3);
/// A count spoken at 12 bits, scaled to the part's full scale.
constexpr uint16_t scaled(uint16_t counts12) {
    return static_cast<uint16_t>((static_cast<uint32_t>(counts12) * adc_steps) / 4096u);
}
constexpr int16_t scaled_tolerance(uint16_t counts12) {
    return scaled(counts12) < 4u ? 4 : static_cast<int16_t>(scaled(counts12));
}

/// The console's transmitter perturbs the readings (measured: spikes of
/// 40..60 counts on VREFINT while a line leaves the ring), so every
/// measurement waits for the ring to empty and the last byte to go.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

uint32_t cycles_now() {
    const uint32_t period = stk()->CMP + 1u;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t cnt = stk()->CNT;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * period + cnt;
        }
    }
}

void spin_cycles(uint32_t cycles) {
    const uint32_t s0 = cycles_now();
    while (cycles_now() - s0 < cycles) {
    }
}
constexpr uint32_t cycles_100ms = 4'800'000UL;

void all_off() {
    Pfic::disable(Adc::irq());
    Pfic::disable(Irq::dma1_channel1);
    Reader::stop();
    T1::init();
    T2::init();
#if BRIO_CH32_HAS_TIM3
    Tim3::init();
#endif
    adc_interrupts = 0;
    awd_interrupts = 0;
    jeoc_interrupts = 0;
}

/// A fresh converter at /8 (6 MHz), VREFINT selected at the longest
/// sample time.
bool adc_ready(uint8_t prescaler = code_div8, AdcConfig extra = {}) {
    all_off();
    extra.prescaler_code = prescaler;
    if (!Adc::init(clock, extra)) {
        return false;
    }
    Adc::sample_time_all(adc_sample_longest);
    Adc::select(AdcInput::vrefint);
    return true;
}

/// `n` conversions of VREFINT into `block` through DMA channel 1, the
/// converter continuous. Returns the cycles the block took, 0 if it
/// never completed.
uint32_t dma_block(uint16_t n, AdcSampleTime t) {
    Reader::stop();
    Adc::sample_time(Adc::vrefint_channel, t);
    Adc::select(AdcInput::vrefint);
    Adc::dma(true);
    Adc::continuous(true);
    (void)Reader::load(DmaTransfer{
        .peripheral = Adc::data_address(), .memory = block, .count = n,
        .config = {.direction = DmaDirection::peripheral_to_memory, .peripheral_increment = false,
                   .memory_increment = true, .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::half}});
    Adc::clear_flags(AdcFlag::all);
    console_drain();
    const uint32_t t0 = cycles_now();
    Adc::start();
    uint32_t took = 0;
    while (!Reader::flag(DmaFlag::complete)) {
        if (cycles_now() - t0 > 48'000'000UL) {
            break;
        }
    }
    if (Reader::flag(DmaFlag::complete)) {
        took = cycles_now() - t0;
    }
    Adc::continuous(false);
    Adc::dma(false);
    Reader::stop();
    // The converter is mid-sequence in continuous mode: a power cycle
    // ends it cleanly.
    Adc::power(false);
    Adc::power(true);
    return took;
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    all_off();
    Adc::bus_clock(true);
    Adc::reset();
    print(serial, "  reset: STATR=", hex(Adc::regs().STATR & 0x1Fu), " CTLR1=", hex(Adc::regs().CTLR1), " CTLR2=",
          hex(Adc::regs().CTLR2), device::adc_has_ctlr3 ? " CTLR3=" : " DLYR=", hex(Adc::regs().CTLR3_DLYR),
          " WDHTR=", hex(Adc::regs().WDHTR), crlf);
    if constexpr (device::adc_has_ctlr3) {
        bench.verdict("the reset values are table 9-7's (CTLR3's ADC_LP set, the thresholds open)",
                      Adc::regs().CTLR1 == 0u && Adc::regs().CTLR2 == 0u && (Adc::regs().CTLR3_DLYR & adc_lp) != 0u &&
                          Adc::regs().WDHTR == adc_max_count && Adc::regs().WDTR1 == 0x0FFF0000u);
    } else {
        bench.verdict("the reset values are table 9-7's (CALVOL at 01, the threshold open at 10 bits, no delay)",
                      Adc::regs().CTLR1 == (1UL << 25) && Adc::regs().CTLR2 == 0u &&
                          Adc::regs().WDHTR == adc_max_count && Adc::regs().CTLR3_DLYR == 0u);
    }
    bench.verdict("the prescaler's two-level code: /2 at 0x00, /8 at 0x18, /12 at 0x14, /128 at 0x1F, and the "
                  "chooser picks /2 for 24 MHz",
                  adc_prescaler_divider(0x18) == 8u && adc_prescaler_divider(0x14) == 12u &&
                      adc_prescaler_divider(0x1F) == 128u &&
                      adc_prescaler_for(SysClock::hz, 24'000'000UL)->code == 0x00u);
    // The bound on ADCCLK is exercised where a code can exceed it: /2 at
    // 48 MHz is 24 MHz, above the CH32V003's 12 and under the CH32V006's 48.
    constexpr bool bound_reachable = adc_clock_hz(SysClock::hz, 0x00) > adc_max_clock_hz;
    bench.verdict("the refusals: an ADCCLK above the part's bound (where a code can exceed it), a "
                  "discontinuous length of 9, JAUTO with a discontinuous group, a watchdog with low above high "
                  "or beyond the full scale, PC0 as an input",
                  (!bound_reachable || !Adc::init(clock, {.prescaler_code = 0x00})) &&
                      !adc_config_valid({.discontinuous = 9}) &&
                      !adc_config_valid({.auto_injected = true, .discontinuous = 1}) &&
                      !adc_watchdog_config_valid({.low = 10, .high = 5}) &&
                      !adc_watchdog_config_valid({.high = adc_max_count + 1u}) && adc_channel_of(Pad{'C', 0}) == 0xFFu);
    const bool up = Adc::init(clock, {.prescaler_code = code_fast});
    print(serial, "  init at /", adc_prescaler_divider(code_fast), ": ADCCLK ", Adc::adcclk_hz(clock) / 1000u,
          " kHz, CTLR2=", hex(Adc::regs().CTLR2), " ADON=", Adc::powered(), crlf);
    bench.verdict("init() brings the converter up at the part's fastest legal ADCCLK with both software "
                  "triggers armed",
                  up && Adc::adcclk_hz(clock) == fast_hz && Adc::powered() &&
                      (Adc::regs().CTLR2 & (adc_exttrig | adc_jexttrig)) == (adc_exttrig | adc_jexttrig));
    if constexpr (device::adc_has_calibration) {
        // The calibration init() ran: RSTCAL and CAL both cleared by the
        // hardware, the code it left in RDATAR printed for the record.
        print(serial, "  the calibration ran at init: CTLR2=", hex(Adc::regs().CTLR2), " RDATAR=",
              hex(Adc::regs().RDATAR), crlf);
        bench.verdict("the calibration completes on its own (RSTCAL and CAL cleared by the hardware) and runs "
                      "again on demand",
                      (Adc::regs().CTLR2 & (adc_cal | adc_rstcal)) == 0u && Adc::calibrate());
    }
    bench.verdict("tCONV: the shortest sample plus the part's tail, the longest plus the same",
                  adc_conversion_half_cycles(adc_sample_shortest) ==
                          adc_sample_half_cycles(adc_sample_shortest) + adc_conversion_tail_half_cycles &&
                      adc_conversion_half_cycles(adc_sample_longest) == 504u);
    all_off();
}

// ===========================================================================
// b - VREFINT
// ===========================================================================

void tb_vrefint() {
    (void)adc_ready(code_div8);
    // The supply from the reference, at every sample time.
    uint16_t vdd_min = 0xFFFF;
    uint16_t vdd_max = 0;
    uint16_t counts[8];
    console_drain();
    for (uint8_t t = 0; t < 8u; ++t) {
        (void)Adc::sample_time(Adc::vrefint_channel, static_cast<AdcSampleTime>(t));
        counts[t] = Adc::read_settled(4);
        const uint16_t vdd = Adc::supply_mv(counts[t]);
        if (vdd < vdd_min) { vdd_min = vdd; }
        if (vdd > vdd_max) { vdd_max = vdd; }
    }
    print(serial, "  VREFINT counts by sample time (the console quiet):");
    for (uint8_t t = 0; t < 8u; ++t) {
        print(serial, " ", counts[t], "(", Adc::supply_mv(counts[t]), "mV)");
    }
    print(serial, crlf);
    // THE PVD AS THE WITNESS: the supply lies between the falling edge of
    // the highest level that reads not low and the rising edge of the
    // first that reads low (open at the top when none does - a 3.3 V
    // board on the CH32V006, whose ladder ends at 2.66 V). A wireless
    // board has no other independent measure of its own VDD, and a
    // board's supply is not a constant: the CH32V003 board runs from
    // the probe's 3V3 through a cable, whose drop has been seen to
    // take it a quarter of a volt below nominal.
    uint32_t floor_mv = 0;
    uint32_t ceiling_mv = 0;
    for (uint8_t code = 0; code <= static_cast<uint8_t>(pvd_level_highest); ++code) {
        const PvdLevel level = static_cast<PvdLevel>(code);
        Pwr::pvd(true, level);
        (void)delay_us(clock, 100);
        if (Pwr::supply_low()) {
            if (ceiling_mv == 0u) { ceiling_mv = pvd_rising_mv(level); }
        } else {
            floor_mv = pvd_falling_mv(level);
        }
    }
    Pwr::pvd(false);
    print(serial, "  the PVD brackets the supply: above ", floor_mv, " mV");
    if (ceiling_mv != 0u) {
        print(serial, ", below ", ceiling_mv, " mV", crlf);
    } else {
        print(serial, ", no level reads low", crlf);
    }
    bench.verdict("the supply derived from the 1.2 V reference lies inside the PVD's bracket at every sample "
                  "time (the reference's 2.5% tolerance allowed)",
                  vdd_max + vdd_max / 40u >= floor_mv && (ceiling_mv == 0u || vdd_min <= ceiling_mv + ceiling_mv / 40u));
    bench.verdict("and the eight sample times agree within 2% (a settled reference)",
                  static_cast<uint16_t>(vdd_max - vdd_min) <= vdd_max / 50u);
    // Two probes for the record: the LED's own milliamps on the supply,
    // and (where the part calibrates) the reading with the calibration
    // register reset.
    Led::clear();
    console_drain();
    const uint16_t led_off = Adc::read_settled(4);
    Led::set();
    console_drain();
    const uint16_t led_on = Adc::read_settled(4);
    print(serial, "  VREFINT with the LED off ", led_off, " (", Adc::supply_mv(led_off), " mV), on ", led_on, " (",
          Adc::supply_mv(led_on), " mV)", crlf);
    if constexpr (device::adc_has_calibration) {
        Adc::regs().CTLR2 |= adc_rstcal;
        for (uint32_t i = 0; i < 100'000u && (Adc::regs().CTLR2 & adc_rstcal) != 0u; ++i) {
        }
        console_drain();
        const uint16_t uncalibrated = Adc::read_settled(4);
        const bool recal = Adc::calibrate();
        console_drain();
        const uint16_t recalibrated = Adc::read_settled(4);
        print(serial, "  VREFINT with the calibration register reset ", uncalibrated, ", calibrated again ",
              recalibrated, " (", recal ? "ok" : "CAL never cleared", ")", crlf);
    }

    // The spread over 64 readings at the longest sample time, the
    // console drained first (measured: 49 counts of spread with a line
    // still leaving, 6 with the console quiet).
    console_drain();
    (void)Adc::sample_time(Adc::vrefint_channel, adc_sample_longest);
    uint16_t lo = 0xFFFF;
    uint16_t hi = 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 64u; ++i) {
        const uint16_t c = Adc::read();
        sum += c;
        if (c < lo) { lo = c; }
        if (c > hi) { hi = c; }
    }
    const uint16_t mean = static_cast<uint16_t>(sum / 64u);
    print(serial, "  64 readings at the longest sample: mean ", mean, " (", Adc::supply_mv(mean), " mV), min ", lo,
          " max ", hi, " (", hi - lo, " counts of spread)", crlf);
    bench.verdict("the spread over 64 readings with the console quiet is within 16 counts of 4096",
                  static_cast<uint16_t>(hi - lo) <= scaled_tolerance(16));
    if constexpr (device::adc_has_calibration) {
        // Vcal, the CH32V003's channel 9: 2/4 of AVDD under the config in
        // force, 3/4 after a re-init that says so - a second known source.
        Adc::select_channel(9);
        const uint16_t half = Adc::read_settled(4);
        (void)adc_ready(code_div8, {.cal_voltage = AdcCalVoltage::three_quarters});
        Adc::select_channel(9);
        const uint16_t three_quarters = Adc::read_settled(4);
        print(serial, "  channel 9 (Vcal): ", half, " counts at 2/4 AVDD, ", three_quarters, " at 3/4 (", adc_steps / 2u,
              " and ", adc_steps * 3u / 4u, " expected)", crlf);
        bench.verdict("Vcal reads 2/4 and 3/4 of the full scale under CALVOL, within 3%",
                      half >= adc_steps / 2u - adc_steps * 3u / 100u && half <= adc_steps / 2u + adc_steps * 3u / 100u &&
                          three_quarters >= adc_steps * 3u / 4u - adc_steps * 3u / 100u &&
                          three_quarters <= adc_steps * 3u / 4u + adc_steps * 3u / 100u);
    } else {
        Adc::select_channel(9);
        const uint16_t opa = Adc::read_settled(2);
        print(serial, "  channel 9 (the OPA's output, off): ", opa, " counts", crlf);
        bench.verdict("channel 9 converts (the OPA off: a number, not a hang)", true);
    }
    all_off();
}

// ===========================================================================
// c - the rate
// ===========================================================================

void tc_rate() {
    struct Run {
        uint8_t prescaler;
        AdcSampleTime t;
    };
    const Run runs[] = {{code_div8, adc_sample_longest}, {code_div8, adc_sample_shortest},
                        {code_fast, sample_mid}, {code_fast, adc_sample_shortest}};
    uint8_t exact = 0;
    uint8_t settled_runs = 0;
    uint16_t fast_spread = 0;
    for (const Run& r : runs) {
        (void)adc_ready(r.prescaler);
        const uint32_t took = dma_block(256, r.t);
        const uint32_t adc_hz = Adc::adcclk_hz(clock);
        // 256 conversions: the expected cycles, in HCLK, are 256 x tCONV
        // x (HCLK / ADCCLK).
        const uint32_t expected = 256u * adc_conversion_half_cycles(r.t) * (SysClock::hz / adc_hz) / 2u;
        const uint32_t per_conv_ns = took != 0u ? (took * 1000u / 48u) / 256u : 0u;
        const bool timed = took != 0u && took >= expected - expected / 20u && took <= expected + expected / 10u + 400u;
        // The block's content: the spread of the 256 readings around the
        // reference. VREFINT is a slow source (DS table 3-5 recommends
        // slow sampling), so at the shortest sample of the fastest clock
        // the spread is the finding, not a fault.
        uint16_t lo = 0xFFFF;
        uint16_t hi = 0;
        for (uint16_t i = 0; i < 256u; ++i) {
            if (block[i] < lo) { lo = block[i]; }
            if (block[i] > hi) { hi = block[i]; }
        }
        const bool settled = static_cast<uint16_t>(hi - lo) <= scaled(200);
        print(serial, "  ADCCLK ", adc_hz / 1000000u, " MHz, sample ", adc_sample_half_cycles(r.t) / 2u,
              (adc_sample_half_cycles(r.t) & 1u) != 0u ? ".5" : "", " cycles: 256 "
              "conversions in ", took, " cycles (", expected, " expected) = ", per_conv_ns, " ns each -> ",
              per_conv_ns != 0u ? 1'000'000u / per_conv_ns : 0u, " ksps; block ", lo, "..", hi,
              timed ? "  timed" : "  OFF", settled ? ", settled" : ", NOT settled", crlf);
        if (timed) {
            ++exact;
        }
        if (settled) {
            ++settled_runs;
        }
        if (&r == &runs[3]) {
            fast_spread = static_cast<uint16_t>(hi - lo);
        }
    }
    bench.verdict("256 continuous conversions through DMA channel 1 at four clock/sample settings take "
                  "tCONV = sample + the part's tail (12.5 or 11 ADCCLK cycles) each, within 5%",
                  exact == 4u);
    bench.verdict("the three slower settings read the reference settled (the block within 200 counts of 4096)",
                  settled_runs >= 3u);
    print(serial, "  -> at the fastest clock and the shortest sample VREFINT spreads ", fast_spread,
          " counts: the reference wants the slow sampling the datasheet recommends", crlf);
    all_off();
}

// ===========================================================================
// d - the triggers
// ===========================================================================

/// `n` conversions on a hardware trigger, through the DMA: the cycles
/// the block took, 0 if it never completed in 300 ms.
uint32_t triggered_block(uint16_t n) {
    Reader::stop();
    Adc::dma(true);
    // Into ONE word: the count is the measurement, not the content.
    (void)Reader::load(DmaTransfer{
        .peripheral = Adc::data_address(), .memory = block, .count = n,
        .config = {.direction = DmaDirection::peripheral_to_memory, .peripheral_increment = false,
                   .memory_increment = false, .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::half}});
    const uint32_t t0 = cycles_now();
    while (!Reader::flag(DmaFlag::complete) && cycles_now() - t0 < 3u * cycles_100ms) {
    }
    const uint32_t took = Reader::flag(DmaFlag::complete) ? cycles_now() - t0 : 0u;
    if (took == 0u) {
        print(serial, "    stall: DMA left ", Reader::count(), ", ADC STATR=", hex(Adc::regs().STATR & 0x1Fu), crlf);
    }
    Adc::dma(false);
    Reader::stop();
    return took;
}

void td_triggers() {
    (void)adc_ready(code_div8);
    (void)Adc::sample_time(Adc::vrefint_channel, sample_mid);
    struct Src {
        const char* name;
        AdcTrigger trigger;
        uint8_t timer;
        TimMasterMode master;
        uint8_t compare_ch;   // 0xFF = TRGO
    };
    const Src sources[] = {{"TIM1 TRGO", AdcTrigger::tim1_trgo, 1, TimMasterMode::update, 0xFF},
                           {"TIM2 TRGO", AdcTrigger::tim2_trgo, 2, TimMasterMode::update, 0xFF},
                           {"TIM1 CC1", AdcTrigger::tim1_cc1, 1, TimMasterMode::reset, 0},
                           {"TIM1 CC2", AdcTrigger::tim1_cc2, 1, TimMasterMode::reset, 1},
                           {"TIM2 CC1", AdcTrigger::tim2_cc1, 2, TimMasterMode::reset, 0},
                           {"TIM2 CC2", AdcTrigger::tim2_cc2, 2, TimMasterMode::reset, 1}};
    uint8_t good = 0;
    uint8_t stalls = 0;
    for (const Src& s : sources) {
        T1::init();
        T2::init();
        // The timer at 10 kHz: an update on TRGO, or a compare on the
        // channel (its CCx event is the trigger, no pad needed).
        if (s.timer == 1u) {
            (void)T1::configure({.prescaler = 47, .period = 99});
            if (s.compare_ch != 0xFFu) {
                (void)T1::output_channel(s.compare_ch, {.mode = TimOutputMode::pwm1, .compare = 50, .enable = true});
                (void)T1::main_output(true);
            }
            (void)T1::master(s.master);
        } else {
            (void)T2::configure({.prescaler = 47, .period = 99});
            if (s.compare_ch != 0xFFu) {
                (void)T2::output_channel(s.compare_ch, {.mode = TimOutputMode::pwm1, .compare = 50, .enable = true});
            }
            (void)T2::master(s.master);
        }
        (void)Adc::trigger(s.trigger);
        Adc::recover();   // a clean converter for each source
        // THE STALL (adc.hpp's recover()): a run that stops converting is
        // recovered and tried once more; the stalls are counted and
        // reported beside the verdict.
        uint32_t took = 0;
        for (uint8_t attempt = 0; attempt < 2u && took == 0u; ++attempt) {
            Adc::clear_flags(AdcFlag::all);
            if (s.timer == 1u) { T1::enable(true); } else { T2::enable(true); }
            took = triggered_block(1000);
            T1::enable(false);
            T2::enable(false);
            if (took == 0u) {
                ++stalls;
                Adc::recover();
            }
        }
        const uint32_t ms_x10 = took / 4800u;
        const bool ok = took != 0u && ms_x10 >= 995u && ms_x10 <= 1005u;
        print(serial, "  ", s.name, " at 10 kHz: 1000 conversions in ", ms_x10 / 10u, ".", ms_x10 % 10u, " ms",
              ok ? "  paced" : "  OFF", crlf);
        if (ok) {
            ++good;
        }
    }
    (void)Adc::trigger(AdcTrigger::software);
    print(serial, "  stalls recovered along the way: ", stalls, " (the converter stopping with STRT set - "
          "adc.hpp's finding; letter y hunts it)", crlf);
    bench.verdict("all six timer triggers of table 9-3 pace the rule group at their 10 kHz (1000 conversions "
                  "in 100 ms through the DMA), a stalled run recovered by an ADON cycle and retried",
                  good == 6u);

    // The INJECTION group on a timer of its own (table 9-4): TIM3's CC1
    // where the part has a TIM3 - TIM3 on CK_INT with a match every
    // 4800 counts - and TIM2's CC3 at 10 kHz otherwise; JEOC counted in
    // the handler.
    T1::init();
    const uint8_t seq[1] = {Adc::vrefint_channel};
    (void)Adc::injected_sequence(seq, 1);
    bool injected_armed = false;
#if BRIO_CH32_HAS_TIM3
    Tim3::init();
    (void)Tim3::configure({.period = 4799});
    (void)Tim3::set_compare(0, 100);
    injected_armed = Adc::injected_trigger(AdcInjectedTrigger::tim3_cc1);
#else
    T2::init();
    (void)T2::configure({.prescaler = 47, .period = 99});
    (void)T2::output_channel(2, {.mode = TimOutputMode::pwm1, .compare = 50, .enable = true});
    injected_armed = Adc::injected_trigger(AdcInjectedTrigger::tim2_cc3);
#endif
    Adc::injected_trigger_enable(true);
    jeoc_interrupts = 0;
    Adc::clear_flags(AdcFlag::all);
    Adc::interrupts(Adc::injected_interrupt, true);
    Pfic::enable(Adc::irq());
#if BRIO_CH32_HAS_TIM3
    Tim3::enable(true);
    spin_cycles(cycles_100ms);
    Tim3::enable(false);
#else
    T2::enable(true);
    spin_cycles(cycles_100ms);
    T2::enable(false);
#endif
    Pfic::disable(Adc::irq());
    Adc::interrupts(Adc::injected_interrupt, false);
    const uint32_t jeocs = jeoc_interrupts;
    print(serial, "  ", device::has_tim3 ? "TIM3 CC1" : "TIM2 CC3", " at 10 kHz pacing the injection group: ",
          jeocs, " JEOC interrupts in 100 ms, last result ", Adc::injected_result(0), crlf);
    if constexpr (device::has_tim3) {
        bench.verdict("TIM3's channel 1 match triggers the injection group at its 10 kHz (1000 +- 5 JEOCs) - "
                      "the streamlined timer's purpose",
                      injected_armed && jeocs >= 995u && jeocs <= 1005u);
    } else {
        bench.verdict("TIM2's channel 3 match triggers the injection group at its 10 kHz (1000 +- 5 JEOCs), "
                      "and the TIM3 codes of table 9-4 are refused on this part",
                      injected_armed && jeocs >= 995u && jeocs <= 1005u &&
                          !Adc::injected_trigger(AdcInjectedTrigger::tim3_cc1));
    }
    (void)Adc::injected_trigger(AdcInjectedTrigger::software);
    all_off();
}

// ===========================================================================
// e - the injection group
// ===========================================================================

void te_injected() {
    // SCAN: without it only the first channel of a group converts (the
    // F1 lineage's rule, 9.2.4's table).
    (void)adc_ready(code_div8, {.scan = true});
    const uint16_t vref = Adc::read_settled(4);
    const uint8_t seq[4] = {Adc::vrefint_channel, Adc::vrefint_channel, Adc::vrefint_channel, Adc::vrefint_channel};
    (void)Adc::injected_sequence(seq, 4);
    const uint16_t offsets[4] = {0, scaled(100), static_cast<uint16_t>(vref), adc_max_count};
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)Adc::injected_offset(i, offsets[i]);
    }
    Adc::clear_flags(AdcFlag::all);
    Adc::injected_start();
    uint32_t spins = 100'000u;
    while (!Adc::injected_ready() && spins-- != 0u) {
    }
    const bool done = Adc::injected_ready();
    int16_t r[4];
    for (uint8_t i = 0; i < 4u; ++i) {
        r[i] = Adc::injected_result(i);
    }
    print(serial, "  VREFINT ", vref, " counts; four injected slots with offsets 0/", scaled(100), "/", vref, "/",
          adc_max_count, ": ", r[0], " ", r[1], " ", r[2], " ", r[3], crlf);
    bench.verdict("the four slots convert in one injected sequence (JEOC)", done);
    {
        const int16_t tol = scaled_tolerance(8);
        const int16_t off1 = static_cast<int16_t>(scaled(100));
        bench.verdict("each result is the reading less its offset, SIGNED: slot 0 near VREFINT, slot 1 an offset "
                      "below, slot 2 near zero, slot 3 the reading less the full scale (about -0.64 of it)",
                      r[0] >= vref - tol && r[0] <= vref + tol && r[1] >= vref - off1 - tol &&
                          r[1] <= vref - off1 + tol && r[2] >= -tol && r[2] <= tol &&
                          r[3] < -static_cast<int16_t>(scaled(2000)) && r[3] > -static_cast<int16_t>(scaled(3000)));
    }

    // Automatic injection after the rule group.
    (void)adc_ready(code_div8, {.scan = true, .auto_injected = true});
    (void)Adc::injected_sequence(seq, 1);
    (void)Adc::injected_offset(0, 0);
    Adc::injected_trigger_enable(false);
    Adc::clear_flags(AdcFlag::all);
    const uint16_t rule = Adc::read();
    spins = 100'000u;
    while (!Adc::injected_ready() && spins-- != 0u) {
    }
    const bool auto_done = Adc::injected_ready();
    const int16_t injected = Adc::injected_result(0);   // the FIRST conversion's result, whatever its slot
    print(serial, "  JAUTO: the rule conversion ", rule, " then the injected one by itself ", injected, crlf);
    bench.verdict("under JAUTO the injection group follows the rule group with no trigger of its own",
                  auto_done && injected >= static_cast<int16_t>(rule) - scaled_tolerance(8) &&
                      injected <= static_cast<int16_t>(rule) + scaled_tolerance(8));

    // Left alignment: the datum in the top adc_bits of RDATAR.
    (void)adc_ready(code_div8, {.left_aligned = true});
    const uint16_t left = Adc::result_counts();
    (void)left;
    const uint16_t raw = [] { uint16_t v; (void)Adc::read(v); return Adc::result(); }();
    print(serial, "  left-aligned RDATAR ", hex(raw), " -> ", Adc::result_counts(), " counts", crlf);
    bench.verdict("left alignment puts the datum in the top bits of RDATAR and result_counts() undoes it",
                  (raw & ((1u << (16u - adc_bits)) - 1u)) == 0u && Adc::result_counts() >= scaled(1200) &&
                      Adc::result_counts() <= scaled(1800));
    all_off();
}

// ===========================================================================
// f - the watchdogs
// ===========================================================================

void tf_watchdogs() {
    (void)adc_ready(code_div8);
    const uint16_t vref = Adc::read_settled(4);
    // Inside the window: quiet.
    (void)Adc::watchdog0({.low = static_cast<uint16_t>(vref - scaled(100)),
                          .high = static_cast<uint16_t>(vref + scaled(100)), .channel = Adc::vrefint_channel,
                          .interrupt = true});
    awd_interrupts = 0;
    Adc::clear_flags(AdcFlag::all);
    Pfic::enable(Adc::irq());
    for (uint8_t i = 0; i < 8u; ++i) {
        (void)Adc::read();
    }
    const uint32_t inside = awd_interrupts;
    const bool awd_inside = Adc::flag(AdcFlag::watchdog);
    // Outside: the window moved above the reading.
    (void)Adc::watchdog0({.low = static_cast<uint16_t>(vref + scaled(200)), .high = adc_max_count,
                          .channel = Adc::vrefint_channel, .interrupt = true});
    for (uint8_t i = 0; i < 8u; ++i) {
        (void)Adc::read();
    }
    const uint32_t outside = awd_interrupts;
    Pfic::disable(Adc::irq());
    Adc::watchdog0_off();
    print(serial, "  watchdog 0 around ", vref, ": inside the window ", inside, " interrupts (AWD ",
          awd_inside ? "up" : "down", "); outside ", outside, " in 8 conversions", crlf);
    bench.verdict("watchdog 0 is quiet with the reading inside its window", inside == 0u && !awd_inside);
    bench.verdict("and raises AWD with its interrupt on every conversion outside it", outside == 8u);

    if constexpr (!device::adc_has_ctlr3) {
        bench.verdict("watchdogs 1 and 2, the scan and the reset are the CH32V006's: the verbs answer false "
                      "and a reset_on_fault config is refused on this part",
                      !Adc::watchdog(1, 0, adc_max_count) && !Adc::watchdog_result(1) &&
                          !Adc::watchdog0({.reset_on_fault = true}));
        all_off();
        return;
    }
    // THE WATCHDOG SCAN (AWD_SCAN): one watchdog per RANK of a scanned
    // rule sequence - watchdog 0 on the first conversion, 1 on the
    // second, 2 on the third, each with its own thresholds (the vendor's
    // own example's arrangement; 9.3.15's "watchdog channel 1" is that
    // rank). VREFINT then PC4 pulled up twice: ranks 2 and 3 at 4095.
    (void)adc_ready(code_div8, {.scan = true});
    Pin<'C', 4>::input(PinPull::up);
    (void)delay_us(clock, 100);
    const uint8_t seq[3] = {Adc::vrefint_channel, In2::channel, In2::channel};
    (void)Adc::sequence(seq, 3);
    (void)Adc::watchdog0({.low = 0, .high = 4095});
    Adc::watchdog_scan(true);
    struct Round {
        const char* name;
        uint16_t lo0, hi0, lo1, hi1, lo2, hi2;
        bool expect0, expect1, expect2;
    };
    const Round rounds[] = {
        {"rank 2 out", 0, 4095, 100, 2000, 0, 4095, false, true, false},
        {"rank 3 out", 0, 4095, 0, 4095, 100, 2000, false, false, true},
        {"rank 1 out", 2000, 4095, 0, 4095, 0, 4095, true, false, false},
        {"all in", 0, 4095, 0, 4095, 0, 4095, false, false, false},
    };
    uint8_t right = 0;
    for (const Round& r : rounds) {
        Adc::regs().WDLTR = r.lo0;
        Adc::regs().WDHTR = r.hi0;
        (void)Adc::watchdog(1, r.lo1, r.hi1);
        (void)Adc::watchdog(2, r.lo2, r.hi2);
        Adc::clear_watchdog_result(0);
        Adc::clear_watchdog_result(1);
        Adc::clear_watchdog_result(2);
        Adc::clear_flags(AdcFlag::all);
        Adc::start();
        (void)delay_us(clock, 500);
        const bool r0 = Adc::watchdog_result(0);
        const bool r1 = Adc::watchdog_result(1);
        const bool r2 = Adc::watchdog_result(2);
        const bool ok = r0 == r.expect0 && r1 == r.expect1 && r2 == r.expect2;
        print(serial, "  scan VREFINT,PC4,PC4 with ", r.name, ": results ", r0, r1, r2, ok ? "  as expected" : "  OFF",
              crlf);
        if (ok) {
            ++right;
        }
    }
    Pin<'C', 4>::release();
    bench.verdict("under AWD_SCAN the three watchdogs guard the first three RANKS of the rule sequence, each "
                  "with its own thresholds, the results in CTLR3 cleared by writing zero",
                  right == 4u);
    all_off();
}

// ===========================================================================
// g - the pads through their pulls
// ===========================================================================

void tg_pads() {
    (void)adc_ready(code_div8);
    struct PadCase {
        const char* name;
        uint8_t channel;
        void (*pull_up)();
        void (*pull_down)();
    };
    const PadCase pads[] = {
        {"PD2 (IN3)", In3::channel, [] { Pin<'D', 2>::input(PinPull::up); }, [] { Pin<'D', 2>::input(PinPull::down); }},
        {"PC4 (IN2)", In2::channel, [] { Pin<'C', 4>::input(PinPull::up); }, [] { Pin<'C', 4>::input(PinPull::down); }},
    };
    uint8_t good = 0;
    for (const PadCase& p : pads) {
        Adc::select_channel(p.channel);
        (void)Adc::sample_time(p.channel, adc_sample_longest);
        p.pull_up();
        console_drain();
        const uint16_t up = Adc::read_settled(4);
        p.pull_down();
        (void)delay_us(clock, 100);
        const uint16_t down = Adc::read_settled(4);
        Pin<'D', 2>::release();
        Pin<'C', 4>::release();
        const bool ok = up >= scaled(3900) && down <= scaled(200);
        print(serial, "  ", p.name, ": pulled up ", up, ", pulled down ", down, ok ? "  as expected" : "  OFF", crlf);
        if (ok) {
            ++good;
        }
    }
    bench.verdict("two pads read near full scale through their pull-up and near zero through their pull-down "
                  "- the pull is a known level and the channel is the pad's",
                  good == 2u);
    all_off();
}

// ===========================================================================
// h - the sampler
// ===========================================================================

namespace ks {

class Probe {
public:
    using Event = std::variant<AnalogSample>;
    static inline EventQueue<Event, 8, P> queue;
    static inline uint16_t count[2] = {0, 0};
    static inline uint16_t last[2] = {0, 0};

    static void init() {
        count[0] = count[1] = 0;
    }
    static void dispatch(const Event& e) {
        brio::match(e, [](const AnalogSample& s) {
            if (s.index < 2u) {
                count[s.index] = static_cast<uint16_t>(count[s.index] + 1u);
                last[s.index] = s.value;
            }
        });
    }
};

using Sampler = AnalogSampler<Adc, P, Subscribers<Probe>, AdcInput::vrefint, In3{}>;
using Loop = Kernel<P, Probe, Sampler>;

void pump() {
    TimeEvents<P>::process();
    while (Loop::step()) {
        TimeEvents<P>::process();
    }
}

}  // namespace ks

volatile bool sampler_live = false;

void th_sampler() {
    (void)adc_ready(code_div8);
    Pin<'D', 2>::input(PinPull::up);   // IN3 at a known level
    ks::Loop::init_all();
    Adc::clear_flags(AdcFlag::all);
    Adc::interrupts(Adc::converted_interrupt, true);
    sampler_live = true;
    Pfic::enable(Adc::irq());
    ks::Sampler::start_every(5);   // a conversion every 5 ms, the inputs alternating
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 205u) {
        ks::pump();
    }
    ks::Sampler::stop();
    ks::pump();
    Pfic::disable(Adc::irq());
    Adc::interrupts(Adc::converted_interrupt, false);
    sampler_live = false;
    Pin<'D', 2>::release();
    print(serial, "  in 205 ms at one conversion per 5 ms: VREFINT ", ks::Probe::count[0], " samples (last ",
          ks::Probe::last[0], "), IN3 ", ks::Probe::count[1], " (last ", ks::Probe::last[1], "), unknown inputs ",
          ks::Sampler::unknown_inputs(), crlf);
    bench.verdict("the AnalogSampler walks its two inputs at the software pace (about 20 samples each in "
                  "205 ms), every result labelled by the input it was taken on",
                  ks::Probe::count[0] >= 19u && ks::Probe::count[0] <= 21u && ks::Probe::count[1] >= 19u &&
                      ks::Probe::count[1] <= 21u && ks::Sampler::unknown_inputs() == 0u);
    bench.verdict("VREFINT's samples are the reference and IN3's the pulled-up pad",
                  ks::Probe::last[0] >= scaled(1200) && ks::Probe::last[0] <= scaled(1800) &&
                      ks::Probe::last[1] >= scaled(3900));
    all_off();
}

// ===========================================================================
// r - the watchdog reset, judged at the next boot
// ===========================================================================

void tr_watchdog_reset() {
    token.pass = 0;
    token.fail = 0;
    (void)adc_ready(code_div8);
    const uint16_t vref = Adc::read_settled(4);
    Reset::clear_flags();
    print(serial, "  VREFINT ", vref, "; watchdog 0 armed with AWD0_RST_EN and its window above the reading: the "
          "next conversion resets the board", crlf);
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
    token.magic = token_magic;
    token.letter = 'r';
    token.leg = 1;
    token.counts = vref;
    (void)Adc::watchdog0({.low = static_cast<uint16_t>(vref + scaled(200)), .high = adc_max_count,
                          .channel = Adc::vrefint_channel, .reset_on_fault = true});
    Adc::clear_flags(AdcFlag::all);
    Adc::start();
    settle_ms(100);
    print(serial, "  NO RESET after 100 ms - the watchdog did not reset the board", crlf);
    token.leg = 0;
    Adc::watchdog0_off();
}

void judge(const char* what, bool ok) {
    bench.verdict(what, ok);
    if (ok) { token.pass = token.pass + 1u; } else { token.fail = token.fail + 1u; }
}

void resume_after_reset() {
    const uint8_t leg = token.leg;
    token.leg = 0;
    bench.reset_tally();
    bench.resume_tally(token.pass, token.fail);
    print(serial, crlf, "-> back from letter r leg ", leg, ": flags=", hex(boot_flags), crlf);
    judge("the ADC's watchdog RESET the board: ADCRSTF stands, nothing else does",
          (boot_flags & ResetFlag::adc) != 0u && (boot_flags & (ResetFlag::software | ResetFlag::watchdog)) == 0u);
    judge("and the boot finds the converter off (the reset took its registers with it)",
          (Adc::regs().CTLR3_DLYR & adc_awd0_rst_en) == 0u);
    bench.end_letter();
}

// ===========================================================================
// x - watchdogs 1 and 2 probed (outside z)
// ===========================================================================

void tx_watchdog_probe() {
    (void)adc_ready(code_div8);
    Adc::sample_time_all(adc_sample_longest);
    // Channel 2 (PC4) pulled up = 4095, channel 1 (PA1) pulled up = 4095,
    // VREFINT about 1500. Windows 100..2000 on both extra watchdogs.
    Pin<'C', 4>::input(PinPull::up);
    Pin<'A', 1>::input(PinPull::up);
    (void)delay_us(clock, 100);
    struct Variant {
        const char* name;
        uint8_t channel;
        bool awd_scan;
        bool awd0_on;
        bool awdsgl;
    };
    const Variant variants[] = {
        {"ch2, nothing else", 2, false, false, false},
        {"ch2, AWD_SCAN", 2, true, false, false},
        {"ch2, AWDEN (watchdog 0 on all)", 2, false, true, false},
        {"ch2, AWDEN + AWDSGL + AWDCH=2", 2, false, true, true},
        {"ch2, AWD_SCAN + AWDEN", 2, true, true, false},
        {"ch1, AWD_SCAN + AWDEN", 1, true, true, false},
        {"ch8 (VREFINT), AWD_SCAN + AWDEN", 8, true, true, false},
        {"ch1, nothing else", 1, false, false, false},
    };
    for (const Variant& v : variants) {
        Adc::regs().CTLR1 &= ~(adc_awden | adc_jawden | adc_awdsgl | adc_awdch_mask | adc_awdie);
        (void)Adc::watchdog(1, 100, 2000);
        (void)Adc::watchdog(2, 100, 2000);
        Adc::regs().WDHTR = 4095;
        Adc::regs().WDLTR = 0;
        if (v.awd0_on) { Adc::regs().CTLR1 |= adc_awden; }
        if (v.awdsgl) { Adc::regs().CTLR1 |= adc_awdsgl | v.channel; }
        Adc::watchdog_scan(v.awd_scan);
        Adc::select_channel(v.channel);
        Adc::clear_watchdog_result(0);
        Adc::clear_watchdog_result(1);
        Adc::clear_watchdog_result(2);
        Adc::clear_flags(AdcFlag::all);
        const uint16_t c = Adc::read_settled(4);
        print(serial, "  ", v.name, ": read ", c, " -> AWD0_RES ", Adc::watchdog_result(0), " AWD1_RES ",
              Adc::watchdog_result(1), " AWD2_RES ", Adc::watchdog_result(2), " AWD flag ",
              Adc::flag(AdcFlag::watchdog), " CTLR3=", hex(Adc::regs().CTLR3_DLYR), crlf);
    }
    // The vendor's own example: SCAN mode, a three-conversion rule
    // sequence, AWD_SCAN, AWDEN - one watchdog per RANK?
    (void)adc_ready(code_div8, {.scan = true});
    Adc::sample_time_all(adc_sample_longest);
    const uint8_t seq[3] = {8, 2, 2};   // VREFINT (~1500), PC4 pulled up (4095), PC4 again
    (void)Adc::sequence(seq, 3);
    Adc::regs().WDHTR = 4095;           // watchdog 0: open
    Adc::regs().WDLTR = 0;
    (void)Adc::watchdog(1, 100, 2000);  // watchdog 1: 4095 is OUT
    (void)Adc::watchdog(2, 0, 4095);    // watchdog 2: open
    Adc::regs().CTLR1 |= adc_awden;
    Adc::watchdog_scan(true);
    Adc::clear_watchdog_result(0);
    Adc::clear_watchdog_result(1);
    Adc::clear_watchdog_result(2);
    Adc::clear_flags(AdcFlag::all);
    Adc::start();
    (void)delay_us(clock, 500);
    print(serial, "  SCAN 8,2,2 + AWD_SCAN + AWDEN, windows open/100..2000/open: AWD0_RES ", Adc::watchdog_result(0),
          " AWD1_RES ", Adc::watchdog_result(1), " AWD2_RES ", Adc::watchdog_result(2), " AWD flag ",
          Adc::flag(AdcFlag::watchdog), " EOC ", Adc::ready(), crlf);
    // And the windows rotated: 2 tight, 1 and 0 open.
    Adc::regs().WDHTR = 4095;
    (void)Adc::watchdog(1, 0, 4095);
    (void)Adc::watchdog(2, 100, 2000);
    Adc::clear_watchdog_result(0);
    Adc::clear_watchdog_result(1);
    Adc::clear_watchdog_result(2);
    Adc::clear_flags(AdcFlag::all);
    Adc::start();
    (void)delay_us(clock, 500);
    print(serial, "  ... windows open/open/100..2000: AWD0_RES ", Adc::watchdog_result(0), " AWD1_RES ",
          Adc::watchdog_result(1), " AWD2_RES ", Adc::watchdog_result(2), crlf);
    // Watchdog 0 tight on rank 1 (VREFINT ~1500 against 2000..4095).
    Adc::regs().WDHTR = 4095;
    Adc::regs().WDLTR = 2000;
    (void)Adc::watchdog(2, 0, 4095);
    Adc::clear_watchdog_result(0);
    Adc::clear_watchdog_result(1);
    Adc::clear_watchdog_result(2);
    Adc::clear_flags(AdcFlag::all);
    Adc::start();
    (void)delay_us(clock, 500);
    print(serial, "  ... windows 2000..4095/open/open: AWD0_RES ", Adc::watchdog_result(0), " AWD1_RES ",
          Adc::watchdog_result(1), " AWD2_RES ", Adc::watchdog_result(2), crlf);
    Pin<'C', 4>::release();
    Pin<'A', 1>::release();
    all_off();
    print(serial, "  (a probe: no verdict)", crlf);
}

// ===========================================================================
// y - the triggered stall, hunted (outside z): adc.hpp's recover() says what it is
// ===========================================================================

/// One triggered run of `n` conversions on TIM2's TRGO at 10 kHz, the
/// order of arming as the variant says. True when the block completed.
volatile uint32_t bus_poll_sink = 0;
uint8_t bus_poll_kind = 0;   // 0 none, 1 a PB2 peripheral (GPIOC INDR), 2 a PB1 one (TIM2 CNT), 3 SRAM, 4 the ADC's STATR
volatile uint16_t eoc_reads = 0;
volatile bool eoc_reader_live = false;

bool one_triggered_run(uint16_t n, bool dma_first, bool power_cycle, AdcSampleTime t) {
    T2::init();
    (void)T2::configure({.prescaler = 47, .period = 99});
    (void)T2::master(TimMasterMode::update);
    (void)Adc::trigger(AdcTrigger::tim2_trgo);
    (void)Adc::sample_time(Adc::vrefint_channel, t);
    if (power_cycle) {
        Adc::power(false);
        Adc::power(true);
    }
    Adc::clear_flags(AdcFlag::all);
    Reader::stop();
    auto arm = [n] {
        Adc::dma(true);
        (void)Reader::load(DmaTransfer{
            .peripheral = Adc::data_address(), .memory = block, .count = n,
            .config = {.direction = DmaDirection::peripheral_to_memory, .peripheral_increment = false,
                       .memory_increment = false, .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::half}});
    };
    if (dma_first) {
        arm();
        T2::enable(true);
    } else {
        T2::enable(true);
        arm();
    }
    const uint32_t t0 = cycles_now();
    while (!Reader::flag(DmaFlag::complete) && cycles_now() - t0 < static_cast<uint32_t>(n) * 4800u * 3u) {
        if (bus_poll_kind == 1u) { bus_poll_sink = Port<'C'>::regs().INDR; }
        else if (bus_poll_kind == 2u) { bus_poll_sink = T2::count(); }
        else if (bus_poll_kind == 3u) { bus_poll_sink = block[0]; }
        else if (bus_poll_kind == 4u) { bus_poll_sink = Adc::regs().STATR; }
    }
    const bool done = Reader::flag(DmaFlag::complete);
    if (!done) {
        const uint16_t left = Reader::count();
        const uint32_t statr = Adc::regs().STATR & 0x1Fu;
        // The interlock hypothesis: the converter waits for RDATAR to be
        // taken. A CPU read of it, then a millisecond: does the block
        // move again?
        const uint16_t datar = Adc::result();
        (void)delay_us(clock, 1000);
        const uint16_t left_after_read = Reader::count();
        // And a software START instead:
        Adc::start();
        (void)delay_us(clock, 1000);
        const uint16_t left_after_start = Reader::count();
        (void)datar; (void)left_after_read; (void)left_after_start;
        print(serial, "    stall: left ", left, " STATR=", hex(statr), crlf);
    }
    T2::enable(false);
    Adc::dma(false);
    Reader::stop();
    return done;
}

/// The same run WITHOUT the DMA: the EOC handler takes each result and
/// counts; the main loop polls a bus. True when n results arrived.
bool one_interrupt_run(uint16_t n) {
    T2::init();
    (void)T2::configure({.prescaler = 47, .period = 99});
    (void)T2::master(TimMasterMode::update);
    (void)Adc::trigger(AdcTrigger::tim2_trgo);
    (void)Adc::sample_time(Adc::vrefint_channel, sample_mid);
    Adc::clear_flags(AdcFlag::all);
    eoc_reads = 0;
    eoc_reader_live = true;
    Adc::interrupts(Adc::converted_interrupt, true);
    Pfic::enable(Adc::irq());
    T2::enable(true);
    const uint32_t t0 = cycles_now();
    while (eoc_reads < n && cycles_now() - t0 < static_cast<uint32_t>(n) * 4800u * 3u) {
        if (bus_poll_kind == 1u) { bus_poll_sink = Port<'C'>::regs().INDR; }
    }
    const bool done = eoc_reads >= n;
    if (!done) {
        print(serial, "    stall (interrupt reader): ", eoc_reads, " results, STATR=", hex(Adc::regs().STATR & 0x1Fu), crlf);
    }
    T2::enable(false);
    Pfic::disable(Adc::irq());
    Adc::interrupts(Adc::converted_interrupt, false);
    eoc_reader_live = false;
    return done;
}

void ty_stall_hunt() {
    struct Variant {
        const char* name;
        bool dma_first;
        bool power_cycle;
        AdcSampleTime t;
    };
    struct Variant2 {
        const char* name;
        bool with_dma;
        uint8_t poll;
    };
    const Variant2 variants[] = {
        {"DMA, the CPU idle in the wait loop", true, 0},
        {"DMA, the CPU polling GPIOC INDR (PB2)", true, 1},
        {"DMA, the CPU polling TIM2 CNT (PB1)", true, 2},
        {"DMA, the CPU polling the ADC's own STATR", true, 4},
        {"DMA, the CPU polling SRAM", true, 3},
        {"NO DMA, the EOC handler reading RDATAR, the CPU polling GPIOC INDR (PB2)", false, 1},
    };
    (void)adc_ready(code_div8);
    for (const Variant2& v : variants) {
        uint8_t stalls = 0;
        bus_poll_kind = v.poll;
        for (uint8_t k = 0; k < 100u; ++k) {
            for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
            }
            const bool ok = v.with_dma ? one_triggered_run(100, true, false, sample_mid)
                                       : one_interrupt_run(100);
            if (!ok) {
                ++stalls;
                Adc::recover();   // so the next run starts clean
            }
        }
        bus_poll_kind = 0;
        print(serial, "  ", v.name, ": ", stalls, " stalls in 100 runs of 100", crlf);
    }
    all_off();
    print(serial, "  (a probe: no verdict)", crlf);
}

void banner() {
    print(serial, crlf, "test_ch32_adc - ", device::part_name, " ADC (RM ch. 9), nothing to wire", crlf);
    print(serial, "  boot flags ", hex(boot_flags), crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void adc_handler() {
    const uint32_t f = brio::Adc::isr();
    adc_interrupts = adc_interrupts + 1u;
    if ((f & brio::AdcFlag::watchdog) != 0u) {
        awd_interrupts = awd_interrupts + 1u;
    }
    if ((f & brio::AdcFlag::injected) != 0u) {
        jeoc_interrupts = jeoc_interrupts + 1u;
    }
    if ((f & brio::AdcFlag::converted) != 0u && sampler_live) {
        brio::post<ks::Sampler>(brio::Sampled{brio::Adc::result_counts(), brio::Adc::selected()});
    }
    if ((f & brio::AdcFlag::converted) != 0u && eoc_reader_live) {
        (void)brio::Adc::result();
        eoc_reads = eoc_reads + 1u;
    }
}

int main() {
    boot_flags = brio::Reset::take_flags();
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless: reset values, the prescaler code, the arithmetic, the refusals", ta_block);
    bench.letter('b', "VREFINT: the supply from the 1.2 V reference, its spread, channel 9", tb_vrefint);
    bench.letter('c', "THE RATE: 256 continuous conversions through the DMA, timed", tc_rate);
    bench.letter('d', "THE TRIGGERS: six timer events pacing the rule group, TIM3 the injection one", td_triggers);
    bench.letter('e', "THE INJECTION GROUP: four offsets, signed results, JAUTO, the alignment", te_injected);
    bench.letter('f', "THE WATCHDOGS: 0 around VREFINT with its interrupt, 1 and 2's results", tf_watchdogs);
    bench.letter('g', "THE PADS through their pulls: IN3 and IN2 up and down", tg_pads);
    bench.letter('h', "THE SAMPLER: AnalogSampler over VREFINT and IN3 at a software pace", th_sampler);
    if constexpr (device::adc_has_ctlr3) {
        bench.letter('r', "THE WATCHDOG RESET (reboots the board): ADCRSTF at the next boot", tr_watchdog_reset, false);
        bench.letter('x', "watchdogs 1 and 2 probed eight ways (no verdict)", tx_watchdog_probe, false);
    }
    bench.letter('y', "the triggered stall hunted, six arrangements of 100 runs (no verdict)", ty_stall_hunt, false);

    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        resume_after_reset();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", " flags=", brio::hex(boot_flags), brio::crlf);
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
