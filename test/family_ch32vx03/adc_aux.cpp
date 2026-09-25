// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The ADC's CH32V30x_D8 extras: ADCx_AUX and the four short sampling
// times beside the long eight, the TIM8 alternative of trigger code 110
// through AFIO's remap (the RC and the VC, which have a TIM8), ADC2's
// request on DMA2's channel 5 (table 11-3), the clock tree's second duty
// bit, and the reference that is a pad on the LQFP100.
#include "ch32vx03/adc.hpp"
#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 96'000'000>;

// ---- the class's facts ------------------------------------------------------
static_assert(adc_has_short_sampling);
static_assert(adc_has_tim8_triggers == ((device::advanced_timer_instances & (1U << 8)) != 0u));
static_assert(adc_has_tim8_triggers == afio_adc_trigger_remap_exists());
static_assert(device::adc_count == 2u);
static_assert(adc_reference == (device::has_vref_pads ? Ref::vref_pad : Ref::vdda));
static_assert(ref_mv(adc_reference, 3300) == 3300);

// ---- the short sampling times (12.3.15) ---------------------------------------
static_assert(adc_sample_half_cycles(AdcShortSampleTime::cycles2_5) == 5u);
static_assert(adc_sample_half_cycles(AdcShortSampleTime::cycles3_5) == 7u);
static_assert(adc_sample_half_cycles(AdcShortSampleTime::cycles4_5) == 9u);
static_assert(adc_sample_half_cycles(AdcShortSampleTime::cycles5_5) == 11u);
static_assert(adc_conversion_half_cycles(AdcShortSampleTime::cycles5_5) == 36u);
static_assert(adc_conversion_ns(AdcShortSampleTime::cycles4_5, 12'000'000UL) == 1'416u);
static_assert(adc_aux_mask == 0x3FFFFUL);

// ---- the second converter's request (table 11-3) --------------------------------
static_assert(Adc<2>::has_dma && Adc<2>::dma_slot == DmaSlot{2, 5} && Adc<2>::dma_channel == 5);
static_assert(Adc<1>::dma_slot == DmaSlot{1, 1});
static_assert(Adc<1>::regular_trigger_remap == Remap::adc1_etrgreg &&
              Adc<2>::injected_trigger_remap == Remap::adc2_etrginj);

using Adc2Row = DmaRequestOf<DmaRequest::adc2>;
using Source2 = DmaPingPongEngine<Adc2Row::controller, Adc2Row::channel, uint16_t>;
alignas(4) volatile uint16_t half_a[8];
alignas(4) volatile uint16_t half_b[8];

void short_time_verbs() {
    constexpr SysClock clock;
    (void)Adc<1>::init(clock, {.scan = true});
    (void)Adc<1>::sample_time(3, AdcShortSampleTime::cycles2_5);
    (void)Adc<1>::sample_time(17, AdcShortSampleTime::cycles5_5);
    (void)Adc<1>::sample_time(18, AdcShortSampleTime::cycles3_5);   // refused: no channel 18
    (void)Adc<1>::short_sample_time(3);
    (void)Adc<1>::aux();
    (void)Adc<1>::conversion_half_cycles(3);
    (void)Adc<1>::sample_time(3, AdcSampleTime::cycles41_5);   // the long table again
    Adc<2>::sample_time_all(AdcShortSampleTime::cycles4_5);
    Adc<2>::sample_time_all(AdcSampleTime::cycles239_5);
    (void)Adc<2>::aux();
}

void trigger_verbs() {
    (void)Adc<1>::trigger(AdcTrigger::exti11);
    (void)Adc<1>::trigger(AdcTrigger::tim3_trgo);
    (void)Adc<1>::trigger();
    (void)Adc<1>::injected_trigger(AdcInjectedTrigger::exti15);
    (void)Adc<1>::injected_trigger();
    Adc<2>::trigger<AdcTrigger::exti11>();
    Adc<2>::injected_trigger<AdcInjectedTrigger::tim4_trgo>();
    // A remapped code at run time: taken where the part has a TIM8, refused
    // with nothing written where it has not.
    (void)Adc<1>::trigger(AdcTrigger::tim8_trgo);
    (void)Adc<2>::injected_trigger(AdcInjectedTrigger::tim8_cc4);
}

/// TIM8's triggers as constants, where the part has the timer.
template <bool has = adc_has_tim8_triggers>
void tim8_trigger_verbs() {
    if constexpr (has) {
        Adc<1>::trigger<AdcTrigger::tim8_trgo>();
        Adc<2>::injected_trigger<AdcInjectedTrigger::tim8_cc4>();
        (void)Afio::remap(Remap::adc1_etrgreg, 1);
        (void)Afio::remap_code(Remap::adc2_etrginj);
    }
}

void dma_verbs() {
    Adc<2>::claim_stream<Source2>();
    (void)Source2::start(half_a, half_b, 8);
    (void)Adc<2>::dma(true);
    (void)Adc<2>::dma();
    Source2::stop();
}

void clock_verbs() {
    (void)Rcc::adc_duty_75(true);
    (void)Rcc::adc_duty_75();
    (void)Rcc::adc_duty_75(false);
    Rcc::adc_duty_extended(false);
}

void all() {
    short_time_verbs();
    trigger_verbs();
    tim8_trigger_verbs();
    dma_verbs();
    clock_verbs();
}
