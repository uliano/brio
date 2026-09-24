// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE SECOND CONVERTER, and with it the dual modes - on the eight parts
// that have two. Datasheet table 2-1 counts the converters beside the
// channels ("9@2", "10@2", "16@1"): the 128 KB part trades ADC2 for six
// more channels, so Adc<2> is instantiated here and nowhere else, and a
// neg TU proves that naming it on that part is a compile error.
//
// WHAT ADC2 IS NOT. Only ADC1 has a DMA request (12.2.7's note 2), only
// ADC1 wakes the temperature sensor and VREFINT (CTLR2.TSVREFE, "only
// applied for ADC1") and only ADC1 carries the dual-mode field (CTLR1's
// DUALMOD, "these bits in ADC2 are reserved"). The three verbs exist on
// both instances and answer FALSE on the second, which is what lets one
// program name either converter without an #if.
#include "ch32v203/adc.hpp"
#include "ch32v203/platform.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 96'000'000>;

static_assert(device::adc_count == 2u);
static_assert(Adc<2>::instance == 2);
static_assert(!Adc<2>::has_dma);
static_assert(!Adc<2>::has_internal_sources);
static_assert(!Adc<2>::has_dual_mode);
static_assert(Adc<1>::has_dual_mode);
/// One vector serves both (table 9-2's entry 34).
static_assert(Adc<2>::irq() == Irq::adc1_2);
static_assert(Adc<1>::irq() == Adc<2>::irq());
/// Two gates, though (RCC_PB2PCENR bits 9 and 10).
static_assert(Adc<1>::gate_bit == rcc_pb2_adc1);
static_assert(Adc<2>::gate_bit == rcc_pb2_adc2);
static_assert(Adc<2>::channels == adc_channels);

void adc2_verbs() {
    SysClock clock;
    (void)Adc<2>::init(clock);
    // The three the second converter has not got: refused rather than
    // written, and refused in init() too when a config asks for them.
    (void)Adc<2>::init(clock, AdcConfig{.dma = true});
    (void)Adc<2>::init(clock, AdcConfig{.internal_sources = true});
    (void)Adc<2>::dma(true);
    (void)Adc<2>::internal_sources(true);
    // Its dual() is a compile error and not a false: DUALMOD is the
    // MASTER's field, never a spelling both instances share (the neg TU
    // beside this one).

    Adc<2>::select_channel(1);
    Adc<2>::sample_time_all(adc_sample_longest);
    Adc<2>::start();
    (void)Adc<2>::read();
    (void)Adc<2>::result_counts();
    (void)Adc<2>::watchdog(AdcWatchdogConfig{.low = 10, .high = 4000});
    (void)Adc<2>::isr();
    Adc<2>::release();
}

/// The dual modes, which are ADC1's field and ADC2's silence: the
/// master is told the mode, the slave is told nothing, and both are set
/// to the same trigger by 12.2.7's own note (the slave takes the
/// software trigger so a spurious edge cannot start it alone).
void adc_dual_modes() {
    SysClock clock;
    (void)Adc<1>::init(clock, AdcConfig{.dma = true});
    (void)Adc<2>::init(clock);
    (void)Adc<1>::dual(AdcDualMode::independent);
    (void)Adc<1>::dual(AdcDualMode::regular_simultaneous);
    (void)Adc<1>::dual(AdcDualMode::injected_simultaneous);
    (void)Adc<1>::dual(AdcDualMode::fast_interleaved);
    (void)Adc<1>::dual(AdcDualMode::slow_interleaved);
    (void)Adc<1>::dual(AdcDualMode::alternate_trigger);
    (void)Adc<1>::dual(AdcDualMode::regular_and_injected_simultaneous);
    (void)Adc<1>::dual(AdcDualMode::regular_simultaneous_and_alternate_trigger);
    (void)Adc<1>::dual(AdcDualMode::injected_simultaneous_and_fast_interleaved);
    (void)Adc<1>::dual(AdcDualMode::injected_simultaneous_and_slow_interleaved);
    (void)Adc<1>::dual();
    /// The follower's datum, in the upper half of the master's register.
    (void)Adc<1>::follower_counts();
    (void)Adc<1>::data();
}
