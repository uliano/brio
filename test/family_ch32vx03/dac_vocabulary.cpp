// The DAC's header on EVERY part of the family: the vocabulary compiles
// where the converter does not exist, so a program for both series can
// include ch32vx03/dac.hpp and name `Dac` in the letters the CH32V303 alone
// compiles - the refusal fires where the converter is USED
// (neg/dac_absent.cpp), not where the header is read.
#include "ch32vx03/dac.hpp"

using namespace brio;

static_assert(dac_trigger_valid(DacTrigger::software) && dac_trigger_valid(DacTrigger::exti9));
static_assert(dac_trigger_valid(DacTrigger::tim2_trgo) && dac_trigger_valid(DacTrigger::tim4_trgo));
static_assert(dac_trigger_timer(DacTrigger::tim7_trgo) == 7);
static_assert(dac_wave_amplitude(5) == 63u);
static_assert(dac_lfsr_preload == 0x0AAAu);
static_assert(dac_dma_slot(1).present() == device::has_dac);
static_assert(dac_channel_config_valid(DacChannelConfig{}));

/// A part without the converter says so, and the letter that would drive
/// it is discarded: the shape a suite for both series takes. The converter
/// is spelled `DacUnit<has>` there, so that the name depends on the
/// template's own parameter and a discarded branch never instantiates it.
template <bool has = device::has_dac>
void drive_if_present() {
    if constexpr (has) {
        using D = DacUnit<has>;
        D::init();
        (void)D::write(1, 2048);
        (void)D::enable(1, true);
    }
}

void f() { drive_if_present(); }
