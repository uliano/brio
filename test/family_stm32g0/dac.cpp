// Family smoke TU: stm32g0/dac.hpp (RM0444 ch. 16). Instantiation only -
// no main(), no hardware.
//
// THE PER-HEADER DIFFERENCE THIS FIXTURE IS FOR: 16.3's own table says
// the G031/G041 have NO DAC, and their device header declares no
// DAC1_BASE - so on the G031 header this file must compile to the
// PRESENCE CHECK AND NOTHING ELSE, which is what the guard below is.
// (Trying to instantiate brio::Dac there is a static_assert, and
// neg/dac_off_the_g031.cpp is where that is proven.) Where the DAC does
// exist it has both channels, its own two DMA request lines, and a
// vector shared with TIM6 and LPTIM1.

#include <stdint.h>

#include "stm32g0/device_tables.hpp"
#include "stm32g0/platform_stm32.hpp"

using namespace brio;

#if defined(STM32G031xx)

static_assert(!dac_present(), "the G031 class has no DAC (16.3)");
static_assert(dac_channels() == 0);

#else

#include "stm32g0/dac.hpp"
#include "util/analog.hpp"

static_assert(dac_present() && dac_channels() == 2, "two output channels (16.3)");
static_assert(dac_irq() == TIM6_DAC_LPTIM1_IRQn,
              "the DAC's only vector is shared with TIM6 and LPTIM1 (table 61)");
static_assert(Dac::dma_request(0) == 8 && Dac::dma_request(1) == 9,
              "table 55 rows 8 and 9");
static_assert(Dac::steps == 4096);

// The trigger table's holes are real (16.4.2's interconnect table).
static_assert(dac_trigger_valid(DacTrigger::software));
static_assert(dac_trigger_valid(DacTrigger::tim6_trgo));
static_assert(dac_trigger_valid(DacTrigger::exti9));
static_assert(!dac_trigger_valid(static_cast<DacTrigger>(4)), "dac_chx_trg4 is nothing");
static_assert(!dac_trigger_valid(static_cast<DacTrigger>(15)));

// The eight modes decoded into the three questions they answer.
static_assert(dac_mode_drives_pin(DacMode::pin_buffered));
static_assert(!dac_mode_drives_pin(DacMode::internal_unbuffered));
static_assert(dac_mode_buffered(DacMode::pin_and_internal_buffered));
static_assert(!dac_mode_buffered(DacMode::pin_unbuffered));
static_assert(dac_mode_sample_hold(DacMode::sample_hold_pin_buffered));
static_assert(!dac_mode_sample_hold(DacMode::internal_unbuffered));

// MAMP: 2^(code+1) - 1, saturating (16.7.1).
static_assert(dac_wave_amplitude(0) == 1 && dac_wave_amplitude(3) == 15 &&
              dac_wave_amplitude(11) == 4095 && dac_wave_amplitude(15) == 4095);

// The refusals.
static_assert(dac_channel_config_valid({}));
static_assert(!dac_channel_config_valid({.trigger = static_cast<DacTrigger>(7)}));
static_assert(!dac_channel_config_valid({.amplitude = 12}));
static_assert(!dac_channel_config_valid({.triggered = false, .wave = DacWave::triangle}),
              "16.7.1: a wave generator is only used with TENx set");
static_assert(dac_channel_config_valid({.triggered = true,
                                        .trigger = DacTrigger::tim6_trgo,
                                        .wave = DacWave::triangle,
                                        .amplitude = 5}));

// util/analog.hpp against this converter's full scale.
static_assert(dac_code(1650, Dac::steps, 3300) == 2048);
static_assert(dac_mv(2048, Dac::steps, 3300) == 1650);

using PadOut1 = Pin<'A', 4>;   // DAC1_OUT1 (16.3)
using PadOut2 = Pin<'A', 5>;   // DAC1_OUT2, and LD4 on a Nucleo-64

void use() {
    Dac::init();
    Dac::claim_pad<PadOut1>();
    Dac::claim_pad<PadOut2>();
    (void)Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::configure(1, {.mode = DacMode::internal_unbuffered,
                             .triggered = true,
                             .trigger = DacTrigger::tim6_trgo,
                             .wave = DacWave::noise,
                             .amplitude = 7,
                             .dma = true,
                             .underrun_interrupt = true});
    (void)Dac::config(0);
    (void)Dac::channel_valid(2);
    (void)Dac::enable(0, true);
    (void)Dac::enabled(0);

    (void)Dac::write(0, 2048);
    (void)Dac::write_left(0, 0x8000);
    (void)Dac::write8(0, 128);
    (void)Dac::write_dual(1024, 3072);
    (void)Dac::code(0);
    (void)Dac::output(0);
    (void)Dac::trigger(0);
    Dac::trigger_both();

    (void)Dac::sample_hold_times(0, 100, 200, 30);
    (void)Dac::busy(0);

    (void)Dac::trim(0);
    (void)Dac::set_trim(0, 16);
    (void)Dac::calibration_mode(0, true);
    (void)Dac::calibration_flag(0);

    (void)Dac::flags();
    Dac::clear_flags(DacFlag::underrun(0));
    (void)Dac::underrun(1);
    (void)Dac::clear_underrun(1);
    (void)Dac::isr();
    (void)Dac::irq();
    (void)Dac::data_address_12r(0);
    (void)Dac::data_address_12l(0);
    (void)Dac::data_address_8r(1);
    (void)Dac::data_address_dual_12r();

    Dac::release_pad<PadOut1>();
    Dac::release();
}

#endif
