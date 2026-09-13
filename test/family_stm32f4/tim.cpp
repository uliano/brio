// Timer family smoke TU (RM0090 ch. 17..20, RM0390 ch. 15..18, RM0383
// ch. 12..14). This chapter is the one where the DEVICE HEADER and the
// MANUAL are furthest apart: the header says which of the fourteen
// instances exist and nothing whatever about what each IS, because every
// register is a member of one TIM_TypeDef. So what this TU really checks
// is that the geometry the reserve states is reached only through the
// header's own presence probe, that the four shared-vector names are
// DERIVED from what shares them, and that every verb of the resource and
// of the nine tasks compiles for every instance that exists here.
//
// A family fixture is the one place that may ask the header a question
// TWICE, by two independent symbols, and assert the two answers agree.
#include "stm32f4/tim.hpp"

using namespace brio;

// ---- which instances exist, three ways over ---------------------------------

// TIM1, TIM5, TIM9 and TIM11 are on every part of the pack; the presence
// probe, the RCC mask and the geometry must agree everywhere.
static_assert(tim_present(1) && tim_present(5) && tim_present(9) && tim_present(11));
static_assert(!tim_present(0) && !tim_present(15) && !tim_present(255));

// The base address, the bus clock mask and the counter width come and go
// together: a timer the header has not got has no mask and no width.
static_assert(tim_present(2) == (tim_bus_clock(2).enable_mask != 0u));
static_assert(tim_present(8) == (tim_bus_clock(8).enable_mask != 0u));
static_assert(tim_present(14) == (tim_bus_clock(14).enable_mask != 0u));
static_assert(tim_present(6) == (tim_counter_bits(6) != 0u));
static_assert(tim_present(12) == (tim_channels(12) != 0u));

// Which bus each is on (RM0090 table 1), asserted for every instance the
// device has: TIM1, TIM8 and TIM9..TIM11 on APB2, the rest on APB1.
static_assert(tim_bus_clock(1).apb2 && tim_bus_clock(9).apb2 && tim_bus_clock(11).apb2);
static_assert(!tim_bus_clock(5).apb2);
#if defined(TIM8_BASE)
static_assert(tim_bus_clock(8).apb2 && tim_bus_clock(12).apb2 == false);
#endif
#if defined(TIM10_BASE)
static_assert(tim_bus_clock(10).apb2);
#endif
#if defined(TIM2_BASE)
static_assert(!tim_bus_clock(2).apb2 && !tim_bus_clock(3).apb2 && !tim_bus_clock(4).apb2);
#endif
#if defined(TIM6_BASE)
static_assert(!tim_bus_clock(6).apb2);
#endif

// ---- the geometry (the manuals', keyed by instance) --------------------------

// TIM2 and TIM5 are the family's two 32-bit counters and no other is.
static_assert(tim_counter_bits(5) == 32 && tim_max_period(5) == 0xFFFFFFFFUL);
static_assert(tim_counter_bits(1) == 16 && tim_max_period(1) == 0xFFFFUL);
static_assert(tim_counter_bits(9) == 16 && tim_counter_bits(11) == 16);
#if defined(TIM2_BASE)
static_assert(tim_counter_bits(2) == 32 && tim_counter_bits(3) == 16 && tim_counter_bits(4) == 16);
#endif

// Channels, complementary outputs, and what follows from them.
static_assert(tim_channels(1) == 4 && tim_complementary_channels(1) == 3);
static_assert(tim_channels(9) == 2 && tim_complementary_channels(9) == 0);
static_assert(tim_channels(11) == 1);
static_assert(tim_channels(5) == 4);
#if defined(TIM6_BASE)
static_assert(tim_channels(6) == 0, "a basic timer has no capture/compare unit");
#endif
#if defined(TIM8_BASE)
static_assert(tim_channels(8) == 4 && tim_complementary_channels(8) == 3);
static_assert(tim_channels(12) == 2 && tim_channels(13) == 1 && tim_channels(14) == 1);
#endif

// The break/dead-time unit, the repetition counter and the complementary
// outputs are the SAME two timers' - one fact under three names.
static_assert(tim_has_break(1) && tim_has_repetition(1));
static_assert(tim_has_break(9) == false && tim_has_repetition(11) == false);
static_assert(tim_has_break(5) == (tim_complementary_channels(5) != 0u));

// The slave controller is wider than the encoder, and the encoder is
// exactly the four-channel set: TIM9/TIM12 slave but do not count a
// quadrature pair (RM0383 14.4.2 leaves SMS 001..011 Reserved).
static_assert(tim_has_slave_mode(9) && !tim_has_encoder(9));
static_assert(tim_has_slave_mode(1) && tim_has_encoder(1));
static_assert(!tim_has_slave_mode(11) && !tim_has_encoder(11));
static_assert(tim_has_encoder(5) && tim_has_direction(5) && tim_has_center_aligned(5));
static_assert(!tim_has_direction(9) && !tim_has_center_aligned(9));

// ETR, the XOR input, the DMA burst engine and the encoder are one set.
static_assert(tim_has_external_trigger(1) == tim_has_encoder(1));
static_assert(tim_has_ti1_xor(5) == tim_has_encoder(5));
static_assert(tim_has_dma_burst(5) == tim_has_encoder(5));
static_assert(!tim_has_external_trigger(9) && !tim_has_dma_burst(9));

// DMA requests reach further than the burst engine: the basic timers have
// UDE and no DCR, TIM9..TIM14 have neither.
#if defined(TIM6_BASE)
static_assert(tim_has_dma_request(6) && !tim_has_dma_burst(6));
#endif
static_assert(!tim_has_dma_request(9) && !tim_has_dma_request(11));

// CR2 - and therefore TRGO - stops at TIM8: TIM9..TIM14 have no such
// register at all, which is why their ITR entries are OC outputs.
static_assert(tim_has_master_mode(1) && tim_has_master_mode(5));
static_assert(!tim_has_master_mode(9) && !tim_has_master_mode(11));
#if defined(TIM6_BASE)
static_assert(tim_has_master_mode(6) && tim_channels(6) == 0);
#endif

// CR1.CKD: every timer with a channel, and not the basic pair.
static_assert(tim_has_clock_division(1) && tim_has_clock_division(11));
#if defined(TIM6_BASE)
static_assert(!tim_has_clock_division(6));
#endif

// The option register: three instances, and the header's field positions
// are three different places in one register.
static_assert(tim_has_option_register(5) && tim_has_option_register(11));
static_assert(!tim_has_option_register(1) && !tim_has_option_register(9));
static_assert(tim_has_option_register(2) == tim_present(2));
// The SAME question, asked the other way: the field position is read off
// the header's own *_Pos macro, and the pack declares ITR1_RMP's on
// exactly the headers that declare a TIM2.
static_assert(tim_option_pos(11) == 0 && tim_option_pos(5) == 6);
static_assert(tim_option_pos(1) == 0xFFu && tim_option_pos(9) == 0xFFu);
static_assert((tim_option_pos(2) != 0xFFu) == tim_has_option_register(2));
#if defined(TIM2_BASE)
static_assert(tim_option_pos(2) == 10);
#endif

// ---- the vectors, derived from presence -------------------------------------

// The advanced-control timers have four lines each and everything else
// has one, so on a one-line timer the four verbs answer the same name -
// which is what lets a handler bind them without asking.
static_assert(tim_has_split_vectors(1));
static_assert(!tim_has_split_vectors(9) && !tim_has_split_vectors(5));
static_assert(tim_irq(5) == tim_cc_irq(5) && tim_irq(5) == tim_break_irq(5) &&
              tim_irq(5) == tim_trigger_irq(5));
static_assert(tim_irq(1) != tim_cc_irq(1) && tim_irq(1) != tim_break_irq(1) &&
              tim_irq(1) != tim_trigger_irq(1));

// THE SHARING IS THE DERIVATION. TIM9, TIM10 and TIM11 sit on TIM1's
// break, update and trigger lines; the header spells TIM1_UP_TIM10 where
// there is a TIM10 and TIM1_UP where there is not, and the reserve reads
// exactly that.
static_assert(tim_irq(9) == TIM1_BRK_TIM9_IRQn && tim_irq(9) == tim_break_irq(1));
static_assert(tim_irq(11) == TIM1_TRG_COM_TIM11_IRQn && tim_irq(11) == tim_trigger_irq(1));
#if defined(TIM10_BASE)
static_assert(tim_irq(10) == TIM1_UP_TIM10_IRQn && tim_irq(10) == tim_irq(1));
#else
static_assert(tim_irq(1) == TIM1_UP_IRQn, "no TIM10, no shared name");
#endif
#if defined(TIM8_BASE)
static_assert(tim_irq(12) == tim_break_irq(8) && tim_irq(13) == tim_irq(8) &&
              tim_irq(14) == tim_trigger_irq(8));
static_assert(tim_cc_irq(8) == TIM8_CC_IRQn);
#endif
// TIM6 shares its line with the DAC where the device has one, and has it
// alone where it has not (the F412 class).
#if defined(TIM6_BASE) && defined(DAC_BASE)
static_assert(tim_irq(6) == TIM6_DAC_IRQn);
#elif defined(TIM6_BASE)
static_assert(tim_irq(6) == TIM6_IRQn);
#endif

// The per-vector flag masks: one line answers for everything, four lines
// split RM0090 table 61's four groups exactly once between them.
static_assert(Tim<5>::vector_flags(tim_irq(5)) == Tim<5>::interrupt_flags);
static_assert(Tim<5>::vector_flags(SysTick_IRQn) == 0u);
static_assert((Tim<1>::vector_flags(tim_irq(1)) | Tim<1>::vector_flags(tim_cc_irq(1)) |
               Tim<1>::vector_flags(tim_break_irq(1)) |
               Tim<1>::vector_flags(tim_trigger_irq(1))) == Tim<1>::interrupt_flags,
              "the four vectors cover every flag an interrupt can be raised for");
static_assert((Tim<1>::vector_flags(tim_irq(1)) & Tim<1>::vector_flags(tim_cc_irq(1))) == 0u,
              "and no flag twice");
static_assert(Tim<1>::vector_flags(tim_break_irq(1)) == Tim<1>::break_flag);
// TIM9's body, called from the vector it SHARES with TIM1's break, must
// answer for TIM9's own flags and not for TIM1's.
static_assert(Tim<9>::vector_flags(TIM1_BRK_TIM9_IRQn) == Tim<9>::interrupt_flags);
static_assert(Tim<1>::vector_flags(TIM1_BRK_TIM9_IRQn) == TIM_SR_BIF);
// THE OVERCAPTURE FLAGS ARE NOT INTERRUPT SOURCES, and their bit
// positions are the CAPTURE/COMPARE DMA enables' - which is why isr()
// masks DIER before it ANDs, and why they are outside every vector's set.
static_assert((Tim<1>::interrupt_flags & Tim<1>::overcapture_flag(0)) == 0u);
static_assert((Tim<1>::all_flags & ~Tim<1>::interrupt_flags) ==
              (TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF));
static_assert(TIM_DIER_CC1DE == TIM_SR_CC1OF && TIM_DIER_CC4DE == TIM_SR_CC4OF,
              "the overlap the isr() body's DIER mask is there for");

// ---- the internal trigger table ---------------------------------------------

// TIM1's row is the same on all three manuals, and TIM5 exists everywhere.
static_assert(tim_internal_trigger(1, 0) == 5);
static_assert(tim_trigger_index_for(1, 5) == 0);
static_assert(tim_internal_trigger(1, 4) == 0, "there is no ITR4");
static_assert(tim_trigger_index_for(1, 0) == 0xFFu);

// A master the device has not got is no link at all - which is how the
// three TIM8 entries disappear on the parts whose manuals print them
// Reserved, with no device name spelled anywhere.
#if defined(TIM8_BASE)
static_assert(tim_internal_trigger(2, 1) == 8 && tim_trigger_index_for(4, 8) == 3);
static_assert(tim_internal_trigger(5, 3) == 8);
#elif defined(TIM2_BASE)
static_assert(tim_internal_trigger(2, 1) == 0 && tim_trigger_index_for(4, 8) == 0xFFu);
static_assert(tim_internal_trigger(5, 3) == 0);
#endif

// TIM9's third and fourth triggers are one-channel timers' OUTPUTS and
// not a TRGO, because those timers have no CR2 - the reserve's
// has_master_mode is what says so.
#if defined(TIM10_BASE)
static_assert(tim_internal_trigger(9, 2) == 10 && tim_internal_trigger_is_oc(9, 2));
#endif
static_assert(tim_internal_trigger(9, 3) == 11 && tim_internal_trigger_is_oc(9, 3));
#if defined(TIM2_BASE)
static_assert(tim_internal_trigger(9, 0) == 2 && !tim_internal_trigger_is_oc(9, 0));
static_assert(tim_internal_trigger(3, 2) == 5 && tim_trigger_index_for(3, 5) == 2);
#else
static_assert(tim_internal_trigger(9, 0) == 0, "no TIM2 on this part, no ITR0 for TIM9");
#endif

// ---- the dead-time generator's arithmetic ------------------------------------

static_assert(tim_dead_time_ticks(0) == 0);
static_assert(tim_dead_time_ticks(127) == 127, "the first range is one tick a code");
static_assert(tim_dead_time_ticks(0x80) == 128, "(64 + 0) x 2");
static_assert(tim_dead_time_ticks(0xBF) == 254, "(64 + 63) x 2");
static_assert(tim_dead_time_ticks(0xC0) == 256, "(32 + 0) x 8");
static_assert(tim_dead_time_ticks(0xDF) == 504);
static_assert(tim_dead_time_ticks(0xE0) == 512, "(32 + 0) x 16");
static_assert(tim_dead_time_ticks(0xFF) == 1008, "the longest this generator reaches");
// The search always rounds UP and never past the ranges.
static_assert(tim_dead_time_ticks(tim_dead_time_code(200)) >= 200);
static_assert(tim_dead_time_code(0) == 0 && tim_dead_time_code(100) == 100);
static_assert(tim_dead_time_code(1008) == 0xFFu);
static_assert(tim_dead_time_code(1009) == 0xFFu, "0xFF is also 'cannot'");

// ---- TIMxCLK from a clock task's own dividers ---------------------------------

// The 16 MHz reset rate compiles on every header of the pack and divides
// neither bus, so every timer counts HCLK there, TIMPRE or not.
using Reset16 = Clock<ClockSource::hsi, 16'000'000>;
static_assert(Reset16::apb1_div == 1 && Reset16::apb2_div == 1);
static_assert(tim_clock_hz(Reset16{}, false) == 16'000'000u);
static_assert(tim_clock_hz(Reset16{}, true) == 16'000'000u);
static_assert(tim_clock_hz(Reset16{}, false, true) == 16'000'000u);
static_assert(Tim<5>::clock_hz(Reset16{}) == 16'000'000u);
static_assert(Tim<1>::on_apb2 && !Tim<5>::on_apb2);

// The rule bites where a bus IS divided, and only the ladders the reserve
// knows can name such a rate - so the check is written for the part
// classes whose manual was read, and skipped elsewhere.
#if defined(STM32F411xE)
using Fast = Clock<ClockSource::pll_hse, 100'000'000, 25'000'000>;
static_assert(Fast::apb1_div == 2 && Fast::apb2_div == 1);
static_assert(tim_clock_hz(Fast{}, false) == 100'000'000u,
              "twice a halved PCLK1 is HCLK again");
static_assert(tim_clock_hz(Fast{}, true) == 100'000'000u);
static_assert(tim_clock_hz(Fast{}, false, true) == 100'000'000u,
              "and TIMPRE cannot move a prescaler of 2");
static_assert(rcc_has_timpre(), "RM0383 6.3.24 gives this part the bit");
#elif defined(STM32F429xx) || defined(STM32F446xx)
using Fast = Clock<ClockSource::pll_hse, 180'000'000, 8'000'000>;
static_assert(Fast::apb1_div == 4 && Fast::apb2_div == 2);
static_assert(tim_clock_hz(Fast{}, false) == 90'000'000u, "twice a quartered PCLK1");
static_assert(tim_clock_hz(Fast{}, true) == 180'000'000u, "twice a halved PCLK2");
static_assert(tim_clock_hz(Fast{}, false, true) == 180'000'000u, "four times, with TIMPRE");
static_assert(rcc_has_timpre());
#elif defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
using Fast = Clock<ClockSource::pll_hse, 168'000'000, 8'000'000>;
static_assert(Fast::apb1_div == 4 && Fast::apb2_div == 2);
static_assert(tim_clock_hz(Fast{}, false) == 84'000'000u);
static_assert(tim_clock_hz(Fast{}, true) == 168'000'000u);
static_assert(!rcc_has_timpre(), "the F405 class's RCC has no TIMPRE at all");
#endif

// ---- the vocabulary's own numbers --------------------------------------------

static_assert(static_cast<uint8_t>(TimSlaveMode::external_clock1) == 7,
              "SMS is THREE bits here - no combined reset+trigger mode");
static_assert(static_cast<uint8_t>(TimOutputMode::pwm2) == 7,
              "OCxM is THREE bits here - no combined or asymmetric PWM");
static_assert(static_cast<uint8_t>(TimCapturePolarity::both) == 3,
              "CCxNP:CCxP, and code 2 is reserved");
static_assert(static_cast<uint8_t>(TimBurstBase::arr) == 11);
static_assert(static_cast<uint8_t>(Tim5Input4::lse) == 2);
static_assert(static_cast<uint8_t>(Tim11Input1::hse_rtc) == 2);
static_assert(static_cast<uint8_t>(Tim2Trigger1::otg_fs_sof) == 2);
static_assert(tim_etr_config_valid({.prescaler = 3, .filter = 15}));
static_assert(!tim_etr_config_valid({.prescaler = 4}));
static_assert(!tim_etr_config_valid({.filter = 16}));

// A configuration is judged for the INSTANCE, at compile time, which is
// what lets a task static_assert on it.
static_assert(Tim<5>::config_valid({.period = 0x1FFFFu}), "a 32-bit counter");
static_assert(!Tim<1>::config_valid({.period = 0x1FFFFu}), "a 16-bit one");
static_assert(!Tim<1>::config_valid({.period = 0}), "a null auto-reload blocks the counter");
static_assert(Tim<1>::config_valid({.repetition = 4}));
static_assert(!Tim<9>::config_valid({.repetition = 4}), "no RCR on this instance");
static_assert(!Tim<9>::config_valid({.direction = TimDirection::down}));
static_assert(!Tim<9>::config_valid({.alignment = TimAlignment::center_up}));
static_assert(!Tim<5>::config_valid({.clock_division = static_cast<TimClockDivision>(3)}));

// ---- every verb of the resource, once ----------------------------------------

template <uint8_t n>
void tim_block_verbs() {
    using T = Tim<n>;
    T::init();
    (void)T::bus_clock();
    T::bus_clock(true);
    T::reset();
    (void)T::regs().CR1;
    (void)T::irq();
    (void)T::cc_irq();
    (void)T::break_irq();
    (void)T::trigger_irq();
    (void)T::vector_flags(T::irq());
    (void)T::clock_hz(Reset16{});
    (void)T::clock_hz(Reset16{}, true);
    (void)T::clock_hz_now(16'000'000u);

    (void)T::configure({.prescaler = 7, .period = 99, .auto_reload_preload = true});
    (void)T::configure({.prescaler = 0, .period = 1, .one_pulse = true,
                        .update_disable = true, .update_on_overflow_only = true});
    T::enable(true);
    (void)T::enabled();
    (void)T::count();
    T::set_count(3);
    (void)T::direction();
    (void)T::prescaler();
    T::set_prescaler(1);
    (void)T::period();
    (void)T::set_period(50);
    (void)T::auto_reload_preload();
    (void)T::repetition();
    (void)T::set_repetition(2);

    T::update();
    (void)T::capture_compare_event(0);
    (void)T::trigger_event();
    (void)T::commutation_event();
    (void)T::break_event();

    (void)T::flags();
    (void)T::flag(T::update_flag);
    T::clear_flags(T::all_flags);
    (void)T::compare_flag(1);
    (void)T::overcapture_flag(1);
    T::interrupts(T::update_interrupt | T::trigger_interrupt | T::break_interrupt |
                      T::commutation_interrupt | T::compare_interrupt(0),
                  true);
    T::interrupts(T::update_dma | T::trigger_dma | T::commutation_dma | T::compare_dma(0),
                  false);
    (void)T::interrupts();
    (void)T::isr();
    (void)T::isr(T::vector_flags(T::irq()));

    (void)T::compare(0);
    (void)T::set_compare(0, 10);
    (void)T::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 5});
    (void)T::output_channel(0, {.mode = TimOutputMode::toggle,
                                .preload = false,
                                .fast = true,
                                .clear_on_ocref_clr = true,
                                .active_low = true,
                                .complementary_active_low = true,
                                .complementary_enable = true,
                                .idle_high = true,
                                .complementary_idle_high = true});
    (void)T::capture_channel(0, {.select = TimChannelSelect::direct,
                                 .polarity = TimCapturePolarity::both,
                                 .prescaler = TimCapturePrescaler::every4,
                                 .filter = 3});
    (void)T::capture_channel(0, {.select = TimChannelSelect::trc});
    (void)T::capture_channel(0, {.select = TimChannelSelect::output});   // refused
    (void)T::output_mode(0, TimOutputMode::force_active);
    (void)T::output_mode(0);
    (void)T::output_mode(3, TimOutputMode::frozen);   // refused where there is no channel 4
    (void)T::channel_enable(0, true);
    (void)T::channel_enabled(0);
    (void)T::complementary_enable(0, true);
    (void)T::complementary_enabled(0);

    (void)T::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::itr0,
                    .master_slave = true});
    (void)T::slave({.mode = TimSlaveMode::encoder3});
    (void)T::slave({.mode = TimSlaveMode::gated, .trigger = TimTrigger::ti1_edge});  // refused
    (void)T::slave_mode();
    (void)T::slave_trigger();
    (void)T::master(TimMasterMode::update);
    (void)T::master(TimMasterMode::oc1ref);
    (void)T::master();
    (void)T::ti1_xor(true);
    (void)T::ti1_xor();
    (void)T::preload_channels(true, true);
    (void)T::compare_dma_on_update(true);

    (void)T::break_dead_time({.dead_time = 0x40, .automatic_output_enable = true,
                              .break_enable = true, .break_active_high = true,
                              .off_state_run = true, .off_state_idle = true, .lock = 1});
    (void)T::break_dead_time({.lock = 4});   // refused
    (void)T::main_output(true);
    (void)T::main_output();
    (void)T::dead_time_ticks();

    (void)T::external_trigger({.inverted = true, .prescaler = 2, .filter = 5,
                               .clock_mode2 = true});
    (void)T::external_trigger({.prescaler = 9});   // refused
    (void)T::external_clock_mode2();

    (void)T::option(1);
    (void)T::option(4);   // refused: the field is two bits
    (void)T::option();

    (void)T::dma_burst(TimBurstBase::arr, 3);
    (void)T::dma_burst(TimBurstBase::cr1, 0);   // refused
    (void)T::burst_base();
    (void)T::burst_length();
    T::dma_burst_off();
    (void)T::dmar_address();
    (void)T::ccr_address(0);

    T::release();
}

// ---- the tasks ---------------------------------------------------------------

// A four-channel timer carries every task this file has; TIM5 is the one
// such instance on every part of the pack, so it is the universal
// fixture and the others are exercised where they exist.
using Wide = Tim<5>;
using Advanced = Tim<1>;
using Small = Tim<11>;

using WidePwm = TimPwm<Wide, 0, 999>;
using AdvancedPwm = TimPwm<Advanced, 3, 255>;
using SmallPwm = TimPwm<Small, 0, 100>;
using Pair = TimPairPwm<Advanced, 0, 1000>;
using WideMeter = TimPeriodMeter<Wide>;
using WideInterval = TimIntervalMeter<Wide, 3>;
using SmallInterval = TimIntervalMeter<Small, 0>;
using WideEvents = TimEventCounter<Wide>;
using WideGate = TimGatedCounter<Wide>;
using WideTick = TimPeriodicTick<Wide>;
using SmallTick = TimPeriodicTick<Small>;
using WidePulse = TimOnePulse<Wide, 1>;
using WideEncoder = TimEncoder<Wide>;

static_assert(PwmChannel<WidePwm> && PwmChannel<AdvancedPwm> && PwmChannel<SmallPwm>);
static_assert(PwmChannel<Pair>);
static_assert(WidePwm::max == 999 && Pair::max == 1000);

void tim_task_verbs() {
    (void)WidePwm::setup(3);
    (void)WidePwm::setup(3, TimOutputMode::pwm2, true);
    WidePwm::duty(500);
    WidePwm::duty(0xFFFFu);   // clamped to max
    (void)WidePwm::duty();
    (void)SmallPwm::setup();
    SmallPwm::duty(50);

    (void)Pair::setup(0, tim_dead_time_code(80), TimClockDivision::div2);
    Pair::duty(400);
    (void)Pair::duty();
    (void)Pair::dead_time_ticks();

    (void)WideMeter::setup(9, 2);
    (void)WideMeter::setup(9, 2, true);
    (void)WideMeter::period_ticks();
    (void)WideMeter::width_ticks();
    (void)WideMeter::period_flag;
    (void)WideMeter::width_flag;
    (void)WideMeter::overrun_flag;
    (void)WideMeter::period_interrupt;

    (void)WideInterval::setup(0, 1, TimCapturePolarity::falling, TimCapturePrescaler::every8);
    (void)WideInterval::interval();
    WideInterval::restart();
    (void)WideInterval::capture_flag;
    (void)WideInterval::overrun_flag;
    (void)WideInterval::capture_interrupt;
    (void)SmallInterval::setup();
    (void)SmallInterval::interval();

    (void)WideEvents::setup(TimTrigger::itr0);
    (void)WideEvents::count();
    WideEvents::restart();

    (void)WideGate::setup(TimTrigger::itr1, 0, 0xFFFFu);
    (void)WideGate::count();
    WideGate::restart();

    (void)WideTick::setup(99, 999);
    WideTick::stop();
    (void)WideTick::flag;
    (void)SmallTick::setup(0, 100, false);

    (void)WidePulse::setup(9, 100, 200);
    (void)WidePulse::setup(9, 100, 0);   // refused
    (void)WidePulse::arm(TimTrigger::ti1);
    WidePulse::fire();
    (void)WidePulse::busy();

    (void)WideEncoder::setup();
    (void)WideEncoder::setup({.mode = TimSlaveMode::encoder1, .filter = 4,
                              .invert_a = true, .invert_b = true},
                             4095);
    (void)WideEncoder::setup({.mode = TimSlaveMode::reset});   // refused
    (void)WideEncoder::count();
    WideEncoder::set_count(0);
    (void)WideEncoder::reversing();
}

// ---- the option registers, each on its own instance ---------------------------

void tim_option_verbs() {
    (void)Tim<5>::input4_source(Tim5Input4::lse);
    (void)Tim<5>::input4_source(Tim5Input4::lsi);
    (void)Tim<5>::input4_source(Tim5Input4::rtc_wakeup);
    (void)Tim<5>::input4_source();
    (void)Tim<11>::input1_source(Tim11Input1::hse_rtc);
    (void)Tim<11>::input1_source();
#if defined(TIM2_BASE)
    (void)Tim<2>::trigger1_source(Tim2Trigger1::otg_fs_sof);
    (void)Tim<2>::trigger1_source();
#endif
}

// ---- pads ---------------------------------------------------------------------

// The AF number is the datasheet's and no header symbol can check it, so
// a pad is a claim the caller writes - here on port A, which every part
// of the family bonds.
constexpr PinSel wave_pad{'A', 6, PinFunction::af2};    // TIM3_CH1 on the F411
constexpr PinSel advanced_pad{'A', 8, PinFunction::af1};  // TIM1_CH1
using WavePad = TimPad<wave_pad>;
using AdvancedPad = TimPad<advanced_pad>;

static_assert(WavePad::selection.port == 'A' && WavePad::selection.pin == 6);

void tim_pad_verbs() {
    WavePad::claim();
    WavePad::claim(PinSpeed::very_high, true);
    WavePad::claim_input(PinPull::up);
    WavePad::drive(true);
    WavePad::set();
    WavePad::clear();
    (void)WavePad::read();
    WavePad::release();
    AdvancedPad::claim();
    AdvancedPad::release();
}

// ---- every instance this device has -------------------------------------------

void tim_every_instance() {
    tim_block_verbs<1>();
    tim_block_verbs<5>();
    tim_block_verbs<9>();
    tim_block_verbs<11>();
#if defined(TIM2_BASE)
    tim_block_verbs<2>();
    tim_block_verbs<3>();
    tim_block_verbs<4>();
#endif
#if defined(TIM6_BASE)
    tim_block_verbs<6>();
#endif
#if defined(TIM7_BASE)
    tim_block_verbs<7>();
#endif
#if defined(TIM8_BASE)
    tim_block_verbs<8>();
    tim_block_verbs<12>();
    tim_block_verbs<13>();
    tim_block_verbs<14>();
#endif
#if defined(TIM10_BASE)
    tim_block_verbs<10>();
#endif
}
