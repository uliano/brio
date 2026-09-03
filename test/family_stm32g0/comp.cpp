// Family smoke TU: stm32g0/comp.hpp (RM0444 ch. 18). Instantiation only.
//
// THE PER-HEADER DIFFERENCES THIS FIXTURE IS FOR, and they are the
// widest in the analog set: the G0B1/G0C1 class carries THREE
// comparators, the G071 class TWO, and the G031 class NONE - each header
// saying so by declaring or not declaring COMPn_BASE. On top of that,
// COMP3's third plus input and third minus input are PE7 and PE8, so
// they exist only where port E is bonded, which is the same class
// boundary reached from the GPIO side.

#include <stdint.h>

#include "stm32g0/device_tables.hpp"
#include "stm32g0/platform_stm32.hpp"

using namespace brio;

#if defined(STM32G031xx)

static_assert(comp_count() == 0, "the G031 class has no comparator (18.1)");
static_assert(!comp_present(1) && comp_exti_line(1) == 0xFF);

#else

#include "stm32g0/comp.hpp"

static_assert(comp_present(1) && comp_present(2));
static_assert(comp_exti_line(1) == 17 && comp_exti_line(2) == 18,
              "13.5.1's line table: the comparators are configurable lines");
static_assert(comp_irq() == adc_irq(),
              "the comparators report on the ADC's vector (table 61)");

#if defined(STM32G0B1xx)
static_assert(comp_count() == 3 && comp_present(3), "the G0B1 class has the third");
static_assert(comp_exti_line(3) == 20);
static_assert(comp_positive_pin(3, CompPositive::input2).port == 'E',
              "COMP3_INP2 is PE7, and port E is this class's");
#else
static_assert(comp_count() == 2 && !comp_present(3), "the G071 class has two");
static_assert(comp_exti_line(3) == 0xFF);
#endif

// The input tables (93..98), which are NOT a pattern.
static_assert(comp_positive_pin(1, CompPositive::input0).port == 'C' &&
              comp_positive_pin(1, CompPositive::input0).pin == 5);
static_assert(comp_positive_pin(1, CompPositive::input2).port == 'A' &&
              comp_positive_pin(1, CompPositive::input2).pin == 1);
static_assert(comp_positive_pin(2, CompPositive::input2).port == 'A' &&
              comp_positive_pin(2, CompPositive::input2).pin == 3);
static_assert(!comp_positive_pin(1, CompPositive::open).valid, "open is not a pad");
static_assert(comp_negative_pin(1, CompNegative::input8).port == 'A' &&
              comp_negative_pin(1, CompNegative::input8).pin == 0);
static_assert(comp_negative_pin(2, CompNegative::input6).port == 'B' &&
              comp_negative_pin(2, CompNegative::input6).pin == 3);
static_assert(!comp_negative_pin(1, CompNegative::vrefint).valid,
              "an internal threshold is not a pad");

// The window partner, which is the asymmetry 18.6.1 states one register
// at a time: 1 borrows from 2, 2 borrows from 1, 3 borrows from 2.
static_assert(comp_window_partner(1) == 2 && comp_window_partner(2) == 1);
#if defined(STM32G0B1xx)
static_assert(comp_window_partner(3) == 2);
#endif

// The refusals.
constexpr CompConfig plain{.positive = CompPositive::input2,
                           .negative = CompNegative::vrefint_half};
static_assert(comp_config_valid(1, plain));
static_assert(!comp_config_valid(1, {.power = static_cast<CompPower>(2)}),
              "18.6.1: the other two PWRMODE codes are Reserved");
static_assert(!comp_config_valid(1, {.blanking = 0x20}), "BLANKSEL is five bits");
static_assert(comp_config_valid(1, {.negative = CompNegative::dac_channel1}) ==
                  dac_present(),
              "a DAC threshold needs a DAC");

using C1 = Comp<1>;
using C2 = Comp<2>;
static_assert(C1::exti_line == 17 && C2::exti_line == 18);
static_assert(C1::window_partner == 2 && C2::window_partner == 1);

void use() {
    C1::init();
    (void)C1::bus_clock();
    (void)C1::claim_inputs(plain);
    (void)C1::configure(plain);
    (void)C1::configure({.positive = CompPositive::input0,
                         .negative = CompNegative::input7,
                         .hysteresis = CompHysteresis::high,
                         .power = CompPower::medium_speed,
                         .inverted = true,
                         .blanking = CompBlank::tim1_oc4 | CompBlank::tim2_oc3});
    (void)C1::enable(true);
    (void)C1::enabled();
    (void)C1::value();
    (void)C1::positive();
    (void)C1::negative();
    (void)C1::hysteresis();
    (void)C1::power();
    (void)C1::inverted();
    (void)C1::window_input();
    (void)C1::window_output();
    (void)C1::blanking();
    (void)C1::locked();
    (void)C1::irq();
    (void)C1::positive_pin(CompPositive::input1);
    (void)C1::negative_pin(CompNegative::input6);
    (void)C1::release();

    (void)C2::configure({.positive = CompPositive::input1,
                         .negative = CompNegative::vrefint,
                         .window_input = true,
                         .window_output = true});
    (void)C2::release();

#if defined(STM32G0B1xx)
    (void)Comp<3>::configure({.positive = CompPositive::input0,
                              .negative = CompNegative::vrefint_quarter});
    (void)Comp<3>::release();
#endif
}

#endif
