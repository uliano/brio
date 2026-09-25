// AFIO family smoke TU: the remap tables as constexpr data, every verb
// of the block, and the two questions a column is judged by - the device
// CLASS's columns and THIS package's bonding.
//
// The tables are the manual's and the same on every part; what differs
// is which columns a part can use, so this TU asserts the answers PER
// PART out of device:: rather than naming a package. The parts that
// prove the sweep is doing something: the CH32V203F6, which bonds
// neither PB6/PB7 (so USART1 has one column) nor PB8/PB9 (so I2C1 has
// one); the CH32V203F8 and G8, which bond no pin of port D at all (so
// the oscillator pads cannot become GPIO and CAN1 has no third column);
// the CH32V203RB, the other device class, whose UART4 reads the other
// table; and the four CH32V303, the third class - USART1's four columns,
// the fields of TIM8..TIM10, UART5..8, SPI3 and the ADC triggers where
// the 256 KB parts have the blocks, and on the LQFP100 the columns of
// ports D and E and the PD0/PD1 field that package does not need.
#include "ch32vx03/afio.hpp"

using namespace brio;

// ---- the fields are where the chapter puts them ---------------------------
static_assert(afio_field_of(Remap::spi1).shift == 0 && afio_field_of(Remap::spi1).width == 1);
static_assert(afio_field_of(Remap::usart3).shift == 4 && afio_field_of(Remap::usart3).width == 2);
static_assert(afio_field_of(Remap::can1).shift == 13 && afio_field_of(Remap::can1).width == 2);
static_assert(afio_field_of(Remap::ptp_pps).shift == 30);
static_assert(afio_field_of(Remap::uart4).second && afio_field_of(Remap::uart4).shift == 16);
static_assert(!afio_field_of(Remap::tim1).second);
// The CH32V303's fields (10.3.2.2, 10.3.2.7), which exist on its class
// alone.
static_assert(afio_field_present(Remap::uart4) && afio_field_present(Remap::spi1));
static_assert(afio_field_present(Remap::tim8) == (device::device_class == DeviceClass::v30x_d8));
static_assert(device::device_class != DeviceClass::v30x_d8 ||
              (!afio_field_of(Remap::spi3).second && afio_field_of(Remap::spi3).shift == 28 &&
               afio_field_of(Remap::tim8).second && afio_field_of(Remap::tim8).shift == 2 &&
               afio_field_of(Remap::tim8).width == 1 && afio_field_of(Remap::tim9).shift == 3 &&
               afio_field_of(Remap::tim9).width == 2 && afio_field_of(Remap::tim10).shift == 5 &&
               afio_field_of(Remap::tim10).width == 2 && afio_field_of(Remap::uart5).shift == 18 &&
               afio_field_of(Remap::uart8).shift == 24 &&
               afio_field_of(Remap::adc1_etrginj).shift == 17 &&
               afio_field_of(Remap::adc2_etrgreg).shift == 20 &&
               afio_field_of(Remap::fsmc_nadv).second &&
               afio_field_of(Remap::fsmc_nadv).shift == 10 &&
               afio_field_of(Remap::can2).shift == 22 && afio_field_of(Remap::eth).shift == 21));
// USART1's field is its low bit; the high one is the CH32V303's alone.
static_assert(afio_field_of(Remap::usart1).width == 1);
static_assert(afio_usart1_high_bit == (device::device_class == DeviceClass::v30x_d8));

// ---- the columns, code by code --------------------------------------------
// TIM1's partial column moves the break input and the three complementary
// outputs and nothing else (table 10-15).
static_assert(afio_tim1_pads(0).ch1 == afio_tim1_pads(1).ch1 &&
              afio_tim1_pads(0).etr == afio_tim1_pads(1).etr &&
              afio_tim1_pads(0).bkin != afio_tim1_pads(1).bkin);
// TIM2's four columns move the channels in two independent pairs.
static_assert(afio_tim2_pads(1).ch3 == afio_tim2_pads(0).ch3 &&
              afio_tim2_pads(2).ch1 == afio_tim2_pads(0).ch1 &&
              afio_tim2_pads(3).ch1 == afio_tim2_pads(1).ch1 &&
              afio_tim2_pads(3).ch3 == afio_tim2_pads(2).ch3);
// TIM9's and TIM10's 1x columns are one column twice.
static_assert(afio_tim9_pads(2).ch1 == afio_tim9_pads(3).ch1 &&
              afio_tim10_pads(2).bkin == afio_tim10_pads(3).bkin);
// TIM8's remap keeps its ETR on PA0.
static_assert(afio_tim8_pads(0).etr == afio_tim8_pads(1).etr);
// A USART instance answers with its own table, and an instance this part
// has not got answers with nothing at all.
static_assert(afio_usart_pads(1, 0).tx == (device::has_usart(1) ? Pad{'A', 9} : Pad{}));
static_assert(afio_usart_pads(4, 0).tx.valid() == device::has_usart(4));
static_assert(afio_usart_pads(5, 0).tx.valid() == device::has_usart(5));
static_assert(afio_usart_pads(8, 1).tx == (device::has_usart(8) ? Pad{'A', 14} : Pad{}));
static_assert(!afio_usart_pads(9, 0).tx.valid());

// ---- what THIS part can reach ---------------------------------------------
// Both SPI1 columns need port A or B pads every package brings out.
static_assert(afio_remap_has_code(Remap::spi1, 0));
static_assert(!afio_remap_has_code(Remap::spi1, 2));
// USART1's remap is PB6/PB7: the smallest package bonds neither, and it
// has no USART1 either. Its two upper columns are the CH32V303's.
static_assert(afio_remap_has_code(Remap::usart1, 1) ==
              (device::has_usart(1) && pad_bonded(Pad{'B', 6})));
static_assert(afio_remap_has_code(Remap::usart1, 2) ==
              (device::has_usart(1) && afio_usart1_high_bit));
static_assert(!afio_remap_has_code(Remap::usart1, 4));
// The oscillator's pads as GPIO: the part table's own fact - every part
// whose package shares them with PD0/PD1, which the LQFP100 does not.
static_assert(afio_remap_has_code(Remap::pd0_pd1, 1) == device::osc_pads_as_pd0_pd1);
static_assert(!device::osc_pads_as_pd0_pd1 || pad_bonded(Pad{'D', 0}));
// TIM3's full column is port C's, which only the 64-pin and 100-pin
// parts bond.
static_assert(afio_remap_has_code(Remap::tim3, 3) == pad_bonded(Pad{'C', 6}));
// The Ethernet's pulse-per-second exists where the Ethernet does.
static_assert(afio_remap_has_code(Remap::ptp_pps, 1) == device::has_ethernet);
// TIM5's channel 4 from the LSI: the timer is three parts'.
static_assert(afio_remap_has_code(Remap::tim5_ch4, 1) == device::has_tim5);
// UART4's second column, on the part that has the fourth serial port.
static_assert(afio_remap_has_code(Remap::uart4, 1) == device::has_usart(4));
// UART5..8 where the part has them; their third columns are port E's.
static_assert(afio_remap_has_code(Remap::uart5, 0) == device::has_usart(5));
static_assert(afio_remap_has_code(Remap::uart8, 2) ==
              (device::has_usart(8) && device::has_port('E')));
// SPI3's two columns where the part has the third SPI.
static_assert(afio_remap_has_code(Remap::spi3, 1) ==
              ((device::spi_instances & (1U << 3)) != 0u));
// The three extra advanced timers, their upper columns the LQFP100's.
static_assert(afio_remap_has_code(Remap::tim8, 0) == afio_advanced_timer(8));
static_assert(afio_remap_has_code(Remap::tim9, 1) == afio_advanced_timer(9));
static_assert(afio_remap_has_code(Remap::tim10, 2) ==
              (afio_advanced_timer(10) && device::has_port('E')));
static_assert(afio_remap_has_code(Remap::tim9, 3) ==
              (afio_advanced_timer(9) && pad_bonded(Pad{'D', 15})));
// The ADC triggers from TIM8, and the FSMC's address-valid pad.
static_assert(afio_remap_has_code(Remap::adc1_etrgreg, 1) == afio_advanced_timer(8));
static_assert(afio_remap_has_code(Remap::adc2_etrginj, 1) ==
              (afio_advanced_timer(8) && device::adc_count >= 2));
static_assert(afio_remap_has_code(Remap::fsmc_nadv, 1) == device::has_fsmc);
// The two fields named only to be refused.
static_assert(!afio_remap_has_code(Remap::can2, 0) && !afio_remap_has_code(Remap::can2, 1));
static_assert(!afio_remap_has_code(Remap::eth, 0) && !afio_remap_has_code(Remap::eth, 1));
// A column is "reachable" as soon as ONE pad of it is bonded; whether
// every signal is there is the other question.
static_assert(afio_column_fully_bonded(Remap::spi1, 0) ==
              (pad_bonded(Pad{'A', 4}) && pad_bonded(Pad{'A', 7})));
static_assert(!afio_column_fully_bonded(Remap::can2, 0));
// A column asked for by its TX pad: the code that puts it there on THIS
// part, or none - UART4's PC10 is code 0 on every class but the D6's,
// and no code at all on the CH32V203C8.
static_assert(afio_usart_remap(4) == Remap::uart4 && afio_usart_remap(8) == Remap::uart8);
static_assert(afio_usart_code_for(4, Pad{'C', 10}) ==
              (device::has_usart(4) && device::device_class != DeviceClass::v20x_d6 ? 0u : 0xFFu));
static_assert(afio_usart_code_for(1, Pad{'A', 9}) == (device::has_usart(1) ? 0u : 0xFFu));
static_assert(afio_usart_code_for(1, Pad{'A', 6}) ==
              (device::has_usart(1) && afio_usart1_high_bit ? 3u : 0xFFu));
static_assert(afio_usart_code_for(9, Pad{'A', 9}) == 0xFFu && afio_usart_code_for(1, Pad{}) == 0xFFu);

/// Every remap this stratum names, walked: the run-time verb takes the
/// codes afio_remap_has_code() allows and refuses the others.
void afio_verbs() {
    Afio::clock_on();
    (void)Afio::regs().PCFR1;

    constexpr Remap all[] = {Remap::spi1,         Remap::i2c1,         Remap::usart1,
                             Remap::usart2,       Remap::usart3,       Remap::tim1,
                             Remap::tim2,         Remap::tim3,         Remap::tim4,
                             Remap::can1,         Remap::pd0_pd1,      Remap::tim5_ch4,
                             Remap::tim2_itr1,    Remap::ptp_pps,      Remap::uart4,
                             Remap::spi3,         Remap::tim8,         Remap::tim9,
                             Remap::tim10,        Remap::uart5,        Remap::uart6,
                             Remap::uart7,        Remap::uart8,        Remap::adc1_etrginj,
                             Remap::adc1_etrgreg, Remap::adc2_etrginj, Remap::adc2_etrgreg,
                             Remap::fsmc_nadv,    Remap::can2,         Remap::eth};
    for (const Remap r : all) {
        for (uint8_t code = 0; code < 4u; ++code) {
            if (Afio::remap(r, code)) {
                (void)Afio::remap_code(r);
            }
            (void)afio_column_fully_bonded(r, code);
        }
        (void)Afio::remap(r, 0);
    }

    // The compile-time face, on a column every part of the series has.
    Afio::remap<Remap::spi1, 0>();
    Afio::remap<Remap::tim2, 0>();
    if constexpr (device::has_usart(1)) {
        Afio::remap<Remap::usart1, 0>();
    }
    // And on the CH32V303's own, where the part has them.
    if constexpr (afio_remap_has_code(Remap::usart1, 3)) {
        Afio::remap<Remap::usart1, 3>();
    }
    if constexpr (afio_remap_has_code(Remap::tim8, 1)) {
        Afio::remap<Remap::tim8, 1>();
    }
    if constexpr (afio_remap_has_code(Remap::spi3, 1)) {
        Afio::remap<Remap::spi3, 1>();
    }

    // The debug port: read, never written (the verb that writes it is
    // spelled long and is not called here - it would cost the probe).
    (void)Afio::debug_config();
    (void)Afio::debug_port_enabled();

    // The EXTI multiplexer, every line and every port this part bonds.
    for (uint8_t line = 0; line < 16u; ++line) {
        (void)Afio::exti_source(line, 'A');
        (void)Afio::exti_source(line);
    }
    (void)Afio::exti_source(16, 'A');
    (void)Afio::exti_source('Z');

    // The event output, on the lowest pad of port A - which every part
    // of this series bonds.
    (void)Afio::event_output('A', 0);
    (void)Afio::event_output_enabled();
    (void)Afio::event_output_port();
    (void)Afio::event_output_pin();
    Afio::event_output_off();
}
