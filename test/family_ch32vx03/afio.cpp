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
// and the CH32V203RB, the other device class, whose UART4 reads the
// other table.
#include "ch32vx03/afio.hpp"

using namespace brio;

// ---- the fields are where the chapter puts them ---------------------------
static_assert(afio_field_of(Remap::spi1).shift == 0 && afio_field_of(Remap::spi1).width == 1);
static_assert(afio_field_of(Remap::usart3).shift == 4 && afio_field_of(Remap::usart3).width == 2);
static_assert(afio_field_of(Remap::can1).shift == 13 && afio_field_of(Remap::can1).width == 2);
static_assert(afio_field_of(Remap::ptp_pps).shift == 30);
static_assert(afio_field_of(Remap::uart4).second && afio_field_of(Remap::uart4).shift == 16);
static_assert(!afio_field_of(Remap::tim1).second);

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
// A USART instance answers with its own table, and an instance this part
// has not got answers with nothing at all.
static_assert(afio_usart_pads(1, 0).tx == (device::has_usart(1) ? Pad{'A', 9} : Pad{}));
static_assert(afio_usart_pads(4, 0).tx.valid() == device::has_usart(4));
static_assert(!afio_usart_pads(5, 0).tx.valid());

// ---- what THIS part can reach ---------------------------------------------
// Both SPI1 columns need port A or B pads every package brings out.
static_assert(afio_remap_has_code(Remap::spi1, 0));
static_assert(!afio_remap_has_code(Remap::spi1, 2));
// USART1's remap is PB6/PB7: the smallest package bonds neither, and it
// has no USART1 either.
static_assert(afio_remap_has_code(Remap::usart1, 1) ==
              (device::has_usart(1) && pad_bonded(Pad{'B', 6})));
// The oscillator's pads as GPIO: exactly the parts whose table bonds them.
static_assert(afio_remap_has_code(Remap::pd0_pd1, 1) == pad_bonded(Pad{'D', 0}));
// TIM3's full column is port C's, which only the 64-pin part bonds.
static_assert(afio_remap_has_code(Remap::tim3, 3) == pad_bonded(Pad{'C', 6}));
// The Ethernet's pulse-per-second exists where the Ethernet does.
static_assert(afio_remap_has_code(Remap::ptp_pps, 1) == device::has_ethernet);
// TIM5's channel 4 from the LSI: the timer is one part's.
static_assert(afio_remap_has_code(Remap::tim5_ch4, 1) == device::has_tim5);
// UART4's second column, on the part that has the fourth serial port.
static_assert(afio_remap_has_code(Remap::uart4, 1) == device::has_usart(4));
// A column is "reachable" as soon as ONE pad of it is bonded; whether
// every signal is there is the other question.
static_assert(afio_column_fully_bonded(Remap::spi1, 0) ==
              (pad_bonded(Pad{'A', 4}) && pad_bonded(Pad{'A', 7})));

/// Every remap this part has, walked: the run-time verb takes the codes
/// afio_remap_has_code() allows and refuses the others.
void afio_verbs() {
    Afio::clock_on();
    (void)Afio::regs().PCFR1;

    constexpr Remap all[] = {Remap::spi1,      Remap::i2c1,      Remap::usart1,
                             Remap::usart2,    Remap::usart3,    Remap::tim1,
                             Remap::tim2,      Remap::tim3,      Remap::tim4,
                             Remap::can1,      Remap::pd0_pd1,   Remap::tim5_ch4,
                             Remap::tim2_itr1, Remap::ptp_pps,   Remap::uart4};
    for (const Remap r : all) {
        for (uint8_t code = 0; code < 4u; ++code) {
            if (Afio::remap(r, code)) {
                (void)Afio::remap_code(r);
            }
        }
        (void)Afio::remap(r, 0);
    }

    // The compile-time face, on a column every part of the series has.
    Afio::remap<Remap::spi1, 0>();
    Afio::remap<Remap::tim2, 0>();

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
