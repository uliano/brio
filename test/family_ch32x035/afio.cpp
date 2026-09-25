// AFIO family smoke TU: the remap fields of AFIO_PCFR1 where the manual
// puts them (RM 8.3.2.1), the USARTs' columns, the codes a part has, the
// debug port's field and the EXTI port multiplexer with this series'
// codes (00 A, 10 B, 11 C).
#include "ch32x035/afio.hpp"

using namespace brio;

static_assert(afio_field_of(Remap::spi1).shift == 0 && afio_field_of(Remap::spi1).width == 2);
static_assert(afio_field_of(Remap::i2c1).shift == 2 && afio_field_of(Remap::i2c1).width == 3);
static_assert(afio_field_of(Remap::usart1).shift == 5 && afio_field_of(Remap::usart1).width == 2);
static_assert(afio_field_of(Remap::usart2).shift == 7 && afio_field_of(Remap::usart2).width == 3);
static_assert(afio_field_of(Remap::usart3).shift == 10 && afio_field_of(Remap::usart3).width == 2);
static_assert(afio_field_of(Remap::usart4).shift == 12 && afio_field_of(Remap::usart4).width == 3);
static_assert(afio_field_of(Remap::tim1).shift == 15 && afio_field_of(Remap::tim1).width == 3);
static_assert(afio_field_of(Remap::tim2).shift == 18 && afio_field_of(Remap::tim2).width == 3);
static_assert(afio_field_of(Remap::tim3).shift == 21 && afio_field_of(Remap::tim3).width == 2);
static_assert(afio_field_of(Remap::pioc).shift == 23 && afio_field_of(Remap::pioc).width == 1);

// The columns (8.3.2.1): USART2's default is PA2/PA3 with CK on PA4, its
// four upper codes are one column, USART3's code 3 has no CK, USART4's
// 1x0 and 1x1 are two.
static_assert(afio_usart_pads(2, 0).tx == Pad{'A', 2} && afio_usart_pads(2, 0).rx == Pad{'A', 3});
static_assert(afio_usart_pads(2, 0).ck == Pad{'A', 4});
static_assert(afio_usart_pads(2, 4).tx == afio_usart_pads(2, 7).tx);
static_assert(!afio_usart_pads(3, 3).ck.valid());
static_assert(afio_usart_pads(4, 4).tx == Pad{'B', 13} && afio_usart_pads(4, 6).tx == Pad{'B', 13});
static_assert(afio_usart_pads(4, 5).tx == Pad{'C', 17} && afio_usart_pads(4, 7).rx == Pad{'C', 16});
static_assert(!afio_usart_pads(1, 4).tx.valid());

// A code that does not fit its field is no code; every package bonds a
// pad of USART2's default column.
static_assert(!afio_remap_has_code(Remap::usart1, 4));
static_assert(!afio_remap_has_code(Remap::pioc, 2));
static_assert(afio_remap_has_code(Remap::usart2, 0));
static_assert(afio_remap_has_code(Remap::tim2, 7));

void afio_verbs() {
    Afio::clock_on();
    (void)Afio::remap(Remap::usart2, 0);
    Afio::remap<Remap::usart2, 0>();
    (void)Afio::remap_code(Remap::usart2);
    (void)Afio::remap(Remap::tim3, 1);
    (void)Afio::debug_config();
    (void)Afio::debug_port_enabled();
    (void)Afio::exti_source(3, 'A');
    (void)Afio::exti_source(19, 'C');
    (void)Afio::exti_source(3);
    (void)Afio::control();
}

// Never called: the one verb that gives the probe's pads away.
void afio_debug_off() { Afio::disable_debug_port_until_reset(); }
