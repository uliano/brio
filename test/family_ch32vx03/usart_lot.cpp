// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The USART's LOT-KEYED features family smoke TU: CTLR4's MARK and SPACE
// parity with MS_ERRIE, CTLR1's M_EXT short words, and STATR's MS_ERR and
// RX_BUSY - the CH32V30x_D8's, on the lots whose penultimate sixth digit
// is not zero (18.10's notes). Every CH32V303 compiles these verbs, since
// the class has them; whether a DIE has them is what the verbs ask at run
// time. A neg TU proves them refused on the CH32V203.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using namespace brio;

using U2 = Usart<2>;
using U3 = Usart<3>;

static_assert(usart_class_has_lot_registers && U2::has_lot_registers);

// CHECK_SEL's codes: 0x off, 10 mark, 11 space; a reserved code reads back
// as off, which is what it means.
static_assert(usart_ctlr4_check(UsartMarkSpace::mark) == usart_check_mark);
static_assert(usart_ctlr4_check(UsartMarkSpace::space) == usart_check_space);
static_assert(usart_ctlr4_check(UsartMarkSpace::off) == 0u);
static_assert(usart_mark_space_of(usart_check_mark | usart_ms_errie) == UsartMarkSpace::mark);
static_assert(usart_mark_space_of(0x1u << 2) == UsartMarkSpace::off);
static_assert(usart_mark_space_valid(UsartMarkSpace::space) &&
              !usart_mark_space_valid(static_cast<UsartMarkSpace>(1)));
static_assert(usart_ctlr4_bits == 0x000Eu);
// M_EXT's codes, and the data a short word carries.
static_assert(usart_ctlr1_short_word(UsartShortWord::five) == usart_m_ext_mask);
static_assert(usart_ctlr1_short_word(UsartShortWord::seven) == (1u << usart_m_ext_shift));
static_assert(usart_short_word_mask(UsartShortWord::six) == 0x3Fu &&
              usart_short_word_mask(UsartShortWord::five) == 0x1Fu);
static_assert(!usart_short_word_valid(static_cast<UsartShortWord>(4)));
// The two status bits beside the ones every class has.
static_assert(usart_rx_busy == (1u << 10) && usart_ms_err == (1u << 11));

void lot_verbs() {
    U2::bus_clock(true);
    U2::reset();
    (void)U2::configure(UartFormat{UartBits::eight, UartParity::even}, 625);
    // The probe, on a disabled port, and what it kept.
    (void)U2::ctlr4_present();
    (void)U2::ctlr4_known();
    (void)U2::mark_space(UsartMarkSpace::mark);
    (void)U2::mark_space();
    (void)U2::mark_space_interrupt(true);
    (void)U2::mark_space(UsartMarkSpace::off);
    (void)U2::short_word(UsartShortWord::seven);
    (void)U2::short_word();
    (void)U2::short_word(UsartShortWord::off);
    U2::enable(true);
    (void)U2::mark_space_error();
    (void)U2::receiving();
    U2::clear_by_read();
    U2::bus_clock(false);

    U3::bus_clock(true);
    (void)U3::ctlr4_present();
    (void)U3::short_word(UsartShortWord::five);
    U3::bus_clock(false);
}
