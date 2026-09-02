// EXTI family smoke TU (RM0444 ch. 13). The GPIO half of this
// peripheral is the uniform half - sixteen lines, numbered by the pin
// number, on every part - so what this TU really checks is that the
// PER-PART half comes from the device header and not from a list: the
// implemented and configurable line masks, the second register group's
// struct members, and the three vectors.
#include "stm32g0/exti.hpp"

using namespace brio;

// ---- the reserve's own answers ---------------------------------------------

static_assert(exti_gpio_lines == 16, "EXTICR is four registers of four fields");

// Lines 0..15 are GPIO lines, configurable, and implemented on every
// part of the family.
static_assert((exti_implemented_mask1 & 0xFFFFu) == 0xFFFFu);
static_assert((exti_configurable_mask1 & 0xFFFFu) == 0xFFFFu);
static_assert(exti_line_implemented(0) && exti_line_configurable(15));

// A configurable line is implemented by construction; the converse is
// not true (a direct line is implemented and not configurable).
static_assert((exti_configurable_mask1 & ~exti_implemented_mask1) == 0u);
static_assert((exti_configurable_mask2 & ~exti_implemented_mask2) == 0u);

// The RTC is line 19 on every part of the x1 line, and it is DIRECT -
// which is what "no trigger selection bit" means. It is named here as
// the one cross-variant landmark of table 65, not as vocabulary this
// driver owns.
static_assert(exti_line_implemented(19) && !exti_line_configurable(19));

// Nothing past the header's own masks exists.
static_assert(!exti_line_implemented(64));
static_assert(!exti_line_configurable(64));

// The multiplexer's port codes: the manual's contiguous A..F encoding,
// gated by the header's port presence.
static_assert(exti_port_code('A') == 0);
static_assert(exti_port_code('B') == 1);
static_assert(exti_port_code('C') == 2);
static_assert(exti_port_code('D') == 3);
static_assert(exti_port_code('F') == 5);
static_assert(exti_port_code('G') == 0xFFu);
static_assert(exti_port_code('E') == (port_exists('E') ? 4u : 0xFFu));

// The three vectors and the lines each serves.
static_assert(exti_gpio_irq(0) == EXTI0_1_IRQn && exti_gpio_irq(1) == EXTI0_1_IRQn);
static_assert(exti_gpio_irq(2) == EXTI2_3_IRQn && exti_gpio_irq(3) == EXTI2_3_IRQn);
static_assert(exti_gpio_irq(4) == EXTI4_15_IRQn && exti_gpio_irq(15) == EXTI4_15_IRQn);
static_assert(exti_vector_lines(EXTI0_1_IRQn) == 0x0003u);
static_assert(exti_vector_lines(EXTI2_3_IRQn) == 0x000Cu);
static_assert(exti_vector_lines(EXTI4_15_IRQn) == 0xFFF0u);
static_assert((exti_vector_lines(EXTI0_1_IRQn) | exti_vector_lines(EXTI2_3_IRQn) |
               exti_vector_lines(EXTI4_15_IRQn)) == 0xFFFFu,
              "the three vectors cover the sixteen GPIO lines exactly once");

// ---- the sense vocabulary ---------------------------------------------------

static_assert(exti_sense_has_rising(ExtiSense::rising));
static_assert(exti_sense_has_falling(ExtiSense::falling));
static_assert(exti_sense_has_rising(ExtiSense::both) &&
              exti_sense_has_falling(ExtiSense::both));
static_assert(!exti_sense_has_rising(ExtiSense::none) &&
              !exti_sense_has_falling(ExtiSense::none));

constexpr ExtiPending nothing{};
static_assert(!nothing.any() && nothing.lines() == 0u);
constexpr ExtiPending both_edges{0x1u, 0x4u};
static_assert(both_edges.any() && both_edges.lines() == 0x5u);

// ---- lines through pads -----------------------------------------------------

using A0 = ExtInt<Pin<'A', 0>>;
using A1 = ExtInt<Pin<'A', 1>>;
using B2 = ExtInt<Pin<'B', 2>>;
using C3 = ExtInt<Pin<'C', 3>>;
using D4 = ExtInt<Pin<'D', 4>>;
using F1 = ExtInt<Pin<'F', 1>>;
using C13 = ExtInt<Pin<'C', 13>>;

static_assert(A0::line == 0 && A0::mask == 1u && A0::port == 'A');
static_assert(C13::line == 13 && C13::mask == (1u << 13));
static_assert(C13::irq() == EXTI4_15_IRQn);
static_assert(B2::irq() == EXTI2_3_IRQn);

// The application-level guard: the same line reached through two ports
// is the bug this expresses (neg/exti_two_pads_on_one_line.cpp is the
// refusal).
static_assert(exti_lines_distinct<A0, A1, B2, C3, D4, C13>());
static_assert(!exti_lines_distinct<A1, F1>(), "PA1 and PF1 are both line 1");
static_assert(exti_lines_distinct<>(), "an empty set is distinct");

void exti_block_verbs() {
    (void)Exti::sense(3, ExtiSense::both);
    (void)Exti::sense(3);
    (void)Exti::trigger(3);
    (void)Exti::rising_pending();
    (void)Exti::falling_pending();
    Exti::clear_rising(0xFFu);
    Exti::clear_falling(0xFFu);
    (void)Exti::rising_pending(3);
    (void)Exti::falling_pending(3);
    (void)Exti::pending(3);
    (void)Exti::clear(3);
    (void)Exti::interrupt(3, true);
    (void)Exti::interrupt(3);
    (void)Exti::event(3, true);
    (void)Exti::event(3);
    (void)Exti::select(3, 'B');
    (void)Exti::selected(3);
    (void)Exti::isr(Exti::vector_lines(EXTI2_3_IRQn));
    (void)Exti::release(3);
    (void)Exti::implemented(19);
    (void)Exti::configurable(19);
    (void)Exti::gpio(19);

    // A direct line: the masks answer, the triggers refuse.
    (void)Exti::interrupt(19, false);
    (void)Exti::event(19, true);
}

void exti_line_verbs() {
    (void)C13::claim(PinPull::up);
    (void)C13::select();
    (void)C13::selected();
    (void)C13::configure(ExtiSense::falling);
    (void)C13::sense();
    (void)C13::arm(true);
    (void)C13::armed();
    (void)C13::event(true);
    (void)C13::event();
    (void)C13::trigger();
    (void)C13::rising_pending();
    (void)C13::falling_pending();
    (void)C13::pending();
    (void)C13::clear();
    C13::release();

    (void)A0::claim();
    (void)B2::claim(PinPull::down);
    (void)D4::claim();
    (void)F1::claim();
}

// The second register group, where the family really differs: on a part
// without it the pointers are null and every verb above answers false
// instead of naming a struct member that does not exist.
void exti_group_two() {
    (void)exti_rtsr2();
    (void)exti_ftsr2();
    (void)exti_swier2();
    (void)exti_rpr2();
    (void)exti_fpr2();
    (void)exti_imr2();
    (void)exti_emr2();
    (void)Exti::interrupt(34, true);
    (void)Exti::sense(34, ExtiSense::rising);
    (void)Exti::trigger(34);
    (void)Exti::pending(34);
}
