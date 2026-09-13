// EXTI + SYSCFG family smoke TU (RM0090 ch. 12 and 9, RM0390 ch. 10 and
// 8, RM0383 ch. 10 and 7). The pin half of this peripheral is the uniform
// half - sixteen lines, numbered by the pin number, on every part - so
// what this TU really checks is that the PER-PART half comes from the
// device header and not from a list: which lines above 15 exist, which
// vector each reaches, and the multiplexer's port codes.
//
// A family fixture is the one place that may ask the header a question
// TWICE, by two independent symbols, and assert the two answers agree -
// which is what the peripheral-presence claims below do.
#include "stm32f4/exti.hpp"
#include "stm32f4/syscfg.hpp"

using namespace brio;

// ---- the reserve's own answers ----------------------------------------------

static_assert(exti_gpio_lines == 16, "SYSCFG_EXTICR is four registers of four fields");

// Lines 0..15 are pin lines and exist on every part of the family.
static_assert((exti_implemented_mask() & 0xFFFFu) == 0xFFFFu);
static_assert(exti_line_implemented(0) && exti_line_implemented(15));

// Every part has the supply monitor and the RTC, so those four lines are
// everywhere; nothing exists past the last line the part declares.
static_assert(exti_line_implemented(16), "PVD");
static_assert(exti_line_implemented(17), "RTC alarm");
static_assert(exti_line_implemented(21), "RTC tamper and timestamp");
static_assert(exti_line_implemented(22), "RTC wake-up");
static_assert(!exti_line_implemented(24));
static_assert(!exti_line_implemented(64));

// THE PERIPHERAL DECIDES, AND THE VECTOR AGREES. The reserve builds lines
// 18, 19 and 20 from the base-address macros of the USB and Ethernet
// controllers; the pack declares the matching wake-up VECTOR on exactly
// the same headers, and exti_line_irq() answers NonMaskableInt_IRQn where
// there is none. So the two derivations can be checked against each other
// with no device name spelled here.
static_assert(exti_line_implemented(18) == (exti_line_irq(18) != NonMaskableInt_IRQn),
              "the USB OTG FS wake-up line and its vector come and go together");
static_assert(exti_line_implemented(19) == (exti_line_irq(19) != NonMaskableInt_IRQn),
              "the Ethernet wake-up line and its vector come and go together");
static_assert(exti_line_implemented(20) == (exti_line_irq(20) != NonMaskableInt_IRQn),
              "the USB OTG HS wake-up line and its vector come and go together");

// Above 22 the EXTI's own bit macro is the whole authority, and the parts
// that declare a line 23 are exactly the parts with an LPTIM1 - a
// correlation only a fixture can state, the driver naming nothing wired
// there.
#if defined(LPTIM1_BASE)
static_assert(exti_line_implemented(23));
#else
static_assert(!exti_line_implemented(23));
#endif

// The multiplexer's port codes: the manual's contiguous A..K encoding,
// gated by the header's port presence.
static_assert(exti_port_code('A') == 0);
static_assert(exti_port_code('B') == 1);
static_assert(exti_port_code('C') == 2);
static_assert(exti_port_code('H') == 7);
static_assert(exti_port_code('L') == 0xFFu);
static_assert(exti_port_code('D') == (port_exists('D') ? 3u : 0xFFu));
static_assert(exti_port_code('K') == (port_exists('K') ? 10u : 0xFFu));

// The vectors and the lines each answers for: five of its own, two
// grouped, and one per peripheral wake-up.
static_assert(exti_line_irq(0) == EXTI0_IRQn && exti_line_irq(4) == EXTI4_IRQn);
static_assert(exti_line_irq(5) == EXTI9_5_IRQn && exti_line_irq(9) == EXTI9_5_IRQn);
static_assert(exti_line_irq(10) == EXTI15_10_IRQn && exti_line_irq(15) == EXTI15_10_IRQn);
static_assert(exti_line_irq(16) == PVD_IRQn && exti_line_irq(17) == RTC_Alarm_IRQn);
static_assert(exti_line_irq(21) == TAMP_STAMP_IRQn && exti_line_irq(22) == RTC_WKUP_IRQn);
static_assert(exti_vector_lines(EXTI0_IRQn) == 0x0001u);
static_assert(exti_vector_lines(EXTI9_5_IRQn) == 0x03E0u);
static_assert(exti_vector_lines(EXTI15_10_IRQn) == 0xFC00u);
static_assert(exti_vector_lines(SysTick_IRQn) == 0u, "a vector that is not the EXTI's");
static_assert((exti_vector_lines(EXTI0_IRQn) | exti_vector_lines(EXTI1_IRQn) |
               exti_vector_lines(EXTI2_IRQn) | exti_vector_lines(EXTI3_IRQn) |
               exti_vector_lines(EXTI4_IRQn) | exti_vector_lines(EXTI9_5_IRQn) |
               exti_vector_lines(EXTI15_10_IRQn)) == 0xFFFFu,
              "the seven pin vectors cover the sixteen pin lines exactly once");

// SYSCFG's gate is a real bit of RCC_APB2ENR, and the PHY selector is the
// Ethernet parts' (plus the F405 class, whose header carries the selector
// without the MAC - which is why has_phy_select() is not "has Ethernet").
static_assert(syscfg_clock_mask != 0u);
static_assert(Syscfg::has_phy_select() == (syscfg_phy_select_mask() != 0u));

// ---- the sense vocabulary ---------------------------------------------------

static_assert(exti_sense_has_rising(ExtiSense::rising));
static_assert(exti_sense_has_falling(ExtiSense::falling));
static_assert(exti_sense_has_rising(ExtiSense::both) &&
              exti_sense_has_falling(ExtiSense::both));
static_assert(!exti_sense_has_rising(ExtiSense::none) &&
              !exti_sense_has_falling(ExtiSense::none));

// ---- lines through pads -----------------------------------------------------

using A0 = ExtInt<Pin<'A', 0>>;
using A5 = ExtInt<Pin<'A', 5>>;
using B0 = ExtInt<Pin<'B', 0>>;
using B6 = ExtInt<Pin<'B', 6>>;
using C13 = ExtInt<Pin<'C', 13>>;
using H1 = ExtInt<Pin<'H', 1>>;

static_assert(A0::line == 0 && A0::mask == 1u && A0::port == 'A');
static_assert(C13::line == 13 && C13::mask == (1u << 13));
static_assert(C13::irq() == EXTI15_10_IRQn);
static_assert(A5::irq() == EXTI9_5_IRQn && B6::irq() == EXTI9_5_IRQn);
static_assert(A0::irq() == EXTI0_IRQn);

// The application-level guard: the same line reached through two ports is
// the bug this expresses (neg/exti_two_pads_on_one_line.cpp is the
// refusal).
static_assert(exti_lines_distinct<A0, A5, B6, C13>());
static_assert(!exti_lines_distinct<A0, B0>(), "PA0 and PB0 are both line 0");
static_assert(exti_lines_distinct<>(), "an empty set is distinct");

// ---- lines named as constants -----------------------------------------------

using Pvd = ExtiLine<16>;
using RtcAlarm = ExtiLine<17>;
using RtcWake = ExtiLine<22>;

static_assert(Pvd::line == 16 && Pvd::mask == (1u << 16));
static_assert(RtcAlarm::irq() == RTC_Alarm_IRQn);
static_assert(RtcWake::irq() == RTC_WKUP_IRQn);
static_assert(Exti::served(RtcWake::mask, 22) && !Exti::served(RtcWake::mask, 21));

// ---- every verb, once -------------------------------------------------------

void exti_block_verbs() {
    (void)Exti::implemented(19);
    (void)Exti::gpio(19);
    (void)Exti::regs().IMR;
    (void)Exti::sense(3, ExtiSense::both);
    (void)Exti::sense(3);
    (void)Exti::trigger(3);
    (void)Exti::triggered(3);
    (void)Exti::pending();
    (void)Exti::pending(3);
    Exti::clear_lines(0xFFu);
    (void)Exti::clear(static_cast<uint8_t>(3));
    (void)Exti::interrupt(3, true);
    (void)Exti::interrupt(3);
    (void)Exti::event(3, true);
    (void)Exti::event(3);
    (void)Exti::select(3, 'B');
    (void)Exti::steal(3, 'A');
    (void)Exti::selected(3);
    (void)Exti::in_use(3);
    (void)Exti::isr(Exti::vector_lines(EXTI3_IRQn));
    (void)Exti::served(0xFFu, 3);
    (void)Exti::release(3);

    // A line the device may not have: every verb answers false, none of
    // them names a register the part has not got (they all do).
    (void)Exti::interrupt(19, true);
    (void)Exti::sense(19, ExtiSense::rising);
    (void)Exti::trigger(19);
    (void)Exti::pending(19);
    (void)Exti::release(19);
}

void exti_pad_verbs() {
    (void)C13::claim(PinPull::up);
    (void)C13::select();
    (void)C13::steal();
    (void)C13::selected();
    (void)C13::configure(ExtiSense::falling);
    (void)C13::sense();
    (void)C13::arm(true);
    (void)C13::armed();
    (void)C13::event(true);
    (void)C13::event();
    (void)C13::trigger();
    (void)C13::triggered();
    (void)C13::pending();
    (void)C13::clear();
    (void)C13::served(0xFFFFu);
    C13::release();

    (void)A0::claim();
    (void)B0::claim(PinPull::down);
    (void)H1::claim();
}

void exti_constant_line_verbs() {
    (void)RtcWake::configure(ExtiSense::rising);
    (void)RtcWake::sense();
    (void)RtcWake::arm(true);
    (void)RtcWake::armed();
    (void)RtcWake::event(true);
    (void)RtcWake::event();
    (void)RtcWake::trigger();
    (void)RtcWake::triggered();
    (void)RtcWake::pending();
    (void)RtcWake::clear();
    (void)RtcWake::served(0u);
    (void)RtcWake::release();
    (void)Pvd::arm(false);
    (void)RtcAlarm::arm(false);
}

void syscfg_verbs() {
    Syscfg::clock(true);
    (void)Syscfg::clock();
    (void)Syscfg::regs().MEMRMP;
    (void)Syscfg::memory_map();
    (void)Syscfg::exti_source(3, 'B');
    (void)Syscfg::exti_source(3);
    Syscfg::compensation_cell(true);
    (void)Syscfg::compensation_cell();
    (void)Syscfg::compensation_ready();
    (void)Syscfg::phy_rmii(true);
    (void)Syscfg::phy_rmii();
    Syscfg::clock(false);
}
