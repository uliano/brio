// EXTI family smoke TU: the twenty-two lines, the sixteen that are pins
// and the six above them, every verb of the block, of a line named as a
// constant and of a line reached through its pad.
//
// WHICH LINES A PART HAS is the peripheral's: the USB device controller,
// the USB host/device one and the Ethernet each own a wake-up line, and
// the sixth belongs to the other device class. So the assertions here ask
// device:: rather than naming a part - the sweep over the nine is what
// proves they are asked.
//
// No pad is spelled as a literal either: the lowest pin each port bonds
// is what the pad-facing half is instantiated on.
#include "ch32v203/exti.hpp"
#include "ch32v203/pfic.hpp"

using namespace brio;

// ---- the lines -------------------------------------------------------------
static_assert(Exti::gpio_lines == 16 && Exti::line_count == 22);
static_assert(Exti::implemented(0) && Exti::implemented(15) && Exti::gpio(15));
static_assert(!Exti::gpio(Exti::line_pvd));
static_assert(Exti::implemented(Exti::line_pvd) && Exti::implemented(Exti::line_rtc_alarm));
static_assert(Exti::implemented(Exti::line_usbd_wakeup) ==
              (device::has_usbd || device::device_class == DeviceClass::v30x_d8));
static_assert(Exti::implemented(Exti::line_eth_wakeup) == device::has_ethernet);
static_assert(Exti::implemented(Exti::line_usbfs_wakeup) == device::has_usbfs);
static_assert(Exti::implemented(Exti::line_osc32k_wakeup) ==
              (device::device_class == DeviceClass::v20x_d8));
static_assert(!Exti::implemented(22) && !Exti::implemented(31));
static_assert((Exti::implemented_mask() & 0xFFFFu) == 0xFFFFu);
static_assert((Exti::implemented_mask() & (1UL << 22)) == 0u);

// ---- the vectors -----------------------------------------------------------
static_assert(Exti::irq(0) == Irq::exti0 && Exti::irq(4) == Irq::exti4);
static_assert(Exti::irq(5) == Irq::exti9_5 && Exti::irq(9) == Irq::exti9_5);
static_assert(Exti::irq(10) == Irq::exti15_10 && Exti::irq(15) == Irq::exti15_10);
static_assert(Exti::irq(Exti::line_pvd) == Irq::pvd);
static_assert(Exti::irq(Exti::line_rtc_alarm) == Irq::rtc_alarm);
static_assert(Exti::irq(Exti::line_eth_wakeup) == std::nullopt);
static_assert(Exti::irq(22) == std::nullopt);
static_assert(Exti::vector_lines(Irq::exti9_5) == 0x3E0u);
static_assert(Exti::vector_lines(Irq::exti15_10) == 0xFC00u);
static_assert(Exti::vector_lines(Irq::usart1) == 0u);
static_assert(Exti::served(0x3E0u, 7) && !Exti::served(0x3E0u, 4));

// ---- the senses ------------------------------------------------------------
static_assert(exti_sense_has_rising(ExtiSense::both) && exti_sense_has_falling(ExtiSense::both));
static_assert(!exti_sense_has_rising(ExtiSense::falling));
static_assert(!exti_sense_has_rising(ExtiSense::none) && !exti_sense_has_falling(ExtiSense::none));

/// The lowest pin of a port this package bonds (the pin family TU's
/// helper, which is also what proves the table is asked).
constexpr uint8_t first_pin(char port) {
    for (uint8_t n = 0; n < 16u; ++n) {
        if ((device::port_pins(port) & static_cast<uint16_t>(1u << n)) != 0u) {
            return n;
        }
    }
    return 0;
}

using PadA = Pin<'A', first_pin('A')>;
using PadB = Pin<'B', first_pin('B')>;
using LineA = ExtInt<PadA>;
using LineB = ExtInt<PadB>;

static_assert(LineA::line == first_pin('A') && LineA::port == 'A');
static_assert(LineA::mask == (1UL << first_pin('A')));
static_assert(exti_lines_distinct<LineA>());
static_assert(exti_lines_distinct<>());
// Two pads of the SAME number are one line, and the check says so.
static_assert(exti_lines_distinct<LineA, LineB>() == (first_pin('A') != first_pin('B')));

/// A line named as a constant: the PVD's, which every part has.
using PvdLine = ExtiLine<Exti::line_pvd>;
static_assert(PvdLine::mask == (1UL << 16));

void exti_verbs() {
    // The block, over every line the part has.
    for (uint8_t line = 0; line < Exti::line_count; ++line) {
        (void)Exti::implemented(line);
        (void)Exti::sense(line, ExtiSense::both);
        (void)Exti::sense(line);
        (void)Exti::interrupt(line, true);
        (void)Exti::interrupt(line);
        (void)Exti::event(line, true);
        (void)Exti::event(line);
        (void)Exti::trigger(line);
        (void)Exti::triggered(line);
        (void)Exti::pending(line);
        (void)Exti::clear(line);
        (void)Exti::in_use(line);
        (void)Exti::selected(line);
        (void)Exti::select(line, 'A');
        (void)Exti::steal(line, 'A');
        (void)Exti::release(line);
    }
    (void)Exti::pending();
    Exti::clear_lines(0xFFFFu);
    (void)Exti::isr(Exti::vector_lines(Irq::exti9_5));

    // One line as a constant.
    (void)PvdLine::configure(ExtiSense::rising);
    (void)PvdLine::sense();
    (void)PvdLine::arm(true);
    (void)PvdLine::armed();
    (void)PvdLine::event(true);
    (void)PvdLine::event();
    (void)PvdLine::trigger();
    (void)PvdLine::triggered();
    (void)PvdLine::pending();
    (void)PvdLine::clear();
    (void)PvdLine::served(0);
    (void)PvdLine::release();
    if (PvdLine::irq().has_value()) {
        Pfic::enable(*PvdLine::irq());
        Pfic::disable(*PvdLine::irq());
    }

    // One line through its pad.
    (void)LineA::claim(PinPull::up);
    (void)LineA::select();
    (void)LineA::steal();
    (void)LineA::selected();
    (void)LineA::configure(ExtiSense::falling);
    (void)LineA::sense();
    (void)LineA::arm(true);
    (void)LineA::armed();
    (void)LineA::event(false);
    (void)LineA::event();
    (void)LineA::trigger();
    (void)LineA::triggered();
    (void)LineA::pending();
    (void)LineA::clear();
    (void)LineA::served(0);
    LineA::release();
}

/// A shared vector's body, the shape an app binds (the attribute is
/// pfic.hpp's one spelling, and this TU is compiled both ways).
extern "C" BRIO_CH32_INTERRUPT void exti9_5_handler() {
    const uint32_t fired = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti9_5));
    if (brio::Exti::served(fired, 5)) {
        (void)fired;
    }
}
